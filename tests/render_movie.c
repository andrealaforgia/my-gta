/* Plays a scripted story through the real game and writes it as raw video:
 * the car drives to the airport, the player steals the gunship, flies it with
 * the cockpit view to an address, switches to the outside view and fires
 * missiles at the houses, flies back, steals the jet fighter, crosses the
 * city and lands. Frames go to standard output as RGBA, ready for
 * scripts/rgba_to_mp4.swift; progress goes to standard error.
 *
 *   render_movie PACKAGE_DIR LAT LON [--dry] [--seconds=N] | rgba_to_mp4 ...
 *
 * LAT and LON are the street address, found in the map data. Needs a GPU, so
 * it is not part of `make test`. A scripted player, not a game feature. */
#include <SDL.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "air_stage.h"
#include "driving_stage.h"
#include "log.h"
#include "projection.h"
#include "renderer.h"
#include "window.h"

#define FPS 30
#define WIDTH 1280
#define HEIGHT 720
#define GUNSHIP 7
#define JET 6

typedef enum {
  P_DRIVE,
  P_STOP,
  P_WALK_TO_GUNSHIP,
  P_HELI_CLIMB,
  P_HELI_CRUISE,
  P_HELI_ATTACK,
  P_HELI_RETURN,
  P_HELI_LAND,
  P_WALK_TO_JET,
  P_JET_TAXI,
  P_JET_TAKEOFF,
  P_JET_CRUISE,
  P_JET_APPROACH,
  P_JET_LAND,
  P_END
} phase_t;

static const char* phase_names[] = {
    "drive",        "stop",        "walk to gunship",
    "heli climb",   "heli cruise", "heli attack",
    "heli return",  "heli land",   "walk to jet",
    "jet taxi",     "jet takeoff", "jet cruise",
    "jet approach", "jet land",    "end"};

typedef struct {
  phase_t phase;
  double phase_time;  /* seconds in this phase */
  double clock;       /* seconds since the start */
  vec3_t target;      /* the address, in the city's metres */
  vec3_t strike_from; /* where the gunship hovers to fire */
  vec3_t airport;
  vec3_t bounds_min, bounds_max; /* what the city's ground covers */
  vec3_t axis, side;             /* along the runway (u) and across it (w) */
  float yaw_aim;                 /* heading to hold while firing */
  int missiles_fired;
  double last_shot;
  int waypoint;
  float blocked;   /* seconds the car has stood against something */
  float reversing; /* seconds left of backing out of trouble */
  float last_path; /* the jet's flight path at the last step */
  bool gear_changed;
  vec3_t last_walker;
  double stuck_for;
} director_t;

static float wrap(float a) {
  while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}
static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static float yaw_to(vec3_t from, vec3_t to) {
  return atan2f(-(to.x - from.x), -(to.z - from.z));
}
static float flat_distance(vec3_t a, vec3_t b) {
  return hypotf(a.x - b.x, a.z - b.z);
}

static float ground_at(driving_stage_t* stage, vec3_t p) {
  ray_t down = {vec3(p.x, p.y + 400.0f, p.z), vec3(0, -1, 0)};
  ray_cast_result_t hit;
  if (physics_ray_cast(stage->physics, down, 1200.0f,
                       COLLISION_LAYER_GROUND | COLLISION_LAYER_BUILDING, &hit))
    return hit.hit.point.y;
  return p.y - 50.0f;
}

/* Whether a column of air at `p` is clear of buildings all the way down, for
 * a helicopter to hang in. */
static bool open_column(driving_stage_t* stage, vec3_t p, float radius) {
  for (int k = 0; k < 9; k++) {
    float a = k == 8 ? 0.0f : (float)k * (float)M_PI / 4.0f;
    float r = k == 8 ? 0.0f : radius;
    vec3_t at = vec3(p.x + sinf(a) * r, 0, p.z + cosf(a) * r);
    ray_t down = {vec3(at.x, 400.0f, at.z), vec3(0, -1, 0)};
    ray_cast_result_t hit;
    if (physics_ray_cast(stage->physics, down, 800.0f, COLLISION_LAYER_BUILDING,
                         &hit))
      return false;
  }
  return true;
}

/* A place to hover and fire from: open ground between 110 and 300 m from the
 * address, as near the line to the airport as it can be. */
static vec3_t find_hover(driving_stage_t* stage, vec3_t target,
                         vec3_t airport) {
  vec3_t toward =
      vec3_normalize(vec3(airport.x - target.x, 0, airport.z - target.z));
  float base = atan2f(toward.x, toward.z);
  for (float dist = 110.0f; dist <= 300.0f; dist += 8.0f)
    for (int turn = 0; turn < 9; turn++) {
      float a = base + ((turn + 1) / 2) * (turn % 2 ? 1.0f : -1.0f) * 0.2f;
      vec3_t p = vec3(target.x + sinf(a) * dist, 0, target.z + cosf(a) * dist);
      if (open_column(stage, p, 14.0f)) {
        p.y = ground_at(stage, p);
        return p;
      }
    }
  vec3_t p = vec3_add(target, vec3_scale(toward, 200.0f));
  p.y = ground_at(stage, p);
  return p;
}

static void phase_to(director_t* d, driving_stage_t* stage, phase_t next,
                     const char* notice) {
  fprintf(stderr, "[%6.1fs] %s -> %s\n", d->clock, phase_names[d->phase],
          phase_names[next]);
  d->phase = next;
  d->phase_time = 0;
  d->waypoint = 0;
  if (notice) driving_stage_say(stage, notice);
}

/* The aircraft's own frame: where (u along the runway, w across it) is. */
static vec3_t airport_point(const director_t* d, float u, float w) {
  return vec3(d->airport.x + d->axis.x * u + d->side.x * w, d->airport.y,
              d->airport.z + d->axis.z * u + d->side.z * w);
}

/* ---- the pilot's hands ---- */

/* Steering to hold or reach a heading: roll or wheel, positive is right. */
static float steer_to(float yaw, float wanted, float gain) {
  return -clampf(wrap(wanted - yaw) * gain, -1.0f, 1.0f);
}

static vec3_t player_position(driving_stage_t* stage) {
  return physics_body(stage->physics, stage->vehicle.body)->position;
}

static void drive_the_car(driving_stage_t* stage, director_t* d,
                          driving_input_t* in) {
  const rigid_body_t* body = physics_body(stage->physics, stage->vehicle.body);
  vec3_t forward = quat_rotate(body->rotation, vec3(0, 0, -1));
  float yaw = atan2f(-forward.x, -forward.z);
  /* The way to the gate: the point 14 m along the satnav line, if there is
   * one, otherwise the gate itself. */
  vec3_t aim = vec3(stage->guide.x, 0, stage->guide.z);
  if (stage->satnav_count > 1) {
    size_t nearest = 0;
    float best = 1e30f;
    for (size_t i = 0; i < stage->satnav_count; i++) {
      float dx = stage->satnav[2 * i] - body->position.x;
      float dz = stage->satnav[2 * i + 1] - body->position.z;
      float dist = dx * dx + dz * dz;
      if (dist < best) best = dist, nearest = i;
    }
    float travelled = 0;
    size_t i = nearest;
    while (i + 1 < stage->satnav_count && travelled < 14.0f) {
      travelled +=
          hypotf(stage->satnav[2 * (i + 1)] - stage->satnav[2 * i],
                 stage->satnav[2 * (i + 1) + 1] - stage->satnav[2 * i + 1]);
      i++;
    }
    /* Past the end of the line, on to the gate. */
    if (i + 1 >= stage->satnav_count && travelled < 14.0f) {
      aim = vec3(stage->guide.x, 0, stage->guide.z);
    } else {
      aim = vec3(stage->satnav[2 * i], 0, stage->satnav[2 * i + 1]);
    }
  }
  in->steering = steer_to(yaw, yaw_to(body->position, aim), 1.6f);
  float speed = vec3_length(body->velocity);
  float distance =
      flat_distance(body->position, vec3(stage->guide.x, 0, stage->guide.z));
  if (getenv("MOVIE_TRACE") && ((int)(stage->simulated_seconds * 2)) % 2 == 0 &&
      stage->simulated_seconds < 30)
    fprintf(stderr,
            "  car %.0f,%.0f speed %.1f yaw %.2f aim %.0f,%.0f steer %.2f dist "
            "%.0f\n",
            body->position.x, body->position.z, vec3_length(body->velocity),
            yaw, aim.x, aim.z, in->steering, distance);
  float want =
      distance < 60.0f ? 5.0f : (fabsf(in->steering) > 0.3f ? 6.0f : 13.0f);
  in->throttle = speed < want ? 0.8f : 0.0f;
  in->brake = speed > want + 2.0f ? 0.6f : 0.0f;
  /* Against a kerb or a post: back out with the wheel the other way, then
   * try again. */
  d->blocked =
      speed < 0.4f && d->phase_time > 4.0 ? d->blocked + 1.0f / FPS : 0.0f;
  if (d->blocked > 1.5f) {
    d->reversing = 2.2f;
    d->blocked = 0;
  }
  if (d->reversing > 0) {
    d->reversing -= 1.0f / FPS;
    in->throttle = 0;
    in->brake = 1.0f;
    in->steering = -in->steering;
  }
}

static void walk_to(driving_stage_t* stage, director_t* d, vec3_t goal,
                    driving_input_t* in) {
  const walker_t* w = &stage->walker;
  float yaw_want = yaw_to(w->position, goal);
  in->steering = steer_to(w->yaw, yaw_want, 2.5f);
  in->throttle = fabsf(wrap(yaw_want - w->yaw)) < 0.9f ? 1.0f : 0.0f;
  in->handbrake = true; /* run */
  /* Walls in the way: after a few seconds without progress, a cut. */
  if (flat_distance(w->position, d->last_walker) < 0.2f)
    d->stuck_for += 1.0 / FPS;
  else
    d->stuck_for = 0;
  d->last_walker = w->position;
}

/* A helicopter: steer to a point, hold a height above the ground, and move at
 * `forward` (0 to 1) of full speed. */
static void fly_helicopter(driving_stage_t* stage, const aircraft_t* a,
                           vec3_t goal, float height_agl, float forward,
                           driving_input_t* in) {
  float ground = ground_at(stage, a->position);
  float height = a->position.y - ground;
  in->steering = steer_to(a->yaw, yaw_to(a->position, goal), 1.8f);
  in->throttle = forward;
  /* Up, down, or hold, with a dead band; a pulse when nearly there. */
  float error = height_agl - height;
  if (error > 3.0f) {
    in->handbrake = true;
  } else if (error < -3.0f) {
    in->lower = true;
  } else if (error > 0.8f) {
    in->handbrake = ((int)(stage->simulated_seconds * 4.0f)) % 2 == 0;
  } else if (error < -0.8f) {
    in->lower = ((int)(stage->simulated_seconds * 4.0f)) % 2 == 0;
  }
}

/* A fixed-wing aircraft in the air: heading, a flight path angle, and a
 * speed. */
static void fly_jet(const aircraft_t* a, float yaw_wanted, float gamma_wanted,
                    float speed_wanted, float* last_path, driving_input_t* in) {
  in->steering = steer_to(a->yaw, yaw_wanted, 2.2f);
  /* Nose up or down to hold the flight path angle; the rate of change damps
   * it. */
  float rate = (a->path - *last_path) * FPS;
  *last_path = a->path;
  float command =
      clampf(6.0f * (gamma_wanted - a->path) - 1.2f * rate, -1.0f, 1.0f);
  if (command > 0)
    in->brake = command;
  else
    in->throttle = -command;
  if (a->speed < speed_wanted - 3.0f)
    in->handbrake = true;
  else if (a->speed > speed_wanted + 3.0f)
    in->lower = true;
}

/* ---- the story ---- */

static void step_director(driving_stage_t* stage, director_t* d,
                          driving_input_t* in) {
  memset(in, 0, sizeof(*in));
  d->phase_time += 1.0 / FPS;
  d->clock += 1.0 / FPS;
  const aircraft_t* flown = air_stage_piloted(stage);
  bool on_foot_phase =
      d->phase == P_WALK_TO_GUNSHIP || d->phase == P_WALK_TO_JET;
  if (!flown && d->phase >= P_HELI_CLIMB && !on_foot_phase &&
      d->phase != P_END && d->phase != P_HELI_LAND) {
    fprintf(stderr, "[%6.1fs] the aircraft is gone in phase %s\n", d->clock,
            phase_names[d->phase]);
    d->phase = P_END;
    return;
  }
  switch (d->phase) {
    case P_DRIVE: {
      drive_the_car(stage, d, in);
      vec3_t p = player_position(stage);
      if (flat_distance(p, vec3(stage->guide.x, 0, stage->guide.z)) < 50.0f ||
          d->phase_time > 80.0)
        phase_to(d, stage, P_STOP, "AT THE AIRPORT");
      break;
    }
    case P_STOP: {
      float speed = vec3_length(
          physics_body(stage->physics, stage->vehicle.body)->velocity);
      in->brake = speed > 0.6f ? 1.0f : 0.0f;
      if (speed < 0.6f && d->phase_time > 1.0) {
        driving_stage_interact(stage); /* out of the car */
        d->last_walker = stage->walker.position;
        phase_to(d, stage, P_WALK_TO_GUNSHIP, "TO THE GUNSHIP");
      }
      break;
    }
    case P_WALK_TO_GUNSHIP: {
      walk_to(stage, d, stage->aircraft[GUNSHIP].position, in);
      if (d->phase_time > 1.5 && air_stage_nearest(stage) &&
          air_stage_nearest(stage) == &stage->aircraft[GUNSHIP]) {
        driving_stage_interact(stage); /* steal it */
        if (stage->piloting == GUNSHIP + 1) {
          stage->air_view = 1; /* the cockpit */
          phase_to(d, stage, P_HELI_CLIMB, "COCKPIT VIEW");
        }
      } else if (d->stuck_for > 3.0 || d->phase_time > 50.0) {
        /* A cut to the aircraft rather than a long wait. */
        stage->walker.position =
            vec3_add(stage->aircraft[GUNSHIP].position, vec3(0, 0, -7.5f));
        d->stuck_for = 0;
      }
      break;
    }
    case P_HELI_CLIMB: {
      fly_helicopter(stage, flown, d->strike_from, 70.0f, 0.0f, in);
      float height = flown->position.y - ground_at(stage, flown->position);
      if (height > 55.0f)
        phase_to(d, stage, P_HELI_CRUISE, "TO CORSO EUROPA 22");
      break;
    }
    case P_HELI_CRUISE: {
      /* To the hover spot near the address, up at 60 m. */
      float distance = flat_distance(flown->position, d->strike_from);
      float slow = clampf(distance / 260.0f, 0.2f, 1.0f);
      fly_helicopter(stage, flown, d->strike_from, 60.0f, 0.62f * slow, in);
      if (distance < 25.0f || d->phase_time > 220.0) {
        stage->air_view = 0; /* the outside view */
        phase_to(d, stage, P_HELI_ATTACK, "OUTSIDE VIEW");
      }
      break;
    }
    case P_HELI_ATTACK: {
      /* Down to rooftop height, face the houses, and fire a few missiles. */
      float height = flown->position.y - ground_at(stage, flown->position);
      float hold =
          flat_distance(flown->position, d->strike_from) > 12.0f ? 0.2f : 0.0f;
      vec3_t goal = hold > 0 ? d->strike_from : d->target;
      /* Turn to face the address while descending over the spot. */
      /* Each missile at a different part of the street. */
      static const float spread[6] = {-0.30f, -0.15f, 0.0f,
                                      0.15f,  0.30f,  0.05f};
      float aim = yaw_to(flown->position, d->target) +
                  spread[d->missiles_fired < 6 ? d->missiles_fired : 5];
      in->steering = steer_to(flown->yaw, aim, 2.0f);
      in->throttle = hold;
      if (hold > 0)
        in->steering =
            steer_to(flown->yaw, yaw_to(flown->position, goal), 1.8f);
      if (height > 11.0f)
        in->lower = true;
      else if (height > 8.0f)
        in->lower = ((int)(stage->simulated_seconds * 5.0f)) % 2 == 0;
      else if (height < 5.0f)
        in->handbrake = true;
      float facing = fabsf(wrap(aim - flown->yaw));
      if (height < 13.0f && facing < 0.08f && d->missiles_fired < 6 &&
          d->clock - d->last_shot > 0.9) {
        if (driving_stage_fire(stage)) {
          d->missiles_fired++;
          d->last_shot = d->clock;
          if (d->missiles_fired == 1) driving_stage_say(stage, "FIRE");
        }
      }
      if ((d->missiles_fired >= 6 && d->clock - d->last_shot > 5.0) ||
          d->phase_time > 90.0)
        phase_to(d, stage, P_HELI_RETURN, "BACK TO THE AIRPORT");
      break;
    }
    case P_HELI_RETURN: {
      vec3_t home = stage->aircraft_home[GUNSHIP].position;
      float distance = flat_distance(flown->position, home);
      float agl =
          distance > 250.0f ? 70.0f : clampf(distance * 0.25f, 12.0f, 70.0f);
      float slow = clampf(distance / 200.0f, 0.1f, 1.0f);
      fly_helicopter(stage, flown, home, agl, 0.62f * slow, in);
      if (distance < 12.0f || d->phase_time > 220.0)
        phase_to(d, stage, P_HELI_LAND, NULL);
      break;
    }
    case P_HELI_LAND: {
      vec3_t home = stage->aircraft_home[GUNSHIP].position;
      float ground = ground_at(stage, flown->position);
      float height = flown->position.y - ground;
      in->steering = steer_to(flown->yaw, yaw_to(flown->position, home), 1.0f);
      /* Down in pulses, so the descent is a few metres a second. */
      bool pulse = ((int)(stage->simulated_seconds * 5.0f)) % 2 == 0;
      if (height > 10.0f)
        in->lower = true;
      else if (height > 0.4f)
        in->lower = pulse;
      if (flown->on_ground && fabsf(flown->speed) < 0.5f &&
          d->phase_time > 2.0) {
        driving_stage_interact(stage); /* out */
        d->last_walker = stage->walker.position;
        phase_to(d, stage, P_WALK_TO_JET, "TO THE JET FIGHTER");
      }
      break;
    }
    case P_WALK_TO_JET: {
      walk_to(stage, d, stage->aircraft[JET].position, in);
      if (d->phase_time > 1.5 && air_stage_nearest(stage) &&
          air_stage_nearest(stage) == &stage->aircraft[JET]) {
        driving_stage_interact(stage);
        if (stage->piloting == JET + 1) {
          stage->air_view = 0;
          phase_to(d, stage, P_JET_TAXI, NULL);
        }
      } else if (d->stuck_for > 3.0 || d->phase_time > 40.0) {
        stage->walker.position =
            vec3_add(stage->aircraft[JET].position, vec3(0, 0, -5.0f));
        d->stuck_for = 0;
      }
      break;
    }
    case P_JET_TAXI: {
      /* Off the apron, along the taxiway, across the link, onto the
       * runway, and round to face along it. */
      static const float route[][2] = {{35, 70}, {30, 58}, {2, 58}, {0, 30},
                                       {0, 6},   {-25, 0}, {-60, 0}};
      const int count = (int)(sizeof(route) / sizeof(route[0]));
      vec3_t goal =
          airport_point(d, route[d->waypoint][0], route[d->waypoint][1]);
      float want = d->waypoint == count - 1 ? 6.0f : 7.5f;
      in->steering = steer_to(flown->yaw, yaw_to(flown->position, goal), 2.0f);
      if (flown->speed < want) in->handbrake = true;
      if (flown->speed > want + 1.5f) in->lower = true;
      if (flat_distance(flown->position, goal) < 9.0f) d->waypoint++;
      if (d->waypoint >= count || d->phase_time > 90.0) {
        phase_to(d, stage, P_JET_TAKEOFF, "TAKE OFF");
        d->gear_changed = false;
      }
      break;
    }
    case P_JET_TAKEOFF: {
      /* Full power, down the runway, the nose up at speed, wheels up. */
      vec3_t far = airport_point(d, -440, 0);
      in->steering = steer_to(flown->yaw, yaw_to(flown->position, far), 2.5f);
      in->handbrake = true;
      if (flown->speed > 46.0f) in->brake = 0.55f;
      float agl = flown->position.y - ground_at(stage, flown->position);
      if (!flown->on_ground && agl > 12.0f && !d->gear_changed) {
        air_stage_toggle_gear(stage);
        d->gear_changed = true;
      }
      if (!flown->on_ground && agl > 45.0f) {
        d->last_path = flown->path;
        stage->air_view = 1; /* the instruments */
        phase_to(d, stage, P_JET_CRUISE, "ACROSS THE CITY");
      }
      if (d->phase_time > 60.0) phase_to(d, stage, P_JET_CRUISE, NULL);
      break;
    }
    case P_JET_CRUISE: {
      /* Over the city and round: out to the west, up over the middle, across
       * the north side, and back down the east side to the approach. Points
       * are given as how far across the city they are (0 to 1). */
      static const float route[][2] = {
          {0.12f, 0.80f}, {0.45f, 0.45f}, {0.80f, 0.12f}, {0.92f, 0.55f}};
      const int count = (int)(sizeof(route) / sizeof(route[0]));
      int k = d->waypoint < count ? d->waypoint : count - 1;
      vec3_t goal = vec3(
          d->bounds_min.x + (d->bounds_max.x - d->bounds_min.x) * route[k][0],
          0,
          d->bounds_min.z + (d->bounds_max.z - d->bounds_min.z) * route[k][1]);
      float agl = flown->position.y - ground_at(stage, flown->position);
      float gamma = clampf((220.0f - agl) * 0.004f, -0.12f, 0.16f);
      fly_jet(flown, yaw_to(flown->position, goal), gamma, 95.0f, &d->last_path,
              in);
      if (flat_distance(flown->position, goal) < 350.0f) d->waypoint++;
      if (d->waypoint >= count || d->phase_time > 150.0) {
        stage->air_view = 0; /* back outside for the landing */
        phase_to(d, stage, P_JET_APPROACH, "LANDING");
        d->gear_changed = false;
      }
      break;
    }
    case P_JET_APPROACH: {
      /* Line up with the runway from the east, over the city: fly to a point
       * on its extended centre line, then down the glide slope. */
      vec3_t away = d->axis; /* from the runway out to where we come from */
      vec3_t threshold = airport_point(d, 400.0f, 0);
      vec3_t start = vec3_add(threshold, vec3_scale(away, 1700.0f));
      float alt = flown->position.y - ground_at(stage, flown->position);
      float to_start = flat_distance(flown->position, start);
      if (to_start > 500.0f && d->waypoint == 0) {
        float gamma = clampf((120.0f - alt) * 0.004f, -0.1f, 0.1f);
        fly_jet(flown, yaw_to(flown->position, start), gamma, 85.0f,
                &d->last_path, in);
      } else {
        d->waypoint = 1;
        /* Final: track the centre line and the three-degree slope. */
        vec3_t into = vec3_scale(away, -1.0f);
        vec3_t rel = vec3_sub(flown->position, threshold);
        float across = vec3_dot(rel, d->side);
        float to_go = vec3_dot(rel, away); /* metres still to the threshold */
        float yaw_line = atan2f(-into.x, -into.z);
        /* Which way is back to the line: toward -side when across > 0. The
         * heading offset steers back onto the centre line. */
        float yaw_cmd = yaw_line + clampf(across * 0.004f, -0.3f, 0.3f);
        /* Touch down about 250 m past the threshold. */
        float slope_alt = 0.052f * (to_go + 250.0f);
        float gamma =
            clampf(-0.052f + (slope_alt - alt) * 0.008f, -0.09f, 0.04f);
        float speed_wanted = clampf(56.0f + 0.005f * to_go, 56.0f, 66.0f);
        fly_jet(flown, yaw_cmd, gamma, speed_wanted, &d->last_path, in);
        if (to_go < 1500.0f && !d->gear_changed) {
          air_stage_toggle_gear(stage); /* wheels down */
          d->gear_changed = true;
        }
        if (to_go < 250.0f && alt < 30.0f) phase_to(d, stage, P_JET_LAND, NULL);
      }
      if (d->phase_time > 220.0) phase_to(d, stage, P_JET_LAND, NULL);
      break;
    }
    case P_JET_LAND: {
      vec3_t into = vec3_scale(d->axis, -1.0f);
      vec3_t threshold = airport_point(d, 400.0f, 0);
      float across = vec3_dot(vec3_sub(flown->position, threshold), d->side);
      float yaw_line = atan2f(-into.x, -into.z);
      float alt = flown->position.y - ground_at(stage, flown->position);
      if (!flown->on_ground) {
        /* The flare: ease the sink to a metre or two a second, power off. */
        float sink_wanted = alt > 9.0f ? -3.2f : (alt > 3.0f ? -2.0f : -1.3f);
        float gamma = sink_wanted / fmaxf(flown->speed, 30.0f);
        fly_jet(flown, yaw_line + clampf(across * 0.004f, -0.3f, 0.3f), gamma,
                0.0f, &d->last_path, in);
        in->handbrake = false;
        in->lower = true;
      } else {
        /* Rolling out: straight, brakes on. */
        in->steering = steer_to(flown->yaw, yaw_line, 2.0f);
        in->lower = true;
        if (flown->speed < 3.0f && d->phase_time > 2.0)
          phase_to(d, stage, P_END, "LANDED");
      }
      if (d->phase_time > 80.0) phase_to(d, stage, P_END, NULL);
      break;
    }
    case P_END:
      in->lower = true;
      break;
  }
}

static int save_bmp(const char* path, int width, int height, uint8_t* rgba) {
  SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(0, width, height, 32,
                                                        SDL_PIXELFORMAT_RGBA32);
  if (!surface) return 0;
  for (int y = 0; y < height; y++)
    memcpy((uint8_t*)surface->pixels + y * surface->pitch,
           rgba + (size_t)(height - 1 - y) * width * 4, (size_t)width * 4);
  int ok = SDL_SaveBMP(surface, path) == 0;
  SDL_FreeSurface(surface);
  return ok;
}

/* Sends the frame to standard output; with MOVIE_STILLS=dir, also keeps one
 * frame in every `still_every` as a BMP, to look at without a player. */
static void write_frame(renderer_t* renderer, uint8_t* pixels, size_t bytes,
                        int frame) {
  renderer_read_pixels(renderer, pixels, bytes);
  fwrite(pixels, 1, bytes, stdout);
  const char* dir = getenv("MOVIE_STILLS");
  /* MOVIE_RANGE=from,to (seconds) keeps a still every half second there. */
  double from = 0, to = 0;
  if (getenv("MOVIE_RANGE"))
    sscanf(getenv("MOVIE_RANGE"), "%lf,%lf", &from, &to);
  bool in_range = to > from && frame >= from * FPS && frame <= to * FPS;
  if (dir && (in_range ? frame % 8 == 0 : frame % (FPS * 5) == 0)) {
    char path[512];
    snprintf(path, sizeof(path), "%s/still_%05d.bmp", dir, frame);
    save_bmp(path, WIDTH, HEIGHT, pixels);
  }
}

int main(int argc, char* argv[]) {
  if (argc < 4) {
    fprintf(stderr,
            "usage: render_movie PACKAGE LAT LON [--dry] [--seconds=N]\n");
    return EXIT_FAILURE;
  }
  const char* package = argv[1];
  double lat = atof(argv[2]), lon = atof(argv[3]);
  bool dry = false;
  double limit = 600.0;
  for (int i = 4; i < argc; i++) {
    if (strcmp(argv[i], "--dry") == 0) dry = true;
    if (strncmp(argv[i], "--seconds=", 10) == 0) limit = atof(argv[i] + 10);
  }
  window_options_t options = window_default_options("movie");
  options.hidden = true;
  options.vsync = false;
  options.width = WIDTH;
  options.height = HEIGHT;
  window_t window;
  if (!window_open(&window, options)) return EXIT_FAILURE;
  renderer_t* renderer =
      renderer_create(window.drawable_width, window.drawable_height);
  if (!renderer) return EXIT_FAILURE;
  stage_options_t stage_options = {SCENE_CITY, package, NULL, false,
                                   true,       10.5f,   0.0f, 24,
                                   140,        0.0f,    false};
  driving_stage_t stage;
  if (!driving_stage_init(&stage, &window, renderer, NULL, stage_options))
    return EXIT_FAILURE;
  const city_package_t* city = stage.scene.package;
  if (!city->airport.present || stage.aircraft_count < 8) {
    fprintf(stderr, "this city has no airport with all its aircraft\n");
    return EXIT_FAILURE;
  }
  director_t d;
  memset(&d, 0, sizeof(d));
  float lo_x = 1e30f, lo_z = 1e30f, hi_x = -1e30f, hi_z = -1e30f;
  for (size_t i = 0; i < city->chunk_count; i++) {
    const aabb_t* b = &city->chunks[i].bounds;
    lo_x = fminf(lo_x, b->min.x), hi_x = fmaxf(hi_x, b->max.x);
    lo_z = fminf(lo_z, b->min.z), hi_z = fmaxf(hi_z, b->max.z);
  }
  d.bounds_min = vec3(lo_x, 0, lo_z);
  d.bounds_max = vec3(hi_x, 0, hi_z);
  fprintf(stderr, "city spans x %.0f..%.0f, z %.0f..%.0f\n", lo_x, hi_x, lo_z,
          hi_z);
  projection_t projection =
      projection_create(city->origin_lat, city->origin_lon);
  local_point_t address = projection_project(&projection, lat, lon);
  d.target = vec3((float)address.x, 0, (float)address.z);
  d.target.y = ground_at(&stage, d.target);
  d.airport = vec3(city->airport.x, city->airport.gate_y, city->airport.z);
  d.axis = vec3(-sinf(city->airport.yaw), 0, -cosf(city->airport.yaw));
  vec3_t home_jet = stage.aircraft_home[JET].position;
  vec3_t lateral = vec3(d.axis.z, 0, -d.axis.x);
  d.side = vec3_dot(vec3_sub(home_jet, d.airport), lateral) > 0
               ? lateral
               : vec3_scale(lateral, -1.0f);
  /* Where the gunship hovers to fire. */
  d.strike_from = find_hover(&stage, d.target, d.airport);
  fprintf(stderr,
          "gate at %.0f, %.0f; address at %.0f, %.0f; hover at %.0f, %.0f; "
          "airport %.0f, %.0f\n",
          stage.guide.x, stage.guide.z, d.target.x, d.target.z, d.strike_from.x,
          d.strike_from.z, d.airport.x, d.airport.z);
  {
    vec3_t gate = vec3(stage.guide.x, 0, stage.guide.z);
    vec3_t rel = vec3_sub(gate, d.airport);
    fprintf(stderr, "axis %.2f,%.2f side %.2f,%.2f; gate at u=%.0f w=%.0f\n",
            d.axis.x, d.axis.z, d.side.x, d.side.z, vec3_dot(rel, d.axis),
            vec3_dot(rel, d.side));
    for (int k = 5; k < 8; k++) {
      vec3_t r = vec3_sub(stage.aircraft_home[k].position, d.airport);
      fprintf(stderr, "aircraft %d at u=%.0f w=%.0f\n", k, vec3_dot(r, d.axis),
              vec3_dot(r, d.side));
    }
  }
  d.phase = P_DRIVE;
  size_t bytes = (size_t)window.drawable_width * window.drawable_height * 4;
  uint8_t* pixels = malloc(bytes);
  int frames = 0;
  while (d.phase != P_END && d.clock < limit) {
    driving_input_t input;
    step_director(&stage, &d, &input);
    driving_stage_advance(&stage, input, 1.0 / FPS);
    if (!dry) {
      driving_stage_render(&stage);
      write_frame(renderer, pixels, bytes, frames);
    }
    if (getenv("MOVIE_TRACE") && d.phase == P_HELI_ATTACK) {
      for (int m = 0; m < 8; m++)
        if (stage.missiles[m].active && frames % 6 == 0)
          fprintf(stderr, "  missile %d at %.0f,%.0f,%.0f age %.2f\n", m,
                  stage.missiles[m].position.x, stage.missiles[m].position.y,
                  stage.missiles[m].position.z, stage.missiles[m].age);
      for (int m = 0; m < 6; m++)
        if (stage.strikes[m].age > 0 && stage.strikes[m].age < 0.05f)
          fprintf(stderr, "  STRIKE at %.0f,%.0f,%.0f\n", stage.strikes[m].at.x,
                  stage.strikes[m].at.y, stage.strikes[m].at.z);
    }
    if (getenv("MOVIE_TRACE") && d.phase >= P_JET_APPROACH &&
        frames % 15 == 0) {
      const aircraft_t* a = air_stage_piloted(&stage);
      if (a)
        fprintf(stderr,
                "  t=%.1f alt=%.0f spd=%.0f path=%.3f pitch=%.3f roll=%.2f "
                "yaw=%.2f gear=%.1f pwr=%.2f pos=%.0f,%.0f\n",
                d.clock, a->position.y - ground_at(&stage, a->position),
                a->speed, a->path, a->pitch, a->roll, a->yaw, a->gear, a->power,
                a->position.x, a->position.z);
    }
    if (++frames % (FPS * 10) == 0) {
      const aircraft_t* a = air_stage_piloted(&stage);
      fprintf(
          stderr, "[%6.1fs] %-14s %s %.0f m/s at %.0f,%.0f,%.0f\n", d.clock,
          phase_names[d.phase], a ? "flying" : "ground",
          a ? a->speed
            : vec3_length(
                  physics_body(stage.physics, stage.vehicle.body)->velocity),
          a ? a->position.x : player_position(&stage).x,
          a ? a->position.y : player_position(&stage).y,
          a ? a->position.z : player_position(&stage).z);
    }
  }
  /* A few seconds on the ground at the end. */
  for (int i = 0; i < FPS * 3 && !dry; i++) {
    driving_input_t brake = {0, 0, 0, false, true};
    driving_stage_advance(&stage, brake, 1.0 / FPS);
    driving_stage_render(&stage);
    write_frame(renderer, pixels, bytes, frames + i);
  }
  fprintf(stderr, "missiles fired: %d\n", d.missiles_fired);
  fprintf(stderr, "done: %d frames, %.1f s, ended in phase %s\n", frames,
          d.clock, phase_names[d.phase]);
  free(pixels);
  driving_stage_destroy(&stage);
  renderer_destroy(renderer);
  window_close(&window);
  return EXIT_SUCCESS;
}
