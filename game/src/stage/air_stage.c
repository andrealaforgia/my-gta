#include "air_stage.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "aircraft_cockpit.h"

#define BOARD_REACH 9.0f
#define AIRCRAFT_LAYERS (COLLISION_LAYER_GROUND | COLLISION_LAYER_BUILDING)

void air_stage_init(driving_stage_t* stage) {
  const city_package_t* package = stage->scene.package;
  if (!package || package->aircraft_count == 0) return;
  stage->air_visual = aircraft_visual_create(stage->renderer);
  stage->has_air = true;
  for (size_t i = 0; i < package->aircraft_count; i++) {
    const city_aircraft_t* craft = &package->aircraft[i];
    stage->aircraft[i] =
        aircraft_create((aircraft_kind_t)craft->kind,
                        vec3(craft->x, craft->y, craft->z), craft->yaw);
    stage->aircraft_home[i] = stage->aircraft[i];
  }
  stage->aircraft_count = package->aircraft_count;
}

void air_stage_destroy(driving_stage_t* stage) {
  if (stage->has_air)
    aircraft_visual_destroy(&stage->air_visual, stage->renderer);
  stage->has_air = false;
}

const aircraft_t* air_stage_piloted(const driving_stage_t* stage) {
  return stage->piloting > 0 ? &stage->aircraft[stage->piloting - 1] : NULL;
}

/* From the air much more of the city is wanted in. */
static void set_streaming(driving_stage_t* stage, bool wide) {
  stage->scene.stream_load_radius = wide ? 1500.0f : 800.0f;
  stage->scene.stream_unload_radius = wide ? 1700.0f : 1000.0f;
}

static chase_camera_params_t camera_for(aircraft_kind_t kind) {
  chase_camera_params_t p = chase_camera_default_params();
  /* Further back and higher behind the bigger aircraft. */
  static const float distances[AIRCRAFT_KIND_COUNT] = {17.0f, 19.0f, 75.0f,
                                                       26.0f, 30.0f};
  static const float heights[AIRCRAFT_KIND_COUNT] = {4.5f, 6.0f, 20.0f, 6.5f,
                                                     9.0f};
  p.distance = distances[kind];
  p.height = heights[kind];
  p.look_height = 1.5f;
  p.look_ahead = 0.0f;
  p.max_look_ahead = 0.0f;
  p.position_rate = 4.0f;
  p.look_rate = 6.0f;
  p.collision_margin = 0.0f;
  p.fov_y = 1.05f;
  return p;
}

static transform_t camera_target(const aircraft_t* a) {
  /* Level: the view does not roll with the wings. */
  return transform_at(vec3_add(a->position, vec3(0, 1.5f, 0)),
                      quat_from_yaw(a->yaw));
}

/* How far the walker is from the aircraft's body: from its centre line, not
 * from its middle, so the nose and the tail of an airliner are in reach. */
static float distance_to_body(const aircraft_t* a, vec3_t from) {
  vec3_t heading = aircraft_heading(a);
  float half = fmaxf(0.0f, aircraft_radius(a->kind) - 3.0f);
  float dx = from.x - a->position.x, dz = from.z - a->position.z;
  float along = fmaxf(-half, fminf(half, dx * heading.x + dz * heading.z));
  return hypotf(dx - heading.x * along, dz - heading.z * along);
}

static int nearest_index(const driving_stage_t* stage) {
  int best = -1;
  float best_distance = BOARD_REACH;
  for (size_t i = 0; i < stage->aircraft_count; i++) {
    float d = distance_to_body(&stage->aircraft[i], stage->walker.position);
    if (d < best_distance) best = (int)i, best_distance = d;
  }
  return best;
}

const aircraft_t* air_stage_nearest(const driving_stage_t* stage) {
  if (!stage->has_air || stage->piloting > 0) return NULL;
  int best = nearest_index(stage);
  return best < 0 ? NULL : &stage->aircraft[best];
}

bool air_stage_board(driving_stage_t* stage) {
  if (!stage->has_air || stage->piloting > 0) return false;
  int best = nearest_index(stage);
  if (best < 0) return false;
  stage->piloting = best + 1;
  stage->air_view = 0;
  set_streaming(stage, true);
  stage->saved_camera_params = stage->camera.params;
  stage->camera.params = camera_for(stage->aircraft[best].kind);
  chase_camera_reset(&stage->camera, camera_target(&stage->aircraft[best]));
  stage->walker.speed = 0;
  static const char* stolen[AIRCRAFT_KIND_COUNT] = {
      "PLANE STOLEN", "HELICOPTER STOLEN", "BOEING 747 STOLEN",
      "JET FIGHTER STOLEN", "GUNSHIP STOLEN"};
  driving_stage_say(stage, stolen[stage->aircraft[best].kind]);
  driving_stage_commit_crime(stage);
  return true;
}

/* Out of an aircraft in the air: a jump, a parachute, and the aircraft left
 * to dive. */
/* Close behind the person and a little above, looking up towards the canopy. */
static chase_camera_params_t chute_camera(void) {
  chase_camera_params_t p = chase_camera_default_params();
  p.distance = 5.0f;
  p.height = 1.3f;
  p.look_height = 1.0f;
  p.look_ahead = 0.0f;
  p.max_look_ahead = 0.0f;
  p.position_rate = 7.0f;
  p.look_rate = 8.0f;
  p.collision_margin = 0.3f;
  p.fov_y = 1.15f;
  return p;
}

static void bail_out(driving_stage_t* stage) {
  aircraft_t* a = &stage->aircraft[stage->piloting - 1];
  a->doomed = true;
  vec3_t heading = aircraft_heading(a);
  vec3_t left = vec3(heading.z, 0, -heading.x);
  stage->walker.position =
      vec3_add(vec3_add(a->position, vec3_scale(left, 2.0f)), vec3(0, 0.6f, 0));
  stage->walker.yaw = a->yaw;
  stage->walker.speed = 0;
  stage->walker.grounded = false;
  stage->piloting = 0;
  /* The jump carries on a little of the aircraft's way. */
  vec3_t carried = vec3_scale(aircraft_velocity(a), 0.6f);
  stage->chute.active = true;
  stage->chute.opened = false;
  stage->chute.time = 0;
  stage->chute.bank = 0;
  stage->chute.velocity = vec3(carried.x, fminf(carried.y, 0.0f), carried.z);
  stage->camera.params = chute_camera();
  chase_camera_reset(
      &stage->camera,
      transform_at(vec3_add(stage->walker.position, vec3(0, 0.9f, 0)),
                   quat_from_yaw(stage->walker.yaw)));
  driving_stage_say(stage, "BAILED OUT");
}

bool air_stage_leave(driving_stage_t* stage) {
  const aircraft_t* a = air_stage_piloted(stage);
  if (!a) return false;
  if (!a->on_ground || a->speed > 3.0f) {
    bail_out(stage);
    return true;
  }
  /* Out on the left of the aircraft. */
  vec3_t heading = aircraft_heading(a);
  vec3_t left = vec3(heading.z, 0, -heading.x);
  vec3_t feet = vec3_add(a->position, vec3_scale(left, 2.8f));
  stage->walker.position = feet;
  stage->walker.yaw = a->yaw;
  stage->walker.speed = 0;
  stage->piloting = 0;
  set_streaming(stage, false);
  stage->camera.params = stage->saved_camera_params;
  chase_camera_reset(&stage->camera,
                     transform_at(vec3_add(feet, vec3(0, 0.9f, 0)),
                                  quat_from_yaw(stage->walker.yaw)));
  return true;
}

void air_stage_stop(driving_stage_t* stage) {
  if (stage->piloting > 0) {
    stage->aircraft[stage->piloting - 1] =
        stage->aircraft_home[stage->piloting - 1];
    stage->camera.params = stage->saved_camera_params;
  }
  /* Nothing stays in the air: whoever was falling lands, and the aircraft
   * with nobody in them are back on their stands. */
  if (stage->chute.active) stage->camera.params = stage->saved_camera_params;
  stage->chute.active = false;
  for (size_t i = 0; i < stage->aircraft_count; i++)
    if (stage->aircraft[i].doomed) stage->aircraft[i] = stage->aircraft_home[i];
  stage->piloting = 0;
  set_streaming(stage, false);
}

/* The area the city's ground covers, which an aircraft is kept over. */
static bool over_the_world(const driving_stage_t* stage, vec3_t p,
                           vec3_t* middle) {
  const city_package_t* package = stage->scene.package;
  if (!package || package->chunk_count == 0) return true;
  float lo_x = 1e30f, lo_z = 1e30f, hi_x = -1e30f, hi_z = -1e30f;
  for (size_t i = 0; i < package->chunk_count; i++) {
    const aabb_t* b = &package->chunks[i].bounds;
    lo_x = fminf(lo_x, b->min.x), hi_x = fmaxf(hi_x, b->max.x);
    lo_z = fminf(lo_z, b->min.z), hi_z = fmaxf(hi_z, b->max.z);
  }
  *middle = vec3((lo_x + hi_x) / 2, 0, (lo_z + hi_z) / 2);
  return p.x > lo_x + 30 && p.x < hi_x - 30 && p.z > lo_z + 30 &&
         p.z < hi_z - 30;
}

static float height_under(const driving_stage_t* stage, vec3_t p) {
  ray_t down = {vec3(p.x, p.y + 2.0f, p.z), vec3(0, -1, 0)};
  ray_cast_result_t hit;
  if (physics_ray_cast(stage->physics, down, 3000.0f, AIRCRAFT_LAYERS, &hit))
    return hit.hit.point.y;
  return -1000.0f; /* nothing streamed in: do not fall through the world */
}

/* Whether the aircraft is about to fly into a wall. */
static bool about_to_hit(const driving_stage_t* stage, const aircraft_t* a,
                         float dt) {
  vec3_t v = aircraft_velocity(a);
  float speed = vec3_length(v);
  if (speed < 3.0f) return false;
  vec3_t direction = vec3_scale(v, 1.0f / speed);
  float reach = speed * dt * 2.0f + aircraft_radius(a->kind) * 0.5f;
  ray_t ahead = {
      vec3_add(a->position, vec3(0, aircraft_height(a->kind) * 0.5f, 0)),
      direction};
  ray_cast_result_t hit;
  return physics_ray_cast(stage->physics, ahead, reach,
                          COLLISION_LAYER_BUILDING, &hit);
}

/* One step of an aircraft, kept over the world, and whether it crashed. */
static bool fly(driving_stage_t* stage, aircraft_t* a,
                aircraft_controls_t controls, bool say_turn_back, float dt) {
  float ground = height_under(stage, a->position);
  if (ground < -999.0f) ground = a->position.y - 0.001f; /* hold in the air */
  /* Past the edge of the map the aircraft is turned back towards it. */
  vec3_t middle;
  if (!over_the_world(stage, a->position, &middle)) {
    float want =
        atan2f(-(middle.x - a->position.x), -(middle.z - a->position.z));
    float off = want - a->yaw;
    off -= 6.2831853f * floorf(off / 6.2831853f + 0.5f);
    a->yaw += fmaxf(-1.0f, fminf(1.0f, off)) * dt;
    if (say_turn_back) driving_stage_say(stage, "TURN BACK");
  }
  bool crashed = about_to_hit(stage, a, dt);
  if (!crashed)
    crashed = aircraft_step(a, controls, ground, dt) == AIRCRAFT_CRASHED;
  return crashed;
}

void air_stage_step(driving_stage_t* stage, driving_input_t input, float dt) {
  aircraft_t* a = &stage->aircraft[stage->piloting - 1];
  stage->air_stick = aircraft_controls_from_input(input);
  bool crashed = fly(stage, a, stage->air_stick, true, dt);
  /* The walker rides along, so what follows the player follows the aircraft. */
  stage->walker.position = a->position;
  stage->walker.speed = 0;
  if (crashed) {
    driving_stage_crash_at(stage, vec3_add(a->position, vec3(0, 1.0f, 0)));
    stage->aircraft[stage->piloting - 1] =
        stage->aircraft_home[stage->piloting - 1];
    stage->piloting = 0;
    set_streaming(stage, false);
  }
}

void air_stage_step_adrift(driving_stage_t* stage, float dt) {
  for (size_t i = 0; i < stage->aircraft_count; i++) {
    aircraft_t* a = &stage->aircraft[i];
    if (!a->doomed) continue;
    aircraft_controls_t none = {0, 0, 0, 0};
    if (fly(stage, a, none, false, dt)) {
      driving_stage_crash_at(stage, vec3_add(a->position, vec3(0, 1.0f, 0)));
      stage->aircraft[i] = stage->aircraft_home[i];
    }
  }
}

#define CHUTE_FALL_SPEED 5.5f
#define CHUTE_FLARE_SPEED 2.6f
#define CHUTE_OPENING_START 0.7f /* seconds of free fall */
#define CHUTE_OPENING_TIME 0.9f

/* How far open the canopy is, 0 to 1. */
static float chute_open(const driving_stage_t* stage) {
  float t = (stage->chute.time - CHUTE_OPENING_START) / CHUTE_OPENING_TIME;
  return t < 0 ? 0.0f : (t > 1 ? 1.0f : t);
}

void air_stage_chute_pose(const driving_stage_t* stage, float* fall,
                          float* flail, float* phase) {
  float open = chute_open(stage);
  /* Belly down with the limbs thrashing in free fall; upright, the arms up to
   * the lines and the legs hanging, once the canopy has filled. */
  *fall = -0.85f * (1.0f - open);
  *flail = 0.75f * (1.0f - open) + 0.3f * open;
  *phase = stage->chute.time * (open < 1.0f ? 11.0f : 1.8f);
}

void air_stage_step_chute(driving_stage_t* stage, driving_input_t input,
                          float dt) {
  walker_t* w = &stage->walker;
  vec3_t* v = &stage->chute.velocity;
  stage->chute.time += dt;
  float open = chute_open(stage);
  if (!stage->chute.opened && stage->chute.time > CHUTE_OPENING_START + 0.25f) {
    /* The canopy fills with a jolt. */
    stage->chute.opened = true;
    if (stage->audio) audio_device_trigger_impact(stage->audio, 0.22f);
  }
  /* The ground, or a roof, under the person. */
  ray_t down = {vec3(w->position.x, w->position.y + 2.0f, w->position.z),
                vec3(0, -1, 0)};
  ray_cast_result_t hit;
  bool found =
      physics_ray_cast(stage->physics, down, 4000.0f, AIRCRAFT_LAYERS, &hit);
  float ground = found ? hit.hit.point.y : w->position.y - 1000.0f;
  stage->chute.height = w->position.y - ground;
  /* Steering turns them, the throttle drifts forward, and holding the brake
   * (or the last few metres) flares: the canopy is hauled down and they slow
   * for the landing. With it shut there is no steering. */
  bool flare =
      open >= 1.0f && (input.brake > 0.3f || stage->chute.height < 3.5f);
  w->yaw -= input.steering * 1.3f * dt * open;
  stage->chute.bank +=
      (input.steering * open - stage->chute.bank) * fminf(1.0f, 3.0f * dt);
  vec3_t forward = walker_forward(w);
  float throttle = fminf(fmaxf(input.throttle, 0.0f), 1.0f);
  float drift = open * (1.5f + 5.0f * throttle) * (flare ? 0.35f : 1.0f);
  float ease = fminf(1.0f, 1.2f * dt);
  v->x += (forward.x * drift - v->x) * ease;
  v->z += (forward.z * drift - v->z) * ease;
  /* Free fall at first, then the canopy takes hold. */
  v->y -= 9.8f * dt;
  if (open > 0) {
    float want = flare ? -CHUTE_FLARE_SPEED : -CHUTE_FALL_SPEED;
    v->y += (want - v->y) * fminf(1.0f, 3.0f * dt * open);
  }
  v->y = fmaxf(v->y, -45.0f);
  w->position = vec3_add(w->position, vec3_scale(*v, dt));
  w->speed = 0;
  if (found && w->position.y <= ground + 0.05f) {
    float speed = -v->y;
    w->position.y = ground;
    w->grounded = true;
    w->fall = 0;
    stage->chute.active = false;
    set_streaming(stage, false);
    stage->camera.params = stage->saved_camera_params;
    if (speed > 14.0f) {
      /* The canopy had no time to open. */
      driving_stage_say(stage, "WASTED");
      driving_stage_reset(stage);
    } else {
      driving_stage_say(stage, "LANDED");
    }
  }
}

void air_stage_follow(driving_stage_t* stage, float frame_seconds) {
  const aircraft_t* a = air_stage_piloted(stage);
  if (!a) return;
  chase_camera_update(&stage->camera, camera_target(a), aircraft_velocity(a),
                      NULL, frame_seconds);
}

void air_stage_draw(driving_stage_t* stage, float night) {
  (void)night;
  if (!stage->has_air) return;
  for (size_t i = 0; i < stage->aircraft_count; i++) {
    const aircraft_t* a = &stage->aircraft[i];
    float dx = a->position.x - stage->camera.position.x;
    float dz = a->position.z - stage->camera.position.z;
    if (dx * dx + dz * dz > 700.0f * 700.0f) continue;
    bool seated = (int)i == stage->piloting - 1 && stage->air_view == 1;
    if (seated)
      aircraft_visual_draw_spinners(&stage->air_visual, stage->renderer, a);
    else
      aircraft_visual_draw(&stage->air_visual, stage->renderer, a, (int)i);
  }
  if (air_stage_piloted(stage))
    aircraft_visual_draw_sight(stage->renderer, air_stage_piloted(stage));
  if (stage->chute.active) {
    const driving_stage_t* c = stage;
    float open = chute_open(c);
    float fall, flail, phase;
    air_stage_chute_pose(c, &fall, &flail, &phase);
    if (stage->has_figures) {
      /* Tipped over about the middle of the body, not the feet. */
      float lift = fall < -0.01f ? 0.6f : 0.0f;
      pedestrian_draw_figure_fallen(
          &stage->people_meshes, stage->renderer,
          vec3_add(stage->walker.position, vec3(0, -lift, 0)),
          stage->walker.yaw, 0.0f, phase, stage->walker_outfit, fall, lift,
          flail, 0.0f);
    }
    /* The canopy sways like a pendulum and leans into a turn. */
    float sway = 0.07f * sinf(stage->chute.time * 2.1f) * open;
    aircraft_visual_draw_canopy(&stage->air_visual, stage->renderer,
                                stage->walker.position, stage->walker.yaw, open,
                                sway + 0.3f * stage->chute.bank);
  }
}

void air_stage_draw_gauge(driving_stage_t* stage) {
  if (stage->chute.active) {
    renderer_t* r = stage->renderer;
    float width = (float)stage->window->drawable_width;
    float height = (float)stage->window->drawable_height;
    float scale = width / 640.0f;
    if (scale < 2) scale = 2;
    char text[48];
    snprintf(text, sizeof(text), "ALT %3.0f M", stage->chute.height);
    renderer_draw_text(r, width - 120.0f * scale, height - 70.0f * scale,
                       scale * 1.6f, vec3(0.7f, 0.9f, 1.0f), text);
    snprintf(text, sizeof(text), "%3.0f M/S DOWN", -stage->chute.velocity.y);
    renderer_draw_text(r, width - 120.0f * scale, height - 56.0f * scale,
                       scale * 1.6f, vec3(1, 1, 1), text);
    return;
  }
  const aircraft_t* a = air_stage_piloted(stage);
  if (!a) return;
  renderer_t* r = stage->renderer;
  float width = (float)stage->window->drawable_width;
  float height = (float)stage->window->drawable_height;
  float scale = width / 640.0f;
  if (scale < 2) scale = 2;
  float x = width - 120.0f * scale, y = height - 70.0f * scale;
  char line[48];
  snprintf(line, sizeof(line), "%3.0f KM/H",
           vec3_length(aircraft_velocity(a)) * 3.6f);
  renderer_draw_text(r, x, y, scale * 1.6f, vec3(1, 1, 1), line);
  y += 14.0f * scale;
  float ground = height_under(stage, a->position);
  snprintf(line, sizeof(line), "ALT %3.0f M",
           ground < -999.0f ? 0.0f : a->position.y - ground);
  renderer_draw_text(r, x, y, scale * 1.6f, vec3(0.7f, 0.9f, 1.0f), line);
  y += 14.0f * scale;
  if (aircraft_is_fixed_wing(a->kind)) {
    float bar = 100.0f * scale;
    renderer_draw_screen_line(r, x, y + 4 * scale, x + bar, y + 4 * scale,
                              4 * scale, vec3(0.12f, 0.12f, 0.14f));
    renderer_draw_screen_line(r, x, y + 4 * scale, x + bar * a->power,
                              y + 4 * scale, 4 * scale, vec3(0.4f, 0.9f, 0.4f));
    renderer_draw_text(r, x, y + 8 * scale, scale, vec3(0.7f, 0.75f, 0.85f),
                       "POWER");
  }
}

render_camera_t air_stage_cockpit_camera(const driving_stage_t* stage) {
  const aircraft_t* a = air_stage_piloted(stage);
  transform_t t = aircraft_transform(a);
  vec3_t eye = transform_point(t, aircraft_cockpit_eye(a->kind));
  vec3_t forward = quat_rotate(t.rotation, vec3(0, 0, -1));
  render_camera_t view = {eye,
                          vec3_add(eye, vec3_scale(forward, 50.0f)),
                          quat_rotate(t.rotation, vec3(0, 1, 0)),
                          1.2f,
                          0.12f,
                          8000.0f};
  return view;
}

void air_stage_draw_cockpit(driving_stage_t* stage) {
  const aircraft_t* a = air_stage_piloted(stage);
  if (!a || stage->air_view != 1) return;
  aircraft_visual_draw_cockpit(&stage->air_visual, stage->renderer, a,
                               stage->air_stick);
}

#define MISSILE_SPEED 190.0f
#define MISSILE_LIFE 8.0f
#define MISSILE_BLAST_RADIUS 12.0f
#define STRIKE_TIME 2.2f

/* Where each missile hangs under the wing, in the aircraft's frame. */
static vec3_t rail(aircraft_kind_t kind, int index) {
  float side = (index & 1) ? 1.0f : -1.0f;
  float k = (float)((index >> 1) & 1);
  return kind == AIRCRAFT_JET ? vec3(side * (1.9f + 1.3f * k), 1.2f, 1.2f)
                              : vec3(side * (2.3f + 1.0f * k), 1.6f, -1.8f);
}

bool air_stage_fire(driving_stage_t* stage) {
  const aircraft_t* a = air_stage_piloted(stage);
  if (!a || !aircraft_has_missiles(a->kind) || stage->paused ||
      stage->map_open || stage->missile_cooldown > 0)
    return false;
  for (size_t i = 0; i < sizeof(stage->missiles) / sizeof(stage->missiles[0]);
       i++) {
    if (stage->missiles[i].active) continue;
    transform_t t = aircraft_transform(a);
    vec3_t forward = quat_rotate(t.rotation, vec3(0, 0, -1));
    stage->missiles[i].active = true;
    stage->missiles[i].age = 0;
    stage->missiles[i].position =
        transform_point(t, rail(a->kind, stage->missile_next));
    stage->missiles[i].velocity =
        vec3_add(vec3_scale(forward, MISSILE_SPEED), aircraft_velocity(a));
    stage->missile_next = (stage->missile_next + 1) & 3;
    stage->missile_cooldown = 0.45f;
    if (stage->audio) audio_device_trigger_shot(stage->audio);
    driving_stage_commit_crime(stage);
    return true;
  }
  return false;
}

/* A missile goes off at `at`: a fireball, and whoever is near is thrown. */
static void strike(driving_stage_t* stage, vec3_t at) {
  size_t count = sizeof(stage->strikes) / sizeof(stage->strikes[0]);
  size_t slot = 0;
  for (size_t i = 0; i < count; i++) {
    if (stage->strikes[i].age <= 0) {
      slot = i;
      break;
    }
    if (stage->strikes[i].age > stage->strikes[slot].age) slot = i;
  }
  stage->strikes[slot].at = at;
  stage->strikes[slot].age = 0.001f;
  if (stage->audio) audio_device_trigger_impact(stage->audio, 1.0f);
  if (stage->people) pedestrians_blast(stage->people, at, MISSILE_BLAST_RADIUS);
  /* Cars in the blast are wrecked: they leave the road for good. */
  if (stage->traffic) {
    int variant;
    vec3_t position, paint;
    float yaw;
    for (int i = 0; i < 6; i++)
      if (!traffic_take_car(stage->traffic, at, MISSILE_BLAST_RADIUS, &variant,
                            &position, &yaw, &paint))
        break;
  }
}

void air_stage_step_missiles(driving_stage_t* stage, float dt) {
  if (stage->missile_cooldown > 0) stage->missile_cooldown -= dt;
  size_t count = sizeof(stage->strikes) / sizeof(stage->strikes[0]);
  for (size_t i = 0; i < count; i++) {
    if (stage->strikes[i].age <= 0) continue;
    stage->strikes[i].age += dt;
    if (stage->strikes[i].age > STRIKE_TIME) stage->strikes[i].age = 0;
  }
  count = sizeof(stage->missiles) / sizeof(stage->missiles[0]);
  for (size_t i = 0; i < count; i++) {
    if (!stage->missiles[i].active) continue;
    vec3_t from = stage->missiles[i].position;
    stage->missiles[i].age += dt;
    stage->missiles[i].velocity.y -= 2.0f * dt;
    vec3_t step = vec3_scale(stage->missiles[i].velocity, dt);
    float length = vec3_length(step);
    ray_t ray = {from, vec3_scale(step, 1.0f / length)};
    ray_cast_result_t hit;
    if (physics_ray_cast(stage->physics, ray, length, COLLISION_LAYER_ALL,
                         &hit)) {
      stage->missiles[i].active = false;
      strike(stage, hit.hit.point);
    } else if (stage->missiles[i].age > MISSILE_LIFE) {
      stage->missiles[i].active = false;
    } else {
      stage->missiles[i].position = vec3_add(from, step);
    }
  }
}

void air_stage_draw_missiles(driving_stage_t* stage) {
  if (!stage->has_air) return;
  for (size_t i = 0; i < sizeof(stage->missiles) / sizeof(stage->missiles[0]);
       i++) {
    if (!stage->missiles[i].active) continue;
    vec3_t velocity = stage->missiles[i].velocity;
    aircraft_visual_draw_missile(
        &stage->air_visual, stage->renderer, stage->missiles[i].position,
        vec3_scale(velocity, 1.0f / vec3_length(velocity)));
  }
  for (size_t i = 0; i < sizeof(stage->strikes) / sizeof(stage->strikes[0]);
       i++)
    if (stage->strikes[i].age > 0)
      car_visual_draw_blast(&stage->car_visual, stage->renderer,
                            stage->strikes[i].at, stage->strikes[i].age);
}
