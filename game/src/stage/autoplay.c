#include "autoplay.h"

#include <math.h>
#include <string.h>

#include "driving_stage.h"

#define PI_F 3.14159265f
#define TRUCK_FIRST_VARIANT 11
#define CAR_CRUISE 12.0f
#define CLIMB_ABOVE_AIRFIELD 110.0f
#define AIRCRAFT_LAYERS (COLLISION_LAYER_GROUND | COLLISION_LAYER_BUILDING)

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static float wrap(float angle) {
  return angle - 2.0f * PI_F * floorf(angle / (2.0f * PI_F) + 0.5f);
}

void autoplay_init(autoplay_t* a) {
  memset(a, 0, sizeof(*a));
  a->enabled = true;
  a->target = -1;
  a->after_airport = ACT_BOARD_PLANE;
  a->after_steal = ACT_RUN_OVER;
}

const char* autoplay_label(const autoplay_t* a) {
  static const char* names[] = {
      "STARTING",          "DRIVE",         "HIT A CAR",
      "STEAL A CAR",       "RUN OVER",      "STEAL A TRUCK",
      "TO THE AIRPORT",    "BOARD A PLANE", "TAKE OFF",
      "FLY OVER THE CITY", "LAND",          "STEAL A HELICOPTER",
      "HELICOPTER",        "THE END"};
  return names[a->act];
}

static void enter(autoplay_t* a, autoplay_act_t act) {
  a->act = act;
  a->act_time = 0;
  a->step = 0;
  a->target = -1;
  a->path_count = 0;
  a->path_clock = 99.0f;
  a->stuck = 0;
  a->reverse_time = 0;
  a->waypoint = 0;
  a->brought = false;
  a->block_time = 0;
  a->pass_time = 0;
  a->detour_hold = 0;
  a->detour_sign = 0;
}

/* ---- where things are ---- */

static vec3_t player_at(const driving_stage_t* s) {
  if (s->piloting > 0) return s->aircraft[s->piloting - 1].position;
  if (s->on_foot) return s->walker.position;
  return physics_body(s->physics, s->vehicle.body)->position;
}

static bool is_car(int variant) { return variant < FLEET_CAR_VARIANTS; }
static bool is_truck(int variant) { return variant >= TRUCK_FIRST_VARIANT; }

/* The nearest traffic car of the wanted sort to a point, or -1. */
static int nearest_vehicle(const driving_stage_t* s, bool trucks, vec3_t from,
                           float within) {
  if (!s->traffic) return -1;
  int best = -1;
  float best_distance = within;
  for (size_t i = 0; i < traffic_car_count(s->traffic); i++) {
    traffic_car_t car = traffic_car(s->traffic, i);
    if (car.pursuing) continue;
    if (trucks ? !is_truck(car.variant) : !is_car(car.variant)) continue;
    float d = hypotf(car.position.x - from.x, car.position.z - from.z);
    if (d < best_distance) best = (int)i, best_distance = d;
  }
  return best;
}

static int nearest_person(const driving_stage_t* s, vec3_t from, float within,
                          float* x, float* z) {
  if (!s->people) return -1;
  int best = -1;
  float best_distance = within;
  for (size_t i = 0; i < pedestrians_count(s->people); i++) {
    pedestrian_t p = pedestrians_get(s->people, i);
    if (p.fall > 0) continue;
    float d = hypotf(p.position.x - from.x, p.position.z - from.z);
    if (d < best_distance) {
      best = (int)i, best_distance = d;
      *x = p.position.x, *z = p.position.z;
    }
  }
  return best;
}

static int people_down(const driving_stage_t* s) {
  int n = 0;
  for (size_t i = 0; s->people && i < pedestrians_count(s->people); i++)
    if (pedestrians_get(s->people, i).fall > 0) n++;
  return n;
}

static float height_above_ground(const driving_stage_t* s, vec3_t at) {
  ray_t down = {vec3(at.x, at.y + 2.0f, at.z), vec3(0, -1, 0)};
  ray_cast_result_t hit;
  if (physics_ray_cast(s->physics, down, 3000.0f, AIRCRAFT_LAYERS, &hit))
    return at.y - hit.hit.point.y;
  return at.y;
}

/* The airfield's own frame: the runway's middle and the way it lies. */
typedef struct {
  float ox, oz, ax, az; /* origin and the unit vector along the runway */
  float length;
  bool present;
} airfield_t;

static airfield_t airfield(const driving_stage_t* s) {
  airfield_t f = {0, 0, 1, 0, 900, false};
  const city_package_t* package = s->scene.package;
  if (!package || !package->airport.present) return f;
  f.present = true;
  f.ox = package->airport.x, f.oz = package->airport.z;
  f.ax = -sinf(package->airport.yaw), f.az = -cosf(package->airport.yaw);
  f.length = package->airport.length;
  return f;
}

/* The middle of the city's ground, which the flights cross. */
static vec3_t city_middle(const driving_stage_t* s) {
  const city_package_t* package = s->scene.package;
  float lo_x = 1e30f, lo_z = 1e30f, hi_x = -1e30f, hi_z = -1e30f;
  for (size_t i = 0; package && i < package->chunk_count; i++) {
    const aabb_t* b = &package->chunks[i].bounds;
    lo_x = fminf(lo_x, b->min.x), hi_x = fmaxf(hi_x, b->max.x);
    lo_z = fminf(lo_z, b->min.z), hi_z = fmaxf(hi_z, b->max.z);
  }
  if (lo_x > hi_x) return vec3(0, 0, 0);
  return vec3((lo_x + hi_x) / 2, 0, (lo_z + hi_z) / 2);
}

/* ---- driving ---- */

static float angle_to(vec3_t heading, float dx, float dz) {
  float d = hypotf(dx, dz);
  if (d < 1e-3f) return 0;
  float forward = (dx * heading.x + dz * heading.z) / d;
  float side = (dx * -heading.z + dz * heading.x) / d; /* + to the right */
  return atan2f(side, forward);
}

/* The speed to keep so as not to run into what is ahead: a car in the lane
 * ahead (moving, parked, or the police) holds the speed down. */
static float speed_limit_ahead(const driving_stage_t* s, vec3_t position,
                               vec3_t nose, float limit, int except,
                               bool passing) {
  float ahead_reach = passing ? 12.0f : 30.0f;
  float lane = passing ? 1.5f : 2.3f;
  if (s->traffic) {
    for (size_t i = 0; i < traffic_car_count(s->traffic); i++) {
      if ((int)i == except) continue;
      traffic_car_t car = traffic_car(s->traffic, i);
      float dx = car.position.x - position.x, dz = car.position.z - position.z;
      float ahead = dx * nose.x + dz * nose.z;
      float lateral = dx * -nose.z + dz * nose.x;
      if (ahead < 0.0f || ahead > ahead_reach || fabsf(lateral) > lane)
        continue;
      limit = fminf(limit, fmaxf(0.0f, car.speed + (ahead - 10.0f) * 0.7f));
    }
  }
  /* Cars crossing in front, at a junction: ease off. */
  if (s->traffic && !passing) {
    for (size_t i = 0; i < traffic_car_count(s->traffic); i++) {
      if ((int)i == except) continue;
      traffic_car_t car = traffic_car(s->traffic, i);
      if (car.speed < 1.5f) continue;
      float dx = car.position.x - position.x, dz = car.position.z - position.z;
      float ahead = dx * nose.x + dz * nose.z;
      float lateral = dx * -nose.z + dz * nose.x;
      if (ahead > 1.0f && ahead < 16.0f && fabsf(lateral) > lane &&
          fabsf(lateral) < 7.0f)
        limit = fminf(limit, 5.0f);
    }
  }
  for (size_t i = 0; i < s->parked.count; i++) {
    const parked_car_t* car = &s->parked.cars[i];
    if (car->taken) continue;
    float dx = car->position.x - position.x, dz = car->position.z - position.z;
    float ahead = dx * nose.x + dz * nose.z;
    if (ahead < 0.0f || ahead > ahead_reach) continue;
    float lateral = dx * -nose.z + dz * nose.x;
    if (fabsf(lateral) > (passing ? 1.5f : 2.1f) || ahead > 28.0f) continue;
    limit = fminf(limit, fmaxf(0.0f, (ahead - 7.0f) * 0.7f));
  }
  return limit;
}

/* How far to move over, sideways (+ to the right), to give the cars standing
 * close ahead room: parked cars sit a little into the lane, and a car only
 * just fits by them. */
static float room_to_pass(const driving_stage_t* s, vec3_t position,
                          vec3_t nose) {
  float push = 0;
  float best = 0;
  for (size_t i = 0; i < s->parked.count; i++) {
    const parked_car_t* car = &s->parked.cars[i];
    if (car->taken) continue;
    float dx = car->position.x - position.x, dz = car->position.z - position.z;
    float ahead = dx * nose.x + dz * nose.z;
    float lateral = dx * -nose.z + dz * nose.x;
    if (ahead < -2.0f || ahead > 14.0f || fabsf(lateral) > 3.0f) continue;
    float need = 3.0f - fabsf(lateral); /* metres short of a clear gap */
    if (need > best) best = need, push = lateral > 0 ? -need : need;
  }
  return clampf(push, -1.8f, 1.8f);
}

/* Steers the car at a point, at about `want` metres a second. */
static driving_input_t car_towards_except(autoplay_t* a,
                                          const driving_stage_t* s, float tx,
                                          float tz, float want, float dt,
                                          int except) {
  driving_input_t in = {0, 0, 0, false, false};
  const rigid_body_t* body = physics_body(s->physics, s->vehicle.body);
  vec3_t nose = quat_rotate(body->rotation, vec3(0, 0, -1));
  float speed = body->velocity.x * nose.x + body->velocity.z * nose.z;
  if (a->reverse_time > 0) {
    float angle = angle_to(vec3(nose.x, 0, nose.z), tx - body->position.x,
                           tz - body->position.z);
    a->reverse_time -= dt;
    in.brake = 1.0f; /* backs up when stopped */
    in.steering = a->reverse_side_hold ? 0.0f : (angle > 0 ? -1.0f : 1.0f);
    if (a->reverse_time <= 0) a->reverse_side_hold = false;
    return in;
  }
  /* Held up behind something for a while: pull out and go round it. */
  if (a->pass_time > 0) {
    /* Swing out: aim a few metres ahead and well to the side. */
    a->pass_time -= dt;
    tx = body->position.x + nose.x * 9.0f - nose.z * a->pass_side * 4.5f;
    tz = body->position.z + nose.z * 9.0f + nose.x * a->pass_side * 4.5f;
    want = fminf(want, 6.0f);
  }
  float push = a->pass_time > 0 ? 0.0f : room_to_pass(s, body->position, nose);
  tx += -nose.z * push;
  tz += nose.x * push;
  float dx = tx - body->position.x, dz = tz - body->position.z;
  float angle = angle_to(vec3(nose.x, 0, nose.z), dx, dz);
  in.steering = clampf(angle * 1.6f, -1.0f, 1.0f);
  float slow = 1.0f - 0.8f * clampf(fabsf(angle) / 1.1f, 0.0f, 1.0f);
  float free = want * slow;
  float target = speed_limit_ahead(s, body->position, nose, free, except,
                                   a->pass_time > 0);
  if (a->pass_time > 0) target = fmaxf(target, 3.0f); /* creep out round it */
  bool blocked = target < 1.0f; /* something is in the way */
  if (blocked && fabsf(speed) < 2.0f)
    a->block_time += dt;
  else
    a->block_time = fmaxf(0.0f, a->block_time - dt);
  if (a->block_time > 1.5f && a->pass_time <= 0) {
    a->reverse_time = 1.8f; /* back off first: the bumpers may be touching */
    a->reverse_side_hold = true;
    a->pass_time = 6.0f;
    /* Out to the left, into the road, first; the other way if that failed. */
    a->pass_tries++;
    a->pass_side = (a->pass_tries % 3 == 0) ? 1.0f : -1.0f;
    a->block_time = 0;
  }
  in.throttle = clampf((target - speed) * 0.4f, 0.0f, 1.0f);
  in.brake = clampf((speed - target) * 0.25f, 0.0f, 1.0f);
  if (want > 0 && fabsf(speed) < 0.7f && !blocked)
    a->stuck += dt;
  else
    a->stuck = 0;
  if (a->stuck > 1.3f) {
    a->stuck = 0;
    a->reverse_time = 1.6f;
  }
  return in;
}

static driving_input_t car_towards(autoplay_t* a, const driving_stage_t* s,
                                   float tx, float tz, float want, float dt) {
  return car_towards_except(a, s, tx, tz, want, dt, -1);
}

/* The next point to steer at on the road to (tx, tz), a little ahead. */
static void road_target_near(autoplay_t* a, driving_stage_t* s, float tx,
                             float tz, float lookahead, float* px, float* pz) {
  vec3_t me = player_at(s);
  a->path_clock += a->frame;
  if (a->path_clock > 0.7f && s->nav) {
    a->path_clock = 0;
    float length = 0;
    a->path_count = nav_path(s->nav, me.x, me.z, tx, tz, a->path, 128, &length);
  }
  *px = tx, *pz = tz;
  if (a->path_count == 0) return;
  /* The carrot: a point `lookahead` metres along the path from the place on
   * it nearest the player, so that it follows a bend instead of cutting it. */
  size_t nearest = 0;
  float nearest_d = 1e30f;
  for (size_t i = 0; i < a->path_count; i++) {
    float d = hypotf(a->path[2 * i] - me.x, a->path[2 * i + 1] - me.z);
    if (d < nearest_d) nearest = i, nearest_d = d;
  }
  float walked = 0, cx = a->path[2 * nearest], cz = a->path[2 * nearest + 1];
  /* From the nearest point on, but not from a point behind the player. */
  for (size_t i = nearest + 1; i < a->path_count; i++) {
    float nx = a->path[2 * i], nz = a->path[2 * i + 1];
    float seg = hypotf(nx - cx, nz - cz);
    if (walked + seg >= lookahead) {
      float t = seg > 1e-3f ? (lookahead - walked) / seg : 0.0f;
      *px = cx + (nx - cx) * t, *pz = cz + (nz - cz) * t;
      goto lane;
    }
    walked += seg;
    cx = nx, cz = nz;
  }
  *px = tx, *pz = tz; /* the path ends within reach: go straight at the end */
  return;
lane:
  /* Keep a little to the right of the centre line, as the way the road runs
   * here. */
  if (lookahead >= 12.0f) {
    for (size_t i = nearest; i + 1 < a->path_count; i++) {
      float dx = a->path[2 * i + 2] - a->path[2 * i];
      float dz = a->path[2 * i + 3] - a->path[2 * i + 1];
      float d = hypotf(dx, dz);
      if (d > 0.5f) {
        *px += -dz / d * 0.3f, *pz += dx / d * 0.3f;
        break;
      }
    }
  }
}

static void road_target(autoplay_t* a, driving_stage_t* s, float tx, float tz,
                        float* px, float* pz) {
  road_target_near(a, s, tx, tz, 16.0f, px, pz);
}

static driving_input_t car_by_road(autoplay_t* a, driving_stage_t* s, float tx,
                                   float tz, float want, float dt) {
  float px, pz;
  road_target(a, s, tx, tz, &px, &pz);
  return car_towards(a, s, px, pz, want, dt);
}

static float car_speed(const driving_stage_t* s) {
  const rigid_body_t* body = physics_body(s->physics, s->vehicle.body);
  return hypotf(body->velocity.x, body->velocity.z);
}

/* Brakes to a stop; true once stopped. */
static bool stop_car(const driving_stage_t* s, driving_input_t* in) {
  in->brake = 1.0f;
  in->throttle = 0;
  return car_speed(s) < 1.5f;
}

/* ---- walking ---- */

#define WALK_OBSTACLES \
  (COLLISION_LAYER_BUILDING | COLLISION_LAYER_BARRIER | COLLISION_LAYER_PROP)

/* Whether the walker could set off in `direction` (a unit vector) for a few
 * metres: no wall, rail or prop in the way, and not into the car left standing.
 */
static bool way_is_clear(const driving_stage_t* s, float dx, float dz) {
  const walker_t* w = &s->walker;
  /* A person is wider than a ray: three, the middle one and one to each side.
   */
  for (int k = -1; k <= 1; k++) {
    float c = cosf(0.28f * (float)k), sn = sinf(0.28f * (float)k);
    ray_t ray = {vec3_add(w->position, vec3(0, 0.55f, 0)),
                 vec3(dx * c - dz * sn, 0, dx * sn + dz * c)};
    ray_cast_result_t hit;
    if (physics_ray_cast(s->physics, ray, k == 0 ? 3.2f : 1.6f, WALK_OBSTACLES,
                         &hit))
      return false;
  }
  const rigid_body_t* car = physics_body(s->physics, s->vehicle.body);
  vec3_t nose = quat_rotate(car->rotation, vec3(0, 0, -1));
  vec3_t half = s->vehicle.params.chassis_half_extents;
  for (float t = 0.6f; t <= 2.6f; t += 0.5f) {
    float px = w->position.x + dx * t - car->position.x;
    float pz = w->position.z + dz * t - car->position.z;
    float along = px * nose.x + pz * nose.z;
    float across = px * -nose.z + pz * nose.x;
    if (fabsf(along) < half.z + 0.7f && fabsf(across) < half.x + 0.7f)
      return false;
  }
  return true;
}

/* On foot towards a point, taking the free direction nearest to it, and
 * sticking with a detour for a moment so as not to dither at a corner. */
static driving_input_t walk_straight(autoplay_t* a, const driving_stage_t* s,
                                     float tx, float tz) {
  driving_input_t in = {0, 0, 0, false, false};
  const walker_t* w = &s->walker;
  float dx = tx - w->position.x, dz = tz - w->position.z;
  float distance = fmaxf(hypotf(dx, dz), 0.01f);
  vec3_t forward = walker_forward(w);
  float want = angle_to(forward, dx, dz); /* + to the right of where we face */
  float goal_x = dx / distance, goal_z = dz / distance;
  float steer_x = goal_x, steer_z = goal_z;
  a->detour_hold -= a->frame;
  if (way_is_clear(s, goal_x, goal_z)) {
    a->detour_hold = 0;
    a->detour_sign = 0;
  } else if (a->detour_hold > 0 && way_is_clear(s, a->detour_x, a->detour_z)) {
    steer_x = a->detour_x, steer_z = a->detour_z;
  } else {
    /* Round it, always the same way about: the free direction nearest the
     * goal on one side. The side is the one with the smaller turn. */
    float goal_angle = atan2f(goal_z, goal_x);
    int first_k[2] = {99, 99};
    for (int side = 0; side < 2; side++)
      for (int k = 1; k < 16; k++) {
        float angle = goal_angle + (float)k * 0.3927f * (side ? 1.0f : -1.0f);
        if (way_is_clear(s, cosf(angle), sinf(angle))) {
          first_k[side] = k;
          break;
        }
      }
    if (a->detour_sign == 0) a->detour_sign = first_k[1] < first_k[0] ? 1 : -1;
    int k = first_k[a->detour_sign > 0 ? 1 : 0];
    if (k < 99) {
      float angle = goal_angle + (float)k * 0.3927f * (float)a->detour_sign;
      steer_x = cosf(angle), steer_z = sinf(angle);
      a->detour_x = steer_x, a->detour_z = steer_z;
      a->detour_hold = 0.8f;
    }
  }
  float angle = angle_to(forward, steer_x, steer_z);
  (void)want;
  in.steering = clampf(angle * 2.5f, -1.0f, 1.0f);
  in.throttle = fabsf(angle) < 0.7f ? 1.0f : 0.1f;
  in.handbrake = distance > 10.0f && fabsf(angle) < 0.4f; /* run */
  return in;
}

/* On foot to a point. */
static driving_input_t walk_towards(autoplay_t* a, driving_stage_t* s, float tx,
                                    float tz, float dt) {
  (void)dt;
  return walk_straight(a, s, tx, tz);
}

static float distance_to(vec3_t from, float x, float z) {
  return hypotf(x - from.x, z - from.z);
}

/* ---- flying ---- */

static driving_input_t aircraft_pitch(driving_input_t in, float pitch) {
  if (pitch >= 0)
    in.brake = clampf(pitch, 0.0f, 1.0f);
  else
    in.throttle = clampf(-pitch, 0.0f, 1.0f);
  return in;
}

/* Raises or cuts the throttle of an aeroplane towards a setting. */
static driving_input_t plane_power(driving_input_t in, const aircraft_t* p,
                                   float want) {
  if (p->power < want - 0.03f)
    in.handbrake = true;
  else if (p->power > want + 0.03f)
    in.lower = true;
  return in;
}

/* An aeroplane in the air: towards a point at a height, at a speed. */
static driving_input_t plane_towards(const aircraft_t* p, float tx, float tz,
                                     float altitude, float speed_want,
                                     float max_path) {
  driving_input_t in = {0, 0, 0, false, false};
  vec3_t h = aircraft_heading(p);
  float angle = angle_to(h, tx - p->position.x, tz - p->position.z);
  in.steering = clampf(angle * 1.5f, -1.0f, 1.0f);
  float path = clampf((altitude - p->position.y) * 0.006f, -0.12f, max_path);
  /* Short of speed: no climbing. */
  if (p->speed < 38.0f) path = fminf(path, 0.0f);
  in = aircraft_pitch(in, clampf((path - p->pitch) * 3.0f, -1.0f, 1.0f));
  float power = clampf(
      0.6f + (speed_want - p->speed) * 0.03f + (path > 0.05f ? 0.2f : 0.0f),
      0.15f, 1.0f);
  return plane_power(in, p, power);
}

static driving_input_t helicopter_towards(const aircraft_t* p, float tx,
                                          float tz, float altitude,
                                          bool forward) {
  driving_input_t in = {0, 0, 0, false, false};
  vec3_t h = aircraft_heading(p);
  float angle = angle_to(h, tx - p->position.x, tz - p->position.z);
  in.steering = clampf(angle * 1.5f, -1.0f, 1.0f);
  float error = altitude - p->position.y;
  if (error > 3.0f)
    in.handbrake = true;
  else if (error < -3.0f)
    in.lower = true;
  if (forward) in.throttle = fabsf(angle) < 0.8f ? 1.0f : 0.0f;
  return in;
}

/* ---- the acts ---- */

static driving_input_t idle_input(void) {
  driving_input_t in = {0, 0, 0, false, false};
  return in;
}

/* Out of the car when it is nearly stopped; true once on foot. */
static bool get_out(driving_stage_t* s, driving_input_t* in) {
  if (s->on_foot) return true;
  if (stop_car(s, in) && s->entering <= 0) driving_stage_interact(s);
  return s->on_foot;
}

/* Walks to a car that is held still and takes it; true once in it. */
static bool take_standing_car(autoplay_t* a, driving_stage_t* s, bool truck,
                              driving_input_t* in) {
  vec3_t me = player_at(s);
  if (!s->on_foot) return s->entering <= 0 && a->step >= 2;
  if (s->entering > 0) {
    a->step = 2;
    return false;
  }
  if (a->target < 0 || a->target >= (int)traffic_car_count(s->traffic) ||
      (truck ? !is_truck(traffic_car(s->traffic, (size_t)a->target).variant)
             : !is_car(traffic_car(s->traffic, (size_t)a->target).variant)))
    a->target = nearest_vehicle(s, truck, me, 400.0f);
  if (a->target < 0) return false;
  traffic_car_t car = traffic_car(s->traffic, (size_t)a->target);
  float d = distance_to(me, car.position.x, car.position.z);
  if (d < 45.0f) traffic_car_hold(s->traffic, (size_t)a->target, true);
  /* Straight at its middle: nothing the walker bumps into is in the way. */
  float tx = car.position.x, ty = car.position.z;
  *in = walk_towards(a, s, tx, ty, 1.0f / 60.0f);
  /* Any car in reach will do for a car, so long as the one at hand is not a
   * lorry that was wanted. */
  bool in_reach = distance_to(me, car.position.x, car.position.z) < 4.05f;
  for (size_t i = 0;
       !truck && !in_reach && s->traffic && i < traffic_car_count(s->traffic);
       i++) {
    traffic_car_t other = traffic_car(s->traffic, i);
    in_reach = !other.pursuing && is_car(other.variant) &&
               distance_to(me, other.position.x, other.position.z) < 4.05f;
  }
  for (size_t i = 0; !truck && !in_reach && i < s->parked.count; i++) {
    const parked_car_t* other = &s->parked.cars[i];
    in_reach = !other->taken &&
               distance_to(me, other->position.x, other->position.z) < 4.05f;
  }
  if (in_reach) {
    traffic_car_hold(s->traffic, (size_t)a->target, false);
    driving_stage_interact(s);
    a->step = 2;
  }
  return false;
}

static void release_target(autoplay_t* a, driving_stage_t* s) {
  if (s->traffic && a->target >= 0 &&
      a->target < (int)traffic_car_count(s->traffic))
    traffic_car_hold(s->traffic, (size_t)a->target, false);
}

/* The drivable street nearest a point: its index, how far along it the point
 * is, and its length; false if there is none within 40 m. */
static bool nearest_street(const driving_stage_t* s, float x, float z,
                           int* route, float* along, float* length, float* tx,
                           float* tz) {
  float best = 40.0f;
  bool found = false;
  for (size_t r = 0; r < s->scene.route_count; r++) {
    const city_route_t* road = &s->scene.routes[r];
    if (!road->drivable || road->point_count < 2) continue;
    float walked = 0;
    for (size_t i = 1; i < road->point_count; i++) {
      float ax = road->points[2 * i - 2], az = road->points[2 * i - 1];
      float bx = road->points[2 * i], bz = road->points[2 * i + 1];
      float dx = bx - ax, dz = bz - az;
      float seg = hypotf(dx, dz);
      if (seg < 1e-3f) continue;
      float t = clampf(((x - ax) * dx + (z - az) * dz) / (seg * seg), 0, 1);
      float d = hypotf(ax + dx * t - x, az + dz * t - z);
      if (d < best) {
        best = d;
        found = true;
        *route = (int)r;
        *along = walked + seg * t;
        *tx = dx / seg, *tz = dz / seg;
      }
      walked += seg;
    }
    if (found && *route == (int)r) *length = walked;
  }
  return found;
}

/* Brings a traffic car to stand in the street some way along the road the
 * player is following (its path), facing the other way, as the traffic does
 * when it puts cars on the road out of sight: the scene wants a car or a
 * lorry within reach. */
static bool stand_vehicle_ahead(autoplay_t* a, driving_stage_t* s, int index,
                                float distance, bool same_lane) {
  if (!s->traffic || index < 0 || a->path_count < 2) return false;
  vec3_t me = player_at(s);
  float walked = 0, px = me.x, pz = me.z, qx = me.x, qz = me.z;
  bool reached = false;
  for (size_t i = 0; i < a->path_count && !reached; i++) {
    float x = a->path[2 * i], z = a->path[2 * i + 1];
    walked += hypotf(x - px, z - pz);
    qx = px, qz = pz;
    px = x, pz = z;
    reached = walked >= distance;
  }
  /* A short path will do if it is long enough to be a good way off. */
  if (!reached && walked < 60.0f) return false;
  int route = 0;
  float along = 0, length = 0, tx = 0, tz = 0;
  if (!nearest_street(s, px, pz, &route, &along, &length, &tx, &tz))
    return false;
  /* Facing back down the way the player is coming. */
  int dir = ((px - qx) * tx + (pz - qz) * tz) >= 0 ? 1 : -1;
  int facing = same_lane ? dir : -dir;
  float s_along = facing > 0 ? along : length - along;
  traffic_car_place(s->traffic, (size_t)index, route, facing, s_along);
  traffic_car_hold(s->traffic, (size_t)index, true);
  return true;
}

/* Puts a person on the pavement of the street ahead, as the people are put
 * on the pavements out of sight; false if there is no street to do it. */
static bool stand_person_ahead(autoplay_t* a, driving_stage_t* s, int index,
                               float distance) {
  if (!s->people || index < 0 || a->path_count < 2) return false;
  vec3_t me = player_at(s);
  float walked = 0, px = me.x, pz = me.z;
  bool reached = false;
  for (size_t i = 0; i < a->path_count && !reached; i++) {
    float x = a->path[2 * i], z = a->path[2 * i + 1];
    walked += hypotf(x - px, z - pz);
    px = x, pz = z;
    reached = walked >= distance;
  }
  if (!reached) return false;
  int route = 0;
  float along = 0, length = 0, tx = 0, tz = 0;
  if (!nearest_street(s, px, pz, &route, &along, &length, &tx, &tz))
    return false;
  if (s->scene.routes[route].pavement < 0.4f || length < 20.0f) return false;
  pedestrians_place(s->people, (size_t)index, route, 1, 1,
                    clampf(along, 6.0f, length - 6.0f));
  return true;
}

/* Somewhere in the middle of the town that a car can drive to from here: the
 * drivable road point nearest the middle of all the roads which the nearest
 * road to the player joins. Worked out once. */
static void find_town(autoplay_t* a, driving_stage_t* s) {
  if (a->town_found || !s->nav) return;
  a->town_found = true;
  float cx = 0, cz = 0;
  size_t n = 0;
  for (size_t r = 0; r < s->scene.route_count; r++) {
    const city_route_t* road = &s->scene.routes[r];
    if (!road->drivable) continue;
    for (size_t i = 0; i < road->point_count; i += 4)
      cx += road->points[2 * i], cz += road->points[2 * i + 1], n++;
  }
  if (n == 0) return;
  cx /= (float)n, cz /= (float)n;
  vec3_t me = player_at(s);
  /* The ten road points nearest the middle, then the first reachable. */
  float best_d[10];
  float best_x[10], best_z[10];
  size_t kept = 0;
  for (size_t r = 0; r < s->scene.route_count; r++) {
    const city_route_t* road = &s->scene.routes[r];
    if (!road->drivable) continue;
    for (size_t i = 0; i < road->point_count; i += 3) {
      float x = road->points[2 * i], z = road->points[2 * i + 1];
      float d = hypotf(x - cx, z - cz);
      if (hypotf(x - me.x, z - me.z) < 300.0f) continue; /* not the airport */
      size_t at = kept < 10 ? kept : 10;
      while (at > 0 && best_d[at - 1] > d) at--;
      if (at >= 10) continue;
      size_t last = kept < 10 ? kept : 9;
      for (size_t k = last; k > at; k--)
        best_d[k] = best_d[k - 1], best_x[k] = best_x[k - 1],
        best_z[k] = best_z[k - 1];
      best_d[at] = d, best_x[at] = x, best_z[at] = z;
      if (kept < 10) kept++;
    }
  }
  for (size_t k = 0; k < kept; k++) {
    float scratch[256];
    if (nav_path(s->nav, me.x, me.z, best_x[k], best_z[k], scratch, 128, NULL) >
        0) {
      a->town_x = best_x[k], a->town_z = best_z[k];
      return;
    }
  }
  a->town_x = cx, a->town_z = cz;
}

/* The road to somewhere in town, so that there is a path to stand things on. */
static void road_to_town(autoplay_t* a, driving_stage_t* s, float* px,
                         float* pz) {
  vec3_t me = player_at(s);
  find_town(a, s);
  float gx = a->town_x, gz = a->town_z;
  if (distance_to(me, gx, gz) < 200.0f) gx = s->guide.x, gz = s->guide.z;
  road_target_near(a, s, gx, gz, 16.0f, px, pz);
}

static void act_hit_car(autoplay_t* a, driving_stage_t* s, driving_input_t* in,
                        float dt) {
  vec3_t me = player_at(s);
  if (s->on_foot) {
    driving_stage_interact(s); /* back in the car */
    return;
  }
  if (!a->brought) {
    /* A car broken down in the lane a good way ahead. */
    float px, pz;
    road_to_town(a, s, &px, &pz);
    int car = nearest_vehicle(s, false, me, 5000.0f);
    if (car >= 0 && stand_vehicle_ahead(a, s, car, 190.0f, true)) {
      a->target = car;
      a->brought = true;
    } else if (a->act_time > 3.0f) {
      a->target = car;
      a->brought = true;
    }
    *in = car_towards(a, s, px, pz, CAR_CRUISE, dt);
    return;
  }
  if (a->target < 0) {
    *in = car_by_road(a, s, s->guide.x, s->guide.z, CAR_CRUISE, dt);
    return;
  }
  traffic_car_t car = traffic_car(s->traffic, (size_t)a->target);
  float d = distance_to(me, car.position.x, car.position.z);
  if (d < 35.0f) {
    *in = car_towards_except(a, s, car.position.x, car.position.z, 14.0f, dt,
                             a->target);
  } else {
    *in = car_by_road(a, s, car.position.x, car.position.z, CAR_CRUISE, dt);
  }
  /* Close enough that the bumpers have met (the cars are about 4.5 m long). */
  if (d < 6.0f || a->act_time > 60.0f) enter(a, ACT_STEAL_CAR);
}

static void act_run_over(autoplay_t* a, driving_stage_t* s, driving_input_t* in,
                         float dt) {
  vec3_t me = player_at(s);
  if (s->on_foot) {
    if (s->entering <= 0) enter(a, ACT_STEAL_CAR);
    return;
  }
  if (a->step == 0) {
    a->downed_before = people_down(s);
    a->step = 1;
  }
  if (!a->brought) {
    /* Somebody on the pavement a good way ahead. */
    float px, pz, ex, ez;
    road_to_town(a, s, &px, &pz);
    int person = nearest_person(s, me, 5000.0f, &ex, &ez);
    if (person >= 0 && stand_person_ahead(a, s, person, 170.0f)) {
      a->target = person;
      a->brought = true;
    } else if (a->act_time > 3.0f) {
      a->target = person;
      a->brought = true;
    }
    *in = car_towards(a, s, px, pz, CAR_CRUISE, dt);
    return;
  }
  float px = s->guide.x, pz = s->guide.z;
  if (a->target >= 0 && a->target < (int)pedestrians_count(s->people)) {
    pedestrian_t person = pedestrians_get(s->people, (size_t)a->target);
    px = person.position.x, pz = person.position.z;
  }
  float d = distance_to(me, px, pz);
  if (d < 30.0f)
    *in = car_towards(a, s, px, pz, 12.0f, dt);
  else
    *in = car_by_road(a, s, px, pz, CAR_CRUISE, dt);
  if (s->people_struck > 0 || people_down(s) > a->downed_before ||
      a->act_time > 60.0f)
    enter(a, ACT_STEAL_TRUCK);
}

/* Takes a car or a lorry: drive up to one, with one stood in the street ahead
 * if none is near, stop beside it, get out, walk over and get in. */
static void act_steal(autoplay_t* a, driving_stage_t* s, driving_input_t* in,
                      float dt, bool truck, autoplay_act_t next) {
  vec3_t me = player_at(s);
  if (!s->on_foot) {
    if (a->step >= 2) {
      /* In: did we get what we went for? */
      if (!truck || s->vehicle_index >= TRUCK_FIRST_VARIANT) {
        release_target(a, s);
        a->after_steal = ACT_RUN_OVER;
        enter(a, next);
        return;
      }
      a->step = 0;
    }
    if (a->target < 0 || a->target >= (int)traffic_car_count(s->traffic) ||
        (truck ? !is_truck(traffic_car(s->traffic, (size_t)a->target).variant)
               : !is_car(traffic_car(s->traffic, (size_t)a->target).variant))) {
      a->target = nearest_vehicle(s, truck, me, 5000.0f);
      a->brought = false;
    }
    /* Too far off: have one standing a little way up the street. */
    if (a->target >= 0 && !a->brought && a->act_time > 3.0f) {
      traffic_car_t far = traffic_car(s->traffic, (size_t)a->target);
      /* A moving one is hard to catch up with: have it stand in the street. */
      if (distance_to(me, far.position.x, far.position.z) > 60.0f)
        stand_vehicle_ahead(a, s, a->target, 160.0f, false);
      a->brought = true;
    }
    if (a->target < 0) {
      find_town(a, s);
      *in = car_by_road(a, s, a->town_x, a->town_z, CAR_CRUISE, dt);
      return;
    }
    traffic_car_t car = traffic_car(s->traffic, (size_t)a->target);
    float d = distance_to(me, car.position.x, car.position.z);
    if (d < 40.0f) traffic_car_hold(s->traffic, (size_t)a->target, true);
    if (d < 18.0f) {
      /* Beside it: stop, and get out. */
      if (stop_car(s, in)) {
        driving_stage_interact(s);
        if (s->on_foot) a->step = 1;
      }
    } else {
      *in = car_by_road(a, s, car.position.x, car.position.z,
                        d < 70.0f ? 8.0f : CAR_CRUISE, dt);
    }
  } else if (take_standing_car(a, s, truck, in)) {
    return;
  }
  if (a->act_time > 100.0f) {
    release_target(a, s);
    a->after_steal = ACT_RUN_OVER;
    if (truck && s->vehicle_index < TRUCK_FIRST_VARIANT && a->truck_tries < 2) {
      /* No luck: another lorry, from the beginning. */
      a->truck_tries++;
      enter(a, ACT_STEAL_TRUCK);
    } else {
      enter(a, next);
    }
  }
}

static void act_steal_car(autoplay_t* a, driving_stage_t* s,
                          driving_input_t* in, float dt) {
  act_steal(a, s, in, dt, false, a->after_steal);
}

static void act_steal_truck(autoplay_t* a, driving_stage_t* s,
                            driving_input_t* in, float dt) {
  act_steal(a, s, in, dt, true, ACT_TO_AIRPORT);
}

static void act_to_airport(autoplay_t* a, driving_stage_t* s,
                           driving_input_t* in, float dt) {
  vec3_t me = player_at(s);
  if (a->step == 0) {
    driving_stage_lose_the_law(s); /* they have given up by now */
    a->step = 1;
  }
  if (s->on_foot) {
    if (s->entering > 0) return;
    /* No car: take the nearest one. */
    enter(a, ACT_STEAL_CAR);
    return;
  }
  float d = distance_to(me, s->guide.x, s->guide.z);
  if (d > 22.0f && a->step == 1) {
    *in = car_by_road(a, s, s->guide.x, s->guide.z, CAR_CRUISE, dt);
    /* Not getting there: start again from the start, which is by the airport.
     */
    if (a->act_time > 90.0f && d > 150.0f) {
      driving_stage_reset(s);
      a->act_time = 0;
    }
    if (a->act_time > 150.0f) a->step = 2;
    return;
  }
  a->step = 2;
  if (get_out(s, in)) enter(a, a->after_airport);
}

/* The plane to take: the one with the most runway ahead of it. */
static int best_plane(const driving_stage_t* s, airfield_t f) {
  int best = -1;
  float best_along = -1e30f;
  for (size_t i = 0; i < s->aircraft_count; i++) {
    if (s->aircraft[i].kind != AIRCRAFT_PLANE) continue;
    float along = (s->aircraft[i].position.x - f.ox) * f.ax +
                  (s->aircraft[i].position.z - f.oz) * f.az;
    if (along > best_along) best = (int)i, best_along = along;
  }
  return best;
}

static int nearest_helicopter(const driving_stage_t* s, vec3_t from) {
  int best = -1;
  float best_distance = 1e30f;
  for (size_t i = 0; i < s->aircraft_count; i++) {
    if (s->aircraft[i].kind != AIRCRAFT_HELICOPTER) continue;
    float d =
        distance_to(from, s->aircraft[i].position.x, s->aircraft[i].position.z);
    if (d < best_distance) best = (int)i, best_distance = d;
  }
  return best;
}

static void board(autoplay_t* a, driving_stage_t* s, int index,
                  driving_input_t* in, autoplay_act_t next) {
  if (s->piloting > 0) {
    const aircraft_t* craft = &s->aircraft[s->piloting - 1];
    a->ground_y = craft->position.y;
    a->yaw_hold = craft->yaw;
    a->leaving = false;
    enter(a, next);
    return;
  }
  if (index < 0) return;
  const aircraft_t* craft = &s->aircraft[index];
  vec3_t me = player_at(s);
  *in = walk_towards(a, s, craft->position.x, craft->position.z, 1.0f / 60.0f);
  if (distance_to(me, craft->position.x, craft->position.z) < 5.0f)
    driving_stage_interact(s);
}

static void act_take_off(autoplay_t* a, driving_stage_t* s,
                         driving_input_t* in) {
  const aircraft_t* p = &s->aircraft[s->piloting - 1];
  *in = idle_input();
  if (p->on_ground) {
    float error = wrap(a->yaw_hold - p->yaw);
    in->steering = clampf(-error * 4.0f, -1.0f, 1.0f);
    in->handbrake = p->power < 0.98f;
    if (p->speed > 30.0f) in->brake = 0.6f; /* the nose comes up */
    if (a->act_time > 90.0f) enter(a, ACT_CROSS_CITY);
    return;
  }
  /* Away, climbing to cruise. */
  float altitude = a->ground_y + CLIMB_ABOVE_AIRFIELD;
  airfield_t f = airfield(s);
  float tx = p->position.x + f.ax * 800.0f, tz = p->position.z + f.az * 800.0f;
  *in = plane_towards(p, tx, tz, altitude, 55.0f, 0.2f);
  if (p->position.y > a->ground_y + 90.0f) enter(a, ACT_CROSS_CITY);
}

static void act_cross_city(autoplay_t* a, driving_stage_t* s,
                           driving_input_t* in) {
  const aircraft_t* p = &s->aircraft[s->piloting - 1];
  airfield_t f = airfield(s);
  vec3_t middle = city_middle(s);
  /* Over the middle of the city, then on beyond it. */
  float bx = middle.x - f.ox, bz = middle.z - f.oz;
  float bl = fmaxf(hypotf(bx, bz), 1.0f);
  float wx[2] = {middle.x, middle.x + bx / bl * 700.0f};
  float wz[2] = {middle.z, middle.z + bz / bl * 700.0f};
  int w = a->waypoint < 2 ? a->waypoint : 1;
  float altitude = a->ground_y + CLIMB_ABOVE_AIRFIELD;
  *in = plane_towards(p, wx[w], wz[w], altitude, 55.0f, 0.15f);
  if (distance_to(p->position, wx[w], wz[w]) < 260.0f) a->waypoint++;
  if (a->waypoint >= 2) enter(a, ACT_LAND);
}

static void act_land(autoplay_t* a, driving_stage_t* s, driving_input_t* in) {
  const aircraft_t* p = &s->aircraft[s->piloting - 1];
  airfield_t f = airfield(s);
  *in = idle_input();
  /* Along the runway (u) and across it (w) from its middle. */
  float rx = p->position.x - f.ox, rz = p->position.z - f.oz;
  float u = rx * f.ax + rz * f.az;
  float touch_u = -f.length * 0.5f + 70.0f;
  if (p->on_ground && a->step >= 1) {
    /* Rolling out along the runway, brakes on. */
    float error = wrap(a->yaw_hold - p->yaw);
    float w = rx * -f.az + rz * f.ax;
    in->steering = clampf(-(error * 4.0f - w * 0.03f), -1.0f, 1.0f);
    in->lower = true;
    if (p->speed < 2.0f) {
      a->leaving = true;
      driving_stage_interact(s);
      if (s->piloting == 0) enter(a, ACT_STEAL_HELICOPTER);
    }
    return;
  }
  if (a->step == 0) {
    /* Out to the start of the approach, then turn in. */
    float tx = f.ox - f.ax * (f.length * 0.5f + 1500.0f);
    float tz = f.oz - f.az * (f.length * 0.5f + 1500.0f);
    float altitude = a->ground_y + CLIMB_ABOVE_AIRFIELD;
    *in = plane_towards(p, tx, tz, altitude, 52.0f, 0.1f);
    if (distance_to(p->position, tx, tz) < 380.0f ||
        u < -f.length * 0.5f - 1400.0f)
      a->step = 1;
    return;
  }
  /* On the glide: aim at the centre line ahead, down the slope. */
  float ahead = u + 260.0f;
  if (ahead > touch_u + 300.0f) ahead = touch_u + 300.0f;
  float tx = f.ox + f.ax * ahead, tz = f.oz + f.az * ahead;
  float height = height_above_ground(s, p->position);
  float slope_height = fmaxf(0.0f, (touch_u - u) * 0.062f);
  float path =
      clampf(-0.062f + (slope_height - height) * 0.004f, -0.12f, 0.03f);
  if (height < 14.0f) path = -0.035f;
  if (height < 4.0f) path = -0.012f;
  vec3_t h = aircraft_heading(p);
  float angle = angle_to(h, tx - p->position.x, tz - p->position.z);
  in->steering = clampf(angle * 1.5f, -0.6f, 0.6f);
  *in = aircraft_pitch(*in, clampf((path - p->pitch) * 3.0f, -1.0f, 1.0f));
  float want = height < 8.0f
                   ? 0.0f
                   : clampf(0.4f + (46.0f - p->speed) * 0.03f, 0.1f, 0.7f);
  *in = plane_power(*in, p, want);
  if (p->on_ground) {
    a->step = 1;
    a->yaw_hold = p->yaw;
  }
  if (a->act_time > 260.0f) a->step = 1;
}

static void act_helicopter(autoplay_t* a, driving_stage_t* s,
                           driving_input_t* in) {
  const aircraft_t* p = &s->aircraft[s->piloting - 1];
  airfield_t f = airfield(s);
  vec3_t middle = city_middle(s);
  float bx = middle.x - f.ox, bz = middle.z - f.oz;
  float bl = fmaxf(hypotf(bx, bz), 1.0f);
  float wx[2] = {middle.x, middle.x + bx / bl * 700.0f};
  float wz[2] = {middle.z, middle.z + bz / bl * 700.0f};
  float altitude = a->ground_y + 90.0f;
  float height = p->position.y - a->ground_y;
  int w = a->waypoint < 2 ? a->waypoint : 1;
  /* Straight up first, then across. */
  *in = helicopter_towards(p, wx[w], wz[w], altitude, height > 40.0f);
  if (a->waypoint < 2 && distance_to(p->position, wx[w], wz[w]) < 120.0f)
    a->waypoint++;
  if (a->waypoint >= 2) {
    driving_stage_say(s, "THAT IS ALL");
    enter(a, ACT_DONE);
  }
}

driving_input_t autoplay_step(autoplay_t* a, driving_stage_t* s, float dt) {
  driving_input_t in = idle_input();
  if (!a->enabled || s->paused) return in;
  a->frame = dt;
  a->act_time += dt;
  /* A crash or an arrest in the air sends it back to the airport. */
  bool flying_act = a->act == ACT_TAKE_OFF || a->act == ACT_CROSS_CITY ||
                    a->act == ACT_LAND || a->act == ACT_HELICOPTER;
  if (flying_act && s->piloting == 0 && !a->leaving) {
    if (++a->retries > 4) {
      enter(a, ACT_DONE);
    } else {
      a->after_airport =
          a->act == ACT_HELICOPTER ? ACT_STEAL_HELICOPTER : ACT_BOARD_PLANE;
      enter(a, ACT_TO_AIRPORT);
    }
    return in;
  }
  /* A car that has gone nowhere for a good while is stuck in a jam or against
   * a wall: first back out, then start again from the start (R). */
  if (!s->on_foot && s->piloting == 0 && a->act != ACT_START &&
      a->act != ACT_DONE) {
    a->watch_clock += dt;
    if (a->watch_clock > 12.0f) {
      vec3_t at = player_at(s);
      if (hypotf(at.x - a->watch_x, at.z - a->watch_z) < 6.0f &&
          !(a->act == ACT_STEAL_TRUCK && s->entering > 0)) {
        if (++a->watch_stuck >= 2) {
          driving_stage_reset(s);
          a->watch_stuck = 0;
        } else {
          a->reverse_time = 2.5f;
        }
      } else {
        a->watch_stuck = 0;
      }
      a->watch_clock = 0;
      a->watch_x = at.x, a->watch_z = at.z;
    }
  } else {
    a->watch_clock = 0;
    a->watch_stuck = 0;
  }
  /* From the run-over on, the police give up now and then, so a chase does not
   * end the show by boxing the car in. */
  a->law_clock += dt;
  if (a->act >= ACT_STEAL_CAR && s->wanted.stars > 0 &&
      (a->law_clock > 25.0f || a->act_time < 2.0f * dt)) {
    driving_stage_lose_the_law(s);
    a->law_clock = 0;
  }
  /* Nothing is left standing where a scene was staged. */
  if (a->act_time < 2.0f * dt && s->traffic) {
    for (size_t i = 0; i < traffic_car_count(s->traffic); i++)
      traffic_car_hold(s->traffic, i, false);
  }
  /* A dent or two is the price of the show, but not a wreck: the car is
   * patched up between the big moments, as a garage would. */
  if (a->act != ACT_HIT_CAR && !s->on_foot && s->piloting == 0 &&
      s->vehicle.damage > 0.5f)
    s->vehicle.damage = 0.25f;
  switch (a->act) {
    case ACT_START:
      if (a->act_time > 1.0f) enter(a, ACT_DRIVE);
      break;
    case ACT_DRIVE:
      if (s->on_foot) {
        driving_stage_interact(s);
        break;
      }
      /* Off the airport road and into town. */
      find_town(a, s);
      in = car_by_road(a, s, a->town_x, a->town_z, CAR_CRUISE, dt);
      if (a->act_time > 24.0f) enter(a, ACT_HIT_CAR);
      break;
    case ACT_HIT_CAR:
      act_hit_car(a, s, &in, dt);
      break;
    case ACT_STEAL_CAR:
      act_steal_car(a, s, &in, dt);
      break;
    case ACT_RUN_OVER:
      act_run_over(a, s, &in, dt);
      break;
    case ACT_STEAL_TRUCK:
      act_steal_truck(a, s, &in, dt);
      break;
    case ACT_TO_AIRPORT:
      act_to_airport(a, s, &in, dt);
      break;
    case ACT_BOARD_PLANE:
      board(a, s, best_plane(s, airfield(s)), &in, ACT_TAKE_OFF);
      if (a->act == ACT_BOARD_PLANE && a->act_time > 90.0f)
        enter(a, ACT_TO_AIRPORT);
      break;
    case ACT_TAKE_OFF:
      act_take_off(a, s, &in);
      break;
    case ACT_CROSS_CITY:
      act_cross_city(a, s, &in);
      break;
    case ACT_LAND:
      act_land(a, s, &in);
      break;
    case ACT_STEAL_HELICOPTER:
      board(a, s, nearest_helicopter(s, player_at(s)), &in, ACT_HELICOPTER);
      break;
    case ACT_HELICOPTER:
      act_helicopter(a, s, &in);
      break;
    case ACT_DONE:
      break;
  }
  return in;
}
