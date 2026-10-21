#include "traffic.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "route_path.h"

#define ACCELERATION 2.4f
#define BRAKING 6.5f
#define LOOKAHEAD 16.0f

typedef struct {
  int route;
  int direction; /* +1 start to end, -1 end to start */
  float s;       /* metres travelled along the route in its direction */
  float speed;
  int variant;
  int paint;
  float wheel_turn;
  bool braking;
  bool responding; /* an emergency vehicle on a call */
  vec3_t position;
  float yaw;
  collider_handle_t collider;
} car_t;

typedef struct {
  route_path_t path;
  const city_route_t* route;
  float length;
  float cruise; /* metres per second */
} lane_t;

struct traffic {
  lane_t* lanes;
  size_t lane_count;
  car_t* cars;
  size_t car_count;
  const fleet_t* fleet;
  uint32_t random;
  float ground_clock;
  const city_signal_t* signals;
  size_t signal_count;
  float signal_time;
  const vec3_t* walkers;
  size_t walker_count;
};

static uint32_t next_random(traffic_t* t) {
  t->random = t->random * 1664525u + 1013904223u;
  return t->random >> 8;
}

/* The point and heading at distance d along a lane, in the direction of
 * travel, before the lane offset. */
static void sample(const lane_t* lane, float d, int direction, float* x,
                   float* z, float* dx, float* dz, float* ground) {
  route_path_sample(&lane->path, d, direction, x, z, dx, dz, ground);
}

static float lane_offset(const lane_t* lane) {
  float offset = lane->route->width * 0.25f;
  return offset > 1.9f ? 1.9f : offset;
}

static void pose(const traffic_t* t, car_t* car) {
  const lane_t* lane = &t->lanes[car->route];
  float x, z, dx, dz, ground;
  sample(lane, car->s, car->direction, &x, &z, &dx, &dz, &ground);
  /* Right of the direction of travel: (-dz, dx) in our x, z frame with z
   * south: right of east is south (+z), so right = (-dz, dx). */
  float offset = lane_offset(lane);
  car->position.x = x - dz * offset;
  car->position.z = z + dx * offset;
  car->yaw = atan2f(-dx, -dz);
  car->position.y = ground + t->fleet->visuals[car->variant].ride_height;
}

static void move_collider(traffic_t* t, physics_world_t* physics, car_t* car) {
  const car_visual_t* visual = &t->fleet->visuals[car->variant];
  physics_set_static_box_transform(
      physics, car->collider,
      transform_at(car->position, quat_from_yaw(car->yaw)));
  (void)visual;
}

traffic_t* traffic_create(const city_route_t* routes, size_t route_count,
                          physics_world_t* physics, const fleet_t* fleet,
                          int car_count, uint32_t seed) {
  traffic_t* t = calloc(1, sizeof(traffic_t));
  if (!t) return NULL;
  t->fleet = fleet;
  t->random = seed * 2654435761u + 12345u;
  t->lane_count = route_count;
  t->lanes = calloc(route_count ? route_count : 1, sizeof(lane_t));
  if (!t->lanes) {
    free(t);
    return NULL;
  }
  float drivable_length = 0;
  for (size_t r = 0; r < route_count; r++) {
    lane_t* lane = &t->lanes[r];
    lane->route = &routes[r];
    if (!route_path_init(&lane->path, &routes[r], physics)) {
      traffic_destroy(t, physics);
      return NULL;
    }
    lane->length = lane->path.length;
    lane->cruise = routes[r].width > 7.5f ? 11.0f : 7.5f;
    if (routes[r].drivable && lane->length > 12.0f)
      drivable_length += lane->length;
  }
  if (drivable_length <= 0 || car_count <= 0) {
    if (car_count > 0) {
      traffic_destroy(t, physics);
      return NULL;
    }
  }
  t->cars = calloc(car_count > 0 ? (size_t)car_count : 1, sizeof(car_t));
  if (!t->cars) {
    traffic_destroy(t, physics);
    return NULL;
  }
  for (int i = 0; i < car_count; i++) {
    car_t* car = &t->cars[t->car_count];
    /* A random drivable place, weighted by the road's length. */
    float pick = (next_random(t) % 10000) / 10000.0f * drivable_length;
    size_t chosen = 0;
    for (size_t r = 0; r < route_count; r++) {
      if (!routes[r].drivable || t->lanes[r].length <= 12.0f) continue;
      chosen = r;
      if (pick < t->lanes[r].length) break;
      pick -= t->lanes[r].length;
    }
    car->route = (int)chosen;
    car->direction = routes[chosen].oneway || (next_random(t) & 1) ? 1 : -1;
    car->s = 6.0f +
             pick * (t->lanes[chosen].length - 12.0f) / t->lanes[chosen].length;
    car->speed = 0;
    int style = (int)(next_random(t) % 1000);
    car->variant = fleet_traffic_variant_for(style);
    car->paint = (int)(next_random(t) % 1000);
    car->responding =
        fleet->visuals[car->variant].emergency && next_random(t) % 100 < 60;
    pose(t, car);
    car->collider = physics_add_static_box(
        physics, transform_at(car->position, quat_from_yaw(car->yaw)),
        fleet->visuals[car->variant].half_extents, COLLISION_LAYER_PROP, 0.5f,
        0);
    t->car_count++;
  }
  return t;
}

void traffic_destroy(traffic_t* t, physics_world_t* physics) {
  if (!t) return;
  for (size_t i = 0; i < t->car_count; i++)
    physics_remove_collider(physics, t->cars[i].collider);
  for (size_t r = 0; r < t->lane_count; r++) route_path_free(&t->lanes[r].path);
  free(t->lanes);
  free(t->cars);
  free(t);
}

void traffic_set_signals(traffic_t* t, const city_signal_t* signals,
                         size_t count) {
  t->signals = signals;
  t->signal_count = count;
}

void traffic_set_walkers(traffic_t* t, const vec3_t* positions, size_t count) {
  t->walkers = positions;
  t->walker_count = count;
}

void traffic_set_signal_time(traffic_t* t, float seconds) {
  t->signal_time = seconds;
}

bool traffic_signal_open(float seconds, int group) {
  float phase = fmodf(seconds + 24.0f * (float)(group & 1), 48.0f);
  return phase >= 24.0f && phase < 45.0f;
}

bool traffic_signals_open(float seconds) {
  return traffic_signal_open(seconds, 0);
}

size_t traffic_car_count(const traffic_t* t) { return t->car_count; }

traffic_car_t traffic_car(const traffic_t* t, size_t index) {
  const car_t* car = &t->cars[index];
  traffic_car_t out = {car->position, car->yaw,       car->speed,
                       car->route,    car->variant,   car->paint,
                       car->braking,  car->wheel_turn};
  return out;
}

void traffic_car_place(traffic_t* t, size_t index, int route, int direction,
                       float distance) {
  car_t* car = &t->cars[index];
  car->route = route;
  car->direction = direction;
  car->s = distance;
  car->speed = 0;
  pose(t, car);
}

/* After a lane's end: choose where to go. Sets route, direction and s. */
static void choose_next(traffic_t* t, car_t* car) {
  const lane_t* here = &t->lanes[car->route];
  int64_t node =
      car->direction > 0 ? here->route->end_node : here->route->start_node;
  int candidates[16][2];
  int count = 0;
  for (size_t r = 0; r < t->lane_count && count < 16; r++) {
    const city_route_t* route = t->lanes[r].route;
    if (!route->drivable || (int)r == car->route || t->lanes[r].length < 6.0f)
      continue;
    if (route->start_node == node) {
      candidates[count][0] = (int)r, candidates[count][1] = 1, count++;
    } else if (route->end_node == node && !route->oneway && count < 16) {
      candidates[count][0] = (int)r, candidates[count][1] = -1, count++;
    }
  }
  if (count == 0) {
    /* A dead end: turn round if the road allows it, else go back to the
     * start of the one-way. */
    if (!here->route->oneway) {
      car->direction = -car->direction;
    } else {
      car->s = 0;
      return;
    }
    car->s = 0;
    car->speed *= 0.3f;
    return;
  }
  int pick = (int)(next_random(t) % (uint32_t)count);
  car->route = candidates[pick][0];
  car->direction = candidates[pick][1];
  car->s = 0;
}

static float curve_speed(const lane_t* lane, float s, int direction) {
  /* Slow for the turn ahead by comparing headings now and 14 m on. */
  float x, z, dx, dz, g, x2, z2, dx2, dz2;
  sample(lane, s, direction, &x, &z, &dx, &dz, &g);
  sample(lane, fminf(s + 14.0f, lane->length), direction, &x2, &z2, &dx2, &dz2,
         &g);
  float cosine = dx * dx2 + dz * dz2;
  return cosine > 0.97f ? 99.0f : (cosine > 0.85f ? 6.0f : 3.5f);
}

void traffic_step(traffic_t* t, physics_world_t* physics, vec3_t player,
                  vec3_t player_velocity, float dt) {
  (void)player_velocity;
  t->ground_clock += dt;
  for (size_t i = 0; i < t->car_count; i++) {
    car_t* car = &t->cars[i];
    const lane_t* lane = &t->lanes[car->route];
    const car_visual_t* own_visual = &t->fleet->visuals[car->variant];
    float own_half_length = own_visual->half_extents.z;
    float own_half_width = own_visual->half_extents.x;
    /* Lorries and buses are slower; the emergency services are not. */
    float target =
        lane->cruise * (own_visual->emergency
                            ? 1.2f
                            : (own_half_length > 3.0f ? 0.85f : 1.0f));
    float to_end = lane->length - car->s;
    if (to_end < 14.0f) target = fminf(target, 4.5f + to_end * 0.2f);
    target = fminf(target, curve_speed(lane, car->s, car->direction));
    /* The way ahead: a point a few metres in front of the nose. */
    float fx = -sinf(car->yaw), fz = -cosf(car->yaw);
    float limit = target;
    bool must_stop = false;
    float gap_to_stop = 1e9f;
    for (size_t j = 0; j <= t->car_count; j++) {
      vec3_t other;
      float other_speed = 0;
      float other_half_length = 2.2f, other_half_width = 0.9f;
      if (j == t->car_count) {
        other = player;
      } else if (j == i) {
        continue;
      } else {
        other = t->cars[j].position;
        other_speed = t->cars[j].speed;
        const car_visual_t* other_visual =
            &t->fleet->visuals[t->cars[j].variant];
        other_half_length = other_visual->half_extents.z;
        other_half_width = other_visual->half_extents.x;
      }
      float rx = other.x - car->position.x, rz = other.z - car->position.z;
      float ahead = rx * fx + rz * fz;
      float side = fabsf(rx * -fz + rz * fx);
      float side_limit = fmaxf(2.1f, own_half_width + other_half_width + 0.3f);
      if (ahead < 1.0f ||
          ahead > LOOKAHEAD + own_half_length + other_half_length ||
          side > side_limit)
        continue;
      /* Of two cars facing each other, only one yields, so they cannot
       * deadlock; the player always has right of way. */
      if (j < t->car_count && j < i) {
        float ofx = -sinf(t->cars[j].yaw), ofz = -cosf(t->cars[j].yaw);
        if (ofx * fx + ofz * fz < -0.2f && t->cars[j].speed > 0.5f) continue;
        if (fabsf(ofx * fx + ofz * fz) < 0.5f && ahead > 7.0f) continue;
      }
      float gap = ahead - (own_half_length + other_half_length + 0.6f);
      if (gap < gap_to_stop) gap_to_stop = gap;
      /* Follow at the leader's pace when there is room. */
      if (gap > 4.0f)
        limit = fminf(limit, fmaxf(other_speed, 1.0f) + gap * 0.25f);
      else
        must_stop = true;
      if (gap < 0.5f) must_stop = true;
    }
    /* Someone crossing in front: stop short of them. */
    for (size_t k = 0; k < t->walker_count; k++) {
      float rx = t->walkers[k].x - car->position.x;
      float rz = t->walkers[k].z - car->position.z;
      float ahead = rx * fx + rz * fz;
      float side = fabsf(rx * -fz + rz * fx);
      if (ahead < 1.0f || ahead > 18.0f || side > 2.8f) continue;
      float room = ahead - (own_half_length + 1.2f);
      if (room < 0.5f)
        must_stop = true;
      else
        limit = fminf(limit, sqrtf(2.0f * BRAKING * 0.6f * room));
    }
    /* Junctions: wait outside while another car is in them, or is closer
     * to them and also coming in (the lower index goes first). */
    if (to_end < 22.0f + own_half_length) {
      float px, pz, pdx, pdz, pg;
      sample(lane, lane->length, car->direction, &px, &pz, &pdx, &pdz, &pg);
      bool blocked = false;
      for (size_t j = 0; j < t->car_count && !blocked; j++) {
        if (j == i) continue;
        const car_t* other = &t->cars[j];
        float ox = other->position.x - px, oz = other->position.z - pz;
        float inside = sqrtf(ox * ox + oz * oz);
        if (inside < 7.0f) {
          /* Cars queued behind me on my own lane are not in the junction. */
          blocked = true;
        } else if (inside < 18.0f && j < i) {
          float ofx = -sinf(other->yaw), ofz = -cosf(other->yaw);
          bool coming = (-ox * ofx - oz * ofz) > 0 && other->speed > 0.3f;
          if (coming && inside < to_end + 2.0f) blocked = true;
        }
      }
      if (blocked) {
        float room = to_end - (5.4f + own_half_length);
        limit =
            fminf(limit, room > 0 ? sqrtf(2.0f * BRAKING * 0.6f * room) : 0.0f);
      }
    }
    /* A red signal ahead for the way this car is heading: stop six metres
     * short of it. */
    for (size_t k = 0; k < t->signal_count; k++) {
      const city_signal_t* signal = &t->signals[k];
      if (signal->group < 0 ||
          traffic_signal_open(t->signal_time, signal->group))
        continue; /* green, or knocked down */
      bool any_heading = signal->heading_x == 0 && signal->heading_z == 0;
      if (!any_heading &&
          signal->heading_x * fx + signal->heading_z * fz < 0.7f)
        continue;
      float rx = signal->x - car->position.x;
      float rz = signal->z - car->position.z;
      float ahead = rx * fx + rz * fz;
      float side = fabsf(rx * -fz + rz * fx);
      if (ahead < 1.0f || ahead > 30.0f || side > 8.0f) continue;
      float room = ahead - 6.0f;
      limit =
          fminf(limit, room > 0 ? sqrtf(2.0f * BRAKING * 0.6f * room) : 0.0f);
    }
    if (must_stop) limit = 0;
    float before = car->speed;
    if (car->speed < limit) {
      car->speed = fminf(limit, car->speed + ACCELERATION * dt);
    } else {
      float slowdown = BRAKING * dt;
      car->speed = fmaxf(limit, car->speed - slowdown);
    }
    car->braking = car->speed < before - 0.002f || must_stop;
    car->s += car->speed * dt;
    car->wheel_turn +=
        car->speed * dt / t->fleet->visuals[car->variant].wheel_radius;
    if (car->s >= lane->length) {
      float overshoot = car->s - lane->length;
      choose_next(t, car);
      car->s += overshoot;
    }
    pose(t, car);
    move_collider(t, physics, car);
  }
}

float traffic_nearest_siren(const traffic_t* t, vec3_t point) {
  float nearest = 1e9f;
  for (size_t i = 0; i < t->car_count; i++) {
    if (!t->cars[i].responding) continue;
    float dx = t->cars[i].position.x - point.x,
          dz = t->cars[i].position.z - point.z;
    float d = sqrtf(dx * dx + dz * dz);
    if (d < nearest) nearest = d;
  }
  return nearest;
}

bool traffic_car_responding(const traffic_t* t, size_t index) {
  return t->cars[index].responding;
}

float traffic_nearest_distance(const traffic_t* t, vec3_t point) {
  float nearest = 1e9f;
  for (size_t i = 0; i < t->car_count; i++) {
    float dx = t->cars[i].position.x - point.x,
          dz = t->cars[i].position.z - point.z;
    float d = sqrtf(dx * dx + dz * dz);
    if (d < nearest) nearest = d;
  }
  return nearest;
}

int traffic_draw(const traffic_t* t, const fleet_t* fleet, renderer_t* renderer,
                 vec3_t eye, float range, float lights_on) {
  int drawn = 0;
  for (size_t i = 0; i < t->car_count; i++) {
    const car_t* car = &t->cars[i];
    float dx = car->position.x - eye.x, dz = car->position.z - eye.z;
    if (dx * dx + dz * dz > range * range) continue;
    car_visual_draw_still(
        &fleet->visuals[car->variant], renderer,
        transform_at(car->position, quat_from_yaw(car->yaw)),
        fleet_paint(car->paint), lights_on, car->braking ? 1.0f : 0.0f,
        car->wheel_turn,
        car->responding ? t->ground_clock + 0.37f * (float)i : -1.0f);
    drawn++;
  }
  return drawn;
}
