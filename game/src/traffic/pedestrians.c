#include "pedestrians.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "route_path.h"

#define LYING_SECONDS 25.0f

typedef struct {
  int route;
  int side;      /* +1 right of the route's direction, -1 left */
  int direction; /* +1 walking start to end, -1 back */
  float u;       /* metres from the route's start */
  float speed;
  float cruise;
  float phase;
  int outfit;
  vec3_t position;
  float yaw;
  float cross;    /* 0 on the pavement, else progress across the road, 0..1 */
  float cooldown; /* seconds before they will cross again */
  int decided;    /* the crossing they last made up their mind about */
  float heading_x, heading_z; /* of the route where they stand */
  float panic;                /* seconds left of running from trouble */
  float down;             /* 0 on their feet, else seconds since a car hit */
  float slide_x, slide_z; /* metres a second, thrown by the car */
  float lift, rise;       /* height off the ground, and its speed, in the air */
  float flight;           /* seconds the throw lasts, 0 for a plain fall */
  float turns;            /* quarter turns the body makes before it lies */
  float stagger;          /* seconds of reeling from a punch */
} person_t;

struct pedestrians {
  route_path_t* paths;
  float* margin_start; /* walkers stop short of junctions, where the road is */
  float* margin_end;
  size_t path_count;
  person_t* people;
  size_t count;
  uint32_t random;
  const float* crossings;
  size_t crossing_count;
  vec3_t focus;
  bool has_focus;
};

static unsigned outfit_hash(int outfit) {
  return ((unsigned)outfit * 2654435761u) >> 12;
}

person_kind_t pedestrian_kind(int outfit) {
  unsigned k = outfit_hash(outfit) % 20;
  if (k < 2) return PERSON_CHILD;
  if (k == 2) return PERSON_ELDER;
  if (k == 3) return PERSON_JOGGER;
  return PERSON_ADULT;
}

float pedestrian_scale(int outfit) {
  unsigned h = outfit_hash(outfit);
  switch (pedestrian_kind(outfit)) {
    case PERSON_CHILD:
      return 0.6f + 0.02f * (float)(h / 20 % 10);
    case PERSON_ELDER:
      return 0.94f;
    default:
      return 0.92f + 0.017f * (float)(h / 20 % 10);
  }
}

static uint32_t next_random(pedestrians_t* p) {
  p->random = p->random * 1664525u + 1013904223u;
  return p->random >> 8;
}

static bool walkable(const city_route_t* route) {
  /* Streets people use, and roads with a pavement to walk on. */
  return route->point_count >= 2 &&
         (!route->drivable || route->pavement > 0.4f);
}

static float lateral_offset(const city_route_t* route, int side) {
  if (route->drivable)
    return side * (route->width * 0.5f + route->pavement * 0.5f);
  float narrow = route->width * 0.25f;
  return side * (narrow < 0.7f ? narrow : 0.7f);
}

static void pose(const pedestrians_t* p, person_t* person) {
  const route_path_t* path = &p->paths[person->route];
  float x, z, dx, dz, ground;
  route_path_sample(path, person->u, 1, &x, &z, &dx, &dz, &ground);
  float offset = lateral_offset(path->route, person->side);
  person->heading_x = dx, person->heading_z = dz;
  if (person->cross > 0) {
    /* Part way over: slide from one pavement to the other, face across. */
    float t = person->cross;
    offset *= 1.0f - 2.0f * t;
    person->position = vec3(x - dz * offset, ground, z + dx * offset);
    float across = -(float)person->side; /* towards the far side */
    person->yaw = atan2f(-across * -dz, -across * dx);
    return;
  }
  /* Right of the route's direction (x east, z south): (-dz, dx). */
  person->position = vec3(x - dz * offset, ground, z + dx * offset);
  person->yaw = person->direction > 0 ? atan2f(-dx, -dz) : atan2f(dx, dz);
}

void pedestrians_set_focus(pedestrians_t* p, vec3_t focus) {
  p->focus = focus;
  p->has_focus = true;
}

/* Moves someone left far behind to a walkable street near the focus. */
static void recycle_person(pedestrians_t* p, person_t* person) {
  for (int attempt = 0; attempt < 48; attempt++) {
    size_t r = next_random(p) % p->path_count;
    const route_path_t* path = &p->paths[r];
    if (!walkable(path->route) || path->length <= 12.0f) continue;
    float first = p->margin_start[r] + 1.0f;
    float last = path->length - p->margin_end[r] - 1.0f;
    if (last < first) continue;
    float u = first + (last - first) * (float)(next_random(p) % 1000) / 1000.0f;
    /* Beyond the distance people are drawn at, so nobody sees one appear. */
    float x, z, dx, dz, ground;
    route_path_sample(path, u, 1, &x, &z, &dx, &dz, &ground);
    float d = hypotf(x - p->focus.x, z - p->focus.z);
    if (d < PEDESTRIAN_DRAW_RANGE + 40.0f || d > 500.0f) continue;
    person->route = (int)r;
    person->side = (next_random(p) & 1) ? 1 : -1;
    person->direction = (next_random(p) & 1) ? 1 : -1;
    person->u = u;
    person->cross = 0;
    person->cooldown = 0;
    person->decided = -1;
    return;
  }
}

void pedestrians_set_crossings(pedestrians_t* p, const float* xz,
                               size_t count) {
  p->crossings = xz;
  p->crossing_count = count;
}

size_t pedestrians_crossing(const pedestrians_t* p, vec3_t* out, size_t max) {
  size_t n = 0;
  for (size_t i = 0; i < p->count && n < max; i++)
    if (p->people[i].cross > 0 && p->people[i].down <= 0)
      out[n++] = p->people[i].position;
  return n;
}

pedestrians_t* pedestrians_create(const city_route_t* routes,
                                  size_t route_count, physics_world_t* physics,
                                  int count, uint32_t seed) {
  pedestrians_t* p = calloc(1, sizeof(pedestrians_t));
  if (!p) return NULL;
  p->random = seed * 2654435761u + 99u;
  p->paths = calloc(route_count ? route_count : 1, sizeof(route_path_t));
  p->people = calloc(count > 0 ? (size_t)count : 1, sizeof(person_t));
  p->margin_start = calloc(route_count ? route_count : 1, sizeof(float));
  p->margin_end = calloc(route_count ? route_count : 1, sizeof(float));
  if (!p->paths || !p->people || !p->margin_start || !p->margin_end) {
    pedestrians_destroy(p);
    return NULL;
  }
  p->path_count = route_count;
  for (size_t r = 0; r < route_count; r++) {
    int at_start = 0, at_end = 0;
    for (size_t q = 0; q < route_count; q++) {
      if (q == r) continue;
      if (routes[q].start_node == routes[r].start_node ||
          routes[q].end_node == routes[r].start_node)
        at_start++;
      if (routes[q].start_node == routes[r].end_node ||
          routes[q].end_node == routes[r].end_node)
        at_end++;
    }
    p->margin_start[r] = at_start > 0 ? 4.6f : 0.0f;
    p->margin_end[r] = at_end > 0 ? 4.6f : 0.0f;
  }
  float total = 0;
  for (size_t r = 0; r < route_count; r++) {
    if (!route_path_init(&p->paths[r], &routes[r], physics)) {
      pedestrians_destroy(p);
      return NULL;
    }
    if (walkable(&routes[r]) && p->paths[r].length > 8.0f)
      total += p->paths[r].length;
  }
  if (total <= 0) {
    pedestrians_destroy(p);
    return NULL;
  }
  for (int i = 0; i < count; i++) {
    float pick = (next_random(p) % 10000) / 10000.0f * total;
    size_t chosen = 0;
    for (size_t r = 0; r < route_count; r++) {
      if (!walkable(&routes[r]) || p->paths[r].length <= 8.0f) continue;
      chosen = r;
      if (pick < p->paths[r].length) break;
      pick -= p->paths[r].length;
    }
    person_t* person = &p->people[p->count++];
    person->route = (int)chosen;
    person->side = (next_random(p) & 1) ? 1 : -1;
    person->direction = (next_random(p) & 1) ? 1 : -1;
    /* Between the junction margins, where the pavement is. */
    float first = p->margin_start[chosen] + 1.0f;
    float last = p->paths[chosen].length - p->margin_end[chosen] - 1.0f;
    if (last < first) last = first;
    person->u = first + (last - first) * pick / p->paths[chosen].length;
    person->outfit = (int)(next_random(p) % 1000);
    float spread = (float)(next_random(p) % 50) / 100.0f;
    switch (pedestrian_kind(person->outfit)) {
      case PERSON_CHILD:
        person->cruise = 1.1f + spread * 0.6f;
        break;
      case PERSON_ELDER:
        person->cruise = 0.8f + spread * 0.3f;
        break;
      case PERSON_JOGGER:
        person->cruise = 2.4f + spread * 1.4f;
        break;
      default:
        person->cruise = 1.0f + spread;
        break;
    }
    /* Every fifth person walks with the one before, a step behind. */
    if (i % 5 == 4 && i > 0 &&
        pedestrian_kind(p->people[i - 1].outfit) != PERSON_JOGGER) {
      const person_t* mate = &p->people[i - 1];
      person->route = mate->route;
      person->side = mate->side;
      person->direction = mate->direction;
      person->u = mate->u - mate->direction * 0.75f;
      person->cruise = mate->cruise;
      if (pedestrian_kind(person->outfit) == PERSON_JOGGER)
        person->outfit = mate->outfit + 1;
    }
    person->speed = person->cruise;
    person->decided = -1;
    pose(p, person);
  }
  return p;
}

void pedestrians_destroy(pedestrians_t* p) {
  if (!p) return;
  for (size_t r = 0; r < p->path_count; r++) route_path_free(&p->paths[r]);
  free(p->paths);
  free(p->margin_start);
  free(p->margin_end);
  free(p->people);
  free(p);
}

#define GRAVITY 9.8f

/* Throws a hit body into the air: it turns over backwards `flips` whole times
 * on the way and lands lying down. */
static void launch(person_t* person, float rise, int flips) {
  person->lift = 0.001f;
  person->rise = rise;
  person->flight = 2.0f * rise / GRAVITY;
  person->turns = 1.0f + 4.0f * (float)flips;
}

/* How far over the body is, in quarter turns: 1 is lying flat. */
static float body_fall(const person_t* person) {
  /* Reeling from a punch: a lean back, not a fall. */
  if (person->down <= 0)
    return person->stagger > 0 ? 0.3f * fminf(1.0f, person->stagger / 0.3f)
                               : 0.0f;
  if (person->flight <= 0) return fminf(1.0f, person->down * 4.0f);
  if (person->down >= person->flight) return 1.0f;
  return person->turns * person->down / person->flight;
}

/* Arms and legs thrash while the body is in the air and for a moment after
 * it lands, then it lies still. */
static float body_flail(const person_t* person) {
  if (person->down <= 0) return person->stagger > 0 ? 0.8f : 0.0f;
  float landed = person->down - person->flight;
  if (landed <= 0) return 1.0f;
  return fmaxf(0.0f, 1.0f - landed / 2.5f);
}

size_t pedestrians_count(const pedestrians_t* p) { return p->count; }

pedestrian_t pedestrians_get(const pedestrians_t* p, size_t index) {
  const person_t* person = &p->people[index];
  pedestrian_t out = {person->position,  person->yaw,    person->speed,
                      person->phase,     person->outfit, person->route,
                      body_fall(person), person->lift,   body_flail(person)};
  return out;
}

void pedestrians_place(pedestrians_t* p, size_t index, int route, int side,
                       int direction, float distance) {
  person_t* person = &p->people[index];
  person->route = route;
  person->side = side;
  person->direction = direction;
  person->u = distance;
  pose(p, person);
}

static void reach_the_end(pedestrians_t* p, person_t* person) {
  const route_path_t* here = &p->paths[person->route];
  int64_t node =
      person->direction > 0 ? here->route->end_node : here->route->start_node;
  int candidates[16][2];
  int count = 0;
  for (size_t r = 0; r < p->path_count && count < 16; r++) {
    if ((int)r == person->route || !walkable(p->paths[r].route) ||
        p->paths[r].length < 4.0f)
      continue;
    if (p->paths[r].route->start_node == node) {
      candidates[count][0] = (int)r, candidates[count][1] = 1, count++;
    } else if (p->paths[r].route->end_node == node && count < 16) {
      candidates[count][0] = (int)r, candidates[count][1] = -1, count++;
    }
  }
  if (count == 0) {
    person->direction = -person->direction;
    return;
  }
  int pick = (int)(next_random(p) % (uint32_t)count);
  person->route = candidates[pick][0];
  person->direction = candidates[pick][1];
  person->u = person->direction > 0 ? p->margin_start[person->route]
                                    : p->paths[person->route].length -
                                          p->margin_end[person->route];
  if (!p->paths[person->route].route->drivable)
    person->side = (next_random(p) & 1) ? 1 : -1;
}

/* A person walking a pavement past a zebra crossing may cross it, if no
 * car of the player's is near and they have not just done so. They make up
 * their mind once per crossing. */
static void maybe_start_crossing(pedestrians_t* p, person_t* person,
                                 vec3_t player) {
  const route_path_t* path = &p->paths[person->route];
  if (!path->route->drivable || path->route->pavement < 0.4f ||
      person->cooldown > 0 || person->speed < 0.3f)
    return;
  float offset = fabsf(lateral_offset(path->route, person->side));
  float right_x = -person->heading_z, right_z = person->heading_x;
  bool near_any = false;
  for (size_t k = 0; k < p->crossing_count; k++) {
    float rx = p->crossings[2 * k] - person->position.x;
    float rz = p->crossings[2 * k + 1] - person->position.z;
    float along = fabsf(rx * person->heading_x + rz * person->heading_z);
    float lateral = fabsf(rx * right_x + rz * right_z);
    if (along > 9.0f || lateral > offset + 3.0f) continue;
    near_any = true;
    if (along > 1.6f || fabsf(lateral - offset) > 1.6f) continue;
    if (person->decided == (int)k) return;
    person->decided = (int)k;
    uint32_t dice = next_random(p) % 100;
    float dx = player.x - person->position.x,
          dz = player.z - person->position.z;
    if (dice < 45 && dx * dx + dz * dz > 15.0f * 15.0f) person->cross = 0.001f;
    return;
  }
  if (!near_any) person->decided = -1;
}

int pedestrians_hit(pedestrians_t* p, vec3_t centre, float yaw,
                    vec3_t half_extents, vec3_t velocity) {
  float speed = hypotf(velocity.x, velocity.z);
  if (speed < 2.5f) return 0;
  float sine = sinf(yaw), cosine = cosf(yaw);
  int hit = 0;
  for (size_t i = 0; i < p->count; i++) {
    person_t* person = &p->people[i];
    if (person->down > 0) continue;
    float dx = person->position.x - centre.x,
          dz = person->position.z - centre.z;
    if (dx * dx + dz * dz > 36.0f) continue;
    /* Into the car's frame: x across, z along (the nose is -z). */
    float across = dx * cosine - dz * sine;
    float along = dx * sine + dz * cosine;
    if (fabsf(across) > half_extents.x + 0.3f ||
        fabsf(along) > half_extents.z + 0.3f)
      continue;
    if (fabsf(person->position.y - centre.y) > 2.5f) continue;
    person->down = 0.001f;
    person->cross = 0;
    /* Thrown ahead of the car, a little faster than it goes. */
    float throw_speed = fminf(speed * 0.9f, 7.0f);
    person->slide_x = velocity.x / speed * throw_speed;
    person->slide_z = velocity.z / speed * throw_speed;
    person->yaw = atan2f(-velocity.x, -velocity.z);
    launch(person, fminf(3.0f + 0.35f * speed, 6.5f), speed > 9.0f ? 2 : 1);
    hit++;
  }
  return hit;
}

int pedestrians_punch(pedestrians_t* p, vec3_t from, vec3_t direction,
                      float reach) {
  person_t* best = NULL;
  float best_distance = reach;
  for (size_t i = 0; i < p->count; i++) {
    person_t* person = &p->people[i];
    if (person->down > 0 || person->stagger > 0) continue;
    float dx = person->position.x - from.x, dz = person->position.z - from.z;
    float d = hypotf(dx, dz);
    if (d > best_distance || fabsf(person->position.y - from.y) > 1.5f)
      continue;
    /* Within sixty degrees of where the fist goes. */
    if (d > 0.05f && (dx * direction.x + dz * direction.z) / d < 0.5f) continue;
    best = person, best_distance = d;
  }
  if (!best) return 0;
  best->stagger = 0.45f;
  best->panic = 9.0f;
  best->cross = 0;
  best->speed = 0;
  /* Away from the puncher, along whichever way the pavement runs. */
  float ax = best->position.x - from.x, az = best->position.z - from.z;
  float along = best->heading_x * ax + best->heading_z * az;
  if ((along < 0) == (best->direction > 0)) best->direction = -best->direction;
  pose(p, best);
  /* The rest of the street takes fright. */
  for (size_t i = 0; i < p->count; i++) {
    person_t* person = &p->people[i];
    float dx = person->position.x - from.x, dz = person->position.z - from.z;
    if (person != best && person->down <= 0 && dx * dx + dz * dz < 9.0f * 9.0f)
      person->panic = fmaxf(person->panic, 6.0f);
  }
  return 1;
}

void pedestrians_alarm(pedestrians_t* p, vec3_t at, float radius) {
  for (size_t i = 0; i < p->count; i++) {
    person_t* person = &p->people[i];
    float dx = person->position.x - at.x, dz = person->position.z - at.z;
    if (person->down <= 0 && dx * dx + dz * dz < radius * radius)
      person->panic = 7.0f;
  }
}

bool pedestrians_shoot(pedestrians_t* p, vec3_t origin, vec3_t direction,
                       float range, float* hit_distance) {
  person_t* best = NULL;
  float best_t = range;
  for (size_t i = 0; i < p->count; i++) {
    person_t* person = &p->people[i];
    if (person->down > 0) continue;
    float s = pedestrian_scale(person->outfit);
    /* The body is a capsule from the feet to the head; test its centre. */
    vec3_t centre = vec3(person->position.x, person->position.y + 0.9f * s,
                         person->position.z);
    vec3_t to = vec3_sub(centre, origin);
    float t = vec3_dot(to, direction);
    if (t <= 0 || t >= best_t) continue;
    vec3_t closest = vec3_add(origin, vec3_scale(direction, t));
    float dx = closest.x - centre.x, dz = closest.z - centre.z;
    float dy = fabsf(closest.y - centre.y);
    if (dx * dx + dz * dz > 0.3f * 0.3f || dy > 0.85f * s) continue;
    best = person, best_t = t;
  }
  if (!best) return false;
  best->down = 0.001f;
  best->cross = 0;
  best->slide_x = direction.x * 3.0f;
  best->slide_z = direction.z * 3.0f;
  best->yaw = atan2f(-direction.x, -direction.z);
  launch(best, 2.2f, 0);
  *hit_distance = best_t;
  return true;
}

void pedestrians_step(pedestrians_t* p, vec3_t player, float dt) {
  for (size_t i = 0; i < p->count; i++) {
    person_t* person = &p->people[i];
    if (p->has_focus) {
      float fx0 = person->position.x - p->focus.x;
      float fz0 = person->position.z - p->focus.z;
      if (fx0 * fx0 + fz0 * fz0 > 700.0f * 700.0f) {
        recycle_person(p, person);
        pose(p, person);
      }
    }
    float dx = player.x - person->position.x,
          dz = player.z - person->position.z;
    float near_car = dx * dx + dz * dz;
    if (person->down > 0) {
      /* Thrown by the car, skidding to a stop, then lying there a while. */
      person->down += dt;
      if (person->lift > 0) {
        /* In the air: no friction, just the arc. */
        person->rise -= GRAVITY * dt;
        person->lift += person->rise * dt;
        if (person->lift <= 0) person->lift = person->rise = 0;
      }
      float friction = person->lift > 0 ? 1.0f : expf(-dt * 3.5f);
      person->slide_x *= friction, person->slide_z *= friction;
      person->position.x += person->slide_x * dt;
      person->position.z += person->slide_z * dt;
      person->speed = 0;
      person->phase += 22.0f * dt * body_flail(person);
      if (person->down > LYING_SECONDS &&
          near_car > PEDESTRIAN_DRAW_RANGE * PEDESTRIAN_DRAW_RANGE) {
        person->down = 0;
        person->lift = person->rise = person->flight = 0;
        person->slide_x = person->slide_z = 0;
        recycle_person(p, person);
        pose(p, person);
      }
      continue;
    }
    if (person->stagger > 0) {
      /* Reeling: nothing else until it passes. */
      person->stagger -= dt;
      person->speed = 0;
      person->phase += 14.0f * dt;
      /* Off like a shot when it passes. */
      if (person->stagger <= 0) person->speed = person->cruise * 1.8f;
      continue;
    }
    /* Hold still while a car is within a few metres. */
    if (person->panic > 0) person->panic -= dt;
    float target = near_car < 4.0f * 4.0f ? 0.0f : person->cruise;
    if (person->panic > 0) target = person->cruise * 2.6f;
    float rate = target < person->speed ? 6.0f : 1.5f;
    if (person->speed < target)
      person->speed = fminf(target, person->speed + rate * dt);
    else
      person->speed = fmaxf(target, person->speed - rate * dt);
    if (person->cooldown > 0) person->cooldown -= dt;
    const route_path_t* path = &p->paths[person->route];
    if (person->cross > 0) {
      /* Over the road at a steady pace; the far pavement ends it. */
      float width = fabsf(2.0f * lateral_offset(path->route, person->side));
      person->cross += (person->speed > 0.2f ? 1.45f : 0.0f) * dt / width;
      person->phase += person->speed * dt * 5.2f;
      if (person->cross >= 1.0f) {
        person->cross = 0;
        person->side = -person->side;
        person->cooldown = 25.0f;
      }
      float target_speed = near_car < 4.0f * 4.0f ? 0.0f : 1.45f;
      person->speed += (target_speed - person->speed) * fminf(1.0f, 6.0f * dt);
      pose(p, person);
      continue;
    }
    maybe_start_crossing(p, person, player);
    float step = person->speed * dt * (float)person->direction;
    person->u += step;
    person->phase += person->speed * dt * 5.2f;
    float length =
        p->paths[person->route].length - p->margin_end[person->route];
    float first = p->margin_start[person->route];
    if (person->u >= length && person->direction > 0) {
      person->u = length;
      reach_the_end(p, person);
    } else if (person->u <= first && person->direction < 0) {
      person->u = first;
      reach_the_end(p, person);
    }
    pose(p, person);
  }
}

pedestrian_meshes_t pedestrian_meshes_create(renderer_t* renderer) {
  pedestrian_meshes_t meshes;
  memset(&meshes, 0, sizeof(meshes));
  mesh_data_t torso = mesh_data_create(), head = mesh_data_create(),
              leg = mesh_data_create(), arm = mesh_data_create();
  mesh_data_add_box(&torso, vec3(0, 1.12f, 0), vec3(0.2f, 0.3f, 0.11f));
  mesh_data_add_box(&head, vec3(0, 1.64f, 0), vec3(0.1f, 0.12f, 0.1f));
  mesh_data_add_box(&head, vec3(0, 1.47f, 0),
                    vec3(0.045f, 0.07f, 0.045f)); /* neck */
  /* Legs and arms hang from a pivot at the origin so they swing about it. */
  mesh_data_add_box(&leg, vec3(0, -0.4f, 0), vec3(0.075f, 0.4f, 0.075f));
  mesh_data_add_box(&arm, vec3(0, -0.28f, 0), vec3(0.055f, 0.3f, 0.055f));
  mesh_data_t skirt = mesh_data_create(), hair_short = mesh_data_create(),
              hair_long = mesh_data_create(), bag = mesh_data_create(),
              backpack = mesh_data_create(), cap = mesh_data_create();
  mesh_data_add_box(&skirt, vec3(0, 0.74f, 0), vec3(0.215f, 0.17f, 0.14f));
  mesh_data_add_box(&hair_short, vec3(0, 1.73f, 0.01f),
                    vec3(0.108f, 0.05f, 0.108f));
  mesh_data_add_box(&hair_long, vec3(0, 1.73f, 0.01f),
                    vec3(0.108f, 0.05f, 0.108f));
  mesh_data_add_box(&hair_long, vec3(0, 1.58f, 0.085f),
                    vec3(0.105f, 0.16f, 0.03f));
  mesh_data_add_box(&bag, vec3(0.3f, 0.88f, 0.02f), vec3(0.04f, 0.12f, 0.13f));
  mesh_data_add_box(&bag, vec3(0.2f, 1.2f, 0.02f), vec3(0.012f, 0.2f, 0.012f));
  mesh_data_add_box(&backpack, vec3(0, 1.15f, 0.17f),
                    vec3(0.14f, 0.21f, 0.07f));
  mesh_data_add_box(&cap, vec3(0, 1.77f, -0.02f), vec3(0.112f, 0.03f, 0.125f));
  meshes.torso = renderer_upload_mesh(renderer, &torso);
  meshes.head = renderer_upload_mesh(renderer, &head);
  meshes.leg = renderer_upload_mesh(renderer, &leg);
  meshes.arm = renderer_upload_mesh(renderer, &arm);
  meshes.skirt = renderer_upload_mesh(renderer, &skirt);
  meshes.hair_short = renderer_upload_mesh(renderer, &hair_short);
  meshes.hair_long = renderer_upload_mesh(renderer, &hair_long);
  meshes.bag = renderer_upload_mesh(renderer, &bag);
  meshes.backpack = renderer_upload_mesh(renderer, &backpack);
  /* Eyes, brows and a mouth on the front of the head (-z). */
  mesh_data_t face = mesh_data_create();
  for (int side = -1; side <= 1; side += 2) {
    mesh_data_add_box(&face, vec3(side * 0.045f, 1.665f, -0.1f),
                      vec3(0.015f, 0.014f, 0.004f));
    mesh_data_add_box(&face, vec3(side * 0.045f, 1.7f, -0.1f),
                      vec3(0.025f, 0.006f, 0.004f));
  }
  mesh_data_add_box(&face, vec3(0, 1.59f, -0.1f), vec3(0.03f, 0.007f, 0.004f));
  meshes.face = renderer_upload_mesh(renderer, &face);
  mesh_data_free(&face);
  meshes.cap = renderer_upload_mesh(renderer, &cap);
  mesh_data_free(&torso);
  mesh_data_free(&head);
  mesh_data_free(&leg);
  mesh_data_free(&arm);
  mesh_data_free(&skirt);
  mesh_data_free(&hair_short);
  mesh_data_free(&hair_long);
  mesh_data_free(&bag);
  mesh_data_free(&backpack);
  mesh_data_free(&cap);
  return meshes;
}

void pedestrian_meshes_set_textures(pedestrian_meshes_t* meshes,
                                    texture_handle_t cloth,
                                    texture_handle_t skin,
                                    texture_handle_t hair) {
  meshes->cloth = cloth, meshes->skin = skin, meshes->hair = hair;
}

void pedestrian_meshes_destroy(pedestrian_meshes_t* meshes,
                               renderer_t* renderer) {
  renderer_release_mesh(renderer, meshes->torso);
  renderer_release_mesh(renderer, meshes->head);
  renderer_release_mesh(renderer, meshes->leg);
  renderer_release_mesh(renderer, meshes->arm);
  renderer_release_mesh(renderer, meshes->skirt);
  renderer_release_mesh(renderer, meshes->hair_short);
  renderer_release_mesh(renderer, meshes->hair_long);
  renderer_release_mesh(renderer, meshes->bag);
  renderer_release_mesh(renderer, meshes->backpack);
  renderer_release_mesh(renderer, meshes->cap);
  renderer_release_mesh(renderer, meshes->face);
}

void pedestrian_draw_figure(const pedestrian_meshes_t* meshes,
                            renderer_t* renderer, vec3_t position, float yaw,
                            float speed, float phase, int outfit) {
  pedestrian_draw_figure_fallen(meshes, renderer, position, yaw, speed, phase,
                                outfit, 0.0f, 0.0f, 0.0f, 0.0f);
}

void pedestrian_draw_figure_fallen(const pedestrian_meshes_t* meshes,
                                   renderer_t* renderer, vec3_t position,
                                   float yaw, float speed, float phase,
                                   int outfit, float fall, float lift,
                                   float flail, float punch) {
  static const float shirts[8][3] = {{0.75f, 0.2f, 0.18f}, {0.2f, 0.35f, 0.65f},
                                     {0.85f, 0.85f, 0.8f}, {0.2f, 0.5f, 0.3f},
                                     {0.9f, 0.7f, 0.2f},   {0.35f, 0.3f, 0.35f},
                                     {0.6f, 0.4f, 0.25f},  {0.1f, 0.1f, 0.12f}};
  static const float trousers[5][3] = {{0.12f, 0.14f, 0.28f},
                                       {0.1f, 0.1f, 0.1f},
                                       {0.45f, 0.38f, 0.28f},
                                       {0.3f, 0.3f, 0.33f},
                                       {0.2f, 0.22f, 0.3f}};
  static const float skins[4][3] = {{0.92f, 0.74f, 0.6f},
                                    {0.8f, 0.58f, 0.44f},
                                    {0.6f, 0.42f, 0.3f},
                                    {0.95f, 0.8f, 0.7f}};
  static const float hairs[6][3] = {
      {0.08f, 0.06f, 0.05f}, {0.25f, 0.15f, 0.08f}, {0.55f, 0.4f, 0.18f},
      {0.75f, 0.62f, 0.35f}, {0.5f, 0.18f, 0.08f},  {0.12f, 0.1f, 0.09f}};
  int o = outfit;
  unsigned h = outfit_hash(o);
  person_kind_t kind = pedestrian_kind(o);
  float s = pedestrian_scale(o);
  const float* shirt = shirts[o % 8];
  const float* trouser = trousers[(o / 8) % 5];
  const float* skin = skins[(o / 40) % 4];
  const float* hair = kind == PERSON_ELDER ? hairs[5] : hairs[h / 7 % 5];
  bool dress = kind == PERSON_ADULT && (h / 3) % 10 < 3;
  bool long_hair = kind != PERSON_ELDER && (dress || (h / 11) % 10 < 2);
  float grey = kind == PERSON_ELDER ? 0.62f : 1.0f;
  material_t shirt_material =
      material_rough(vec3(shirt[0], shirt[1], shirt[2]), 0.9f);
  material_t skin_material =
      material_rough(vec3(skin[0], skin[1], skin[2]), 0.8f);
  material_t hair_material = material_rough(
      vec3(hair[0] * grey + (kind == PERSON_ELDER ? 0.3f : 0.0f),
           hair[1] * grey + (kind == PERSON_ELDER ? 0.3f : 0.0f),
           hair[2] * grey + (kind == PERSON_ELDER ? 0.3f : 0.0f)),
      0.9f);
  material_t trouser_material =
      material_rough(vec3(trouser[0], trouser[1], trouser[2]), 0.9f);
  shirt_material.texture = meshes->cloth;
  trouser_material.texture = meshes->cloth;
  skin_material.texture = meshes->skin;
  hair_material.texture = meshes->hair;
  mat4_t scaling = mat4_scaling(vec3(s, s, s));
  /* Tipped backwards, and lifted by the thickness of a body. On the ground it
   * turns about the feet; thrown into the air, about the middle of the body. */
  position.y += 0.14f * s * fminf(fall, 1.0f) + lift;
  quat_t tip = quat_from_axis_angle(vec3(1, 0, 0), fall * 1.5708f);
  quat_t turn = quat_multiply(quat_from_yaw(yaw), tip);
  float pivot = 0.9f * s * fminf(1.0f, lift / 0.6f);
  if (pivot > 0) {
    vec3_t swung = quat_rotate(turn, vec3(0, pivot, 0));
    position = vec3_add(position, vec3(-swung.x, pivot - swung.y, -swung.z));
  }
  transform_t base = transform_at(position, turn);
  mat4_t model = mat4_multiply(transform_to_matrix(base), scaling);
  renderer_draw_mesh(renderer, meshes->torso, shirt_material, model);
  renderer_draw_mesh(renderer, meshes->head, skin_material, model);
  renderer_draw_mesh(renderer, meshes->face,
                     material_rough(vec3(0.05f, 0.04f, 0.04f), 0.6f), model);
  if (kind == PERSON_ELDER)
    renderer_draw_mesh(renderer, meshes->cap,
                       material_rough(vec3(0.3f, 0.28f, 0.25f), 0.9f), model);
  else
    renderer_draw_mesh(renderer,
                       long_hair ? meshes->hair_long : meshes->hair_short,
                       hair_material, model);
  if (dress)
    renderer_draw_mesh(renderer, meshes->skirt, trouser_material, model);
  if (kind == PERSON_CHILD || (kind == PERSON_ADULT && (h / 5) % 10 == 0))
    renderer_draw_mesh(renderer, meshes->backpack,
                       material_rough(vec3(shirt[2], shirt[0], shirt[1]), 0.9f),
                       model);
  else if (kind == PERSON_ADULT && (h / 5) % 10 < 3)
    renderer_draw_mesh(renderer, meshes->bag,
                       material_rough(vec3(0.35f, 0.22f, 0.12f), 0.9f), model);
  float amplitude = kind == PERSON_JOGGER ? 0.85f : 0.55f;
  /* Someone really running (the player) throws arms and legs wider. */
  if (speed > 3.5f) amplitude = fmaxf(amplitude, 1.0f);
  float swing = sinf(phase) * amplitude *
                fminf(speed, kind == PERSON_JOGGER ? 3.2f : 1.4f) /
                (kind == PERSON_JOGGER ? 3.2f : 1.4f);
  for (int side = -1; side <= 1; side += 2) {
    float angle = side * swing;
    float arm_angle = -angle;
    /* The right arm (+x) thrown forward in a punch. */
    if (punch > 0 && side == 1) arm_angle = 1.55f * punch;
    if (flail > 0) {
      /* Legs kick and arms flung up and waved about, out of step. */
      angle = flail * (side * 1.0f * sinf(phase) + 0.4f * sinf(phase * 2.3f));
      arm_angle = -flail * (2.0f + 0.9f * sinf(phase * 1.7f + side * 1.3f));
    }
    /* Pivot positions in the person's frame: hips and shoulders. */
    transform_t leg = base;
    leg.position = transform_point(base, vec3(side * 0.1f * s, 0.8f * s, 0));
    leg.rotation = quat_multiply(base.rotation,
                                 quat_from_axis_angle(vec3(1, 0, 0), angle));
    renderer_draw_mesh(renderer, meshes->leg,
                       dress ? skin_material : trouser_material,
                       mat4_multiply(transform_to_matrix(leg), scaling));
    transform_t arm = base;
    arm.position = transform_point(base, vec3(side * 0.27f * s, 1.38f * s, 0));
    arm.rotation = quat_multiply(
        base.rotation, quat_from_axis_angle(vec3(1, 0, 0), arm_angle));
    renderer_draw_mesh(renderer, meshes->arm, shirt_material,
                       mat4_multiply(transform_to_matrix(arm), scaling));
  }
}

int pedestrians_draw(const pedestrians_t* p, const pedestrian_meshes_t* meshes,
                     renderer_t* renderer, vec3_t eye, float range) {
  int drawn = 0;
  for (size_t i = 0; i < p->count; i++) {
    const person_t* person = &p->people[i];
    float dx = person->position.x - eye.x, dz = person->position.z - eye.z;
    if (dx * dx + dz * dz > range * range) continue;
    pedestrian_draw_figure_fallen(meshes, renderer, person->position,
                                  person->yaw, person->speed, person->phase,
                                  person->outfit, body_fall(person),
                                  person->lift, body_flail(person), 0.0f);
    drawn++;
  }
  return drawn;
}
