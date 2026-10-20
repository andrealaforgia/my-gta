#include "pedestrians.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "route_path.h"

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
} person_t;

struct pedestrians {
  route_path_t* paths;
  float* margin_start; /* walkers stop short of junctions, where the road is */
  float* margin_end;
  size_t path_count;
  person_t* people;
  size_t count;
  uint32_t random;
};

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
  /* Right of the route's direction (x east, z south): (-dz, dx). */
  person->position = vec3(x - dz * offset, ground, z + dx * offset);
  person->yaw = person->direction > 0 ? atan2f(-dx, -dz) : atan2f(dx, dz);
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
    person->u = 2.0f + pick * (p->paths[chosen].length - 4.0f) /
                           p->paths[chosen].length;
    person->cruise = 1.0f + (next_random(p) % 50) / 100.0f;
    person->speed = person->cruise;
    person->outfit = (int)(next_random(p) % 1000);
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

size_t pedestrians_count(const pedestrians_t* p) { return p->count; }

pedestrian_t pedestrians_get(const pedestrians_t* p, size_t index) {
  const person_t* person = &p->people[index];
  pedestrian_t out = {person->position, person->yaw,    person->speed,
                      person->phase,    person->outfit, person->route};
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

void pedestrians_step(pedestrians_t* p, vec3_t player, float dt) {
  for (size_t i = 0; i < p->count; i++) {
    person_t* person = &p->people[i];
    float dx = player.x - person->position.x,
          dz = player.z - person->position.z;
    float near_car = dx * dx + dz * dz;
    /* Hold still while a car is within a few metres. */
    float target = near_car < 4.0f * 4.0f ? 0.0f : person->cruise;
    float rate = target < person->speed ? 6.0f : 1.5f;
    if (person->speed < target)
      person->speed = fminf(target, person->speed + rate * dt);
    else
      person->speed = fmaxf(target, person->speed - rate * dt);
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
  mesh_data_t torso = mesh_data_create(), head = mesh_data_create(),
              leg = mesh_data_create(), arm = mesh_data_create();
  mesh_data_add_box(&torso, vec3(0, 1.12f, 0), vec3(0.2f, 0.3f, 0.11f));
  mesh_data_add_box(&head, vec3(0, 1.64f, 0), vec3(0.1f, 0.12f, 0.1f));
  /* Legs and arms hang from a pivot at the origin so they swing about it. */
  mesh_data_add_box(&leg, vec3(0, -0.4f, 0), vec3(0.075f, 0.4f, 0.075f));
  mesh_data_add_box(&arm, vec3(0, -0.28f, 0), vec3(0.055f, 0.3f, 0.055f));
  meshes.torso = renderer_upload_mesh(renderer, &torso);
  meshes.head = renderer_upload_mesh(renderer, &head);
  meshes.leg = renderer_upload_mesh(renderer, &leg);
  meshes.arm = renderer_upload_mesh(renderer, &arm);
  mesh_data_free(&torso);
  mesh_data_free(&head);
  mesh_data_free(&leg);
  mesh_data_free(&arm);
  return meshes;
}

void pedestrian_meshes_destroy(pedestrian_meshes_t* meshes,
                               renderer_t* renderer) {
  renderer_release_mesh(renderer, meshes->torso);
  renderer_release_mesh(renderer, meshes->head);
  renderer_release_mesh(renderer, meshes->leg);
  renderer_release_mesh(renderer, meshes->arm);
}

int pedestrians_draw(const pedestrians_t* p, const pedestrian_meshes_t* meshes,
                     renderer_t* renderer, vec3_t eye, float range) {
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
  int drawn = 0;
  for (size_t i = 0; i < p->count; i++) {
    const person_t* person = &p->people[i];
    float dx = person->position.x - eye.x, dz = person->position.z - eye.z;
    if (dx * dx + dz * dz > range * range) continue;
    int o = person->outfit;
    const float* shirt = shirts[o % 8];
    const float* trouser = trousers[(o / 8) % 5];
    const float* skin = skins[(o / 40) % 4];
    transform_t base =
        transform_at(person->position, quat_from_yaw(person->yaw));
    mat4_t model = transform_to_matrix(base);
    renderer_draw_mesh(renderer, meshes->torso,
                       material_rough(vec3(shirt[0], shirt[1], shirt[2]), 0.9f),
                       model);
    renderer_draw_mesh(renderer, meshes->head,
                       material_rough(vec3(skin[0], skin[1], skin[2]), 0.8f),
                       model);
    float swing =
        sinf(person->phase) * 0.55f * fminf(person->speed, 1.4f) / 1.4f;
    for (int side = -1; side <= 1; side += 2) {
      float angle = side * swing;
      /* Pivot positions in the person's frame: hips and shoulders. */
      transform_t leg = base;
      leg.position = transform_point(base, vec3(side * 0.1f, 0.8f, 0));
      leg.rotation = quat_multiply(base.rotation,
                                   quat_from_axis_angle(vec3(1, 0, 0), angle));
      renderer_draw_mesh(
          renderer, meshes->leg,
          material_rough(vec3(trouser[0], trouser[1], trouser[2]), 0.9f),
          transform_to_matrix(leg));
      transform_t arm = base;
      arm.position = transform_point(base, vec3(side * 0.27f, 1.38f, 0));
      arm.rotation = quat_multiply(base.rotation,
                                   quat_from_axis_angle(vec3(1, 0, 0), -angle));
      renderer_draw_mesh(
          renderer, meshes->arm,
          material_rough(vec3(shirt[0], shirt[1], shirt[2]), 0.9f),
          transform_to_matrix(arm));
    }
    drawn++;
  }
  return drawn;
}
