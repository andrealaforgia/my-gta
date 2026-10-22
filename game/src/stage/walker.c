#include "walker.h"

#include <math.h>

#define OBSTACLES \
  (COLLISION_LAYER_BUILDING | COLLISION_LAYER_BARRIER | COLLISION_LAYER_PROP)
#define TURN_RATE 2.8f

void walker_init(walker_t* walker, vec3_t feet, float yaw) {
  *walker = (walker_t){0};
  walker->position = feet;
  walker->yaw = yaw;
  walker->grounded = true;
}

vec3_t walker_forward(const walker_t* walker) {
  return vec3(-sinf(walker->yaw), 0.0f, -cosf(walker->yaw));
}

/* The height of the ground under (x, z), looking down from `from_y`; false if
 * there is none within reach. */
static bool ground_at(const physics_world_t* physics, float x, float z,
                      float from_y, float reach, float* height) {
  ray_t down = {vec3(x, from_y, z), vec3(0, -1, 0)};
  ray_cast_result_t hit;
  if (!physics_ray_cast(physics, down, reach, COLLISION_LAYER_GROUND, &hit))
    return false;
  *height = hit.hit.point.y;
  return true;
}

/* Keeps the walker's body clear of walls and props: from chest and knee
 * height, rays go out in several directions and anything closer than the
 * walker's radius pushes back along its normal. */
static void keep_clear(walker_t* walker, const physics_world_t* physics) {
  static const int directions = 8;
  /* At the height of a knee-high wall, which everything of concern reaches. */
  for (int d = 0; d < directions; d++) {
    float angle = 6.2831853f * (float)d / (float)directions;
    vec3_t dir = vec3(cosf(angle), 0, sinf(angle));
    ray_t ray = {vec3_add(walker->position, vec3(0, 0.55f, 0)), dir};
    ray_cast_result_t hit;
    if (!physics_ray_cast(physics, ray, WALKER_RADIUS, OBSTACLES, &hit))
      continue;
    float push = WALKER_RADIUS - hit.hit.distance;
    vec3_t normal = hit.hit.normal;
    float flat = sqrtf(normal.x * normal.x + normal.z * normal.z);
    if (flat < 0.2f) continue; /* the top or bottom of something */
    walker->position.x += normal.x / flat * push;
    walker->position.z += normal.z / flat * push;
  }
}

void walker_step(walker_t* walker, const physics_world_t* physics,
                 walker_input_t input, float dt) {
  walker->yaw -= input.turn * TURN_RATE * dt;
  float wanted =
      input.forward * (input.run ? WALKER_RUN_SPEED : WALKER_WALK_SPEED);
  if (input.forward < 0) wanted *= 0.5f;
  float rate = 12.0f * dt;
  walker->speed += fmaxf(-rate, fminf(rate, wanted - walker->speed));
  vec3_t forward = walker_forward(walker);
  float nx = walker->position.x + forward.x * walker->speed * dt;
  float nz = walker->position.z + forward.z * walker->speed * dt;
  /* A step up too high is a wall: stay where we are. */
  float ground;
  bool found =
      ground_at(physics, nx, nz, walker->position.y + 1.4f, 4.0f, &ground);
  if (found && ground - walker->position.y > WALKER_STEP_HEIGHT) {
    walker->speed = 0;
  } else if (found || !walker->grounded) {
    walker->position.x = nx;
    walker->position.z = nz;
  }
  if (fabsf(walker->speed) > 0.05f || input.turn != 0.0f)
    keep_clear(walker, physics); /* standing still, nothing to push */
  /* Down to the ground, falling if it has dropped away. */
  if (ground_at(physics, walker->position.x, walker->position.z,
                walker->position.y + 1.4f, 8.0f, &ground)) {
    if (ground >= walker->position.y - 0.05f || walker->grounded) {
      float follow = fminf(1.0f, 18.0f * dt);
      walker->position.y += (ground - walker->position.y) * follow;
      if (fabsf(ground - walker->position.y) < 0.03f) {
        walker->position.y = ground;
        walker->grounded = true;
      }
      walker->fall = 0;
    } else {
      walker->grounded = false;
      walker->fall += 9.8f * dt;
      walker->position.y -= walker->fall * dt;
      if (walker->position.y <= ground) {
        walker->position.y = ground;
        walker->grounded = true;
        walker->fall = 0;
      }
    }
  }
  walker->phase += fabsf(walker->speed) * 3.6f * dt;
}

void walker_push_out_of_box(walker_t* walker, vec3_t centre, float yaw,
                            vec3_t half_extents) {
  /* In the box's own frame. */
  float s = sinf(yaw), c = cosf(yaw);
  float dx = walker->position.x - centre.x, dz = walker->position.z - centre.z;
  float local_x = dx * c - dz * s, local_z = dx * s + dz * c;
  float reach_x = half_extents.x + WALKER_RADIUS;
  float reach_z = half_extents.z + WALKER_RADIUS;
  if (walker->position.y > centre.y + half_extents.y) return; /* on top */
  if (fabsf(local_x) >= reach_x || fabsf(local_z) >= reach_z) return;
  /* Out by the nearest side. */
  float over_x = reach_x - fabsf(local_x), over_z = reach_z - fabsf(local_z);
  if (over_x < over_z)
    local_x += (local_x < 0 ? -1.0f : 1.0f) * over_x;
  else
    local_z += (local_z < 0 ? -1.0f : 1.0f) * over_z;
  walker->position.x = centre.x + local_x * c + local_z * s;
  walker->position.z = centre.z - local_x * s + local_z * c;
}
