#include "chase_camera.h"

#include <math.h>

#include "interpolation.h"

chase_camera_params_t chase_camera_default_params(void) {
  chase_camera_params_t p = {6.5f, 2.4f, 1.0f, 0.12f, 6.0f,
                             5.0f, 8.0f, 0.3f, 1.05f};
  return p;
}

chase_camera_t chase_camera_create(chase_camera_params_t params) {
  chase_camera_t camera = {params, {0, 0, 0}, {0, 0, 0}, false};
  return camera;
}

static vec3_t flat_forward(transform_t target) {
  vec3_t forward = quat_forward(target.rotation);
  forward.y = 0;
  forward = vec3_normalize(forward);
  if (vec3_length(forward) < 0.5f) forward = vec3(0, 0, -1);
  return forward;
}

static vec3_t desired_position(const chase_camera_t* camera,
                               transform_t target) {
  vec3_t forward = flat_forward(target);
  return vec3_add(
      vec3_add_scaled(target.position, forward, -camera->params.distance),
      vec3(0, camera->params.height, 0));
}

static vec3_t desired_look(const chase_camera_t* camera, transform_t target,
                           vec3_t velocity) {
  vec3_t forward = flat_forward(target);
  float ahead = vec3_dot(velocity, forward) * camera->params.look_ahead;
  ahead = clampf(ahead, 0.0f, camera->params.max_look_ahead);
  return vec3_add(vec3_add_scaled(target.position, forward, ahead),
                  vec3(0, camera->params.look_height, 0));
}

void chase_camera_reset(chase_camera_t* camera, transform_t target) {
  camera->position = desired_position(camera, target);
  camera->look_target = desired_look(camera, target, vec3_zero());
  camera->initialised = true;
}

static vec3_t avoid_geometry(const chase_camera_t* camera,
                             const physics_world_t* world, vec3_t from,
                             vec3_t to) {
  if (!world) return to;
  vec3_t offset = vec3_sub(to, from);
  float length = vec3_length(offset);
  if (length < 1e-4f) return to;
  ray_t ray = {from, vec3_scale(offset, 1.0f / length)};
  ray_cast_result_t result;
  if (!physics_ray_cast(world, ray, length,
                        COLLISION_LAYER_BUILDING | COLLISION_LAYER_BARRIER,
                        &result))
    return to;
  float pulled =
      fmaxf(result.hit.distance - camera->params.collision_margin, 0.5f);
  return vec3_add_scaled(from, ray.direction, pulled);
}

static vec3_t damp_vector(vec3_t value, vec3_t target, float rate, float dt) {
  return vec3_lerp(value, target, 1.0f - expf(-rate * dt));
}

void chase_camera_update(chase_camera_t* camera, transform_t target,
                         vec3_t target_velocity, const physics_world_t* world,
                         float dt) {
  if (!camera->initialised) chase_camera_reset(camera, target);
  vec3_t wanted = desired_position(camera, target);
  camera->position =
      damp_vector(camera->position, wanted, camera->params.position_rate, dt);
  vec3_t anchor =
      vec3_add(target.position, vec3(0, camera->params.look_height, 0));
  camera->position = avoid_geometry(camera, world, anchor, camera->position);
  camera->look_target = damp_vector(
      camera->look_target, desired_look(camera, target, target_velocity),
      camera->params.look_rate, dt);
}

render_camera_t chase_camera_render(const chase_camera_t* camera) {
  render_camera_t view = {camera->position,
                          camera->look_target,
                          {0, 1, 0},
                          camera->params.fov_y,
                          0.2f,
                          1500.0f};
  return view;
}
