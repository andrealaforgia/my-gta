/**
 * @file chase_camera.h
 * @brief A spring-damped chase camera: sits behind and above the car,
 * looks ahead as speed rises, and pulls in when static geometry sits
 * between it and the car.
 */
#ifndef ENGINE_CAMERA_CHASE_CAMERA_H_
#define ENGINE_CAMERA_CHASE_CAMERA_H_

#include "physics_world.h"
#include "renderer.h"
#include "transform.h"

typedef struct {
  float distance;       /* m behind the car */
  float height;         /* m above the car's origin */
  float look_height;    /* m above the car's origin to look at */
  float look_ahead;     /* m ahead per m/s of speed */
  float max_look_ahead; /* m */
  float position_rate;  /* approach rate per second */
  float look_rate;
  float collision_margin;
  float fov_y;
} chase_camera_params_t;

typedef struct {
  chase_camera_params_t params;
  vec3_t position;
  vec3_t look_target;
  bool initialised;
} chase_camera_t;

chase_camera_params_t chase_camera_default_params(void);
chase_camera_t chase_camera_create(chase_camera_params_t params);
void chase_camera_reset(chase_camera_t* camera, transform_t target);
/* world may be NULL when no collision avoidance is wanted. */
void chase_camera_update(chase_camera_t* camera, transform_t target,
                         vec3_t target_velocity, const physics_world_t* world,
                         float dt);
render_camera_t chase_camera_render(const chase_camera_t* camera);

#endif  // ENGINE_CAMERA_CHASE_CAMERA_H_
