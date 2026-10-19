/**
 * @file quat.h
 * @brief Unit quaternions (x, y, z, w) for orientation. Rotation about +Y by
 * a positive angle turns forward (-Z) towards -X: anticlockwise from above.
 */
#ifndef ENGINE_MATH_QUAT_H_
#define ENGINE_MATH_QUAT_H_

#include "vec3.h"

typedef struct {
  float x;
  float y;
  float z;
  float w;
} quat_t;

quat_t quat(float x, float y, float z, float w);
quat_t quat_identity(void);
quat_t quat_from_axis_angle(vec3_t axis, float radians);
quat_t quat_from_yaw(float radians);
/* a then b: the result rotates by b first, then by a (matrix order). */
quat_t quat_multiply(quat_t a, quat_t b);
quat_t quat_conjugate(quat_t q);
quat_t quat_normalize(quat_t q);
vec3_t quat_rotate(quat_t q, vec3_t v);
vec3_t quat_forward(quat_t q); /* rotated -Z */
vec3_t quat_right(quat_t q);   /* rotated +X */
vec3_t quat_up(quat_t q);      /* rotated +Y */
float quat_yaw(quat_t q);      /* heading about +Y, radians */
/* Advance an orientation by an angular velocity (radians per second,
 * world frame) over dt seconds. */
quat_t quat_integrate(quat_t q, vec3_t angular_velocity, float dt);
/* Normalised linear interpolation along the shorter arc. */
quat_t quat_nlerp(quat_t a, quat_t b, float t);

#endif  // ENGINE_MATH_QUAT_H_
