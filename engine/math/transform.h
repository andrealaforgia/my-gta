/**
 * @file transform.h
 * @brief Position, rotation and scale. Applied as scale, then rotation,
 * then translation.
 */
#ifndef ENGINE_MATH_TRANSFORM_H_
#define ENGINE_MATH_TRANSFORM_H_

#include "mat4.h"
#include "quat.h"
#include "vec3.h"

typedef struct {
  vec3_t position;
  quat_t rotation;
  vec3_t scale;
} transform_t;

transform_t transform_identity(void);
transform_t transform_at(vec3_t position, quat_t rotation);
mat4_t transform_to_matrix(transform_t t);
vec3_t transform_point(transform_t t, vec3_t local);
vec3_t transform_direction(transform_t t, vec3_t local);
vec3_t transform_inverse_point(transform_t t, vec3_t world);
transform_t transform_lerp(transform_t a, transform_t b, float t);

#endif  // ENGINE_MATH_TRANSFORM_H_
