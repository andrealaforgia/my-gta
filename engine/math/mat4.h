/**
 * @file mat4.h
 * @brief 4x4 column-major matrices: m[column * 4 + row]. Vectors are
 * columns, so mat4_multiply(a, b) applies b first.
 */
#ifndef ENGINE_MATH_MAT4_H_
#define ENGINE_MATH_MAT4_H_

#include "quat.h"
#include "vec3.h"

typedef struct {
  float m[16];
} mat4_t;

mat4_t mat4_identity(void);
mat4_t mat4_multiply(mat4_t a, mat4_t b);
mat4_t mat4_translation(vec3_t t);
mat4_t mat4_scaling(vec3_t s);
mat4_t mat4_from_quat(quat_t q);
mat4_t mat4_transpose(mat4_t a);
/* Inverse of a rotation + translation matrix (no scale). */
mat4_t mat4_inverse_rigid(mat4_t a);
/* General inverse; returns identity for a singular matrix. */
mat4_t mat4_inverse(mat4_t a);
/* Vertical field of view in radians, right-handed, looking down -Z,
 * clip z in [-1, 1] as OpenGL expects. */
mat4_t mat4_perspective(float fov_y, float aspect, float near_plane, float far_plane);
mat4_t mat4_look_at(vec3_t eye, vec3_t target, vec3_t up);
vec3_t mat4_transform_point(mat4_t a, vec3_t p);
vec3_t mat4_transform_direction(mat4_t a, vec3_t d);
/* Transform and divide by w. */
vec3_t mat4_project_point(mat4_t a, vec3_t p);

#endif  // ENGINE_MATH_MAT4_H_
