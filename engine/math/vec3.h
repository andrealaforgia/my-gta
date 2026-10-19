/**
 * @file vec3.h
 * @brief Three-component float vectors. +X east, +Y up, +Z south; see
 * docs/decisions/ADR-007-coordinate-conventions.md.
 */
#ifndef ENGINE_MATH_VEC3_H_
#define ENGINE_MATH_VEC3_H_

typedef struct {
  float x;
  float y;
  float z;
} vec3_t;

vec3_t vec3(float x, float y, float z);
vec3_t vec3_zero(void);
vec3_t vec3_add(vec3_t a, vec3_t b);
vec3_t vec3_sub(vec3_t a, vec3_t b);
vec3_t vec3_scale(vec3_t a, float factor);
vec3_t vec3_negate(vec3_t a);
vec3_t vec3_multiply(vec3_t a, vec3_t b);
float vec3_dot(vec3_t a, vec3_t b);
vec3_t vec3_cross(vec3_t a, vec3_t b);
float vec3_length(vec3_t a);
float vec3_length_squared(vec3_t a);
float vec3_distance(vec3_t a, vec3_t b);
/* The zero vector normalises to the zero vector. */
vec3_t vec3_normalize(vec3_t a);
vec3_t vec3_lerp(vec3_t a, vec3_t b, float t);
vec3_t vec3_min(vec3_t a, vec3_t b);
vec3_t vec3_max(vec3_t a, vec3_t b);
/* Add b * factor to a. */
vec3_t vec3_add_scaled(vec3_t a, vec3_t b, float factor);

#endif  // ENGINE_MATH_VEC3_H_
