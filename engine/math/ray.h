/**
 * @file ray.h
 * @brief Rays, planes and the intersection tests the physics and camera
 * need. Directions are expected to be unit length.
 */
#ifndef ENGINE_MATH_RAY_H_
#define ENGINE_MATH_RAY_H_

#include <stdbool.h>

#include "aabb.h"
#include "vec3.h"

typedef struct {
  vec3_t origin;
  vec3_t direction;
} ray_t;

typedef struct {
  float distance;
  vec3_t point;
  vec3_t normal;
} ray_hit_t;

typedef struct {
  vec3_t normal;
  float distance; /* n . p = distance for points on the plane */
} plane_t;

/* Front and back faces both count; the normal faces the ray origin. */
bool ray_triangle(ray_t ray, vec3_t a, vec3_t b, vec3_t c, ray_hit_t* hit);
bool ray_aabb(ray_t ray, aabb_t box, float* t_min, float* t_max);
bool ray_plane(ray_t ray, plane_t plane, float* t);

plane_t plane_from_point_normal(vec3_t point, vec3_t normal);
/* Anticlockwise a, b, c seen from the front. */
plane_t plane_from_points(vec3_t a, vec3_t b, vec3_t c);
float plane_distance(plane_t plane, vec3_t point);

#endif  // ENGINE_MATH_RAY_H_
