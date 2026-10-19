/**
 * @file frustum.h
 * @brief View frustum extracted from a projection * view matrix, for
 * culling.
 */
#ifndef ENGINE_MATH_FRUSTUM_H_
#define ENGINE_MATH_FRUSTUM_H_

#include <stdbool.h>

#include "aabb.h"
#include "mat4.h"
#include "ray.h"

typedef struct {
  plane_t planes[6]; /* normals point inwards */
} frustum_t;

frustum_t frustum_from_matrix(mat4_t view_projection);
bool frustum_intersects_aabb(frustum_t f, aabb_t box);
bool frustum_intersects_sphere(frustum_t f, vec3_t center, float radius);

#endif  // ENGINE_MATH_FRUSTUM_H_
