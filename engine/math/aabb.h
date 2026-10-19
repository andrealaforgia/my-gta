/**
 * @file aabb.h
 * @brief Axis-aligned bounding boxes.
 */
#ifndef ENGINE_MATH_AABB_H_
#define ENGINE_MATH_AABB_H_

#include <stdbool.h>

#include "transform.h"
#include "vec3.h"

typedef struct {
  vec3_t min;
  vec3_t max;
} aabb_t;

/* An empty box has min > max so that including any point defines it. */
aabb_t aabb_empty(void);
bool aabb_is_empty(aabb_t box);
aabb_t aabb_from_center(vec3_t center, vec3_t half_extents);
aabb_t aabb_include(aabb_t box, vec3_t point);
aabb_t aabb_merge(aabb_t a, aabb_t b);
bool aabb_contains(aabb_t box, vec3_t point);
bool aabb_overlaps(aabb_t a, aabb_t b);
vec3_t aabb_center(aabb_t box);
vec3_t aabb_half_extents(aabb_t box);
/* Bounding box of the eight transformed corners. */
aabb_t aabb_transform(aabb_t box, transform_t t);

#endif  // ENGINE_MATH_AABB_H_
