#include "aabb.h"

#include <float.h>

aabb_t aabb_empty(void) {
  aabb_t box = {{FLT_MAX, FLT_MAX, FLT_MAX}, {-FLT_MAX, -FLT_MAX, -FLT_MAX}};
  return box;
}

bool aabb_is_empty(aabb_t box) {
  return box.min.x > box.max.x || box.min.y > box.max.y || box.min.z > box.max.z;
}

aabb_t aabb_from_center(vec3_t center, vec3_t half_extents) {
  aabb_t box = {vec3_sub(center, half_extents), vec3_add(center, half_extents)};
  return box;
}

aabb_t aabb_include(aabb_t box, vec3_t point) {
  box.min = vec3_min(box.min, point);
  box.max = vec3_max(box.max, point);
  return box;
}

aabb_t aabb_merge(aabb_t a, aabb_t b) {
  if (aabb_is_empty(a)) return b;
  if (aabb_is_empty(b)) return a;
  return aabb_include(aabb_include(a, b.min), b.max);
}

bool aabb_contains(aabb_t box, vec3_t p) {
  return p.x >= box.min.x && p.x <= box.max.x && p.y >= box.min.y && p.y <= box.max.y &&
         p.z >= box.min.z && p.z <= box.max.z;
}

bool aabb_overlaps(aabb_t a, aabb_t b) {
  return a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y &&
         a.max.y >= b.min.y && a.min.z <= b.max.z && a.max.z >= b.min.z;
}

vec3_t aabb_center(aabb_t box) { return vec3_scale(vec3_add(box.min, box.max), 0.5f); }

vec3_t aabb_half_extents(aabb_t box) {
  return vec3_scale(vec3_sub(box.max, box.min), 0.5f);
}

aabb_t aabb_transform(aabb_t box, transform_t t) {
  aabb_t result = aabb_empty();
  for (int corner = 0; corner < 8; corner++) {
    vec3_t local = vec3((corner & 1) ? box.max.x : box.min.x,
                        (corner & 2) ? box.max.y : box.min.y,
                        (corner & 4) ? box.max.z : box.min.z);
    result = aabb_include(result, transform_point(t, local));
  }
  return result;
}
