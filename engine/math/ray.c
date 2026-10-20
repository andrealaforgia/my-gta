#include "ray.h"

#include <math.h>

bool ray_triangle(ray_t ray, vec3_t a, vec3_t b, vec3_t c, ray_hit_t* hit) {
  const float epsilon = 1e-7f;
  vec3_t edge1 = vec3_sub(b, a);
  vec3_t edge2 = vec3_sub(c, a);
  vec3_t p = vec3_cross(ray.direction, edge2);
  float determinant = vec3_dot(edge1, p);
  if (fabsf(determinant) < epsilon) return false;
  float inverse = 1.0f / determinant;
  vec3_t s = vec3_sub(ray.origin, a);
  float u = vec3_dot(s, p) * inverse;
  if (u < 0 || u > 1) return false;
  vec3_t q = vec3_cross(s, edge1);
  float v = vec3_dot(ray.direction, q) * inverse;
  if (v < 0 || u + v > 1) return false;
  float t = vec3_dot(edge2, q) * inverse;
  if (t < 0) return false;
  hit->distance = t;
  hit->point = vec3_add_scaled(ray.origin, ray.direction, t);
  vec3_t normal = vec3_normalize(vec3_cross(edge1, edge2));
  if (vec3_dot(normal, ray.direction) > 0) normal = vec3_negate(normal);
  hit->normal = normal;
  return true;
}

static bool slab(float origin, float direction, float lo, float hi,
                 float* t_min, float* t_max) {
  if (fabsf(direction) < 1e-12f) return origin >= lo && origin <= hi;
  float inverse = 1.0f / direction;
  float t1 = (lo - origin) * inverse;
  float t2 = (hi - origin) * inverse;
  if (t1 > t2) {
    float swap = t1;
    t1 = t2;
    t2 = swap;
  }
  if (t1 > *t_min) *t_min = t1;
  if (t2 < *t_max) *t_max = t2;
  return *t_min <= *t_max;
}

bool ray_aabb(ray_t ray, aabb_t box, float* t_min, float* t_max) {
  *t_min = -INFINITY;
  *t_max = INFINITY;
  if (!slab(ray.origin.x, ray.direction.x, box.min.x, box.max.x, t_min, t_max))
    return false;
  if (!slab(ray.origin.y, ray.direction.y, box.min.y, box.max.y, t_min, t_max))
    return false;
  if (!slab(ray.origin.z, ray.direction.z, box.min.z, box.max.z, t_min, t_max))
    return false;
  return *t_max >= 0;
}

bool ray_plane(ray_t ray, plane_t plane, float* t) {
  float denominator = vec3_dot(plane.normal, ray.direction);
  if (fabsf(denominator) < 1e-12f) return false;
  *t = (plane.distance - vec3_dot(plane.normal, ray.origin)) / denominator;
  return *t >= 0;
}

plane_t plane_from_point_normal(vec3_t point, vec3_t normal) {
  plane_t plane;
  plane.normal = vec3_normalize(normal);
  plane.distance = vec3_dot(plane.normal, point);
  return plane;
}

plane_t plane_from_points(vec3_t a, vec3_t b, vec3_t c) {
  return plane_from_point_normal(a, vec3_cross(vec3_sub(b, a), vec3_sub(c, a)));
}

float plane_distance(plane_t plane, vec3_t point) {
  return vec3_dot(plane.normal, point) - plane.distance;
}
