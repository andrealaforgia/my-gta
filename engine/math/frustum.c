#include "frustum.h"

#include <math.h>

static plane_t normalised(float a, float b, float c, float d) {
  /* Plane as a x + b y + c z + d >= 0 inside; stored as n . p = -d. */
  float length = sqrtf(a * a + b * b + c * c);
  plane_t plane;
  plane.normal = vec3(a / length, b / length, c / length);
  plane.distance = -d / length;
  return plane;
}

frustum_t frustum_from_matrix(mat4_t vp) {
  /* Gribb and Hartmann: rows of the matrix combined with the fourth row.
   * Row i of a column-major matrix is m[0*4+i], m[1*4+i], m[2*4+i], m[3*4+i]. */
  const float* m = vp.m;
  frustum_t f;
  f.planes[0] = normalised(m[3] + m[0], m[7] + m[4], m[11] + m[8], m[15] + m[12]);   /* left */
  f.planes[1] = normalised(m[3] - m[0], m[7] - m[4], m[11] - m[8], m[15] - m[12]);   /* right */
  f.planes[2] = normalised(m[3] + m[1], m[7] + m[5], m[11] + m[9], m[15] + m[13]);   /* bottom */
  f.planes[3] = normalised(m[3] - m[1], m[7] - m[5], m[11] - m[9], m[15] - m[13]);   /* top */
  f.planes[4] = normalised(m[3] + m[2], m[7] + m[6], m[11] + m[10], m[15] + m[14]);  /* near */
  f.planes[5] = normalised(m[3] - m[2], m[7] - m[6], m[11] - m[10], m[15] - m[14]);  /* far */
  return f;
}

bool frustum_intersects_aabb(frustum_t f, aabb_t box) {
  for (int i = 0; i < 6; i++) {
    vec3_t n = f.planes[i].normal;
    /* The corner furthest along the normal; if it is outside, all are. */
    vec3_t far_corner = vec3(n.x >= 0 ? box.max.x : box.min.x, n.y >= 0 ? box.max.y : box.min.y,
                             n.z >= 0 ? box.max.z : box.min.z);
    if (plane_distance(f.planes[i], far_corner) < 0) return false;
  }
  return true;
}

bool frustum_intersects_sphere(frustum_t f, vec3_t center, float radius) {
  for (int i = 0; i < 6; i++)
    if (plane_distance(f.planes[i], center) < -radius) return false;
  return true;
}
