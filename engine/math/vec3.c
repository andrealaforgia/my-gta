#include "vec3.h"

#include <math.h>

vec3_t vec3(float x, float y, float z) {
  vec3_t v = {x, y, z};
  return v;
}

vec3_t vec3_zero(void) { return vec3(0, 0, 0); }

vec3_t vec3_add(vec3_t a, vec3_t b) {
  return vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

vec3_t vec3_sub(vec3_t a, vec3_t b) {
  return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

vec3_t vec3_scale(vec3_t a, float factor) {
  return vec3(a.x * factor, a.y * factor, a.z * factor);
}

vec3_t vec3_negate(vec3_t a) { return vec3(-a.x, -a.y, -a.z); }

vec3_t vec3_multiply(vec3_t a, vec3_t b) {
  return vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

float vec3_dot(vec3_t a, vec3_t b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

vec3_t vec3_cross(vec3_t a, vec3_t b) {
  return vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.x);
}

float vec3_length_squared(vec3_t a) { return vec3_dot(a, a); }

float vec3_length(vec3_t a) { return sqrtf(vec3_length_squared(a)); }

float vec3_distance(vec3_t a, vec3_t b) { return vec3_length(vec3_sub(a, b)); }

vec3_t vec3_normalize(vec3_t a) {
  float length = vec3_length(a);
  if (length <= 1e-12f) return vec3_zero();
  return vec3_scale(a, 1.0f / length);
}

vec3_t vec3_lerp(vec3_t a, vec3_t b, float t) {
  return vec3_add(a, vec3_scale(vec3_sub(b, a), t));
}

vec3_t vec3_min(vec3_t a, vec3_t b) {
  return vec3(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
}

vec3_t vec3_max(vec3_t a, vec3_t b) {
  return vec3(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
}

vec3_t vec3_add_scaled(vec3_t a, vec3_t b, float factor) {
  return vec3_add(a, vec3_scale(b, factor));
}
