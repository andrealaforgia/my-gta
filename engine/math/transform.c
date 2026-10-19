#include "transform.h"

transform_t transform_identity(void) {
  transform_t t = {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
  return t;
}

transform_t transform_at(vec3_t position, quat_t rotation) {
  transform_t t = transform_identity();
  t.position = position;
  t.rotation = rotation;
  return t;
}

mat4_t transform_to_matrix(transform_t t) {
  return mat4_multiply(mat4_translation(t.position),
                       mat4_multiply(mat4_from_quat(t.rotation), mat4_scaling(t.scale)));
}

vec3_t transform_point(transform_t t, vec3_t local) {
  return vec3_add(t.position, quat_rotate(t.rotation, vec3_multiply(local, t.scale)));
}

vec3_t transform_direction(transform_t t, vec3_t local) {
  return quat_rotate(t.rotation, local);
}

vec3_t transform_inverse_point(transform_t t, vec3_t world) {
  vec3_t rotated = quat_rotate(quat_conjugate(t.rotation), vec3_sub(world, t.position));
  return vec3(rotated.x / t.scale.x, rotated.y / t.scale.y, rotated.z / t.scale.z);
}

transform_t transform_lerp(transform_t a, transform_t b, float t) {
  transform_t r;
  r.position = vec3_lerp(a.position, b.position, t);
  r.rotation = quat_nlerp(a.rotation, b.rotation, t);
  r.scale = vec3_lerp(a.scale, b.scale, t);
  return r;
}
