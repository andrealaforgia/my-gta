#include "quat.h"

#include <math.h>

quat_t quat(float x, float y, float z, float w) {
  quat_t q = {x, y, z, w};
  return q;
}

quat_t quat_identity(void) { return quat(0, 0, 0, 1); }

quat_t quat_from_axis_angle(vec3_t axis, float radians) {
  vec3_t n = vec3_normalize(axis);
  float half = radians * 0.5f;
  float s = sinf(half);
  return quat(n.x * s, n.y * s, n.z * s, cosf(half));
}

quat_t quat_from_yaw(float radians) {
  return quat_from_axis_angle(vec3(0, 1, 0), radians);
}

quat_t quat_multiply(quat_t a, quat_t b) {
  return quat(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
              a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
              a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
              a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

quat_t quat_conjugate(quat_t q) { return quat(-q.x, -q.y, -q.z, q.w); }

quat_t quat_normalize(quat_t q) {
  float length = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (length <= 1e-12f) return quat_identity();
  return quat(q.x / length, q.y / length, q.z / length, q.w / length);
}

vec3_t quat_rotate(quat_t q, vec3_t v) {
  /* v' = v + 2w(u x v) + 2(u x (u x v)) with u = (x, y, z). */
  vec3_t u = vec3(q.x, q.y, q.z);
  vec3_t t = vec3_scale(vec3_cross(u, v), 2.0f);
  return vec3_add(vec3_add(v, vec3_scale(t, q.w)), vec3_cross(u, t));
}

vec3_t quat_forward(quat_t q) { return quat_rotate(q, vec3(0, 0, -1)); }
vec3_t quat_right(quat_t q) { return quat_rotate(q, vec3(1, 0, 0)); }
vec3_t quat_up(quat_t q) { return quat_rotate(q, vec3(0, 1, 0)); }

float quat_yaw(quat_t q) {
  vec3_t f = quat_forward(q);
  return atan2f(-f.x, -f.z);
}

quat_t quat_integrate(quat_t q, vec3_t angular_velocity, float dt) {
  quat_t omega =
      quat(angular_velocity.x, angular_velocity.y, angular_velocity.z, 0);
  quat_t derivative = quat_multiply(omega, q);
  quat_t advanced =
      quat(q.x + derivative.x * 0.5f * dt, q.y + derivative.y * 0.5f * dt,
           q.z + derivative.z * 0.5f * dt, q.w + derivative.w * 0.5f * dt);
  return quat_normalize(advanced);
}

quat_t quat_nlerp(quat_t a, quat_t b, float t) {
  float cosine = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  if (cosine < 0) b = quat(-b.x, -b.y, -b.z, -b.w);
  return quat_normalize(quat(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                             a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t));
}
