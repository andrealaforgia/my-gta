#include "mat4.h"

#include <math.h>

#define AT(a, column, row) ((a).m[(column) * 4 + (row)])

mat4_t mat4_identity(void) {
  mat4_t r = {{0}};
  r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1;
  return r;
}

mat4_t mat4_multiply(mat4_t a, mat4_t b) {
  mat4_t r;
  for (int column = 0; column < 4; column++) {
    for (int row = 0; row < 4; row++) {
      float sum = 0;
      for (int k = 0; k < 4; k++) sum += AT(a, k, row) * AT(b, column, k);
      AT(r, column, row) = sum;
    }
  }
  return r;
}

mat4_t mat4_translation(vec3_t t) {
  mat4_t r = mat4_identity();
  r.m[12] = t.x;
  r.m[13] = t.y;
  r.m[14] = t.z;
  return r;
}

mat4_t mat4_scaling(vec3_t s) {
  mat4_t r = mat4_identity();
  r.m[0] = s.x;
  r.m[5] = s.y;
  r.m[10] = s.z;
  return r;
}

mat4_t mat4_from_quat(quat_t q) {
  mat4_t r = mat4_identity();
  float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
  float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
  float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
  AT(r, 0, 0) = 1 - 2 * (yy + zz);
  AT(r, 0, 1) = 2 * (xy + wz);
  AT(r, 0, 2) = 2 * (xz - wy);
  AT(r, 1, 0) = 2 * (xy - wz);
  AT(r, 1, 1) = 1 - 2 * (xx + zz);
  AT(r, 1, 2) = 2 * (yz + wx);
  AT(r, 2, 0) = 2 * (xz + wy);
  AT(r, 2, 1) = 2 * (yz - wx);
  AT(r, 2, 2) = 1 - 2 * (xx + yy);
  return r;
}

mat4_t mat4_transpose(mat4_t a) {
  mat4_t r;
  for (int column = 0; column < 4; column++)
    for (int row = 0; row < 4; row++) AT(r, column, row) = AT(a, row, column);
  return r;
}

mat4_t mat4_inverse_rigid(mat4_t a) {
  mat4_t r = mat4_identity();
  for (int column = 0; column < 3; column++)
    for (int row = 0; row < 3; row++) AT(r, column, row) = AT(a, row, column);
  vec3_t t = vec3(a.m[12], a.m[13], a.m[14]);
  vec3_t rotated = mat4_transform_direction(r, t);
  r.m[12] = -rotated.x;
  r.m[13] = -rotated.y;
  r.m[14] = -rotated.z;
  return r;
}

mat4_t mat4_inverse(mat4_t a) {
  const float* m = a.m;
  float inv[16];
  inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] +
           m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
  inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] -
           m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
  inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] +
           m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
  inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] -
            m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
  inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] -
           m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
  inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] +
           m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
  inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] -
           m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
  inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] +
            m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
  inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] +
           m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
  inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] -
           m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
  inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] +
            m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
  inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] -
            m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
  inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] -
           m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
  inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] +
           m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
  inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] -
            m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
  inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] +
            m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];
  float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
  if (fabsf(det) < 1e-12f) return mat4_identity();
  mat4_t r;
  for (int i = 0; i < 16; i++) r.m[i] = inv[i] / det;
  return r;
}

mat4_t mat4_perspective(float fov_y, float aspect, float near_plane,
                        float far_plane) {
  mat4_t r = {{0}};
  float f = 1.0f / tanf(fov_y * 0.5f);
  r.m[0] = f / aspect;
  r.m[5] = f;
  r.m[10] = (far_plane + near_plane) / (near_plane - far_plane);
  r.m[11] = -1;
  r.m[14] = (2 * far_plane * near_plane) / (near_plane - far_plane);
  return r;
}

mat4_t mat4_orthographic(float left, float right, float bottom, float top,
                         float near_plane, float far_plane) {
  mat4_t r = mat4_identity();
  r.m[0] = 2.0f / (right - left);
  r.m[5] = 2.0f / (top - bottom);
  r.m[10] = -2.0f / (far_plane - near_plane);
  r.m[12] = -(right + left) / (right - left);
  r.m[13] = -(top + bottom) / (top - bottom);
  r.m[14] = -(far_plane + near_plane) / (far_plane - near_plane);
  return r;
}

mat4_t mat4_look_at(vec3_t eye, vec3_t target, vec3_t up) {
  vec3_t forward = vec3_normalize(vec3_sub(target, eye));
  vec3_t side = vec3_normalize(vec3_cross(forward, up));
  vec3_t true_up = vec3_cross(side, forward);
  mat4_t r = mat4_identity();
  AT(r, 0, 0) = side.x;
  AT(r, 1, 0) = side.y;
  AT(r, 2, 0) = side.z;
  AT(r, 0, 1) = true_up.x;
  AT(r, 1, 1) = true_up.y;
  AT(r, 2, 1) = true_up.z;
  AT(r, 0, 2) = -forward.x;
  AT(r, 1, 2) = -forward.y;
  AT(r, 2, 2) = -forward.z;
  r.m[12] = -vec3_dot(side, eye);
  r.m[13] = -vec3_dot(true_up, eye);
  r.m[14] = vec3_dot(forward, eye);
  return r;
}

vec3_t mat4_transform_point(mat4_t a, vec3_t p) {
  return vec3(
      AT(a, 0, 0) * p.x + AT(a, 1, 0) * p.y + AT(a, 2, 0) * p.z + AT(a, 3, 0),
      AT(a, 0, 1) * p.x + AT(a, 1, 1) * p.y + AT(a, 2, 1) * p.z + AT(a, 3, 1),
      AT(a, 0, 2) * p.x + AT(a, 1, 2) * p.y + AT(a, 2, 2) * p.z + AT(a, 3, 2));
}

vec3_t mat4_transform_direction(mat4_t a, vec3_t d) {
  return vec3(AT(a, 0, 0) * d.x + AT(a, 1, 0) * d.y + AT(a, 2, 0) * d.z,
              AT(a, 0, 1) * d.x + AT(a, 1, 1) * d.y + AT(a, 2, 1) * d.z,
              AT(a, 0, 2) * d.x + AT(a, 1, 2) * d.y + AT(a, 2, 2) * d.z);
}

vec3_t mat4_project_point(mat4_t a, vec3_t p) {
  vec3_t t = mat4_transform_point(a, p);
  float w =
      AT(a, 0, 3) * p.x + AT(a, 1, 3) * p.y + AT(a, 2, 3) * p.z + AT(a, 3, 3);
  if (fabsf(w) < 1e-12f) return t;
  return vec3_scale(t, 1.0f / w);
}
