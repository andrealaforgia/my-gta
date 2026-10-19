#include <assert.h>

#include "aabb.h"
#include "frustum.h"
#include "interpolation.h"
#include "mat4.h"
#include "quat.h"
#include "ray.h"
#include "test_groups.h"
#include "test_support.h"
#include "transform.h"
#include "vec3.h"

static bool near_vec(vec3_t a, vec3_t b) {
  return near_to(a.x, b.x, 1e-5) && near_to(a.y, b.y, 1e-5) &&
         near_to(a.z, b.z, 1e-5);
}

/* ---- vec3 ---- */

static void vectors_add_subtract_scale_and_measure(void) {
  vec3_t a = vec3(1, 2, 3);
  vec3_t b = vec3(3, 6, 9);
  assert(near_vec(vec3_add(a, b), vec3(4, 8, 12)));
  assert(near_vec(vec3_sub(b, a), vec3(2, 4, 6)));
  assert(near_vec(vec3_scale(a, 2), vec3(2, 4, 6)));
  assert(near(vec3_dot(a, b), 3 + 12 + 27));
  assert(near(vec3_length(vec3(3, 0, 4)), 5));
  assert(near(vec3_distance(a, b), sqrt(4 + 16 + 36)));
  assert(near_vec(vec3_lerp(a, b, 0.5f), vec3(2, 4, 6)));
}

static void cross_product_is_right_handed(void) {
  vec3_t east = vec3(1, 0, 0);
  vec3_t up = vec3(0, 1, 0);
  vec3_t north = vec3(0, 0, -1);
  assert(near_vec(vec3_cross(east, up), vec3(0, 0, 1)));
  /* forward x up = right */
  assert(near_vec(vec3_cross(north, up), east));
  assert(near_vec(vec3_cross(up, north), vec3(-1, 0, 0)));
}

static void normalising_zero_gives_zero_not_nan(void) {
  vec3_t n = vec3_normalize(vec3(0, 0, 0));
  assert(near_vec(n, vec3(0, 0, 0)));
  assert(near(vec3_length(vec3_normalize(vec3(0, 7, 0))), 1));
}

/* ---- quat ---- */

static void identity_rotation_leaves_vectors_alone(void) {
  assert(near_vec(quat_rotate(quat_identity(), vec3(1, 2, 3)), vec3(1, 2, 3)));
}

static void positive_yaw_turns_forward_towards_the_left(void) {
  /* Forward is -Z. A quarter turn about +Y (anticlockwise seen from above)
   * takes forward to -X, which is to the left of a car facing -Z. */
  quat_t q = quat_from_axis_angle(vec3(0, 1, 0), (float)M_PI / 2);
  assert(near_vec(quat_rotate(q, vec3(0, 0, -1)), vec3(-1, 0, 0)));
  assert(near_vec(quat_forward(q), vec3(-1, 0, 0)));
  assert(near_vec(quat_right(q), vec3(0, 0, -1)));
  assert(near_vec(quat_up(q), vec3(0, 1, 0)));
}

static void quaternion_products_compose_rotations(void) {
  quat_t yaw = quat_from_axis_angle(vec3(0, 1, 0), (float)M_PI / 2);
  quat_t twice = quat_multiply(yaw, yaw);
  assert(near_vec(quat_rotate(twice, vec3(0, 0, -1)), vec3(0, 0, 1)));
  quat_t back = quat_multiply(yaw, quat_conjugate(yaw));
  assert(near_vec(quat_rotate(back, vec3(1, 2, 3)), vec3(1, 2, 3)));
}

static void integrating_angular_velocity_turns_the_orientation(void) {
  quat_t q = quat_identity();
  vec3_t omega = vec3(0, (float)M_PI, 0); /* half a turn per second */
  for (int i = 0; i < 100; i++) q = quat_integrate(q, omega, 0.005f);
  assert(near_to(vec3_length(vec3(q.x, q.y, q.z)), sinf((float)M_PI / 4), 1e-3));
  vec3_t forward = quat_forward(q);
  assert(near_to(forward.x, -1, 1e-3) && near_to(forward.z, 0, 1e-3));
}

static void yaw_can_be_read_back_from_a_quaternion(void) {
  quat_t q = quat_from_axis_angle(vec3(0, 1, 0), 0.7f);
  assert(near_to(quat_yaw(q), 0.7f, 1e-5));
}

/* ---- mat4 ---- */

static void matrices_are_column_major(void) {
  mat4_t t = mat4_translation(vec3(5, 6, 7));
  assert(near(t.m[12], 5) && near(t.m[13], 6) && near(t.m[14], 7));
  assert(near(t.m[15], 1));
  assert(near_vec(mat4_transform_point(t, vec3(1, 1, 1)), vec3(6, 7, 8)));
  assert(near_vec(mat4_transform_direction(t, vec3(1, 1, 1)), vec3(1, 1, 1)));
}

static void matrix_product_applies_the_right_factor_first(void) {
  mat4_t translate = mat4_translation(vec3(10, 0, 0));
  mat4_t scale = mat4_scaling(vec3(2, 2, 2));
  mat4_t m = mat4_multiply(translate, scale);
  assert(near_vec(mat4_transform_point(m, vec3(1, 0, 0)), vec3(12, 0, 0)));
}

static void rotation_matrix_matches_the_quaternion(void) {
  quat_t q = quat_from_axis_angle(vec3(0, 1, 0), 0.4f);
  mat4_t r = mat4_from_quat(q);
  vec3_t v = vec3(1, 2, 3);
  assert(near_vec(mat4_transform_point(r, v), quat_rotate(q, v)));
}

static void rigid_inverse_undoes_the_transform(void) {
  transform_t t = {vec3(1, 2, 3), quat_from_axis_angle(vec3(1, 0, 0), 0.3f),
                   vec3(1, 1, 1)};
  mat4_t m = transform_to_matrix(t);
  mat4_t inverse = mat4_inverse_rigid(m);
  vec3_t p = vec3(4, 5, 6);
  assert(near_vec(mat4_transform_point(inverse, mat4_transform_point(m, p)), p));
}

static void general_inverse_undoes_scale_too(void) {
  transform_t t = {vec3(1, 2, 3), quat_from_axis_angle(vec3(0, 1, 0), 0.3f),
                   vec3(2, 3, 4)};
  mat4_t m = transform_to_matrix(t);
  mat4_t inverse = mat4_inverse(m);
  vec3_t p = vec3(4, 5, 6);
  assert(near_vec(mat4_transform_point(inverse, mat4_transform_point(m, p)), p));
}

static void perspective_maps_near_to_minus_one_and_far_to_one(void) {
  mat4_t p = mat4_perspective(1.0f, 16.0f / 9.0f, 0.5f, 100.0f);
  vec3_t near_point = mat4_project_point(p, vec3(0, 0, -0.5f));
  vec3_t far_point = mat4_project_point(p, vec3(0, 0, -100.0f));
  assert(near_to(near_point.z, -1, 1e-4));
  assert(near_to(far_point.z, 1, 1e-4));
  vec3_t right = mat4_project_point(p, vec3(1, 0, -10));
  vec3_t up = mat4_project_point(p, vec3(0, 1, -10));
  assert(right.x > 0 && near(right.y, 0));
  assert(up.y > 0 && near(up.x, 0));
  assert(near_to(up.y, right.x * 16.0f / 9.0f, 1e-4));
}

static void look_at_faces_the_target_down_minus_z(void) {
  mat4_t view = mat4_look_at(vec3(0, 0, 10), vec3(0, 0, 0), vec3(0, 1, 0));
  assert(near_vec(mat4_transform_point(view, vec3(0, 0, 0)), vec3(0, 0, -10)));
  assert(near_vec(mat4_transform_point(view, vec3(1, 0, 10)), vec3(1, 0, 0)));
  mat4_t from_east = mat4_look_at(vec3(10, 0, 0), vec3(0, 0, 0), vec3(0, 1, 0));
  assert(near_vec(mat4_transform_point(from_east, vec3(0, 0, 0)), vec3(0, 0, -10)));
  /* Looking west from the east, north (-Z) is on the right. */
  assert(near_vec(mat4_transform_point(from_east, vec3(10, 0, -1)), vec3(1, 0, 0)));
}

/* ---- transform ---- */

static void transforms_apply_scale_then_rotation_then_translation(void) {
  transform_t t = {vec3(10, 0, 0), quat_from_axis_angle(vec3(0, 1, 0), (float)M_PI / 2),
                   vec3(2, 2, 2)};
  assert(near_vec(transform_point(t, vec3(0, 0, -1)), vec3(8, 0, 0)));
  assert(near_vec(transform_inverse_point(t, vec3(8, 0, 0)), vec3(0, 0, -1)));
}

static void transforms_interpolate_position_and_rotation(void) {
  transform_t a = {vec3(0, 0, 0), quat_identity(), vec3(1, 1, 1)};
  transform_t b = {vec3(2, 0, 0), quat_from_axis_angle(vec3(0, 1, 0), 1.0f),
                   vec3(1, 1, 1)};
  transform_t mid = transform_lerp(a, b, 0.5f);
  assert(near_vec(mid.position, vec3(1, 0, 0)));
  assert(near_to(quat_yaw(mid.rotation), 0.5f, 1e-4));
}

/* ---- aabb ---- */

static void boxes_grow_to_contain_points_and_test_overlap(void) {
  aabb_t box = aabb_empty();
  box = aabb_include(box, vec3(1, 2, 3));
  box = aabb_include(box, vec3(-1, 0, 5));
  assert(near_vec(box.min, vec3(-1, 0, 3)) && near_vec(box.max, vec3(1, 2, 5)));
  assert(aabb_contains(box, vec3(0, 1, 4)));
  assert(!aabb_contains(box, vec3(0, 3, 4)));
  aabb_t other = aabb_from_center(vec3(1, 2, 5), vec3(0.5f, 0.5f, 0.5f));
  assert(aabb_overlaps(box, other));
  assert(!aabb_overlaps(box, aabb_from_center(vec3(5, 5, 5), vec3(1, 1, 1))));
  assert(near_vec(aabb_center(box), vec3(0, 1, 4)));
}

static void boxes_transform_to_a_bounding_box_of_their_corners(void) {
  aabb_t unit = aabb_from_center(vec3(0, 0, 0), vec3(1, 0.5f, 2));
  transform_t t = {vec3(5, 0, 0), quat_from_axis_angle(vec3(0, 1, 0), (float)M_PI / 2),
                   vec3(1, 1, 1)};
  aabb_t world = aabb_transform(unit, t);
  assert(near_vec(world.min, vec3(3, -0.5f, -1)));
  assert(near_vec(world.max, vec3(7, 0.5f, 1)));
}

/* ---- rays and planes ---- */

static void ray_hits_a_triangle_in_front_but_not_behind(void) {
  vec3_t a = vec3(-1, 0, -1), b = vec3(1, 0, -1), c = vec3(0, 0, 1);
  ray_t down = {vec3(0, 5, 0), vec3(0, -1, 0)};
  ray_hit_t hit;
  assert(ray_triangle(down, a, b, c, &hit));
  assert(near(hit.distance, 5));
  assert(near_vec(hit.point, vec3(0, 0, 0)));
  assert(near_vec(hit.normal, vec3(0, 1, 0)));
  ray_t up = {vec3(0, 5, 0), vec3(0, 1, 0)};
  assert(!ray_triangle(up, a, b, c, &hit));
  ray_t beside = {vec3(5, 5, 0), vec3(0, -1, 0)};
  assert(!ray_triangle(beside, a, b, c, &hit));
}

static void ray_hits_a_box_from_outside_and_from_inside(void) {
  aabb_t box = aabb_from_center(vec3(0, 0, 0), vec3(1, 1, 1));
  ray_t r = {vec3(-5, 0, 0), vec3(1, 0, 0)};
  float t_min, t_max;
  assert(ray_aabb(r, box, &t_min, &t_max));
  assert(near(t_min, 4) && near(t_max, 6));
  ray_t inside = {vec3(0, 0, 0), vec3(0, 1, 0)};
  assert(ray_aabb(inside, box, &t_min, &t_max));
  assert(t_min <= 0 && near(t_max, 1));
  ray_t miss = {vec3(-5, 3, 0), vec3(1, 0, 0)};
  assert(!ray_aabb(miss, box, &t_min, &t_max));
}

static void planes_measure_signed_distance(void) {
  plane_t floor = plane_from_point_normal(vec3(0, 2, 0), vec3(0, 1, 0));
  assert(near(plane_distance(floor, vec3(7, 5, 7)), 3));
  assert(near(plane_distance(floor, vec3(7, -1, 7)), -3));
  plane_t p = plane_from_points(vec3(0, 0, 0), vec3(1, 0, 0), vec3(0, 0, -1));
  assert(near_vec(p.normal, vec3(0, 1, 0)));
}

/* ---- frustum ---- */

static void frustum_keeps_what_the_camera_can_see(void) {
  mat4_t view = mat4_look_at(vec3(0, 0, 0), vec3(0, 0, -1), vec3(0, 1, 0));
  mat4_t proj = mat4_perspective(1.0f, 1.0f, 0.1f, 100.0f);
  frustum_t f = frustum_from_matrix(mat4_multiply(proj, view));
  assert(frustum_intersects_aabb(f, aabb_from_center(vec3(0, 0, -10), vec3(1, 1, 1))));
  assert(!frustum_intersects_aabb(f, aabb_from_center(vec3(0, 0, 10), vec3(1, 1, 1))));
  assert(!frustum_intersects_aabb(f, aabb_from_center(vec3(0, 0, -200), vec3(1, 1, 1))));
  assert(!frustum_intersects_aabb(f, aabb_from_center(vec3(50, 0, -10), vec3(1, 1, 1))));
  assert(frustum_intersects_aabb(f, aabb_from_center(vec3(50, 0, -10), vec3(49, 1, 1))));
  assert(frustum_intersects_sphere(f, vec3(0, 0, -10), 1));
  assert(!frustum_intersects_sphere(f, vec3(0, 0, 10), 1));
}

/* ---- interpolation ---- */

static void damping_approaches_the_target_without_overshoot(void) {
  float value = 0;
  for (int i = 0; i < 100; i++) value = damp_towards(value, 10, 5.0f, 0.016f);
  assert(value > 9.9f && value <= 10.0f);
  assert(near(clampf(5, 0, 1), 1) && near(clampf(-5, 0, 1), 0));
  assert(near(move_towards(0, 10, 3), 3) && near(move_towards(9, 10, 3), 10));
  assert(near(lerpf(0, 10, 0.25f), 2.5f));
}

void run_math_tests(void) {
  RUN(vectors_add_subtract_scale_and_measure);
  RUN(cross_product_is_right_handed);
  RUN(normalising_zero_gives_zero_not_nan);
  RUN(identity_rotation_leaves_vectors_alone);
  RUN(positive_yaw_turns_forward_towards_the_left);
  RUN(quaternion_products_compose_rotations);
  RUN(integrating_angular_velocity_turns_the_orientation);
  RUN(yaw_can_be_read_back_from_a_quaternion);
  RUN(matrices_are_column_major);
  RUN(matrix_product_applies_the_right_factor_first);
  RUN(rotation_matrix_matches_the_quaternion);
  RUN(rigid_inverse_undoes_the_transform);
  RUN(general_inverse_undoes_scale_too);
  RUN(perspective_maps_near_to_minus_one_and_far_to_one);
  RUN(look_at_faces_the_target_down_minus_z);
  RUN(transforms_apply_scale_then_rotation_then_translation);
  RUN(transforms_interpolate_position_and_rotation);
  RUN(boxes_grow_to_contain_points_and_test_overlap);
  RUN(boxes_transform_to_a_bounding_box_of_their_corners);
  RUN(ray_hits_a_triangle_in_front_but_not_behind);
  RUN(ray_hits_a_box_from_outside_and_from_inside);
  RUN(planes_measure_signed_distance);
  RUN(frustum_keeps_what_the_camera_can_see);
  RUN(damping_approaches_the_target_without_overshoot);
}
