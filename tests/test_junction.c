#include <assert.h>
#include <math.h>

#include "junction_geometry.h"
#include "test_groups.h"
#include "test_support.h"

/* Point-in-polygon on (x, z) by ray crossing. */
static bool inside(const local_point_t* polygon, size_t count, double x,
                   double z) {
  bool in = false;
  for (size_t i = 0, j = count - 1; i < count; j = i++) {
    bool crosses = (polygon[i].z > z) != (polygon[j].z > z);
    if (!crosses) continue;
    double at = (polygon[j].x - polygon[i].x) * (z - polygon[i].z) /
                    (polygon[j].z - polygon[i].z) +
                polygon[i].x;
    if (x < at) in = !in;
  }
  return in;
}

static junction_arm_t arm(double dx, double dz, double half_width,
                          double pavement) {
  junction_arm_t a = {{dx, dz}, half_width, pavement};
  return a;
}

static local_point_t origin(void) {
  local_point_t p = {0, 0};
  return p;
}

static void a_crossroads_is_filled_to_its_corners_and_roads_cut_back(void) {
  /* East, north (-z), west, south, all 6 m wide with 2 m pavements. */
  junction_arm_t arms[4] = {arm(1, 0, 3, 2), arm(0, -1, 3, 2), arm(-1, 0, 3, 2),
                            arm(0, 1, 3, 2)};
  junction_layout_t layout;
  assert(junction_layout(origin(), arms, 4, 0.2, &layout));
  for (int i = 0; i < 4; i++) assert(near_to(layout.cut[i], 3.2, 1e-6));
  assert(layout.polygon_count >= 8);
  assert(inside(layout.polygon, layout.polygon_count, 2.9, 2.9));
  assert(inside(layout.polygon, layout.polygon_count, -2.9, -2.9));
  assert(inside(layout.polygon, layout.polygon_count, 3.1, 0));
  assert(!inside(layout.polygon, layout.polygon_count, 4.0, 0));
  assert(!inside(layout.polygon, layout.polygon_count, 3.5, 3.5));
  assert(layout.corner_count == 4);
  /* The pavement corner between east and north covers the outside of
   * the road corner. */
  bool covered = false;
  for (size_t c = 0; c < layout.corner_count; c++)
    if (inside(layout.corners[c].polygon, layout.corners[c].count, 4.0, -4.0))
      covered = true;
  assert(covered);
}

static void a_t_junction_keeps_a_straight_far_edge(void) {
  junction_arm_t arms[3] = {arm(1, 0, 3, 2), arm(0, -1, 3, 2),
                            arm(-1, 0, 3, 2)};
  junction_layout_t layout;
  assert(junction_layout(origin(), arms, 3, 0.2, &layout));
  assert(near_to(layout.cut[0], 3.2, 1e-6) &&
         near_to(layout.cut[2], 3.2, 1e-6));
  assert(near_to(layout.cut[1], 3.2, 1e-6));
  assert(inside(layout.polygon, layout.polygon_count, 0, 2.9));
  assert(!inside(layout.polygon, layout.polygon_count, 0, 3.2));
  assert(inside(layout.polygon, layout.polygon_count, 2.9, -2.9));
  assert(layout.corner_count == 2); /* no pavement corner across the bar */
}

static void a_right_angle_bend_fills_the_outside_corner(void) {
  junction_arm_t arms[2] = {arm(1, 0, 3, 2), arm(0, -1, 3, 2)};
  junction_layout_t layout;
  assert(junction_layout(origin(), arms, 2, 0.2, &layout));
  assert(near_to(layout.cut[0], 3.2, 1e-6) &&
         near_to(layout.cut[1], 3.2, 1e-6));
  assert(inside(layout.polygon, layout.polygon_count, -2.5,
                2.5)); /* outer corner */
  assert(inside(layout.polygon, layout.polygon_count, 2.5,
                -2.5)); /* inner corner */
}

static void a_straight_continuation_needs_no_patch(void) {
  junction_arm_t arms[2] = {arm(1, 0, 3, 2), arm(-1, 0, 3, 2)};
  junction_layout_t layout;
  assert(!junction_layout(origin(), arms, 2, 0.2, &layout));
  assert(near(layout.cut[0], 0) && near(layout.cut[1], 0));
}

static void a_shallow_bend_is_bevelled_not_spiked(void) {
  double a = 10.0 * M_PI / 180.0;
  junction_arm_t arms[2] = {arm(1, 0, 3, 2), arm(-cos(a), -sin(a), 3, 2)};
  junction_layout_t layout;
  assert(junction_layout(origin(), arms, 2, 0.2, &layout));
  for (size_t i = 0; i < layout.polygon_count; i++)
    assert(hypot(layout.polygon[i].x, layout.polygon[i].z) < 8.0);
  assert(layout.cut[0] < 2.0 && layout.cut[1] < 2.0);
}

static void a_fork_is_cut_back_until_the_ribbons_separate(void) {
  /* Two 6 m roads leaving 15 degrees apart, as the carriageways of a
   * dual road do: their edges only clear each other 23 m out, so that is
   * where both must be cut, and the patch fills the wedge between. */
  double a = 15.0 * M_PI / 180.0;
  junction_arm_t arms[3] = {arm(1, 0, 3, 2), arm(cos(a), -sin(a), 3, 2),
                            arm(-1, 0, 3, 2)};
  junction_layout_t layout;
  assert(junction_layout(origin(), arms, 3, 0.2, &layout));
  /* The left edge of one arm meets the right edge of the other where
   * s sin(a) - w cos(a) = w, so s = w (1 + cos a) / sin a. */
  double expected = 3.0 * (1.0 + cos(a)) / sin(a) + 0.2;
  assert(near_to(layout.cut[0], expected, 1e-3) &&
         near_to(layout.cut[1], expected, 1e-3));
  assert(inside(layout.polygon, layout.polygon_count, 10.0,
                -1.3)); /* between the arms */
  assert(inside(layout.polygon, layout.polygon_count, 20.0, -2.6));
  assert(!inside(layout.polygon, layout.polygon_count, 10.0, -8.0));
}

static void sharp_wedges_get_no_pavement_corner_over_the_road(void) {
  /* A road leaving 40 degrees off another, as at the foot of a slip road:
   * the pavement wedge between them would lie across the road, so none
   * is laid, and no corner covers any part of an arm's carriageway. */
  double a = 40.0 * M_PI / 180.0;
  junction_arm_t arms[3] = {arm(1, 0, 3, 2), arm(cos(a), -sin(a), 3, 2),
                            arm(-1, 0, 3, 2)};
  junction_layout_t layout;
  assert(junction_layout(origin(), arms, 3, 0.2, &layout));
  for (size_t c = 0; c < layout.corner_count; c++) {
    for (double along = layout.cut[0] + 0.5; along < layout.cut[0] + 8.0;
         along += 1.0)
      for (double across = -2.5; across <= 2.5; across += 1.0)
        assert(!inside(layout.corners[c].polygon, layout.corners[c].count,
                       along, across));
    for (double along = layout.cut[1] + 0.5; along < layout.cut[1] + 8.0;
         along += 1.0)
      for (double across = -2.5; across <= 2.5; across += 1.0)
        assert(!inside(layout.corners[c].polygon, layout.corners[c].count,
                       along * cos(a) + across * sin(a),
                       -along * sin(a) + across * cos(a)));
  }
}

static void different_widths_meet_cleanly(void) {
  junction_arm_t arms[3] = {arm(1, 0, 5, 2), arm(0, -1, 2, 1),
                            arm(-1, 0, 5, 2)};
  junction_layout_t layout;
  assert(junction_layout(origin(), arms, 3, 0.2, &layout));
  assert(near_to(layout.cut[1], 5.2,
                 1e-6)); /* the narrow road clears the wide one */
  assert(near_to(layout.cut[0], 2.2, 1e-6));
  assert(inside(layout.polygon, layout.polygon_count, 0, -4.9));
  assert(inside(layout.polygon, layout.polygon_count, 2.1, 0));
  assert(!inside(layout.polygon, layout.polygon_count, 4.9,
                 0)); /* ribbon, not patch */
}

static void polylines_cut_from_both_ends_across_vertices(void) {
  local_point_t points[4] = {{0, 0}, {10, 0}, {10, -10}, {20, -10}};
  local_point_t out[6];
  size_t count = polyline_cut(points, 4, 12.0, 3.0, out);
  assert(count == 3);
  assert(near(out[0].x, 10) && near(out[0].z, -2));
  assert(near(out[1].x, 10) && near(out[1].z, -10));
  assert(near(out[2].x, 17) && near(out[2].z, -10));
  count = polyline_cut(points, 4, 2.0, 2.0, out);
  assert(count == 4 && near(out[0].x, 2) && near(out[3].x, 18));
  /* Cuts longer than the line leave a short stub rather than nothing. */
  count = polyline_cut(points, 4, 20.0, 20.0, out);
  assert(count == 2 && polyline_length(out, 2) > 0.99);
}

void run_junction_tests(void) {
  RUN(a_crossroads_is_filled_to_its_corners_and_roads_cut_back);
  RUN(a_t_junction_keeps_a_straight_far_edge);
  RUN(a_right_angle_bend_fills_the_outside_corner);
  RUN(a_straight_continuation_needs_no_patch);
  RUN(a_shallow_bend_is_bevelled_not_spiked);
  RUN(a_fork_is_cut_back_until_the_ribbons_separate);
  RUN(sharp_wedges_get_no_pavement_corner_over_the_road);
  RUN(different_widths_meet_cleanly);
  RUN(polylines_cut_from_both_ends_across_vertices);
}
