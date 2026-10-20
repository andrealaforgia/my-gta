#include "junction_geometry.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PARALLEL_SINE 0.05
#define MITRE_LIMIT 2.5 /* times the larger half width */

static plane_vec_t pv(double x, double z) {
  plane_vec_t v = {x, z};
  return v;
}
static plane_vec_t right_of(plane_vec_t d) { return pv(-d.z, d.x); }
static plane_vec_t left_of(plane_vec_t d) { return pv(d.z, -d.x); }
static local_point_t point_at(local_point_t c, plane_vec_t d, double t) {
  local_point_t p = {c.x + d.x * t, c.z + d.z * t};
  return p;
}
static local_point_t shift(local_point_t p, plane_vec_t d, double t) {
  return point_at(p, d, t);
}
static double angle_of(plane_vec_t d) {
  return atan2(-d.z, d.x);
} /* x, north */

typedef struct {
  junction_arm_t arm;
  size_t original;
} sorted_arm_t;

static int by_angle(const void* a, const void* b) {
  double x = angle_of(((const sorted_arm_t*)a)->arm.direction);
  double y = angle_of(((const sorted_arm_t*)b)->arm.direction);
  return x < y ? -1 : (x > y ? 1 : 0);
}

/* Intersection of the line through a along u with the line through b along
 * v: a + u * t = b + v * s. Returns false when parallel. */
static bool intersect(local_point_t a, plane_vec_t u, local_point_t b,
                      plane_vec_t v, double* t, double* s) {
  double determinant = u.x * (-v.z) - u.z * (-v.x);
  if (fabs(determinant) < 1e-9) return false;
  double dx = b.x - a.x, dz = b.z - a.z;
  *t = (dx * (-v.z) - dz * (-v.x)) / determinant;
  *s = (u.x * dz - u.z * dx) / determinant;
  return true;
}

typedef struct {
  local_point_t points[2];
  size_t count;
  double t_along_first;  /* distance along arm i where its edge ends */
  double s_along_second; /* distance along arm i + 1 where its edge ends */
} corner_t;

/* The corner between arm i's left edge and arm j's right edge, at the
 * given offsets from the centre lines. */
static corner_t corner_between(local_point_t centre, junction_arm_t i,
                               double offset_i, junction_arm_t j,
                               double offset_j) {
  corner_t corner;
  local_point_t a = shift(centre, left_of(i.direction), offset_i);
  local_point_t b = shift(centre, right_of(j.direction), offset_j);
  double sine =
      fabs(i.direction.x * (-j.direction.z) - (-i.direction.z) * j.direction.x);
  double t, s;
  double limit = MITRE_LIMIT * fmax(offset_i, offset_j) + 1.0;
  bool crosses =
      sine > PARALLEL_SINE && intersect(a, i.direction, b, j.direction, &t, &s);
  /* Ahead of both arms (a fork) the corner is wherever the edges really
   * clear each other, however far; behind them (the outside of a bend)
   * a distant corner would be a spike, so it is bevelled. */
  bool fork = crosses && t > 0 && s > 0;
  if (crosses && (fork || (fabs(t) < limit && fabs(s) < limit))) {
    corner.points[0] = point_at(a, i.direction, t);
    corner.count = 1;
    corner.t_along_first = t;
    corner.s_along_second = s;
    return corner;
  }
  /* Parallel, or so sharp the mitre would spike: bevel between the two
   * edge points at the centre. */
  corner.points[0] = a;
  corner.points[1] = b;
  corner.count = 2;
  corner.t_along_first = 0;
  corner.s_along_second = 0;
  if (sine <= PARALLEL_SINE && fabs(offset_i - offset_j) < 1e-9) {
    corner.count = 1; /* the same point twice */
  }
  return corner;
}

static void add_point(local_point_t* polygon, size_t* count, size_t capacity,
                      local_point_t p) {
  if (*count < capacity) polygon[(*count)++] = p;
}

bool junction_layout(local_point_t centre, const junction_arm_t* arms,
                     size_t arm_count, double margin, junction_layout_t* out) {
  memset(out, 0, sizeof(*out));
  if (arm_count < 2 || arm_count > JUNCTION_MAX_ARMS) return false;
  sorted_arm_t sorted[JUNCTION_MAX_ARMS];
  for (size_t i = 0; i < arm_count; i++) {
    sorted[i].arm = arms[i];
    sorted[i].original = i;
  }
  qsort(sorted, arm_count, sizeof(sorted_arm_t), by_angle);

  corner_t road_corner[JUNCTION_MAX_ARMS];
  double cut[JUNCTION_MAX_ARMS] = {0};
  for (size_t i = 0; i < arm_count; i++) {
    size_t j = (i + 1) % arm_count;
    road_corner[i] =
        corner_between(centre, sorted[i].arm, sorted[i].arm.half_width,
                       sorted[j].arm, sorted[j].arm.half_width);
    if (road_corner[i].t_along_first > cut[i])
      cut[i] = road_corner[i].t_along_first;
    if (road_corner[i].s_along_second > cut[j])
      cut[j] = road_corner[i].s_along_second;
  }
  bool any_cut = false;
  for (size_t i = 0; i < arm_count; i++) {
    if (cut[i] > 1e-6) {
      cut[i] += margin;
      any_cut = true;
    }
    out->cut[sorted[i].original] = cut[i];
  }
  if (!any_cut) return false;

  /* The patch: each arm's end edge, right to left, then the corner to the
   * next arm anticlockwise. */
  for (size_t i = 0; i < arm_count; i++) {
    const junction_arm_t* a = &sorted[i].arm;
    local_point_t end = point_at(centre, a->direction, cut[i]);
    add_point(out->polygon, &out->polygon_count, JUNCTION_MAX_POLYGON,
              shift(end, right_of(a->direction), a->half_width));
    add_point(out->polygon, &out->polygon_count, JUNCTION_MAX_POLYGON,
              shift(end, left_of(a->direction), a->half_width));
    for (size_t k = 0; k < road_corner[i].count; k++)
      add_point(out->polygon, &out->polygon_count, JUNCTION_MAX_POLYGON,
                road_corner[i].points[k]);
  }
  for (size_t i = 0; i < out->polygon_count; i++) {
    double d =
        hypot(out->polygon[i].x - centre.x, out->polygon[i].z - centre.z);
    if (d > out->outer_radius) out->outer_radius = d;
  }

  /* Pavement corners between neighbouring arms that both have pavements
   * and are not a straight continuation of each other. */
  for (size_t i = 0; i < arm_count; i++) {
    size_t j = (i + 1) % arm_count;
    const junction_arm_t* a = &sorted[i].arm;
    const junction_arm_t* b = &sorted[j].arm;
    if (a->pavement <= 0 || b->pavement <= 0) continue;
    double sine = fabs(a->direction.x * (-b->direction.z) -
                       (-a->direction.z) * b->direction.x);
    double cosine =
        a->direction.x * b->direction.x + a->direction.z * b->direction.z;
    if (sine <= PARALLEL_SINE && cosine < 0) continue; /* opposite arms */
    corner_t outer = corner_between(centre, *a, a->half_width + a->pavement, *b,
                                    b->half_width + b->pavement);
    pavement_corner_t* fill = &out->corners[out->corner_count];
    local_point_t end_a = point_at(centre, a->direction, cut[i]);
    local_point_t end_b = point_at(centre, b->direction, cut[j]);
    fill->count = 0;
    add_point(fill->polygon, &fill->count, JUNCTION_CORNER_POINTS,
              shift(end_a, left_of(a->direction), a->half_width));
    add_point(fill->polygon, &fill->count, JUNCTION_CORNER_POINTS,
              shift(end_a, left_of(a->direction), a->half_width + a->pavement));
    for (size_t k = 0; k < outer.count; k++)
      add_point(fill->polygon, &fill->count, JUNCTION_CORNER_POINTS,
                outer.points[k]);
    add_point(
        fill->polygon, &fill->count, JUNCTION_CORNER_POINTS,
        shift(end_b, right_of(b->direction), b->half_width + b->pavement));
    add_point(fill->polygon, &fill->count, JUNCTION_CORNER_POINTS,
              shift(end_b, right_of(b->direction), b->half_width));
    for (size_t k = road_corner[i].count; k > 0; k--)
      add_point(fill->polygon, &fill->count, JUNCTION_CORNER_POINTS,
                road_corner[i].points[k - 1]);
    /* Kerb along the road side: from arm b's inner end back to arm a's. */
    fill->kerb_count = 0;
    fill->kerb[fill->kerb_count++] =
        shift(end_b, right_of(b->direction), b->half_width);
    for (size_t k = road_corner[i].count; k > 0; k--)
      fill->kerb[fill->kerb_count++] = road_corner[i].points[k - 1];
    fill->kerb[fill->kerb_count++] =
        shift(end_a, left_of(a->direction), a->half_width);
    out->corner_count++;
  }
  return true;
}

double polyline_length(const local_point_t* points, size_t count) {
  double length = 0;
  for (size_t i = 1; i < count; i++)
    length +=
        hypot(points[i].x - points[i - 1].x, points[i].z - points[i - 1].z);
  return length;
}

static local_point_t along(const local_point_t* points, size_t count,
                           double distance, size_t* segment) {
  double walked = 0;
  for (size_t i = 1; i < count; i++) {
    double step =
        hypot(points[i].x - points[i - 1].x, points[i].z - points[i - 1].z);
    if (walked + step >= distance || i == count - 1) {
      double t = step > 1e-12 ? (distance - walked) / step : 0;
      if (t < 0) t = 0;
      if (t > 1) t = 1;
      local_point_t p = {points[i - 1].x + (points[i].x - points[i - 1].x) * t,
                         points[i - 1].z + (points[i].z - points[i - 1].z) * t};
      *segment = i;
      return p;
    }
    walked += step;
  }
  *segment = count - 1;
  return points[count - 1];
}

size_t polyline_cut(const local_point_t* points, size_t count,
                    double from_start, double from_end, local_point_t* out) {
  double length = polyline_length(points, count);
  const double minimum = 1.0;
  if (from_start + from_end > length - minimum) {
    /* Too short to honour both cuts: share what is there. */
    double available = fmax(length - minimum, 0.0);
    double total = from_start + from_end;
    from_start = total > 0 ? available * from_start / total : 0;
    from_end = total > 0 ? available * from_end / total : 0;
  }
  size_t first_segment, last_segment;
  local_point_t first = along(points, count, from_start, &first_segment);
  local_point_t last = along(points, count, length - from_end, &last_segment);
  size_t n = 0;
  out[n++] = first;
  for (size_t i = first_segment; i < last_segment; i++) out[n++] = points[i];
  out[n++] = last;
  /* Drop points that coincide with the cut ends. */
  size_t m = 0;
  for (size_t i = 0; i < n; i++) {
    if (m > 0 && hypot(out[i].x - out[m - 1].x, out[i].z - out[m - 1].z) < 1e-6)
      continue;
    out[m++] = out[i];
  }
  if (m < 2) {
    out[1] = last;
    m = 2;
  }
  return m;
}
