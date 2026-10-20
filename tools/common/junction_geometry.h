/**
 * @file junction_geometry.h
 * @brief Where roads meet. Given the arms leaving a junction, intersects
 * the adjacent road edges to find the corners, works out how far each
 * road must be cut back so its edges clear the corners, and returns the
 * polygon that fills the gap plus the pavement corner fills between
 * neighbouring arms. Pure geometry in the (x, z) plane, x east, z south.
 */
#ifndef TOOLS_COMMON_JUNCTION_GEOMETRY_H_
#define TOOLS_COMMON_JUNCTION_GEOMETRY_H_

#include <stdbool.h>
#include <stddef.h>

#include "projection.h"

#define JUNCTION_MAX_ARMS 8
#define JUNCTION_MAX_POLYGON (JUNCTION_MAX_ARMS * 4)
#define JUNCTION_CORNER_POINTS 8

typedef struct {
  double x, z;
} plane_vec_t;

typedef struct {
  plane_vec_t direction; /* unit, leaving the junction */
  double half_width;
  double pavement; /* width, 0 for none */
} junction_arm_t;

typedef struct {
  local_point_t polygon[JUNCTION_CORNER_POINTS];
  size_t count;
  /* The road-side edge of the fill, for the kerb: inner points in order. */
  local_point_t kerb[4];
  size_t kerb_count;
} pavement_corner_t;

typedef struct {
  double cut[JUNCTION_MAX_ARMS]; /* metres each arm is shortened */
  local_point_t polygon[JUNCTION_MAX_POLYGON];
  size_t polygon_count;
  pavement_corner_t corners[JUNCTION_MAX_ARMS];
  size_t corner_count;
  double outer_radius; /* furthest polygon point from the centre */
} junction_layout_t;

/* Returns false when no patch is needed (a straight continuation or a
 * single arm); cuts are still filled in. Arms may be in any order. */
bool junction_layout(local_point_t centre, const junction_arm_t* arms,
                     size_t arm_count, double margin, junction_layout_t* out);

/* Shortens a polyline by `from_start` and `from_end` metres, keeping at
 * least a short stub. `out` needs room for `count` points. Returns the
 * new count. */
size_t polyline_cut(const local_point_t* points, size_t count,
                    double from_start, double from_end, local_point_t* out);
double polyline_length(const local_point_t* points, size_t count);

#endif  // TOOLS_COMMON_JUNCTION_GEOMETRY_H_
