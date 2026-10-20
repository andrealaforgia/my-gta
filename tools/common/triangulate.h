/**
 * @file triangulate.h
 * @brief Ear-clipping triangulation of a simple polygon given
 * anticlockwise in (x, north). Degenerate input gets a fan and a false
 * return so callers can report it.
 */
#ifndef TOOLS_COMMON_TRIANGULATE_H_
#define TOOLS_COMMON_TRIANGULATE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "projection.h"

/* out_indices must hold 3 * (count - 2) entries. Returns the triangle count. */
size_t triangulate_polygon(const local_point_t* points, size_t count,
                           uint32_t* out_indices, bool* clean);
double polygon_signed_area(const local_point_t* points, size_t count);

/* Triangulates an anticlockwise outline with clockwise holes by bridging
 * each hole to the outline. `merged` receives the combined ring (room for
 * outer + sum(holes) + 2 * hole_count points), `out_indices` indexes
 * into it (room for 3 * (merged - 2)). Returns the triangle count. */
size_t triangulate_with_holes(const local_point_t* outer, size_t outer_count,
                              const local_point_t* const* holes,
                              const size_t* hole_counts, size_t hole_count,
                              local_point_t* merged, size_t* merged_count,
                              uint32_t* out_indices, bool* clean);

#endif  // TOOLS_COMMON_TRIANGULATE_H_
