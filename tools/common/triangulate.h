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

#endif  // TOOLS_COMMON_TRIANGULATE_H_
