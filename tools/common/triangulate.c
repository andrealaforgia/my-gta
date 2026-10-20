#include "triangulate.h"

#include <math.h>
#include <stdlib.h>

double polygon_signed_area(const local_point_t* points, size_t count) {
  double area = 0;
  for (size_t i = 0; i < count; i++) {
    local_point_t p = points[i], q = points[(i + 1) % count];
    area += p.x * -q.z - q.x * -p.z;
  }
  return area * 0.5;
}

/* Twice the signed area of a, b, c in (x, north). */
static double cross(local_point_t a, local_point_t b, local_point_t c) {
  return (b.x - a.x) * (-(c.z - a.z)) - (-(b.z - a.z)) * (c.x - a.x);
}

static bool inside(local_point_t p, local_point_t a, local_point_t b,
                   local_point_t c) {
  return cross(a, b, p) >= -1e-9 && cross(b, c, p) >= -1e-9 &&
         cross(c, a, p) >= -1e-9;
}

size_t triangulate_polygon(const local_point_t* points, size_t count,
                           uint32_t* out, bool* clean) {
  *clean = true;
  if (count < 3) return 0;
  uint32_t* remaining = malloc(count * sizeof(uint32_t));
  if (!remaining) {
    *clean = false;
    return 0;
  }
  bool reversed = polygon_signed_area(points, count) < 0;
  for (size_t i = 0; i < count; i++)
    remaining[i] = (uint32_t)(reversed ? count - 1 - i : i);
  size_t left = count, triangles = 0, guard = 0;
  while (left > 3 && guard < count * count) {
    bool clipped = false;
    for (size_t i = 0; i < left; i++) {
      uint32_t ia = remaining[(i + left - 1) % left], ib = remaining[i],
               ic = remaining[(i + 1) % left];
      local_point_t a = points[ia], b = points[ib], c = points[ic];
      if (cross(a, b, c) <= 1e-9) continue; /* reflex or flat */
      bool ear = true;
      for (size_t j = 0; j < left && ear; j++) {
        uint32_t ip = remaining[j];
        if (ip == ia || ip == ib || ip == ic) continue;
        if (inside(points[ip], a, b, c)) ear = false;
      }
      if (!ear) continue;
      out[triangles * 3] = ia;
      out[triangles * 3 + 1] = ib;
      out[triangles * 3 + 2] = ic;
      triangles++;
      for (size_t j = i; j + 1 < left; j++) remaining[j] = remaining[j + 1];
      left--;
      clipped = true;
      break;
    }
    guard++;
    if (!clipped) {
      /* Self-intersecting or degenerate: fall back to a fan so the roof
       * exists, and say so. */
      *clean = false;
      for (size_t j = 1; j + 1 < left; j++) {
        out[triangles * 3] = remaining[0];
        out[triangles * 3 + 1] = remaining[j];
        out[triangles * 3 + 2] = remaining[j + 1];
        triangles++;
      }
      free(remaining);
      return triangles;
    }
  }
  if (left == 3) {
    out[triangles * 3] = remaining[0];
    out[triangles * 3 + 1] = remaining[1];
    out[triangles * 3 + 2] = remaining[2];
    triangles++;
  }
  free(remaining);
  return triangles;
}
