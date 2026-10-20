#include "triangulate.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

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

static bool same_point(local_point_t p, local_point_t q) {
  return fabs(p.x - q.x) < 1e-9 && fabs(p.z - q.z) < 1e-9;
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
        local_point_t p = points[ip];
        /* Bridged holes repeat vertices; a coincident point is a corner,
         * not an intruder. */
        if (same_point(p, a) || same_point(p, b) || same_point(p, c)) continue;
        if (inside(p, a, b, c)) ear = false;
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

/* Whether segments a-b and c-d cross, ignoring shared endpoints. */
static bool segments_cross(local_point_t a, local_point_t b, local_point_t c,
                           local_point_t d) {
  double d1 = cross(c, d, a), d2 = cross(c, d, b), d3 = cross(a, b, c),
         d4 = cross(a, b, d);
  return ((d1 > 1e-9 && d2 < -1e-9) || (d1 < -1e-9 && d2 > 1e-9)) &&
         ((d3 > 1e-9 && d4 < -1e-9) || (d3 < -1e-9 && d4 > 1e-9));
}

static bool bridge_is_clear(local_point_t from, local_point_t to,
                            const local_point_t* ring, size_t count) {
  for (size_t i = 0; i < count; i++) {
    local_point_t a = ring[i], b = ring[(i + 1) % count];
    if (segments_cross(from, to, a, b)) return false;
  }
  return true;
}

size_t triangulate_with_holes(const local_point_t* outer, size_t outer_count,
                              const local_point_t* const* holes,
                              const size_t* hole_counts, size_t hole_count,
                              local_point_t* merged, size_t* merged_count,
                              uint32_t* out_indices, bool* clean) {
  size_t count = outer_count;
  memcpy(merged, outer, outer_count * sizeof(local_point_t));
  *clean = true;
  for (size_t h = 0; h < hole_count; h++) {
    const local_point_t* hole = holes[h];
    size_t n = hole_counts[h];
    if (n < 3) continue;
    /* The hole vertex furthest east, then the nearest ring vertex it can
     * see; the bridge joins them in both directions. */
    size_t start = 0;
    for (size_t i = 1; i < n; i++)
      if (hole[i].x > hole[start].x) start = i;
    size_t best = count;
    double best_distance = 1e300;
    for (size_t i = 0; i < count; i++) {
      double distance =
          hypot(merged[i].x - hole[start].x, merged[i].z - hole[start].z);
      if (distance >= best_distance) continue;
      bool clear = bridge_is_clear(hole[start], merged[i], merged, count);
      for (size_t k = 0; k < hole_count && clear; k++)
        clear =
            bridge_is_clear(hole[start], merged[i], holes[k], hole_counts[k]);
      if (clear) {
        best = i;
        best_distance = distance;
      }
    }
    if (best == count) {
      *clean = false; /* no visible bridge: the hole is left filled */
      continue;
    }
    /* Insert after `best`: hole[start..], hole[start], merged[best]. */
    size_t insert = n + 2;
    memmove(merged + best + 1 + insert, merged + best + 1,
            (count - best - 1) * sizeof(local_point_t));
    size_t w = best + 1;
    for (size_t i = 0; i < n; i++) merged[w++] = hole[(start + i) % n];
    merged[w++] = hole[start];
    merged[w++] = merged[best];
    count += insert;
  }
  *merged_count = count;
  bool ears_clean;
  size_t triangles =
      triangulate_polygon(merged, count, out_indices, &ears_clean);
  if (!ears_clean) *clean = false;
  return triangles;
}
