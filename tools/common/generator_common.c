/* Mesh and geometry helpers the generator's parts share. */

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "config.h"
#include "generator_internal.h"
#include "junction_geometry.h"
#include "log.h"
#include "physics_world.h"
#include "triangulate.h"

/* ---- mesh helpers ---- */

vec3_t gen_at(local_point_t p, float y) {
  return vec3((float)p.x, y, (float)p.z);
}

bool gen_add_vertex(mesh_data_t* mesh, vec3_t position, vec3_t normal, float u,
                    float v) {
  vertex_t vertex = {position, normal, u, v};
  return mesh_data_add_vertex(mesh, vertex);
}

/* Adds a, b, c wound so the face normal points along `up_hint`. */
bool gen_add_face(mesh_data_t* mesh, uint32_t a, uint32_t b, uint32_t c,
                  vec3_t up_hint) {
  vec3_t pa = mesh->vertices[a].position, pb = mesh->vertices[b].position,
         pc = mesh->vertices[c].position;
  vec3_t normal = vec3_cross(vec3_sub(pb, pa), vec3_sub(pc, pa));
  if (vec3_dot(normal, up_hint) < 0)
    return mesh_data_add_triangle(mesh, a, c, b);
  return mesh_data_add_triangle(mesh, a, b, c);
}

vec2_t gen_v2(double x, double z) {
  vec2_t v = {x, z};
  return v;
}

vec2_t gen_v2_sub(local_point_t a, local_point_t b) {
  return gen_v2(a.x - b.x, a.z - b.z);
}

double gen_v2_length(vec2_t v) { return hypot(v.x, v.z); }

vec2_t gen_v2_normalize(vec2_t v) {
  double length = gen_v2_length(v);
  return length > 1e-9 ? gen_v2(v.x / length, v.z / length) : gen_v2(0, 0);
}

/* Right-hand side of a direction, seen from above with y up. */
vec2_t gen_v2_right(vec2_t d) { return gen_v2(-d.z, d.x); }

local_point_t gen_offset(local_point_t p, vec2_t d, double amount) {
  local_point_t q = {p.x + d.x * amount, p.z + d.z * amount};
  return q;
}

/* Lateral unit offsets per point with mitre at the interior joints. */
void gen_lateral_offsets(const local_point_t* points, size_t count,
                         vec2_t* laterals) {
  for (size_t i = 0; i < count; i++) {
    vec2_t before = i > 0
                        ? gen_v2_normalize(gen_v2_sub(points[i], points[i - 1]))
                        : gen_v2(0, 0);
    vec2_t after = i + 1 < count
                       ? gen_v2_normalize(gen_v2_sub(points[i + 1], points[i]))
                       : gen_v2(0, 0);
    vec2_t direction =
        gen_v2_normalize(gen_v2(before.x + after.x, before.z + after.z));
    if (gen_v2_length(direction) < 1e-6) direction = i > 0 ? before : after;
    vec2_t right = gen_v2_right(direction);
    if (i > 0 && i + 1 < count) {
      double cosine = before.x * after.x + before.z * after.z;
      double half = sqrt(fmax((1.0 + cosine) * 0.5, 0.05)); /* cos(theta / 2) */
      double scale = fmin(1.0 / half, 2.5);
      right = gen_v2(right.x * scale, right.z * scale);
    }
    laterals[i] = right;
  }
}

/* A strip between offsets a..b (metres right of the centre line, negative is
 * left) at height y, with uv u across 0..1 and v along in texture repeats. */
bool gen_add_strip(mesh_data_t* mesh, const local_point_t* points,
                   const vec2_t* laterals, size_t count, const double* from,
                   const double* to, float y, float texture_length) {
  uint32_t base = (uint32_t)mesh->vertex_count;
  double distance = 0;
  for (size_t i = 0; i < count; i++) {
    if (i > 0) distance += gen_v2_length(gen_v2_sub(points[i], points[i - 1]));
    float v = (float)(distance / texture_length);
    if (!gen_add_vertex(mesh,
                        gen_at(gen_offset(points[i], laterals[i], from[i]), y),
                        vec3(0, 1, 0), 0, v))
      return false;
    if (!gen_add_vertex(mesh,
                        gen_at(gen_offset(points[i], laterals[i], to[i]), y),
                        vec3(0, 1, 0), 1, v))
      return false;
  }
  for (size_t i = 0; i + 1 < count; i++) {
    uint32_t a = base + (uint32_t)i * 2, b = a + 1, c = a + 2, d = a + 3;
    if (!gen_add_face(mesh, a, b, d, vec3(0, 1, 0)) ||
        !gen_add_face(mesh, a, d, c, vec3(0, 1, 0)))
      return false;
  }
  return true;
}

/* A vertical face along the offset line, from y0 to y1, facing `outward`. */
bool gen_add_kerb(mesh_data_t* mesh, const local_point_t* points,
                  const vec2_t* laterals, size_t count, const double* along,
                  float y0, float y1, bool facing_right) {
  uint32_t base = (uint32_t)mesh->vertex_count;
  for (size_t i = 0; i < count; i++) {
    vec2_t right = gen_v2_normalize(laterals[i]);
    vec3_t normal = vec3((float)(facing_right ? right.x : -right.x), 0,
                         (float)(facing_right ? right.z : -right.z));
    local_point_t p = gen_offset(points[i], laterals[i], along[i]);
    if (!gen_add_vertex(mesh, gen_at(p, y0), normal, (float)i, 0) ||
        !gen_add_vertex(mesh, gen_at(p, y1), normal, (float)i, 1))
      return false;
  }
  for (size_t i = 0; i + 1 < count; i++) {
    uint32_t a = base + (uint32_t)i * 2, b = a + 1, c = a + 2, d = a + 3;
    vec3_t hint = mesh->vertices[a].normal;
    if (!gen_add_face(mesh, a, b, d, hint) ||
        !gen_add_face(mesh, a, d, c, hint))
      return false;
  }
  return true;
}

city_chunk_t* gen_chunk_at(layout_t* layout, local_point_t p) {
  int column = (int)floor((p.x - layout->min_x) / layout->options->chunk_size);
  int row = (int)floor((p.z - layout->min_z) / layout->options->chunk_size);
  if (column < 0) column = 0;
  if (row < 0) row = 0;
  if (column >= layout->columns) column = layout->columns - 1;
  if (row >= layout->rows) row = layout->rows - 1;
  return city_package_chunk(layout->package, column, row);
}

local_point_t gen_centroid(const local_point_t* points, size_t count) {
  local_point_t c = {0, 0};
  for (size_t i = 0; i < count; i++) {
    c.x += points[i].x;
    c.z += points[i].z;
  }
  c.x /= (double)count;
  c.z /= (double)count;
  return c;
}

void gen_finish_part(city_chunk_t* chunk, city_part_t* part) {
  chunk->bounds = aabb_merge(chunk->bounds, mesh_data_bounds(&part->mesh));
}

double gen_point_segment_distance(local_point_t p, local_point_t a,
                                  local_point_t b) {
  double abx = b.x - a.x, abz = b.z - a.z;
  double length_squared = abx * abx + abz * abz;
  double t = length_squared > 1e-12
                 ? ((p.x - a.x) * abx + (p.z - a.z) * abz) / length_squared
                 : 0;
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  return hypot(p.x - (a.x + abx * t), p.z - (a.z + abz * t));
}

/* Drops repeated consecutive points, and the last if it repeats the first. */
static size_t dedupe_ring(const local_point_t* in, size_t count,
                          local_point_t* out) {
  size_t n = 0;
  for (size_t i = 0; i < count; i++) {
    if (n > 0 && hypot(in[i].x - out[n - 1].x, in[i].z - out[n - 1].z) < 1e-4)
      continue;
    out[n++] = in[i];
  }
  while (n > 1 &&
         hypot(out[n - 1].x - out[0].x, out[n - 1].z - out[0].z) < 1e-4)
    n--;
  return n;
}

bool gen_add_polygon_part(city_chunk_t* chunk, city_part_t* part,
                          const local_point_t* raw, size_t raw_count, float y,
                          float uv_scale) {
  local_point_t polygon[JUNCTION_MAX_POLYGON];
  size_t count = dedupe_ring(raw, raw_count, polygon);
  if (count < 3 || fabs(polygon_signed_area(polygon, count)) < 0.01)
    return true;
  uint32_t* indices = malloc(3 * (count - 2) * sizeof(uint32_t));
  if (!indices) return false;
  bool clean;
  size_t triangles = triangulate_polygon(polygon, count, indices, &clean);
  uint32_t base = (uint32_t)part->mesh.vertex_count;
  for (size_t i = 0; i < count; i++)
    if (!gen_add_vertex(&part->mesh, gen_at(polygon[i], y), vec3(0, 1, 0),
                        (float)(polygon[i].x * uv_scale),
                        (float)(polygon[i].z * uv_scale))) {
      free(indices);
      return false;
    }
  for (size_t t = 0; t < triangles; t++) {
    local_point_t tri[3] = {polygon[indices[t * 3]],
                            polygon[indices[t * 3 + 1]],
                            polygon[indices[t * 3 + 2]]};
    if (fabs(polygon_signed_area(tri, 3)) < 1e-4) continue; /* a sliver */
    if (!gen_add_face(&part->mesh, base + indices[t * 3],
                      base + indices[t * 3 + 1], base + indices[t * 3 + 2],
                      vec3(0, 1, 0))) {
      free(indices);
      return false;
    }
  }
  free(indices);
  gen_finish_part(chunk, part);
  return true;
}

/* Position `distance` metres along a polyline and the segment it lies on. */
local_point_t gen_along_polyline(const local_point_t* points, size_t count,
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

bool gen_point_inside(local_point_t p, const local_point_t* ring,
                      size_t count) {
  bool inside = false;
  for (size_t i = 0, j = count - 1; i < count; j = i++) {
    bool crosses = (ring[i].z > p.z) != (ring[j].z > p.z);
    if (!crosses) continue;
    double at =
        (ring[j].x - ring[i].x) * (p.z - ring[i].z) / (ring[j].z - ring[i].z) +
        ring[i].x;
    if (p.x < at) inside = !inside;
  }
  return inside;
}

bool gen_add_wall_quad(city_part_t* walls, local_point_t p, local_point_t q,
                       float y0, float y1, vec3_t normal, float u0, float u1,
                       float v0, float v1) {
  uint32_t base = (uint32_t)walls->mesh.vertex_count;
  if (!gen_add_vertex(&walls->mesh, gen_at(p, y0), normal, u0, v0) ||
      !gen_add_vertex(&walls->mesh, gen_at(q, y0), normal, u1, v0) ||
      !gen_add_vertex(&walls->mesh, gen_at(q, y1), normal, u1, v1) ||
      !gen_add_vertex(&walls->mesh, gen_at(p, y1), normal, u0, v1))
    return false;
  return gen_add_face(&walls->mesh, base, base + 1, base + 2, normal) &&
         gen_add_face(&walls->mesh, base, base + 2, base + 3, normal);
}

bounding_rectangle_t gen_bounding_rectangle(const local_point_t* points,
                                            size_t count) {
  bounding_rectangle_t best;
  best.area = 1e300;
  for (size_t i = 0; i < count; i++) {
    vec2_t axis =
        gen_v2_normalize(gen_v2_sub(points[(i + 1) % count], points[i]));
    if (gen_v2_length(axis) < 0.5) continue;
    vec2_t across = gen_v2_right(axis);
    double a0 = 1e300, a1 = -1e300, c0 = 1e300, c1 = -1e300;
    for (size_t k = 0; k < count; k++) {
      double a = points[k].x * axis.x + points[k].z * axis.z;
      double c = points[k].x * across.x + points[k].z * across.z;
      a0 = fmin(a0, a), a1 = fmax(a1, a), c0 = fmin(c0, c), c1 = fmax(c1, c);
    }
    double area = (a1 - a0) * (c1 - c0);
    if (area < best.area) {
      best.axis = axis, best.across = across;
      best.along_min = a0, best.along_max = a1, best.across_min = c0,
      best.across_max = c1;
      best.area = area;
    }
  }
  if (best.along_max - best.along_min < best.across_max - best.across_min) {
    /* Swap so the axis follows the long side. */
    bounding_rectangle_t swapped = best;
    swapped.axis = best.across, swapped.across = gen_v2_right(best.across);
    swapped.along_min = best.across_min, swapped.along_max = best.across_max;
    swapped.across_min = -best.along_max, swapped.across_max = -best.along_min;
    best = swapped;
  }
  return best;
}

vec3_t gen_rectangle_point(const bounding_rectangle_t* r, double along,
                           double across, float y) {
  return vec3((float)(r->axis.x * along + r->across.x * across), y,
              (float)(r->axis.z * along + r->across.z * across));
}

double gen_terrain_at(const layout_t* layout, double x, double z) {
  return layout->terrain ? city_terrain_height(layout->terrain, x, z) : 0.0;
}

void gen_lift_part(city_part_t* part, float dy) {
  for (size_t v = 0; v < part->mesh.vertex_count; v++)
    part->mesh.vertices[v].position.y += dy;
}
