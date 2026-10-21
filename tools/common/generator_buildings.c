/* Buildings: walls, roofs, porticoes, landmarks, doors and chimneys. */

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

static double ridge_across(const ridge_t* ridge, local_point_t p) {
  return p.x * ridge->across.x + p.z * ridge->across.z;
}

static double ridge_height(const ridge_t* ridge, double eaves,
                           local_point_t p) {
  return eaves + ridge->rise -
         ridge->slope * fabs(ridge_across(ridge, p) - ridge->mid);
}

static bool street_is_arcaded(const city_generator_options_t* o,
                              const char* name) {
  if (!name[0] || !o->arcade_streets[0]) return false;
  size_t length = strlen(name);
  const char* cursor = o->arcade_streets;
  while (*cursor) {
    const char* end = strchr(cursor, ',');
    size_t token = end ? (size_t)(end - cursor) : strlen(cursor);
    if (token == length && strncmp(cursor, name, length) == 0) return true;
    cursor += token + (end ? 1 : 0);
  }
  return false;
}

/* True when edge i of a building faces a named arcaded pedestrian street
 * closely and runs alongside it, on a building deep enough to set a
 * ground floor back. */
bool gen_edge_faces_arcade(const city_model_t* model,
                           const city_generator_options_t* o,
                           const city_building_t* building, size_t i,
                           double depth_available) {
  if (!o->arcade_streets[0] || depth_available < 8.0) return false;
  local_point_t a = building->points[i];
  local_point_t b = building->points[(i + 1) % building->point_count];
  double length = hypot(b.x - a.x, b.z - a.z);
  if (length < 6.0) return false;
  vec2_t direction = gen_v2_normalize(gen_v2_sub(b, a));
  local_point_t middle = {(a.x + b.x) * 0.5, (a.z + b.z) * 0.5};
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* road = &model->roads[r];
    if (road->class != ROAD_CLASS_PEDESTRIAN ||
        !street_is_arcaded(o, road->name))
      continue;
    for (size_t k = 1; k < road->point_count; k++) {
      vec2_t along =
          gen_v2_normalize(gen_v2_sub(road->points[k], road->points[k - 1]));
      if (fabs(direction.x * along.x + direction.z * along.z) < 0.9) continue;
      if (gen_point_segment_distance(middle, road->points[k - 1],
                                     road->points[k]) < road->width * 0.5 + 3.5)
        return true;
    }
  }
  return false;
}

double gen_building_depth(const city_building_t* building) {
  /* The shorter side of the bounding rectangle. */
  double best = 1e30;
  for (size_t i = 0; i < building->point_count; i++) {
    vec2_t axis = gen_v2_normalize(
        gen_v2_sub(building->points[(i + 1) % building->point_count],
                   building->points[i]));
    if (gen_v2_length(axis) < 0.5) continue;
    double lo = 1e30, hi = -1e30, lo2 = 1e30, hi2 = -1e30;
    for (size_t k = 0; k < building->point_count; k++) {
      double along =
          building->points[k].x * axis.x + building->points[k].z * axis.z;
      double across =
          -building->points[k].x * axis.z + building->points[k].z * axis.x;
      lo = fmin(lo, along), hi = fmax(hi, along);
      lo2 = fmin(lo2, across), hi2 = fmax(hi2, across);
    }
    best = fmin(best, fmin(hi - lo, hi2 - lo2));
  }
  return best;
}

static bool arcade_continues_at(const layout_t* layout, local_point_t p) {
  int touching = 0;
  for (size_t e = 0; e < layout->arcade_edge_count; e++) {
    const arcade_edge_t* edge = &layout->arcade_edges[e];
    if (hypot(edge->a.x - p.x, edge->a.z - p.z) < 1.5) touching++;
    if (hypot(edge->b.x - p.x, edge->b.z - p.z) < 1.5) touching++;
  }
  return touching >= 2; /* this edge and its neighbour */
}

/* One square pillar standing at `centre`, aligned to the edge. */
static bool add_pillar(city_part_t* walls, local_point_t centre, vec2_t along,
                       float y0, float y1, float storey) {
  double h = ARCADE_PILLAR * 0.5;
  vec2_t out = gen_v2(-along.z, along.x);
  local_point_t corner[4];
  double signs[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
  for (int k = 0; k < 4; k++) {
    corner[k].x =
        centre.x + along.x * h * signs[k][0] + out.x * h * signs[k][1];
    corner[k].z =
        centre.z + along.z * h * signs[k][0] + out.z * h * signs[k][1];
  }
  for (int k = 0; k < 4; k++) {
    local_point_t p = corner[k], q = corner[(k + 1) % 4];
    vec2_t d = gen_v2_normalize(gen_v2_sub(q, p));
    vec3_t normal = vec3((float)-d.z, 0, (float)d.x);
    /* The corners run anticlockwise seen from above: signs above go
     * (-,-), (+,-), (+,+), (-,+). */
    if (!gen_add_wall_quad(walls, p, q, y0, y1, normal, 0, 0.25f, y0 / storey,
                           y1 / storey))
      return false;
  }
  return true;
}

/* The ground floor of an edge set back behind pillars: the wall at the
 * back, the returns at the ends unless the arcade carries on, the soffit
 * and the pillars. `floor` collects the paved walkway. */
static bool add_portico(const layout_t* layout, city_part_t* walls,
                        city_part_t* walkway, local_point_t a, local_point_t b,
                        float height, float skirt) {
  const city_generator_options_t* o = layout->options;
  vec2_t along = gen_v2_sub(b, a);
  double length = gen_v2_length(along);
  vec2_t direction = gen_v2_normalize(along);
  vec3_t outward = vec3((float)-direction.z, 0, (float)direction.x);
  vec2_t inward = gen_v2(direction.z, -direction.x);
  local_point_t a2 = gen_offset(a, inward, ARCADE_DEPTH);
  local_point_t b2 = gen_offset(b, inward, ARCADE_DEPTH);
  float storey = o->storey_height;
  float top = fminf(height, 1.5f * storey);
  float u1 = (float)(length / o->window_width);
  /* The back wall faces out toward the street. */
  if (!gen_add_wall_quad(walls, a2, b2, -skirt, top, outward, 0, u1,
                         -skirt / storey, top / storey))
    return false;
  /* Returns close the ends of an arcade that stops there. */
  if (!arcade_continues_at(layout, a) &&
      !gen_add_wall_quad(walls, a, a2, -skirt, top,
                         vec3((float)direction.x, 0, (float)direction.z), 0,
                         (float)(ARCADE_DEPTH / o->window_width),
                         -skirt / storey, top / storey))
    return false;
  if (!arcade_continues_at(layout, b) &&
      !gen_add_wall_quad(walls, b2, b, -skirt, top,
                         vec3((float)-direction.x, 0, (float)-direction.z), 0,
                         (float)(ARCADE_DEPTH / o->window_width),
                         -skirt / storey, top / storey))
    return false;
  /* The soffit overhead, facing down. */
  uint32_t base = (uint32_t)walls->mesh.vertex_count;
  vec3_t down = vec3(0, -1, 0);
  if (!gen_add_vertex(&walls->mesh, gen_at(a, top), down, 0, 0) ||
      !gen_add_vertex(&walls->mesh, gen_at(b, top), down, u1, 0) ||
      !gen_add_vertex(&walls->mesh, gen_at(b2, top), down, u1, 1) ||
      !gen_add_vertex(&walls->mesh, gen_at(a2, top), down, 0, 1))
    return false;
  if (!gen_add_face(&walls->mesh, base, base + 1, base + 2, down) ||
      !gen_add_face(&walls->mesh, base, base + 2, base + 3, down))
    return false;
  /* Pillars along the building line, a face in from it so the upper
   * walls overhang them. */
  int pillars = (int)floor(length / ARCADE_SPACING + 0.5);
  if (pillars < 1) pillars = 1;
  for (int k = 0; k < pillars; k++) {
    double t = (k + 0.5) / pillars * length;
    local_point_t centre =
        gen_offset(gen_offset(a, direction, t), inward, ARCADE_PILLAR * 0.5);
    if (!add_pillar(walls, centre, direction, -skirt, top, storey))
      return false;
  }
  /* Round arches between the pillars: a spandrel panel on the building line
   * from the curve up to the soffit, facing the street. */
  float spring = fminf(top - 2.2f, 2.5f);
  float radius = 1.9f;
  for (int k = 0; k + 1 < pillars; k++) {
    double t = (k + 1.0) / pillars * length;
    double centre_along = t;
    for (int seg = 0; seg < 8; seg++) {
      double a0 = M_PI * seg / 8.0, a1 = M_PI * (seg + 1) / 8.0;
      local_point_t p0 =
          gen_offset(a, direction, centre_along + cos(a0) * radius);
      local_point_t p1 =
          gen_offset(a, direction, centre_along + cos(a1) * radius);
      p0 = gen_offset(p0, inward, ARCADE_PILLAR * 0.5);
      p1 = gen_offset(p1, inward, ARCADE_PILLAR * 0.5);
      float y0 = spring + (float)sin(a0) * radius,
            y1 = spring + (float)sin(a1) * radius;
      if (y0 > top - 0.05f) y0 = top - 0.05f;
      if (y1 > top - 0.05f) y1 = top - 0.05f;
      uint32_t base = (uint32_t)walls->mesh.vertex_count;
      float u0 = (float)((centre_along + cos(a0) * radius) / o->window_width);
      float u1v = (float)((centre_along + cos(a1) * radius) / o->window_width);
      if (!gen_add_vertex(&walls->mesh, gen_at(p0, y0), outward, u0,
                          y0 / storey) ||
          !gen_add_vertex(&walls->mesh, gen_at(p1, y1), outward, u1v,
                          y1 / storey) ||
          !gen_add_vertex(&walls->mesh, gen_at(p1, top), outward, u1v,
                          top / storey) ||
          !gen_add_vertex(&walls->mesh, gen_at(p0, top), outward, u0,
                          top / storey))
        return false;
      /* Seen from the street (a to b to the left of it): wound to face out. */
      if (!gen_add_face(&walls->mesh, base, base + 1, base + 2, outward) ||
          !gen_add_face(&walls->mesh, base, base + 2, base + 3, outward))
        return false;
    }
  }
  /* The paved walkway under it. */
  float y = o->road_height + 0.02f;
  uint32_t f = (uint32_t)walkway->mesh.vertex_count;
  vec3_t up = vec3(0, 1, 0);
  if (!gen_add_vertex(&walkway->mesh, gen_at(a, y), up, 0, 0) ||
      !gen_add_vertex(&walkway->mesh, gen_at(b, y), up, u1, 0) ||
      !gen_add_vertex(&walkway->mesh, gen_at(b2, y), up, u1, 1) ||
      !gen_add_vertex(&walkway->mesh, gen_at(a2, y), up, 0, 1))
    return false;
  return gen_add_face(&walkway->mesh, f, f + 1, f + 2, up) &&
         gen_add_face(&walkway->mesh, f, f + 2, f + 3, up);
}

/* True when edge i of a building runs alongside a main street or a
 * pedestrian street close by and the building is tall enough for a shop
 * on its ground floor; most buildings on such a street have one. */
static bool edge_faces_shops(const city_model_t* model,
                             const city_building_t* building, size_t i) {
  if (building->height < 6.0f) return false;
  unsigned hash = (unsigned)((uint64_t)building->source_id * 2654435761u >> 9);
  if (hash % 100 >= 65) return false;
  local_point_t a = building->points[i];
  local_point_t b = building->points[(i + 1) % building->point_count];
  if (hypot(b.x - a.x, b.z - a.z) < 5.0) return false;
  vec2_t direction = gen_v2_normalize(gen_v2_sub(b, a));
  local_point_t middle = {(a.x + b.x) * 0.5, (a.z + b.z) * 0.5};
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* road = &model->roads[r];
    bool main_street = road->class == ROAD_CLASS_PRIMARY ||
                       road->class == ROAD_CLASS_SECONDARY ||
                       road->class == ROAD_CLASS_TERTIARY ||
                       road->class == ROAD_CLASS_PEDESTRIAN;
    if (!main_street) continue;
    for (size_t k = 1; k < road->point_count; k++) {
      vec2_t along =
          gen_v2_normalize(gen_v2_sub(road->points[k], road->points[k - 1]));
      if (fabs(direction.x * along.x + direction.z * along.z) < 0.9) continue;
      if (gen_point_segment_distance(middle, road->points[k - 1],
                                     road->points[k]) < road->width * 0.5 + 7.5)
        return true;
    }
  }
  return false;
}

/* Wall quads around a ring. The ring is anticlockwise for an outline and
 * clockwise for a courtyard, so "outside is to the right" faces the
 * street in one case and the courtyard in the other. */
static bool add_walls(const layout_t* layout, city_part_t* walls,
                      city_part_t* walkway, city_part_t* shops,
                      const local_point_t* ring, size_t count, float h,
                      const ridge_t* ridge, float skirt, const bool* arcaded,
                      const bool* shop) {
  const city_generator_options_t* o = layout->options;
  for (size_t i = 0; i < count; i++) {
    local_point_t a = ring[i], b = ring[(i + 1) % count];
    vec2_t along = gen_v2_sub(b, a);
    double length = gen_v2_length(along);
    if (length < 1e-6) continue;
    vec2_t direction = gen_v2_normalize(along);
    vec3_t normal = vec3((float)-direction.z, 0, (float)direction.x);
    bool portico = arcaded && arcaded[i];
    bool shopfront = !portico && shop && shop[i];
    float storey = o->storey_height;
    float bottom = portico ? fminf(h, 1.5f * storey)
                           : (shopfront ? fminf(h, storey) : -skirt);
    /* Split where the edge crosses the ridge so the top follows it. */
    local_point_t pieces[3] = {a, b, b};
    size_t piece_count = 2;
    if (ridge) {
      double ca = ridge_across(ridge, a) - ridge->mid;
      double cb = ridge_across(ridge, b) - ridge->mid;
      if (ca * cb < 0) {
        double t = ca / (ca - cb);
        local_point_t crossing = {a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t};
        pieces[1] = crossing;
        pieces[2] = b;
        piece_count = 3;
      }
    }
    double travelled = 0;
    for (size_t k = 0; k + 1 < piece_count; k++) {
      local_point_t p = pieces[k], q = pieces[k + 1];
      double span = hypot(q.x - p.x, q.z - p.z);
      float top_p = ridge ? (float)ridge_height(ridge, h, p) : h;
      float top_q = ridge ? (float)ridge_height(ridge, h, q) : h;
      float u0 = (float)(travelled / o->window_width);
      float u1 = (float)((travelled + span) / o->window_width);
      uint32_t base = (uint32_t)walls->mesh.vertex_count;
      /* Above a portico the texture restarts at a clean storey line. */
      float v_bottom =
          portico ? 0.0f : (shopfront ? bottom / storey : -skirt / storey);
      float v_shift = portico ? bottom / storey : 0.0f;
      if (shopfront && !gen_add_wall_quad(shops, p, q, -skirt, bottom, normal,
                                          (float)(travelled / 4.0),
                                          (float)((travelled + span) / 4.0),
                                          -skirt / storey, bottom / storey))
        return false;
      if (!gen_add_vertex(&walls->mesh, gen_at(p, bottom), normal, u0,
                          v_bottom) ||
          !gen_add_vertex(&walls->mesh, gen_at(q, bottom), normal, u1,
                          v_bottom) ||
          !gen_add_vertex(&walls->mesh, gen_at(q, top_q), normal, u1,
                          top_q / storey - v_shift) ||
          !gen_add_vertex(&walls->mesh, gen_at(p, top_p), normal, u0,
                          top_p / storey - v_shift))
        return false;
      if (!gen_add_face(&walls->mesh, base, base + 1, base + 2, normal) ||
          !gen_add_face(&walls->mesh, base, base + 2, base + 3, normal))
        return false;
      travelled += span;
    }
    if (portico && !add_portico(layout, walls, walkway, a, b, h, skirt))
      return false;
  }
  return true;
}

/* Footprints that nearly fill their bounding rectangle and have no
 * courtyard get a pitched roof with eaves; a narrow, long one a gabled
 * roof. Every other building under 22 m, over a hut and under a
 * warehouse, gets a ridge roof that follows its outline. Taller and
 * larger ones stay flat. */
static roof_shape_t roof_shape(const city_building_t* building) {
  double footprint =
      fabs(polygon_signed_area(building->points, building->point_count));
  bool nearly_rectangular = false;
  roof_shape_t shape = ROOF_FLAT;
  if (building->hole_count == 0 && building->point_count >= 4) {
    bounding_rectangle_t r =
        gen_bounding_rectangle(building->points, building->point_count);
    if (r.area > 0 && footprint / r.area >= 0.82) {
      nearly_rectangular = true;
      double width = r.across_max - r.across_min;
      double length = r.along_max - r.along_min;
      shape = width < 9.0 && length > 1.6 * width ? ROOF_GABLED : ROOF_HIPPED;
    }
  }
  if (nearly_rectangular) return shape;
  if (building->height < 22.0f && footprint > 25.0 && footprint < 9000.0)
    return ROOF_RIDGE;
  return ROOF_FLAT;
}

/* A pitched roof with eaves on footprints that nearly fill their bounding
 * rectangle and have no courtyard: hipped, or gabled when the footprint is
 * a narrow terrace, with the gable triangles added to the walls. Sets
 * *done when it built one. */
static bool add_pitched_roof(city_part_t* roof, city_part_t* walls,
                             const city_building_t* building, float h,
                             const city_generator_options_t* o, bool* done) {
  *done = false;
  if (building->hole_count > 0 || building->point_count < 4) return true;
  bounding_rectangle_t r =
      gen_bounding_rectangle(building->points, building->point_count);
  double footprint =
      fabs(polygon_signed_area(building->points, building->point_count));
  if (r.area <= 0 || footprint / r.area < 0.82) return true;
  const double eaves = 0.35;
  double width = r.across_max - r.across_min;
  double length = r.along_max - r.along_min;
  bool gabled = width < 9.0 && length > 1.6 * width;
  double a0 = r.along_min - (gabled ? 0 : eaves);
  double a1 = r.along_max + (gabled ? 0 : eaves);
  double c0 = r.across_min - eaves, c1 = r.across_max + eaves;
  double half_span = (c1 - c0) * 0.5, mid = (c0 + c1) * 0.5;
  double inset = gabled ? 0 : fmin(half_span, (a1 - a0) * 0.5);
  /* A tall slender block is a tower: it ends in a spire, not a cap. */
  bool tower = h > 30.0f && width < 16.0 && length < 16.0;
  float rise = tower ? (float)(width * 1.6) : (float)fmin(half_span * 0.6, 4.0);
  float ridge_y = h + rise;
  vec3_t e00 = gen_rectangle_point(&r, a0, c0, h),
         e10 = gen_rectangle_point(&r, a1, c0, h);
  vec3_t e11 = gen_rectangle_point(&r, a1, c1, h),
         e01 = gen_rectangle_point(&r, a0, c1, h);
  vec3_t r0 = gen_rectangle_point(&r, a0 + inset, mid, ridge_y);
  vec3_t r1 = gen_rectangle_point(&r, a1 - inset, mid, ridge_y);
  float s = 0.4f;
  uint32_t base = (uint32_t)roof->mesh.vertex_count;
  vec3_t corners[6] = {e00, e10, e11, e01, r0, r1};
  double us[6] = {a0, a1, a1, a0, a0 + inset, a1 - inset};
  double vs[6] = {c0, c0, c1, c1, mid, mid};
  for (int i = 0; i < 6; i++)
    if (!gen_add_vertex(&roof->mesh, corners[i], vec3(0, 1, 0),
                        (float)(us[i] * s), (float)(vs[i] * s)))
      return false;
  vec3_t up = vec3(0, 1, 0);
  /* Two long slopes, two hips, and the underside of the eaves facing down. */
  if (!gen_add_face(&roof->mesh, base + 0, base + 1, base + 5, up) ||
      !gen_add_face(&roof->mesh, base + 0, base + 5, base + 4, up) ||
      !gen_add_face(&roof->mesh, base + 2, base + 3, base + 4, up) ||
      !gen_add_face(&roof->mesh, base + 2, base + 4, base + 5, up))
    return false;
  if (!gabled &&
      (!gen_add_face(&roof->mesh, base + 1, base + 2, base + 5, up) ||
       !gen_add_face(&roof->mesh, base + 3, base + 0, base + 4, up)))
    return false;
  if (gabled) {
    /* The gable triangles stand on the end walls, facing out along the
     * axis, textured like the wall below. */
    vec3_t ends[2][3] = {{e10, e11, r1}, {e01, e00, r0}};
    for (int end = 0; end < 2; end++) {
      vec3_t outward = vec3((float)r.axis.x, 0, (float)r.axis.z);
      if (end == 1) outward = vec3_scale(outward, -1);
      uint32_t g = (uint32_t)walls->mesh.vertex_count;
      for (int i = 0; i < 3; i++) {
        float u = (float)(((i == 0)   ? c0
                           : (i == 1) ? c1
                                      : mid) /
                          o->window_width);
        float v = (ends[end][i].y) / o->storey_height;
        if (!gen_add_vertex(&walls->mesh, ends[end][i], outward, u, v))
          return false;
      }
      if (!gen_add_face(&walls->mesh, g, g + 1, g + 2, outward)) return false;
    }
  }
  uint32_t under = (uint32_t)roof->mesh.vertex_count;
  vec3_t down = vec3(0, -1, 0);
  for (int i = 0; i < 4; i++)
    if (!gen_add_vertex(&roof->mesh, corners[i], down, (float)(us[i] * s),
                        (float)(vs[i] * s)))
      return false;
  if (!gen_add_face(&roof->mesh, under + 0, under + 1, under + 2, down) ||
      !gen_add_face(&roof->mesh, under + 0, under + 2, under + 3, down))
    return false;
  *done = true;
  return true;
}

/* One roof triangle over the footprint, cut along the ridge so the two
 * slopes meet in a crisp line, lifted onto the roof surface. */
static bool add_ridge_face(city_part_t* roof, const local_point_t* poly,
                           size_t count, float eaves, const ridge_t* ridge) {
  for (size_t i = 1; i + 1 < count; i++) {
    local_point_t corners[3] = {poly[0], poly[i], poly[i + 1]};
    if (fabs(polygon_signed_area(corners, 3)) < 1e-7) continue;
    uint32_t base = (uint32_t)roof->mesh.vertex_count;
    for (int k = 0; k < 3; k++) {
      double c = ridge_across(ridge, corners[k]) - ridge->mid;
      double sign = c < 0 ? -1.0 : 1.0;
      /* A corner on the ridge takes the side its triangle lies on. */
      if (fabs(c) < 1e-6) {
        double sum = 0;
        for (int m = 0; m < 3; m++)
          sum += ridge_across(ridge, corners[m]) - ridge->mid;
        sign = sum < 0 ? -1.0 : 1.0;
      }
      vec3_t normal = vec3_normalize(
          vec3((float)(sign * ridge->slope * ridge->across.x), 1.0f,
               (float)(sign * ridge->slope * ridge->across.z)));
      float y = (float)ridge_height(ridge, eaves, corners[k]);
      if (!gen_add_vertex(&roof->mesh, gen_at(corners[k], y), normal,
                          (float)(corners[k].x * 0.4),
                          (float)(corners[k].z * 0.4)))
        return false;
    }
    if (!gen_add_face(&roof->mesh, base, base + 1, base + 2, vec3(0, 1, 0)))
      return false;
  }
  return true;
}

static bool add_ridge_triangle(city_part_t* roof, const local_point_t* tri,
                               float eaves, const ridge_t* ridge) {
  double side[3];
  bool crosses = false;
  for (int k = 0; k < 3; k++) {
    side[k] = ridge_across(ridge, tri[k]) - ridge->mid;
  }
  for (int k = 0; k < 3; k++)
    if (side[k] * side[(k + 1) % 3] < -1e-9) crosses = true;
  if (!crosses) return add_ridge_face(roof, tri, 3, eaves, ridge);
  /* Clip the triangle against the ridge line into a negative-side and a
   * positive-side polygon (each of three or four corners). */
  local_point_t negative[4], positive[4];
  size_t nn = 0, np = 0;
  for (int k = 0; k < 3; k++) {
    local_point_t a = tri[k], b = tri[(k + 1) % 3];
    double sa = side[k], sb = side[(k + 1) % 3];
    if (sa <= 0) negative[nn++] = a;
    if (sa >= 0) positive[np++] = a;
    if (sa * sb < -1e-9) {
      double t = sa / (sa - sb);
      local_point_t m = {a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t};
      negative[nn++] = m;
      positive[np++] = m;
    }
  }
  return add_ridge_face(roof, negative, nn, eaves, ridge) &&
         add_ridge_face(roof, positive, np, eaves, ridge);
}

/* A building stands level at the ground height of its centre. */
static void place_on_terrain(const layout_t* layout,
                             const city_building_t* building,
                             city_part_t* walls, city_part_t* roof) {
  if (!layout->terrain || layout->terrain->cols == 0) return;
  local_point_t c = gen_centroid(building->points, building->point_count);
  float base = (float)gen_terrain_at(layout, c.x, c.z);
  gen_lift_part(walls, base);
  gen_lift_part(roof, base);
}

/* Puts the finished building on the ground and keeps its parts. The
 * walkway part is dropped when no edge needed one. */
static void finish_building(layout_t* layout, const city_building_t* building,
                            city_chunk_t* chunk, city_part_t* walls,
                            city_part_t* roof) {
  place_on_terrain(layout, building, walls, roof);
  gen_finish_part(chunk, walls);
  gen_finish_part(chunk, roof);
  /* The walkway and the shop fronts are the last two parts; keep the ones
   * that have anything in them. */
  size_t n = chunk->part_count;
  city_part_t* walkway = &chunk->parts[n - 2];
  city_part_t* shops = &chunk->parts[n - 1];
  if (shops->mesh.vertex_count == 0) {
    mesh_data_free(&shops->mesh);
    chunk->part_count--;
  } else {
    float base = 0;
    if (layout->terrain && layout->terrain->cols > 0) {
      local_point_t c = gen_centroid(building->points, building->point_count);
      base = (float)gen_terrain_at(layout, c.x, c.z);
    }
    for (size_t v = 0; v < shops->mesh.vertex_count; v++)
      shops->mesh.vertices[v].position.y += base;
    gen_finish_part(chunk, shops);
  }
  if (walkway->mesh.vertex_count == 0) {
    mesh_data_free(&walkway->mesh);
    /* With shops they move down into its place; without, it was the last
     * part left. */
    if (chunk->part_count == n) chunk->parts[n - 2] = chunk->parts[n - 1];
    chunk->part_count--;
  } else {
    gen_finish_part(chunk, walkway);
  }
}

static bool name_has(const char* name, const char* word) {
  size_t n = strlen(word);
  for (const char* c = name; *c; c++) {
    size_t k = 0;
    while (k < n && c[k] && tolower((unsigned char)c[k]) == word[k]) k++;
    if (k == n) return true;
  }
  return false;
}

static landmark_t landmark_for(const city_building_t* building) {
  if (name_has(building->name, "municipio")) return LANDMARK_TOWN_HALL;
  if (name_has(building->name, "chiesa") || name_has(building->name, "duomo"))
    return LANDMARK_CHURCH;
  return LANDMARK_NONE;
}

/* A square tower from the ground to `height`, ending in a four-sided spire
 * that rises `spire` further. Walls go into `walls`, the spire into `roof`. */
static bool add_tower(city_part_t* walls, city_part_t* roof,
                      local_point_t centre, vec2_t axis, double half,
                      float height, float spire, float skirt,
                      const city_generator_options_t* o) {
  vec2_t across = gen_v2(-axis.z, axis.x);
  local_point_t corner[4];
  double signs[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
  for (int k = 0; k < 4; k++) {
    corner[k].x =
        centre.x + axis.x * half * signs[k][0] + across.x * half * signs[k][1];
    corner[k].z =
        centre.z + axis.z * half * signs[k][0] + across.z * half * signs[k][1];
  }
  for (int k = 0; k < 4; k++) {
    local_point_t p = corner[k], q = corner[(k + 1) % 4];
    vec2_t d = gen_v2_normalize(gen_v2_sub(q, p));
    vec3_t normal = vec3((float)-d.z, 0, (float)d.x);
    float u = (float)(2 * half / o->window_width);
    if (!gen_add_wall_quad(walls, p, q, -skirt, height, normal, 0, u,
                           -skirt / o->storey_height,
                           height / o->storey_height))
      return false;
  }
  /* The spire: a pyramid from slightly overhanging eaves to the apex. */
  vec3_t apex = vec3((float)centre.x, height + spire, (float)centre.z);
  for (int k = 0; k < 4; k++) {
    local_point_t p = corner[k], q = corner[(k + 1) % 4];
    local_point_t mid = {(p.x + q.x) * 0.5, (p.z + q.z) * 0.5};
    vec3_t normal = vec3_normalize(vec3((float)(mid.x - centre.x),
                                        spire / (float)half * 0.5f,
                                        (float)(mid.z - centre.z)));
    uint32_t base = (uint32_t)roof->mesh.vertex_count;
    if (!gen_add_vertex(&roof->mesh, gen_at(p, height), normal, 0, 0) ||
        !gen_add_vertex(&roof->mesh, gen_at(q, height), normal, 1, 0) ||
        !gen_add_vertex(&roof->mesh, apex, normal, 0.5f, 1))
      return false;
    if (!gen_add_face(&roof->mesh, base, base + 1, base + 2, normal))
      return false;
  }
  return true;
}

/* The nearest road distance from a point, or a large number. */
static double distance_to_roads(const city_model_t* model, local_point_t p) {
  double nearest = 1e30;
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* road = &model->roads[r];
    if (road->class == ROAD_CLASS_WATERWAY) continue;
    for (size_t k = 1; k < road->point_count; k++)
      nearest = fmin(nearest, gen_point_segment_distance(p, road->points[k - 1],
                                                         road->points[k]));
  }
  return nearest;
}

/* Adds the landmark's tower to the building's parts, returning the centre
 * and the top height of its clock (or 0 for none) through the outputs. */
static bool add_landmark(layout_t* layout, const city_building_t* building,
                         city_part_t* walls, city_part_t* roof, float skirt,
                         landmark_t kind, local_point_t* tower_centre,
                         vec2_t* tower_axis, float* clock_height) {
  const city_generator_options_t* o = layout->options;
  bounding_rectangle_t r =
      gen_bounding_rectangle(building->points, building->point_count);
  double mid_along = 0.5 * (r.along_min + r.along_max);
  double mid_across = 0.5 * (r.across_min + r.across_max);
  double half = kind == LANDMARK_TOWN_HALL ? 3.2 : 2.7;
  /* The side or end that faces the nearer street. */
  local_point_t a, b;
  double inset = half + 0.3;
  if (kind == LANDMARK_TOWN_HALL) {
    a = (local_point_t){
        r.axis.x * mid_along + r.across.x * (r.across_min + inset),
        r.axis.z * mid_along + r.across.z * (r.across_min + inset)};
    b = (local_point_t){
        r.axis.x * mid_along + r.across.x * (r.across_max - inset),
        r.axis.z * mid_along + r.across.z * (r.across_max - inset)};
  } else {
    a = (local_point_t){
        r.axis.x * (r.along_min + inset) + r.across.x * mid_across,
        r.axis.z * (r.along_min + inset) + r.across.z * mid_across};
    b = (local_point_t){
        r.axis.x * (r.along_max - inset) + r.across.x * mid_across,
        r.axis.z * (r.along_max - inset) + r.across.z * mid_across};
  }
  local_point_t centre =
      distance_to_roads(layout->model, a) <= distance_to_roads(layout->model, b)
          ? a
          : b;
  float top = kind == LANDMARK_TOWN_HALL ? 26.0f : 30.0f;
  float spire = kind == LANDMARK_TOWN_HALL ? 7.0f : 9.0f;
  *tower_centre = centre;
  *tower_axis = r.axis;
  *clock_height = kind == LANDMARK_TOWN_HALL ? top - 4.5f : 0.0f;
  return add_tower(walls, roof, centre, r.axis, half, top, spire, skirt, o);
}

/* Four clock faces, one a side, drawn glowing like the lamps at night. */
static bool add_clock_faces(layout_t* layout, const city_building_t* building,
                            city_chunk_t* chunk, local_point_t centre,
                            vec2_t axis, float height) {
  city_part_t* clock =
      city_chunk_add_part(chunk, "lamp_light", PART_COLLIDER_NONE, 0, 0,
                          (uint64_t)building->source_id);
  if (!clock) return false;
  double half = 3.2 + 0.03;
  vec2_t across = gen_v2(-axis.z, axis.x);
  for (int k = 0; k < 4; k++) {
    double angle = k * M_PI * 0.5;
    vec2_t out = gen_v2(axis.x * cos(angle) + across.x * sin(angle),
                        axis.z * cos(angle) + across.z * sin(angle));
    vec2_t side = gen_v2(-out.z, out.x);
    local_point_t face = {centre.x + out.x * half, centre.z + out.z * half};
    local_point_t p = gen_offset(face, side, -1.2),
                  q = gen_offset(face, side, 1.2);
    vec3_t normal = vec3((float)out.x, 0, (float)out.z);
    if (!gen_add_wall_quad(clock, p, q, height - 1.2f, height + 1.2f, normal, 0,
                           1, 0, 1))
      return false;
  }
  float base = layout->terrain ? (float)city_terrain_height(layout->terrain,
                                                            centre.x, centre.z)
                               : 0.0f;
  for (size_t v = 0; v < clock->mesh.vertex_count; v++)
    clock->mesh.vertices[v].position.y += base;
  gen_finish_part(chunk, clock);
  return true;
}

/* A brick chimney on a pitched roof, standing on the slope a little off
 * the ridge and rising above it. About four buildings in ten have one. */
static bool add_chimney(layout_t* layout, const city_building_t* building,
                        city_chunk_t* chunk, float h) {
  unsigned hash = (unsigned)((uint64_t)building->source_id * 1103515245u >> 5);
  if (hash % 100 >= 42) return true;
  bounding_rectangle_t r =
      gen_bounding_rectangle(building->points, building->point_count);
  double half_span = (r.across_max - r.across_min) * 0.5;
  float rise = (float)fmin(half_span * 0.6, 4.0);
  double along = r.along_min + (r.along_max - r.along_min) *
                                   (0.25 + 0.5 * ((hash >> 8) % 100) / 100.0);
  double side = ((hash >> 16) & 1) ? 0.35 : -0.35;
  double across = 0.5 * (r.across_min + r.across_max) + side * half_span;
  local_point_t p = {r.axis.x * along + r.across.x * across,
                     r.axis.z * along + r.across.z * across};
  float base = h + rise * 0.55f, top = h + rise + 1.3f;
  city_part_t* part = city_chunk_add_part(chunk, "chimney", PART_COLLIDER_NONE,
                                          0, 0, (uint64_t)building->source_id);
  if (!part) return false;
  if (!mesh_data_add_box(&part->mesh,
                         vec3((float)p.x, 0.5f * (base + top), (float)p.z),
                         vec3(0.28f, 0.5f * (top - base), 0.28f)))
    return false;
  float lift =
      layout->terrain && layout->terrain->cols > 0
          ? (float)gen_terrain_at(
                layout, gen_centroid(building->points, building->point_count).x,
                gen_centroid(building->points, building->point_count).z)
          : 0.0f;
  for (size_t v = 0; v < part->mesh.vertex_count; v++)
    part->mesh.vertices[v].position.y += lift;
  gen_finish_part(chunk, part);
  return true;
}

/* Street doors: on the walls that face a street at pavement distance, a
 * door a metre and a quarter wide, one in the middle of a short wall and two
 * on a long one. About seven buildings in ten. */
static bool add_doors(layout_t* layout, const city_building_t* building,
                      city_chunk_t* chunk, const bool* skip) {
  unsigned hash = (unsigned)((uint64_t)building->source_id * 2654435761u >> 11);
  if (hash % 100 >= 70) return true;
  const city_model_t* model = layout->model;
  city_part_t* part = NULL;
  size_t count = building->point_count;
  float lift = 0;
  if (layout->terrain && layout->terrain->cols > 0) {
    local_point_t c = gen_centroid(building->points, count);
    lift = (float)gen_terrain_at(layout, c.x, c.z);
  }
  for (size_t i = 0; i < count; i++) {
    if (skip && skip[i]) continue;
    local_point_t a = building->points[i],
                  b = building->points[(i + 1) % count];
    double length = hypot(b.x - a.x, b.z - a.z);
    if (length < 5.5) continue;
    local_point_t middle = {(a.x + b.x) * 0.5, (a.z + b.z) * 0.5};
    vec2_t direction = gen_v2_normalize(gen_v2_sub(b, a));
    vec2_t out =
        gen_v2(-direction.z, direction.x); /* outward, as the walls' normals */
    local_point_t ahead = gen_offset(middle, out, 3.0);
    double distance = 1e30;
    for (size_t r = 0; r < model->road_count; r++) {
      const city_road_t* road = &model->roads[r];
      if (road->class == ROAD_CLASS_WATERWAY ||
          road->class == ROAD_CLASS_MOTORWAY)
        continue;
      for (size_t k = 1; k < road->point_count; k++)
        distance =
            fmin(distance, gen_point_segment_distance(
                               ahead, road->points[k - 1], road->points[k]));
    }
    if (distance > 11.0) continue;
    int doors = length > 15.0 ? 2 : 1;
    for (int d = 0; d < doors; d++) {
      double centre = doors == 1 ? length * 0.5 : length * (d == 0 ? 0.3 : 0.7);
      double shift = ((double)((hash >> (d * 4)) % 5) - 2.0) * 0.25;
      local_point_t at = gen_offset(a, direction, centre + shift);
      if (!part) {
        part = city_chunk_add_part(chunk, "door", PART_COLLIDER_NONE, 0, 0,
                                   (uint64_t)building->source_id);
        if (!part) return false;
      }
      local_point_t p = gen_offset(gen_offset(at, out, 0.025), direction, -0.6);
      local_point_t q = gen_offset(gen_offset(at, out, 0.025), direction, 0.6);
      vec3_t normal = vec3((float)out.x, 0, (float)out.z);
      if (!gen_add_wall_quad(part, p, q, lift + 0.0f, lift + 2.4f, normal, 0, 1,
                             1, 0))
        return false;
    }
  }
  if (!part) return true;
  gen_finish_part(chunk, part);
  return true;
}

/* Finishes the building's parts, with its landmark tower and clock. */
static bool finish_landmark_building(layout_t* layout,
                                     const city_building_t* building,
                                     city_chunk_t* chunk, city_part_t* walls,
                                     city_part_t* roof, landmark_t landmark,
                                     float skirt) {
  local_point_t centre = {0, 0};
  vec2_t axis = gen_v2(1, 0);
  float clock_height = 0;
  if (landmark != LANDMARK_NONE &&
      !add_landmark(layout, building, walls, roof, skirt, landmark, &centre,
                    &axis, &clock_height))
    return false;
  finish_building(layout, building, chunk, walls, roof);
  if (landmark == LANDMARK_NONE && !add_doors(layout, building, chunk, NULL))
    return false;
  if (landmark == LANDMARK_NONE) {
    roof_shape_t shape = roof_shape(building);
    if ((shape == ROOF_HIPPED || shape == ROOF_GABLED) &&
        !add_chimney(layout, building, chunk, building->height))
      return false;
  }
  if (clock_height > 0 &&
      !add_clock_faces(layout, building, chunk, centre, axis, clock_height))
    return false;
  return true;
}

bool generate_building(layout_t* layout, const city_building_t* building,
                       city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  size_t count = building->point_count;
  city_chunk_t* chunk =
      gen_chunk_at(layout, gen_centroid(building->points, count));
  char facade[32];
  unsigned variant = (unsigned)((uint64_t)building->source_id * 2654435761u %
                                (unsigned)o->facade_variants);
  snprintf(facade, sizeof(facade), "facade%u", variant);
  bool on_the_arcade = false;
  for (size_t i = 0; i < count && !on_the_arcade; i++)
    on_the_arcade = gen_edge_faces_arcade(layout->model, o, building, i,
                                          gen_building_depth(building));
  if (on_the_arcade) snprintf(facade, sizeof(facade), "facade_fresco");
  landmark_t landmark = landmark_for(building);
  if (landmark == LANDMARK_TOWN_HALL)
    snprintf(facade, sizeof(facade), "facade_brick");
  city_part_t* walls = city_chunk_add_part(
      chunk, facade, PART_COLLIDER_MESH, COLLISION_LAYER_BUILDING,
      o->building_friction, (uint64_t)building->source_id);
  unsigned roof_hash =
      (unsigned)((uint64_t)building->source_id * 2246822519u >> 7);
  const char* roof_material = roof_shape(building) == ROOF_FLAT ? "roof_flat"
                              : roof_hash % 10 < 7              ? "roof"
                                                                : "roof_slate";
  city_part_t* roof =
      walls
          ? city_chunk_add_part(chunk, roof_material, PART_COLLIDER_MESH,
                                COLLISION_LAYER_BUILDING, o->building_friction,
                                (uint64_t)building->source_id)
          : NULL;
  city_part_t* walkway =
      roof ? city_chunk_add_part(chunk, "paving", PART_COLLIDER_MESH,
                                 COLLISION_LAYER_GROUND, o->pavement_friction,
                                 (uint64_t)building->source_id)
           : NULL;
  city_part_t* shops =
      walkway
          ? city_chunk_add_part(chunk, "shopfront", PART_COLLIDER_MESH,
                                COLLISION_LAYER_BUILDING, o->building_friction,
                                (uint64_t)building->source_id)
          : NULL;
  if (!walls || !roof || !walkway || !shops) return false;
  /* Adding parts may have moved the array: walls, roof, walkway and shops are
   * the last four. */
  walls = &chunk->parts[chunk->part_count - 4];
  roof = &chunk->parts[chunk->part_count - 3];
  walkway = &chunk->parts[chunk->part_count - 2];
  shops = &chunk->parts[chunk->part_count - 1];
  bool* arcaded = calloc(count, sizeof(bool));
  bool* shop = calloc(count, sizeof(bool));
  if (!arcaded || !shop) {
    free(arcaded);
    free(shop);
    return false;
  }
  double depth = gen_building_depth(building);
  for (size_t i = 0; i < count; i++) {
    arcaded[i] = gen_edge_faces_arcade(layout->model, o, building, i, depth);
    shop[i] = landmark == LANDMARK_NONE &&
              edge_faces_shops(layout->model, building, i);
  }
  float h = building->height;
  ridge_t ridge_data;
  const ridge_t* ridge = NULL;
  if (roof_shape(building) == ROOF_RIDGE) {
    bounding_rectangle_t r = gen_bounding_rectangle(building->points, count);
    double half = (r.across_max - r.across_min) * 0.5;
    ridge_data.across = r.across;
    ridge_data.mid = (r.across_min + r.across_max) * 0.5;
    ridge_data.rise = fmin(fmax(half * 0.55, 0.8), 4.0);
    ridge_data.slope = ridge_data.rise / fmax(half, 0.5);
    ridge = &ridge_data;
  }
  /* On sloping ground the walls reach below the walkway so no gap shows. */
  float skirt = layout->terrain && layout->terrain->cols > 0 ? 3.0f : 0.0f;
  bool walls_ok = add_walls(layout, walls, walkway, shops, building->points,
                            count, h, ridge, skirt, arcaded, shop);
  free(arcaded);
  free(shop);
  if (!walls_ok) return false;
  for (size_t hole = 0; hole < building->hole_count; hole++)
    if (!add_walls(layout, walls, walkway, shops, building->holes[hole].points,
                   building->holes[hole].count, h, ridge, skirt, NULL, NULL))
      return false;
  bool pitched = false;
  if (!add_pitched_roof(roof, walls, building, h, o, &pitched)) return false;
  if (pitched) {
    if (!finish_landmark_building(layout, building, chunk, walls, roof,
                                  landmark, skirt))
      return false;
    stats->buildings++;
    return true;
  }
  size_t merged_capacity = count + 2 * building->hole_count;
  for (size_t hole = 0; hole < building->hole_count; hole++)
    merged_capacity += building->holes[hole].count;
  local_point_t* merged = malloc(merged_capacity * sizeof(local_point_t));
  uint32_t* indices = malloc(3 * merged_capacity * sizeof(uint32_t));
  const local_point_t** hole_points = calloc(
      building->hole_count ? building->hole_count : 1, sizeof(local_point_t*));
  size_t* hole_counts =
      calloc(building->hole_count ? building->hole_count : 1, sizeof(size_t));
  if (!merged || !indices || !hole_points || !hole_counts) {
    free(merged);
    free(indices);
    free(hole_points);
    free(hole_counts);
    return false;
  }
  for (size_t hole = 0; hole < building->hole_count; hole++) {
    hole_points[hole] = building->holes[hole].points;
    hole_counts[hole] = building->holes[hole].count;
  }
  size_t merged_count;
  bool clean;
  size_t triangles = triangulate_with_holes(
      building->points, count, hole_points, hole_counts, building->hole_count,
      merged, &merged_count, indices, &clean);
  if (!clean) {
    LOG_WARN("generate",
             "building %lld has an irregular footprint; roof approximated",
             (long long)building->source_id);
    stats->buildings_unclean++;
  }
  bool roof_ok = true;
  if (ridge) {
    for (size_t t = 0; t < triangles && roof_ok; t++) {
      local_point_t tri[3] = {merged[indices[t * 3]],
                              merged[indices[t * 3 + 1]],
                              merged[indices[t * 3 + 2]]};
      if (fabs(polygon_signed_area(tri, 3)) < 1e-6) continue;
      roof_ok = add_ridge_triangle(roof, tri, h, ridge);
    }
  } else {
    for (size_t i = 0; i < merged_count && roof_ok; i++)
      roof_ok = gen_add_vertex(&roof->mesh, gen_at(merged[i], h), vec3(0, 1, 0),
                               (float)(merged[i].x * 0.4),
                               (float)(merged[i].z * 0.4));
    for (size_t t = 0; t < triangles && roof_ok; t++) {
      local_point_t tri[3] = {merged[indices[t * 3]],
                              merged[indices[t * 3 + 1]],
                              merged[indices[t * 3 + 2]]};
      if (fabs(polygon_signed_area(tri, 3)) < 1e-6) continue;
      roof_ok = gen_add_face(&roof->mesh, indices[t * 3], indices[t * 3 + 1],
                             indices[t * 3 + 2], vec3(0, 1, 0));
    }
  }
  free(merged);
  free(indices);
  free(hole_points);
  free(hole_counts);
  if (!roof_ok) return false;
  if (!finish_landmark_building(layout, building, chunk, walls, roof, landmark,
                                skirt))
    return false;
  stats->buildings++;
  return true;
}
