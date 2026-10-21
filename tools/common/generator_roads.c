/* Roads, junctions, pavements, markings, signs, lamps and bridges. */

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

static bool in_focus(focus_t focus, local_point_t sample) {
  return focus.reach <= 0 || hypot(sample.x - focus.point.x,
                                   sample.z - focus.point.z) <= focus.reach;
}

/* The nearest building edge to the road's centre line, sampled every two
 * metres, or a large number when nothing is near. */
static double corridor_half_width(const city_model_t* model,
                                  const city_road_t* road, double reach,
                                  focus_t focus) {
  double min_x = 1e30, max_x = -1e30, min_z = 1e30, max_z = -1e30;
  for (size_t i = 0; i < road->point_count; i++) {
    min_x = fmin(min_x, road->points[i].x),
    max_x = fmax(max_x, road->points[i].x);
    min_z = fmin(min_z, road->points[i].z),
    max_z = fmax(max_z, road->points[i].z);
  }
  if (focus.reach > 0) {
    min_x = fmax(min_x, focus.point.x - focus.reach);
    max_x = fmin(max_x, focus.point.x + focus.reach);
    min_z = fmax(min_z, focus.point.z - focus.reach);
    max_z = fmin(max_z, focus.point.z + focus.reach);
  }
  double nearest = 1e30;
  for (size_t b = 0; b < model->building_count; b++) {
    const city_building_t* building = &model->buildings[b];
    bool near_road = false;
    for (size_t i = 0; i < building->point_count && !near_road; i++) {
      local_point_t p = building->points[i];
      near_road = p.x > min_x - reach && p.x < max_x + reach &&
                  p.z > min_z - reach && p.z < max_z + reach;
    }
    if (!near_road) continue;
    for (size_t i = 0; i < building->point_count; i++) {
      local_point_t a = building->points[i];
      local_point_t c = building->points[(i + 1) % building->point_count];
      for (size_t r = 1; r < road->point_count; r++) {
        local_point_t p = road->points[r - 1], q = road->points[r];
        double length = hypot(q.x - p.x, q.z - p.z);
        int samples = (int)(length / 2.0) + 1;
        for (int k = 0; k <= samples; k++) {
          double t = (double)k / (double)samples;
          local_point_t sample = {p.x + (q.x - p.x) * t, p.z + (q.z - p.z) * t};
          if (!in_focus(focus, sample)) continue;
          double d = gen_point_segment_distance(sample, a, c);
          if (d < nearest) nearest = d;
        }
      }
    }
  }
  return nearest;
}

/* Half the gap to the nearest other road's centre line that shares no
 * node with this one (paths and streams excepted), or a large number. */
static double corridor_to_roads(const city_model_t* model,
                                const city_road_t* road, double reach,
                                focus_t focus) {
  double nearest = 1e30;
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* other = &model->roads[r];
    if (other == road || other->class == ROAD_CLASS_PATH ||
        other->class == ROAD_CLASS_WATERWAY)
      continue;
    if (focus.reach > 0) {
      bool near = false;
      for (size_t k = 1; k < other->point_count && !near; k++)
        near =
            gen_point_segment_distance(focus.point, other->points[k - 1],
                                       other->points[k]) < focus.reach + reach;
      if (!near) continue;
    }
    bool shares_node = false;
    for (size_t i = 0; i < road->point_count && !shares_node; i++)
      for (size_t k = 0; k < other->point_count && !shares_node; k++)
        if (road->node_ids[i] != 0 && road->node_ids[i] == other->node_ids[k])
          shares_node = true;
    if (shares_node) continue;
    for (size_t i = 1; i < road->point_count; i++) {
      local_point_t p = road->points[i - 1], q = road->points[i];
      vec2_t direction = gen_v2_normalize(gen_v2_sub(q, p));
      double length = hypot(q.x - p.x, q.z - p.z);
      int samples = (int)(length / 3.0) + 1;
      for (int k = 0; k <= samples; k++) {
        double t = (double)k / (double)samples;
        local_point_t sample = {p.x + (q.x - p.x) * t, p.z + (q.z - p.z) * t};
        if (!in_focus(focus, sample)) continue;
        for (size_t m = 1; m < other->point_count; m++) {
          local_point_t a = other->points[m - 1], b = other->points[m];
          vec2_t other_direction = gen_v2_normalize(gen_v2_sub(b, a));
          /* Only a road running alongside counts: nearly parallel, and
           * the sample lies beside the segment rather than past its end. */
          double cosine =
              direction.x * other_direction.x + direction.z * other_direction.z;
          if (fabs(cosine) < 0.9) continue;
          double abx = b.x - a.x, abz = b.z - a.z;
          double length2 = abx * abx + abz * abz;
          if (length2 < 1e-9) continue;
          double u =
              ((sample.x - a.x) * abx + (sample.z - a.z) * abz) / length2;
          if (u < 0 || u > 1) continue;
          double d = gen_point_segment_distance(sample, a, b);
          if (d < reach && d < nearest) nearest = d;
        }
      }
    }
  }
  return nearest * 0.5;
}

static road_fit_t fit_at(const city_model_t* model, const city_road_t* road,
                         const city_generator_options_t* o, focus_t focus) {
  road_fit_t fit;
  fit.half_width = road->width * 0.5;
  bool has_pavement = o->pavements && road->class != ROAD_CLASS_SERVICE &&
                      road->class != ROAD_CLASS_PEDESTRIAN &&
                      road->class != ROAD_CLASS_PATH &&
                      road->class != ROAD_CLASS_WATERWAY;
  fit.pavement = has_pavement ? o->pavement_width : 0;
  if (road->class == ROAD_CLASS_PATH || road->class == ROAD_CLASS_WATERWAY)
    return fit;
  const double gap = 0.3, narrowest = 1.75;
  /* Another road close alongside: keep this road's width and give up
   * the pavement on the gap, so two carriageways meet at a median. */
  double to_roads = corridor_to_roads(
      model, road, 2.0 * (fit.half_width + fit.pavement + 1.0), focus);
  if (to_roads < fit.half_width + fit.pavement + gap) {
    double available = fmax(to_roads - 0.1, narrowest);
    fit.half_width = fmin(fit.half_width, available);
    fit.pavement = fmin(fmax(available - fit.half_width, 0.0), fit.pavement);
    if (fit.pavement < 0.4) fit.pavement = 0;
  }
  double wanted = fit.half_width + fit.pavement + gap;
  double corridor = corridor_half_width(model, road, wanted + 1.0, focus);
  if (corridor >= wanted) return fit;
  double available = fmax(corridor - gap, narrowest);
  fit.half_width =
      fmin(fit.half_width, fmax(narrowest, available - fit.pavement));
  fit.pavement = fmin(fmax(available - fit.half_width, 0.0), fit.pavement);
  if (fit.pavement < 0.4) fit.pavement = 0;
  return fit;
}

/* The fit the junctions see: the tighter of the road's two ends. */
road_fit_t gen_fit_road(const city_model_t* model, const city_road_t* road,
                        const city_generator_options_t* o) {
  focus_t start = {road->points[0], FIT_REACH};
  focus_t end = {road->points[road->point_count - 1], FIT_REACH};
  road_fit_t a = fit_at(model, road, o, start), b = fit_at(model, road, o, end);
  road_fit_t fit = {fmin(a.half_width, b.half_width),
                    fmin(a.pavement, b.pavement)};
  return fit;
}

/* Widths along a road: each point fitted to what stands near it, never
 * wider than the fit at the ends where the junction geometry was laid
 * out, and never changing faster than a gentle taper. A road with no
 * pavement at its ends has none anywhere; one with pavements keeps at
 * least a sliver so the strips never collapse. */
static void fit_along(const city_model_t* model, const city_road_t* road,
                      const city_generator_options_t* o, road_fit_t ends,
                      const local_point_t* points, size_t count, double* halves,
                      double* pavements) {
  for (size_t i = 0; i < count; i++) {
    focus_t focus = {points[i], FIT_REACH};
    road_fit_t fit = fit_at(model, road, o, focus);
    halves[i] = fit.half_width;
    pavements[i] = ends.pavement > 0 ? fmax(fit.pavement, 0.3) : 0;
  }
  halves[0] = ends.half_width, halves[count - 1] = ends.half_width;
  pavements[0] = ends.pavement, pavements[count - 1] = ends.pavement;
  const double taper = 0.15; /* metres of width per metre of road */
  for (int pass = 0; pass < 2; pass++) {
    for (size_t i = 1; i < count; i++) {
      size_t k = pass == 0 ? i : count - 1 - i,
             prev = pass == 0 ? k - 1 : k + 1;
      double d =
          hypot(points[k].x - points[prev].x, points[k].z - points[prev].z);
      halves[k] = fmin(halves[k], halves[prev] + taper * d);
      double outer = fmin(halves[k] + pavements[k],
                          halves[prev] + pavements[prev] + taper * d);
      if (pavements[k] > 0) pavements[k] = fmax(outer - halves[k], 0.3);
    }
  }
}

/* A kerb face along a polyline of points on the road edge, facing the
 * road (which is on the `road_side` of the line's direction). */
static bool add_kerb_line(mesh_data_t* mesh, const local_point_t* points,
                          size_t count, float y0, float y1,
                          local_point_t road_point) {
  for (size_t i = 1; i < count; i++) {
    local_point_t a = points[i - 1], b = points[i];
    if (hypot(b.x - a.x, b.z - a.z) < 1e-4) continue;
    vec2_t d = gen_v2_normalize(gen_v2_sub(b, a));
    vec2_t r = gen_v2_right(d);
    /* Face the kerb towards the road. */
    vec2_t to_road = gen_v2_normalize(gen_v2_sub(road_point, a));
    bool road_on_right = to_road.x * r.x + to_road.z * r.z > 0;
    vec3_t normal = vec3((float)(road_on_right ? r.x : -r.x), 0,
                         (float)(road_on_right ? r.z : -r.z));
    uint32_t base = (uint32_t)mesh->vertex_count;
    if (!gen_add_vertex(mesh, gen_at(a, y0), normal, 0, 0) ||
        !gen_add_vertex(mesh, gen_at(b, y0), normal, 1, 0) ||
        !gen_add_vertex(mesh, gen_at(b, y1), normal, 1, 1) ||
        !gen_add_vertex(mesh, gen_at(a, y1), normal, 0, 1))
      return false;
    if (!gen_add_face(mesh, base, base + 1, base + 2, normal) ||
        !gen_add_face(mesh, base, base + 2, base + 3, normal))
      return false;
  }
  return true;
}

/* A flat white quad across a road arm: from `from` to `to` metres out
 * along the arm, between lateral offsets `left` and `right`. */
bool gen_add_marking_quad(mesh_data_t* mesh, local_point_t centre,
                          vec2_t direction, double from, double to, double left,
                          double right, float y) {
  vec2_t side = gen_v2_right(direction);
  vec3_t a =
      gen_at(gen_offset(gen_offset(centre, direction, from), side, left), y);
  vec3_t b =
      gen_at(gen_offset(gen_offset(centre, direction, from), side, right), y);
  vec3_t c =
      gen_at(gen_offset(gen_offset(centre, direction, to), side, right), y);
  vec3_t d =
      gen_at(gen_offset(gen_offset(centre, direction, to), side, left), y);
  /* An arm with no direction, or a line narrower than it is long, would
   * make slivers: skip rather than paint a degenerate quad. */
  if (vec3_length(vec3_cross(vec3_sub(b, a), vec3_sub(d, a))) < 1e-4f)
    return true;
  uint32_t base = (uint32_t)mesh->vertex_count;
  vec3_t up = vec3(0, 1, 0);
  if (!gen_add_vertex(mesh, a, up, 0, 0) ||
      !gen_add_vertex(mesh, b, up, 1, 0) ||
      !gen_add_vertex(mesh, c, up, 1, 1) || !gen_add_vertex(mesh, d, up, 0, 1))
    return false;
  return gen_add_face(mesh, base, base + 1, base + 2, up) &&
         gen_add_face(mesh, base, base + 2, base + 3, up);
}

/* Stop lines and zebra crossings on every drivable arm of a junction of
 * three or more roads, just past the arm's cut: a line across the road,
 * then stripes where the arm has pavements for people to cross between. */
static int road_rank(road_class_t class) {
  switch (class) {
    case ROAD_CLASS_MOTORWAY:
      return 0;
    case ROAD_CLASS_PRIMARY:
      return 1;
    case ROAD_CLASS_SECONDARY:
      return 2;
    case ROAD_CLASS_TERTIARY:
      return 3;
    default:
      return 4;
  }
}

/* A stop sign at the end of each side street that meets a bigger road: a
 * pole on the pavement just short of the junction, on the right as the
 * driver sees it, carrying a red plate that faces the oncoming car. */
static bool generate_stop_signs(layout_t* layout, const city_model_t* model,
                                const city_junction_t* junction,
                                const junction_arm_t* arms,
                                const arm_source_t* sources, size_t arm_count,
                                const junction_layout_t* shape,
                                const road_fit_t* fits) {
  const city_generator_options_t* o = layout->options;
  int best = 99;
  for (size_t a = 0; a < arm_count; a++) {
    road_class_t c = model->roads[sources[a].road].class;
    if (c == ROAD_CLASS_PEDESTRIAN) continue;
    best = best < road_rank(c) ? best : road_rank(c);
  }
  if (arm_count < 3) return true;
  for (size_t a = 0; a < arm_count; a++) {
    const city_road_t* road = &model->roads[sources[a].road];
    if (road->class == ROAD_CLASS_PEDESTRIAN || road_rank(road->class) <= best)
      continue;
    if (polyline_length(road->points, road->point_count) < 25.0 ||
        fits[sources[a].road].pavement <= 0)
      continue;
    vec2_t d = {arms[a].direction.x, arms[a].direction.z};
    /* A car heading into the junction travels along -d; its right is
     * (dz, -dx) in this frame (x east, z south). */
    vec2_t right = gen_v2(d.z, -d.x);
    double half = fits[sources[a].road].half_width;
    local_point_t foot =
        gen_offset(gen_offset(junction->position, d, shape->cut[a] + 1.4),
                   right, half + 0.7);
    float y0 = o->kerb_height;
    /* Each sign is a pole of its own, so the car can knock it down. */
    city_chunk_t* chunk = gen_chunk_at(layout, foot);
    uint64_t source = gen_pole_source(layout, (uint64_t)junction->node_id);
    city_part_t* pole = city_chunk_add_part(chunk, "sign_pole",
                                            PART_COLLIDER_MESH,
                                            COLLISION_LAYER_PROP, 0.5f, source);
    if (!pole || !city_chunk_add_part(chunk, "sign_red", PART_COLLIDER_NONE, 0,
                                      0, source))
      return false;
    pole = &chunk->parts[chunk->part_count - 2];
    city_part_t* plate = &chunk->parts[chunk->part_count - 1];
    if (!mesh_data_add_box(&pole->mesh,
                           vec3((float)foot.x, y0 + 1.1f, (float)foot.z),
                           vec3(0.035f, 1.1f, 0.035f)))
      return false;
    /* The plate: thin across the heading, square on to the car. */
    vec3_t half_extents =
        vec3(0.3f * (float)fabs(right.x) + 0.015f * (float)fabs(d.x), 0.3f,
             0.3f * (float)fabs(right.z) + 0.015f * (float)fabs(d.z));
    if (!mesh_data_add_box(&plate->mesh,
                           vec3((float)foot.x, y0 + 2.0f, (float)foot.z),
                           half_extents))
      return false;
    gen_finish_part(chunk, pole);
    gen_finish_part(chunk, plate);
  }
  return true;
}

/* Whether a junction gets traffic lights: where three or more streets of
 * tertiary rank or better meet, or where a fourth street joins two of them.
 * A main road with a side street keeps its stop sign; a junction with a
 * mapped signal nearby keeps that instead. */
static bool junction_is_signalised(const city_model_t* model,
                                   const city_junction_t* junction,
                                   const arm_source_t* sources,
                                   size_t arm_count) {
  if (arm_count < 3) return false;
  int big = 0;
  for (size_t a = 0; a < arm_count; a++) {
    road_class_t c = model->roads[sources[a].road].class;
    if (c == ROAD_CLASS_PEDESTRIAN) continue;
    int rank = road_rank(c);
    if (rank <= 3) big++;
  }
  if (!(big >= 3 || (arm_count >= 4 && big >= 2))) return false;
  for (size_t f = 0; f < model->furniture_count; f++)
    if (model->furniture[f].kind == OSM_FURNITURE_SIGNALS &&
        hypot(model->furniture[f].x - junction->position.x,
              model->furniture[f].z - junction->position.z) < 30.0)
      return false;
  return true;
}

/* A signal head on the pavement at the entry of each street, facing the
 * traffic that comes in, in the phase group of its axis: streets running
 * east-west keep group 0, north-south group 1, so crossing streets take
 * turns. The traffic is told where each stands and which way it governs. */
static bool generate_junction_signals(
    layout_t* layout, const city_model_t* model,
    const city_junction_t* junction, const junction_arm_t* arms,
    const arm_source_t* sources, size_t arm_count,
    const junction_layout_t* shape, const road_fit_t* fits) {
  for (size_t a = 0; a < arm_count; a++) {
    const city_road_t* road = &model->roads[sources[a].road];
    if (road->class == ROAD_CLASS_PEDESTRIAN ||
        polyline_length(road->points, road->point_count) < 12.0)
      continue;
    vec2_t d = {arms[a].direction.x, arms[a].direction.z};
    vec2_t right = gen_v2(d.z, -d.x);
    double half = fits[sources[a].road].half_width;
    local_point_t foot =
        gen_offset(gen_offset(junction->position, d, shape->cut[a] + 1.2),
                   right, half + 0.8);
    int group = fabs(d.x) >= fabs(d.z) ? 0 : 1;
    if (!gen_add_signal_head(
            layout,
            vec3((float)foot.x, layout->options->kerb_height, (float)foot.z), d,
            false, group, (uint64_t)junction->node_id) ||
        !gen_register_signal(layout, foot, gen_v2(-d.x, -d.z), group))
      return false;
  }
  return true;
}

static bool generate_markings(layout_t* layout, const city_model_t* model,
                              const city_junction_t* junction,
                              const junction_arm_t* arms,
                              const arm_source_t* sources, size_t arm_count,
                              const junction_layout_t* shape,
                              const road_fit_t* fits) {
  const city_generator_options_t* o = layout->options;
  if (arm_count < 3) return true;
  city_chunk_t* chunk = gen_chunk_at(layout, junction->position);
  city_part_t* part = city_chunk_add_part(chunk, "marking", PART_COLLIDER_NONE,
                                          0, 0, (uint64_t)junction->node_id);
  if (!part) return false;
  float y = o->road_height + 0.004f;
  for (size_t a = 0; a < arm_count; a++) {
    const city_road_t* road = &model->roads[sources[a].road];
    if (road->class == ROAD_CLASS_PEDESTRIAN) continue;
    /* Short links between two close junctions get no paint at all, so
     * crossings are not stacked a few metres apart. */
    double road_length = polyline_length(road->points, road->point_count);
    if (road_length < 12.0) continue;
    double half = fits[sources[a].road].half_width;
    vec2_t direction = {arms[a].direction.x, arms[a].direction.z};
    double start = shape->cut[a] + 0.3;
    /* The stop line, on the entering side only: the right half for a
     * two-way road, the whole width for a one-way one. */
    if (!gen_add_marking_quad(&part->mesh, junction->position, direction, start,
                              start + 0.35, road->oneway ? -half + 0.3 : 0.15,
                              half - 0.3, y))
      return false;
    if (fits[sources[a].road].pavement <= 0 || road_length < 25.0) continue;
    double zebra_from = start + 0.9;
    for (double across = -half + 0.4; across + 0.5 <= half - 0.4; across += 1.0)
      if (!gen_add_marking_quad(&part->mesh, junction->position, direction,
                                zebra_from, zebra_from + 2.4, across,
                                across + 0.5, y))
        return false;
  }
  if (part->mesh.vertex_count == 0) {
    mesh_data_free(&part->mesh);
    chunk->part_count--;
  } else {
    gen_finish_part(chunk, part);
  }
  if (junction_is_signalised(model, junction, sources, arm_count))
    return generate_junction_signals(layout, model, junction, arms, sources,
                                     arm_count, shape, fits);
  return generate_stop_signs(layout, model, junction, arms, sources, arm_count,
                             shape, fits);
}

bool generate_junction(layout_t* layout, const city_model_t* model,
                       const city_junction_t* junction, const road_fit_t* fits,
                       road_cuts_t* cuts, city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  junction_arm_t arms[JUNCTION_MAX_ARMS];
  arm_source_t sources[JUNCTION_MAX_ARMS];
  size_t arm_count = 0;
  for (size_t r = 0; r < model->road_count && arm_count < JUNCTION_MAX_ARMS;
       r++) {
    const city_road_t* road = &model->roads[r];
    for (int end = 0; end < 2 && arm_count < JUNCTION_MAX_ARMS; end++) {
      size_t index = end ? road->point_count - 1 : 0;
      if (road->node_ids[index] != junction->node_id) continue;
      if (road->class == ROAD_CLASS_PATH || road->class == ROAD_CLASS_WATERWAY)
        continue; /* paths and streams lie on top or below */
      size_t neighbour = end ? road->point_count - 2 : 1;
      vec2_t direction = gen_v2_normalize(
          gen_v2_sub(road->points[neighbour], road->points[index]));
      arms[arm_count].direction.x = direction.x;
      arms[arm_count].direction.z = direction.z;
      arms[arm_count].half_width = fits[r].half_width;
      arms[arm_count].pavement = fits[r].pavement;
      sources[arm_count].road = r;
      sources[arm_count].at_start = end == 0;
      arm_count++;
    }
  }
  if (arm_count < 2) return true;
  junction_layout_t shape;
  bool patch =
      junction_layout(junction->position, arms, arm_count, 0.2, &shape);
  for (size_t a = 0; a < arm_count; a++) {
    road_cuts_t* cut = &cuts[sources[a].road];
    if (sources[a].at_start)
      cut->start = shape.cut[a];
    else
      cut->end = shape.cut[a];
  }
  if (!patch) return true;

  city_chunk_t* chunk = gen_chunk_at(layout, junction->position);
  bool all_pedestrian = true;
  for (size_t a = 0; a < arm_count; a++)
    if (model->roads[sources[a].road].class != ROAD_CLASS_PEDESTRIAN)
      all_pedestrian = false;
  city_part_t* part = city_chunk_add_part(
      chunk, all_pedestrian ? "paving" : "junction", PART_COLLIDER_MESH,
      COLLISION_LAYER_GROUND, o->road_friction, (uint64_t)junction->node_id);
  if (!part) return false;
  if (!gen_add_polygon_part(chunk, part, shape.polygon, shape.polygon_count,
                            o->road_height, 0.1f))
    return false;
  stats->junction_patches++;
  if (shape.corner_count == 0)
    return generate_markings(layout, model, junction, arms, sources, arm_count,
                             &shape, fits);
  city_part_t* pavement = city_chunk_add_part(
      chunk, "pavement", PART_COLLIDER_MESH, COLLISION_LAYER_GROUND,
      o->pavement_friction, (uint64_t)junction->node_id);
  city_part_t* kerb =
      pavement ? city_chunk_add_part(chunk, "kerb", PART_COLLIDER_NONE, 0, 0,
                                     (uint64_t)junction->node_id)
               : NULL;
  if (!pavement || !kerb) return false;
  pavement = &chunk->parts[chunk->part_count - 2];
  for (size_t c = 0; c < shape.corner_count; c++) {
    const pavement_corner_t* corner = &shape.corners[c];
    if (!gen_add_polygon_part(chunk, pavement, corner->polygon, corner->count,
                              o->kerb_height, 0.5f))
      return false;
    if (!add_kerb_line(&kerb->mesh, corner->kerb, corner->kerb_count,
                       o->road_height, o->kerb_height, junction->position))
      return false;
  }
  gen_finish_part(chunk, kerb);
  return generate_markings(layout, model, junction, arms, sources, arm_count,
                           &shape, fits);
}

/* Lamp posts on the pavement, alternating sides, every lamp_spacing
 * metres along the road: a post, an arm over the road and a head, with
 * a collider so the car can hit them. */
static bool generate_lamps(layout_t* layout, const city_road_t* road,
                           const local_point_t* points, const vec2_t* laterals,
                           size_t count, const double* halves,
                           const double* pavements,
                           city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  double length = polyline_length(points, count);
  if (length < o->lamp_spacing * 0.6) return true;
  const float height = 4.5f;
  int side = 1;
  for (double along = o->lamp_spacing * 0.5; along < length;
       along += o->lamp_spacing, side = -side) {
    size_t segment;
    local_point_t on_line = gen_along_polyline(points, count, along, &segment);
    vec2_t lateral = gen_v2_normalize(laterals[segment > 0 ? segment - 1 : 0]);
    size_t at_point = segment < count ? segment : count - 1;
    double post_offset = side * (halves[at_point] + pavements[at_point] * 0.5);
    local_point_t base = gen_offset(on_line, lateral, post_offset);
    local_point_t head = gen_offset(on_line, lateral, post_offset - side * 0.9);
    float y0 = o->kerb_height;
    /* Each lamp is a pole of its own, so the car can knock it down. */
    city_chunk_t* chunk = gen_chunk_at(layout, base);
    uint64_t source = gen_pole_source(layout, (uint64_t)road->source_id);
    city_part_t* post =
        city_chunk_add_part(chunk, "lamp", PART_COLLIDER_MESH,
                            COLLISION_LAYER_PROP, 0.5f, source);
    if (!post ||
        !city_chunk_add_part(chunk, "lamp_light", PART_COLLIDER_NONE, 0, 0,
                             source))
      return false;
    post = &chunk->parts[chunk->part_count - 2];
    city_part_t* lights = &chunk->parts[chunk->part_count - 1];
    if (!mesh_data_add_box(
            &post->mesh, vec3((float)base.x, y0 + height * 0.5f, (float)base.z),
            vec3(0.07f, height * 0.5f, 0.07f)))
      return false;
    vec3_t arm_centre = vec3((float)((base.x + head.x) * 0.5), y0 + height,
                             (float)((base.z + head.z) * 0.5));
    vec3_t arm_half = vec3((float)fabs(head.x - base.x) * 0.5f + 0.06f, 0.06f,
                           (float)fabs(head.z - base.z) * 0.5f + 0.06f);
    if (!mesh_data_add_box(&post->mesh, arm_centre, arm_half)) return false;
    if (!mesh_data_add_box(
            &lights->mesh,
            vec3((float)head.x, y0 + height - 0.12f, (float)head.z),
            vec3(0.22f, 0.1f, 0.22f)))
      return false;
    gen_finish_part(chunk, post);
    gen_finish_part(chunk, lights);
    stats->lamps++;
  }
  return true;
}

/* Points every `step` metres along straight stretches, so the width fit
 * sees what stands beside the middle of a long segment, not just its
 * ends. */
static size_t densify(const local_point_t* in, size_t count, double step,
                      local_point_t* out) {
  size_t n = 0;
  for (size_t i = 0; i < count; i++) {
    if (i > 0) {
      local_point_t a = in[i - 1], b = in[i];
      int pieces = (int)(hypot(b.x - a.x, b.z - a.z) / step);
      for (int k = 1; k <= pieces; k++) {
        double t = (double)k / (double)(pieces + 1);
        local_point_t q = {a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t};
        out[n++] = q;
      }
    }
    out[n++] = in[i];
  }
  return n;
}

/* True where a point lies on the carriageway or pavement of a road that
 * cars drive on, other than `self`. Footpaths are not drawn there. */
static bool on_a_street(const city_model_t* model, const city_road_t* self,
                        local_point_t p, double pavement_width) {
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* other = &model->roads[r];
    if (other == self || other->class == ROAD_CLASS_PATH ||
        other->class == ROAD_CLASS_WATERWAY ||
        other->class == ROAD_CLASS_PEDESTRIAN)
      continue;
    double reach = other->width * 0.5 + pavement_width;
    for (size_t k = 1; k < other->point_count; k++)
      if (gen_point_segment_distance(p, other->points[k - 1],
                                     other->points[k]) < reach)
        return true;
  }
  return false;
}

/* Round iron covers in the carriageway every forty metres or so, a little
 * off the middle of a lane. */
static bool generate_manholes(layout_t* layout, const city_road_t* road,
                              const local_point_t* points,
                              const vec2_t* laterals, size_t count,
                              const double* halves) {
  const city_generator_options_t* o = layout->options;
  double length = polyline_length(points, count);
  if (length < 20.0 || road->class == ROAD_CLASS_MOTORWAY) return true;
  city_chunk_t* chunk = gen_chunk_at(layout, gen_centroid(points, count));
  city_part_t* part = city_chunk_add_part(chunk, "manhole", PART_COLLIDER_NONE,
                                          0, 0, (uint64_t)road->source_id);
  if (!part) return false;
  uint64_t seed = (uint64_t)road->source_id * 0x2545f4914f6cdd1dull + 5;
  float y = o->road_height + 0.008f;
  for (double along = 9.0 + (double)(seed % 17); along < length - 6.0;
       along += 36.0 + (double)((seed >> 12) % 14)) {
    seed ^= seed >> 12, seed ^= seed << 25, seed ^= seed >> 27;
    size_t segment;
    local_point_t on_line = gen_along_polyline(points, count, along, &segment);
    vec2_t lateral = gen_v2_normalize(laterals[segment > 0 ? segment - 1 : 0]);
    size_t at_point = segment < count ? segment : count - 1;
    double side = ((seed >> 20) & 1) ? 0.5 : -0.5;
    local_point_t centre =
        gen_offset(on_line, lateral,
                   side * halves[at_point] + ((seed >> 30) % 5) * 0.1 - 0.2);
    uint32_t base = (uint32_t)part->mesh.vertex_count;
    vec3_t up = vec3(0, 1, 0);
    if (!gen_add_vertex(&part->mesh, gen_at(centre, y), up, 0.5f, 0.5f))
      return false;
    for (int k = 0; k < 12; k++) {
      double angle = k * M_PI / 6.0;
      local_point_t rim = {centre.x + cos(angle) * 0.34,
                           centre.z + sin(angle) * 0.34};
      if (!gen_add_vertex(&part->mesh, gen_at(rim, y), up, 0, 0)) return false;
    }
    for (int k = 0; k < 12; k++)
      if (!gen_add_face(&part->mesh, base, base + 1 + (uint32_t)k,
                        base + 1 + (uint32_t)((k + 1) % 12), up))
        return false;
  }
  if (part->mesh.vertex_count == 0) {
    mesh_data_free(&part->mesh);
    chunk->part_count--;
    return true;
  }
  gen_finish_part(chunk, part);
  return true;
}

/* How far along a road's centre line the point nearest to p lies, and how
 * long the road is. */
static double arc_position(const city_road_t* road, local_point_t p,
                           double* total) {
  double best = 1e30, at = 0, run = 0;
  for (size_t k = 1; k < road->point_count; k++) {
    local_point_t a = road->points[k - 1], b = road->points[k];
    double length = hypot(b.x - a.x, b.z - a.z);
    double abx = b.x - a.x, abz = b.z - a.z;
    double u = length > 1e-9
                   ? ((p.x - a.x) * abx + (p.z - a.z) * abz) / (length * length)
                   : 0;
    u = u < 0 ? 0 : (u > 1 ? 1 : u);
    double d = hypot(p.x - (a.x + abx * u), p.z - (a.z + abz * u));
    if (d < best) best = d, at = run + u * length;
    run += length;
  }
  *total = run;
  return at;
}

/* A footbridge: the deck climbs from the ground along a ramp at each end to
 * 0.25 m plus 4 cm per metre of its length, at most 4.5 m, with a parapet along
 * both edges, and a slab below it. The deck is raised in place (it is a
 * collider like any paving), the parapet and underside go into a part of their
 * own. */
static bool raise_footbridge(city_chunk_t* chunk, const city_road_t* road,
                             city_part_t* deck,
                             const city_generator_options_t* o) {
  (void)o;
  double total;
  arc_position(road, road->points[0], &total);
  if (total < 6.0) {
    gen_finish_part(chunk, deck);
    return true;
  }
  /* Long bridges clear a lorry; short ones over a stream only a metre. */
  const double height = fmin(4.5, 0.25 + 0.04 * total);
  double ramp = fmin(16.0, total * 0.4);
  for (size_t v = 0; v < deck->mesh.vertex_count; v++) {
    vec3_t* p = &deck->mesh.vertices[v].position;
    local_point_t q = {p->x, p->z};
    double along = arc_position(road, q, &total);
    double rise = fmin(along / ramp, (total - along) / ramp);
    rise = rise < 0 ? 0 : (rise > 1 ? 1 : rise);
    p->y += (float)(height * rise * rise * (3.0 - 2.0 * rise));
  }
  /* Parapet and underside from the deck's own edge vertices: the strip's
   * vertices come in left/right pairs per point along the run. */
  city_part_t* rail = city_chunk_add_part(chunk, "rail", PART_COLLIDER_NONE, 0,
                                          0, (uint64_t)road->source_id);
  if (!rail) return false;
  deck = &chunk->parts[chunk->part_count - 2];
  mesh_data_t* d = &deck->mesh;
  for (size_t v = 0; v + 3 < d->vertex_count; v += 2) {
    /* Pairs (v, v+1) and (v+2, v+3) are consecutive cross-sections of one
     * run; a gap between runs shows as a long jump, which we skip. */
    vec3_t l0 = d->vertices[v].position, r0 = d->vertices[v + 1].position;
    vec3_t l1 = d->vertices[v + 2].position, r1 = d->vertices[v + 3].position;
    if (vec3_distance(l0, l1) > 6.0f) continue;
    vec3_t up = vec3(0, 1, 0), down = vec3(0, -1, 0);
    float parapet = 1.05f, slab = 0.35f;
    /* Left and right parapet panels, two-sided. */
    vec3_t edge[2][2] = {{l0, l1}, {r0, r1}};
    for (int side = 0; side < 2; side++) {
      vec3_t a = edge[side][0], b = edge[side][1];
      vec3_t ta = vec3_add(a, vec3_scale(up, parapet)),
             tb = vec3_add(b, vec3_scale(up, parapet));
      if (!mesh_data_add_quad(&rail->mesh, a, b, tb, ta, 1) ||
          !mesh_data_add_quad(&rail->mesh, a, ta, tb, b, 1))
        return false;
    }
    /* The underside, facing down. */
    vec3_t la = vec3_add(l0, vec3_scale(up, -slab)),
           lb = vec3_add(l1, vec3_scale(up, -slab));
    vec3_t ra = vec3_add(r0, vec3_scale(up, -slab)),
           rb = vec3_add(r1, vec3_scale(up, -slab));
    (void)down;
    if (!mesh_data_add_quad(&rail->mesh, la, ra, rb, lb, 1)) return false;
    /* The deck's edge face, left and right. */
    if (!mesh_data_add_quad(&rail->mesh, l0, la, lb, l1, 1) ||
        !mesh_data_add_quad(&rail->mesh, r1, rb, ra, r0, 1))
      return false;
  }
  rail = &chunk->parts[chunk->part_count - 1];
  gen_finish_part(chunk, deck);
  gen_finish_part(chunk, rail);
  return true;
}

bool generate_road(layout_t* layout, const city_model_t* model,
                   const city_road_t* road, road_fit_t fit, road_cuts_t cuts,
                   city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  local_point_t* cut = malloc(road->point_count * sizeof(local_point_t));
  size_t cut_count = cut ? polyline_cut(road->points, road->point_count,
                                        cuts.start, cuts.end, cut)
                         : 0;
  const double step = road->class == ROAD_CLASS_PATH ? 2.0 : 8.0;
  size_t capacity = road->point_count + 2 +
                    (size_t)(cut ? polyline_length(cut, cut_count) / step : 0);
  local_point_t* points = malloc(capacity * sizeof(local_point_t));
  vec2_t* laterals = malloc(capacity * sizeof(vec2_t));
  double* halves = malloc(capacity * sizeof(double));
  double* pavements = malloc(capacity * sizeof(double));
  double* inner = malloc(capacity * sizeof(double));
  double* outer = malloc(capacity * sizeof(double));
  bool ok = cut && points && laterals && halves && pavements && inner && outer;
  size_t count = ok ? densify(cut, cut_count, step, points) : 0;
  free(cut);
  if (!ok || polyline_length(points, count) < 0.05) { /* nothing to draw */
    free(points);
    free(laterals);
    free(halves);
    free(pavements);
    free(inner);
    free(outer);
    return ok;
  }
  gen_lateral_offsets(points, count, laterals);
  fit_along(model, road, o, fit, points, count, halves, pavements);
  city_chunk_t* chunk = gen_chunk_at(layout, gen_centroid(points, count));
  bool for_people =
      road->class == ROAD_CLASS_PEDESTRIAN || road->class == ROAD_CLASS_PATH;
  bool water = road->class == ROAD_CLASS_WATERWAY;
  /* Streets for people and paths sit above the roads and the squares
   * they cross; streams just above the grass and below everything else,
   * with nothing to collide with. */
  float height = for_people ? o->road_height + 0.02f
                 : water    ? 0.02f
                            : o->road_height;
  city_part_t* part = city_chunk_add_part(
      chunk,
      water        ? "water"
      : for_people ? "paving"
                   : "road",
      water ? PART_COLLIDER_NONE : PART_COLLIDER_MESH, COLLISION_LAYER_GROUND,
      o->road_friction, (uint64_t)road->source_id);
  for (size_t i = 0; i < count; i++)
    inner[i] = -halves[i], outer[i] = halves[i];
  if (road->class == ROAD_CLASS_PATH) {
    /* Only the stretches that are not on a street: runs of consecutive
     * points off the asphalt and pavements, each its own strip. */
    size_t run_start = 0;
    bool in_run = false;
    ok = part != NULL;
    for (size_t i = 0; ok && i <= count; i++) {
      bool outside = i < count &&
                     (road->bridge ||
                      !on_a_street(model, road, points[i], o->pavement_width));
      if (outside && !in_run) run_start = i, in_run = true;
      if (!outside && in_run) {
        in_run = false;
        if (i - run_start >= 2)
          ok =
              gen_add_strip(&part->mesh, points + run_start,
                            laterals + run_start, i - run_start,
                            inner + run_start, outer + run_start, height, 3.0f);
      }
    }
    if (ok && part->mesh.vertex_count == 0) {
      mesh_data_free(&part->mesh); /* a path entirely on streets */
      chunk->part_count--;
      part = NULL;
    }
  } else {
    ok = part &&
         gen_add_strip(&part->mesh, points, laterals, count, inner, outer,
                       height, for_people ? 2.0f : o->road_texture_length);
  }
  if (ok && part && road->bridge && road->class == ROAD_CLASS_PATH &&
      road->layer >= 1) {
    /* The footbridge finishes its own parts: adding the parapet may move
     * the deck, so no pointer to it is kept. */
    ok = raise_footbridge(chunk, road, part, o);
    part = NULL;
  }
  if (ok && part) gen_finish_part(chunk, part);
  stats->roads++;

  if (ok && fit.pavement > 0) {
    city_part_t* pavement = city_chunk_add_part(
        chunk, "pavement", PART_COLLIDER_MESH, COLLISION_LAYER_GROUND,
        o->pavement_friction, (uint64_t)road->source_id);
    city_part_t* kerb =
        pavement ? city_chunk_add_part(chunk, "kerb", PART_COLLIDER_NONE, 0, 0,
                                       (uint64_t)road->source_id)
                 : NULL;
    ok = pavement && kerb;
    if (ok) pavement = &chunk->parts[chunk->part_count - 2];
    for (int side = -1; side <= 1 && ok; side += 2) {
      for (size_t i = 0; i < count; i++) {
        double edge = side * halves[i], far = side * (halves[i] + pavements[i]);
        inner[i] = fmin(edge, far), outer[i] = fmax(edge, far);
      }
      ok = gen_add_strip(&pavement->mesh, points, laterals, count, inner, outer,
                         o->kerb_height, o->pavement_width);
      for (size_t i = 0; i < count && ok; i++) inner[i] = side * halves[i];
      ok = ok && gen_add_kerb(&kerb->mesh, points, laterals, count, inner,
                              o->road_height, o->kerb_height, side < 0);
    }
    if (ok) {
      gen_finish_part(chunk, pavement);
      gen_finish_part(chunk, kerb);
      stats->pavements++;
    }
  }
  if (ok && o->lamp_spacing > 0 && road->class != ROAD_CLASS_PATH &&
      road->class != ROAD_CLASS_SERVICE && road->class != ROAD_CLASS_WATERWAY)
    ok = generate_lamps(layout, road, points, laterals, count, halves,
                        pavements, stats);
  if (ok && !for_people && !water)
    ok = generate_manholes(layout, road, points, laterals, count, halves);
  free(points);
  free(laterals);
  free(halves);
  free(pavements);
  free(inner);
  free(outer);
  return ok;
}
