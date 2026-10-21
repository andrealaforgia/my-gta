/* The generator's settings, spawn, map labels, routes and its entry point. */
#include "city_generator.h"

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

city_generator_options_t city_generator_default_options(void) {
  city_generator_options_t o = {150.0f, 0.06f, true,  2.0f,  0.18f, 8.0f,
                                3.0f,   3.2f,  12,    0.25f, 1.0f,  0.9f,
                                0.7f,   0.6f,  28.0f, 14.0f, {0}};
  return o;
}

bool city_generator_options_load(city_generator_options_t* o,
                                 const char* path) {
  *o = city_generator_default_options();
  config_t config;
  if (!config_load(&config, path)) {
    LOG_ERROR("generate", "cannot read %s", path);
    config_free(&config);
    return false;
  }
  const char* arcades = config_get_string(&config, "arcade_streets", "");
  snprintf(o->arcade_streets, sizeof(o->arcade_streets), "%s", arcades);
  o->chunk_size = config_get_float(&config, "chunk_size", o->chunk_size);
  o->road_height = config_get_float(&config, "road_height", o->road_height);
  o->pavements = config_get_bool(&config, "pavements", o->pavements);
  o->pavement_width =
      config_get_float(&config, "pavement_width", o->pavement_width);
  o->kerb_height = config_get_float(&config, "kerb_height", o->kerb_height);
  o->road_texture_length =
      config_get_float(&config, "road_texture_length", o->road_texture_length);
  o->window_width = config_get_float(&config, "window_width", o->window_width);
  o->storey_height =
      config_get_float(&config, "storey_height", o->storey_height);
  o->facade_variants =
      config_get_int(&config, "facade_variants", o->facade_variants);
  o->ground_uv_scale =
      config_get_float(&config, "ground_uv_scale", o->ground_uv_scale);
  o->road_friction =
      config_get_float(&config, "road_friction", o->road_friction);
  o->pavement_friction =
      config_get_float(&config, "pavement_friction", o->pavement_friction);
  o->ground_friction =
      config_get_float(&config, "ground_friction", o->ground_friction);
  o->building_friction =
      config_get_float(&config, "building_friction", o->building_friction);
  o->lamp_spacing = config_get_float(&config, "lamp_spacing", o->lamp_spacing);
  o->tree_spacing = config_get_float(&config, "tree_spacing", o->tree_spacing);
  if (config_defaulted_count(&config) > 0)
    LOG_WARN("generate", "%s: %zu settings missing, defaults used", path,
             config_defaulted_count(&config));
  config_free(&config);
  bool valid = o->chunk_size >= 20 && o->facade_variants >= 1;
  if (!valid)
    LOG_ERROR("generate",
              "%s: chunk_size must be >= 20 and facade_variants >= 1", path);
  return valid;
}

uint64_t city_generator_options_hash(const city_generator_options_t* o) {
  /* FNV-1a over a textual rendering so padding bytes do not matter. */
  char text[512];
  snprintf(text, sizeof(text),
           "%g %g %d %g %g %g %g %g %d %g %g %g %g %g %g %g %s", o->chunk_size,
           o->road_height, o->pavements, o->pavement_width, o->kerb_height,
           o->road_texture_length, o->window_width, o->storey_height,
           o->facade_variants, o->ground_uv_scale, o->road_friction,
           o->pavement_friction, o->ground_friction, o->building_friction,
           o->lamp_spacing, o->tree_spacing, o->arcade_streets);
  uint64_t hash = 1469598103934665603ull;
  for (const char* c = text; *c; c++)
    hash = (hash ^ (uint8_t)*c) * 1099511628211ull;
  return hash;
}

/* The spawn: the longest ordinary road whose midpoint lies within the
 * central part of the extract, so a drive starts in town and the first
 * junction is close. */
static void choose_spawn(city_package_t* package, const city_model_t* model,
                         const city_generator_options_t* o) {
  const city_road_t* best = NULL;
  double best_length = 0;
  double extent = 0;
  for (size_t r = 0; r < model->road_count; r++)
    for (size_t i = 0; i < model->roads[r].point_count; i++) {
      local_point_t p = model->roads[r].points[i];
      extent = fmax(extent, fmax(fabs(p.x), fabs(p.z)));
    }
  for (int pass = 0; pass < 2 && !best; pass++) {
    double limit = pass == 0 ? extent * 0.4 : 1e30;
    for (size_t r = 0; r < model->road_count; r++) {
      const city_road_t* road = &model->roads[r];
      if (road->class == ROAD_CLASS_SERVICE ||
          road->class == ROAD_CLASS_MOTORWAY ||
          road->class == ROAD_CLASS_PEDESTRIAN ||
          road->class == ROAD_CLASS_PATH || road->class == ROAD_CLASS_WATERWAY)
        continue;
      local_point_t middle = gen_centroid(road->points, road->point_count);
      if (fabs(middle.x) > limit || fabs(middle.z) > limit) continue;
      double length = 0;
      for (size_t i = 1; i < road->point_count; i++)
        length +=
            gen_v2_length(gen_v2_sub(road->points[i], road->points[i - 1]));
      if (length > best_length) {
        best_length = length;
        best = road;
      }
    }
  }
  if (!best) best = &model->roads[0];
  /* Midpoint of the longest segment of the chosen road, facing the end of
   * the road nearer the centre of town so the drive heads into it. */
  size_t longest = 0;
  double longest_length = -1;
  for (size_t i = 1; i < best->point_count; i++) {
    double length =
        gen_v2_length(gen_v2_sub(best->points[i], best->points[i - 1]));
    if (length > longest_length) {
      longest_length = length;
      longest = i;
    }
  }
  local_point_t a = best->points[longest - 1], b = best->points[longest];
  local_point_t first = best->points[0],
                last = best->points[best->point_count - 1];
  bool towards_last = hypot(last.x, last.z) <= hypot(first.x, first.z);
  vec2_t direction = towards_last ? gen_v2_normalize(gen_v2_sub(b, a))
                                  : gen_v2_normalize(gen_v2_sub(a, b));
  package->spawn_position =
      vec3((float)((a.x + b.x) * 0.5), o->road_height + 0.9f,
           (float)((a.z + b.z) * 0.5));
  package->spawn_yaw = (float)atan2(-direction.x, -direction.z);
}

/* The map's street names: each distinct name once, at the middle of its
 * longest piece, longest streets first. Capitals and plain ASCII only,
 * as the built-in font has nothing else. */
static void choose_labels(city_package_t* package, const city_model_t* model) {
  typedef struct {
    char text[32];
    double total, best;
    local_point_t at;
  } entry_t;
  entry_t* entries = calloc(model->road_count + 1, sizeof(entry_t));
  if (!entries) return;
  size_t count = 0;
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* road = &model->roads[r];
    if (!road->name[0] || road->class == ROAD_CLASS_WATERWAY ||
        road->class == ROAD_CLASS_PATH)
      continue;
    char text[32];
    size_t n = 0;
    for (const char* c = road->name; *c && n + 1 < sizeof(text); c++) {
      unsigned char u = (unsigned char)*c;
      if (u >= 0x80) continue;
      if (*c == '(') break; /* "(già Riviera...)" asides do not fit */
      text[n++] = *c == '_' ? ' ' : (char)toupper(u);
    }
    while (n > 0 && text[n - 1] == ' ') n--;
    text[n] = 0;
    if (n == 0) continue;
    double length = polyline_length(road->points, road->point_count);
    size_t e = 0;
    while (e < count && strcmp(entries[e].text, text) != 0) e++;
    if (e == count) {
      memcpy(entries[e].text, text, sizeof(text));
      count++;
    }
    entries[e].total += length;
    if (length > entries[e].best) {
      size_t segment;
      entries[e].best = length;
      entries[e].at = gen_along_polyline(road->points, road->point_count,
                                         length * 0.5, &segment);
    }
  }
  for (size_t l = 0; l < CITY_PACKAGE_MAX_LABELS && l < count; l++) {
    size_t top = l;
    for (size_t e = l + 1; e < count; e++)
      if (entries[e].total > entries[top].total) top = e;
    entry_t chosen = entries[top];
    entries[top] = entries[l];
    entries[l] = chosen;
    city_label_t* label = &package->labels[package->label_count++];
    label->x = (float)chosen.at.x;
    label->z = (float)chosen.at.z;
    memcpy(label->text, chosen.text, sizeof(label->text));
  }
  free(entries);
}

/* The road as the runtime sees it: its centre line, and the places along
 * it where cars stand at the kerb. */
static bool add_route_and_parking(city_package_t* package,
                                  const city_model_t* model,
                                  const city_road_t* road, road_fit_t fit,
                                  road_cuts_t cuts) {
  if (road->class == ROAD_CLASS_WATERWAY) return true;
  bool drivable =
      road->class != ROAD_CLASS_PEDESTRIAN && road->class != ROAD_CLASS_PATH;
  city_route_t route;
  memset(&route, 0, sizeof(route));
  route.drivable = drivable;
  route.oneway = road->oneway;
  route.width = (float)(fit.half_width * 2);
  route.pavement = (float)fit.pavement;
  route.start_node = road->node_ids[0];
  route.end_node = road->node_ids[road->point_count - 1];
  /* Junction ids of zero come from rounding arcs; ends always carry real ones,
   * but keep the field non-zero for readers. */
  if (route.start_node == 0) route.start_node = 1;
  if (route.end_node == 0) route.end_node = 1;
  route.point_count = road->point_count;
  route.points = malloc(road->point_count * 2 * sizeof(float));
  if (!route.points) return false;
  for (size_t i = 0; i < road->point_count; i++) {
    route.points[2 * i] = (float)road->points[i].x;
    route.points[2 * i + 1] = (float)road->points[i].z;
  }
  if (!city_package_add_route(package, &route)) {
    free(route.points);
    return false;
  }
  (void)model;
  if (!drivable || road->class == ROAD_CLASS_MOTORWAY ||
      road->class == ROAD_CLASS_SERVICE || fit.half_width < 2.6 ||
      fit.pavement <= 0)
    return true;
  local_point_t* points = malloc(road->point_count * sizeof(local_point_t));
  vec2_t* laterals = malloc(road->point_count * sizeof(vec2_t));
  if (!points || !laterals) {
    free(points);
    free(laterals);
    return false;
  }
  size_t count = polyline_cut(road->points, road->point_count, cuts.start,
                              cuts.end, points);
  gen_lateral_offsets(points, count, laterals);
  double length = polyline_length(points, count);
  uint64_t seed = (uint64_t)road->source_id * 0x9e3779b97f4a7c15ull + 7;
  bool ok = true;
  for (double along = 8.0; ok && along < length - 8.0; along += 6.2) {
    seed ^= seed >> 12, seed ^= seed << 25, seed ^= seed >> 27;
    if ((seed >> 20) % 100 >= 55) continue;
    size_t segment;
    local_point_t on_line = gen_along_polyline(points, count, along, &segment);
    vec2_t lateral = gen_v2_normalize(laterals[segment > 0 ? segment - 1 : 0]);
    int side = ((seed >> 33) & 1) ? 1 : -1;
    local_point_t spot =
        gen_offset(on_line, lateral, side * (fit.half_width - 1.0));
    vec2_t direction =
        gen_v2(-lateral.z, lateral.x); /* the way the road runs */
    direction = gen_v2_normalize(direction);
    if (side < 0) direction = gen_v2(-direction.x, -direction.z);
    double jitter = (double)((seed >> 41) % 100) / 100.0 * 0.06 - 0.03;
    city_parking_t place = {(float)spot.x, (float)spot.z,
                            (float)(atan2(-direction.x, -direction.z) + jitter),
                            (int)((seed >> 50) % 64)};
    ok = city_package_add_parking(package, place);
  }
  free(points);
  free(laterals);
  return ok;
}

bool city_generate(city_package_t* package, const city_model_t* model,
                   const city_generator_options_t* options, const char* name,
                   city_generator_stats_t* stats) {
  memset(stats, 0, sizeof(*stats));
  *package = city_package_create();
  if (model->road_count == 0) {
    LOG_ERROR("generate", "the model has no roads");
    return false;
  }
  snprintf(package->name, sizeof(package->name), "%s", name);
  snprintf(package->source, sizeof(package->source), "%s", model->source);
  snprintf(package->source_date, sizeof(package->source_date), "%s",
           model->import_date);
  time_t now = time(NULL);
  strftime(package->generated, sizeof(package->generated), "%Y-%m-%d",
           gmtime(&now));
  package->bounds_lat_lon[0] = model->bounds.min_lat;
  package->bounds_lat_lon[1] = model->bounds.min_lon;
  package->bounds_lat_lon[2] = model->bounds.max_lat;
  package->bounds_lat_lon[3] = model->bounds.max_lon;
  package->origin_lat = model->projection.origin_lat;
  package->origin_lon = model->projection.origin_lon;
  package->chunk_size = options->chunk_size;
  package->config_hash = city_generator_options_hash(options);

  /* Chunk grid over everything, with a margin so the ground extends past
   * the last road. */
  double min_x = 1e30, min_z = 1e30, max_x = -1e30, max_z = -1e30;
  for (size_t r = 0; r < model->road_count; r++)
    for (size_t i = 0; i < model->roads[r].point_count; i++) {
      local_point_t p = model->roads[r].points[i];
      min_x = fmin(min_x, p.x), max_x = fmax(max_x, p.x),
      min_z = fmin(min_z, p.z), max_z = fmax(max_z, p.z);
    }
  for (size_t b = 0; b < model->building_count; b++)
    for (size_t i = 0; i < model->buildings[b].point_count; i++) {
      local_point_t p = model->buildings[b].points[i];
      min_x = fmin(min_x, p.x), max_x = fmax(max_x, p.x),
      min_z = fmin(min_z, p.z), max_z = fmax(max_z, p.z);
    }
  /* Parks and water reach beyond the extract's edge: the ground must go
   * as far as they do, or they hang over nothing. */
  for (size_t a = 0; a < model->area_count; a++)
    for (size_t i = 0; i < model->areas[a].point_count; i++) {
      local_point_t p = model->areas[a].points[i];
      min_x = fmin(min_x, p.x), max_x = fmax(max_x, p.x),
      min_z = fmin(min_z, p.z), max_z = fmax(max_z, p.z);
    }
  const double margin = 40.0;
  layout_t layout;
  layout.package = package;
  layout.options = options;
  layout.terrain = &model->terrain;
  layout.model = model;
  layout.arcade_edges = NULL;
  layout.arcade_edge_count = 0;
  layout.min_x =
      floor((min_x - margin) / options->chunk_size) * options->chunk_size;
  layout.min_z =
      floor((min_z - margin) / options->chunk_size) * options->chunk_size;
  layout.columns =
      (int)ceil((max_x + margin - layout.min_x) / options->chunk_size);
  layout.rows =
      (int)ceil((max_z + margin - layout.min_z) / options->chunk_size);
  if (layout.columns < 1) layout.columns = 1;
  if (layout.rows < 1) layout.rows = 1;

  road_fit_t* fits = calloc(model->road_count, sizeof(road_fit_t));
  road_cuts_t* cuts = calloc(model->road_count, sizeof(road_cuts_t));
  if (!fits || !cuts) {
    free(fits);
    free(cuts);
    return false;
  }
  for (size_t r = 0; r < model->road_count; r++)
    fits[r] = gen_fit_road(model, &model->roads[r], options);
  bool ok = generate_ground(&layout, stats);
  for (size_t j = 0; ok && j < model->junction_count; j++)
    ok = generate_junction(&layout, model, &model->junctions[j], fits, cuts,
                           stats);
  for (size_t r = 0; ok && r < model->road_count; r++)
    ok = generate_road(&layout, model, &model->roads[r], fits[r], cuts[r],
                       stats);
  for (size_t r = 0; ok && r < model->road_count; r++)
    ok = add_route_and_parking(package, model, &model->roads[r], fits[r],
                               cuts[r]);
  for (size_t a = 0; ok && a < model->area_count; a++)
    ok = generate_area(&layout, &model->areas[a], stats);
  /* Edges that face an arcaded street, so neighbouring porticoes join. */
  if (options->arcade_streets[0]) {
    layout.arcade_edges =
        calloc(model->building_count * 8 + 8, sizeof(arcade_edge_t));
    size_t capacity = model->building_count * 8 + 8;
    for (size_t b = 0; ok && b < model->building_count; b++) {
      const city_building_t* building = &model->buildings[b];
      double depth = gen_building_depth(building);
      for (size_t i = 0; i < building->point_count; i++)
        if (gen_edge_faces_arcade(model, options, building, i, depth) &&
            layout.arcade_edge_count < capacity) {
          arcade_edge_t* edge =
              &layout.arcade_edges[layout.arcade_edge_count++];
          edge->a = building->points[i];
          edge->b = building->points[(i + 1) % building->point_count];
        }
    }
  }
  for (size_t b = 0; ok && b < model->building_count; b++)
    ok = generate_building(&layout, &model->buildings[b], stats);
  if (ok) ok = generate_street_trees(&layout, model, stats);
  if (ok) ok = generate_furniture(&layout, model, stats);
  if (ok) ok = generate_crossings(&layout, model, fits);
  free(layout.arcade_edges);
  free(fits);
  free(cuts);
  ok = ok && gen_apply_terrain(&layout);
  if (!ok) {
    LOG_ERROR("generate", "out of memory while generating");
    return false;
  }
  choose_spawn(package, model, options);
  choose_labels(package, model);
  for (size_t f = 0; f < model->furniture_count; f++)
    if (model->furniture[f].kind == OSM_FURNITURE_SIGNALS)
      city_package_add_signal(package, (float)model->furniture[f].x,
                              (float)model->furniture[f].z);
  /* The car starts on an empty road. */
  size_t kept = 0;
  for (size_t i = 0; i < package->parked_count; i++) {
    float dx = package->parked[i].x - package->spawn_position.x;
    float dz = package->parked[i].z - package->spawn_position.z;
    if (dx * dx + dz * dz > 100.0f)
      package->parked[kept++] = package->parked[i];
  }
  package->parked_count = kept;
  package->spawn_position.y += (float)gen_terrain_at(
      &layout, package->spawn_position.x, package->spawn_position.z);
  LOG_INFO("generate",
           "%d x %d chunks, %zu roads, %zu junction patches, %zu pavements, "
           "%zu lamps, %zu areas, %zu trees, %zu buildings (%zu approximated), "
           "%zu triangles",
           layout.columns, layout.rows, stats->roads, stats->junction_patches,
           stats->pavements, stats->lamps, stats->areas, stats->trees,
           stats->buildings, stats->buildings_unclean,
           city_package_triangle_count(package));
  return true;
}
