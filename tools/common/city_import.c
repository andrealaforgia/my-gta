#include "city_import.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "log.h"

city_import_options_t city_import_default_options(void) {
  city_import_options_t o;
  o.default_width[ROAD_CLASS_MOTORWAY] = 14.0f;
  o.default_width[ROAD_CLASS_PRIMARY] = 10.0f;
  o.default_width[ROAD_CLASS_SECONDARY] = 8.0f;
  o.default_width[ROAD_CLASS_TERTIARY] = 7.0f;
  o.default_width[ROAD_CLASS_RESIDENTIAL] = 6.0f;
  o.default_width[ROAD_CLASS_SERVICE] = 4.0f;
  o.default_width[ROAD_CLASS_UNCLASSIFIED] = 6.0f;
  o.storey_height = 3.2f;
  o.default_height_min = 6.0f;
  o.default_height_max = 15.0f;
  o.min_road_length = 0.5f;
  o.include_service_roads = true;
  return o;
}

/* ---- classification ---- */

static bool classify_highway(const char* highway, bool include_service,
                             road_class_t* class) {
  static const struct {
    const char* tag;
    road_class_t class;
  } table[] = {{"motorway", ROAD_CLASS_MOTORWAY},
               {"motorway_link", ROAD_CLASS_MOTORWAY},
               {"trunk", ROAD_CLASS_PRIMARY},
               {"trunk_link", ROAD_CLASS_PRIMARY},
               {"primary", ROAD_CLASS_PRIMARY},
               {"primary_link", ROAD_CLASS_PRIMARY},
               {"secondary", ROAD_CLASS_SECONDARY},
               {"secondary_link", ROAD_CLASS_SECONDARY},
               {"tertiary", ROAD_CLASS_TERTIARY},
               {"tertiary_link", ROAD_CLASS_TERTIARY},
               {"residential", ROAD_CLASS_RESIDENTIAL},
               {"living_street", ROAD_CLASS_RESIDENTIAL},
               {"unclassified", ROAD_CLASS_UNCLASSIFIED},
               {"road", ROAD_CLASS_UNCLASSIFIED},
               {"service", ROAD_CLASS_SERVICE}};
  for (size_t i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
    if (strcmp(highway, table[i].tag) == 0) {
      if (table[i].class == ROAD_CLASS_SERVICE && !include_service)
        return false;
      *class = table[i].class;
      return true;
    }
  }
  return false;
}

static float parse_metres(const char* text, bool* ok) {
  char* end;
  float value = strtof(text, &end);
  *ok = end != text && value > 0 && value < 200;
  return value;
}

static bool tag_is_yes(const char* value) {
  return value && (strcmp(value, "yes") == 0 || strcmp(value, "true") == 0 ||
                   strcmp(value, "1") == 0 || strcmp(value, "-1") == 0);
}

/* ---- roads ---- */

typedef struct {
  int64_t node_id;
  int uses; /* how many accepted ways pass through */
  int ends; /* how many accepted ways start or end here */
} node_use_t;

static int compare_node_use(const void* a, const void* b) {
  int64_t x = ((const node_use_t*)a)->node_id,
          y = ((const node_use_t*)b)->node_id;
  return x < y ? -1 : (x > y ? 1 : 0);
}

static node_use_t* find_use(node_use_t* uses, size_t count, int64_t id) {
  node_use_t key = {id, 0, 0};
  return bsearch(&key, uses, count, sizeof(node_use_t), compare_node_use);
}

typedef struct {
  const osm_way_t* way;
  road_class_t class;
  local_point_t* points;
  int64_t* node_ids;
  size_t point_count;
} cleaned_way_t;

/* Project and drop repeated consecutive nodes or coincident points. */
static bool clean_way(const osm_data_t* osm, const projection_t* projection,
                      const osm_way_t* way, cleaned_way_t* out) {
  out->points = calloc(way->node_count, sizeof(local_point_t));
  out->node_ids = calloc(way->node_count, sizeof(int64_t));
  out->point_count = 0;
  if (!out->points || !out->node_ids) return false;
  for (size_t i = 0; i < way->node_count; i++) {
    const osm_node_t* node = osm_find_node(osm, way->node_refs[i]);
    if (!node) continue;
    local_point_t p = projection_project(projection, node->lat, node->lon);
    if (out->point_count > 0) {
      local_point_t q = out->points[out->point_count - 1];
      bool same_node = out->node_ids[out->point_count - 1] == node->id;
      bool same_place = fabs(p.x - q.x) < 0.01 && fabs(p.z - q.z) < 0.01;
      if (same_node || same_place) continue;
    }
    out->points[out->point_count] = p;
    out->node_ids[out->point_count] = node->id;
    out->point_count++;
  }
  return true;
}

static double polyline_length(const local_point_t* points, size_t count) {
  double length = 0;
  for (size_t i = 1; i < count; i++)
    length +=
        hypot(points[i].x - points[i - 1].x, points[i].z - points[i - 1].z);
  return length;
}

static bool emit_segment(city_model_t* model, const cleaned_way_t* cleaned,
                         size_t from, size_t to,
                         const city_import_options_t* options) {
  const osm_way_t* way = cleaned->way;
  city_road_t* road = city_model_add_road(model);
  if (!road) return false;
  road->source_id = way->id;
  road->class = cleaned->class;
  const char* width = osm_tag(way->tags, way->tag_count, "width");
  bool ok = false;
  if (width) road->width = parse_metres(width, &ok);
  if (!ok) {
    const char* lanes = osm_tag(way->tags, way->tag_count, "lanes");
    int lane_count = lanes ? atoi(lanes) : 0;
    road->width = lane_count > 0 ? (float)lane_count * 3.25f
                                 : options->default_width[cleaned->class];
    road->width_inferred = true;
    model->stats.widths_inferred++;
  }
  road->oneway = tag_is_yes(osm_tag(way->tags, way->tag_count, "oneway"));
  const char* layer = osm_tag(way->tags, way->tag_count, "layer");
  road->layer = layer ? atoi(layer) : 0;
  road->bridge = tag_is_yes(osm_tag(way->tags, way->tag_count, "bridge"));
  road->tunnel = tag_is_yes(osm_tag(way->tags, way->tag_count, "tunnel"));
  const char* surface = osm_tag(way->tags, way->tag_count, "surface");
  const char* name = osm_tag(way->tags, way->tag_count, "name");
  snprintf(road->surface, sizeof(road->surface), "%s", surface ? surface : "");
  snprintf(road->name, sizeof(road->name), "%s", name ? name : "");
  for (char* c = road->name; *c; c++)
    if (*c == ' ') *c = '_';
  size_t count = to - from + 1;
  road->points = malloc(count * sizeof(local_point_t));
  road->node_ids = malloc(count * sizeof(int64_t));
  if (!road->points || !road->node_ids) return false;
  memcpy(road->points, cleaned->points + from, count * sizeof(local_point_t));
  memcpy(road->node_ids, cleaned->node_ids + from, count * sizeof(int64_t));
  road->point_count = count;
  return true;
}

static void import_roads(city_model_t* model, const osm_data_t* osm,
                         const city_import_options_t* options) {
  cleaned_way_t* cleaned = calloc(osm->way_count, sizeof(cleaned_way_t));
  size_t cleaned_count = 0;
  if (!cleaned) return;
  for (size_t i = 0; i < osm->way_count; i++) {
    const osm_way_t* way = &osm->ways[i];
    const char* highway = osm_tag(way->tags, way->tag_count, "highway");
    if (!highway) continue;
    model->stats.ways_total++;
    road_class_t class;
    if (!classify_highway(highway, options->include_service_roads, &class)) {
      model->stats.ways_ignored++;
      continue;
    }
    cleaned_way_t* c = &cleaned[cleaned_count];
    c->way = way;
    c->class = class;
    if (!clean_way(osm, &model->projection, way, c)) continue;
    if (c->point_count < 2 ||
        polyline_length(c->points, c->point_count) < options->min_road_length) {
      LOG_WARN("import", "way %lld is degenerate and was dropped",
               (long long)way->id);
      model->stats.roads_degenerate++;
      free(c->points);
      free(c->node_ids);
      continue;
    }
    cleaned_count++;
  }

  /* Count how each node is used across the accepted ways. */
  size_t use_capacity = 0;
  for (size_t i = 0; i < cleaned_count; i++)
    use_capacity += cleaned[i].point_count;
  node_use_t* uses =
      calloc(use_capacity ? use_capacity : 1, sizeof(node_use_t));
  size_t use_count = 0;
  for (size_t i = 0; i < cleaned_count; i++)
    for (size_t p = 0; p < cleaned[i].point_count; p++) {
      uses[use_count].node_id = cleaned[i].node_ids[p];
      use_count++;
    }
  qsort(uses, use_count, sizeof(node_use_t), compare_node_use);
  size_t unique = 0;
  for (size_t i = 0; i < use_count; i++) {
    if (unique == 0 || uses[unique - 1].node_id != uses[i].node_id)
      uses[unique++] = uses[i];
  }
  use_count = unique;
  for (size_t i = 0; i < cleaned_count; i++) {
    for (size_t p = 0; p < cleaned[i].point_count; p++) {
      node_use_t* use = find_use(uses, use_count, cleaned[i].node_ids[p]);
      use->uses++;
      if (p == 0 || p == cleaned[i].point_count - 1) use->ends++;
    }
  }

  /* Split each way at interior nodes shared with another way. */
  for (size_t i = 0; i < cleaned_count; i++) {
    cleaned_way_t* c = &cleaned[i];
    size_t start = 0;
    for (size_t p = 1; p < c->point_count; p++) {
      bool last = p == c->point_count - 1;
      bool shared = find_use(uses, use_count, c->node_ids[p])->uses > 1;
      if (last || shared) {
        emit_segment(model, c, start, p, options);
        model->stats.roads_accepted++;
        start = p;
      }
    }
    free(c->points);
    free(c->node_ids);
  }
  free(cleaned);

  /* Junctions: every node where a segment ends; degree is the number of
   * segment ends there. */
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* road = &model->roads[r];
    for (int end = 0; end < 2; end++) {
      size_t index = end ? road->point_count - 1 : 0;
      int64_t node = road->node_ids[index];
      city_junction_t* junction = NULL;
      for (size_t j = 0; j < model->junction_count; j++)
        if (model->junctions[j].node_id == node)
          junction = &model->junctions[j];
      if (!junction) {
        junction = city_model_add_junction(model);
        if (!junction) break;
        junction->node_id = node;
        junction->position = road->points[index];
      }
      junction->degree++;
    }
  }
  free(uses);
}

/* ---- buildings ---- */

static unsigned stable_hash(int64_t id) {
  uint64_t h = (uint64_t)id * 0x9e3779b97f4a7c15ull;
  h ^= h >> 29;
  h *= 0xbf58476d1ce4e5b9ull;
  h ^= h >> 32;
  return (unsigned)h;
}

static double signed_area(const local_point_t* points, size_t count) {
  /* In (x, north) with north = -z: positive means anticlockwise from above. */
  double area = 0;
  for (size_t i = 0; i < count; i++) {
    local_point_t p = points[i], q = points[(i + 1) % count];
    area += p.x * -q.z - q.x * -p.z;
  }
  return area * 0.5;
}

static void import_building(city_model_t* model, const osm_data_t* osm,
                            const osm_way_t* way,
                            const city_import_options_t* options) {
  bool closed = way->node_count >= 4 &&
                way->node_refs[0] == way->node_refs[way->node_count - 1];
  if (!closed) {
    LOG_WARN("import", "building %lld is not a closed ring",
             (long long)way->id);
    model->stats.buildings_malformed++;
    return;
  }
  cleaned_way_t cleaned;
  if (!clean_way(osm, &model->projection, way, &cleaned)) return;
  size_t count = cleaned.point_count;
  if (count > 1 && cleaned.node_ids[count - 1] == cleaned.node_ids[0]) count--;
  if (count < 3 || fabs(signed_area(cleaned.points, count)) < 1.0) {
    LOG_WARN("import", "building %lld has no area", (long long)way->id);
    model->stats.buildings_malformed++;
    free(cleaned.points);
    free(cleaned.node_ids);
    return;
  }
  city_building_t* building = city_model_add_building(model);
  if (!building) return;
  building->source_id = way->id;
  const char* category = osm_tag(way->tags, way->tag_count, "building");
  snprintf(building->category, sizeof(building->category), "%s",
           category ? category : "yes");
  const char* height = osm_tag(way->tags, way->tag_count, "height");
  const char* levels = osm_tag(way->tags, way->tag_count, "building:levels");
  bool ok = false;
  if (height) building->height = parse_metres(height, &ok);
  if (ok) {
    building->height_source = HEIGHT_TAGGED;
  } else if (levels && atoi(levels) > 0) {
    building->levels = atoi(levels);
    building->height = (float)building->levels * options->storey_height;
    building->height_source = HEIGHT_FROM_LEVELS;
  } else {
    float t = (float)(stable_hash(way->id) % 1000) / 999.0f;
    building->height =
        options->default_height_min +
        (options->default_height_max - options->default_height_min) * t;
    building->height_source = HEIGHT_DEFAULT;
    model->stats.heights_defaulted++;
  }
  if (building->levels == 0)
    building->levels = (int)(building->height / options->storey_height + 0.5f);
  building->points = malloc(count * sizeof(local_point_t));
  building->point_count = count;
  if (signed_area(cleaned.points, count) > 0) {
    memcpy(building->points, cleaned.points, count * sizeof(local_point_t));
  } else {
    for (size_t i = 0; i < count; i++)
      building->points[i] = cleaned.points[count - 1 - i];
  }
  free(cleaned.points);
  free(cleaned.node_ids);
  model->stats.buildings_accepted++;
}

static void import_buildings(city_model_t* model, const osm_data_t* osm,
                             const city_import_options_t* options) {
  for (size_t i = 0; i < osm->way_count; i++) {
    const osm_way_t* way = &osm->ways[i];
    if (osm_tag(way->tags, way->tag_count, "building"))
      import_building(model, osm, way, options);
  }
  for (size_t i = 0; i < osm->relation_count; i++) {
    const osm_relation_t* relation = &osm->relations[i];
    if (osm_tag(relation->tags, relation->tag_count, "building")) {
      LOG_WARN("import",
               "relation %lld (multipolygon building) is not supported and was "
               "skipped",
               (long long)relation->id);
      model->stats.relations_skipped++;
    }
  }
}

/* ---- entry ---- */

static osm_bounds_t bounds_from_nodes(const osm_data_t* osm) {
  osm_bounds_t b = {90, 180, -90, -180};
  for (size_t i = 0; i < osm->node_count; i++) {
    const osm_node_t* n = &osm->nodes[i];
    if (n->lat < b.min_lat) b.min_lat = n->lat;
    if (n->lat > b.max_lat) b.max_lat = n->lat;
    if (n->lon < b.min_lon) b.min_lon = n->lon;
    if (n->lon > b.max_lon) b.max_lon = n->lon;
  }
  return b;
}

bool city_import(city_model_t* model, const osm_data_t* osm,
                 city_import_options_t options, const char* source_path) {
  memset(model, 0, sizeof(*model));
  if (osm->node_count == 0) {
    LOG_ERROR("import", "no nodes in the extract");
    return false;
  }
  model->bounds = osm->has_bounds ? osm->bounds : bounds_from_nodes(osm);
  model->projection = projection_for_bounds(model->bounds);
  snprintf(model->source, sizeof(model->source), "%s",
           source_path ? source_path : "");
  time_t now = time(NULL);
  strftime(model->import_date, sizeof(model->import_date), "%Y-%m-%d",
           gmtime(&now));
  import_roads(model, osm, &options);
  import_buildings(model, osm, &options);
  LOG_INFO("import",
           "%zu road segments, %zu junctions, %zu buildings; %zu ways ignored, "
           "%zu degenerate, %zu malformed buildings, %zu relations skipped",
           model->road_count, model->junction_count, model->building_count,
           model->stats.ways_ignored, model->stats.roads_degenerate,
           model->stats.buildings_malformed, model->stats.relations_skipped);
  return model->road_count > 0;
}
