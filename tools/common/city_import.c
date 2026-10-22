#include "city_import.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "log.h"
#include "triangulate.h"

city_import_options_t city_import_default_options(void) {
  city_import_options_t o;
  o.default_width[ROAD_CLASS_MOTORWAY] = 14.0f;
  o.default_width[ROAD_CLASS_PRIMARY] = 10.0f;
  o.default_width[ROAD_CLASS_SECONDARY] = 8.0f;
  o.default_width[ROAD_CLASS_TERTIARY] = 7.0f;
  o.default_width[ROAD_CLASS_RESIDENTIAL] = 6.0f;
  o.default_width[ROAD_CLASS_SERVICE] = 4.0f;
  o.default_width[ROAD_CLASS_UNCLASSIFIED] = 6.0f;
  o.default_width[ROAD_CLASS_PEDESTRIAN] = 5.0f;
  o.default_width[ROAD_CLASS_PATH] = 2.0f;
  o.default_width[ROAD_CLASS_WATERWAY] = 2.5f;
  o.storey_height = 3.2f;
  o.default_height_min = 6.0f;
  o.default_height_max = 15.0f;
  o.min_road_length = 0.5f;
  o.snap_distance = 3.5f;
  o.bend_radius = 8.0f;
  o.neighbour_reach = 80.0f;
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
               {"service", ROAD_CLASS_SERVICE},
               {"pedestrian", ROAD_CLASS_PEDESTRIAN},
               {"footway", ROAD_CLASS_PATH},
               {"cycleway", ROAD_CLASS_PATH},
               {"path", ROAD_CLASS_PATH},
               {"steps", ROAD_CLASS_PATH},
               {"bridleway", ROAD_CLASS_PATH}};
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
    if (cleaned->class == ROAD_CLASS_WATERWAY) {
      const char* kind = osm_tag(way->tags, way->tag_count, "waterway");
      road->width = strcmp(kind, "river") == 0   ? 12.0f
                    : strcmp(kind, "canal") == 0 ? 5.0f
                                                 : 2.5f;
    }
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

/* Where a way ends within snap_distance of another way's segment without
 * sharing a node, insert a point on that segment and give both the same
 * fresh node id, so they become a junction. Mappers leave such gaps. */
static void snap_dangling_ends(city_model_t* model, cleaned_way_t* ways,
                               size_t way_count,
                               const city_import_options_t* options) {
  const double reach = options->snap_distance;
  int64_t next_id = -1;
  for (size_t i = 0; i < way_count; i++) {
    for (int end = 0; end < 2; end++) {
      cleaned_way_t* way = &ways[i];
      if (way->point_count < 2) continue;
      size_t index = end ? way->point_count - 1 : 0;
      int64_t node = way->node_ids[index];
      bool shared = false;
      for (size_t k = 0; k < way_count && !shared; k++) {
        if (k == i) continue;
        for (size_t q = 0; q < ways[k].point_count && !shared; q++)
          if (ways[k].node_ids[q] == node) shared = true;
      }
      if (shared) continue;
      local_point_t p = way->points[index];
      /* A vertex of another way within reach: snap to that vertex and
       * share its node, no split needed. */
      {
        size_t vertex_way = way_count, vertex_index = 0;
        double vertex_distance = reach;
        for (size_t k = 0; k < way_count; k++) {
          if (k == i || ways[k].class == ROAD_CLASS_WATERWAY ||
              way->class == ROAD_CLASS_WATERWAY)
            continue;
          for (size_t q = 0; q < ways[k].point_count; q++) {
            /* Never onto a node this way already has, and never so close
             * to the neighbouring point that the way collapses. */
            bool own = false;
            for (size_t n = 0; n < way->point_count && !own; n++)
              if (way->node_ids[n] == ways[k].node_ids[q]) own = true;
            size_t neighbour = end ? index - 1 : 1;
            double from_neighbour =
                hypot(ways[k].points[q].x - way->points[neighbour].x,
                      ways[k].points[q].z - way->points[neighbour].z);
            if (own || from_neighbour < 1.0) continue;
            double distance =
                hypot(p.x - ways[k].points[q].x, p.z - ways[k].points[q].z);
            if (distance < vertex_distance) {
              vertex_distance = distance;
              vertex_way = k;
              vertex_index = q;
            }
          }
        }
        if (vertex_way != way_count) {
          way->points[index] = ways[vertex_way].points[vertex_index];
          way->node_ids[index] = ways[vertex_way].node_ids[vertex_index];
          model->stats.ends_snapped++;
          continue;
        }
      }
      size_t best_way = way_count, best_segment = 0;
      double best_distance = reach, best_t = 0;
      for (size_t k = 0; k < way_count; k++) {
        if (k == i || ways[k].class == ROAD_CLASS_WATERWAY ||
            way->class == ROAD_CLASS_WATERWAY)
          continue;
        for (size_t q = 1; q < ways[k].point_count; q++) {
          local_point_t a = ways[k].points[q - 1], b = ways[k].points[q];
          double abx = b.x - a.x, abz = b.z - a.z;
          double length2 = abx * abx + abz * abz;
          if (length2 < 1e-9) continue;
          double t = ((p.x - a.x) * abx + (p.z - a.z) * abz) / length2;
          if (t <= 0.02 || t >= 0.98)
            continue; /* near a vertex: let that vertex be it */
          double cx = a.x + abx * t, cz = a.z + abz * t;
          double distance = hypot(p.x - cx, p.z - cz);
          if (distance < best_distance) {
            best_distance = distance;
            best_way = k;
            best_segment = q;
            best_t = t;
          }
        }
      }
      if (best_way == way_count) continue;
      cleaned_way_t* other = &ways[best_way];
      local_point_t a = other->points[best_segment - 1],
                    b = other->points[best_segment];
      local_point_t on = {a.x + (b.x - a.x) * best_t,
                          a.z + (b.z - a.z) * best_t};
      local_point_t* grown_points = realloc(
          other->points, (other->point_count + 1) * sizeof(local_point_t));
      int64_t* grown_ids =
          realloc(other->node_ids, (other->point_count + 1) * sizeof(int64_t));
      if (!grown_points || !grown_ids) {
        if (grown_points) other->points = grown_points;
        if (grown_ids) other->node_ids = grown_ids;
        continue;
      }
      other->points = grown_points;
      other->node_ids = grown_ids;
      memmove(other->points + best_segment + 1, other->points + best_segment,
              (other->point_count - best_segment) * sizeof(local_point_t));
      memmove(other->node_ids + best_segment + 1,
              other->node_ids + best_segment,
              (other->point_count - best_segment) * sizeof(int64_t));
      other->points[best_segment] = on;
      other->node_ids[best_segment] = next_id;
      other->point_count++;
      way->points[index] = on;
      way->node_ids[index] = next_id;
      next_id--;
      model->stats.ends_snapped++;
    }
  }
}

/* Two segments meeting end to end at a node nothing else uses are one
 * road with a bend in it, not a junction: join them so the bend is
 * rounded like any other. Only like with like, and one-way roads only
 * head to tail. */
static bool join_roads(city_road_t* first, bool first_at_end,
                       city_road_t* second, bool second_at_end) {
  if (first->class != second->class || first->oneway != second->oneway ||
      fabs(first->width - second->width) > 0.01 ||
      first->layer != second->layer || first->bridge != second->bridge ||
      first->tunnel != second->tunnel)
    return false;
  if (first->oneway && (!first_at_end || second_at_end)) return false;
  size_t count = first->point_count + second->point_count - 1;
  local_point_t* points = malloc(count * sizeof(local_point_t));
  int64_t* ids = malloc(count * sizeof(int64_t));
  if (!points || !ids) {
    free(points);
    free(ids);
    return false;
  }
  size_t n = 0;
  for (size_t i = 0; i < first->point_count; i++) {
    size_t k = first_at_end ? i : first->point_count - 1 - i;
    points[n] = first->points[k], ids[n] = first->node_ids[k], n++;
  }
  for (size_t i = 1; i < second->point_count; i++) {
    size_t k = second_at_end ? second->point_count - 1 - i : i;
    points[n] = second->points[k], ids[n] = second->node_ids[k], n++;
  }
  free(first->points);
  free(first->node_ids);
  first->points = points, first->node_ids = ids, first->point_count = n;
  return true;
}

static void merge_through_roads(city_model_t* model) {
  for (size_t j = 0; j < model->junction_count;) {
    city_junction_t* junction = &model->junctions[j];
    size_t found[2] = {SIZE_MAX, SIZE_MAX};
    bool at_end[2] = {false, false};
    int hits = 0;
    for (size_t r = 0; r < model->road_count && hits <= 2; r++) {
      city_road_t* road = &model->roads[r];
      for (int end = 0; end < 2; end++) {
        size_t index = end ? road->point_count - 1 : 0;
        if (road->node_ids[index] != junction->node_id) continue;
        if (hits < 2) found[hits] = r, at_end[hits] = end;
        hits++;
      }
    }
    if (junction->degree != 2 || hits != 2 || found[0] == found[1] ||
        !join_roads(&model->roads[found[0]], at_end[0], &model->roads[found[1]],
                    at_end[1])) {
      j++;
      continue;
    }
    city_road_t* gone = &model->roads[found[1]];
    free(gone->points);
    free(gone->node_ids);
    memmove(gone, gone + 1,
            (model->road_count - found[1] - 1) * sizeof(city_road_t));
    model->road_count--;
    memmove(junction, junction + 1,
            (model->junction_count - j - 1) * sizeof(city_junction_t));
    model->junction_count--;
    model->stats.roads_merged++;
  }
}

/* Mapped roads are polylines with corners where the real road curves.
 * Each interior corner that turns more than a few degrees is replaced by
 * a quadratic arc tangent to both legs, reaching at most `radius` or
 * just under half a leg back from the corner, so neighbouring arcs never
 * overlap. The ends of a road, and so its junction geometry, do not
 * move. */
static void round_bends(city_model_t* model, float radius) {
  if (radius <= 0) return;
  for (size_t r = 0; r < model->road_count; r++) {
    city_road_t* road = &model->roads[r];
    if (road->point_count < 3) continue;
    size_t capacity = road->point_count * 12;
    local_point_t* points = malloc(capacity * sizeof(local_point_t));
    int64_t* ids = malloc(capacity * sizeof(int64_t));
    if (!points || !ids) {
      free(points);
      free(ids);
      return;
    }
    size_t n = 0;
    points[n] = road->points[0], ids[n] = road->node_ids[0], n++;
    for (size_t i = 1; i + 1 < road->point_count; i++) {
      local_point_t prev = road->points[i - 1], p = road->points[i],
                    next = road->points[i + 1];
      double l1 = hypot(p.x - prev.x, p.z - prev.z);
      double l2 = hypot(next.x - p.x, next.z - p.z);
      double cosine = l1 > 1e-9 && l2 > 1e-9
                          ? ((p.x - prev.x) * (next.x - p.x) +
                             (p.z - prev.z) * (next.z - p.z)) /
                                (l1 * l2)
                          : 1.0;
      double angle = acos(fmax(-1.0, fmin(1.0, cosine)));
      double d = fmin(radius, fmin(l1, l2) * 0.45);
      if (angle < 0.12 || d < 0.5) {
        points[n] = p, ids[n] = road->node_ids[i], n++;
        continue;
      }
      local_point_t a = {p.x - (p.x - prev.x) / l1 * d,
                         p.z - (p.z - prev.z) / l1 * d};
      local_point_t b = {p.x + (next.x - p.x) / l2 * d,
                         p.z + (next.z - p.z) / l2 * d};
      int samples = (int)(angle / 0.17) + 2;
      if (samples > 10) samples = 10;
      for (int k = 0; k <= samples; k++) {
        double t = (double)k / samples, u = 1.0 - t;
        local_point_t q = {u * u * a.x + 2 * u * t * p.x + t * t * b.x,
                           u * u * a.z + 2 * u * t * p.z + t * t * b.z};
        points[n] = q;
        ids[n] = k == samples / 2 ? road->node_ids[i] : 0;
        n++;
      }
      model->stats.bends_rounded++;
    }
    size_t last = road->point_count - 1;
    points[n] = road->points[last], ids[n] = road->node_ids[last], n++;
    free(road->points);
    free(road->node_ids);
    road->points = points, road->node_ids = ids, road->point_count = n;
  }
}

static void import_roads(city_model_t* model, const osm_data_t* osm,
                         const city_import_options_t* options) {
  cleaned_way_t* cleaned = calloc(osm->way_count, sizeof(cleaned_way_t));
  size_t cleaned_count = 0;
  if (!cleaned) return;
  for (size_t i = 0; i < osm->way_count; i++) {
    const osm_way_t* way = &osm->ways[i];
    const char* highway = osm_tag(way->tags, way->tag_count, "highway");
    const char* waterway = osm_tag(way->tags, way->tag_count, "waterway");
    bool closed = way->node_count >= 4 &&
                  way->node_refs[0] == way->node_refs[way->node_count - 1];
    if (!highway && !(waterway && !closed)) continue;
    model->stats.ways_total++;
    if (tag_is_yes(osm_tag(way->tags, way->tag_count, "area"))) {
      model->stats.ways_total--; /* a square: handled with the areas */
      continue;
    }
    const char* tunnel = osm_tag(way->tags, way->tag_count, "tunnel");
    if (tunnel && strcmp(tunnel, "no") != 0) {
      /* Underpasses and passages under buildings are not on the surface
       * we draw; drawn there they run across gardens and rooftops. */
      model->stats.tunnels_skipped++;
      model->stats.ways_ignored++;
      continue;
    }
    const char* service = osm_tag(way->tags, way->tag_count, "service");
    const char* access = osm_tag(way->tags, way->tag_count, "access");
    if (highway && strcmp(highway, "service") == 0 &&
        ((service && (strcmp(service, "driveway") == 0 ||
                      strcmp(service, "parking_aisle") == 0 ||
                      strcmp(service, "drive-through") == 0 ||
                      strcmp(service, "emergency_access") == 0)) ||
         (access &&
          (strcmp(access, "private") == 0 || strcmp(access, "no") == 0)))) {
      model->stats.ways_ignored++; /* clutter inside blocks and car parks */
      continue;
    }
    const char* footway = osm_tag(way->tags, way->tag_count, "footway");
    if (footway && (strcmp(footway, "sidewalk") == 0 ||
                    strcmp(footway, "crossing") == 0)) {
      model->stats.ways_ignored++; /* the generated pavements cover these */
      continue;
    }
    road_class_t class;
    if (!highway) {
      bool is_water =
          strcmp(waterway, "stream") == 0 || strcmp(waterway, "river") == 0 ||
          strcmp(waterway, "canal") == 0 || strcmp(waterway, "ditch") == 0 ||
          strcmp(waterway, "drain") == 0;
      if (!is_water) {
        model->stats.ways_ignored++;
        continue;
      }
      class = ROAD_CLASS_WATERWAY;
    } else if (!classify_highway(highway, options->include_service_roads,
                                 &class)) {
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

  snap_dangling_ends(model, cleaned, cleaned_count, options);

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
  merge_through_roads(model);
  round_bends(model, options->bend_radius);
}

/* ---- rings: closed outlines from ways or multipolygon relations ---- */

typedef struct {
  local_point_t* points;
  int64_t* node_ids;
  size_t count;
} ring_t;

static void ring_free(ring_t* ring) {
  free(ring->points);
  free(ring->node_ids);
  memset(ring, 0, sizeof(*ring));
}

/* A closed way as a ring: projected, deduplicated, last point dropped. */
static bool ring_from_way(const osm_data_t* osm, const projection_t* projection,
                          const osm_way_t* way, ring_t* ring) {
  cleaned_way_t cleaned;
  bool closed = way->node_count >= 4 &&
                way->node_refs[0] == way->node_refs[way->node_count - 1];
  if (!closed || !clean_way(osm, projection, way, &cleaned)) return false;
  size_t count = cleaned.point_count;
  if (count > 1 && cleaned.node_ids[count - 1] == cleaned.node_ids[0]) count--;
  ring->points = cleaned.points;
  ring->node_ids = cleaned.node_ids;
  ring->count = count;
  return true;
}

/* Chains a relation's outer member ways into closed rings, reversing ways
 * as needed. Calls `emit` for each closed ring; returns how many member
 * ways were left over unclosed. */
typedef void (*ring_visitor_t)(city_model_t* model,
                               const osm_relation_t* relation, ring_t* ring,
                               const city_import_options_t* options);

static size_t assemble_rings(city_model_t* model, const osm_data_t* osm,
                             const osm_relation_t* relation,
                             const city_import_options_t* options,
                             const char* role, ring_visitor_t emit) {
  size_t member_count = relation->member_count;
  const osm_way_t** ways =
      calloc(member_count ? member_count : 1, sizeof(*ways));
  bool* used = calloc(member_count ? member_count : 1, sizeof(bool));
  if (!ways || !used) {
    free(ways);
    free(used);
    return 0;
  }
  size_t way_count = 0;
  for (size_t m = 0; m < member_count; m++) {
    const osm_member_t* member = &relation->members[m];
    if (member->type != OSM_MEMBER_WAY) continue;
    bool wanted =
        strcmp(role, "outer") == 0
            ? (member->role[0] == '\0' || strcmp(member->role, "outer") == 0)
            : strcmp(member->role, role) == 0;
    if (!wanted) continue;
    const osm_way_t* way = osm_find_way(osm, member->ref);
    if (way && way->node_count >= 2) ways[way_count++] = way;
  }
  size_t unclosed = 0;
  for (size_t first = 0; first < way_count; first++) {
    if (used[first]) continue;
    size_t capacity = 64, count = 0;
    local_point_t* points = malloc(capacity * sizeof(local_point_t));
    int64_t* ids = malloc(capacity * sizeof(int64_t));
    if (!points || !ids) {
      free(points);
      free(ids);
      break;
    }
    int64_t start_node = ways[first]->node_refs[0];
    int64_t end_node = start_node;
    size_t current = first;
    bool forward = true;
    bool closed = false;
    for (size_t step = 0; step < way_count && !closed; step++) {
      const osm_way_t* way = ways[current];
      used[current] = true;
      size_t n = way->node_count;
      for (size_t k = 0; k < n; k++) {
        size_t index = forward ? k : n - 1 - k;
        if (count == capacity) {
          capacity *= 2;
          local_point_t* grown_points =
              realloc(points, capacity * sizeof(local_point_t));
          int64_t* grown_ids = realloc(ids, capacity * sizeof(int64_t));
          if (!grown_points || !grown_ids) {
            free(grown_points ? grown_points : points);
            free(grown_ids ? grown_ids : ids);
            free(ways);
            free(used);
            return unclosed;
          }
          points = grown_points;
          ids = grown_ids;
        }
        const osm_node_t* node = osm_find_node(osm, way->node_refs[index]);
        if (!node) continue;
        if (count > 0 && ids[count - 1] == node->id) continue;
        points[count] =
            projection_project(&model->projection, node->lat, node->lon);
        ids[count] = node->id;
        count++;
      }
      end_node = forward ? way->node_refs[n - 1] : way->node_refs[0];
      if (end_node == start_node) {
        closed = true;
        break;
      }
      /* Find an unused way starting or ending at end_node. */
      bool found = false;
      for (size_t candidate = 0; candidate < way_count && !found; candidate++) {
        if (used[candidate]) continue;
        const osm_way_t* next = ways[candidate];
        if (next->node_refs[0] == end_node) {
          current = candidate;
          forward = true;
          found = true;
        } else if (next->node_refs[next->node_count - 1] == end_node) {
          current = candidate;
          forward = false;
          found = true;
        }
      }
      if (!found) break;
    }
    if (closed && count > 1 && ids[count - 1] == ids[0]) count--;
    if (closed && count >= 3) {
      ring_t ring = {points, ids, count};
      emit(model, relation, &ring, options);
      ring_free(&ring);
    } else {
      unclosed++;
      free(points);
      free(ids);
    }
  }
  free(ways);
  free(used);
  return unclosed;
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

/* Copies a ring anticlockwise into a fresh array. */
static local_point_t* anticlockwise_copy(const local_point_t* points,
                                         size_t count) {
  local_point_t* copy = malloc(count * sizeof(local_point_t));
  if (!copy) return NULL;
  if (signed_area(points, count) > 0) {
    memcpy(copy, points, count * sizeof(local_point_t));
  } else {
    for (size_t i = 0; i < count; i++) copy[i] = points[count - 1 - i];
  }
  return copy;
}

/* Kinds of building whose height is known without a tag. Zero means no
 * idea. */
static float typical_height(const char* category,
                            const city_import_options_t* options) {
  static const char* two_storey[] = {
      "house", "detached", "semidetached_house", "terrace", "farm", "villa"};
  static const char* low[] = {"hut",     "shed",  "garage", "garages",   "roof",
                              "carport", "kiosk", "cabin",  "greenhouse"};
  for (size_t i = 0; i < sizeof(two_storey) / sizeof(two_storey[0]); i++)
    if (strcmp(category, two_storey[i]) == 0)
      return 2.0f * options->storey_height;
  for (size_t i = 0; i < sizeof(low) / sizeof(low[0]); i++)
    if (strcmp(category, low[i]) == 0) return 3.0f;
  if (strcmp(category, "bungalow") == 0) return options->storey_height;
  if (strcmp(category, "apartments") == 0) return 5.0f * options->storey_height;
  if (strcmp(category, "church") == 0 || strcmp(category, "cathedral") == 0)
    return 16.0f;
  return 0;
}

static local_point_t ring_centre(const local_point_t* points, size_t count) {
  local_point_t c = {0, 0};
  for (size_t i = 0; i < count; i++) c.x += points[i].x, c.z += points[i].z;
  if (count) c.x /= (double)count, c.z /= (double)count;
  return c;
}

static int compare_floats(const void* a, const void* b) {
  float fa = *(const float*)a, fb = *(const float*)b;
  return (fa > fb) - (fa < fb);
}

/* Untagged buildings of no telling kind take the median height of the
 * tagged buildings around them, which keeps a street of palazzi at the
 * palazzi's height; with nobody nearby, a deterministic default. */
static void settle_heights(city_model_t* model,
                           const city_import_options_t* options) {
  size_t count = model->building_count;
  float* nearby = malloc((count ? count : 1) * sizeof(float));
  if (!nearby) return;
  for (size_t b = 0; b < count; b++) {
    city_building_t* building = &model->buildings[b];
    if (building->height_source != HEIGHT_DEFAULT) continue;
    local_point_t centre = ring_centre(building->points, building->point_count);
    size_t found = 0;
    for (size_t o = 0; o < count; o++) {
      const city_building_t* other = &model->buildings[o];
      if (other->height_source != HEIGHT_TAGGED &&
          other->height_source != HEIGHT_FROM_LEVELS)
        continue;
      local_point_t oc = ring_centre(other->points, other->point_count);
      if (hypot(oc.x - centre.x, oc.z - centre.z) <= options->neighbour_reach)
        nearby[found++] = other->height;
    }
    unsigned hash = stable_hash(building->source_id);
    if (found > 0) {
      qsort(nearby, found, sizeof(float), compare_floats);
      float median = found % 2
                         ? nearby[found / 2]
                         : 0.5f * (nearby[found / 2 - 1] + nearby[found / 2]);
      /* Streets are not uniform: a storey up or down, mostly the same. */
      static const int storeys[4] = {-1, 0, 0, 1};
      building->height =
          fmaxf(2.0f * options->storey_height,
                median + (float)storeys[hash % 4] * options->storey_height);
      building->height_source = HEIGHT_FROM_NEIGHBOURS;
    } else {
      /* Two to four storeys, weighted towards three. */
      static const int storeys[10] = {2, 2, 3, 3, 3, 3, 4, 4, 4, 2};
      float height = (float)storeys[hash % 10] * options->storey_height;
      building->height = fminf(fmaxf(height, options->default_height_min),
                               options->default_height_max);
      model->stats.heights_defaulted++;
    }
    if (building->levels == 0)
      building->levels =
          (int)(building->height / options->storey_height + 0.5f);
  }
  free(nearby);
}

static void add_building(city_model_t* model, int64_t source_id,
                         const osm_tag_t* tags, size_t tag_count,
                         const ring_t* ring,
                         const city_import_options_t* options) {
  if (ring->count < 3 || fabs(signed_area(ring->points, ring->count)) < 1.0) {
    LOG_WARN("import", "building %lld has no area", (long long)source_id);
    model->stats.buildings_malformed++;
    return;
  }
  city_building_t* building = city_model_add_building(model);
  if (!building) return;
  building->source_id = source_id;
  const char* category = osm_tag(tags, tag_count, "building");
  snprintf(building->category, sizeof(building->category), "%s",
           category && category[0] ? category : "yes");
  const char* name = osm_tag(tags, tag_count, "name");
  if (name) {
    snprintf(building->name, sizeof(building->name), "%s", name);
    for (char* c = building->name; *c; c++)
      if (*c == ' ') *c = '_';
  }
  const char* height = osm_tag(tags, tag_count, "height");
  const char* levels = osm_tag(tags, tag_count, "building:levels");
  bool ok = false;
  if (height) building->height = parse_metres(height, &ok);
  if (ok) {
    building->height_source = HEIGHT_TAGGED;
  } else if (levels && atoi(levels) > 0) {
    building->levels = atoi(levels);
    building->height = (float)building->levels * options->storey_height;
    building->height_source = HEIGHT_FROM_LEVELS;
  } else {
    float typical = typical_height(building->category, options);
    if (typical > 0) {
      building->height = typical;
      building->height_source = HEIGHT_FROM_CATEGORY;
    } else {
      building->height = 0; /* settled once the neighbours are known */
      building->height_source = HEIGHT_DEFAULT;
    }
  }
  if (building->levels == 0 && building->height > 0)
    building->levels = (int)(building->height / options->storey_height + 0.5f);
  building->points = anticlockwise_copy(ring->points, ring->count);
  building->point_count = building->points ? ring->count : 0;
  model->stats.buildings_accepted++;
}

static void import_building_way(city_model_t* model, const osm_data_t* osm,
                                const osm_way_t* way,
                                const city_import_options_t* options) {
  ring_t ring;
  if (!ring_from_way(osm, &model->projection, way, &ring)) {
    LOG_WARN("import", "building %lld is not a closed ring",
             (long long)way->id);
    model->stats.buildings_malformed++;
    return;
  }
  add_building(model, way->id, way->tags, way->tag_count, &ring, options);
  ring_free(&ring);
}

static void emit_relation_building(city_model_t* model,
                                   const osm_relation_t* relation, ring_t* ring,
                                   const city_import_options_t* options) {
  add_building(model, relation->id, relation->tags, relation->tag_count, ring,
               options);
}

static bool point_in_ring(local_point_t p, const local_point_t* ring,
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

/* An inner ring becomes a courtyard of the relation's building that
 * contains it, stored clockwise so its walls face inwards. */
static void emit_relation_hole(city_model_t* model,
                               const osm_relation_t* relation, ring_t* ring,
                               const city_import_options_t* options) {
  (void)options;
  for (size_t b = 0; b < model->building_count; b++) {
    city_building_t* building = &model->buildings[b];
    if (building->source_id != relation->id) continue;
    if (!point_in_ring(ring->points[0], building->points,
                       building->point_count))
      continue;
    local_point_t* copy = anticlockwise_copy(ring->points, ring->count);
    if (!copy) return;
    for (size_t i = 0; i < ring->count / 2; i++) { /* reverse to clockwise */
      local_point_t swap = copy[i];
      copy[i] = copy[ring->count - 1 - i];
      copy[ring->count - 1 - i] = swap;
    }
    if (!city_building_add_hole(building, copy, ring->count)) free(copy);
    return;
  }
  LOG_WARN("import", "relation %lld: an inner ring lies in no outer ring",
           (long long)relation->id);
}

/* ---- barriers ---- */

static bool classify_barrier(const osm_way_t* way, barrier_kind_t* kind) {
  const char* barrier = osm_tag(way->tags, way->tag_count, "barrier");
  if (!barrier) return false;
  if (strcmp(barrier, "wall") == 0 || strcmp(barrier, "retaining_wall") == 0 ||
      strcmp(barrier, "city_wall") == 0) {
    *kind = BARRIER_WALL;
  } else if (strcmp(barrier, "fence") == 0) {
    *kind = BARRIER_FENCE;
  } else if (strcmp(barrier, "hedge") == 0) {
    *kind = BARRIER_HEDGE;
  } else {
    return false; /* gates, bollards, kerbs: not drawn as a line */
  }
  return true;
}

static void import_barriers(city_model_t* model, const osm_data_t* osm) {
  for (size_t i = 0; i < osm->way_count; i++) {
    const osm_way_t* way = &osm->ways[i];
    barrier_kind_t kind;
    if (osm_tag(way->tags, way->tag_count, "building") ||
        osm_tag(way->tags, way->tag_count, "highway") ||
        !classify_barrier(way, &kind))
      continue;
    cleaned_way_t cleaned = {0};
    bool kept = clean_way(osm, &model->projection, way, &cleaned) &&
                cleaned.point_count >= 2 &&
                polyline_length(cleaned.points, cleaned.point_count) >= 1.5;
    city_barrier_t* barrier = kept ? city_model_add_barrier(model) : NULL;
    if (barrier) {
      barrier->source_id = way->id;
      barrier->kind = kind;
      barrier->points = cleaned.points; /* the model owns them now */
      barrier->point_count = cleaned.point_count;
    } else {
      free(cleaned.points);
    }
    free(cleaned.node_ids);
  }
}

/* ---- the sea ---- */

static void add_area(city_model_t* model, int64_t source_id, area_kind_t kind,
                     const ring_t* ring);

typedef struct {
  double low_x, low_z, high_x, high_z;
} extent_t;

/* Where a ray from p along (dx, dz) leaves the extent. */
static local_point_t leave_extent(const extent_t* e, local_point_t p, double dx,
                                  double dz) {
  double t = 1e9;
  if (dx > 1e-9) t = fmin(t, (e->high_x - p.x) / dx);
  if (dx < -1e-9) t = fmin(t, (e->low_x - p.x) / dx);
  if (dz > 1e-9) t = fmin(t, (e->high_z - p.z) / dz);
  if (dz < -1e-9) t = fmin(t, (e->low_z - p.z) / dz);
  if (t < 0 || t > 1e8) t = 0;
  local_point_t out = {p.x + dx * t, p.z + dz * t};
  return out;
}

/* Position round the extent's edge, from its low corner, going along the
 * low z edge first. */
static double edge_position(const extent_t* e, local_point_t p) {
  double w = e->high_x - e->low_x, h = e->high_z - e->low_z;
  double d[4] = {fabs(p.z - e->low_z), fabs(p.x - e->high_x),
                 fabs(p.z - e->high_z), fabs(p.x - e->low_x)};
  int edge = 0;
  for (int i = 1; i < 4; i++)
    if (d[i] < d[edge]) edge = i;
  switch (edge) {
    case 0:
      return p.x - e->low_x;
    case 1:
      return w + (p.z - e->low_z);
    case 2:
      return w + h + (e->high_x - p.x);
    default:
      return 2 * w + h + (e->high_z - p.z);
  }
}

static local_point_t edge_corner(const extent_t* e, int corner) {
  static const int cx[4] = {0, 1, 1, 0}, cz[4] = {0, 0, 1, 1};
  local_point_t p = {cx[corner] ? e->high_x : e->low_x,
                     cz[corner] ? e->high_z : e->low_z};
  return p;
}

/* Cuts a convex polygon to a rectangle (Sutherland-Hodgman); returns the
 * point count of the result, which may be under three. */
static size_t clip_to_rectangle(const local_point_t* in, size_t count,
                                const extent_t* box, local_point_t* out) {
  local_point_t a[16], b[16];
  size_t n = count > 8 ? 8 : count;
  memcpy(a, in, n * sizeof(local_point_t));
  for (int side = 0; side < 4 && n > 0; side++) {
    size_t m = 0;
    for (size_t i = 0; i < n; i++) {
      local_point_t p = a[i], q = a[(i + 1) % n];
      double pv, qv;
      switch (side) {
        case 0:
          pv = p.x - box->low_x, qv = q.x - box->low_x;
          break;
        case 1:
          pv = box->high_x - p.x, qv = box->high_x - q.x;
          break;
        case 2:
          pv = p.z - box->low_z, qv = q.z - box->low_z;
          break;
        default:
          pv = box->high_z - p.z, qv = box->high_z - q.z;
          break;
      }
      if (pv >= 0 && m < 15) b[m++] = p;
      if ((pv >= 0) != (qv >= 0) && m < 15) {
        double t = pv / (pv - qv);
        local_point_t x = {p.x + (q.x - p.x) * t, p.z + (q.z - p.z) * t};
        b[m++] = x;
      }
    }
    memcpy(a, b, m * sizeof(local_point_t));
    n = m;
  }
  memcpy(out, a, n * sizeof(local_point_t));
  return n;
}

/* A stretch of coastline inside the extract, closed off at the extract's
 * edge at both ends: it starts and ends on the edge. */
typedef struct {
  local_point_t* points;
  size_t count;
  double s_start, s_end; /* positions round the edge */
  int64_t source_id;
  bool linked;
} coast_run_t;

/* Fills one ring of sea, clockwise (the sea is on the right of the coast):
 * triangulated, and every triangle cut into 100 m pieces so that each
 * belongs to a chunk near it. */
static void fill_sea(city_model_t* model, int64_t source_id,
                     local_point_t* ring, size_t count) {
  if (count < 3 || signed_area(ring, count) >= 0) return;
  /* Anticlockwise for the triangulator. */
  for (size_t i = 0; i < count / 2; i++) {
    local_point_t tmp = ring[i];
    ring[i] = ring[count - 1 - i];
    ring[count - 1 - i] = tmp;
  }
  uint32_t* indices = malloc(3 * (count - 2) * sizeof(uint32_t));
  if (!indices) return;
  bool clean;
  size_t triangles = triangulate_polygon(ring, count, indices, &clean);
  const double cell = 100.0;
  for (size_t t = 0; t < triangles; t++) {
    local_point_t tri[3] = {ring[indices[3 * t]], ring[indices[3 * t + 1]],
                            ring[indices[3 * t + 2]]};
    double min_x = fmin(tri[0].x, fmin(tri[1].x, tri[2].x));
    double max_x = fmax(tri[0].x, fmax(tri[1].x, tri[2].x));
    double min_z = fmin(tri[0].z, fmin(tri[1].z, tri[2].z));
    double max_z = fmax(tri[0].z, fmax(tri[1].z, tri[2].z));
    for (double gx = floor(min_x / cell); gx * cell <= max_x; gx += 1.0)
      for (double gz = floor(min_z / cell); gz * cell <= max_z; gz += 1.0) {
        extent_t box = {gx * cell, gz * cell, (gx + 1) * cell, (gz + 1) * cell};
        local_point_t piece[16];
        size_t pn = clip_to_rectangle(tri, 3, &box, piece);
        if (pn < 3) continue;
        ring_t r = {piece, NULL, pn};
        add_area(model, source_id, AREA_WATER, &r);
      }
  }
  free(indices);
}

/* Joins the runs end to start round the edge of the extract, clockwise (the
 * sea being on the right of the coast), into rings of sea, and fills them. */
static void sea_from_runs(city_model_t* model, coast_run_t* runs,
                          size_t run_count, const extent_t* extent) {
  double perimeter = 2 * (extent->high_x - extent->low_x) +
                     2 * (extent->high_z - extent->low_z);
  double corners[4];
  for (int c = 0; c < 4; c++)
    corners[c] = edge_position(extent, edge_corner(extent, c));
  size_t total = 0;
  for (size_t i = 0; i < run_count; i++) total += runs[i].count + 6;
  local_point_t* ring = malloc((total + 8) * sizeof(local_point_t));
  if (!ring) return;
  for (size_t first = 0; first < run_count; first++) {
    if (runs[first].linked) continue;
    size_t n = 0, current = first;
    bool closed = false;
    for (size_t guard = 0; guard <= run_count && !closed; guard++) {
      coast_run_t* run = &runs[current];
      run->linked = true;
      for (size_t k = 0; k < run->count; k++) ring[n++] = run->points[k];
      /* On from this run's end, clockwise, to the nearest start. */
      size_t next = run_count;
      double best = 1e30;
      for (size_t q = 0; q < run_count; q++) {
        if (runs[q].linked && q != first) continue;
        double d = fmod(runs[q].s_start - run->s_end + perimeter, perimeter);
        if (d < best) best = d, next = q;
      }
      if (next == run_count) break;
      for (int c = 0; c < 4; c++) {
        double along = fmod(corners[c] - run->s_end + perimeter, perimeter);
        if (along > 0.01 && along < best - 0.01)
          ring[n++] = edge_corner(extent, c);
      }
      if (next == first)
        closed = true;
      else
        current = next;
    }
    if (closed) fill_sea(model, runs[first].source_id, ring, n);
  }
  free(ring);
}

/* Stretches of the coastlines that lie inside the extract. */
static bool add_run(coast_run_t** runs, size_t* count, size_t* capacity,
                    const local_point_t* points, size_t n, int64_t source_id,
                    const extent_t* extent) {
  if (n < 2) return true;
  local_point_t* own = malloc((n + 2) * sizeof(local_point_t));
  if (!own) return false;
  size_t m = 1; /* room for the start at the edge */
  for (size_t i = 0; i < n; i++) {
    local_point_t p = {fmin(fmax(points[i].x, extent->low_x), extent->high_x),
                       fmin(fmax(points[i].z, extent->low_z), extent->high_z)};
    if (m == 1 || hypot(p.x - own[m - 1].x, p.z - own[m - 1].z) > 0.1)
      own[m++] = p;
  }
  if (m < 3) {
    free(own);
    return true;
  }
  /* Carry both ends straight on to the edge. */
  own[0] =
      leave_extent(extent, own[1], own[1].x - own[2].x, own[1].z - own[2].z);
  local_point_t end =
      leave_extent(extent, own[m - 1], own[m - 1].x - own[m - 2].x,
                   own[m - 1].z - own[m - 2].z);
  own[m++] = end;
  if (*count == *capacity) {
    size_t grown = *capacity ? *capacity * 2 : 8;
    coast_run_t* bigger = realloc(*runs, grown * sizeof(coast_run_t));
    if (!bigger) {
      free(own);
      return false;
    }
    *runs = bigger;
    *capacity = grown;
  }
  coast_run_t* run = &(*runs)[(*count)++];
  run->points = own;
  run->count = m;
  run->s_start = edge_position(extent, own[0]);
  run->s_end = edge_position(extent, own[m - 1]);
  run->source_id = source_id;
  run->linked = false;
  return true;
}

/* A coastline has the land on its left and the sea on its right. The sea is
 * laid as water over everything to the right of the coast, out to the edge of
 * the extract, with a low sea wall along the coast to keep cars out. The
 * coast is often cut into several ways and may leave the extract and come
 * back (a harbour mole), so the ways are joined end to end first, split where
 * nodes are missing, and the stretches joined round the edge. */
static void import_coast(city_model_t* model, const osm_data_t* osm) {
  local_point_t corner_a = projection_project(
      &model->projection, model->bounds.min_lat, model->bounds.min_lon);
  local_point_t corner_b = projection_project(
      &model->projection, model->bounds.max_lat, model->bounds.max_lon);
  extent_t extent = {fmin(corner_a.x, corner_b.x), fmin(corner_a.z, corner_b.z),
                     fmax(corner_a.x, corner_b.x),
                     fmax(corner_a.z, corner_b.z)};
  /* The coastline ways as chains of node ids, joined where one ends at the
   * node another starts from. */
  size_t chain_count = 0;
  int64_t** chains = NULL;
  size_t* chain_sizes = NULL;
  int64_t* chain_sources = NULL;
  for (size_t i = 0; i < osm->way_count; i++) {
    const osm_way_t* way = &osm->ways[i];
    const char* natural = osm_tag(way->tags, way->tag_count, "natural");
    if (!natural || strcmp(natural, "coastline") != 0 || way->node_count < 2)
      continue;
    int64_t** grown_chains =
        realloc(chains, (chain_count + 1) * sizeof(*chains));
    size_t* grown_sizes =
        realloc(chain_sizes, (chain_count + 1) * sizeof(size_t));
    int64_t* grown_sources =
        realloc(chain_sources, (chain_count + 1) * sizeof(int64_t));
    if (grown_chains) chains = grown_chains;
    if (grown_sizes) chain_sizes = grown_sizes;
    if (grown_sources) chain_sources = grown_sources;
    if (!grown_chains || !grown_sizes || !grown_sources) break;
    chains[chain_count] = malloc(way->node_count * sizeof(int64_t));
    if (!chains[chain_count]) break;
    memcpy(chains[chain_count], way->node_refs,
           way->node_count * sizeof(int64_t));
    chain_sizes[chain_count] = way->node_count;
    chain_sources[chain_count] = way->id;
    chain_count++;
  }
  for (bool joined = true; joined;) {
    joined = false;
    for (size_t a = 0; a < chain_count && !joined; a++)
      for (size_t b = 0; b < chain_count && !joined; b++) {
        if (a == b || chains[a][chain_sizes[a] - 1] != chains[b][0]) continue;
        int64_t* both = realloc(
            chains[a], (chain_sizes[a] + chain_sizes[b]) * sizeof(int64_t));
        if (!both) continue;
        memcpy(both + chain_sizes[a], chains[b] + 1,
               (chain_sizes[b] - 1) * sizeof(int64_t));
        chains[a] = both;
        chain_sizes[a] += chain_sizes[b] - 1;
        free(chains[b]);
        chains[b] = chains[chain_count - 1];
        chain_sizes[b] = chain_sizes[chain_count - 1];
        chain_sources[b] = chain_sources[chain_count - 1];
        chain_count--;
        joined = true;
      }
  }
  coast_run_t* runs = NULL;
  size_t run_count = 0, run_capacity = 0;
  for (size_t c = 0; c < chain_count; c++) {
    bool island = chains[c][0] == chains[c][chain_sizes[c] - 1];
    local_point_t* stretch = malloc(chain_sizes[c] * sizeof(local_point_t));
    size_t n = 0;
    for (size_t i = 0; stretch && i <= chain_sizes[c] && !island; i++) {
      const osm_node_t* node =
          i < chain_sizes[c] ? osm_find_node(osm, chains[c][i]) : NULL;
      if (node) {
        stretch[n++] =
            projection_project(&model->projection, node->lat, node->lon);
        continue;
      }
      /* A node is missing, so the coast is out of the extract here. */
      if (n >= 2) {
        add_run(&runs, &run_count, &run_capacity, stretch, n, chain_sources[c],
                &extent);
        city_barrier_t* wall = city_model_add_barrier(model);
        if (wall) {
          wall->points = malloc(n * sizeof(local_point_t));
          if (wall->points) {
            memcpy(wall->points, stretch, n * sizeof(local_point_t));
            wall->point_count = n;
            wall->source_id = chain_sources[c];
            wall->kind = BARRIER_SEAWALL;
          }
        }
      }
      n = 0;
    }
    free(stretch);
  }
  sea_from_runs(model, runs, run_count, &extent);
  for (size_t i = 0; i < run_count; i++) free(runs[i].points);
  free(runs);
  for (size_t c = 0; c < chain_count; c++) free(chains[c]);
  free(chains);
  free(chain_sizes);
  free(chain_sources);
}

/* ---- areas ---- */

static bool classify_area(const osm_tag_t* tags, size_t tag_count,
                          area_kind_t* kind) {
  const char* natural = osm_tag(tags, tag_count, "natural");
  const char* waterway = osm_tag(tags, tag_count, "waterway");
  const char* landuse = osm_tag(tags, tag_count, "landuse");
  const char* leisure = osm_tag(tags, tag_count, "leisure");
  const char* amenity = osm_tag(tags, tag_count, "amenity");
  const char* highway = osm_tag(tags, tag_count, "highway");
  const char* place = osm_tag(tags, tag_count, "place");
  if ((natural && strcmp(natural, "water") == 0) ||
      (waterway && strcmp(waterway, "riverbank") == 0) ||
      (landuse &&
       (strcmp(landuse, "reservoir") == 0 || strcmp(landuse, "basin") == 0))) {
    *kind = AREA_WATER;
    return true;
  }
  if (amenity && strcmp(amenity, "parking") == 0) {
    *kind = AREA_PARKING;
    return true;
  }
  if ((highway && strcmp(highway, "pedestrian") == 0 &&
       tag_is_yes(osm_tag(tags, tag_count, "area"))) ||
      (place && strcmp(place, "square") == 0)) {
    *kind = AREA_SQUARE;
    return true;
  }
  static const char* green_leisure[] = {"park", "garden", "pitch", "playground",
                                        "dog_park"};
  static const char* green_landuse[] = {"grass",     "forest",
                                        "meadow",    "village_green",
                                        "flowerbed", "recreation_ground",
                                        "cemetery",  "orchard",
                                        "farmland",  "greenfield"};
  static const char* green_natural[] = {"wood", "scrub", "grassland", "heath"};
  for (size_t i = 0;
       leisure && i < sizeof(green_leisure) / sizeof(green_leisure[0]); i++)
    if (strcmp(leisure, green_leisure[i]) == 0) {
      *kind = AREA_PARK;
      return true;
    }
  for (size_t i = 0;
       landuse && i < sizeof(green_landuse) / sizeof(green_landuse[0]); i++)
    if (strcmp(landuse, green_landuse[i]) == 0) {
      *kind = AREA_PARK;
      return true;
    }
  for (size_t i = 0;
       natural && i < sizeof(green_natural) / sizeof(green_natural[0]); i++)
    if (strcmp(natural, green_natural[i]) == 0) {
      *kind = AREA_PARK;
      return true;
    }
  return false;
}

static void add_area(city_model_t* model, int64_t source_id, area_kind_t kind,
                     const ring_t* ring) {
  if (ring->count < 3 || fabs(signed_area(ring->points, ring->count)) < 1.0)
    return;
  city_area_t* area = city_model_add_area(model);
  if (!area) return;
  area->source_id = source_id;
  area->kind = kind;
  area->points = anticlockwise_copy(ring->points, ring->count);
  area->point_count = area->points ? ring->count : 0;
  model->stats.areas_accepted++;
}

static void emit_relation_area(city_model_t* model,
                               const osm_relation_t* relation, ring_t* ring,
                               const city_import_options_t* options) {
  (void)options;
  area_kind_t kind;
  if (classify_area(relation->tags, relation->tag_count, &kind))
    add_area(model, relation->id, kind, ring);
}

static void import_buildings_and_areas(city_model_t* model,
                                       const osm_data_t* osm,
                                       const city_import_options_t* options) {
  for (size_t i = 0; i < osm->way_count; i++) {
    const osm_way_t* way = &osm->ways[i];
    area_kind_t kind;
    if (osm_tag(way->tags, way->tag_count, "building")) {
      import_building_way(model, osm, way, options);
    } else if (classify_area(way->tags, way->tag_count, &kind)) {
      ring_t ring;
      if (ring_from_way(osm, &model->projection, way, &ring)) {
        add_area(model, way->id, kind, &ring);
        ring_free(&ring);
      }
    }
  }
  for (size_t i = 0; i < osm->relation_count; i++) {
    const osm_relation_t* relation = &osm->relations[i];
    const char* type = osm_tag(relation->tags, relation->tag_count, "type");
    if (!type || strcmp(type, "multipolygon") != 0) continue;
    area_kind_t kind;
    size_t unclosed = 0;
    if (osm_tag(relation->tags, relation->tag_count, "building")) {
      unclosed = assemble_rings(model, osm, relation, options, "outer",
                                emit_relation_building);
      unclosed += assemble_rings(model, osm, relation, options, "inner",
                                 emit_relation_hole);
    } else if (classify_area(relation->tags, relation->tag_count, &kind)) {
      unclosed = assemble_rings(model, osm, relation, options, "outer",
                                emit_relation_area);
    } else {
      continue;
    }
    if (unclosed > 0) {
      LOG_WARN("import", "relation %lld: %zu outer rings do not close",
               (long long)relation->id, unclosed);
      model->stats.rings_unclosed += unclosed;
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
  import_buildings_and_areas(model, osm, &options);
  import_barriers(model, osm);
  import_coast(model, osm);
  for (size_t n = 0; n < osm->node_count; n++) {
    if (osm->nodes[n].furniture != OSM_FURNITURE_NONE) {
      city_furniture_t* grown = realloc(
          model->furniture, (model->furniture_count + 1) * sizeof(*grown));
      if (!grown) break;
      model->furniture = grown;
      local_point_t p = projection_project(
          &model->projection, osm->nodes[n].lat, osm->nodes[n].lon);
      city_furniture_t item = {(int)osm->nodes[n].furniture, p.x, p.z};
      model->furniture[model->furniture_count++] = item;
    }
    if (!osm->nodes[n].tree) continue;
    local_point_t* grown =
        realloc(model->trees, (model->tree_count + 1) * sizeof(*grown));
    if (!grown) break;
    model->trees = grown;
    model->trees[model->tree_count++] = projection_project(
        &model->projection, osm->nodes[n].lat, osm->nodes[n].lon);
  }
  settle_heights(model, &options);
  LOG_INFO("import",
           "%zu road segments, %zu junctions, %zu buildings, %zu areas; %zu "
           "ways ignored, "
           "%zu degenerate, %zu malformed buildings, %zu relations skipped, "
           "%zu ends snapped, %zu roads merged, %zu bends rounded, %zu tunnels "
           "skipped",
           model->road_count, model->junction_count, model->building_count,
           model->area_count, model->stats.ways_ignored,
           model->stats.roads_degenerate, model->stats.buildings_malformed,
           model->stats.relations_skipped, model->stats.ends_snapped,
           model->stats.roads_merged, model->stats.bends_rounded,
           model->stats.tunnels_skipped);
  return model->road_count > 0;
}
