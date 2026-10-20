#include "city_model.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "log.h"

static const char* class_names[ROAD_CLASS_COUNT] = {
    "motorway", "primary",      "secondary",  "tertiary", "residential",
    "service",  "unclassified", "pedestrian", "path",     "waterway"};
static const char* area_names[AREA_KIND_COUNT] = {"water", "park", "parking",
                                                  "square"};

const char* area_kind_name(area_kind_t kind) {
  return kind < AREA_KIND_COUNT ? area_names[kind] : "park";
}

bool area_kind_from_name(const char* name, area_kind_t* kind) {
  for (int i = 0; i < AREA_KIND_COUNT; i++) {
    if (strcmp(area_names[i], name) == 0) {
      *kind = (area_kind_t)i;
      return true;
    }
  }
  return false;
}

const char* road_class_name(road_class_t class) {
  return class < ROAD_CLASS_COUNT ? class_names[class] : "unclassified";
}

bool road_class_from_name(const char* name, road_class_t* class) {
  for (int i = 0; i < ROAD_CLASS_COUNT; i++) {
    if (strcmp(class_names[i], name) == 0) {
      *class = (road_class_t)i;
      return true;
    }
  }
  return false;
}

static void* grow(void* items, size_t count, size_t item_size) {
  if ((count & (count - 1)) != 0 && count != 0)
    return items; /* not a power of two: room left */
  size_t capacity = count == 0 ? 16 : count * 2;
  return realloc(items, capacity * item_size);
}

city_road_t* city_model_add_road(city_model_t* model) {
  void* grown = grow(model->roads, model->road_count, sizeof(city_road_t));
  if (!grown) return NULL;
  model->roads = grown;
  city_road_t* road = &model->roads[model->road_count++];
  memset(road, 0, sizeof(*road));
  return road;
}

city_building_t* city_model_add_building(city_model_t* model) {
  void* grown =
      grow(model->buildings, model->building_count, sizeof(city_building_t));
  if (!grown) return NULL;
  model->buildings = grown;
  city_building_t* building = &model->buildings[model->building_count++];
  memset(building, 0, sizeof(*building));
  return building;
}

city_junction_t* city_model_add_junction(city_model_t* model) {
  void* grown =
      grow(model->junctions, model->junction_count, sizeof(city_junction_t));
  if (!grown) return NULL;
  model->junctions = grown;
  city_junction_t* junction = &model->junctions[model->junction_count++];
  memset(junction, 0, sizeof(*junction));
  return junction;
}

city_area_t* city_model_add_area(city_model_t* model) {
  void* grown = grow(model->areas, model->area_count, sizeof(city_area_t));
  if (!grown) return NULL;
  model->areas = grown;
  city_area_t* area = &model->areas[model->area_count++];
  memset(area, 0, sizeof(*area));
  return area;
}

void city_model_free(city_model_t* model) {
  for (size_t i = 0; i < model->road_count; i++) {
    free(model->roads[i].points);
    free(model->roads[i].node_ids);
  }
  for (size_t i = 0; i < model->building_count; i++)
    free(model->buildings[i].points);
  for (size_t i = 0; i < model->area_count; i++) free(model->areas[i].points);
  free(model->roads);
  free(model->buildings);
  free(model->areas);
  free(model->junctions);
  memset(model, 0, sizeof(*model));
}

/* ---- writing ---- */

static FILE* open_in(const char* directory, const char* name,
                     const char* mode) {
  char path[1024];
  snprintf(path, sizeof(path), "%s/%s", directory, name);
  FILE* file = fopen(path, mode);
  if (!file) LOG_ERROR("city", "cannot open %s", path);
  return file;
}

static const char* or_dash(const char* text) { return text[0] ? text : "-"; }

static bool write_roads(const city_model_t* model, const char* directory) {
  FILE* file = open_in(directory, "roads.txt", "w");
  if (!file) return false;
  fprintf(file, "# city-intermediate %d roads\n", CITY_MODEL_VERSION);
  for (size_t i = 0; i < model->road_count; i++) {
    const city_road_t* r = &model->roads[i];
    fprintf(file, "road %lld %s %.3f %d %d %d %d %d %s %s %zu\n",
            (long long)r->source_id, road_class_name(r->class), r->width,
            r->width_inferred, r->oneway, r->layer, r->bridge, r->tunnel,
            or_dash(r->surface), or_dash(r->name), r->point_count);
    for (size_t p = 0; p < r->point_count; p++)
      fprintf(file, "p %.4f %.4f %lld\n", r->points[p].x, r->points[p].z,
              (long long)r->node_ids[p]);
  }
  for (size_t i = 0; i < model->junction_count; i++) {
    const city_junction_t* j = &model->junctions[i];
    fprintf(file, "junction %lld %.4f %.4f %d\n", (long long)j->node_id,
            j->position.x, j->position.z, j->degree);
  }
  fclose(file);
  return true;
}

static bool write_buildings(const city_model_t* model, const char* directory) {
  FILE* file = open_in(directory, "buildings.txt", "w");
  if (!file) return false;
  fprintf(file, "# city-intermediate %d buildings\n", CITY_MODEL_VERSION);
  for (size_t i = 0; i < model->building_count; i++) {
    const city_building_t* b = &model->buildings[i];
    fprintf(file, "building %lld %.3f %d %d %s %zu\n", (long long)b->source_id,
            b->height, (int)b->height_source, b->levels, or_dash(b->category),
            b->point_count);
    for (size_t p = 0; p < b->point_count; p++)
      fprintf(file, "p %.4f %.4f\n", b->points[p].x, b->points[p].z);
  }
  fclose(file);
  return true;
}

static bool write_areas(const city_model_t* model, const char* directory) {
  FILE* file = open_in(directory, "areas.txt", "w");
  if (!file) return false;
  fprintf(file, "# city-intermediate %d areas\n", CITY_MODEL_VERSION);
  for (size_t i = 0; i < model->area_count; i++) {
    const city_area_t* a = &model->areas[i];
    fprintf(file, "area %lld %s %zu\n", (long long)a->source_id,
            area_kind_name(a->kind), a->point_count);
    for (size_t p = 0; p < a->point_count; p++)
      fprintf(file, "p %.4f %.4f\n", a->points[p].x, a->points[p].z);
  }
  fclose(file);
  return true;
}

static bool write_manifest(const city_model_t* model, const char* directory) {
  FILE* file = open_in(directory, "manifest.txt", "w");
  if (!file) return false;
  const city_stats_t* s = &model->stats;
  fprintf(file, "# city-intermediate %d manifest\n", CITY_MODEL_VERSION);
  fprintf(file, "source %s\n", or_dash(model->source));
  fprintf(file, "import_date %s\n", or_dash(model->import_date));
  fprintf(file,
          "licence ODbL-1.0 OpenStreetMap contributors "
          "https://www.openstreetmap.org/copyright\n");
  fprintf(file, "bounds %.7f %.7f %.7f %.7f\n", model->bounds.min_lat,
          model->bounds.min_lon, model->bounds.max_lat, model->bounds.max_lon);
  fprintf(file, "origin %.7f %.7f\n", model->projection.origin_lat,
          model->projection.origin_lon);
  fprintf(file, "projection equirectangular x=east z=south metres\n");
  fprintf(file, "counts roads %zu junctions %zu buildings %zu areas %zu\n",
          model->road_count, model->junction_count, model->building_count,
          model->area_count);
  fprintf(file,
          "stats ways_total %zu roads_accepted %zu ways_ignored %zu "
          "roads_degenerate %zu "
          "buildings_accepted %zu buildings_malformed %zu relations_skipped "
          "%zu widths_inferred %zu "
          "heights_defaulted %zu\n",
          s->ways_total, s->roads_accepted, s->ways_ignored,
          s->roads_degenerate, s->buildings_accepted, s->buildings_malformed,
          s->relations_skipped, s->widths_inferred, s->heights_defaulted);
  fclose(file);
  return true;
}

bool city_model_write(const city_model_t* model, const char* directory) {
  char command[1100];
  snprintf(command, sizeof(command), "mkdir -p '%s'", directory);
  if (system(command) != 0) {
    LOG_ERROR("city", "cannot create %s", directory);
    return false;
  }
  return write_roads(model, directory) && write_buildings(model, directory) &&
         write_areas(model, directory) && write_manifest(model, directory);
}

/* ---- reading ---- */

static void copy_field(char* out, size_t size, const char* text) {
  if (strcmp(text, "-") == 0) text = "";
  snprintf(out, size, "%s", text);
}

static bool read_roads(city_model_t* model, const char* directory) {
  FILE* file = open_in(directory, "roads.txt", "r");
  if (!file) return false;
  char line[512];
  city_road_t* road = NULL;
  size_t expected = 0;
  while (fgets(line, sizeof(line), file)) {
    if (line[0] == '#') continue;
    if (strncmp(line, "road ", 5) == 0) {
      char class_name[CITY_TEXT_LENGTH], surface[CITY_TEXT_LENGTH],
          name[CITY_TEXT_LENGTH];
      long long id;
      int inferred, oneway, layer, bridge, tunnel;
      float width;
      size_t count;
      if (sscanf(line, "road %lld %63s %f %d %d %d %d %d %63s %63s %zu", &id,
                 class_name, &width, &inferred, &oneway, &layer, &bridge,
                 &tunnel, surface, name, &count) != 11) {
        fclose(file);
        return false;
      }
      road = city_model_add_road(model);
      if (!road) break;
      road->source_id = id;
      if (!road_class_from_name(class_name, &road->class))
        road->class = ROAD_CLASS_UNCLASSIFIED;
      road->width = width;
      road->width_inferred = inferred != 0;
      road->oneway = oneway != 0;
      road->layer = layer;
      road->bridge = bridge != 0;
      road->tunnel = tunnel != 0;
      copy_field(road->surface, sizeof(road->surface), surface);
      copy_field(road->name, sizeof(road->name), name);
      road->points = calloc(count, sizeof(local_point_t));
      road->node_ids = calloc(count, sizeof(int64_t));
      expected = count;
    } else if (strncmp(line, "p ", 2) == 0 && road &&
               road->point_count < expected) {
      double x, z;
      long long node;
      if (sscanf(line, "p %lf %lf %lld", &x, &z, &node) != 3) break;
      road->points[road->point_count].x = x;
      road->points[road->point_count].z = z;
      road->node_ids[road->point_count] = node;
      road->point_count++;
    } else if (strncmp(line, "junction ", 9) == 0) {
      long long node;
      double x, z;
      int degree;
      if (sscanf(line, "junction %lld %lf %lf %d", &node, &x, &z, &degree) != 4)
        break;
      city_junction_t* junction = city_model_add_junction(model);
      if (!junction) break;
      junction->node_id = node;
      junction->position.x = x;
      junction->position.z = z;
      junction->degree = degree;
    }
  }
  fclose(file);
  return true;
}

static bool read_buildings(city_model_t* model, const char* directory) {
  FILE* file = open_in(directory, "buildings.txt", "r");
  if (!file) return false;
  char line[512];
  city_building_t* building = NULL;
  size_t expected = 0;
  while (fgets(line, sizeof(line), file)) {
    if (line[0] == '#') continue;
    if (strncmp(line, "building ", 9) == 0) {
      long long id;
      float height;
      int source, levels;
      char category[CITY_TEXT_LENGTH];
      size_t count;
      if (sscanf(line, "building %lld %f %d %d %63s %zu", &id, &height, &source,
                 &levels, category, &count) != 6) {
        fclose(file);
        return false;
      }
      building = city_model_add_building(model);
      if (!building) break;
      building->source_id = id;
      building->height = height;
      building->height_source = (height_source_t)source;
      building->levels = levels;
      copy_field(building->category, sizeof(building->category), category);
      building->points = calloc(count, sizeof(local_point_t));
      expected = count;
    } else if (strncmp(line, "p ", 2) == 0 && building &&
               building->point_count < expected) {
      double x, z;
      if (sscanf(line, "p %lf %lf", &x, &z) != 2) break;
      building->points[building->point_count].x = x;
      building->points[building->point_count].z = z;
      building->point_count++;
    }
  }
  fclose(file);
  return true;
}

static bool read_areas(city_model_t* model, const char* directory) {
  FILE* file = open_in(directory, "areas.txt", "r");
  if (!file) return false;
  char line[512];
  city_area_t* area = NULL;
  size_t expected = 0;
  while (fgets(line, sizeof(line), file)) {
    if (line[0] == '#') continue;
    if (strncmp(line, "area ", 5) == 0) {
      long long id;
      char kind[CITY_TEXT_LENGTH];
      size_t count;
      if (sscanf(line, "area %lld %63s %zu", &id, kind, &count) != 3) {
        fclose(file);
        return false;
      }
      area = city_model_add_area(model);
      if (!area) break;
      area->source_id = id;
      if (!area_kind_from_name(kind, &area->kind)) area->kind = AREA_PARK;
      area->points = calloc(count, sizeof(local_point_t));
      expected = count;
    } else if (strncmp(line, "p ", 2) == 0 && area &&
               area->point_count < expected) {
      double x, z;
      if (sscanf(line, "p %lf %lf", &x, &z) != 2) break;
      area->points[area->point_count].x = x;
      area->points[area->point_count].z = z;
      area->point_count++;
    }
  }
  fclose(file);
  return true;
}

static bool read_manifest(city_model_t* model, const char* directory) {
  FILE* file = open_in(directory, "manifest.txt", "r");
  if (!file) return false;
  char line[512];
  while (fgets(line, sizeof(line), file)) {
    line[strcspn(line, "\n")] = '\0';
    if (strncmp(line, "source ", 7) == 0) {
      copy_field(model->source, sizeof(model->source), line + 7);
    } else if (strncmp(line, "import_date ", 12) == 0) {
      copy_field(model->import_date, sizeof(model->import_date), line + 12);
    } else if (strncmp(line, "bounds ", 7) == 0) {
      sscanf(line, "bounds %lf %lf %lf %lf", &model->bounds.min_lat,
             &model->bounds.min_lon, &model->bounds.max_lat,
             &model->bounds.max_lon);
    } else if (strncmp(line, "origin ", 7) == 0) {
      double lat, lon;
      if (sscanf(line, "origin %lf %lf", &lat, &lon) == 2) {
        model->projection = projection_create(lat, lon);
      }
    }
  }
  fclose(file);
  return true;
}

bool city_model_read(city_model_t* model, const char* directory) {
  memset(model, 0, sizeof(*model));
  return read_manifest(model, directory) && read_roads(model, directory) &&
         read_buildings(model, directory) && read_areas(model, directory);
}
