#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "city_import.h"
#include "city_lidar.h"
#include "city_model.h"
#include "log.h"
#include "osm_reader.h"

static void usage(void) {
  printf(
      "city-import --input <file.osm> --output <directory> "
      "[--no-service-roads] [--elevation <lattice.txt>]\n"
      "  [--dtm <terrain.asc>]... [--dsm <surface.asc>]...\n\n"
      "Reads an OpenStreetMap XML extract and writes the intermediate city\n"
      "model (roads.txt, buildings.txt, manifest.txt) in local metres.\n"
      "--dtm and --dsm take ESRI ASCII grids of airborne laser heights in\n"
      "UTM zone 33N, repeated for each tile: the terrain model gives the\n"
      "ground, the surface model minus it the heights of buildings with no\n"
      "mapped height.\n"
      "Exits 1 on failure, 2 on bad arguments.\n");
}

int main(int argc, char* argv[]) {
  const char* input = NULL;
  const char* output = NULL;
  const char* elevation = NULL;
  const char* dtm_paths[64];
  const char* dsm_paths[64];
  size_t dtm_count = 0, dsm_count = 0;
  city_import_options_t options = city_import_default_options();
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--input") == 0 && i + 1 < argc) {
      input = argv[++i];
    } else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
      output = argv[++i];
    } else if (strcmp(argv[i], "--elevation") == 0 && i + 1 < argc) {
      elevation = argv[++i];
    } else if (strcmp(argv[i], "--dtm") == 0 && i + 1 < argc &&
               dtm_count < 64) {
      dtm_paths[dtm_count++] = argv[++i];
    } else if (strcmp(argv[i], "--dsm") == 0 && i + 1 < argc &&
               dsm_count < 64) {
      dsm_paths[dsm_count++] = argv[++i];
    } else if (strcmp(argv[i], "--no-service-roads") == 0) {
      options.include_service_roads = false;
    } else if (strcmp(argv[i], "--help") == 0) {
      usage();
      return 0;
    } else {
      fprintf(stderr, "unknown argument %s\n", argv[i]);
      usage();
      return 2;
    }
  }
  if (!input || !output) {
    usage();
    return 2;
  }
  osm_data_t osm;
  if (!osm_load(&osm, input)) return 1;
  osm_clip_to_bounds(&osm, 150.0);
  LOG_INFO("import", "%s: %zu nodes, %zu ways, %zu relations", input,
           osm.node_count, osm.way_count, osm.relation_count);
  city_model_t model;
  bool ok = city_import(&model, &osm, options, input);
  osm_free(&osm);
  if (!ok) {
    LOG_ERROR("import", "nothing drivable found in %s", input);
    return 1;
  }
  if (elevation && !city_model_load_elevation(&model, elevation)) {
    city_model_free(&model);
    return 1;
  }
  if (dtm_count > 0) {
    lidar_window_t window = city_model_utm_window(&model, 80.0);
    lidar_set_t dtm, dsm;
    lidar_set_init(&dtm);
    lidar_set_init(&dsm);
    bool loaded = true;
    for (size_t i = 0; i < dtm_count && loaded; i++)
      loaded = lidar_set_load_asc(&dtm, dtm_paths[i], &window) || dtm.count > 0;
    for (size_t i = 0; i < dsm_count && loaded; i++)
      loaded = lidar_set_load_asc(&dsm, dsm_paths[i], &window) || dsm.count > 0;
    loaded = loaded && dtm.count > 0 && city_model_apply_dtm(&model, &dtm, 3.0);
    if (loaded && dsm.count > 0) city_model_apply_dsm(&model, &dsm, &dtm);
    lidar_set_free(&dtm);
    lidar_set_free(&dsm);
    if (!loaded) {
      LOG_ERROR("import", "the laser height grids do not cover this extract");
      city_model_free(&model);
      return 1;
    }
  }
  ok = city_model_write(&model, output);
  if (ok) LOG_INFO("import", "wrote %s", output);
  city_model_free(&model);
  return ok ? 0 : 1;
}
