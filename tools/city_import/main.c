#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "city_import.h"
#include "city_model.h"
#include "log.h"
#include "osm_reader.h"

static void usage(void) {
  printf(
      "city-import --input <file.osm> --output <directory> "
      "[--no-service-roads]\n\n"
      "Reads an OpenStreetMap XML extract and writes the intermediate city\n"
      "model (roads.txt, buildings.txt, manifest.txt) in local metres.\n"
      "Exits 1 on failure, 2 on bad arguments.\n");
}

int main(int argc, char* argv[]) {
  const char* input = NULL;
  const char* output = NULL;
  city_import_options_t options = city_import_default_options();
  options.include_service_roads = false; /* driveways and car parks clutter */
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--input") == 0 && i + 1 < argc) {
      input = argv[++i];
    } else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
      output = argv[++i];
    } else if (strcmp(argv[i], "--service-roads") == 0) {
      options.include_service_roads = true;
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
  LOG_INFO("import", "%s: %zu nodes, %zu ways, %zu relations", input,
           osm.node_count, osm.way_count, osm.relation_count);
  city_model_t model;
  bool ok = city_import(&model, &osm, options, input);
  osm_free(&osm);
  if (!ok) {
    LOG_ERROR("import", "nothing drivable found in %s", input);
    return 1;
  }
  ok = city_model_write(&model, output);
  if (ok) LOG_INFO("import", "wrote %s", output);
  city_model_free(&model);
  return ok ? 0 : 1;
}
