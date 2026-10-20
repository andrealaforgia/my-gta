#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "city_generator.h"
#include "city_model.h"
#include "city_package.h"
#include "log.h"

static void usage(void) {
  printf(
      "city-generate --input <intermediate directory> --config <settings.cfg> "
      "--output <package directory> [--name <name>]\n\n"
      "Turns the intermediate city model into a cooked package: road ribbons,\n"
      "junction patches, pavements, buildings, ground tiles, chunked, with a\n"
      "manifest carrying provenance. Exits 1 on failure, 2 on bad "
      "arguments.\n");
}

static const char* basename_of(const char* path) {
  const char* slash = strrchr(path, '/');
  return slash && slash[1] ? slash + 1 : path;
}

int main(int argc, char* argv[]) {
  const char* input = NULL;
  const char* config = NULL;
  const char* output = NULL;
  const char* name = NULL;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--input") == 0 && i + 1 < argc) {
      input = argv[++i];
    } else if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
      config = argv[++i];
    } else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
      output = argv[++i];
    } else if (strcmp(argv[i], "--name") == 0 && i + 1 < argc) {
      name = argv[++i];
    } else if (strcmp(argv[i], "--help") == 0) {
      usage();
      return 0;
    } else {
      fprintf(stderr, "unknown argument %s\n", argv[i]);
      usage();
      return 2;
    }
  }
  if (!input || !config || !output) {
    usage();
    return 2;
  }
  city_generator_options_t options;
  if (!city_generator_options_load(&options, config)) return 1;
  city_model_t model;
  if (!city_model_read(&model, input)) {
    LOG_ERROR("generate", "cannot read the intermediate model in %s", input);
    return 1;
  }
  city_package_t package;
  city_generator_stats_t stats;
  bool ok = city_generate(&package, &model, &options,
                          name ? name : basename_of(output), &stats);
  city_model_free(&model);
  if (ok) ok = city_package_write(&package, output);
  if (ok)
    LOG_INFO("generate", "wrote %s (%zu chunks)", output, package.chunk_count);
  city_package_free(&package);
  return ok ? 0 : 1;
}
