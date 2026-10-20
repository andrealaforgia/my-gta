#include <stdio.h>
#include <string.h>

#include "city_package.h"
#include "city_validate.h"
#include "log.h"

int main(int argc, char* argv[]) {
  if (argc != 2 || strcmp(argv[1], "--help") == 0) {
    printf(
        "city-validate <package directory>\n\nChecks a cooked city package and "
        "prints a summary. Exits 1 if any issue is found.\n");
    return argc == 2 ? 0 : 2;
  }
  city_package_t package;
  if (!city_package_read(&package, argv[1])) return 1;
  city_validation_t report;
  bool ok = city_validate(&package, &report);
  printf(
      "%s: %zu chunks, %zu parts, %zu triangles, %zu colliders, %zu roads, %zu "
      "junctions, %zu buildings, "
      "%zu issues\n",
      argv[1], report.chunks, report.parts, report.triangles, report.colliders,
      report.roads, report.junctions, report.buildings, report.issues);
  printf("source %s (%s), licence %s, generated %s, settings %016llx\n",
         package.source, package.source_date, package.licence,
         package.generated, (unsigned long long)package.config_hash);
  city_package_free(&package);
  return ok ? 0 : 1;
}
