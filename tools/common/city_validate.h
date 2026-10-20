/**
 * @file city_validate.h
 * @brief Structural checks on a cooked city package: well-formed meshes,
 * indices in range, finite values, non-degenerate triangles, chunk
 * bounds, a spawn, colliders and provenance.
 */
#ifndef TOOLS_COMMON_CITY_VALIDATE_H_
#define TOOLS_COMMON_CITY_VALIDATE_H_

#include <stdbool.h>
#include <stddef.h>

#include "city_package.h"

typedef struct {
  size_t chunks;
  size_t parts;
  size_t triangles;
  size_t colliders;
  size_t roads;
  size_t junctions;
  size_t buildings;
  size_t issues;
} city_validation_t;

bool city_validate(const city_package_t* package, city_validation_t* report);

#endif  // TOOLS_COMMON_CITY_VALIDATE_H_
