#include "city_package.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "log.h"

city_package_t city_package_create(void) {
  city_package_t package;
  memset(&package, 0, sizeof(package));
  package.generator_version = CITY_PACKAGE_VERSION;
  snprintf(package.licence, sizeof(package.licence),
           "ODbL-1.0 (c) OpenStreetMap contributors");
  return package;
}

void city_package_free(city_package_t* package) {
  for (size_t c = 0; c < package->chunk_count; c++) {
    for (size_t p = 0; p < package->chunks[c].part_count; p++)
      mesh_data_free(&package->chunks[c].parts[p].mesh);
    free(package->chunks[c].parts);
  }
  free(package->chunks);
  for (size_t r = 0; r < package->route_count; r++)
    free(package->routes[r].points);
  free(package->routes);
  free(package->parked);
  free(package->signals);
  free(package->crossings);
  free(package->bus_stops);
  memset(package, 0, sizeof(*package));
}

bool city_package_add_route(city_package_t* package,
                            const city_route_t* route) {
  city_route_t* grown = realloc(
      package->routes, (package->route_count + 1) * sizeof(city_route_t));
  if (!grown) return false;
  package->routes = grown;
  package->routes[package->route_count++] = *route;
  return true;
}

bool city_package_add_signal(city_package_t* package, city_signal_t signal) {
  city_signal_t* grown = realloc(
      package->signals, (package->signal_count + 1) * sizeof(city_signal_t));
  if (!grown) return false;
  package->signals = grown;
  package->signals[package->signal_count++] = signal;
  return true;
}

bool city_package_add_crossing(city_package_t* package, float x, float z) {
  float* grown = realloc(package->crossings,
                         (package->crossing_count + 1) * 2 * sizeof(float));
  if (!grown) return false;
  package->crossings = grown;
  package->crossings[2 * package->crossing_count] = x;
  package->crossings[2 * package->crossing_count + 1] = z;
  package->crossing_count++;
  return true;
}

bool city_package_add_bus_stop(city_package_t* package, float x, float z) {
  float* grown = realloc(package->bus_stops,
                         (package->bus_stop_count + 1) * 2 * sizeof(float));
  if (!grown) return false;
  package->bus_stops = grown;
  package->bus_stops[2 * package->bus_stop_count] = x;
  package->bus_stops[2 * package->bus_stop_count + 1] = z;
  package->bus_stop_count++;
  return true;
}

bool city_package_add_parking(city_package_t* package, city_parking_t spot) {
  city_parking_t* grown = realloc(
      package->parked, (package->parked_count + 1) * sizeof(city_parking_t));
  if (!grown) return false;
  package->parked = grown;
  package->parked[package->parked_count++] = spot;
  return true;
}

city_chunk_t* city_package_chunk(city_package_t* package, int column, int row) {
  for (size_t c = 0; c < package->chunk_count; c++)
    if (package->chunks[c].column == column && package->chunks[c].row == row)
      return &package->chunks[c];
  if (package->chunk_count == package->chunk_capacity) {
    size_t capacity =
        package->chunk_capacity ? package->chunk_capacity * 2 : 16;
    city_chunk_t* grown =
        realloc(package->chunks, capacity * sizeof(city_chunk_t));
    if (!grown) return NULL;
    package->chunks = grown;
    package->chunk_capacity = capacity;
  }
  city_chunk_t* chunk = &package->chunks[package->chunk_count++];
  memset(chunk, 0, sizeof(*chunk));
  chunk->column = column;
  chunk->row = row;
  chunk->bounds = aabb_empty();
  snprintf(chunk->file, sizeof(chunk->file), "chunk_%d_%d.txt", column, row);
  return chunk;
}

city_part_t* city_chunk_add_part(city_chunk_t* chunk, const char* material,
                                 part_collider_t collider, int layer,
                                 float friction, uint64_t source_id) {
  if (chunk->part_count == chunk->part_capacity) {
    size_t capacity = chunk->part_capacity ? chunk->part_capacity * 2 : 4;
    city_part_t* grown = realloc(chunk->parts, capacity * sizeof(city_part_t));
    if (!grown) return NULL;
    chunk->parts = grown;
    chunk->part_capacity = capacity;
  }
  city_part_t* part = &chunk->parts[chunk->part_count++];
  memset(part, 0, sizeof(*part));
  snprintf(part->material, sizeof(part->material), "%s", material);
  part->collider = collider;
  part->layer = layer;
  part->friction = friction;
  part->source_id = source_id;
  part->mesh = mesh_data_create();
  return part;
}

size_t city_package_triangle_count(const city_package_t* package) {
  size_t total = 0;
  for (size_t c = 0; c < package->chunk_count; c++)
    for (size_t p = 0; p < package->chunks[c].part_count; p++)
      total += mesh_data_triangle_count(&package->chunks[c].parts[p].mesh);
  return total;
}

/* ---- writing ---- */

static FILE* open_in(const char* directory, const char* name,
                     const char* mode) {
  char path[1024];
  snprintf(path, sizeof(path), "%s/%s", directory, name);
  FILE* file = fopen(path, mode);
  if (!file) LOG_ERROR("package", "cannot open %s", path);
  return file;
}

static bool write_chunk(const city_chunk_t* chunk, const char* directory) {
  FILE* file = open_in(directory, chunk->file, "w");
  if (!file) return false;
  fprintf(file, "# city-chunk %d\n", CITY_PACKAGE_VERSION);
  fprintf(file, "bounds %.3f %.3f %.3f %.3f %.3f %.3f\n", chunk->bounds.min.x,
          chunk->bounds.min.y, chunk->bounds.min.z, chunk->bounds.max.x,
          chunk->bounds.max.y, chunk->bounds.max.z);
  for (size_t p = 0; p < chunk->part_count; p++) {
    const city_part_t* part = &chunk->parts[p];
    fprintf(file, "part %s %s %d %.3f %llu %zu %zu\n", part->material,
            part->collider == PART_COLLIDER_MESH ? "mesh" : "none", part->layer,
            part->friction, (unsigned long long)part->source_id,
            part->mesh.vertex_count, part->mesh.index_count);
    for (size_t v = 0; v < part->mesh.vertex_count; v++) {
      const vertex_t* vertex = &part->mesh.vertices[v];
      fprintf(file, "v %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
              vertex->position.x, vertex->position.y, vertex->position.z,
              vertex->normal.x, vertex->normal.y, vertex->normal.z, vertex->u,
              vertex->v);
    }
    for (size_t i = 0; i + 2 < part->mesh.index_count; i += 3)
      fprintf(file, "i %u %u %u\n", part->mesh.indices[i],
              part->mesh.indices[i + 1], part->mesh.indices[i + 2]);
  }
  fclose(file);
  return true;
}

bool city_package_write(const city_package_t* package, const char* directory) {
  char command[1100];
  snprintf(command, sizeof(command), "mkdir -p '%s'", directory);
  if (system(command) != 0) return false;
  FILE* file = open_in(directory, "manifest.txt", "w");
  if (!file) return false;
  fprintf(file, "# city-package %d\n", CITY_PACKAGE_VERSION);
  fprintf(file, "name %s\n", package->name);
  fprintf(file, "source %s\n", package->source[0] ? package->source : "-");
  fprintf(file, "source_date %s\n",
          package->source_date[0] ? package->source_date : "-");
  fprintf(file, "licence %s\n", package->licence);
  fprintf(file, "generated %s\n",
          package->generated[0] ? package->generated : "-");
  fprintf(file, "generator_version %d\n", package->generator_version);
  fprintf(file, "config_hash %016llx\n",
          (unsigned long long)package->config_hash);
  fprintf(file, "bounds_lat_lon %.7f %.7f %.7f %.7f\n",
          package->bounds_lat_lon[0], package->bounds_lat_lon[1],
          package->bounds_lat_lon[2], package->bounds_lat_lon[3]);
  fprintf(file, "origin %.7f %.7f\n", package->origin_lat, package->origin_lon);
  fprintf(file, "projection equirectangular x=east y=up z=south metres\n");
  fprintf(file, "chunk_size %.1f\n", package->chunk_size);
  fprintf(file, "spawn %.3f %.3f %.3f %.4f\n", package->spawn_position.x,
          package->spawn_position.y, package->spawn_position.z,
          package->spawn_yaw);
  for (size_t i = 0; i < package->label_count; i++)
    fprintf(file, "label %.2f %.2f %s\n", package->labels[i].x,
            package->labels[i].z, package->labels[i].text);
  for (size_t r = 0; r < package->route_count; r++) {
    const city_route_t* route = &package->routes[r];
    fprintf(file, "route %d %d %.2f %.2f %lld %lld %zu\n", route->drivable,
            route->oneway, route->width, route->pavement,
            (long long)route->start_node, (long long)route->end_node,
            route->point_count);
    if (route->name[0]) fprintf(file, "rn %s\n", route->name);
    for (size_t k = 0; k < route->point_count; k++)
      fprintf(file, "rp %.3f %.3f\n", route->points[2 * k],
              route->points[2 * k + 1]);
  }
  for (size_t i = 0; i < package->signal_count; i++)
    fprintf(file, "signal %.3f %.3f %.3f %.3f %d\n", package->signals[i].x,
            package->signals[i].z, package->signals[i].heading_x,
            package->signals[i].heading_z, package->signals[i].group);
  for (size_t i = 0; i < package->crossing_count; i++)
    fprintf(file, "crossing %.3f %.3f\n", package->crossings[2 * i],
            package->crossings[2 * i + 1]);
  for (size_t i = 0; i < package->bus_stop_count; i++)
    fprintf(file, "busstop %.3f %.3f\n", package->bus_stops[2 * i],
            package->bus_stops[2 * i + 1]);
  for (size_t i = 0; i < package->parked_count; i++)
    fprintf(file, "park %.3f %.3f %.4f %d\n", package->parked[i].x,
            package->parked[i].z, package->parked[i].yaw,
            package->parked[i].variant);
  for (size_t c = 0; c < package->chunk_count; c++)
    fprintf(file, "chunk %d %d %s\n", package->chunks[c].column,
            package->chunks[c].row, package->chunks[c].file);
  fclose(file);
  for (size_t c = 0; c < package->chunk_count; c++)
    if (!write_chunk(&package->chunks[c], directory)) return false;
  return true;
}

/* ---- reading ---- */

static bool read_chunk(city_chunk_t* chunk, const char* directory) {
  FILE* file = open_in(directory, chunk->file, "r");
  if (!file) return false;
  char line[512];
  city_part_t* part = NULL;
  bool ok = true;
  while (ok && fgets(line, sizeof(line), file)) {
    if (line[0] == '#') continue;
    if (strncmp(line, "bounds ", 7) == 0) {
      aabb_t* b = &chunk->bounds;
      ok = sscanf(line, "bounds %f %f %f %f %f %f", &b->min.x, &b->min.y,
                  &b->min.z, &b->max.x, &b->max.y, &b->max.z) == 6;
    } else if (strncmp(line, "part ", 5) == 0) {
      char material[32], collider[16];
      int layer;
      float friction;
      unsigned long long source;
      size_t vertices, indices;
      ok = sscanf(line, "part %31s %15s %d %f %llu %zu %zu", material, collider,
                  &layer, &friction, &source, &vertices, &indices) == 7;
      if (!ok) break;
      part = city_chunk_add_part(chunk, material,
                                 strcmp(collider, "mesh") == 0
                                     ? PART_COLLIDER_MESH
                                     : PART_COLLIDER_NONE,
                                 layer, friction, source);
      ok = part != NULL;
    } else if (strncmp(line, "v ", 2) == 0 && part) {
      vertex_t vertex;
      ok = sscanf(line, "v %f %f %f %f %f %f %f %f", &vertex.position.x,
                  &vertex.position.y, &vertex.position.z, &vertex.normal.x,
                  &vertex.normal.y, &vertex.normal.z, &vertex.u,
                  &vertex.v) == 8 &&
           mesh_data_add_vertex(&part->mesh, vertex);
    } else if (strncmp(line, "i ", 2) == 0 && part) {
      unsigned a, b, c;
      ok = sscanf(line, "i %u %u %u", &a, &b, &c) == 3 &&
           mesh_data_add_triangle(&part->mesh, a, b, c);
    }
  }
  fclose(file);
  if (!ok) LOG_ERROR("package", "%s/%s is malformed", directory, chunk->file);
  return ok;
}

bool city_package_read(city_package_t* package, const char* directory) {
  *package = city_package_create();
  FILE* file = open_in(directory, "manifest.txt", "r");
  if (!file) return false;
  char line[512];
  bool ok = true;
  int version = 0;
  while (ok && fgets(line, sizeof(line), file)) {
    line[strcspn(line, "\n")] = '\0';
    if (sscanf(line, "# city-package %d", &version) == 1) continue;
    if (strncmp(line, "name ", 5) == 0) {
      snprintf(package->name, sizeof(package->name), "%s", line + 5);
    } else if (strncmp(line, "source ", 7) == 0) {
      snprintf(package->source, sizeof(package->source), "%s", line + 7);
    } else if (strncmp(line, "source_date ", 12) == 0) {
      snprintf(package->source_date, sizeof(package->source_date), "%s",
               line + 12);
    } else if (strncmp(line, "licence ", 8) == 0) {
      snprintf(package->licence, sizeof(package->licence), "%s", line + 8);
    } else if (strncmp(line, "generated ", 10) == 0) {
      snprintf(package->generated, sizeof(package->generated), "%s", line + 10);
    } else if (strncmp(line, "generator_version ", 18) == 0) {
      package->generator_version = atoi(line + 18);
    } else if (strncmp(line, "config_hash ", 12) == 0) {
      unsigned long long hash;
      if (sscanf(line, "config_hash %llx", &hash) == 1) {
        package->config_hash = hash;
      }
    } else if (strncmp(line, "bounds_lat_lon ", 15) == 0) {
      ok =
          sscanf(line, "bounds_lat_lon %lf %lf %lf %lf",
                 &package->bounds_lat_lon[0], &package->bounds_lat_lon[1],
                 &package->bounds_lat_lon[2], &package->bounds_lat_lon[3]) == 4;
    } else if (strncmp(line, "origin ", 7) == 0) {
      ok = sscanf(line, "origin %lf %lf", &package->origin_lat,
                  &package->origin_lon) == 2;
    } else if (strncmp(line, "chunk_size ", 11) == 0) {
      ok = sscanf(line, "chunk_size %f", &package->chunk_size) == 1;
    } else if (strncmp(line, "spawn ", 6) == 0) {
      ok = sscanf(line, "spawn %f %f %f %f", &package->spawn_position.x,
                  &package->spawn_position.y, &package->spawn_position.z,
                  &package->spawn_yaw) == 4;
    } else if (strncmp(line, "route ", 6) == 0) {
      city_route_t route;
      memset(&route, 0, sizeof(route));
      long long start, end;
      unsigned long count;
      ok = sscanf(line, "route %d %d %f %f %lld %lld %lu", &route.drivable,
                  &route.oneway, &route.width, &route.pavement, &start, &end,
                  &count) == 7 &&
           count >= 2 && count < 100000;
      if (ok) {
        route.start_node = start, route.end_node = end;
        route.point_count = count;
        route.points = malloc(count * 2 * sizeof(float));
        ok = route.points != NULL;
        for (unsigned long k = 0; ok && k < count; k++) {
          char point_line[128];
          ok = fgets(point_line, sizeof(point_line), file) != NULL;
          /* An optional name line comes first. */
          if (ok && strncmp(point_line, "rn ", 3) == 0) {
            sscanf(point_line, "rn %47s", route.name);
            ok = fgets(point_line, sizeof(point_line), file) != NULL;
          }
          ok = ok && sscanf(point_line, "rp %f %f", &route.points[2 * k],
                            &route.points[2 * k + 1]) == 2;
        }
        if (ok) ok = city_package_add_route(package, &route);
        if (!ok) free(route.points);
      }
    } else if (strncmp(line, "signal ", 7) == 0) {
      /* Older packages give only x and z: a signal for any heading. */
      city_signal_t signal = {0, 0, 0, 0, 0};
      ok = sscanf(line, "signal %f %f %f %f %d", &signal.x, &signal.z,
                  &signal.heading_x, &signal.heading_z, &signal.group) >= 2 &&
           city_package_add_signal(package, signal);
    } else if (strncmp(line, "crossing ", 9) == 0) {
      float x, z;
      ok = sscanf(line, "crossing %f %f", &x, &z) == 2 &&
           city_package_add_crossing(package, x, z);
    } else if (strncmp(line, "busstop ", 8) == 0) {
      float x, z;
      ok = sscanf(line, "busstop %f %f", &x, &z) == 2 &&
           city_package_add_bus_stop(package, x, z);
    } else if (strncmp(line, "park ", 5) == 0) {
      city_parking_t spot;
      ok = sscanf(line, "park %f %f %f %d", &spot.x, &spot.z, &spot.yaw,
                  &spot.variant) == 4 &&
           city_package_add_parking(package, spot);
    } else if (strncmp(line, "label ", 6) == 0) {
      if (package->label_count < CITY_PACKAGE_MAX_LABELS) {
        city_label_t* label = &package->labels[package->label_count];
        char text[32] = "";
        ok = sscanf(line, "label %f %f %31[^\n]", &label->x, &label->z, text) ==
             3;
        snprintf(label->text, sizeof(label->text), "%s", text);
        package->label_count++;
      }
    } else if (strncmp(line, "chunk ", 6) == 0) {
      int column, row;
      char name[CITY_PACKAGE_TEXT];
      ok = sscanf(line, "chunk %d %d %127s", &column, &row, name) == 3;
      if (ok) {
        city_chunk_t* chunk = city_package_chunk(package, column, row);
        ok = chunk != NULL;
        if (chunk) snprintf(chunk->file, sizeof(chunk->file), "%s", name);
      }
    }
  }
  fclose(file);
  if (version != CITY_PACKAGE_VERSION) {
    LOG_ERROR("package", "%s: expected city-package %d, found %d", directory,
              CITY_PACKAGE_VERSION, version);
    return false;
  }
  if (!ok) {
    LOG_ERROR("package", "%s/manifest.txt is malformed", directory);
    return false;
  }
  for (size_t c = 0; c < package->chunk_count; c++)
    if (!read_chunk(&package->chunks[c], directory)) return false;
  return true;
}
