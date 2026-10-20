#include "city_validate.h"

#include <math.h>
#include <string.h>

#include "log.h"

#define MAX_REPORTED 20

static void issue(city_validation_t* report, const char* chunk,
                  const char* message, unsigned long long id) {
  report->issues++;
  if (report->issues <= MAX_REPORTED)
    LOG_ERROR("validate", "%s: %s (source %llu)", chunk, message, id);
}

static bool finite_vec(vec3_t v) {
  return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static void validate_part(const city_chunk_t* chunk, const city_part_t* part,
                          city_validation_t* report) {
  const mesh_data_t* mesh = &part->mesh;
  unsigned long long id = part->source_id;
  if (mesh->vertex_count == 0 || mesh->index_count == 0)
    issue(report, chunk->file, "empty part", id);
  if (mesh->index_count % 3 != 0)
    issue(report, chunk->file, "index count is not a multiple of three", id);
  for (size_t v = 0; v < mesh->vertex_count; v++) {
    const vertex_t* vertex = &mesh->vertices[v];
    if (!finite_vec(vertex->position) || !finite_vec(vertex->normal) ||
        !isfinite(vertex->u) || !isfinite(vertex->v)) {
      issue(report, chunk->file, "non-finite vertex", id);
      break;
    }
    if (fabsf(vec3_length(vertex->normal) - 1.0f) > 0.01f) {
      issue(report, chunk->file, "normal is not unit length", id);
      break;
    }
    aabb_t grown = chunk->bounds;
    grown.min = vec3_sub(grown.min, vec3(0.01f, 0.01f, 0.01f));
    grown.max = vec3_add(grown.max, vec3(0.01f, 0.01f, 0.01f));
    if (!aabb_contains(grown, vertex->position)) {
      issue(report, chunk->file, "vertex outside the chunk bounds", id);
      break;
    }
  }
  for (size_t i = 0; i + 2 < mesh->index_count; i += 3) {
    uint32_t a = mesh->indices[i], b = mesh->indices[i + 1],
             c = mesh->indices[i + 2];
    if (a >= mesh->vertex_count || b >= mesh->vertex_count ||
        c >= mesh->vertex_count) {
      issue(report, chunk->file, "index out of range", id);
      break;
    }
    vec3_t pa = mesh->vertices[a].position, pb = mesh->vertices[b].position,
           pc = mesh->vertices[c].position;
    if (vec3_length(vec3_cross(vec3_sub(pb, pa), vec3_sub(pc, pa))) < 1e-6f) {
      issue(report, chunk->file, "degenerate triangle", id);
      break;
    }
  }
  report->triangles += mesh->index_count / 3;
  if (part->collider == PART_COLLIDER_MESH) report->colliders++;
  if (strcmp(part->material, "road") == 0) report->roads++;
  if (strcmp(part->material, "junction") == 0) report->junctions++;
  if (strncmp(part->material, "roof", 4) == 0) report->buildings++;
}

bool city_validate(const city_package_t* package, city_validation_t* report) {
  memset(report, 0, sizeof(*report));
  if (package->chunk_count == 0) issue(report, "manifest", "no chunks", 0);
  if (package->source[0] == '\0' || strcmp(package->source, "-") == 0)
    issue(report, "manifest", "no source recorded", 0);
  if (package->licence[0] == '\0')
    issue(report, "manifest", "no licence recorded", 0);
  if (package->chunk_size <= 0)
    issue(report, "manifest", "chunk size missing", 0);
  bool spawn_inside = false;
  for (size_t c = 0; c < package->chunk_count; c++) {
    const city_chunk_t* chunk = &package->chunks[c];
    report->chunks++;
    if (aabb_is_empty(chunk->bounds))
      issue(report, chunk->file, "empty bounds", 0);
    for (size_t p = 0; p < chunk->part_count; p++) {
      report->parts++;
      validate_part(chunk, &chunk->parts[p], report);
    }
    aabb_t flat = chunk->bounds;
    flat.min.y = -100;
    flat.max.y = 100;
    if (aabb_contains(flat, package->spawn_position)) spawn_inside = true;
  }
  if (!spawn_inside)
    issue(report, "manifest", "spawn is outside every chunk", 0);
  if (report->roads == 0) issue(report, "manifest", "no road surfaces", 0);
  if (report->colliders == 0) issue(report, "manifest", "no colliders", 0);
  if (report->issues > MAX_REPORTED)
    LOG_ERROR("validate", "... %zu issues in all", report->issues);
  return report->issues == 0;
}
