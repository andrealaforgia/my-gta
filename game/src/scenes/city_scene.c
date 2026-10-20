#include "city_scene.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "city_package.h"
#include "log.h"

typedef struct {
  const char* name;
  texture_handle_t texture;
  vec3_t tint;
  float roughness;
} material_binding_t;

static material_t material_for(const char* name,
                               const texture_set_t* textures) {
  material_binding_t bindings[] = {
      {"road", textures->road, vec3(0.95f, 0.95f, 0.95f), 0.95f},
      {"junction", textures->asphalt, vec3(0.95f, 0.95f, 0.95f), 0.95f},
      {"pavement", textures->concrete, vec3(0.95f, 0.95f, 0.95f), 0.9f},
      {"kerb", textures->concrete, vec3(0.75f, 0.75f, 0.75f), 0.9f},
      {"grass", textures->grass, vec3(0.9f, 0.9f, 0.9f), 1.0f},
      {"ground", textures->earth, vec3(0.95f, 0.95f, 0.95f), 1.0f},
      {"roof", textures->roof, vec3(0.9f, 0.9f, 0.9f), 0.9f},
      {"facade_fresco", textures->facade_fresco, vec3(1, 1, 1), 0.8f},
      {"facade_brick", textures->facade_brick, vec3(1, 1, 1), 0.85f},
      {"shopfront", textures->shopfront, vec3(1, 1, 1), 0.7f},
      {"furniture", {0}, vec3(0.3f, 0.32f, 0.34f), 0.6f},
      {"signal_red", {0}, vec3(1.0f, 0.12f, 0.1f), 0.4f},
      {"signal_amber", {0}, vec3(1.0f, 0.62f, 0.08f), 0.4f},
      {"signal_green", {0}, vec3(0.15f, 1.0f, 0.3f), 0.4f},
      {"chimney", textures->roof, vec3(0.7f, 0.5f, 0.45f), 0.9f},
      {"manhole", {0}, vec3(0.16f, 0.16f, 0.17f), 0.5f},
      {"sign_pole", {0}, vec3(0.45f, 0.46f, 0.48f), 0.5f},
      {"sign_red", {0}, vec3(0.78f, 0.08f, 0.07f), 0.5f},
      {"roof_slate", textures->roof_slate, vec3(0.95f, 0.95f, 0.95f), 0.8f},
      {"roof_flat", textures->roof_flat, vec3(0.95f, 0.95f, 0.95f), 0.9f},
      {"lamp", {0}, vec3(0.22f, 0.23f, 0.25f), 0.45f},
      {"lamp_light", {0}, vec3(1.0f, 0.88f, 0.6f), 0.4f},
      {"paving", textures->concrete, vec3(0.92f, 0.86f, 0.76f), 0.85f},
      {"parking", textures->asphalt, vec3(0.85f, 0.85f, 0.85f), 0.95f},
      {"park", textures->grass, vec3(0.72f, 0.86f, 0.62f), 1.0f},
      {"water", {0}, vec3(0.2f, 0.36f, 0.5f), 0.12f},
      {"trunk", {0}, vec3(0.32f, 0.24f, 0.16f), 0.9f},
      {"leaves", {0}, vec3(0.22f, 0.42f, 0.16f), 0.95f},
      {"marking", {0}, vec3(0.9f, 0.9f, 0.86f), 0.7f},
  };
  int style = -1;
  if (sscanf(name, "facade%d", &style) == 1 && style >= 0 &&
      style < CITY_FACADE_STYLES) {
    material_t m = material_rough(vec3(1, 1, 1), 0.8f);
    m.texture = textures->facades[style];
    return m;
  }
  for (size_t i = 0; i < sizeof(bindings) / sizeof(bindings[0]); i++) {
    if (strcmp(bindings[i].name, name) == 0) {
      material_t m = material_rough(bindings[i].tint, bindings[i].roughness);
      m.texture = bindings[i].texture;
      if (strcmp(name, "lamp") == 0) m.metallic = 0.7f;
      if (strcmp(name, "water") == 0) m.metallic = 0.4f;
      return m;
    }
  }
  return material_rough(vec3(1, 0, 1), 0.8f); /* unknown materials shout */
}

/* One upload per chunk and material: the merged mesh keeps draw calls low. */
static bool upload_merged(scene_t* scene, scene_context_t c,
                          const city_chunk_t* chunk, const char* material,
                          int* draws) {
  mesh_data_t merged = mesh_data_create();
  for (size_t p = 0; p < chunk->part_count; p++) {
    const city_part_t* part = &chunk->parts[p];
    if (strcmp(part->material, material) != 0) continue;
    if (!mesh_data_append(&merged, &part->mesh)) {
      mesh_data_free(&merged);
      return false;
    }
  }
  if (merged.vertex_count == 0) {
    mesh_data_free(&merged);
    return true;
  }
  mesh_handle_t handle = renderer_upload_mesh(c.renderer, &merged);
  scene_track_mesh(scene, handle);
  char label[ENTITY_LABEL_LENGTH];
  snprintf(label, sizeof(label), "%s %d,%d", material, chunk->column,
           chunk->row);
  world_add(c.world, transform_identity(), handle,
            material_for(material, c.textures), mesh_data_bounds(&merged),
            label);
  (*draws)++;
  mesh_data_free(&merged);
  return true;
}

static bool add_colliders(scene_context_t c, const city_chunk_t* chunk,
                          int* colliders) {
  for (size_t p = 0; p < chunk->part_count; p++) {
    const city_part_t* part = &chunk->parts[p];
    if (part->collider != PART_COLLIDER_MESH) continue;
    vec3_t* positions = malloc(part->mesh.vertex_count * sizeof(vec3_t));
    if (!positions) return false;
    for (size_t v = 0; v < part->mesh.vertex_count; v++)
      positions[v] = part->mesh.vertices[v].position;
    collider_handle_t handle = physics_add_static_mesh(
        c.physics, positions, part->mesh.vertex_count, part->mesh.indices,
        part->mesh.index_count, (collision_layer_t)part->layer, part->friction,
        part->source_id);
    free(positions);
    if (handle.id == 0) return false;
    (*colliders)++;
  }
  return true;
}

bool scene_build_city(scene_t* scene, scene_context_t c,
                      const char* package_directory) {
  memset(scene, 0, sizeof(*scene));
  if (!package_directory) {
    LOG_ERROR("scene", "no city package given");
    return false;
  }
  city_package_t package;
  if (!city_package_read(&package, package_directory)) return false;
  snprintf(scene->name, sizeof(scene->name), "city %s", package.name);
  static const char* materials[] = {
      "grass",    "road",       "junction",  "pavement",      "kerb",
      "roof",     "roof_slate", "roof_flat", "facade_fresco", "facade_brick",
      "facade0",  "facade1",    "facade2",   "facade3",       "facade4",
      "facade5",  "facade6",    "facade7",   "facade8",       "facade9",
      "facade10", "facade11",   "lamp",      "lamp_light",    "paving",
      "parking",  "park",       "water",     "trunk",         "leaves",
      "marking",  "ground"};
  int draws = 0, colliders = 0;
  bool ok = true;
  for (size_t ch = 0; ok && ch < package.chunk_count; ch++) {
    const city_chunk_t* chunk = &package.chunks[ch];
    for (size_t m = 0; ok && m < sizeof(materials) / sizeof(materials[0]); m++)
      ok = upload_merged(scene, c, chunk, materials[m], &draws);
    if (ok) ok = add_colliders(c, chunk, &colliders);
  }
  if (!city_map_build(&scene->map, &package, 1024))
    LOG_WARN("scene", "no map for %s", package.name);
  memcpy(scene->labels, package.labels, sizeof(scene->labels));
  scene->label_count = package.label_count;
  scene->spawn_position = package.spawn_position;
  scene->spawn_rotation = quat_from_yaw(package.spawn_yaw);
  render_environment_t environment = {vec3(0.5f, 0.8f, 0.3f),
                                      vec3(1.15f, 1.08f, 0.95f),
                                      0.3f,
                                      vec3(0.62f, 0.74f, 0.9f),
                                      150.0f,
                                      900.0f,
                                      110.0f,
                                      vec3(0.3f, 0.5f, 0.85f),
                                      0.0f,
                                      1.0f,
                                      vec3(0, 0, 0),
                                      vec3(0, 0, -1),
                                      0.0f,
                                      0.4f,
                                      0.0f};
  scene->environment = environment;
  LOG_INFO("scene",
           "%s: %zu chunks, %zu triangles, %d meshes, %d colliders; source %s "
           "(%s), %s",
           package.name, package.chunk_count,
           city_package_triangle_count(&package), draws, colliders,
           package.source, package.source_date, package.licence);
  /* The runtime keeps the roads and the parking places. */
  scene->routes = package.routes;
  scene->route_count = package.route_count;
  scene->signals = package.signals;
  scene->signal_count = package.signal_count;
  package.signals = NULL, package.signal_count = 0;
  scene->parked = package.parked;
  scene->parked_count = package.parked_count;
  package.routes = NULL, package.route_count = 0;
  package.parked = NULL, package.parked_count = 0;
  city_package_free(&package);
  if (!ok) LOG_ERROR("scene", "could not build the city scene");
  return ok;
}
