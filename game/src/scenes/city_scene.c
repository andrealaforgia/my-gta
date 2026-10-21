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
      {"signal_r2", {0}, vec3(1.0f, 0.12f, 0.1f), 0.4f},
      {"signal_a2", {0}, vec3(1.0f, 0.62f, 0.08f), 0.4f},
      {"signal_g2", {0}, vec3(0.15f, 1.0f, 0.3f), 0.4f},
      {"chimney", textures->roof, vec3(0.7f, 0.5f, 0.45f), 0.9f},
      {"rail", {0}, vec3(0.28f, 0.29f, 0.3f), 0.6f},
      {"wall", {0}, vec3(0.68f, 0.63f, 0.55f), 0.9f},
      {"fence", {0}, vec3(0.42f, 0.32f, 0.22f), 0.85f},
      {"hedge", textures->grass, vec3(0.3f, 0.5f, 0.2f), 1.0f},
      {"manhole", {0}, vec3(0.16f, 0.16f, 0.17f), 0.5f},
      {"sign_pole", {0}, vec3(0.45f, 0.46f, 0.48f), 0.5f},
      {"sign_red", {0}, vec3(0.78f, 0.08f, 0.07f), 0.5f},
      {"roof_slate", textures->roof_slate, vec3(0.95f, 0.95f, 0.95f), 0.8f},
      {"roof_flat", textures->roof_flat, vec3(0.95f, 0.95f, 0.95f), 0.9f},
      {"lamp", {0}, vec3(0.22f, 0.23f, 0.25f), 0.45f},
      {"lamp_light", {0}, vec3(1.0f, 0.88f, 0.6f), 0.4f},
      {"paving", textures->paving, vec3(1.0f, 0.95f, 0.88f), 0.85f},
      {"door", textures->door, vec3(1, 1, 1), 0.7f},
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
    m.wall_ao = 0.8f;
    return m;
  }
  for (size_t i = 0; i < sizeof(bindings) / sizeof(bindings[0]); i++) {
    if (strcmp(bindings[i].name, name) == 0) {
      material_t m = material_rough(bindings[i].tint, bindings[i].roughness);
      m.texture = bindings[i].texture;
      if (strcmp(name, "lamp") == 0) m.metallic = 0.7f;
      if (strcmp(name, "water") == 0) m.metallic = 0.4f;
      if (strcmp(name, "facade_fresco") == 0 ||
          strcmp(name, "facade_brick") == 0 || strcmp(name, "shopfront") == 0)
        m.wall_ao = 0.8f;
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
    if (strcmp(part->material, material) != 0 ||
        city_source_is_pole(part->source_id))
      continue; /* poles are entities of their own */
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

/* Lamp posts, signals and road signs: each pole's parts become entities of
 * their own and its solid part a collider of its own, so one can be knocked
 * over without touching the rest. */
static bool add_poles(scene_t* scene, scene_context_t c,
                      const city_chunk_t* chunk, int* draws, int* colliders) {
  size_t p = 0;
  while (p < chunk->part_count) {
    if (!city_source_is_pole(chunk->parts[p].source_id)) {
      p++;
      continue;
    }
    uint64_t id = chunk->parts[p].source_id;
    pole_t pole;
    memset(&pole, 0, sizeof(pole));
    pole.kind = POLE_LAMP;
    aabb_t bounds = aabb_empty();
    size_t first = p;
    for (; p < chunk->part_count && chunk->parts[p].source_id == id; p++) {
      const city_part_t* part = &chunk->parts[p];
      if (part->mesh.vertex_count == 0) continue;
      if (strcmp(part->material, "sign_pole") == 0) pole.kind = POLE_SIGN;
      if (strcmp(part->material, "furniture") == 0) pole.kind = POLE_SIGNAL;
      mesh_data_t copy = mesh_data_create();
      if (!mesh_data_append(&copy, &part->mesh)) {
        mesh_data_free(&copy);
        return false;
      }
      mesh_handle_t handle = renderer_upload_mesh(c.renderer, &copy);
      scene_track_mesh(scene, handle);
      char label[ENTITY_LABEL_LENGTH];
      snprintf(label, sizeof(label), "%s %d,%d", part->material, chunk->column,
               chunk->row);
      aabb_t local = mesh_data_bounds(&copy);
      bounds = aabb_merge(bounds, local);
      if (pole.entity_count < POLE_MAX_ENTITIES)
        pole.entities[pole.entity_count++] =
            world_add(c.world, transform_identity(), handle,
                      material_for(part->material, c.textures), local, label);
      (*draws)++;
      mesh_data_free(&copy);
      if (part->collider == PART_COLLIDER_MESH) {
        vec3_t* positions = malloc(part->mesh.vertex_count * sizeof(vec3_t));
        if (!positions) return false;
        for (size_t v = 0; v < part->mesh.vertex_count; v++)
          positions[v] = part->mesh.vertices[v].position;
        pole.collider = physics_add_static_mesh(
            c.physics, positions, part->mesh.vertex_count, part->mesh.indices,
            part->mesh.index_count, (collision_layer_t)part->layer,
            part->friction, part->source_id);
        free(positions);
        if (pole.collider.id == 0) return false;
        (*colliders)++;
      }
    }
    /* The foot is where the lowest vertices sit. */
    pole.height = bounds.max.y - bounds.min.y;
    float foot_x = 0, foot_z = 0;
    int on_ground = 0;
    for (size_t q = first; q < p; q++)
      for (size_t v = 0; v < chunk->parts[q].mesh.vertex_count; v++) {
        vec3_t position = chunk->parts[q].mesh.vertices[v].position;
        if (position.y > bounds.min.y + 0.05f) continue;
        foot_x += position.x, foot_z += position.z, on_ground++;
      }
    pole.foot =
        vec3(on_ground ? foot_x / (float)on_ground : bounds.min.x, bounds.min.y,
             on_ground ? foot_z / (float)on_ground : bounds.min.z);
    scene->pole_count++;
    pole_t* grown = realloc(scene->poles, scene->pole_count * sizeof(pole_t));
    if (!grown) return false;
    scene->poles = grown;
    scene->poles[scene->pole_count - 1] = pole;
  }
  return true;
}

static bool add_colliders(scene_context_t c, const city_chunk_t* chunk,
                          int* colliders) {
  for (size_t p = 0; p < chunk->part_count; p++) {
    const city_part_t* part = &chunk->parts[p];
    if (part->collider != PART_COLLIDER_MESH ||
        city_source_is_pole(part->source_id))
      continue;
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
  int draws = 0, colliders = 0;
  bool ok = true;
  for (size_t ch = 0; ok && ch < package.chunk_count; ch++) {
    const city_chunk_t* chunk = &package.chunks[ch];
    /* Every material the chunk uses, once, in order of first appearance:
     * a material the generator adds is drawn without being listed here. */
    for (size_t p = 0; ok && p < chunk->part_count; p++) {
      bool seen = false;
      for (size_t q = 0; q < p && !seen; q++)
        seen = strcmp(chunk->parts[q].material, chunk->parts[p].material) == 0;
      if (!seen)
        ok = upload_merged(scene, c, chunk, chunk->parts[p].material, &draws);
    }
    if (ok) ok = add_poles(scene, c, chunk, &draws, &colliders);
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
                                      0.0f,
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
  scene->crossings = package.crossings;
  scene->crossing_count = package.crossing_count;
  package.crossings = NULL, package.crossing_count = 0;
  scene->bus_stops = package.bus_stops;
  scene->bus_stop_count = package.bus_stop_count;
  package.bus_stops = NULL, package.bus_stop_count = 0;
  scene->parked = package.parked;
  scene->parked_count = package.parked_count;
  package.routes = NULL, package.route_count = 0;
  package.parked = NULL, package.parked_count = 0;
  city_package_free(&package);
  if (!ok) LOG_ERROR("scene", "could not build the city scene");
  return ok;
}
