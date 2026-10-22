#include "city_scene.h"

#include <math.h>
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
      {"pavement", textures->concrete, vec3(0.72f, 0.72f, 0.72f), 0.9f},
      {"kerb", textures->concrete, vec3(0.6f, 0.6f, 0.6f), 0.9f},
      {"grass", textures->grass, vec3(0.9f, 0.9f, 0.9f), 1.0f},
      {"ground", textures->earth, vec3(0.78f, 0.78f, 0.78f), 1.0f},
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
      {"paving", textures->paving, vec3(0.8f, 0.76f, 0.7f), 0.85f},
      {"cornice", {0}, vec3(0.8f, 0.76f, 0.68f), 0.85f},
      {"door", textures->door, vec3(1, 1, 1), 0.7f},
      {"parking", textures->asphalt, vec3(0.78f, 0.78f, 0.78f), 0.95f},
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

/* Growable lists on a chunk's state. */
static bool remember_entity(chunk_state_t* st, uint32_t id) {
  if (st->entity_count == st->entity_capacity) {
    size_t capacity = st->entity_capacity ? st->entity_capacity * 2 : 64;
    uint32_t* grown = realloc(st->entities, capacity * sizeof(*grown));
    if (!grown) return false;
    st->entities = grown;
    st->entity_capacity = capacity;
  }
  st->entities[st->entity_count++] = id;
  return true;
}

static bool remember_mesh(chunk_state_t* st, mesh_handle_t mesh) {
  if (st->mesh_count == st->mesh_capacity) {
    size_t capacity = st->mesh_capacity ? st->mesh_capacity * 2 : 64;
    mesh_handle_t* grown = realloc(st->meshes, capacity * sizeof(*grown));
    if (!grown) return false;
    st->meshes = grown;
    st->mesh_capacity = capacity;
  }
  st->meshes[st->mesh_count++] = mesh;
  return true;
}

static bool remember_collider(chunk_state_t* st, collider_handle_t handle) {
  if (st->collider_count == st->collider_capacity) {
    size_t capacity = st->collider_capacity ? st->collider_capacity * 2 : 32;
    collider_handle_t* grown =
        realloc(st->colliders, capacity * sizeof(*grown));
    if (!grown) return false;
    st->colliders = grown;
    st->collider_capacity = capacity;
  }
  st->colliders[st->collider_count++] = handle;
  return true;
}

/* One upload per chunk and material: the merged mesh keeps draw calls low. */
static bool upload_merged(chunk_state_t* st, scene_context_t c,
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
  char label[ENTITY_LABEL_LENGTH];
  snprintf(label, sizeof(label), "%s %d,%d", material, chunk->column,
           chunk->row);
  uint32_t entity = world_add(c.world, transform_identity(), handle,
                              material_for(material, c.textures),
                              mesh_data_bounds(&merged), label);
  (*draws)++;
  mesh_data_free(&merged);
  return remember_mesh(st, handle) && remember_entity(st, entity);
}

/* Lamp posts, signals and road signs: each pole's parts become entities of
 * their own and its solid part a collider of its own, so one can be knocked
 * over without touching the rest. */
static bool add_poles(scene_t* scene, chunk_state_t* st, scene_context_t c,
                      const city_chunk_t* chunk, int chunk_index, int* draws,
                      int* colliders) {
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
    pole.chunk = chunk_index;
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
      if (!remember_mesh(st, handle)) return false;
      char label[ENTITY_LABEL_LENGTH];
      snprintf(label, sizeof(label), "%s %d,%d", part->material, chunk->column,
               chunk->row);
      aabb_t local = mesh_data_bounds(&copy);
      bounds = aabb_merge(bounds, local);
      uint32_t entity =
          world_add(c.world, transform_identity(), handle,
                    material_for(part->material, c.textures), local, label);
      if (!remember_entity(st, entity)) return false;
      if (pole.entity_count < POLE_MAX_ENTITIES)
        pole.entities[pole.entity_count++] = entity;
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

/* Solid parts of a chunk: with `ground` the ground layer (roads, kerbs,
 * pavements, grass), which stays for good, otherwise buildings and props,
 * which go with the chunk. */
static bool add_colliders(chunk_state_t* st, scene_context_t c,
                          const city_chunk_t* chunk, bool ground,
                          int* colliders) {
  for (size_t p = 0; p < chunk->part_count; p++) {
    const city_part_t* part = &chunk->parts[p];
    if (part->collider != PART_COLLIDER_MESH ||
        city_source_is_pole(part->source_id) ||
        (part->layer == COLLISION_LAYER_GROUND) != ground)
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
    if (!ground && !remember_collider(st, handle)) return false;
    (*colliders)++;
  }
  return true;
}

static bool load_chunk(scene_t* scene, size_t index, int* draws,
                       int* colliders) {
  scene_context_t c = scene->context;
  const city_chunk_t* chunk = &scene->package->chunks[index];
  chunk_state_t* st = &scene->chunk_states[index];
  bool ok = true;
  /* Every material the chunk uses, once, in order of first appearance: a
   * material the generator adds is drawn without being listed here. */
  for (size_t p = 0; ok && p < chunk->part_count; p++) {
    bool seen = false;
    for (size_t q = 0; q < p && !seen; q++)
      seen = strcmp(chunk->parts[q].material, chunk->parts[p].material) == 0;
    if (!seen)
      ok = upload_merged(st, c, chunk, chunk->parts[p].material, draws);
  }
  if (ok) ok = add_poles(scene, st, c, chunk, (int)index, draws, colliders);
  if (ok) ok = add_colliders(st, c, chunk, false, colliders);
  st->loaded = true;
  return ok;
}

static void unload_chunk(scene_t* scene, size_t index) {
  scene_context_t c = scene->context;
  chunk_state_t* st = &scene->chunk_states[index];
  for (size_t i = 0; i < st->entity_count; i++)
    world_remove(c.world, st->entities[i]);
  for (size_t i = 0; i < st->mesh_count; i++)
    renderer_release_mesh(c.renderer, st->meshes[i]);
  for (size_t i = 0; i < st->collider_count; i++)
    physics_remove_collider(c.physics, st->colliders[i]);
  for (size_t i = 0; i < scene->pole_count; i++) {
    pole_t* pole = &scene->poles[i];
    if (pole->chunk != (int)index || pole->state == POLE_GONE) continue;
    if (pole->collider.id != 0)
      physics_remove_collider(c.physics, pole->collider);
    pole->collider.id = 0;
    pole->state = POLE_GONE;
    scene->poles_gone++;
  }
  st->entity_count = st->mesh_count = st->collider_count = 0;
  st->loaded = false;
  /* Drop the gone poles once they pile up, so the list stays short. */
  if (scene->poles_gone > 2048) {
    size_t kept = 0;
    for (size_t i = 0; i < scene->pole_count; i++)
      if (scene->poles[i].state != POLE_GONE)
        scene->poles[kept++] = scene->poles[i];
    scene->pole_count = kept;
    scene->poles_gone = 0;
  }
}

/* Metres from a point to a chunk's footprint. */
static float chunk_distance(const city_chunk_t* chunk, vec3_t centre) {
  float dx = fmaxf(
      fmaxf(chunk->bounds.min.x - centre.x, centre.x - chunk->bounds.max.x),
      0.0f);
  float dz = fmaxf(
      fmaxf(chunk->bounds.min.z - centre.z, centre.z - chunk->bounds.max.z),
      0.0f);
  return sqrtf(dx * dx + dz * dz);
}

void scene_stream(scene_t* scene, vec3_t centre, int budget) {
  if (!scene->package) return;
  int draws = 0, colliders = 0;
  /* Nearest first, so what is under the wheels is there before the rest. */
  while (true) {
    size_t best = scene->package->chunk_count;
    float best_distance = scene->stream_load_radius;
    for (size_t i = 0; i < scene->package->chunk_count; i++) {
      if (scene->chunk_states[i].loaded) continue;
      float d = chunk_distance(&scene->package->chunks[i], centre);
      if (d < best_distance) best_distance = d, best = i;
    }
    if (best == scene->package->chunk_count) break;
    if (best_distance > scene->stream_near_radius && budget <= 0) break;
    if (!load_chunk(scene, best, &draws, &colliders))
      LOG_ERROR("scene", "could not load chunk %zu", best);
    budget--;
  }
  for (size_t i = 0; i < scene->package->chunk_count; i++)
    if (scene->chunk_states[i].loaded &&
        chunk_distance(&scene->package->chunks[i], centre) >
            scene->stream_unload_radius)
      unload_chunk(scene, i);
}

void scene_release_streamed(scene_t* scene, renderer_t* renderer) {
  if (scene->package) {
    for (size_t i = 0; i < scene->package->chunk_count; i++) {
      chunk_state_t* st = &scene->chunk_states[i];
      for (size_t m = 0; m < st->mesh_count; m++)
        renderer_release_mesh(renderer, st->meshes[m]);
      free(st->entities);
      free(st->meshes);
      free(st->colliders);
    }
    city_package_free(scene->package);
    free(scene->package);
    scene->package = NULL;
  }
  free(scene->chunk_states);
  scene->chunk_states = NULL;
}

bool scene_build_city(scene_t* scene, scene_context_t c,
                      const char* package_directory) {
  memset(scene, 0, sizeof(*scene));
  if (!package_directory) {
    LOG_ERROR("scene", "no city package given");
    return false;
  }
  scene->context = c;
  scene->stream_load_radius = 800.0f;
  scene->stream_unload_radius = 1000.0f;
  scene->stream_near_radius = 250.0f;
  scene->package = calloc(1, sizeof(city_package_t));
  if (!scene->package) return false;
  if (!city_package_read(scene->package, package_directory)) return false;
  city_package_t package = *scene->package; /* a view; the scene owns it */
  scene->chunk_states = calloc(package.chunk_count ? package.chunk_count : 1,
                               sizeof(chunk_state_t));
  if (!scene->chunk_states) return false;
  snprintf(scene->name, sizeof(scene->name), "city %s", package.name);
  int draws = 0, colliders = 0;
  bool ok = true;
  /* Every road, pavement and field is solid from the start, so far-off
   * traffic has a surface; buildings and props come with their chunk. */
  for (size_t ch = 0; ok && ch < package.chunk_count; ch++)
    ok = add_colliders(&scene->chunk_states[ch], c, &package.chunks[ch], true,
                       &colliders);
  /* A big city needs a finer picture to stay readable in the corner. */
  if (!city_map_build(&scene->map, &package,
                      package.chunk_count > 200 ? 4096 : 1024))
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
  *scene->package = package;
  /* The first chunks, all at once: everything within reach of the spawn. */
  if (ok) scene_stream(scene, scene->spawn_position, 1 << 20);
  if (!ok) LOG_ERROR("scene", "could not build the city scene");
  return ok;
}
