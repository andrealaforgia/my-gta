#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "log.h"
#include "scene.h"

void scene_track_mesh(scene_t* scene, mesh_handle_t mesh) {
  if (scene->mesh_count < SCENE_MAX_MESHES)
    scene->meshes[scene->mesh_count++] = mesh;
}

void scene_release(scene_t* scene, renderer_t* renderer) {
  for (int i = 0; i < scene->mesh_count; i++)
    renderer_release_mesh(renderer, scene->meshes[i]);
  scene->mesh_count = 0;
}

static material_t textured(texture_handle_t texture, vec3_t tint,
                           float roughness) {
  material_t m = material_rough(tint, roughness);
  m.texture = texture;
  return m;
}

static void add_static_box(scene_t* scene, scene_context_t c, vec3_t center,
                           vec3_t half, material_t material,
                           collision_layer_t layer, const char* label) {
  mesh_data_t mesh = mesh_data_create();
  mesh_data_add_box(&mesh, vec3_zero(), half);
  mesh_handle_t handle = renderer_upload_mesh(c.renderer, &mesh);
  scene_track_mesh(scene, handle);
  transform_t t = transform_at(center, quat_identity());
  world_add(c.world, t, handle, material, mesh_data_bounds(&mesh), label);
  physics_add_static_box(c.physics, t, half, layer, 0.8f, 0);
  mesh_data_free(&mesh);
}

static void add_ground(scene_t* scene, scene_context_t c, float half_size) {
  mesh_data_t mesh = mesh_data_create();
  mesh_data_add_grid(&mesh, half_size, 20.0f);
  /* The grid's uv is one repeat per cell; scale so asphalt grain is ~2 m. */
  for (size_t i = 0; i < mesh.vertex_count; i++) {
    mesh.vertices[i].u *= 10.0f;
    mesh.vertices[i].v *= 10.0f;
  }
  mesh_handle_t handle = renderer_upload_mesh(c.renderer, &mesh);
  scene_track_mesh(scene, handle);
  world_add(c.world, transform_identity(), handle,
            textured(c.textures->asphalt, vec3(0.9f, 0.9f, 0.9f), 0.95f),
            mesh_data_bounds(&mesh), "ground");
  vec3_t* positions = malloc(mesh.vertex_count * sizeof(vec3_t));
  if (positions) {
    for (size_t i = 0; i < mesh.vertex_count; i++)
      positions[i] = mesh.vertices[i].position;
    physics_add_static_mesh(c.physics, positions, mesh.vertex_count,
                            mesh.indices, mesh.index_count,
                            COLLISION_LAYER_GROUND, 1.0f, 0);
    free(positions);
  }
  mesh_data_free(&mesh);
}

static void add_ramp(scene_t* scene, scene_context_t c, vec3_t base,
                     float length, float width, float height) {
  /* A wedge rising towards -Z, with a flat top and a drop at the far end. */
  float hw = width * 0.5f;
  vec3_t a = vec3_add(base, vec3(-hw, 0, 0)),
         b = vec3_add(base, vec3(hw, 0, 0));
  vec3_t c1 = vec3_add(base, vec3(hw, height, -length)),
         d = vec3_add(base, vec3(-hw, height, -length));
  vec3_t e = vec3_add(base, vec3(hw, 0, -length)),
         f = vec3_add(base, vec3(-hw, 0, -length));
  mesh_data_t mesh = mesh_data_create();
  mesh_data_add_quad(&mesh, a, b, c1, d, 0.2f); /* slope, facing up */
  mesh_data_add_quad(&mesh, d, c1, e, f, 0.2f); /* back face */
  /* Sides as triangles. */
  uint32_t base_index = (uint32_t)mesh.vertex_count;
  vertex_t side[6] = {{a, {-1, 0, 0}, 0, 0}, {d, {-1, 0, 0}, 1, 1},
                      {f, {-1, 0, 0}, 1, 0}, {b, {1, 0, 0}, 0, 0},
                      {e, {1, 0, 0}, 1, 0},  {c1, {1, 0, 0}, 1, 1}};
  for (int i = 0; i < 6; i++) mesh_data_add_vertex(&mesh, side[i]);
  mesh_data_add_triangle(&mesh, base_index, base_index + 1, base_index + 2);
  mesh_data_add_triangle(&mesh, base_index + 3, base_index + 4, base_index + 5);
  mesh_handle_t handle = renderer_upload_mesh(c.renderer, &mesh);
  scene_track_mesh(scene, handle);
  world_add(c.world, transform_identity(), handle,
            textured(c.textures->concrete, vec3(0.8f, 0.8f, 0.8f), 0.9f),
            mesh_data_bounds(&mesh), "ramp");
  vec3_t* positions = malloc(mesh.vertex_count * sizeof(vec3_t));
  if (positions) {
    for (size_t i = 0; i < mesh.vertex_count; i++)
      positions[i] = mesh.vertices[i].position;
    physics_add_static_mesh(c.physics, positions, mesh.vertex_count,
                            mesh.indices, mesh.index_count,
                            COLLISION_LAYER_GROUND, 0.9f, 0);
    free(positions);
  }
  mesh_data_free(&mesh);
}

bool scene_build_test_track(scene_t* scene, scene_context_t c) {
  memset(scene, 0, sizeof(*scene));
  snprintf(scene->name, sizeof(scene->name), "test track");
  const float half = 200.0f;
  add_ground(scene, c, half);

  material_t concrete =
      textured(c.textures->concrete, vec3(0.85f, 0.85f, 0.85f), 0.9f);
  material_t cone = material_rough(vec3(0.95f, 0.45f, 0.05f), 0.6f);
  material_t block = textured(c.textures->facade, vec3(0.9f, 0.9f, 0.9f), 0.8f);

  /* Perimeter barriers. */
  float wall_h = 1.0f, wall_t = 0.4f;
  add_static_box(scene, c, vec3(0, wall_h, -half), vec3(half, wall_h, wall_t),
                 concrete, COLLISION_LAYER_BARRIER, "north wall");
  add_static_box(scene, c, vec3(0, wall_h, half), vec3(half, wall_h, wall_t),
                 concrete, COLLISION_LAYER_BARRIER, "south wall");
  add_static_box(scene, c, vec3(-half, wall_h, 0), vec3(wall_t, wall_h, half),
                 concrete, COLLISION_LAYER_BARRIER, "west wall");
  add_static_box(scene, c, vec3(half, wall_h, 0), vec3(wall_t, wall_h, half),
                 concrete, COLLISION_LAYER_BARRIER, "east wall");

  /* A slalom of cones ahead of the spawn, a ramp to the right, a bump
   * strip to the left and some blocks to collide with. */
  for (int i = 0; i < 8; i++) {
    float x = (i % 2 == 0) ? -2.5f : 2.5f;
    add_static_box(scene, c, vec3(x, 0.35f, -40.0f - i * 12.0f),
                   vec3(0.2f, 0.35f, 0.2f), cone, COLLISION_LAYER_PROP, "cone");
  }
  add_ramp(scene, c, vec3(30, 0, -40), 14.0f, 6.0f, 1.6f);
  for (int i = 0; i < 6; i++)
    add_static_box(scene, c, vec3(-30, 0.05f, -40.0f - i * 6.0f),
                   vec3(3.0f, 0.05f, 0.3f), concrete, COLLISION_LAYER_GROUND,
                   "bump");
  add_static_box(scene, c, vec3(60, 4, -120), vec3(10, 4, 10), block,
                 COLLISION_LAYER_BUILDING, "block");
  add_static_box(scene, c, vec3(-60, 6, -120), vec3(8, 6, 14), block,
                 COLLISION_LAYER_BUILDING, "block");
  add_static_box(scene, c, vec3(0, 1.5f, -160), vec3(1.5f, 1.5f, 1.5f),
                 concrete, COLLISION_LAYER_PROP, "crate");

  scene->spawn_position = vec3(0, 0.9f, 0);
  scene->spawn_rotation = quat_identity();
  render_environment_t environment = {vec3(0.45f, 0.9f, 0.35f),
                                      vec3(1.15f, 1.1f, 0.98f),
                                      0.3f,
                                      vec3(0.58f, 0.72f, 0.92f),
                                      120.0f,
                                      700.0f,
                                      90.0f};
  scene->environment = environment;
  LOG_INFO("scene", "test track: %zu entities, %zu colliders",
           world_count(c.world), physics_collider_count(c.physics));
  return true;
}
