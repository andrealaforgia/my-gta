#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "city_generator.h"
#include "city_import.h"
#include "city_package.h"
#include "city_validate.h"
#include "osm_reader.h"
#include "test_groups.h"
#include "test_support.h"
#include "triangulate.h"

static city_model_t fixture_model(void) {
  osm_data_t osm;
  assert(osm_load(&osm, "tests/fixtures/district.osm"));
  city_model_t model;
  assert(city_import(&model, &osm, city_import_default_options(), "tests/fixtures/district.osm"));
  osm_free(&osm);
  return model;
}

static city_package_t generate_fixture(city_generator_stats_t* stats) {
  city_model_t model = fixture_model();
  city_generator_options_t options = city_generator_default_options();
  city_package_t package;
  assert(city_generate(&package, &model, &options, "district", stats));
  city_model_free(&model);
  return package;
}

static size_t count_material(const city_package_t* package, const char* prefix) {
  size_t count = 0;
  for (size_t c = 0; c < package->chunk_count; c++)
    for (size_t p = 0; p < package->chunks[c].part_count; p++)
      if (strncmp(package->chunks[c].parts[p].material, prefix, strlen(prefix)) == 0) count++;
  return count;
}

static const city_part_t* part_for(const city_package_t* package, const char* material, uint64_t source) {
  for (size_t c = 0; c < package->chunk_count; c++)
    for (size_t p = 0; p < package->chunks[c].part_count; p++) {
      const city_part_t* part = &package->chunks[c].parts[p];
      if (strcmp(part->material, material) == 0 && part->source_id == source) return part;
    }
  return NULL;
}

static void ear_clipping_handles_convex_and_concave_rings(void) {
  local_point_t square[4] = {{0, 0}, {10, 0}, {10, -10}, {0, -10}};
  uint32_t indices[12];
  bool clean;
  assert(triangulate_polygon(square, 4, indices, &clean) == 2 && clean);
  /* An L shape, anticlockwise in (x, north). */
  local_point_t l_shape[6] = {{0, 0}, {10, 0}, {10, -4}, {4, -4}, {4, -10}, {0, -10}};
  assert(triangulate_polygon(l_shape, 6, indices, &clean) == 4 && clean);
  double area = 0;
  for (int t = 0; t < 4; t++) {
    local_point_t tri[3] = {l_shape[indices[t * 3]], l_shape[indices[t * 3 + 1]], l_shape[indices[t * 3 + 2]]};
    area += fabs(polygon_signed_area(tri, 3));
  }
  assert(near_to(area, 64.0, 1e-6));
  local_point_t clockwise[4] = {{0, 0}, {0, -10}, {10, -10}, {10, 0}};
  assert(triangulate_polygon(clockwise, 4, indices, &clean) == 2 && clean);
}

static void every_road_junction_and_building_becomes_geometry(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  assert(stats.roads == 14);
  assert(count_material(&package, "road") == 14);
  assert(stats.junction_patches >= 6); /* the five T and cross junctions plus n3 */
  assert(count_material(&package, "junction") == stats.junction_patches);
  assert(stats.buildings == 4 && count_material(&package, "roof") == 4 && count_material(&package, "facade") == 4);
  assert(stats.pavements == 13); /* every road except the service stub */
  assert(stats.ground_tiles == package.chunk_count);
  assert(package.chunk_count >= 4);
  city_package_free(&package);
}

static void road_ribbons_have_their_width_and_roofs_their_height(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  const city_part_t* main_street = part_for(&package, "road", 102);
  assert(main_street);
  aabb_t bounds = mesh_data_bounds(&main_street->mesh);
  assert(near_to(bounds.max.z - bounds.min.z, 9.0f, 0.05f)); /* an east-west road, 9 m wide */
  assert(near(bounds.min.y, 0.02f));
  const city_part_t* roof = part_for(&package, "roof", 201);
  assert(roof && roof->mesh.vertex_count == 4 && mesh_data_triangle_count(&roof->mesh) == 2);
  for (size_t v = 0; v < roof->mesh.vertex_count; v++) assert(near(roof->mesh.vertices[v].position.y, 12));
  const city_part_t* walls = part_for(&package, "facade", 201);
  if (!walls) {
    for (int i = 0; i < 4 && !walls; i++) {
      char name[16];
      snprintf(name, sizeof(name), "facade%d", i);
      walls = part_for(&package, name, 201);
    }
  }
  assert(walls && mesh_data_triangle_count(&walls->mesh) == 8);
  assert(walls->collider == PART_COLLIDER_MESH && walls->layer == 2);
  /* Wall normals point away from the footprint centre. */
  aabb_t footprint = mesh_data_bounds(&walls->mesh);
  vec3_t centre = aabb_center(footprint);
  for (size_t v = 0; v < walls->mesh.vertex_count; v++) {
    vec3_t outward = vec3_sub(walls->mesh.vertices[v].position, centre);
    outward.y = 0;
    assert(vec3_dot(outward, walls->mesh.vertices[v].normal) > 0);
  }
  city_package_free(&package);
}

static void faces_point_up_or_outward_and_the_spawn_sits_on_a_road(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  for (size_t c = 0; c < package.chunk_count; c++) {
    for (size_t p = 0; p < package.chunks[c].part_count; p++) {
      const city_part_t* part = &package.chunks[c].parts[p];
      if (strcmp(part->material, "kerb") == 0) continue;
      bool flat = strncmp(part->material, "facade", 6) != 0;
      for (size_t i = 0; i + 2 < part->mesh.index_count; i += 3) {
        vec3_t a = part->mesh.vertices[part->mesh.indices[i]].position;
        vec3_t b = part->mesh.vertices[part->mesh.indices[i + 1]].position;
        vec3_t d = part->mesh.vertices[part->mesh.indices[i + 2]].position;
        vec3_t n = vec3_cross(vec3_sub(b, a), vec3_sub(d, a));
        if (flat) assert(n.y > 0);
        else assert(vec3_dot(n, part->mesh.vertices[part->mesh.indices[i]].normal) > 0);
      }
    }
  }
  assert(package.spawn_position.y > 0.5f);
  bool on_road = false;
  for (size_t c = 0; c < package.chunk_count && !on_road; c++)
    for (size_t p = 0; p < package.chunks[c].part_count && !on_road; p++) {
      const city_part_t* part = &package.chunks[c].parts[p];
      if (strcmp(part->material, "road") != 0) continue;
      aabb_t b = mesh_data_bounds(&part->mesh);
      b.min.y = -1;
      b.max.y = 2;
      if (aabb_contains(b, package.spawn_position)) on_road = true;
    }
  assert(on_road);
  city_package_free(&package);
}

static void the_package_round_trips_and_validates(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  const char* directory = "/tmp/gta-like-test-package";
  assert(city_package_write(&package, directory));
  city_package_t loaded;
  assert(city_package_read(&loaded, directory));
  assert(loaded.chunk_count == package.chunk_count);
  assert(city_package_triangle_count(&loaded) == city_package_triangle_count(&package));
  assert(near_to(loaded.spawn_position.x, package.spawn_position.x, 1e-3));
  assert(loaded.config_hash == package.config_hash);
  assert(strcmp(loaded.source, "tests/fixtures/district.osm") == 0);
  city_validation_t report;
  assert(city_validate(&loaded, &report));
  assert(report.roads == 14 && report.buildings == 4 && report.issues == 0);
  /* Corrupt an index and the validator must object. */
  loaded.chunks[0].parts[0].mesh.indices[0] = 999999;
  assert(!city_validate(&loaded, &report));
  assert(report.issues >= 1);
  city_package_free(&loaded);
  city_package_free(&package);
}

static void generation_is_deterministic(void) {
  city_generator_stats_t stats;
  city_package_t a = generate_fixture(&stats);
  city_package_t b = generate_fixture(&stats);
  assert(city_package_write(&a, "/tmp/gta-like-test-package-a"));
  assert(city_package_write(&b, "/tmp/gta-like-test-package-b"));
  assert(system("diff -r /tmp/gta-like-test-package-a /tmp/gta-like-test-package-b > /dev/null") == 0);
  city_package_free(&a);
  city_package_free(&b);
}

static void generator_settings_load_and_hash(void) {
  city_generator_options_t options;
  assert(city_generator_options_load(&options, "assets/city/default.cfg"));
  assert(near(options.chunk_size, 150) && options.pavements);
  city_generator_options_t defaults = city_generator_default_options();
  assert(city_generator_options_hash(&options) == city_generator_options_hash(&defaults));
  options.chunk_size = 100;
  assert(city_generator_options_hash(&options) != city_generator_options_hash(&defaults));
  assert(!city_generator_options_load(&options, "assets/city/missing.cfg"));
}

void run_generator_tests(void) {
  RUN(ear_clipping_handles_convex_and_concave_rings);
  RUN(every_road_junction_and_building_becomes_geometry);
  RUN(road_ribbons_have_their_width_and_roofs_their_height);
  RUN(faces_point_up_or_outward_and_the_spawn_sits_on_a_road);
  RUN(the_package_round_trips_and_validates);
  RUN(generation_is_deterministic);
  RUN(generator_settings_load_and_hash);
}
