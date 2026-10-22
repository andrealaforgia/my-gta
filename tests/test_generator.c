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
  assert(city_import(&model, &osm, city_import_default_options(),
                     "tests/fixtures/district.osm"));
  osm_free(&osm);
  return model;
}

static city_package_t generate_fixture(city_generator_stats_t* stats) {
  city_model_t model = fixture_model();
  city_generator_options_t options = city_generator_default_options();
  snprintf(options.airport_side, sizeof(options.airport_side), "-");
  city_package_t package;
  assert(city_generate(&package, &model, &options, "district", stats));
  city_model_free(&model);
  return package;
}

static size_t count_material(const city_package_t* package,
                             const char* prefix) {
  size_t count = 0;
  for (size_t c = 0; c < package->chunk_count; c++)
    for (size_t p = 0; p < package->chunks[c].part_count; p++)
      if (strncmp(package->chunks[c].parts[p].material, prefix,
                  strlen(prefix)) == 0)
        count++;
  return count;
}

static const city_part_t* part_for(const city_package_t* package,
                                   const char* material, uint64_t source) {
  for (size_t c = 0; c < package->chunk_count; c++)
    for (size_t p = 0; p < package->chunks[c].part_count; p++) {
      const city_part_t* part = &package->chunks[c].parts[p];
      if (strcmp(part->material, material) == 0 &&
          city_source_original(part->source_id) == source)
        return part;
    }
  return NULL;
}

static const city_part_t* facade_for(const city_package_t* package,
                                     uint64_t source) {
  for (int i = 0; i < 12; i++) {
    char name[16];
    snprintf(name, sizeof(name), "facade%d", i);
    const city_part_t* walls = part_for(package, name, source);
    if (walls) return walls;
  }
  const city_part_t* brick = part_for(package, "facade_brick", source);
  return brick ? brick : part_for(package, "facade_fresco", source);
}

static const city_part_t* roof_for(const city_package_t* package,
                                   uint64_t source) {
  const char* kinds[] = {"roof", "roof_slate", "roof_flat"};
  for (int i = 0; i < 3; i++) {
    const city_part_t* roof = part_for(package, kinds[i], source);
    if (roof) return roof;
  }
  return NULL;
}

static void ear_clipping_handles_convex_and_concave_rings(void) {
  local_point_t square[4] = {{0, 0}, {10, 0}, {10, -10}, {0, -10}};
  uint32_t indices[12];
  bool clean;
  assert(triangulate_polygon(square, 4, indices, &clean) == 2 && clean);
  /* An L shape, anticlockwise in (x, north). */
  local_point_t l_shape[6] = {{0, 0},  {10, 0},  {10, -4},
                              {4, -4}, {4, -10}, {0, -10}};
  assert(triangulate_polygon(l_shape, 6, indices, &clean) == 4 && clean);
  double area = 0;
  for (int t = 0; t < 4; t++) {
    local_point_t tri[3] = {l_shape[indices[t * 3]],
                            l_shape[indices[t * 3 + 1]],
                            l_shape[indices[t * 3 + 2]]};
    area += fabs(polygon_signed_area(tri, 3));
  }
  assert(near_to(area, 64.0, 1e-6));
  local_point_t clockwise[4] = {{0, 0}, {0, -10}, {10, -10}, {10, 0}};
  assert(triangulate_polygon(clockwise, 4, indices, &clean) == 2 && clean);
}

static double triangle_area_sum(const local_point_t* points,
                                const uint32_t* indices, size_t triangles) {
  double area = 0;
  for (size_t t = 0; t < triangles; t++) {
    local_point_t tri[3] = {points[indices[t * 3]], points[indices[t * 3 + 1]],
                            points[indices[t * 3 + 2]]};
    area += fabs(polygon_signed_area(tri, 3));
  }
  return area;
}

static void ear_clipping_cuts_holes_out(void) {
  local_point_t outer[4] = {{0, 0}, {10, 0}, {10, -10}, {0, -10}};
  local_point_t hole[4] = {{3, -3}, {3, -7}, {7, -7}, {7, -3}}; /* clockwise */
  const local_point_t* holes[1] = {hole};
  size_t hole_counts[1] = {4};
  local_point_t merged[16];
  uint32_t indices[48];
  size_t merged_count;
  bool clean;
  size_t triangles = triangulate_with_holes(
      outer, 4, holes, hole_counts, 1, merged, &merged_count, indices, &clean);
  assert(clean && merged_count == 10);
  assert(triangles == 8);
  assert(
      near_to(triangle_area_sum(merged, indices, triangles), 100 - 16, 1e-6));
}

static void courtyards_are_open_to_the_sky(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  const city_part_t* roof = roof_for(&package, 404);
  /* The courtyard block has a ridge roof cut around its courtyard. */
  assert(roof && mesh_data_triangle_count(&roof->mesh) > 8);
  const city_part_t* walls = facade_for(&package, 404);
  assert(walls && mesh_data_triangle_count(&walls->mesh) >= 16);
  /* Courtyard walls face into the courtyard: their normals point towards
   * the centre of the footprint. */
  aabb_t bounds = mesh_data_bounds(&walls->mesh);
  vec3_t centre = aabb_center(bounds);
  int inward = 0, outward = 0;
  for (size_t v = 0; v < walls->mesh.vertex_count; v++) {
    vec3_t to_centre = vec3_sub(centre, walls->mesh.vertices[v].position);
    to_centre.y = 0;
    if (vec3_dot(to_centre, walls->mesh.vertices[v].normal) > 0)
      inward++;
    else
      outward++;
  }
  assert(inward >= 16 && inward == outward);
  city_package_free(&package);
}

static void every_road_junction_and_building_becomes_geometry(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  assert(stats.roads == 17); /* the n1 corner joined by the importer */
  assert(count_material(&package, "road") ==
         13); /* paths, the Corso and the stream are not asphalt */
  assert(stats.junction_patches >=
         6); /* the five T and cross junctions plus n3 */
  assert(count_material(&package, "junction") == stats.junction_patches);
  assert(stats.buildings == 8 && count_material(&package, "roof") == 8 &&
         count_material(&package, "facade") == 8);
  assert(stats.pavements == 12); /* every road except the service stub */
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
  assert(near_to(bounds.max.z - bounds.min.z, 9.0f,
                 0.05f)); /* an east-west road, 9 m wide */
  assert(near(bounds.min.y, 0.06f));
  /* A rectangular footprint gets a hipped roof: eaves at the wall height,
   * a ridge above it, two hips and two long slopes plus the underside of
   * the eaves. */
  const city_part_t* roof = roof_for(&package, 201);
  assert(roof && mesh_data_triangle_count(&roof->mesh) == 8);
  aabb_t roof_bounds = mesh_data_bounds(&roof->mesh);
  assert(near(roof_bounds.min.y, 12) && roof_bounds.max.y > 13.0f &&
         roof_bounds.max.y < 17.0f);
  /* The L-shaped footprint gets a ridge roof over its outline. */
  const city_part_t* ridge = roof_for(&package, 203);
  assert(ridge && mesh_data_triangle_count(&ridge->mesh) > 4);
  aabb_t ridge_bounds = mesh_data_bounds(&ridge->mesh);
  assert(ridge_bounds.max.y > ridge_bounds.min.y + 1.0f);
  const city_part_t* walls = facade_for(&package, 201);
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

static void roads_narrow_to_fit_between_buildings(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  /* Building 208 stands 3.2 m from the centre line of South Street's
   * first segment, so the 6 m road and its 2 m pavements cannot fit. */
  const city_part_t* squeezed = part_for(&package, "road", 101);
  assert(squeezed);
  /* The ribbon's vertices come in left/right pairs per point, so the
   * width varies along the road: squeezed beside the building, the full
   * 6 m where nothing stands near (the road continues round the n1
   * corner as one merged road). */
  float narrowest = 1e9f, widest = 0;
  for (size_t v = 0; v + 1 < squeezed->mesh.vertex_count; v += 2) {
    float width = vec3_distance(squeezed->mesh.vertices[v].position,
                                squeezed->mesh.vertices[v + 1].position);
    narrowest = fminf(narrowest, width);
    widest = fmaxf(widest, width);
  }
  for (size_t v = 0; v + 1 < squeezed->mesh.vertex_count; v += 2)
    assert(narrowest > 3.4f && narrowest < 3.7f);
  assert(widest > 5.9f && widest < 6.1f);
  const city_part_t* pavement = part_for(&package, "pavement", 101);
  assert(pavement);
  /* Beside the building the pavement is a sliver: road plus strips stay
   * inside the gap. */
  aabb_t footprint = mesh_data_bounds(&facade_for(&package, 208)->mesh);
  for (size_t v = 0; v < pavement->mesh.vertex_count; v++) {
    vec3_t p = pavement->mesh.vertices[v].position;
    if (p.x > footprint.min.x && p.x < footprint.max.x)
      assert(p.z > footprint.max.z - 0.05f);
  }
  /* An unconstrained road keeps its full width. */
  const city_part_t* free_road = part_for(&package, "road", 103);
  assert(free_road);
  aabb_t free_bounds = mesh_data_bounds(&free_road->mesh);
  assert(near_to(free_bounds.max.z - free_bounds.min.z, 6.0f, 0.05f));
  city_package_free(&package);
}

static void people_streets_paths_and_areas_get_their_own_surfaces(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  const city_part_t* corso = part_for(&package, "paving", 301);
  assert(corso && corso->collider == PART_COLLIDER_MESH);
  assert(part_for(&package, "pavement", 301) ==
         NULL); /* the whole street is paved */
  const city_part_t* footway = part_for(&package, "paving", 108);
  assert(footway);
  aabb_t footway_bounds = mesh_data_bounds(&footway->mesh);
  assert(footway_bounds.min.y > 0.06f); /* above the roads it crosses */
  assert(part_for(&package, "lamp", 108) == NULL);
  const city_part_t* water = part_for(&package, "water", 305);
  assert(water && water->collider == PART_COLLIDER_NONE);
  const city_part_t* stream = part_for(&package, "water", 309);
  assert(stream && stream->collider == PART_COLLIDER_NONE);
  assert(part_for(&package, "lamp", 309) == NULL);
  const city_part_t* parking = part_for(&package, "parking", 306);
  assert(parking && parking->collider == PART_COLLIDER_MESH);
  const city_part_t* square = part_for(&package, "paving", 304);
  assert(square);
  /* Streets for people and paths cross squares, so they sit above them
   * rather than fighting for the same height. */
  float square_top = mesh_data_bounds(&square->mesh).max.y;
  assert(mesh_data_bounds(&corso->mesh).min.y > square_top + 0.003f);
  assert(footway_bounds.min.y > square_top + 0.003f);
  assert(part_for(&package, "park", 403));
  assert(stats.areas == 5);
  /* Multipolygon buildings are built like any other. */
  assert(roof_for(&package, 402) && roof_for(&package, 401));
  city_package_free(&package);
}

static void trees_grow_in_parks_and_nowhere_else(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  assert(stats.trees > 0);
  const city_part_t* trunks = part_for(&package, "trunk", 403);
  const city_part_t* leaves = part_for(&package, "leaves", 403);
  assert(trunks && leaves && trunks->collider == PART_COLLIDER_MESH);
  assert(leaves->collider == PART_COLLIDER_NONE);
  assert(part_for(&package, "trunk", 306) == NULL); /* no trees in a car park */
  aabb_t bounds = mesh_data_bounds(&leaves->mesh);
  assert(bounds.min.y > 1.0f && bounds.max.y < 12.0f);
  city_package_free(&package);
}

static void parallel_carriageways_share_the_gap_between_them(void) {
  /* Two one-way residential streets 7 m apart with no shared node, as a
   * dual carriageway is mapped: 6 m roads with 2 m pavements would
   * overlap, so the pavements between them give way and the roads stay
   * their width. */
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.004\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/><node id=\"2\" "
      "lat=\"45.0000\" lon=\"12.0030\"/>"
      "<node id=\"3\" lat=\"45.000063\" lon=\"12.0000\"/><node id=\"4\" "
      "lat=\"45.000063\" lon=\"12.0030\"/>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/><tag k=\"oneway\" v=\"yes\"/></way>"
      "<way id=\"11\"><nd ref=\"4\"/><nd ref=\"3\"/><tag k=\"highway\" "
      "v=\"residential\"/><tag k=\"oneway\" v=\"yes\"/></way>"
      "</osm>";
  osm_data_t osm;
  assert(osm_parse(&osm, xml));
  city_model_t model;
  assert(city_import(&model, &osm, city_import_default_options(), "inline"));
  osm_free(&osm);
  city_generator_options_t options = city_generator_default_options();
  snprintf(options.airport_side, sizeof(options.airport_side), "-");
  options.lamp_spacing = 0;
  city_package_t package;
  city_generator_stats_t stats;
  assert(city_generate(&package, &model, &options, "dual", &stats));
  const city_part_t* road = part_for(&package, "road", 10);
  assert(road);
  aabb_t bounds = mesh_data_bounds(&road->mesh);
  assert(
      near_to(bounds.max.z - bounds.min.z, 6.0f, 0.05f)); /* full width kept */
  const city_part_t* pavement = part_for(&package, "pavement", 10);
  /* Either no pavement at all, or one that stays clear of the other road. */
  if (pavement) {
    aabb_t p = mesh_data_bounds(&pavement->mesh);
    assert(p.max.z - p.min.z < 6.0f + 2.0f * 0.5f + 0.1f);
  }
  city_package_free(&package);
  city_model_free(&model);
}

static void narrow_terraces_get_gabled_roofs(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  /* Building 208 is a 39 x 6 m terrace: the ridge runs the full length,
   * so the roof is two slopes and the underside of the eaves, and each
   * end wall rises to the ridge as a gable in the facade material. */
  const city_part_t* roof = roof_for(&package, 208);
  assert(roof && mesh_data_triangle_count(&roof->mesh) == 6);
  const city_part_t* walls = facade_for(&package, 208);
  assert(walls && mesh_data_triangle_count(&walls->mesh) == 10);
  aabb_t roof_bounds = mesh_data_bounds(&roof->mesh);
  aabb_t wall_bounds = mesh_data_bounds(&walls->mesh);
  float eaves = roof_bounds.min.y;
  assert(roof_bounds.max.y > eaves + 1.5f && roof_bounds.max.y < eaves + 2.5f);
  assert(
      near(wall_bounds.max.y, roof_bounds.max.y)); /* gables reach the ridge */
  /* No eaves past the gable ends; 0.35 m eaves along the sides of the
   * 5.7 m wide terrace. */
  assert(near_to(roof_bounds.max.x - roof_bounds.min.x,
                 wall_bounds.max.x - wall_bounds.min.x, 0.05f));
  assert(near_to(roof_bounds.max.z - roof_bounds.min.z, 6.4f, 0.2f));
  /* The wide block keeps its hips. */
  assert(mesh_data_triangle_count(&roof_for(&package, 201)->mesh) == 8);
  city_package_free(&package);
}

static city_package_t generate_inline(const char* xml, city_model_t* model) {
  osm_data_t osm;
  assert(osm_parse(&osm, xml));
  assert(city_import(model, &osm, city_import_default_options(), "inline"));
  osm_free(&osm);
  city_generator_options_t options = city_generator_default_options();
  snprintf(options.airport_side, sizeof(options.airport_side), "-");
  city_package_t package;
  city_generator_stats_t stats;
  assert(city_generate(&package, model, &options, "inline", &stats));
  return package;
}

static void irregular_buildings_get_ridge_roofs_and_tall_ones_stay_flat(void) {
  /* An L-shaped building 9 m high (building 1) and a 30 m tower with the
   * same plan (building 2). */
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.00026\"/>"
      "<node id=\"3\" lat=\"45.00009\" lon=\"12.00026\"/>"
      "<node id=\"4\" lat=\"45.00009\" lon=\"12.00013\"/>"
      "<node id=\"5\" lat=\"45.00018\" lon=\"12.00013\"/>"
      "<node id=\"6\" lat=\"45.00018\" lon=\"12.0000\"/>"
      "<node id=\"11\" lat=\"45.0010\" lon=\"12.0000\"/>"
      "<node id=\"12\" lat=\"45.0010\" lon=\"12.00026\"/>"
      "<node id=\"13\" lat=\"45.00109\" lon=\"12.00026\"/>"
      "<node id=\"14\" lat=\"45.00109\" lon=\"12.00013\"/>"
      "<node id=\"15\" lat=\"45.00118\" lon=\"12.00013\"/>"
      "<node id=\"16\" lat=\"45.00118\" lon=\"12.0000\"/>"
      "<way id=\"1\"><nd ref=\"1\"/><nd ref=\"2\"/><nd ref=\"3\"/><nd "
      "ref=\"4\"/><nd ref=\"5\"/><nd ref=\"6\"/><nd ref=\"1\"/><tag "
      "k=\"building\" v=\"yes\"/><tag k=\"height\" v=\"9\"/></way>"
      "<way id=\"2\"><nd ref=\"11\"/><nd ref=\"12\"/><nd ref=\"13\"/><nd "
      "ref=\"14\"/><nd ref=\"15\"/><nd ref=\"16\"/><nd ref=\"11\"/><tag "
      "k=\"building\" v=\"yes\"/><tag k=\"height\" v=\"30\"/></way>"
      "<way id=\"3\"><nd ref=\"1\"/><nd ref=\"11\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>"
      "</osm>";
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  const city_part_t* roof = roof_for(&package, 1);
  assert(roof && strcmp(roof->material, "roof_flat") != 0);
  aabb_t roof_bounds = mesh_data_bounds(&roof->mesh);
  assert(roof_bounds.min.y > 8.99f && roof_bounds.min.y < 9.5f);
  assert(roof_bounds.max.y > 10.0f && roof_bounds.max.y < 14.0f);
  const city_part_t* walls = facade_for(&package, 1);
  assert(walls);
  /* The walls rise to the roof: the gable ends reach the ridge. */
  assert(
      near_to(mesh_data_bounds(&walls->mesh).max.y, roof_bounds.max.y, 0.1f));
  /* Every roof face leans upward or sideways, never down. */
  for (size_t i = 0; i + 2 < roof->mesh.index_count; i += 3) {
    vec3_t a = roof->mesh.vertices[roof->mesh.indices[i]].position;
    vec3_t b = roof->mesh.vertices[roof->mesh.indices[i + 1]].position;
    vec3_t c = roof->mesh.vertices[roof->mesh.indices[i + 2]].position;
    assert(vec3_cross(vec3_sub(b, a), vec3_sub(c, a)).y > 0);
  }
  /* The tower stays flat. */
  const city_part_t* flat = part_for(&package, "roof_flat", 2);
  assert(flat);
  aabb_t flat_bounds = mesh_data_bounds(&flat->mesh);
  assert(near(flat_bounds.min.y, 30) && near(flat_bounds.max.y, 30));
  city_package_free(&package);
  city_model_free(&model);
}

static float slope_height(float x) { return 100.0f + 0.1f * x; }

static void everything_follows_the_terrain(void) {
  /* A street and a house on ground rising 10 % to the east: the terrain
   * is 100 m at x = 0 and climbs 0.1 m per metre. */
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0020\"/>"
      "<node id=\"3\" lat=\"45.0004\" lon=\"12.0006\"/>"
      "<node id=\"4\" lat=\"45.0004\" lon=\"12.0009\"/>"
      "<node id=\"5\" lat=\"45.0007\" lon=\"12.0009\"/>"
      "<node id=\"6\" lat=\"45.0007\" lon=\"12.0006\"/>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>"
      "<way id=\"20\"><nd ref=\"3\"/><nd ref=\"4\"/><nd ref=\"5\"/><nd "
      "ref=\"6\"/><nd ref=\"3\"/><tag k=\"building\" v=\"yes\"/><tag "
      "k=\"height\" v=\"8\"/></way>"
      "</osm>";
  osm_data_t osm;
  assert(osm_parse(&osm, xml));
  city_model_t model;
  assert(city_import(&model, &osm, city_import_default_options(), "inline"));
  osm_free(&osm);
  float heights[4] = {slope_height(-1000), slope_height(1000),
                      slope_height(-1000), slope_height(1000)};
  model.terrain.min_x = -1000, model.terrain.min_z = -1000;
  model.terrain.dx = 2000, model.terrain.dz = 2000;
  model.terrain.cols = 2, model.terrain.rows = 2;
  model.terrain.heights = malloc(sizeof(heights));
  memcpy(model.terrain.heights, heights, sizeof(heights));
  city_generator_options_t options = city_generator_default_options();
  snprintf(options.airport_side, sizeof(options.airport_side), "-");
  options.lamp_spacing = 0;
  city_package_t package;
  city_generator_stats_t stats;
  assert(city_generate(&package, &model, &options, "hill", &stats));
  /* The road surface sits 6 cm above the ground at every vertex. */
  const city_part_t* road = part_for(&package, "road", 10);
  assert(road && road->mesh.vertex_count >= 4);
  for (size_t v = 0; v < road->mesh.vertex_count; v++) {
    vec3_t p = road->mesh.vertices[v].position;
    assert(near_to(p.y, slope_height(p.x) + 0.06f, 0.05f));
  }
  /* The ground is a finer grid, just under the road. */
  size_t ground_vertices = 0;
  for (size_t c = 0; c < package.chunk_count; c++)
    for (size_t p = 0; p < package.chunks[c].part_count; p++) {
      const city_part_t* ground = &package.chunks[c].parts[p];
      if (strcmp(ground->material, "ground") != 0) continue;
      ground_vertices += ground->mesh.vertex_count;
      for (size_t v = 0; v < ground->mesh.vertex_count; v++) {
        vec3_t q = ground->mesh.vertices[v].position;
        assert(near_to(q.y, slope_height(q.x) - 0.05f, 0.02f));
      }
    }
  assert(ground_vertices > 4 * package.chunk_count);
  /* The house stands on the slope with a skirt below, and its roof is at
   * least its 8 m above where it stands. */
  const city_part_t* walls = facade_for(&package, 20);
  const city_part_t* roof = roof_for(&package, 20);
  assert(walls && roof);
  aabb_t wall_bounds = mesh_data_bounds(&walls->mesh);
  float base = slope_height(0.5f * (wall_bounds.min.x + wall_bounds.max.x));
  assert(wall_bounds.min.y < base - 1.5f);
  assert(mesh_data_bounds(&roof->mesh).min.y > base + 7.9f);
  /* The car spawns on the road, on the slope. */
  assert(near_to(package.spawn_position.y,
                 slope_height(package.spawn_position.x) + 0.06f + 0.9f, 0.1f));
  city_package_free(&package);
  city_model_free(&model);
}

static void footpaths_stop_at_the_pavement_instead_of_crossing_the_road(void) {
  /* A residential street with a footway running straight across it with
   * no shared node: the footway is drawn on either side but not over the
   * asphalt and its pavements. */
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.001\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0020\"/>"
      "<node id=\"3\" lat=\"45.0003\" lon=\"12.0010\"/>"
      "<node id=\"4\" lat=\"44.9997\" lon=\"12.0010\"/>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>"
      "<way id=\"11\"><nd ref=\"3\"/><nd ref=\"4\"/><tag k=\"highway\" "
      "v=\"footway\"/></way>"
      "</osm>";
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  const city_part_t* road = part_for(&package, "road", 10);
  const city_part_t* path = part_for(&package, "paving", 11);
  assert(road && path);
  aabb_t road_bounds = mesh_data_bounds(&road->mesh);
  float road_z = 0.5f * (road_bounds.min.z + road_bounds.max.z);
  size_t north = 0, south = 0;
  for (size_t v = 0; v < path->mesh.vertex_count; v++) {
    float dz = path->mesh.vertices[v].position.z - road_z;
    assert(fabsf(dz) > 4.5f); /* 3 m of road and 2 m of pavement, less slack */
    if (dz < 0)
      north++;
    else
      south++;
  }
  assert(north > 0 && south > 0); /* it is still there on both sides */
  city_package_free(&package);
  city_model_free(&model);
}

static void buildings_on_a_named_street_get_a_portico(void) {
  /* A pedestrian "Corso Test" along the middle with a 21 x 12 m building
   * 4 m north of it, 12 m high. With the street named in
   * arcade_streets the ground floor is set back 3 m behind pillars and
   * the walkway under it is paved; without, the wall is plain. */
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0005\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0005\" lon=\"12.0020\"/>"
      "<node id=\"3\" lat=\"45.00054\" lon=\"12.0006\"/>"
      "<node id=\"4\" lat=\"45.00054\" lon=\"12.0009\"/>"
      "<node id=\"5\" lat=\"45.00065\" lon=\"12.0009\"/>"
      "<node id=\"6\" lat=\"45.00065\" lon=\"12.0006\"/>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"pedestrian\"/><tag k=\"name\" v=\"Corso Test\"/></way>"
      "<way id=\"20\"><nd ref=\"3\"/><nd ref=\"4\"/><nd ref=\"5\"/><nd "
      "ref=\"6\"/><nd ref=\"3\"/><tag k=\"building\" v=\"yes\"/><tag "
      "k=\"height\" v=\"12\"/></way>"
      "</osm>";
  osm_data_t osm;
  assert(osm_parse(&osm, xml));
  city_model_t model;
  assert(city_import(&model, &osm, city_import_default_options(), "inline"));
  osm_free(&osm);
  city_generator_options_t plain_options = city_generator_default_options();
  snprintf(plain_options.airport_side, sizeof(plain_options.airport_side), "-");
  plain_options.lamp_spacing = 0;
  city_generator_options_t arcade_options = plain_options;
  snprintf(arcade_options.arcade_streets, sizeof(arcade_options.arcade_streets),
           "Corso_Test");
  city_package_t plain, arcaded;
  city_generator_stats_t stats;
  assert(city_generate(&plain, &model, &plain_options, "plain", &stats));
  assert(city_generate(&arcaded, &model, &arcade_options, "arcade", &stats));
  const city_part_t* plain_walls = facade_for(&plain, 20);
  const city_part_t* walls = facade_for(&arcaded, 20);
  assert(plain_walls && walls);
  /* Houses on the arcaded street are painted, as along the real Corso. */
  assert(strcmp(walls->material, "facade_fresco") == 0);
  assert(strcmp(plain_walls->material, "facade_fresco") != 0);
  assert(mesh_data_triangle_count(&walls->mesh) >
         mesh_data_triangle_count(&plain_walls->mesh) + 20);
  /* The street-facing edge is the building's southernmost: find its z. */
  aabb_t plain_bounds = mesh_data_bounds(&plain_walls->mesh);
  float edge_z = plain_bounds.max.z;
  float inset_z = edge_z - 3.0f;
  size_t on_edge_low = 0, on_inset_low = 0, on_edge_high = 0;
  for (size_t v = 0; v < walls->mesh.vertex_count; v++) {
    vec3_t p = walls->mesh.vertices[v].position;
    if (fabsf(p.z - edge_z) < 0.05f && p.y < 5.0f) on_edge_low++;
    if (fabsf(p.z - edge_z) < 0.05f && p.y > 11.9f) on_edge_high++;
    if (fabsf(p.z - inset_z) < 0.05f && p.y < 5.0f) on_inset_low++;
  }
  assert(on_edge_high >= 2); /* the upper floors stay on the building line */
  assert(on_edge_low >= 8);  /* pillars and the soffit edge stand on it */
  assert(on_inset_low >= 4); /* the ground floor wall is set back */
  /* Between the pillars the opening is round-headed: the front face carries
   * spandrels whose lower edge rises to a crown above the springing. */
  float crown = 0, springing = 1e9f;
  for (size_t v = 0; v < walls->mesh.vertex_count; v++) {
    vec3_t p = walls->mesh.vertices[v].position;
    if (fabsf(p.z - (edge_z - 0.35f)) < 0.05f && p.y > 2.0f && p.y < 4.7f) {
      crown = fmaxf(crown, p.y);
      springing = fminf(springing, p.y);
    }
  }
  assert(crown > 4.2f && springing < 2.8f);
  assert(part_for(&arcaded, "paving", 20)); /* the walkway is paved */
  assert(part_for(&plain, "paving", 20) == NULL);
  city_package_free(&plain);
  city_package_free(&arcaded);
  city_model_free(&model);
}

static void a_slender_tower_gets_a_tall_spire(void) {
  /* A 9 x 9 m bell tower, 60 m high, beside a 9 x 9 m house 10 m high. */
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.00012\"/>"
      "<node id=\"3\" lat=\"45.00008\" lon=\"12.00012\"/>"
      "<node id=\"4\" lat=\"45.00008\" lon=\"12.0000\"/>"
      "<node id=\"11\" lat=\"45.0005\" lon=\"12.0000\"/>"
      "<node id=\"12\" lat=\"45.0005\" lon=\"12.00012\"/>"
      "<node id=\"13\" lat=\"45.00058\" lon=\"12.00012\"/>"
      "<node id=\"14\" lat=\"45.00058\" lon=\"12.0000\"/>"
      "<way id=\"1\"><nd ref=\"1\"/><nd ref=\"2\"/><nd ref=\"3\"/><nd "
      "ref=\"4\"/><nd ref=\"1\"/><tag k=\"building\" v=\"yes\"/><tag "
      "k=\"height\" v=\"60\"/></way>"
      "<way id=\"2\"><nd ref=\"11\"/><nd ref=\"12\"/><nd ref=\"13\"/><nd "
      "ref=\"14\"/><nd ref=\"11\"/><tag k=\"building\" v=\"yes\"/><tag "
      "k=\"height\" v=\"10\"/></way>"
      "<way id=\"3\"><nd ref=\"1\"/><nd ref=\"11\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>"
      "</osm>";
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  aabb_t tower = mesh_data_bounds(&roof_for(&package, 1)->mesh);
  aabb_t house = mesh_data_bounds(&roof_for(&package, 2)->mesh);
  assert(tower.max.y - tower.min.y > 12.0f); /* a spire, not a cap */
  assert(house.max.y - house.min.y < 6.0f);
  city_package_free(&package);
  city_model_free(&model);
}

static void single_trees_stand_where_they_are_mapped(void) {
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0020\"/>"
      "<node id=\"3\" lat=\"45.0003\" lon=\"12.0010\"><tag k=\"natural\" "
      "v=\"tree\"/></node>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way></osm>";
  osm_data_t osm;
  assert(osm_parse(&osm, xml));
  city_model_t model;
  assert(city_import(&model, &osm, city_import_default_options(), "inline"));
  osm_free(&osm);
  city_generator_options_t options = city_generator_default_options();
  snprintf(options.airport_side, sizeof(options.airport_side), "-");
  city_package_t package;
  city_generator_stats_t stats;
  assert(city_generate(&package, &model, &options, "tree", &stats));
  assert(stats.trees == 1);
  const city_part_t* trunk = part_for(&package, "trunk", 0);
  assert(trunk && trunk->collider == PART_COLLIDER_MESH);
  vec3_t where = aabb_center(mesh_data_bounds(&trunk->mesh));
  assert(near_to(where.x, model.trees[0].x, 0.3) &&
         near_to(where.z, model.trees[0].z, 0.3));
  city_package_free(&package);
  city_model_free(&model);
}

static void no_tree_grows_out_of_a_street(void) {
  /* A road through a park, a mapped tree on its middle and another on the
   * pavement just beside it. */
  const char* xml =
      "<osm><bounds minlat=\"44.998\" minlon=\"11.998\" maxlat=\"45.002\" "
      "maxlon=\"12.004\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0040\"/>"
      "<node id=\"3\" lat=\"45.0000\" lon=\"12.0010\"><tag k=\"natural\" "
      "v=\"tree\"/></node>"
      "<node id=\"4\" lat=\"45.00005\" lon=\"12.0020\"><tag k=\"natural\" "
      "v=\"tree\"/></node>"
      "<node id=\"5\" lat=\"44.9992\" lon=\"12.0002\"/>"
      "<node id=\"6\" lat=\"44.9992\" lon=\"12.0038\"/>"
      "<node id=\"7\" lat=\"45.0008\" lon=\"12.0038\"/>"
      "<node id=\"8\" lat=\"45.0008\" lon=\"12.0002\"/>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>"
      "<way id=\"20\"><nd ref=\"5\"/><nd ref=\"6\"/><nd ref=\"7\"/><nd "
      "ref=\"8\"/><nd ref=\"5\"/><tag k=\"leisure\" v=\"park\"/></way></osm>";
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  float half = model.roads[0].width * 0.5f;
  size_t trunks = 0;
  for (size_t c = 0; c < package.chunk_count; c++)
    for (size_t p = 0; p < package.chunks[c].part_count; p++) {
      const city_part_t* part = &package.chunks[c].parts[p];
      if (strcmp(part->material, "trunk") != 0) continue;
      for (size_t v = 0; v < part->mesh.vertex_count; v++)
        assert(fabsf(part->mesh.vertices[v].position.z) > half);
      trunks += part->mesh.vertex_count > 0;
    }
  assert(trunks >= 1); /* the park still has its trees */
  city_package_free(&package);
  city_model_free(&model);
}

static double seg_distance(double px, double pz, double ax, double az,
                           double bx, double bz) {
  double dx = bx - ax, dz = bz - az, l2 = dx * dx + dz * dz;
  double u = l2 > 1e-12 ? ((px - ax) * dx + (pz - az) * dz) / l2 : 0;
  u = u < 0 ? 0 : (u > 1 ? 1 : u);
  return hypot(px - (ax + u * dx), pz - (az + u * dz));
}

static void the_package_carries_routes_and_parking_places(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  /* One route per road that people or cars use: everything but the stream. */
  assert(package.route_count == stats.roads - 1);
  size_t drivable = 0;
  for (size_t i = 0; i < package.route_count; i++) {
    const city_route_t* route = &package.routes[i];
    assert(route->point_count >= 2 && route->width > 0);
    if (route->drivable) drivable++;
    assert(route->start_node != 0 && route->end_node != 0);
  }
  assert(drivable >= 10);
  /* Parking places lie along drivable streets at the kerb, clear of the
   * junctions, and the package keeps them. */
  assert(package.parked_count > 0);
  for (size_t i = 0; i < package.parked_count; i++) {
    const city_parking_t* spot = &package.parked[i];
    double nearest = 1e9, nearest_end = 1e9;
    for (size_t r = 0; r < package.route_count; r++) {
      const city_route_t* route = &package.routes[r];
      if (!route->drivable) continue;
      for (size_t k = 1; k < route->point_count; k++) {
        double d = seg_distance(spot->x, spot->z, route->points[2 * (k - 1)],
                                route->points[2 * (k - 1) + 1],
                                route->points[2 * k], route->points[2 * k + 1]);
        if (d < nearest) nearest = d;
      }
      local_point_t first = {route->points[0], route->points[1]};
      local_point_t last = {route->points[2 * (route->point_count - 1)],
                            route->points[2 * (route->point_count - 1) + 1]};
      nearest_end =
          fmin(nearest_end, fmin(hypot(spot->x - first.x, spot->z - first.z),
                                 hypot(spot->x - last.x, spot->z - last.z)));
    }
    assert(nearest > 1.0 && nearest < 4.0);
    assert(nearest_end > 8.0);
  }
  const char* directory = "/tmp/gta-like-test-routes";
  assert(city_package_write(&package, directory));
  city_package_t loaded;
  assert(city_package_read(&loaded, directory));
  assert(loaded.route_count == package.route_count);
  assert(loaded.parked_count == package.parked_count);
  assert(near_to(loaded.parked[0].yaw, package.parked[0].yaw, 1e-3f));
  assert(loaded.routes[0].point_count == package.routes[0].point_count);
  city_package_free(&loaded);
  city_package_free(&package);
}

static void the_spawn_can_be_set_to_a_place_in_a_larger_extract(void) {
  /* Two towns a kilometre apart: the spawn goes where the settings say. */
  const char* xml =
      "<osm><bounds minlat=\"44.995\" minlon=\"11.995\" maxlat=\"45.005\" "
      "maxlon=\"12.015\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"11.9990\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"3\" lat=\"45.0000\" lon=\"12.0120\"/>"
      "<node id=\"4\" lat=\"45.0000\" lon=\"12.0130\"/>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>"
      "<way id=\"11\"><nd ref=\"3\"/><nd ref=\"4\"/><tag k=\"highway\" "
      "v=\"residential\"/></way></osm>";
  osm_data_t osm;
  assert(osm_parse(&osm, xml));
  city_model_t model;
  assert(city_import(&model, &osm, city_import_default_options(), "inline"));
  osm_free(&osm);
  city_generator_options_t options = city_generator_default_options();
  snprintf(options.airport_side, sizeof(options.airport_side), "-");
  options.spawn_latitude = 45.0000;
  options.spawn_longitude = 12.0125; /* the eastern road */
  city_package_t package;
  city_generator_stats_t stats;
  assert(city_generate(&package, &model, &options, "two", &stats));
  local_point_t east = projection_project(&model.projection, 45.0, 12.0125);
  assert(fabs(package.spawn_position.x - east.x) < 60.0);
  options.spawn_longitude = 11.9995; /* the western one */
  city_package_free(&package);
  assert(city_generate(&package, &model, &options, "two", &stats));
  local_point_t west = projection_project(&model.projection, 45.0, 11.9995);
  assert(fabs(package.spawn_position.x - west.x) < 60.0);
  city_package_free(&package);
  city_model_free(&model);
}

static void the_town_hall_and_churches_get_towers_and_the_hall_is_brick(void) {
  /* A 40 x 20 m "Municipio" and a 12 x 30 m "Chiesa", each 14 m high, and a
   * plain house of the same size as the hall. */
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.00051\"/>"
      "<node id=\"3\" lat=\"45.00018\" lon=\"12.00051\"/>"
      "<node id=\"4\" lat=\"45.00018\" lon=\"12.0000\"/>"
      "<node id=\"11\" lat=\"45.0006\" lon=\"12.0000\"/>"
      "<node id=\"12\" lat=\"45.0006\" lon=\"12.00015\"/>"
      "<node id=\"13\" lat=\"45.00087\" lon=\"12.00015\"/>"
      "<node id=\"14\" lat=\"45.00087\" lon=\"12.0000\"/>"
      "<node id=\"21\" lat=\"45.0012\" lon=\"12.0000\"/>"
      "<node id=\"22\" lat=\"45.0012\" lon=\"12.00051\"/>"
      "<node id=\"23\" lat=\"45.00138\" lon=\"12.00051\"/>"
      "<node id=\"24\" lat=\"45.00138\" lon=\"12.0000\"/>"
      "<way id=\"1\"><nd ref=\"1\"/><nd ref=\"2\"/><nd ref=\"3\"/><nd "
      "ref=\"4\"/><nd ref=\"1\"/><tag k=\"building\" v=\"yes\"/><tag "
      "k=\"height\" v=\"14\"/><tag k=\"name\" v=\"Municipio di Test\"/></way>"
      "<way id=\"2\"><nd ref=\"11\"/><nd ref=\"12\"/><nd ref=\"13\"/><nd "
      "ref=\"14\"/><nd ref=\"11\"/><tag k=\"building\" v=\"yes\"/><tag "
      "k=\"height\" v=\"14\"/><tag k=\"name\" v=\"Chiesa di Test\"/></way>"
      "<way id=\"3\"><nd ref=\"21\"/><nd ref=\"22\"/><nd ref=\"23\"/><nd "
      "ref=\"24\"/><nd ref=\"21\"/><tag k=\"building\" v=\"yes\"/><tag "
      "k=\"height\" v=\"14\"/></way>"
      "<way id=\"9\"><nd ref=\"1\"/><nd ref=\"21\"/><tag k=\"highway\" "
      "v=\"residential\"/></way></osm>";
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  const city_part_t* hall = facade_for(&package, 1);
  assert(hall && strcmp(hall->material, "facade_brick") == 0);
  const city_part_t* hall_roof = roof_for(&package, 1);
  float hall_top = fmaxf(mesh_data_bounds(&hall->mesh).max.y,
                         mesh_data_bounds(&hall_roof->mesh).max.y);
  assert(hall_top > 14.0f + 14.0f); /* a clock tower and its spire */
  const city_part_t* clock = part_for(&package, "lamp_light", 1);
  assert(clock && mesh_data_triangle_count(&clock->mesh) == 8); /* 4 faces */
  const city_part_t* church = facade_for(&package, 2);
  const city_part_t* church_roof = roof_for(&package, 2);
  assert(strcmp(church->material, "facade_brick") != 0);
  float church_top = fmaxf(mesh_data_bounds(&church->mesh).max.y,
                           mesh_data_bounds(&church_roof->mesh).max.y);
  assert(church_top > 28.0f); /* a campanile */
  const city_part_t* house = facade_for(&package, 3);
  const city_part_t* house_roof = roof_for(&package, 3);
  assert(strcmp(house->material, "facade_brick") != 0);
  assert(mesh_data_bounds(&house_roof->mesh).max.y < 14.0f + 6.0f);
  assert(part_for(&package, "lamp_light", 3) == NULL);
  city_package_free(&package);
  city_model_free(&model);
}

static void buildings_on_main_streets_get_shopfronts_at_street_level(void) {
  /* A secondary road with eight 12 x 12 m buildings along its north side at
   * 5 m from its edge, and one house 200 m off. */
  char xml[8192];
  int length = snprintf(
      xml, sizeof(xml),
      "<osm><bounds minlat=\"44.995\" minlon=\"11.999\" maxlat=\"45.003\" "
      "maxlon=\"12.004\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0030\"/>"
      "<way id=\"9\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"secondary\"/></way>");
  for (int i = 0; i < 8; i++) {
    double lon0 = 12.0002 + i * 0.00032, lon1 = lon0 + 0.00017;
    length += snprintf(
        xml + length, sizeof(xml) - (size_t)length,
        "<node id=\"%d\" lat=\"45.00008\" lon=\"%.6f\"/>"
        "<node id=\"%d\" lat=\"45.00008\" lon=\"%.6f\"/>"
        "<node id=\"%d\" lat=\"45.00020\" lon=\"%.6f\"/>"
        "<node id=\"%d\" lat=\"45.00020\" lon=\"%.6f\"/>"
        "<way id=\"%d\"><nd ref=\"%d\"/><nd ref=\"%d\"/><nd ref=\"%d\"/><nd "
        "ref=\"%d\"/><nd ref=\"%d\"/><tag k=\"building\" v=\"yes\"/><tag "
        "k=\"height\" v=\"13\"/></way>",
        10 + 4 * i, lon0, 11 + 4 * i, lon1, 12 + 4 * i, lon1, 13 + 4 * i, lon0,
        100 + i, 10 + 4 * i, 11 + 4 * i, 12 + 4 * i, 13 + 4 * i, 10 + 4 * i);
  }
  length += snprintf(
      xml + length, sizeof(xml) - (size_t)length,
      "<node id=\"90\" lat=\"44.9970\" lon=\"12.0002\"/>"
      "<node id=\"91\" lat=\"44.9970\" lon=\"12.00037\"/>"
      "<node id=\"92\" lat=\"44.99712\" lon=\"12.00037\"/>"
      "<node id=\"93\" lat=\"44.99712\" lon=\"12.0002\"/>"
      "<way id=\"200\"><nd ref=\"90\"/><nd ref=\"91\"/><nd ref=\"92\"/><nd "
      "ref=\"93\"/><nd ref=\"90\"/><tag k=\"building\" v=\"yes\"/><tag "
      "k=\"height\" v=\"9\"/></way></osm>");
  assert((size_t)length < sizeof(xml));
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  int shops = 0;
  for (int i = 0; i < 8; i++) {
    const city_part_t* shop = part_for(&package, "shopfront", 100 + i);
    if (!shop) continue;
    shops++;
    aabb_t band = mesh_data_bounds(&shop->mesh);
    assert(near_to(band.max.y, 3.2f, 0.01f) && band.min.y <= 0.0f);
    assert(shop->collider == PART_COLLIDER_MESH);
    /* The plain walls start where the shop window ends. */
    const city_part_t* walls = facade_for(&package, 100 + i);
    assert(walls &&
           near_to(mesh_data_bounds(&walls->mesh).max.y, 13.0f, 0.01f));
    /* The sides without a shop still reach the ground. */
    assert(mesh_data_bounds(&walls->mesh).min.y <= 0.0f);
  }
  assert(shops >= 3 && shops <= 7); /* most, not all */
  assert(part_for(&package, "shopfront", 200) ==
         NULL); /* the house, far from a street */
  city_package_free(&package);
  city_model_free(&model);
}

static void some_buildings_on_side_streets_have_a_shop_too(void) {
  char xml[16384];
  int length = snprintf(
      xml, sizeof(xml),
      "<osm><bounds minlat=\"44.995\" minlon=\"11.999\" maxlat=\"45.003\" "
      "maxlon=\"12.004\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0060\"/>"
      "<way id=\"9\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>");
  const int houses = 24;
  for (int i = 0; i < houses; i++) {
    double lon0 = 12.0002 + i * 0.00022, lon1 = lon0 + 0.00017;
    length += snprintf(
        xml + length, sizeof(xml) - (size_t)length,
        "<node id=\"%d\" lat=\"45.00008\" lon=\"%.6f\"/>"
        "<node id=\"%d\" lat=\"45.00008\" lon=\"%.6f\"/>"
        "<node id=\"%d\" lat=\"45.00020\" lon=\"%.6f\"/>"
        "<node id=\"%d\" lat=\"45.00020\" lon=\"%.6f\"/>"
        "<way id=\"%d\"><nd ref=\"%d\"/><nd ref=\"%d\"/><nd ref=\"%d\"/><nd "
        "ref=\"%d\"/><nd ref=\"%d\"/><tag k=\"building\" v=\"yes\"/><tag "
        "k=\"height\" v=\"9\"/></way>",
        10 + 4 * i, lon0, 11 + 4 * i, lon1, 12 + 4 * i, lon1, 13 + 4 * i, lon0,
        100 + i, 10 + 4 * i, 11 + 4 * i, 12 + 4 * i, 13 + 4 * i, 10 + 4 * i);
  }
  length += snprintf(xml + length, sizeof(xml) - (size_t)length, "</osm>");
  assert((size_t)length < sizeof(xml));
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  int shops = 0;
  for (int i = 0; i < houses; i++)
    shops += part_for(&package, "shopfront", 100 + i) != NULL;
  /* A quiet street has fewer shops than a main one, but some. */
  assert(shops >= 2 && shops <= houses / 2 + 2);
  city_package_free(&package);
  city_model_free(&model);
}

static void side_streets_get_a_stop_sign_at_the_main_road(void) {
  /* A primary road through node 2 and a residential street leaving it
   * south: only the side street gets a sign. */
  const char* xml =
      "<osm><bounds minlat=\"44.998\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0012\"/>"
      "<node id=\"3\" lat=\"45.0000\" lon=\"12.0024\"/>"
      "<node id=\"4\" lat=\"44.9990\" lon=\"12.0012\"/>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><nd ref=\"3\"/><tag "
      "k=\"highway\" v=\"primary\"/></way>"
      "<way id=\"11\"><nd ref=\"2\"/><nd ref=\"4\"/><tag k=\"highway\" "
      "v=\"residential\"/></way></osm>";
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  const city_part_t* plate = part_for(&package, "sign_red", 2);
  const city_part_t* pole = part_for(&package, "sign_pole", 2);
  assert(plate && pole);
  assert(mesh_data_triangle_count(&plate->mesh) == 12); /* one sign */
  assert(mesh_data_triangle_count(&pole->mesh) == 12);
  /* It stands just short of the junction, on the side street's right: the
   * junction is at the origin of the street, 110 m south of the road's
   * start, and the street runs south, so the plate is south of the
   * junction and, for a car heading north, to the east. */
  vec3_t where = aabb_center(mesh_data_bounds(&plate->mesh));
  const city_junction_t* junction = NULL;
  for (size_t j = 0; j < model.junction_count; j++)
    if (model.junctions[j].node_id == 2) junction = &model.junctions[j];
  assert(junction);
  float dx = where.x - (float)junction->position.x;
  float dz = where.z - (float)junction->position.z;
  assert(dz > 4.0f && dz < 14.0f && dx > 1.5f && dx < 6.0f);
  assert(where.y > 1.7f && where.y < 2.4f);
  city_package_free(&package);
  city_model_free(&model);
}

static void roads_have_manhole_covers_here_and_there(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  size_t covers = 0;
  for (size_t c = 0; c < package.chunk_count; c++)
    for (size_t p = 0; p < package.chunks[c].part_count; p++)
      if (strcmp(package.chunks[c].parts[p].material, "manhole") == 0) {
        covers++;
        const city_part_t* part = &package.chunks[c].parts[p];
        assert(part->collider == PART_COLLIDER_NONE);
        aabb_t b = mesh_data_bounds(&part->mesh);
        assert(b.min.y > 0.06f && b.max.y < 0.09f); /* just above the road */
      }
  assert(covers >= 3);
  city_package_free(&package);
}

static void benches_bus_stops_bollards_and_signals_stand_where_mapped(void) {
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0030\"/>"
      "<node id=\"3\" lat=\"45.0001\" lon=\"12.0005\"><tag k=\"amenity\" "
      "v=\"bench\"/></node>"
      "<node id=\"4\" lat=\"45.0001\" lon=\"12.0010\"><tag k=\"highway\" "
      "v=\"bus_stop\"/></node>"
      "<node id=\"5\" lat=\"45.0001\" lon=\"12.0015\"><tag k=\"barrier\" "
      "v=\"bollard\"/></node>"
      "<node id=\"6\" lat=\"45.0001\" lon=\"12.0020\"><tag k=\"highway\" "
      "v=\"traffic_signals\"/></node>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way></osm>";
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  /* Every item is a small solid thing standing near where it was mapped. */
  size_t found = 0;
  for (size_t f = 0; f < model.furniture_count; f++) {
    float x = (float)model.furniture[f].x, z = (float)model.furniture[f].z;
    bool near = false;
    for (size_t c = 0; c < package.chunk_count; c++)
      for (size_t p = 0; p < package.chunks[c].part_count; p++) {
        const city_part_t* part = &package.chunks[c].parts[p];
        if (strcmp(part->material, "furniture") != 0) continue;
        assert(part->collider == PART_COLLIDER_MESH);
        aabb_t bounds = mesh_data_bounds(&part->mesh);
        assert(bounds.max.y < 4.5f && bounds.min.y >= -0.01f);
        for (size_t v = 0; v < part->mesh.vertex_count && !near; v++) {
          vec3_t q = part->mesh.vertices[v].position;
          near = fabsf(q.x - x) < 2.0f && fabsf(q.z - z) < 2.0f;
        }
      }
    if (near) found++;
  }
  assert(found >= 3); /* bench, bus stop, bollard (the signal is a pole too) */
  /* A signal head shows its three lamps, and the model also keeps where it is.
   */
  assert(part_for(&package, "signal_red", 0) != NULL);
  assert(part_for(&package, "signal_amber", 0) != NULL);
  assert(part_for(&package, "signal_green", 0) != NULL);
  assert(package.signal_count == 2); /* one for each way along the road */
  assert(package.signals[0].group == package.signals[1].group);
  assert(near_to(package.signals[0].heading_x, -package.signals[1].heading_x,
                 1e-3));
  city_package_free(&package);
  city_model_free(&model);
}

static void walls_stand_where_mapped_but_never_across_a_street(void) {
  /* A street along z = 0, a wall running north-south across it, and a
   * fence well to one side. */
  const char* xml =
      "<osm><bounds minlat=\"44.998\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0030\"/>"
      "<node id=\"3\" lat=\"45.0004\" lon=\"12.0010\"/>"
      "<node id=\"4\" lat=\"44.9996\" lon=\"12.0010\"/>"
      "<node id=\"5\" lat=\"45.0010\" lon=\"12.0020\"/>"
      "<node id=\"6\" lat=\"45.0010\" lon=\"12.0026\"/>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>"
      "<way id=\"11\"><nd ref=\"3\"/><nd ref=\"4\"/><tag k=\"barrier\" "
      "v=\"wall\"/></way>"
      "<way id=\"12\"><nd ref=\"5\"/><nd ref=\"6\"/><tag k=\"barrier\" "
      "v=\"fence\"/></way></osm>";
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  const city_part_t* fence = part_for(&package, "fence", 12);
  assert(part_for(&package, "wall", 11) != NULL && fence != NULL);
  /* The wall is there on both sides of the street, in whichever chunks it
   * crosses, and none of it lies on the carriageway. */
  bool north = false, south = false;
  float half = model.roads[0].width * 0.5f, top = 0;
  for (size_t c = 0; c < package.chunk_count; c++)
    for (size_t p = 0; p < package.chunks[c].part_count; p++) {
      const city_part_t* wall = &package.chunks[c].parts[p];
      if (strcmp(wall->material, "wall") != 0 || wall->source_id != 11)
        continue;
      assert(wall->collider == PART_COLLIDER_MESH);
      top = fmaxf(top, mesh_data_bounds(&wall->mesh).max.y);
      for (size_t v = 0; v < wall->mesh.vertex_count; v++) {
        float z = wall->mesh.vertices[v].position.z;
        assert(fabsf(z) > half);
        if (z < 0) north = true;
        if (z > 0) south = true;
      }
    }
  assert(north && south);
  assert(top > 1.5f && top < 2.5f);
  assert(mesh_data_bounds(&fence->mesh).max.y < top);
  city_package_free(&package);
  city_model_free(&model);
}

static city_package_t cross_of(const char* highway, city_model_t* model) {
  char xml[2048];
  snprintf(xml, sizeof(xml),
           "<osm><bounds minlat=\"44.998\" minlon=\"11.998\" "
           "maxlat=\"45.002\" maxlon=\"12.002\"/>"
           "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
           "<node id=\"2\" lat=\"45.0000\" lon=\"12.0015\"/>"
           "<node id=\"3\" lat=\"45.0000\" lon=\"11.9985\"/>"
           "<node id=\"4\" lat=\"45.0010\" lon=\"12.0000\"/>"
           "<node id=\"5\" lat=\"44.9990\" lon=\"12.0000\"/>"
           "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
           "v=\"%s\"/></way>"
           "<way id=\"11\"><nd ref=\"1\"/><nd ref=\"3\"/><tag k=\"highway\" "
           "v=\"%s\"/></way>"
           "<way id=\"12\"><nd ref=\"1\"/><nd ref=\"4\"/><tag k=\"highway\" "
           "v=\"%s\"/></way>"
           "<way id=\"13\"><nd ref=\"1\"/><nd ref=\"5\"/><tag k=\"highway\" "
           "v=\"%s\"/></way></osm>",
           highway, highway, highway, highway);
  return generate_inline(xml, model);
}

static void busy_crossings_get_traffic_lights_in_two_phases(void) {
  city_model_t model;
  city_package_t package = cross_of("tertiary", &model);
  /* One head at the entry of each of the four streets, east-west streets in
   * group 0 and north-south in group 1, each governing traffic heading into
   * the junction. */
  assert(package.signal_count == 4);
  int groups[2] = {0, 0};
  for (size_t i = 0; i < package.signal_count; i++) {
    const city_signal_t* signal = &package.signals[i];
    groups[signal->group]++;
    /* Heading points at the junction (the origin) from where it stands. */
    float to_x = -signal->x, to_z = -signal->z;
    float length = hypotf(to_x, to_z);
    assert(length > 3.0f && length < 25.0f);
    assert((to_x * signal->heading_x + to_z * signal->heading_z) / length >
           0.6f);
    assert(near_to(signal->heading_x * signal->heading_x +
                       signal->heading_z * signal->heading_z,
                   1.0, 1e-3));
    /* A head on the east-west street governs east-west traffic. */
    bool east_west = fabsf(signal->heading_x) > fabsf(signal->heading_z);
    assert(signal->group == (east_west ? 0 : 1));
  }
  assert(groups[0] == 2 && groups[1] == 2);
  /* Heads are built with lenses for both phases. */
  assert(part_for(&package, "signal_red",
                  (uint64_t)model.junctions[0].node_id) != NULL ||
         part_for(&package, "signal_r2",
                  (uint64_t)model.junctions[0].node_id) != NULL);
  city_package_free(&package);
  city_model_free(&model);
  /* Quiet residential streets keep the stop signs and get no lights. */
  package = cross_of("residential", &model);
  assert(package.signal_count == 0);
  city_package_free(&package);
  city_model_free(&model);
}

static void street_facing_roofs_get_a_stone_cornice(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  size_t cornices = 0;
  for (size_t c = 0; c < package.chunk_count; c++)
    for (size_t p = 0; p < package.chunks[c].part_count; p++) {
      const city_part_t* part = &package.chunks[c].parts[p];
      if (strcmp(part->material, "cornice") != 0) continue;
      cornices++;
      assert(part->collider == PART_COLLIDER_NONE);
      aabb_t b = mesh_data_bounds(&part->mesh);
      assert(near_to(b.max.y - b.min.y, 0.45, 0.01)); /* a band */
      /* It sits at the top of its building's walls. */
      const city_part_t* walls = facade_for(&package, part->source_id);
      assert(walls);
      assert(near_to(mesh_data_bounds(&walls->mesh).max.y, b.max.y, 0.01) ||
             mesh_data_bounds(&walls->mesh).max.y >= b.max.y - 0.01);
    }
  assert(cornices >= 1 && cornices <= stats.buildings);
  city_package_free(&package);
}

static void nothing_is_lifted_twice_on_raised_ground(void) {
  /* Ground at 100 m everywhere: every part of the town, including what is
   * attached to buildings, must sit within the tallest building's height
   * above it, not 100 m above that. */
  city_model_t model = fixture_model();
  float heights[4] = {100, 100, 100, 100};
  model.terrain.min_x = -1000, model.terrain.min_z = -1000;
  model.terrain.dx = 2000, model.terrain.dz = 2000;
  model.terrain.cols = 2, model.terrain.rows = 2;
  model.terrain.heights = malloc(sizeof(heights));
  memcpy(model.terrain.heights, heights, sizeof(heights));
  city_generator_options_t options = city_generator_default_options();
  snprintf(options.airport_side, sizeof(options.airport_side), "-");
  city_package_t package;
  city_generator_stats_t stats;
  assert(city_generate(&package, &model, &options, "raised", &stats));
  size_t attached = 0;
  for (size_t c = 0; c < package.chunk_count; c++)
    for (size_t p = 0; p < package.chunks[c].part_count; p++) {
      const city_part_t* part = &package.chunks[c].parts[p];
      if (part->mesh.vertex_count == 0) continue;
      aabb_t b = mesh_data_bounds(&part->mesh);
      bool ground = strcmp(part->material, "ground") == 0;
      /* Walls may reach a few metres below, on a slope. */
      if (!ground) assert(b.min.y > 94.0f && b.max.y < 160.0f);
      if (strcmp(part->material, "door") == 0 ||
          strcmp(part->material, "chimney") == 0 ||
          strcmp(part->material, "cornice") == 0 ||
          strcmp(part->material, "shopfront") == 0)
        attached++;
    }
  assert(attached >= 2);
  city_package_free(&package);
  city_model_free(&model);
}

static void a_package_with_crossings_has_no_empty_parts(void) {
  /* Crossing nodes that are not on a street cars use make nothing at all. */
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0020\"/>"
      "<node id=\"3\" lat=\"45.0010\" lon=\"12.0010\"><tag k=\"highway\" "
      "v=\"crossing\"/></node>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way></osm>";
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  city_validation_t report;
  assert(city_validate(&package, &report) && report.issues == 0);
  city_package_free(&package);
  city_model_free(&model);
}

static void mapped_crossings_get_zebra_stripes_away_from_junctions(void) {
  /* A 6 m street with a crossing node at its middle and another 4 m from
   * its end, where the junction already has its own. */
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.004\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0012\"><tag k=\"highway\" "
      "v=\"crossing\"/></node>"
      "<node id=\"3\" lat=\"45.0000\" lon=\"12.0024\"/>"
      "<node id=\"4\" lat=\"45.0000\" lon=\"12.0022\"><tag k=\"highway\" "
      "v=\"crossing\"/></node>"
      "<node id=\"5\" lat=\"44.9992\" lon=\"12.0024\"/>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><nd ref=\"4\"/><nd "
      "ref=\"3\"/><tag k=\"highway\" v=\"residential\"/></way>"
      "<way id=\"11\"><nd ref=\"3\"/><nd ref=\"5\"/><tag k=\"highway\" "
      "v=\"residential\"/></way></osm>";
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  const city_part_t* zebra = part_for(&package, "marking", 0);
  assert(zebra);
  aabb_t bounds = mesh_data_bounds(&zebra->mesh);
  /* Only the one in the middle: stripes across the 6 m street, 2.4 m long. */
  vec3_t middle = aabb_center(bounds);
  float x_of_crossing = 0;
  for (size_t i = 0; i < model.road_count; i++)
    for (size_t k = 0; k < model.roads[i].point_count; k++)
      if (model.roads[i].node_ids[k] == 2)
        x_of_crossing = (float)model.roads[i].points[k].x;
  assert(near_to(middle.x, x_of_crossing, 0.3f));
  assert(near_to(bounds.max.x - bounds.min.x, 2.4f, 0.05f));
  assert(bounds.max.z - bounds.min.z > 3.5f &&
         bounds.max.z - bounds.min.z < 6.0f);
  assert(mesh_data_triangle_count(&zebra->mesh) >= 8);
  city_package_free(&package);
  city_model_free(&model);
}

static void pitched_roofs_sprout_chimneys_above_the_ridge(void) {
  /* Twelve 20 x 10 m houses in a row, 9 m high. */
  char xml[8192];
  int length = snprintf(
      xml, sizeof(xml),
      "<osm><bounds minlat=\"44.998\" minlon=\"11.999\" maxlat=\"45.003\" "
      "maxlon=\"12.006\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0050\"/>"
      "<way id=\"9\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>");
  for (int i = 0; i < 12; i++) {
    double lon0 = 12.0001 + i * 0.0004, lon1 = lon0 + 0.00026;
    length += snprintf(
        xml + length, sizeof(xml) - (size_t)length,
        "<node id=\"%d\" lat=\"45.0004\" lon=\"%.6f\"/>"
        "<node id=\"%d\" lat=\"45.0004\" lon=\"%.6f\"/>"
        "<node id=\"%d\" lat=\"45.00049\" lon=\"%.6f\"/>"
        "<node id=\"%d\" lat=\"45.00049\" lon=\"%.6f\"/>"
        "<way id=\"%d\"><nd ref=\"%d\"/><nd ref=\"%d\"/><nd ref=\"%d\"/><nd "
        "ref=\"%d\"/><nd ref=\"%d\"/><tag k=\"building\" v=\"yes\"/><tag "
        "k=\"height\" v=\"9\"/></way>",
        10 + 4 * i, lon0, 11 + 4 * i, lon1, 12 + 4 * i, lon1, 13 + 4 * i, lon0,
        100 + i, 10 + 4 * i, 11 + 4 * i, 12 + 4 * i, 13 + 4 * i, 10 + 4 * i);
  }
  length += snprintf(xml + length, sizeof(xml) - (size_t)length, "</osm>");
  assert((size_t)length < sizeof(xml));
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  int chimneys = 0;
  for (int i = 0; i < 12; i++) {
    const city_part_t* chimney = part_for(&package, "chimney", 100 + i);
    if (!chimney) continue;
    chimneys++;
    aabb_t b = mesh_data_bounds(&chimney->mesh);
    const city_part_t* roof = roof_for(&package, 100 + i);
    float ridge = mesh_data_bounds(&roof->mesh).max.y;
    assert(b.max.y > ridge + 0.5f &&
           b.max.y < ridge + 2.0f);            /* stands clear above */
    assert(b.min.y < ridge && b.min.y > 8.5f); /* rooted in the roof */
    assert(b.max.x - b.min.x < 0.8f && b.max.z - b.min.z < 0.8f);
  }
  assert(chimneys >= 3 && chimneys <= 9); /* many, not all */
  city_package_free(&package);
  city_model_free(&model);
}

static void a_footbridge_rises_over_what_it_crosses_with_ramps_and_railings(
    void) {
  /* A 120 m footway tagged bridge=yes, layer=1, over a stream-side street
   * that runs across its middle. */
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0017\"/>"
      "<node id=\"3\" lat=\"45.0008\" lon=\"12.0009\"/>"
      "<node id=\"4\" lat=\"44.9992\" lon=\"12.0009\"/>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"footway\"/><tag k=\"bridge\" v=\"yes\"/><tag k=\"layer\" "
      "v=\"1\"/></way>"
      "<way id=\"11\"><nd ref=\"3\"/><nd ref=\"4\"/><tag k=\"highway\" "
      "v=\"residential\"/></way></osm>";
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  const city_part_t* deck = part_for(&package, "paving", 10);
  assert(deck && deck->collider == PART_COLLIDER_MESH);
  float lowest = 1e9f, highest = -1e9f, at_ends = 1e9f, at_middle = 0;
  float x_min = 1e9f, x_max = -1e9f;
  for (size_t v = 0; v < deck->mesh.vertex_count; v++) {
    vec3_t p = deck->mesh.vertices[v].position;
    x_min = fminf(x_min, p.x), x_max = fmaxf(x_max, p.x);
  }
  float x_mid = 0.5f * (x_min + x_max);
  for (size_t v = 0; v < deck->mesh.vertex_count; v++) {
    vec3_t p = deck->mesh.vertices[v].position;
    lowest = fminf(lowest, p.y), highest = fmaxf(highest, p.y);
    if (fabsf(p.x - x_mid) < 3.0f) at_middle = fmaxf(at_middle, p.y);
    if (p.x < x_min + 0.5f) at_ends = fminf(at_ends, p.y);
  }
  assert(lowest < 0.2f); /* it starts on the ground */
  assert(at_ends < 0.3f);
  assert(at_middle > 4.0f &&
         highest < 5.2f); /* and clears a lorry in the middle */
  /* Over the stretch crossing the street the deck is the better part of 4 m up.
   */
  assert(part_for(&package, "rail", 10) != NULL);
  const city_part_t* rail = part_for(&package, "rail", 10);
  assert(rail->collider == PART_COLLIDER_NONE);
  aabb_t rb = mesh_data_bounds(&rail->mesh);
  assert(rb.max.y > at_middle + 0.8f && rb.max.y < at_middle + 1.4f);
  /* A footway that is not a bridge stays flat. */
  city_package_free(&package);
  city_model_free(&model);
}

static void street_facing_walls_get_doors_at_the_pavement(void) {
  /* A 24 x 10 m house 8 m from a residential street, and one 300 m away. */
  const char* xml =
      "<osm><bounds minlat=\"44.995\" minlon=\"11.999\" maxlat=\"45.003\" "
      "maxlon=\"12.005\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0040\"/>"
      "<node id=\"11\" lat=\"45.00010\" lon=\"12.0010\"/>"
      "<node id=\"12\" lat=\"45.00010\" lon=\"12.00131\"/>"
      "<node id=\"13\" lat=\"45.00019\" lon=\"12.00131\"/>"
      "<node id=\"14\" lat=\"45.00019\" lon=\"12.0010\"/>"
      "<node id=\"21\" lat=\"44.9970\" lon=\"12.0010\"/>"
      "<node id=\"22\" lat=\"44.9970\" lon=\"12.00131\"/>"
      "<node id=\"23\" lat=\"44.99709\" lon=\"12.00131\"/>"
      "<node id=\"24\" lat=\"44.99709\" lon=\"12.0010\"/>"
      "<way id=\"9\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>"
      "<way id=\"20\"><nd ref=\"11\"/><nd ref=\"12\"/><nd ref=\"13\"/><nd "
      "ref=\"14\"/><nd ref=\"11\"/><tag k=\"building\" v=\"house\"/><tag "
      "k=\"height\" v=\"7\"/></way>"
      "<way id=\"21\"><nd ref=\"21\"/><nd ref=\"22\"/><nd ref=\"23\"/><nd "
      "ref=\"24\"/><nd ref=\"21\"/><tag k=\"building\" v=\"house\"/><tag "
      "k=\"height\" v=\"7\"/></way></osm>";
  city_model_t model;
  city_package_t package = generate_inline(xml, &model);
  const city_part_t* doors = part_for(&package, "door", 20);
  /* Seven in ten buildings have doors, picked by id; this id is one. */
  assert(doors && mesh_data_triangle_count(&doors->mesh) % 2 == 0);
  {
    aabb_t b = mesh_data_bounds(&doors->mesh);
    assert(b.min.y > -0.01f && b.min.y < 0.3f);
    assert(b.max.y > 2.0f && b.max.y < 2.8f);
    /* The 24 m wall is long enough for two doors, 1.2 m wide each, at
     * about a third and two thirds of its length. */
    assert(mesh_data_triangle_count(&doors->mesh) == 4);
    assert(b.max.x - b.min.x > 8.0f && b.max.x - b.min.x < 12.0f);
    /* On the wall that faces the street (the south wall, z largest). */
    const city_part_t* walls = facade_for(&package, 20);
    float wall_z = mesh_data_bounds(&walls->mesh).max.z;
    assert(fabsf(b.max.z - wall_z) < 0.12f && fabsf(b.min.z - wall_z) < 0.12f);
  }
  assert(part_for(&package, "door", 21) == NULL); /* far from any street */
  city_package_free(&package);
  city_model_free(&model);
}

static void junctions_get_stop_lines_and_zebra_crossings(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  /* The crossroads at node 5: four arms, each with a stop line and a
   * zebra just beyond the arm's cut, painted a little above the road. */
  const city_part_t* marking = part_for(&package, "marking", 5);
  assert(marking && marking->collider == PART_COLLIDER_NONE);
  assert(mesh_data_triangle_count(&marking->mesh) >= 4 * 2 * 4);
  aabb_t bounds = mesh_data_bounds(&marking->mesh);
  assert(bounds.min.y > 0.06f && bounds.max.y < 0.08f);
  /* Nothing is painted more than 16 m from the junction centre. */
  const city_part_t* patch = part_for(&package, "junction", 5);
  assert(patch);
  vec3_t centre = aabb_center(mesh_data_bounds(&patch->mesh));
  for (size_t v = 0; v < marking->mesh.vertex_count; v++) {
    vec3_t d = vec3_sub(marking->mesh.vertices[v].position, centre);
    d.y = 0;
    assert(vec3_length(d) < 16.0f && vec3_length(d) > 3.0f);
  }
  /* A dead end gets none. */
  assert(part_for(&package, "marking", 14) == NULL);
  city_package_free(&package);
}

static void lamp_posts_stand_on_the_pavements(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  assert(stats.lamps > 10);
  size_t lamp_parts = count_material(&package, "lamp");
  /* Every lamp is a pole of its own (a post and arm, and a head apart so it
   * can glow at night), numbered so the game can knock it down. */
  assert(lamp_parts == 2 * stats.lamps); /* posts and heads */
  const city_part_t* heads = part_for(&package, "lamp_light", 103);
  assert(heads && heads->collider == PART_COLLIDER_NONE);
  assert(mesh_data_triangle_count(&heads->mesh) == 12); /* one box */
  assert(city_source_is_pole(heads->source_id));
  const city_part_t* lamps = part_for(&package, "lamp", 103);
  assert(lamps && lamps->collider == PART_COLLIDER_MESH && lamps->layer == 8);
  assert(lamps->source_id == heads->source_id ||
         city_source_original(lamps->source_id) == 103);
  /* Road 103 runs east-west, 6 m wide with 2 m pavements: posts stand
   * about 4 m either side of the centre line. */
  float lowest = 1e9f, highest = -1e9f;
  for (size_t c = 0; c < package.chunk_count; c++)
    for (size_t p = 0; p < package.chunks[c].part_count; p++) {
      const city_part_t* part = &package.chunks[c].parts[p];
      if (strcmp(part->material, "lamp") != 0 ||
          city_source_original(part->source_id) != 103)
        continue;
      aabb_t bounds = mesh_data_bounds(&part->mesh);
      assert(near_to(bounds.min.y, 0.18f, 0.01f)); /* on the pavement */
      assert(bounds.max.y > 4.5f && bounds.max.y < 5.0f);
      lowest = fminf(lowest, bounds.min.z);
      highest = fmaxf(highest, bounds.max.z);
    }
  assert(highest - lowest > 7.0f && highest - lowest < 9.5f);
  city_package_free(&package);
}

static void faces_point_up_or_outward_and_the_spawn_sits_on_a_road(void) {
  city_generator_stats_t stats;
  city_package_t package = generate_fixture(&stats);
  for (size_t c = 0; c < package.chunk_count; c++) {
    for (size_t p = 0; p < package.chunks[c].part_count; p++) {
      const city_part_t* part = &package.chunks[c].parts[p];
      if (strcmp(part->material, "kerb") == 0 ||
          strcmp(part->material, "marking") == 0 ||
          strcmp(part->material, "manhole") == 0 ||
          strcmp(part->material, "door") == 0 ||
          strcmp(part->material, "sign") == 0 ||
          strcmp(part->material, "balcony") == 0 ||
          strcmp(part->material, "cornice") == 0 ||
          strcmp(part->material, "rail") == 0 ||
          strcmp(part->material, "chimney") == 0 ||
          strcmp(part->material, "furniture") == 0 ||
          strcmp(part->material, "signal_red") == 0 ||
          strcmp(part->material, "signal_amber") == 0 ||
          strcmp(part->material, "signal_green") == 0 ||
          strcmp(part->material, "signal_r2") == 0 ||
          strcmp(part->material, "signal_a2") == 0 ||
          strcmp(part->material, "signal_g2") == 0 ||
          strcmp(part->material, "wall") == 0 ||
          strcmp(part->material, "fence") == 0 ||
          strcmp(part->material, "hedge") == 0 ||
          strcmp(part->material, "sign_red") == 0 ||
          strcmp(part->material, "sign_pole") == 0 ||
          strcmp(part->material, "lamp") == 0 ||
          strcmp(part->material, "lamp_light") == 0 ||
          strcmp(part->material, "trunk") == 0 ||
          strcmp(part->material, "leaves") == 0)
        continue;
      bool flat = strncmp(part->material, "facade", 6) != 0 &&
                  strncmp(part->material, "shopfront", 9) != 0 &&
                  strncmp(part->material, "roof", 4) != 0; /* eaves face down */
      for (size_t i = 0; i + 2 < part->mesh.index_count; i += 3) {
        vec3_t a = part->mesh.vertices[part->mesh.indices[i]].position;
        vec3_t b = part->mesh.vertices[part->mesh.indices[i + 1]].position;
        vec3_t d = part->mesh.vertices[part->mesh.indices[i + 2]].position;
        vec3_t n = vec3_cross(vec3_sub(b, a), vec3_sub(d, a));
        if (flat)
          assert(n.y > 0);
        else
          assert(vec3_dot(
                     n, part->mesh.vertices[part->mesh.indices[i]].normal) > 0);
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
  assert(city_package_triangle_count(&loaded) ==
         city_package_triangle_count(&package));
  assert(near_to(loaded.spawn_position.x, package.spawn_position.x, 1e-3));
  assert(loaded.config_hash == package.config_hash);
  /* Named streets become map labels in capitals, longest first, and
   * survive the manifest. The stream is not a street. */
  assert(package.label_count == 3);
  bool main_street = false;
  for (size_t i = 0; i < package.label_count; i++) {
    assert(strcmp(package.labels[i].text, "ROGGIA") != 0);
    if (strcmp(package.labels[i].text, "MAIN STREET") == 0) main_street = true;
  }
  assert(main_street);
  assert(loaded.label_count == package.label_count);
  assert(strcmp(loaded.labels[0].text, package.labels[0].text) == 0);
  assert(near_to(loaded.labels[0].x, package.labels[0].x, 1e-2));
  assert(strcmp(loaded.source, "tests/fixtures/district.osm") == 0);
  city_validation_t report;
  assert(city_validate(&loaded, &report));
  assert(report.roads == 13 && report.buildings == 8 && report.issues == 0);
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
  assert(system("diff -r /tmp/gta-like-test-package-a "
                "/tmp/gta-like-test-package-b > /dev/null") == 0);
  city_package_free(&a);
  city_package_free(&b);
}

static void generator_settings_load_and_hash(void) {
  city_generator_options_t options;
  assert(city_generator_options_load(&options, "assets/city/default.cfg"));
  assert(near(options.chunk_size, 150) && options.pavements);
  city_generator_options_t defaults = city_generator_default_options();
  assert(city_generator_options_hash(&options) ==
         city_generator_options_hash(&defaults));
  options.chunk_size = 100;
  assert(city_generator_options_hash(&options) !=
         city_generator_options_hash(&defaults));
  assert(!city_generator_options_load(&options, "assets/city/missing.cfg"));
}

void run_generator_tests(void) {
  RUN(ear_clipping_handles_convex_and_concave_rings);
  RUN(ear_clipping_cuts_holes_out);
  RUN(courtyards_are_open_to_the_sky);
  RUN(every_road_junction_and_building_becomes_geometry);
  RUN(road_ribbons_have_their_width_and_roofs_their_height);
  RUN(roads_narrow_to_fit_between_buildings);
  RUN(people_streets_paths_and_areas_get_their_own_surfaces);
  RUN(trees_grow_in_parks_and_nowhere_else);
  RUN(parallel_carriageways_share_the_gap_between_them);
  RUN(narrow_terraces_get_gabled_roofs);
  RUN(irregular_buildings_get_ridge_roofs_and_tall_ones_stay_flat);
  RUN(everything_follows_the_terrain);
  RUN(footpaths_stop_at_the_pavement_instead_of_crossing_the_road);
  RUN(buildings_on_a_named_street_get_a_portico);
  RUN(a_slender_tower_gets_a_tall_spire);
  RUN(single_trees_stand_where_they_are_mapped);
  RUN(no_tree_grows_out_of_a_street);
  RUN(the_package_carries_routes_and_parking_places);
  RUN(the_spawn_can_be_set_to_a_place_in_a_larger_extract);
  RUN(the_town_hall_and_churches_get_towers_and_the_hall_is_brick);
  RUN(buildings_on_main_streets_get_shopfronts_at_street_level);
  RUN(some_buildings_on_side_streets_have_a_shop_too);
  RUN(side_streets_get_a_stop_sign_at_the_main_road);
  RUN(roads_have_manhole_covers_here_and_there);
  RUN(benches_bus_stops_bollards_and_signals_stand_where_mapped);
  RUN(walls_stand_where_mapped_but_never_across_a_street);
  RUN(busy_crossings_get_traffic_lights_in_two_phases);
  RUN(street_facing_roofs_get_a_stone_cornice);
  RUN(nothing_is_lifted_twice_on_raised_ground);
  RUN(a_package_with_crossings_has_no_empty_parts);
  RUN(mapped_crossings_get_zebra_stripes_away_from_junctions);
  RUN(pitched_roofs_sprout_chimneys_above_the_ridge);
  RUN(a_footbridge_rises_over_what_it_crosses_with_ramps_and_railings);
  RUN(street_facing_walls_get_doors_at_the_pavement);
  RUN(junctions_get_stop_lines_and_zebra_crossings);
  RUN(lamp_posts_stand_on_the_pavements);
  RUN(faces_point_up_or_outward_and_the_spawn_sits_on_a_road);
  RUN(the_package_round_trips_and_validates);
  RUN(generation_is_deterministic);
  RUN(generator_settings_load_and_hash);
}
