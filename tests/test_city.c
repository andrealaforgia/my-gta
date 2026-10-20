#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "city_import.h"
#include "city_model.h"
#include "osm_reader.h"
#include "test_groups.h"
#include "test_support.h"
#include "triangulate.h"

static city_model_t import_fixture(void) {
  osm_data_t osm;
  assert(osm_load(&osm, "tests/fixtures/district.osm"));
  city_import_options_t options = city_import_default_options();
  city_model_t model;
  assert(city_import(&model, &osm, options, "tests/fixtures/district.osm"));
  osm_free(&osm);
  return model;
}

static const city_road_t* road_from(const city_model_t* model,
                                    int64_t source_id, int index) {
  int seen = 0;
  for (size_t i = 0; i < model->road_count; i++)
    if (model->roads[i].source_id == source_id && seen++ == index)
      return &model->roads[i];
  return NULL;
}

static const city_junction_t* junction(const city_model_t* model,
                                       int64_t node_id) {
  for (size_t i = 0; i < model->junction_count; i++)
    if (model->junctions[i].node_id == node_id) return &model->junctions[i];
  return NULL;
}

static const city_building_t* building(const city_model_t* model,
                                       int64_t source_id) {
  for (size_t i = 0; i < model->building_count; i++)
    if (model->buildings[i].source_id == source_id) return &model->buildings[i];
  return NULL;
}

static void roads_are_split_at_junctions_and_classified(void) {
  city_model_t model = import_fixture();
  /* Six grid ways of two segments each, the curved road, the stub road,
   * the footway, the Corso, the cycleway and the stream make 18 segments;
   * the pair meeting end to end at the n1 corner is joined into one. */
  assert(model.road_count == 17 && model.stats.roads_merged == 1);
  const city_road_t* main_west = road_from(&model, 102, 0);
  assert(main_west && main_west->class == ROAD_CLASS_PRIMARY);
  assert(near(main_west->width, 9.0f) && !main_west->width_inferred);
  assert(main_west->point_count == 2);
  const city_road_t* south = road_from(&model, 101, 0);
  assert(south && south->width_inferred && south->width > 4 &&
         south->width < 8);
  const city_road_t* oneway = road_from(&model, 105, 1);
  assert(oneway && oneway->oneway);
  const city_road_t* curved = road_from(&model, 107, 0);
  assert(curved && curved->point_count > 5 && /* its bends are rounded */
         strcmp(curved->surface, "cobblestone") == 0);
  const city_road_t* footway = road_from(&model, 108, 0);
  assert(footway && footway->class == ROAD_CLASS_PATH && footway->width < 3.0f);
  const city_road_t* corso = road_from(&model, 301, 0);
  assert(corso && corso->class == ROAD_CLASS_PEDESTRIAN);
  assert(road_from(&model, 302, 0)->class == ROAD_CLASS_PATH); /* cycleway */
  assert(road_from(&model, 303, 0) ==
         NULL); /* a sidewalk: the pavements cover it */
  assert(road_from(&model, 304, 0) ==
         NULL); /* a pedestrian area is an area, not a road */
  city_model_free(&model);
}

static void degenerate_points_and_ways_are_cleaned(void) {
  city_model_t model = import_fixture();
  /* The stub ends at node 14; its repeated node collapsed, so no two
   * consecutive points coincide on the road that now carries it. */
  const city_road_t* stub = NULL;
  for (size_t i = 0; i < model.road_count && !stub; i++)
    if (model.roads[i].node_ids[model.roads[i].point_count - 1] == 14 ||
        model.roads[i].node_ids[0] == 14)
      stub = &model.roads[i];
  assert(stub);
  for (size_t i = 1; i < stub->point_count; i++)
    assert(hypot(stub->points[i].x - stub->points[i - 1].x,
                 stub->points[i].z - stub->points[i - 1].z) > 0.1);
  assert(road_from(&model, 110, 0) == NULL); /* zero length dropped */
  assert(model.stats.ways_ignored >= 1);
  assert(model.stats.roads_degenerate == 1);
  city_model_free(&model);
}

static void junctions_carry_their_degree_and_position(void) {
  city_model_t model = import_fixture();
  assert(junction(&model, 5) && junction(&model, 5)->degree == 4);
  assert(junction(&model, 2) && junction(&model, 2)->degree == 3);
  assert(junction(&model, 8) && junction(&model, 8)->degree == 3);
  /* The grid corner at n1 joined two like roads end to end, so it is a
   * bend inside one road now, not a junction. */
  assert(junction(&model, 1) == NULL);
  assert(junction(&model, 3) &&
         junction(&model, 3)->degree == 3); /* corner plus the curved road */
  assert(junction(&model, 13) &&
         junction(&model, 13)->degree == 2); /* unlike roads stay apart */
  assert(junction(&model, 14) && junction(&model, 14)->degree == 1);
  const city_junction_t* centre = junction(&model, 5);
  const local_point_t* corner = NULL;
  for (size_t r = 0; r < model.road_count && !corner; r++)
    for (size_t i = 0; i < model.roads[r].point_count && !corner; i++)
      if (model.roads[r].node_ids[i] == 1) corner = &model.roads[r].points[i];
  assert(corner);
  /* n5 is north-east of n1: larger x, smaller z. */
  assert(centre->position.x > corner->x + 90 &&
         centre->position.z < corner->z - 90);
  city_model_free(&model);
}

static void buildings_get_heights_from_tags_levels_or_defaults(void) {
  city_model_t model = import_fixture();
  assert(model.building_count == 8); /* five ways and three multipolygons */
  const city_building_t* tagged = building(&model, 201);
  assert(tagged && near(tagged->height, 12) &&
         tagged->height_source == HEIGHT_TAGGED);
  const city_building_t* levels = building(&model, 202);
  assert(levels && levels->levels == 4 &&
         levels->height_source == HEIGHT_FROM_LEVELS);
  assert(near_to(levels->height, 4 * 3.2f, 1e-4));
  /* The untagged block has no tagged neighbour within reach, so it keeps
   * the deterministic default. */
  const city_building_t* plain = building(&model, 203);
  assert(plain && plain->height_source == HEIGHT_DEFAULT &&
         plain->height >= 6 && plain->height <= 15);
  assert(plain->point_count == 6);
  assert(building(&model, 204) == NULL);
  assert(model.stats.buildings_malformed == 1);
  const city_building_t* palazzo = building(&model, 401);
  assert(palazzo && near(palazzo->height, 20) && palazzo->point_count == 4);
  const city_building_t* church = building(&model, 402);
  assert(church && church->point_count == 4 &&
         strcmp(church->category, "church") == 0);
  const city_building_t* palazzo_with_courtyard = building(&model, 404);
  assert(palazzo_with_courtyard && palazzo_with_courtyard->hole_count == 1);
  assert(palazzo_with_courtyard->holes[0].count == 4);
  /* The courtyard ring is clockwise, the opposite of the outline. */
  assert(polygon_signed_area(palazzo_with_courtyard->holes[0].points, 4) < 0);
  assert(model.stats.relations_skipped == 0);
  city_model_free(&model);
}

static float signed_area_x_north(const city_building_t* b) {
  float area = 0;
  for (size_t i = 0; i < b->point_count; i++) {
    local_point_t p = b->points[i], q = b->points[(i + 1) % b->point_count];
    area += (float)(p.x * -q.z - q.x * -p.z);
  }
  return area * 0.5f;
}

static void building_rings_are_anticlockwise_seen_from_above(void) {
  city_model_t model = import_fixture();
  for (size_t i = 0; i < model.building_count; i++)
    assert(signed_area_x_north(&model.buildings[i]) > 0);
  assert(building(&model, 205)->point_count == 4);
  city_model_free(&model);
}

static void default_heights_are_stable_for_the_same_source(void) {
  city_model_t a = import_fixture();
  city_model_t b = import_fixture();
  assert(near(building(&a, 203)->height, building(&b, 203)->height));
  city_model_free(&a);
  city_model_free(&b);
}

static const city_area_t* area(const city_model_t* model, int64_t source_id) {
  for (size_t i = 0; i < model->area_count; i++)
    if (model->areas[i].source_id == source_id) return &model->areas[i];
  return NULL;
}

static void areas_come_from_land_use_water_parking_and_squares(void) {
  city_model_t model = import_fixture();
  assert(model.area_count == 5); /* the park is both a way and a relation */
  assert(area(&model, 304) && area(&model, 304)->kind == AREA_SQUARE);
  assert(area(&model, 305) && area(&model, 305)->kind == AREA_WATER);
  assert(area(&model, 306) && area(&model, 306)->kind == AREA_PARKING);
  assert(area(&model, 403) &&
         area(&model, 403)->kind == AREA_PARK); /* a relation */
  for (size_t i = 0; i < model.area_count; i++) {
    assert(model.areas[i].point_count >= 4);
    assert(polygon_signed_area(model.areas[i].points,
                               model.areas[i].point_count) > 0);
  }
  city_model_free(&model);
}

static void dangling_road_ends_snap_onto_the_road_they_nearly_reach(void) {
  /* A north-south street whose last node stops about a metre short of
   * an east-west street, as happens when a mapper forgot to join them. */
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0020\"/>"
      "<node id=\"3\" lat=\"45.0015\" lon=\"12.0010\"/>"
      "<node id=\"4\" lat=\"45.0000100\" lon=\"12.0010\"/>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>"
      "<way id=\"11\"><nd ref=\"3\"/><nd ref=\"4\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>"
      "</osm>";
  osm_data_t osm;
  assert(osm_parse(&osm, xml));
  city_model_t model;
  assert(city_import(&model, &osm, city_import_default_options(), "inline"));
  osm_free(&osm);
  /* The east-west street is split where the other one arrives, so there
   * are three segments and a T-junction of degree three. */
  assert(model.road_count == 3);
  int degree_three = 0;
  for (size_t j = 0; j < model.junction_count; j++)
    if (model.junctions[j].degree == 3) degree_three++;
  assert(degree_three == 1);
  assert(model.stats.ends_snapped == 1);
  city_model_free(&model);
}

static void two_ways_meeting_end_to_end_become_one_road_with_a_rounded_bend(
    void) {
  /* A street mapped as two ways, east then north, joined at node 2 that
   * nothing else uses. */
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0010\"/>"
      "<node id=\"3\" lat=\"45.0007\" lon=\"12.0010\"/>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>"
      "<way id=\"11\"><nd ref=\"2\"/><nd ref=\"3\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>"
      "</osm>";
  osm_data_t osm;
  assert(osm_parse(&osm, xml));
  city_model_t model;
  assert(city_import(&model, &osm, city_import_default_options(), "inline"));
  osm_free(&osm);
  assert(model.road_count == 1 && model.stats.roads_merged == 1);
  assert(junction(&model, 2) == NULL && model.junction_count == 2);
  const city_road_t* road = &model.roads[0];
  assert(road->node_ids[0] == 1 && road->node_ids[road->point_count - 1] == 3);
  /* The right angle is now an arc: several points, no turn sharper than
   * about twenty degrees, the ends where the nodes are. */
  assert(road->point_count >= 6 && model.stats.bends_rounded == 1);
  for (size_t i = 1; i + 1 < road->point_count; i++) {
    local_point_t a = road->points[i - 1], b = road->points[i],
                  c = road->points[i + 1];
    double l1 = hypot(b.x - a.x, b.z - a.z), l2 = hypot(c.x - b.x, c.z - b.z);
    double cosine =
        ((b.x - a.x) * (c.x - b.x) + (b.z - a.z) * (c.z - b.z)) / (l1 * l2);
    assert(cosine > cos(0.35));
  }
  city_model_free(&model);
}

static void untagged_buildings_take_heights_from_their_kind(void) {
  /* A house and a hut with no height tags, and a plain building 30 m
   * from a 20 m palazzo. */
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0002\"/>"
      "<node id=\"3\" lat=\"45.0002\" lon=\"12.0002\"/>"
      "<node id=\"4\" lat=\"45.0002\" lon=\"12.0000\"/>"
      "<node id=\"5\" lat=\"45.0000\" lon=\"12.0004\"/>"
      "<node id=\"6\" lat=\"45.0000\" lon=\"12.0006\"/>"
      "<node id=\"7\" lat=\"45.0002\" lon=\"12.0006\"/>"
      "<node id=\"8\" lat=\"45.0002\" lon=\"12.0004\"/>"
      "<node id=\"9\" lat=\"45.0010\" lon=\"12.0000\"/>"
      "<node id=\"10\" lat=\"45.0010\" lon=\"12.0002\"/>"
      "<node id=\"11\" lat=\"45.0012\" lon=\"12.0002\"/>"
      "<node id=\"12\" lat=\"45.0012\" lon=\"12.0000\"/>"
      "<node id=\"13\" lat=\"45.0010\" lon=\"12.0010\"/>"
      "<node id=\"14\" lat=\"45.0010\" lon=\"12.0012\"/>"
      "<node id=\"15\" lat=\"45.0012\" lon=\"12.0012\"/>"
      "<node id=\"16\" lat=\"45.0012\" lon=\"12.0010\"/>"
      "<way id=\"20\"><nd ref=\"1\"/><nd ref=\"2\"/><nd ref=\"3\"/><nd "
      "ref=\"4\"/><nd ref=\"1\"/><tag k=\"building\" v=\"yes\"/><tag "
      "k=\"height\" v=\"20\"/></way>"
      "<way id=\"21\"><nd ref=\"5\"/><nd ref=\"6\"/><nd ref=\"7\"/><nd "
      "ref=\"8\"/><nd ref=\"5\"/><tag k=\"building\" v=\"yes\"/></way>"
      "<way id=\"22\"><nd ref=\"9\"/><nd ref=\"10\"/><nd ref=\"11\"/><nd "
      "ref=\"12\"/><nd ref=\"9\"/><tag k=\"building\" v=\"house\"/></way>"
      "<way id=\"23\"><nd ref=\"13\"/><nd ref=\"14\"/><nd ref=\"15\"/><nd "
      "ref=\"16\"/><nd ref=\"13\"/><tag k=\"building\" v=\"hut\"/></way>"
      "<way id=\"30\"><nd ref=\"1\"/><nd ref=\"13\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>"
      "</osm>";
  osm_data_t osm;
  assert(osm_parse(&osm, xml));
  city_model_t model;
  assert(city_import(&model, &osm, city_import_default_options(), "inline"));
  osm_free(&osm);
  const city_building_t* plain = building(&model, 21);
  assert(plain && plain->height_source == HEIGHT_FROM_NEIGHBOURS &&
         near(plain->height, 20));
  const city_building_t* house = building(&model, 22);
  assert(house && house->height_source == HEIGHT_FROM_CATEGORY &&
         near_to(house->height, 6.4f, 1e-4) && house->levels == 2);
  const city_building_t* hut = building(&model, 23);
  assert(hut && hut->height_source == HEIGHT_FROM_CATEGORY &&
         near(hut->height, 3));
  assert(model.stats.heights_defaulted == 0);
  city_model_free(&model);
}

static void tunnels_and_driveways_are_not_drawn_but_alleys_are(void) {
  /* Four streets: an underpass (tunnel=yes), a passage under a building,
   * a driveway and an alley, plus a plain residential street. Only the
   * street and the alley are on the surface and drivable. */
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.003\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0010\"/>"
      "<node id=\"3\" lat=\"45.0005\" lon=\"12.0000\"/>"
      "<node id=\"4\" lat=\"45.0005\" lon=\"12.0010\"/>"
      "<node id=\"5\" lat=\"45.0010\" lon=\"12.0000\"/>"
      "<node id=\"6\" lat=\"45.0010\" lon=\"12.0010\"/>"
      "<node id=\"7\" lat=\"45.0015\" lon=\"12.0000\"/>"
      "<node id=\"8\" lat=\"45.0015\" lon=\"12.0010\"/>"
      "<node id=\"9\" lat=\"45.0020\" lon=\"12.0000\"/>"
      "<node id=\"10\" lat=\"45.0020\" lon=\"12.0010\"/>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way>"
      "<way id=\"11\"><nd ref=\"3\"/><nd ref=\"4\"/><tag k=\"highway\" "
      "v=\"unclassified\"/><tag k=\"tunnel\" v=\"yes\"/><tag k=\"layer\" "
      "v=\"-1\"/></way>"
      "<way id=\"12\"><nd ref=\"5\"/><nd ref=\"6\"/><tag k=\"highway\" "
      "v=\"residential\"/><tag k=\"tunnel\" v=\"building_passage\"/></way>"
      "<way id=\"13\"><nd ref=\"7\"/><nd ref=\"8\"/><tag k=\"highway\" "
      "v=\"service\"/><tag k=\"service\" v=\"driveway\"/></way>"
      "<way id=\"14\"><nd ref=\"9\"/><nd ref=\"10\"/><tag k=\"highway\" "
      "v=\"service\"/><tag k=\"service\" v=\"alley\"/></way>"
      "</osm>";
  osm_data_t osm;
  assert(osm_parse(&osm, xml));
  city_model_t model;
  assert(city_import(&model, &osm, city_import_default_options(), "inline"));
  osm_free(&osm);
  assert(model.road_count == 2);
  assert(road_from(&model, 10, 0) && road_from(&model, 14, 0));
  assert(road_from(&model, 11, 0) == NULL && road_from(&model, 12, 0) == NULL);
  assert(road_from(&model, 13, 0) == NULL);
  assert(model.stats.tunnels_skipped == 2);
  city_model_free(&model);
}

static void an_elevation_lattice_becomes_a_terrain_that_survives_the_model(
    void) {
  const char* lattice_path = "/tmp/gta-like-test-elevation.txt";
  FILE* file = fopen(lattice_path, "w");
  assert(file);
  /* A lattice of 4 columns and 3 rows around the fixture, rising 2 m to
   * the east per column and 1 m to the south per row; north row first. */
  fprintf(file, "# city-elevation 1\nsource test\nlicence none\n");
  fprintf(file, "grid 45.0030 11.9990 0.0010 0.0020 4 3\n");
  for (int row = 0; row < 3; row++) {
    fprintf(file, "row");
    for (int col = 0; col < 4; col++) fprintf(file, " %d", 20 + 2 * col + row);
    fprintf(file, "\n");
  }
  fclose(file);
  city_model_t model = import_fixture();
  assert(model.terrain.cols == 0);
  assert(city_model_load_elevation(&model, lattice_path));
  assert(model.terrain.cols == 4 && model.terrain.rows == 3);
  /* Heights are relative to the lowest, which is 0. */
  float lowest = 1e9f;
  for (int i = 0; i < 12; i++) lowest = fminf(lowest, model.terrain.heights[i]);
  assert(near_to(lowest, 0.0f, 1e-4f));
  const city_terrain_t* terrain = &model.terrain;
  double west = terrain->min_x, east = terrain->min_x + 3 * terrain->dx;
  double north = terrain->min_z, south = terrain->min_z + 2 * terrain->dz;
  double z_mid = 0.5 * (north + south);
  assert(city_terrain_height(terrain, east, z_mid) >
         city_terrain_height(terrain, west, z_mid) + 3.0);
  assert(city_terrain_height(terrain, 0.5 * (west + east), south) >
         city_terrain_height(terrain, 0.5 * (west + east), north) + 0.5);
  /* Outside the lattice the nearest edge applies, and it is continuous. */
  assert(near_to(city_terrain_height(terrain, east + 500, z_mid),
                 city_terrain_height(terrain, east, z_mid), 1e-4));
  double mid_x = 0.5 * (west + east);
  assert(fabs(city_terrain_height(terrain, mid_x, z_mid) -
              city_terrain_height(terrain, mid_x + 0.01, z_mid)) < 0.01);
  /* The terrain is part of the written model. */
  const char* directory = "/tmp/gta-like-test-terrain-model";
  assert(city_model_write(&model, directory));
  city_model_t loaded;
  assert(city_model_read(&loaded, directory));
  assert(loaded.terrain.cols == 4 && loaded.terrain.rows == 3);
  assert(near_to(city_terrain_height(&loaded.terrain, mid_x, z_mid),
                 city_terrain_height(terrain, mid_x, z_mid), 1e-3));
  city_model_free(&loaded);
  city_model_free(&model);
}

static void single_trees_are_imported_and_survive_the_model(void) {
  const char* xml =
      "<osm><bounds minlat=\"44.999\" minlon=\"11.999\" maxlat=\"45.002\" "
      "maxlon=\"12.003\"/>"
      "<node id=\"1\" lat=\"45.0000\" lon=\"12.0000\"/>"
      "<node id=\"2\" lat=\"45.0000\" lon=\"12.0020\"/>"
      "<node id=\"3\" lat=\"45.0002\" lon=\"12.0010\"><tag k=\"natural\" "
      "v=\"tree\"/></node>"
      "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
      "v=\"residential\"/></way></osm>";
  osm_data_t osm;
  assert(osm_parse(&osm, xml));
  city_model_t model;
  assert(city_import(&model, &osm, city_import_default_options(), "inline"));
  osm_free(&osm);
  assert(model.tree_count == 1);
  /* 0.0002 degrees (22 m) north of the street, over its middle. */
  const city_road_t* street = road_from(&model, 10, 0);
  assert(street);
  assert(near_to(model.trees[0].z, street->points[0].z - 22.2, 0.5));
  assert(near_to(model.trees[0].x,
                 0.5 * (street->points[0].x + street->points[1].x), 1.0));
  const char* directory = "/tmp/gta-like-test-trees-model";
  assert(city_model_write(&model, directory));
  city_model_t loaded;
  assert(city_model_read(&loaded, directory));
  assert(loaded.tree_count == 1);
  assert(near_to(loaded.trees[0].z, model.trees[0].z, 1e-3));
  city_model_free(&loaded);
  city_model_free(&model);
}

static void the_model_round_trips_through_its_text_format(void) {
  city_model_t model = import_fixture();
  const char* directory = "/tmp/gta-like-test-intermediate";
  assert(city_model_write(&model, directory));
  city_model_t loaded;
  assert(city_model_read(&loaded, directory));
  assert(loaded.road_count == model.road_count);
  assert(loaded.building_count == model.building_count);
  assert(building(&loaded, 404) && building(&loaded, 404)->hole_count == 1);
  assert(loaded.junction_count == model.junction_count);
  assert(loaded.area_count == model.area_count);
  assert(area(&loaded, 305) && area(&loaded, 305)->kind == AREA_WATER);
  assert(near_to(area(&loaded, 305)->points[1].x,
                 area(&model, 305)->points[1].x, 1e-3));
  const city_road_t* curved = road_from(&loaded, 107, 0);
  assert(curved &&
         curved->point_count == road_from(&model, 107, 0)->point_count &&
         strcmp(curved->surface, "cobblestone") == 0);
  assert(near_to(curved->points[2].x, road_from(&model, 107, 0)->points[2].x,
                 1e-3));
  assert(near(loaded.projection.origin_lat, model.projection.origin_lat));
  assert(strcmp(loaded.source, "tests/fixtures/district.osm") == 0);
  city_model_free(&loaded);
  city_model_free(&model);
}

void run_city_tests(void) {
  RUN(roads_are_split_at_junctions_and_classified);
  RUN(degenerate_points_and_ways_are_cleaned);
  RUN(junctions_carry_their_degree_and_position);
  RUN(buildings_get_heights_from_tags_levels_or_defaults);
  RUN(building_rings_are_anticlockwise_seen_from_above);
  RUN(default_heights_are_stable_for_the_same_source);
  RUN(areas_come_from_land_use_water_parking_and_squares);
  RUN(dangling_road_ends_snap_onto_the_road_they_nearly_reach);
  RUN(two_ways_meeting_end_to_end_become_one_road_with_a_rounded_bend);
  RUN(untagged_buildings_take_heights_from_their_kind);
  RUN(tunnels_and_driveways_are_not_drawn_but_alleys_are);
  RUN(an_elevation_lattice_becomes_a_terrain_that_survives_the_model);
  RUN(single_trees_are_imported_and_survive_the_model);
  RUN(the_model_round_trips_through_its_text_format);
}
