#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "city_import.h"
#include "city_model.h"
#include "osm_reader.h"
#include "test_groups.h"
#include "test_support.h"

static city_model_t import_fixture(void) {
  osm_data_t osm;
  assert(osm_load(&osm, "tests/fixtures/district.osm"));
  city_import_options_t options = city_import_default_options();
  city_model_t model;
  assert(city_import(&model, &osm, options, "tests/fixtures/district.osm"));
  osm_free(&osm);
  return model;
}

static const city_road_t* road_from(const city_model_t* model, int64_t source_id, int index) {
  int seen = 0;
  for (size_t i = 0; i < model->road_count; i++)
    if (model->roads[i].source_id == source_id && seen++ == index) return &model->roads[i];
  return NULL;
}

static const city_junction_t* junction(const city_model_t* model, int64_t node_id) {
  for (size_t i = 0; i < model->junction_count; i++)
    if (model->junctions[i].node_id == node_id) return &model->junctions[i];
  return NULL;
}

static const city_building_t* building(const city_model_t* model, int64_t source_id) {
  for (size_t i = 0; i < model->building_count; i++)
    if (model->buildings[i].source_id == source_id) return &model->buildings[i];
  return NULL;
}

static void roads_are_split_at_junctions_and_classified(void) {
  city_model_t model = import_fixture();
  /* Six grid ways of two segments each, the curved road, the stub road. */
  assert(model.road_count == 14);
  const city_road_t* main_west = road_from(&model, 102, 0);
  assert(main_west && main_west->class == ROAD_CLASS_PRIMARY);
  assert(near(main_west->width, 9.0f) && !main_west->width_inferred);
  assert(main_west->point_count == 2);
  const city_road_t* south = road_from(&model, 101, 0);
  assert(south && south->width_inferred && south->width > 4 && south->width < 8);
  const city_road_t* oneway = road_from(&model, 105, 1);
  assert(oneway && oneway->oneway);
  const city_road_t* curved = road_from(&model, 107, 0);
  assert(curved && curved->point_count == 5 && strcmp(curved->surface, "cobblestone") == 0);
  assert(road_from(&model, 108, 0) == NULL); /* footway ignored */
  city_model_free(&model);
}

static void degenerate_points_and_ways_are_cleaned(void) {
  city_model_t model = import_fixture();
  const city_road_t* stub = road_from(&model, 109, 0);
  assert(stub && stub->point_count == 3); /* the repeated node collapsed */
  assert(road_from(&model, 110, 0) == NULL);  /* zero length dropped */
  assert(model.stats.ways_ignored >= 1);
  assert(model.stats.roads_degenerate == 1);
  city_model_free(&model);
}

static void junctions_carry_their_degree_and_position(void) {
  city_model_t model = import_fixture();
  assert(junction(&model, 5) && junction(&model, 5)->degree == 4);
  assert(junction(&model, 2) && junction(&model, 2)->degree == 3);
  assert(junction(&model, 8) && junction(&model, 8)->degree == 3);
  assert(junction(&model, 1) && junction(&model, 1)->degree == 2);
  assert(junction(&model, 3) && junction(&model, 3)->degree == 3); /* corner plus the curved road */
  assert(junction(&model, 13) && junction(&model, 13)->degree == 1); /* cul-de-sac */
  const city_junction_t* centre = junction(&model, 5);
  const city_junction_t* corner = junction(&model, 1);
  /* n5 is north-east of n1: larger x, smaller z. */
  assert(centre->position.x > corner->position.x + 90 && centre->position.z < corner->position.z - 90);
  city_model_free(&model);
}

static void buildings_get_heights_from_tags_levels_or_defaults(void) {
  city_model_t model = import_fixture();
  assert(model.building_count == 4);
  const city_building_t* tagged = building(&model, 201);
  assert(tagged && near(tagged->height, 12) && tagged->height_source == HEIGHT_TAGGED);
  const city_building_t* levels = building(&model, 202);
  assert(levels && levels->levels == 4 && levels->height_source == HEIGHT_FROM_LEVELS);
  assert(near_to(levels->height, 4 * 3.2f, 1e-4));
  const city_building_t* plain = building(&model, 203);
  assert(plain && plain->height_source == HEIGHT_DEFAULT && plain->height >= 6 && plain->height <= 15);
  assert(plain->point_count == 6);
  assert(building(&model, 204) == NULL);
  assert(model.stats.buildings_malformed == 1);
  assert(model.stats.relations_skipped == 1);
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
  for (size_t i = 0; i < model.building_count; i++) assert(signed_area_x_north(&model.buildings[i]) > 0);
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

static void the_model_round_trips_through_its_text_format(void) {
  city_model_t model = import_fixture();
  const char* directory = "/tmp/gta-like-test-intermediate";
  assert(city_model_write(&model, directory));
  city_model_t loaded;
  assert(city_model_read(&loaded, directory));
  assert(loaded.road_count == model.road_count);
  assert(loaded.building_count == model.building_count);
  assert(loaded.junction_count == model.junction_count);
  const city_road_t* curved = road_from(&loaded, 107, 0);
  assert(curved && curved->point_count == 5 && strcmp(curved->surface, "cobblestone") == 0);
  assert(near_to(curved->points[2].x, road_from(&model, 107, 0)->points[2].x, 1e-3));
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
  RUN(the_model_round_trips_through_its_text_format);
}
