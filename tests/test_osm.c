#include <assert.h>
#include <string.h>

#include "osm_reader.h"
#include "projection.h"
#include "test_groups.h"
#include "test_support.h"

static void the_fixture_loads_with_nodes_ways_tags_and_relations(void) {
  osm_data_t data;
  assert(osm_load(&data, "tests/fixtures/district.osm"));
  assert(data.node_count == 49);
  assert(data.way_count == 18);
  assert(data.relation_count == 1);
  const osm_node_t* node = osm_find_node(&data, 5);
  assert(node && near(node->lat, 45.0009) && near(node->lon, 12.0013));
  const osm_way_t* way = osm_find_way(&data, 102);
  assert(way && way->node_count == 3 && way->node_refs[1] == 5);
  assert(strcmp(osm_tag(way->tags, way->tag_count, "highway"), "primary") == 0);
  assert(strcmp(osm_tag(way->tags, way->tag_count, "name"), "Main Street") ==
         0);
  assert(osm_tag(way->tags, way->tag_count, "bridge") == NULL);
  assert(near(data.bounds.min_lat, 44.9990) &&
         near(data.bounds.max_lon, 12.0050));
  osm_free(&data);
}

static void a_missing_or_broken_file_is_reported(void) {
  osm_data_t data;
  assert(!osm_load(&data, "tests/fixtures/nope.osm"));
  osm_free(&data);
  assert(!osm_parse(&data, "<osm><way id=\"1\"><nd ref=\"7\"/></osm>"));
  osm_free(&data);
}

static void unknown_elements_are_skipped_and_entities_unescaped(void) {
  osm_data_t data;
  assert(osm_parse(&data,
                   "<osm><note>hi</note><node id=\"1\" lat=\"1\" lon=\"2\"/>"
                   "<way id=\"3\"><nd ref=\"1\"/><tag k=\"name\" v=\"Rock "
                   "&amp; Roll\"/></way></osm>"));
  const osm_way_t* way = osm_find_way(&data, 3);
  assert(way && strcmp(osm_tag(way->tags, way->tag_count, "name"),
                       "Rock & Roll") == 0);
  osm_free(&data);
}

static void projection_gives_metres_with_east_x_and_south_z(void) {
  projection_t projection = projection_create(45.0, 12.0);
  local_point_t origin = projection_project(&projection, 45.0, 12.0);
  assert(near(origin.x, 0) && near(origin.z, 0));
  local_point_t north = projection_project(&projection, 45.0009, 12.0);
  assert(near_to(north.z, -100.1, 0.5) && near(north.x, 0));
  local_point_t east = projection_project(&projection, 45.0, 12.0013);
  assert(near_to(east.x, 102.3, 1.0) && near(east.z, 0));
  double lat, lon;
  projection_unproject(&projection, east, &lat, &lon);
  assert(near_to(lat, 45.0, 1e-9) && near_to(lon, 12.0013, 1e-9));
}

static void projection_centre_comes_from_the_bounds(void) {
  osm_bounds_t bounds = {44.0, 10.0, 46.0, 14.0};
  projection_t projection = projection_for_bounds(bounds);
  assert(near(projection.origin_lat, 45.0) &&
         near(projection.origin_lon, 12.0));
}

void run_osm_tests(void) {
  RUN(the_fixture_loads_with_nodes_ways_tags_and_relations);
  RUN(a_missing_or_broken_file_is_reported);
  RUN(unknown_elements_are_skipped_and_entities_unescaped);
  RUN(projection_gives_metres_with_east_x_and_south_z);
  RUN(projection_centre_comes_from_the_bounds);
}
