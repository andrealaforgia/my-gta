/* LiDAR heights: ASCII grids in the regional UTM frame, the terrain they
 * give, and the building heights read from surface minus terrain. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "city_import.h"
#include "city_lidar.h"
#include "city_model.h"
#include "lidar.h"
#include "osm_reader.h"
#include "test_groups.h"
#include "test_support.h"

/* An ESRI ASCII grid of cols x rows cells from (west, south), whose value at
 * a cell centre is f(easting, northing). One cell may be no-data. */
static void write_asc(const char* path, int cols, int rows, double west,
                      double south, double cell, double (*f)(double, double),
                      int hole_col, int hole_row) {
  FILE* file = fopen(path, "w");
  assert(file);
  fprintf(file, "ncols %d\nnrows %d\nxllcorner %.3f\nyllcorner %.3f\n", cols,
          rows, west, south);
  fprintf(file, "cellsize %.3f\nNODATA_value -9999\n", cell);
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      double e = west + (c + 0.5) * cell;
      double n = south + (rows - 1 - r + 0.5) * cell;
      if (c == hole_col && r == hole_row)
        fprintf(file, "-9999 ");
      else
        fprintf(file, "%.3f ", f(e, n));
    }
    fprintf(file, "\n");
  }
  fclose(file);
}

static double ramp(double e, double n) {
  (void)n;
  return 100.0 + 0.5 * (e - 1000.0);
}

static void utm_matches_known_points(void) {
  double e, n;
  /* On the central meridian of zone 33 the easting is the false easting and
   * the northing the scaled meridian arc: 4 982 950 m at 45 degrees. */
  utm33_forward(45.0, 15.0, &e, &n);
  assert(near_to(e, 500000.0, 0.01) && near_to(n, 4982950.0, 15.0));
  /* Pordenone: about 181 km west of the meridian, in the regional bounds. */
  utm33_forward(45.9569, 12.6603, &e, &n);
  assert(e > 318000 && e < 320500 && n > 5091000 && n < 5093000);
  assert(e > 290497 && e < 417495 && n > 5048165 && n < 5168481);
  /* Three degrees of longitude at 45 degrees north is about 236 km. */
  double e2, n2;
  utm33_forward(45.0, 12.0, &e2, &n2);
  assert(near_to(500000.0 - e2, 235800.0, 1500.0));
}

static void an_ascii_grid_is_read_sampled_and_cropped(void) {
  const char* path = "/tmp/gta-like-test-grid.asc";
  write_asc(path, 10, 8, 1000.0, 2000.0, 2.0, ramp, 4, 3);
  lidar_set_t set;
  lidar_set_init(&set);
  assert(lidar_set_load_asc(&set, path, NULL));
  float v;
  /* Cell centres hold the formula; between them it is interpolated. */
  assert(lidar_set_sample(&set, 1005.0, 2007.0, &v) && near_to(v, 102.5, 0.01));
  assert(lidar_set_sample(&set, 1006.0, 2007.0, &v) && near_to(v, 103.0, 0.01));
  /* The no-data cell and anywhere off the grid give nothing. */
  double hole_e = 1000.0 + 4.5 * 2.0;
  double hole_n = 2000.0 + (8 - 1 - 3 + 0.5) * 2.0;
  assert(!lidar_set_sample(&set, hole_e, hole_n, &v));
  assert(!lidar_set_sample(&set, 900.0, 2007.0, &v));
  assert(!lidar_set_sample(&set, 1005.0, 2100.0, &v));
  lidar_set_free(&set);
  /* A window keeps only what it needs, and still answers inside it. */
  lidar_window_t window = {1006.0, 2004.0, 1012.0, 2012.0};
  lidar_set_init(&set);
  assert(lidar_set_load_asc(&set, path, &window));
  assert(set.tiles[0].cols < 10 && set.tiles[0].rows < 8);
  assert(lidar_set_sample(&set, 1009.0, 2008.0, &v) && near_to(v, 104.5, 0.01));
  assert(!lidar_set_sample(&set, 1001.0, 2008.0, &v));
  lidar_set_free(&set);
  /* A missing or malformed file is refused. */
  lidar_set_init(&set);
  assert(!lidar_set_load_asc(&set, "/tmp/gta-like-no-such-grid.asc", NULL));
  lidar_set_free(&set);
}

static const char* area =
    "<osm><bounds minlat=\"45.9570\" minlon=\"12.6580\" maxlat=\"45.9590\" "
    "maxlon=\"12.6600\"/>"
    "<node id=\"1\" lat=\"45.9575\" lon=\"12.6582\"/>"
    "<node id=\"2\" lat=\"45.9575\" lon=\"12.6598\"/>"
    /* House A about the origin, 21 m by 16 m, no height known. */
    "<node id=\"11\" lat=\"45.95793\" lon=\"12.65886\"/>"
    "<node id=\"12\" lat=\"45.95793\" lon=\"12.65914\"/>"
    "<node id=\"13\" lat=\"45.95807\" lon=\"12.65914\"/>"
    "<node id=\"14\" lat=\"45.95807\" lon=\"12.65886\"/>"
    /* House B 100 m east, mapped as 6 m. */
    "<node id=\"21\" lat=\"45.95793\" lon=\"12.66006\"/>"
    "<node id=\"22\" lat=\"45.95793\" lon=\"12.66034\"/>"
    "<node id=\"23\" lat=\"45.95807\" lon=\"12.66034\"/>"
    "<node id=\"24\" lat=\"45.95807\" lon=\"12.66006\"/>"
    "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><tag k=\"highway\" "
    "v=\"residential\"/></way>"
    "<way id=\"100\"><nd ref=\"11\"/><nd ref=\"12\"/><nd ref=\"13\"/><nd "
    "ref=\"14\"/><nd ref=\"11\"/><tag k=\"building\" v=\"yes\"/></way>"
    "<way id=\"200\"><nd ref=\"21\"/><nd ref=\"22\"/><nd ref=\"23\"/><nd "
    "ref=\"24\"/><nd ref=\"21\"/><tag k=\"building\" v=\"yes\"/><tag "
    "k=\"height\" v=\"6\"/></way></osm>";

static city_model_t area_model(void) {
  osm_data_t osm;
  assert(osm_parse(&osm, area));
  city_model_t model;
  assert(city_import(&model, &osm, city_import_default_options(), "inline"));
  osm_free(&osm);
  return model;
}

static double origin_e, origin_n;

static double slope_east(double e, double n) {
  (void)n;
  return 50.0 + 0.02 * (e - origin_e);
}

/* The surface: 9 m above the ground over a block round the origin. */
static double surface_with_a_roof(double e, double n) {
  bool roof = fabs(e - origin_e) < 13.0 && fabs(n - origin_n) < 10.0;
  return slope_east(e, n) + (roof ? 9.0 : 0.0);
}

static lidar_set_t grid_over_the_area(double (*f)(double, double),
                                      const char* path) {
  utm33_forward(45.9580, 12.6590, &origin_e, &origin_n);
  write_asc(path, 500, 400, origin_e - 250.0, origin_n - 200.0, 1.0, f, -1, -1);
  lidar_set_t set;
  lidar_set_init(&set);
  assert(lidar_set_load_asc(&set, path, NULL));
  return set;
}

static void a_terrain_model_gives_the_ground_its_real_slope(void) {
  city_model_t model = area_model();
  lidar_set_t dtm = grid_over_the_area(slope_east, "/tmp/gta-like-dtm.asc");
  assert(city_model_apply_dtm(&model, &dtm, 5.0));
  const city_terrain_t* t = &model.terrain;
  assert(t->cols > 10 && t->rows > 5 && near_to(t->dx, 5.0, 0.01));
  /* 2 % climbing east: 100 m east is 2 m higher, whatever the north-south
   * position, and the lowest point of the grid is zero. */
  double low = city_terrain_height(t, 0, 0);
  double high = city_terrain_height(t, 100, 0);
  assert(near_to(high - low, 2.0, 0.1));
  assert(near_to(city_terrain_height(t, 0, -60), low, 0.15));
  float lowest = 1e9f;
  for (int i = 0; i < t->cols * t->rows; i++)
    lowest = fminf(lowest, t->heights[i]);
  assert(near_to(lowest, 0.0, 1e-4));
  lidar_set_free(&dtm);
  city_model_free(&model);
}

static void building_heights_come_from_surface_minus_terrain(void) {
  city_model_t model = area_model();
  lidar_set_t dtm = grid_over_the_area(slope_east, "/tmp/gta-like-dtm.asc");
  lidar_set_t dsm =
      grid_over_the_area(surface_with_a_roof, "/tmp/gta-like-dsm.asc");
  size_t changed = city_model_apply_dsm(&model, &dsm, &dtm);
  assert(changed == 1); /* the mapped house keeps the mapped height */
  const city_building_t* a = NULL;
  const city_building_t* b = NULL;
  for (size_t i = 0; i < model.building_count; i++) {
    if (model.buildings[i].source_id == 100) a = &model.buildings[i];
    if (model.buildings[i].source_id == 200) b = &model.buildings[i];
  }
  assert(a && b);
  assert(near_to(a->height, 9.0, 0.3) && a->height_source == HEIGHT_FROM_LIDAR);
  assert(near_to(b->height, 6.0, 1e-4) && b->height_source == HEIGHT_TAGGED);
  lidar_set_free(&dsm);
  lidar_set_free(&dtm);
  city_model_free(&model);
}

void run_lidar_tests(void) {
  RUN(utm_matches_known_points);
  RUN(an_ascii_grid_is_read_sampled_and_cropped);
  RUN(a_terrain_model_gives_the_ground_its_real_slope);
  RUN(building_heights_come_from_surface_minus_terrain);
}
