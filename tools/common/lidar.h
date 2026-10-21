/**
 * @file lidar.h
 * @brief Elevation rasters from airborne laser surveys, as ESRI ASCII grids
 * in a metric UTM frame (the Friuli Venezia Giulia regional LiDAR comes as
 * RDN2008 / UTM zone 33N, within a metre of WGS84 here). A set holds several
 * tiles; sampling finds the tile that covers a point. Offline tool code:
 * nothing here is used at runtime.
 */
#ifndef TOOLS_COMMON_LIDAR_H_
#define TOOLS_COMMON_LIDAR_H_

#include <stdbool.h>
#include <stddef.h>

/* Geographic degrees to UTM zone 33N (central meridian 15 degrees east)
 * metres, GRS80 / WGS84 ellipsoid: Kruger's series, good to millimetres. */
void utm33_forward(double lat, double lon, double* easting, double* northing);

typedef struct {
  double west, south; /* the lower-left corner of the lower-left cell */
  double cell;        /* metres */
  int cols, rows;
  float* values; /* row 0 is the northern row; NAN where there is no data */
} lidar_tile_t;

typedef struct {
  lidar_tile_t* tiles;
  size_t count;
} lidar_set_t;

/* A window in the grid's own coordinates, to read only what is needed. */
typedef struct {
  double west, south, east, north;
} lidar_window_t;

void lidar_set_init(lidar_set_t* set);
void lidar_set_free(lidar_set_t* set);
/* Reads one ASCII grid into the set; `window` may be NULL for the whole
 * file. False when the file is missing, malformed or misses the window. */
bool lidar_set_load_asc(lidar_set_t* set, const char* path,
                        const lidar_window_t* window);
/* The height at a point, interpolated between cell centres; false off the
 * grid or where the nearest cell has no data. */
bool lidar_set_sample(const lidar_set_t* set, double easting, double northing,
                      float* value);

#endif  // TOOLS_COMMON_LIDAR_H_
