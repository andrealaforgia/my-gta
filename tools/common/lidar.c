#include "lidar.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "log.h"

void utm33_forward(double lat, double lon, double* easting, double* northing) {
  const double a = 6378137.0, f = 1.0 / 298.257222101, k0 = 0.9996;
  const double n = f / (2.0 - f);
  const double big_a =
      a / (1.0 + n) * (1.0 + n * n / 4.0 + n * n * n * n / 64.0);
  const double alpha[3] = {n / 2.0 - 2.0 * n * n / 3.0 + 5.0 * n * n * n / 16.0,
                           13.0 * n * n / 48.0 - 3.0 * n * n * n / 5.0,
                           61.0 * n * n * n / 240.0};
  double phi = lat * M_PI / 180.0;
  double dlambda = (lon - 15.0) * M_PI / 180.0;
  double c = 2.0 * sqrt(n) / (1.0 + n);
  double t = sinh(atanh(sin(phi)) - c * atanh(c * sin(phi)));
  double xi = atan2(t, cos(dlambda));
  double eta = atanh(sin(dlambda) / sqrt(1.0 + t * t));
  double xi_sum = xi, eta_sum = eta;
  for (int j = 1; j <= 3; j++) {
    xi_sum += alpha[j - 1] * sin(2.0 * j * xi) * cosh(2.0 * j * eta);
    eta_sum += alpha[j - 1] * cos(2.0 * j * xi) * sinh(2.0 * j * eta);
  }
  *easting = 500000.0 + k0 * big_a * eta_sum;
  *northing = k0 * big_a * xi_sum;
}

void lidar_set_init(lidar_set_t* set) { memset(set, 0, sizeof(*set)); }

void lidar_set_free(lidar_set_t* set) {
  for (size_t i = 0; i < set->count; i++) free(set->tiles[i].values);
  free(set->tiles);
  memset(set, 0, sizeof(*set));
}

typedef struct {
  int cols, rows;
  double west, south, cell, nodata;
} asc_header_t;

static bool read_header(FILE* file, asc_header_t* h) {
  bool have[5] = {false, false, false, false, false};
  bool west_is_centre = false, south_is_centre = false;
  asc_header_t blank = {0, 0, 0.0, 0.0, 0.0, -9999.0};
  *h = blank;
  for (int line = 0; line < 6; line++) {
    char key[32];
    double value;
    long position = ftell(file);
    if (fscanf(file, "%31s %lf", key, &value) != 2) return false;
    for (char* p = key; *p; p++) *p = (char)tolower((unsigned char)*p);
    if (strcmp(key, "ncols") == 0) {
      h->cols = (int)value, have[0] = true;
    } else if (strcmp(key, "nrows") == 0) {
      h->rows = (int)value, have[1] = true;
    } else if (strncmp(key, "xll", 3) == 0) {
      h->west = value, have[2] = true,
      west_is_centre = strcmp(key, "xllcenter") == 0;
    } else if (strncmp(key, "yll", 3) == 0) {
      h->south = value, have[3] = true,
      south_is_centre = strcmp(key, "yllcenter") == 0;
    } else if (strcmp(key, "cellsize") == 0) {
      h->cell = value, have[4] = true;
    } else if (strcmp(key, "nodata_value") == 0) {
      h->nodata = value;
    } else {
      fseek(file, position, SEEK_SET); /* the first row of data: no more keys */
      break;
    }
  }
  if (!(have[0] && have[1] && have[2] && have[3] && have[4])) return false;
  /* A "centre" origin is the middle of the first cell, not its corner. */
  if (west_is_centre) h->west -= h->cell * 0.5;
  if (south_is_centre) h->south -= h->cell * 0.5;
  return h->cols > 1 && h->rows > 1 && h->cell > 0 && h->cols <= 100000 &&
         h->rows <= 100000;
}

bool lidar_set_load_asc(lidar_set_t* set, const char* path,
                        const lidar_window_t* window) {
  FILE* file = fopen(path, "r");
  if (!file) {
    LOG_ERROR("lidar", "cannot open %s", path);
    return false;
  }
  asc_header_t h;
  if (!read_header(file, &h)) {
    LOG_ERROR("lidar", "%s: not an ESRI ASCII grid", path);
    fclose(file);
    return false;
  }
  /* The cells to keep: those covering the window, plus one for the
   * interpolation. Rows count from the north. */
  int c0 = 0, c1 = h.cols - 1, r0 = 0, r1 = h.rows - 1;
  double top = h.south + h.rows * h.cell;
  if (window) {
    c0 = (int)floor((window->west - h.west) / h.cell) - 1;
    c1 = (int)floor((window->east - h.west) / h.cell) + 1;
    r0 = (int)floor((top - window->north) / h.cell) - 1;
    r1 = (int)floor((top - window->south) / h.cell) + 1;
    if (c0 < 0) c0 = 0;
    if (r0 < 0) r0 = 0;
    if (c1 > h.cols - 1) c1 = h.cols - 1;
    if (r1 > h.rows - 1) r1 = h.rows - 1;
    if (c0 > c1 || r0 > r1) {
      fclose(file);
      return false; /* the file lies outside the window */
    }
  }
  int cols = c1 - c0 + 1, rows = r1 - r0 + 1;
  float* values = malloc((size_t)cols * (size_t)rows * sizeof(float));
  if (!values) {
    fclose(file);
    return false;
  }
  bool ok = true;
  for (int r = 0; r < h.rows && ok; r++) {
    for (int c = 0; c < h.cols && ok; c++) {
      float v;
      if (fscanf(file, "%f", &v) != 1) {
        ok = false;
        break;
      }
      if (r < r0 || r > r1 || c < c0 || c > c1) continue;
      values[(size_t)(r - r0) * (size_t)cols + (size_t)(c - c0)] =
          fabs((double)v - h.nodata) < 1e-3 ? NAN : v;
    }
    if (r > r1) break; /* the rest is below the window */
  }
  fclose(file);
  if (!ok) {
    LOG_ERROR("lidar", "%s: the grid ends early", path);
    free(values);
    return false;
  }
  lidar_tile_t* grown = realloc(set->tiles, (set->count + 1) * sizeof(*grown));
  if (!grown) {
    free(values);
    return false;
  }
  set->tiles = grown;
  lidar_tile_t* tile = &set->tiles[set->count++];
  tile->west = h.west + c0 * h.cell;
  tile->south = h.south + (h.rows - 1 - r1) * h.cell;
  tile->cell = h.cell;
  tile->cols = cols;
  tile->rows = rows;
  tile->values = values;
  LOG_INFO("lidar", "%s: %d x %d cells of %.2f m kept", path, cols, rows,
           h.cell);
  return true;
}

static bool sample_tile(const lidar_tile_t* t, double e, double n,
                        float* value) {
  double top = t->south + t->rows * t->cell;
  if (e < t->west || e > t->west + t->cols * t->cell || n < t->south || n > top)
    return false;
  double u = (e - t->west) / t->cell - 0.5;
  double v = (top - n) / t->cell - 0.5;
  int nearest_c = (int)floor(u + 0.5), nearest_r = (int)floor(v + 0.5);
  if (nearest_c < 0) nearest_c = 0;
  if (nearest_r < 0) nearest_r = 0;
  if (nearest_c > t->cols - 1) nearest_c = t->cols - 1;
  if (nearest_r > t->rows - 1) nearest_r = t->rows - 1;
  if (isnan(t->values[(size_t)nearest_r * (size_t)t->cols + (size_t)nearest_c]))
    return false;
  int c = (int)floor(u), r = (int)floor(v);
  double fu = u - c, fv = v - r;
  double sum = 0, weight = 0;
  for (int dr = 0; dr <= 1; dr++)
    for (int dc = 0; dc <= 1; dc++) {
      int cc = c + dc, rr = r + dr;
      if (cc < 0) cc = 0;
      if (rr < 0) rr = 0;
      if (cc > t->cols - 1) cc = t->cols - 1;
      if (rr > t->rows - 1) rr = t->rows - 1;
      float x = t->values[(size_t)rr * (size_t)t->cols + (size_t)cc];
      if (isnan(x)) continue;
      double w = (dc ? fu : 1.0 - fu) * (dr ? fv : 1.0 - fv);
      sum += w * x, weight += w;
    }
  if (weight <= 0) return false;
  *value = (float)(sum / weight);
  return true;
}

bool lidar_set_sample(const lidar_set_t* set, double easting, double northing,
                      float* value) {
  for (size_t i = 0; i < set->count; i++)
    if (sample_tile(&set->tiles[i], easting, northing, value)) return true;
  return false;
}
