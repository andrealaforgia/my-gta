/* Procedural textures: the properties that make them read as what they are. */
#include <assert.h>
#include <math.h>
#include <stdlib.h>

#include "test_groups.h"
#include "test_support.h"
#include "textures.h"

static float luminance_at(const uint8_t* pixels, int x, int y) {
  const uint8_t* p = pixels + (y * TEXTURE_SIZE + x) * 4;
  return (0.3f * p[0] + 0.6f * p[1] + 0.1f * p[2]) / 255.0f;
}

static float mean_over(const uint8_t* pixels, float u0, float u1) {
  double sum = 0;
  int count = 0;
  for (int y = 0; y < TEXTURE_SIZE; y++)
    for (int x = (int)(u0 * TEXTURE_SIZE); x < (int)(u1 * TEXTURE_SIZE); x++) {
      sum += luminance_at(pixels, x, y);
      count++;
    }
  return (float)(sum / count);
}

static void roads_show_wheel_tracks_and_lane_markings(void) {
  uint8_t* road = texture_road(2);
  /* Two lanes: tracks at an eighth and three eighths across each. */
  float track = mean_over(road, 0.115f, 0.135f);
  float between = mean_over(road, 0.2f, 0.3f);
  assert(track < between * 0.94f);
  /* The centre line is still the brightest thing on it. */
  float centre = 0;
  for (int y = 0; y < 32; y++)
    centre += luminance_at(road, TEXTURE_SIZE / 2, y);
  assert(centre / 32 > 2.0f * between);
  /* Patches and cracks: the surface is not uniform along its length. */
  float lowest = 1, highest = 0;
  for (int row = 0; row < 8; row++) {
    double sum = 0;
    for (int y = row * 32; y < row * 32 + 32; y++)
      for (int x = (int)(0.2f * TEXTURE_SIZE); x < (int)(0.3f * TEXTURE_SIZE);
           x++)
        sum += luminance_at(road, x, y);
    float mean = (float)(sum / (32.0 * (0.1f * TEXTURE_SIZE)));
    lowest = fminf(lowest, mean), highest = fmaxf(highest, mean);
  }
  assert(highest > lowest * 1.05f);
  free(road);
}

void run_textures_tests(void) {
  RUN(roads_show_wheel_tracks_and_lane_markings);
}
