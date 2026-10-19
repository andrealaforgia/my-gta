#ifndef TESTS_TEST_SUPPORT_H_
#define TESTS_TEST_SUPPORT_H_
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define RUN(test)            \
  do {                       \
    printf("  %s\n", #test); \
    test();                  \
  } while (0)

static inline bool near_to(double a, double b, double tolerance) {
  return fabs(a - b) <= tolerance;
}

static inline bool near(double a, double b) { return near_to(a, b, 1e-6); }

#endif  // TESTS_TEST_SUPPORT_H_
