#include <stdio.h>
#include <string.h>

#include "test_allocator.h"
#include "test_groups.h"

typedef struct {
  const char* name;
  void (*run)(void);
} test_group_t;

static const test_group_t groups[] = {
    {"math", run_math_tests},
    {"core", run_core_tests},
};

int main(int argc, char* argv[]) {
  if (argc != 2) {
    fprintf(stderr, "usage: tests <group>\n");
    return 2;
  }
  for (size_t i = 0; i < sizeof(groups) / sizeof(groups[0]); i++) {
    if (strcmp(groups[i].name, argv[1]) == 0) {
      groups[i].run();
      if (outstanding_allocations() != 0) {
        fprintf(stderr, "%zu allocations leaked\n", outstanding_allocations());
        return 1;
      }
      return 0;
    }
  }
  fprintf(stderr, "unknown test group %s\n", argv[1]);
  return 2;
}
