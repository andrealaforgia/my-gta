#include "test_allocator.h"

#include <stdlib.h>

static size_t outstanding = 0;
static int allocations_until_failure = -1;

static int should_fail(void) {
  if (allocations_until_failure < 0) return 0;
  if (allocations_until_failure == 0) return 1;
  allocations_until_failure--;
  return 0;
}

void* test_malloc(size_t size) {
  if (should_fail()) return NULL;
  void* pointer = malloc(size);
  if (pointer) outstanding++;
  return pointer;
}

void* test_calloc(size_t count, size_t size) {
  if (should_fail()) return NULL;
  void* pointer = calloc(count, size);
  if (pointer) outstanding++;
  return pointer;
}

void* test_realloc(void* pointer, size_t size) {
  if (should_fail()) return NULL;
  void* grown = realloc(pointer, size);
  if (grown && !pointer) outstanding++;
  return grown;
}

void test_free(void* pointer) {
  if (pointer) outstanding--;
  free(pointer);
}

void fail_allocation_after(int count) { allocations_until_failure = count; }

size_t outstanding_allocations(void) { return outstanding; }
