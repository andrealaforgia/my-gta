#include "osm_reader.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "log.h"

/* ---- growable arrays ---- */

static bool grow(void** items, size_t* capacity, size_t needed, size_t item_size) {
  if (needed <= *capacity) return true;
  size_t wanted = *capacity ? *capacity * 2 : 256;
  while (wanted < needed) wanted *= 2;
  void* grown = realloc(*items, wanted * item_size);
  if (!grown) return false;
  *items = grown;
  *capacity = wanted;
  return true;
}

typedef struct {
  osm_data_t* data;
  size_t node_capacity, way_capacity, relation_capacity;
  osm_way_t* way;           /* way being read */
  size_t ref_capacity;
  osm_relation_t* relation; /* relation being read */
  osm_tag_t** tags;         /* tag list of the open element */
  size_t* tag_count;
  size_t tag_capacity;
  bool ok;
} parser_t;

/* ---- tiny XML scanning ---- */

static const char* skip_space(const char* s) {
  while (*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r') s++;
  return s;
}

static void unescape(char* text) {
  static const struct {
    const char* entity;
    char character;
  } entities[] = {{"&amp;", '&'}, {"&quot;", '"'}, {"&apos;", '\''}, {"&lt;", '<'}, {"&gt;", '>'}};
  char* out = text;
  for (const char* in = text; *in;) {
    bool replaced = false;
    if (*in == '&') {
      for (size_t i = 0; i < sizeof(entities) / sizeof(entities[0]); i++) {
        size_t length = strlen(entities[i].entity);
        if (strncmp(in, entities[i].entity, length) == 0) {
          *out++ = entities[i].character;
          in += length;
          replaced = true;
          break;
        }
      }
    }
    if (!replaced) *out++ = *in++;
  }
  *out = '\0';
}

/* Finds attribute `name` inside the tag text [start, end). */
static bool attribute(const char* start, const char* end, const char* name, char* out, size_t size) {
  size_t name_length = strlen(name);
  for (const char* s = start; s + name_length + 2 < end; s++) {
    if (strncmp(s, name, name_length) != 0 || s[name_length] != '=') continue;
    if (s > start && !(s[-1] == ' ' || s[-1] == '\t' || s[-1] == '\n')) continue;
    const char* quote = s + name_length + 1;
    if (*quote != '"' && *quote != '\'') continue;
    char delimiter = *quote;
    const char* value = quote + 1;
    const char* close = memchr(value, delimiter, (size_t)(end - value));
    if (!close) return false;
    size_t length = (size_t)(close - value);
    if (length >= size) length = size - 1;
    memcpy(out, value, length);
    out[length] = '\0';
    unescape(out);
    return true;
  }
  return false;
}

static bool open_tags(parser_t* p, osm_tag_t** tags, size_t* count) {
  p->tags = tags;
  p->tag_count = count;
  p->tag_capacity = 0;
  return true;
}

static void add_tag(parser_t* p, const char* start, const char* end) {
  if (!p->tags) return;
  osm_tag_t tag;
  if (!attribute(start, end, "k", tag.key, sizeof(tag.key))) return;
  if (!attribute(start, end, "v", tag.value, sizeof(tag.value))) tag.value[0] = '\0';
  if (!grow((void**)p->tags, &p->tag_capacity, *p->tag_count + 1, sizeof(osm_tag_t))) {
    p->ok = false;
    return;
  }
  (*p->tags)[(*p->tag_count)++] = tag;
}

static void start_node(parser_t* p, const char* start, const char* end) {
  char id[32], lat[32], lon[32];
  if (!attribute(start, end, "id", id, sizeof(id)) || !attribute(start, end, "lat", lat, sizeof(lat)) ||
      !attribute(start, end, "lon", lon, sizeof(lon))) {
    p->ok = false;
    return;
  }
  osm_data_t* d = p->data;
  if (!grow((void**)&d->nodes, &p->node_capacity, d->node_count + 1, sizeof(osm_node_t))) {
    p->ok = false;
    return;
  }
  osm_node_t* node = &d->nodes[d->node_count++];
  node->id = strtoll(id, NULL, 10);
  node->lat = strtod(lat, NULL);
  node->lon = strtod(lon, NULL);
  p->tags = NULL; /* node tags are not kept */
}

static void start_way(parser_t* p, const char* start, const char* end) {
  char id[32];
  if (!attribute(start, end, "id", id, sizeof(id))) {
    p->ok = false;
    return;
  }
  osm_data_t* d = p->data;
  if (!grow((void**)&d->ways, &p->way_capacity, d->way_count + 1, sizeof(osm_way_t))) {
    p->ok = false;
    return;
  }
  osm_way_t* way = &d->ways[d->way_count++];
  memset(way, 0, sizeof(*way));
  way->id = strtoll(id, NULL, 10);
  p->way = way;
  p->ref_capacity = 0;
  open_tags(p, &way->tags, &way->tag_count);
}

static void add_ref(parser_t* p, const char* start, const char* end) {
  if (!p->way) return;
  char ref[32];
  if (!attribute(start, end, "ref", ref, sizeof(ref))) return;
  if (!grow((void**)&p->way->node_refs, &p->ref_capacity, p->way->node_count + 1, sizeof(int64_t))) {
    p->ok = false;
    return;
  }
  p->way->node_refs[p->way->node_count++] = strtoll(ref, NULL, 10);
}

static void start_relation(parser_t* p, const char* start, const char* end) {
  char id[32];
  if (!attribute(start, end, "id", id, sizeof(id))) {
    p->ok = false;
    return;
  }
  osm_data_t* d = p->data;
  if (!grow((void**)&d->relations, &p->relation_capacity, d->relation_count + 1, sizeof(osm_relation_t))) {
    p->ok = false;
    return;
  }
  osm_relation_t* relation = &d->relations[d->relation_count++];
  memset(relation, 0, sizeof(*relation));
  relation->id = strtoll(id, NULL, 10);
  p->relation = relation;
  open_tags(p, &relation->tags, &relation->tag_count);
}

static void read_bounds(parser_t* p, const char* start, const char* end) {
  char value[32];
  osm_bounds_t* b = &p->data->bounds;
  bool ok = attribute(start, end, "minlat", value, sizeof(value));
  b->min_lat = strtod(value, NULL);
  ok = ok && attribute(start, end, "minlon", value, sizeof(value));
  b->min_lon = strtod(value, NULL);
  ok = ok && attribute(start, end, "maxlat", value, sizeof(value));
  b->max_lat = strtod(value, NULL);
  ok = ok && attribute(start, end, "maxlon", value, sizeof(value));
  b->max_lon = strtod(value, NULL);
  p->data->has_bounds = ok;
}

static void element(parser_t* p, const char* start, const char* end) {
  /* start points after '<', end at '>' */
  if (*start == '?' || *start == '!') return;
  if (*start == '/') {
    const char* name = start + 1;
    if (strncmp(name, "way", 3) == 0) {
      p->way = NULL;
      p->tags = NULL;
    } else if (strncmp(name, "relation", 8) == 0) {
      p->relation = NULL;
      p->tags = NULL;
    } else if (strncmp(name, "node", 4) == 0) {
      p->tags = NULL;
    }
    return;
  }
  const char* name_end = start;
  while (name_end < end && *name_end != ' ' && *name_end != '/' && *name_end != '\n' &&
         *name_end != '\t')
    name_end++;
  size_t length = (size_t)(name_end - start);
  bool self_closing = end > start && end[-1] == '/';
  if (length == 4 && strncmp(start, "node", 4) == 0) {
    start_node(p, name_end, end);
  } else if (length == 3 && strncmp(start, "way", 3) == 0) {
    start_way(p, name_end, end);
    if (self_closing) p->way = NULL;
  } else if (length == 2 && strncmp(start, "nd", 2) == 0) {
    add_ref(p, name_end, end);
  } else if (length == 3 && strncmp(start, "tag", 3) == 0) {
    add_tag(p, name_end, end);
  } else if (length == 8 && strncmp(start, "relation", 8) == 0) {
    start_relation(p, name_end, end);
    if (self_closing) p->relation = NULL;
  } else if (length == 6 && strncmp(start, "bounds", 6) == 0) {
    read_bounds(p, name_end, end);
  }
  /* member, osm and anything else: skipped */
}

bool osm_parse(osm_data_t* data, const char* xml) {
  memset(data, 0, sizeof(*data));
  parser_t p;
  memset(&p, 0, sizeof(p));
  p.data = data;
  p.ok = true;
  const char* cursor = xml;
  while (p.ok && (cursor = strchr(cursor, '<')) != NULL) {
    const char* start = cursor + 1;
    const char* end;
    if (strncmp(start, "!--", 3) == 0) {
      end = strstr(start, "-->");
      if (!end) break;
      cursor = end + 3;
      continue;
    }
    end = strchr(start, '>');
    if (!end) break;
    element(&p, skip_space(start), end);
    cursor = end + 1;
  }
  if (!p.ok) {
    LOG_ERROR("osm", "malformed element near byte %ld", (long)(cursor ? cursor - xml : -1));
    return false;
  }
  if (p.way || p.relation) {
    LOG_ERROR("osm", "unterminated way or relation");
    return false;
  }
  for (size_t i = 0; i < data->way_count; i++) {
    for (size_t r = 0; r < data->ways[i].node_count; r++) {
      if (!osm_find_node(data, data->ways[i].node_refs[r])) {
        LOG_ERROR("osm", "way %lld refers to missing node %lld", (long long)data->ways[i].id,
                  (long long)data->ways[i].node_refs[r]);
        return false;
      }
    }
  }
  return true;
}

bool osm_load(osm_data_t* data, const char* path) {
  memset(data, 0, sizeof(*data));
  FILE* file = fopen(path, "rb");
  if (!file) {
    LOG_ERROR("osm", "cannot open %s", path);
    return false;
  }
  fseek(file, 0, SEEK_END);
  long size = ftell(file);
  fseek(file, 0, SEEK_SET);
  if (size < 0) {
    fclose(file);
    return false;
  }
  char* text = malloc((size_t)size + 1);
  if (!text) {
    fclose(file);
    return false;
  }
  size_t read = fread(text, 1, (size_t)size, file);
  fclose(file);
  text[read] = '\0';
  bool ok = osm_parse(data, text);
  free(text);
  return ok;
}

void osm_free(osm_data_t* data) {
  for (size_t i = 0; i < data->way_count; i++) {
    free(data->ways[i].node_refs);
    free(data->ways[i].tags);
  }
  for (size_t i = 0; i < data->relation_count; i++) free(data->relations[i].tags);
  free(data->nodes);
  free(data->ways);
  free(data->relations);
  memset(data, 0, sizeof(*data));
}

static int compare_node_ids(const void* a, const void* b) {
  int64_t x = ((const osm_node_t*)a)->id, y = ((const osm_node_t*)b)->id;
  return x < y ? -1 : (x > y ? 1 : 0);
}

const osm_node_t* osm_find_node(const osm_data_t* data, int64_t id) {
  /* Files are usually id-ordered; binary search when they are, scan otherwise. */
  static const osm_data_t* sorted_for = NULL;
  static bool sorted = false;
  if (sorted_for != data) {
    sorted = true;
    for (size_t i = 1; i < data->node_count && sorted; i++)
      if (data->nodes[i - 1].id > data->nodes[i].id) sorted = false;
    sorted_for = data;
  }
  if (sorted) {
    osm_node_t key = {id, 0, 0};
    return bsearch(&key, data->nodes, data->node_count, sizeof(osm_node_t), compare_node_ids);
  }
  for (size_t i = 0; i < data->node_count; i++)
    if (data->nodes[i].id == id) return &data->nodes[i];
  return NULL;
}

const osm_way_t* osm_find_way(const osm_data_t* data, int64_t id) {
  for (size_t i = 0; i < data->way_count; i++)
    if (data->ways[i].id == id) return &data->ways[i];
  return NULL;
}

const char* osm_tag(const osm_tag_t* tags, size_t count, const char* key) {
  for (size_t i = 0; i < count; i++)
    if (strcmp(tags[i].key, key) == 0) return tags[i].value;
  return NULL;
}
