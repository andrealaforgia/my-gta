.DEFAULT_GOAL := all

CC ?= cc
SDL2_CONFIG ?= $(shell command -v sdl2-config 2>/dev/null || echo /opt/homebrew/bin/sdl2-config)

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S), Linux)
    INSTALL_CMD = sudo apt-get update && sudo apt-get install -y libsdl2-dev libgl1-mesa-dev
    DEV_INSTALL_CMD = sudo apt-get update && sudo apt-get install -y cpplint clang-format
    GL_LFLAGS = -lGL
else ifeq ($(UNAME_S), Darwin)
    INSTALL_CMD = brew install sdl2
    DEV_INSTALL_CMD = brew install cpplint clang-format
    GL_LFLAGS = -framework OpenGL
else
    $(error Unsupported platform)
endif

SDL2_CFLAGS := $(shell $(SDL2_CONFIG) --cflags)
SDL2_LFLAGS := $(shell $(SDL2_CONFIG) --libs)

ENGINE_DIRS = engine/core engine/math engine/platform engine/renderer engine/renderer/gl \
              engine/assets engine/world engine/physics engine/audio engine/vehicle engine/camera
GAME_DIRS = $(wildcard game/src/*)
TOOL_DIRS = tools/city_import tools/city_generate tools/city_validate tools/common

ENGINE_SRC = $(foreach dir,$(ENGINE_DIRS),$(wildcard $(dir)/*.c))
GAME_SRC = $(foreach dir,$(GAME_DIRS),$(wildcard $(dir)/*.c))
TOOL_SRC = $(foreach dir,$(TOOL_DIRS),$(wildcard $(dir)/*.c))
HEADERS = $(foreach dir,$(ENGINE_DIRS) $(GAME_DIRS) $(TOOL_DIRS),$(wildcard $(dir)/*.h))
TEST_SRC = $(wildcard tests/*.c)

ENGINE_OBJ = $(ENGINE_SRC:.c=.o)
GAME_OBJ = $(GAME_SRC:.c=.o)
TOOL_OBJ = $(TOOL_SRC:.c=.o)
DEP = $(ENGINE_OBJ:.o=.d) $(GAME_OBJ:.o=.d) $(TOOL_OBJ:.o=.d)

-include $(DEP)

INCLUDES = $(foreach dir,$(ENGINE_DIRS) $(GAME_DIRS) $(TOOL_DIRS),-I$(dir))

OPT ?= -O2
CFLAGS := -ggdb3 $(OPT) --std=c99 -Wall -Wextra -pedantic-errors -DGL_SILENCE_DEPRECATION \
          $(INCLUDES) $(SDL2_CFLAGS)
LFLAGS := $(SDL2_LFLAGS) $(GL_LFLAGS) -lm

TARGET = gta-like
ENGINE_LIB = libengine.a
CITY_IMPORT = city-import
CITY_GENERATE = city-generate
CITY_VALIDATE = city-validate
SNAPSHOT = render_snapshot
SOUND = render_sound

.PHONY: all tools install dev_install clean lint format test sanitize snapshot sound city fixture-city run run-city

all: $(TARGET) tools

tools: $(CITY_IMPORT) $(CITY_GENERATE) $(CITY_VALIDATE)

$(ENGINE_LIB): $(ENGINE_OBJ)
	ar rcs $@ $^

$(TARGET): $(GAME_OBJ) $(ENGINE_LIB)
	$(CC) -o $@ $^ $(LFLAGS)

$(CITY_IMPORT): tools/city_import/main.o $(filter-out tools/%/main.o,$(TOOL_OBJ)) $(ENGINE_LIB)
	$(CC) -o $@ $^ $(LFLAGS)

$(CITY_GENERATE): tools/city_generate/main.o $(filter-out tools/%/main.o,$(TOOL_OBJ)) $(ENGINE_LIB)
	$(CC) -o $@ $^ $(LFLAGS)

$(CITY_VALIDATE): tools/city_validate/main.o $(filter-out tools/%/main.o,$(TOOL_OBJ)) $(ENGINE_LIB)
	$(CC) -o $@ $^ $(LFLAGS)

$(SNAPSHOT): $(filter-out game/src/main/main.o,$(GAME_OBJ)) tests/render_snapshot.c $(ENGINE_LIB)
	$(CC) $(CFLAGS) -o $@ $^ $(LFLAGS)

snapshot: $(SNAPSHOT)
	./$(SNAPSHOT) snapshot.bmp

$(SOUND): tests/render_sound.c $(ENGINE_LIB)
	$(CC) $(CFLAGS) -o $@ $^ $(LFLAGS)

sound: $(SOUND)
	./$(SOUND) engine.wav

%.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -c -o $@ $<

install:
	$(INSTALL_CMD)

dev_install:
	$(DEV_INSTALL_CMD)

lint:
	cpplint --quiet --root=. $(ENGINE_SRC) $(GAME_SRC) $(TOOL_SRC) $(HEADERS) $(TEST_SRC)

format:
	clang-format -i $(ENGINE_SRC) $(GAME_SRC) $(TOOL_SRC) $(HEADERS) $(TEST_SRC)

test:
	python3 tests/run_tests.py

sanitize:
	python3 tests/run_tests.py --sanitize

# make city EXTRACT=data/pordenone.osm CITY=pordenone
EXTRACT ?= data/pordenone.osm
CITY ?= $(basename $(notdir $(EXTRACT)))
CITY_CONFIG ?= assets/city/default.cfg

city: tools
	./$(CITY_IMPORT) --input $(EXTRACT) --output data/intermediate/$(CITY)
	./$(CITY_GENERATE) --input data/intermediate/$(CITY) --config $(CITY_CONFIG) --output assets_cooked/cities/$(CITY)
	./$(CITY_VALIDATE) assets_cooked/cities/$(CITY)

fixture-city: tools
	$(MAKE) city EXTRACT=tests/fixtures/district.osm CITY=district

run: $(TARGET)
	./$(TARGET) --scene=track

run-city: $(TARGET)
	./$(TARGET) --scene=city:assets_cooked/cities/$(CITY)

clean:
	rm -f $(ENGINE_OBJ) $(GAME_OBJ) $(TOOL_OBJ) $(DEP) $(ENGINE_LIB) $(TARGET) \
	      $(CITY_IMPORT) $(CITY_GENERATE) $(CITY_VALIDATE) $(SNAPSHOT) $(SOUND) snapshot.bmp engine.wav
