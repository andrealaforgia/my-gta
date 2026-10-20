"""Compile the engine, prototype and tools with a tracking allocator and run
every test group without opening a window."""

import argparse
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SDL2_CONFIG = os.environ.get("SDL2_CONFIG") or (
    "sdl2-config" if subprocess.run(["which", "sdl2-config"], capture_output=True).returncode == 0
    else "/opt/homebrew/bin/sdl2-config"
)

GROUPS = (
    "math",
    "core",
    "mesh",
    "physics",
    "vehicle",
    "camera",
    "audio",
    "world",
    "osm",
    "city",
    "junction",
    "generator",
    "citydrive",
    "citymap",
    "daylight",
    "traffic",
)


def source_directories():
    directories = [p for p in (ROOT / "engine").rglob("*") if p.is_dir()]
    directories += [p for p in (ROOT / "game/src").iterdir() if p.is_dir()]
    directories += [p for p in (ROOT / "tools").iterdir() if p.is_dir()]
    return sorted(directories)


def sources():
    found = []
    for directory in source_directories():
        found += sorted(directory.glob("*.c"))
    return [p for p in found if p.name not in ("main.c", "clock.c") and "renderer/gl" not in str(p)]


def build(destination, sanitize):
    compiler = shlex.split(os.environ.get("CC", "cc"))
    flags = ["-std=c99", "-O1", "-g", "-Wall", "-Wextra", "-Werror", "-DGL_SILENCE_DEPRECATION"]
    if sanitize:
        flags += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
    includes = ["-I" + str(d) for d in source_directories()] + ["-I" + str(ROOT / "tests")]
    sdl_flags = shlex.split(subprocess.check_output([SDL2_CONFIG, "--cflags"], text=True))
    sdl_libs = shlex.split(subprocess.check_output([SDL2_CONFIG, "--libs"], text=True))
    test_sources = sorted((ROOT / "tests").glob("test_*.c"))
    objects = []
    for index, source in enumerate(sources() + test_sources):
        target = destination.parent / (str(index) + ".o")
        instrumentation = (
            []
            if source.name in ("test_allocator.c", "test_clock.c")
            else [
                "-include", str(ROOT / "tests/test_allocator.h"),
                "-Dmalloc=test_malloc", "-Dcalloc=test_calloc",
                "-Drealloc=test_realloc", "-Dfree=test_free",
            ]
        )
        subprocess.run(
            compiler + flags + includes + sdl_flags + instrumentation
            + ["-c", str(source), "-o", str(target)],
            check=True,
        )
        objects.append(str(target))
    subprocess.run(compiler + flags + objects + sdl_libs + ["-lm", "-o", str(destination)], check=True)


def runtime_environment():
    environment = dict(os.environ)
    environment.setdefault("UBSAN_OPTIONS", "halt_on_error=1")
    environment.setdefault("SDL_VIDEODRIVER", "dummy")
    environment.setdefault("SDL_AUDIODRIVER", "dummy")
    if sys.platform == "darwin":
        libraries = shlex.split(subprocess.check_output([SDL2_CONFIG, "--libs"], text=True))
        directories = [flag[2:] for flag in libraries if flag.startswith("-L")]
        if environment.get("DYLD_LIBRARY_PATH"):
            directories.append(environment["DYLD_LIBRARY_PATH"])
        environment["DYLD_LIBRARY_PATH"] = ":".join(directories)
        environment.setdefault("ASAN_OPTIONS", "detect_leaks=0")
    return environment


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sanitize", action="store_true")
    parser.add_argument("groups", nargs="*", default=list(GROUPS))
    args = parser.parse_args()
    failures = 0
    with tempfile.TemporaryDirectory(prefix="gta-like-tests-") as directory:
        binary = Path(directory) / "tests"
        build(binary, args.sanitize)
        for name in args.groups:
            try:
                result = subprocess.run(
                    [str(binary), name], text=True, capture_output=True, timeout=60,
                    cwd=str(ROOT), env=runtime_environment(),
                )
                passed = result.returncode == 0
                output = result.stdout + result.stderr
            except subprocess.TimeoutExpired:
                passed, output = False, "Test exceeded 60 seconds"
            print(("PASS " if passed else "FAIL ") + name)
            if not passed:
                failures += 1
                print(output)
    raise SystemExit(bool(failures))


if __name__ == "__main__":
    main()
