#!/usr/bin/env python3
"""Turns a GLSL file into a C array of string pieces for glShaderSource.

    scripts/embed_shader.py NAME input.frag output.inc

A single C string literal may be only about 4 000 characters under
-pedantic, so the shader is cut into pieces at line ends and handed to
OpenGL together. The output defines `static const char* const
NAME_parts[]` and `NAME_PART_COUNT`.
"""
import sys


def main(argv):
    name, source, target = argv[1:4]
    lines = open(source).read().rstrip("\n").split("\n")
    pieces, current, size = [], [], 0
    for line in lines:
        if size + len(line) > 3000:
            pieces.append(current)
            current, size = [], 0
        current.append(line)
        size += len(line) + 1
    pieces.append(current)
    out = ["/* Generated from %s by scripts/embed_shader.py; do not edit. */" % source,
           "static const char* const %s_parts[] = {" % name]
    rendered = []
    for piece in pieces:
        rendered.append("\n".join(
            '"%s\\n"' % l.replace("\\", "\\\\").replace('"', '\\"')
            for l in piece))
    out.append(",\n".join(rendered))
    out.append("};")
    out.append("#define %s_PART_COUNT %d" % (name.upper(), len(pieces)))
    open(target, "w").write("\n".join(out) + "\n")


if __name__ == "__main__":
    main(sys.argv)
