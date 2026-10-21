#!/usr/bin/env python3
"""Downloads an OpenStreetMap extract larger than the API allows in one call.

    scripts/fetch_osm.py MIN_LON MIN_LAT MAX_LON MAX_LAT OUTPUT [TILE_DEG]

The public map API refuses boxes with more than 50 000 nodes, so the box is
fetched in tiles (0.008 degrees by default), one request a second, and the
tiles are merged: nodes, ways and relations kept once each by id. Writes OSM
XML that city-import reads. Standard library only. Data is © OpenStreetMap
contributors, ODbL 1.0.
"""
import sys
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET

API = "https://api.openstreetmap.org/api/0.6/map?bbox=%.6f,%.6f,%.6f,%.6f"
USER_AGENT = "gta-like-prototype/0.1"


def fetch(box):
    request = urllib.request.Request(API % box,
                                     headers={"User-Agent": USER_AGENT})
    for attempt in range(6):
        try:
            with urllib.request.urlopen(request, timeout=120) as response:
                return ET.fromstring(response.read())
        except urllib.error.HTTPError as error:
            if error.code == 400:  # too many nodes: caller splits the tile
                return None
            wait = 5 + 10 * attempt
        except Exception:  # network hiccup
            wait = 5 + 10 * attempt
        print("  retrying in %d s" % wait, file=sys.stderr)
        time.sleep(wait)
    raise SystemExit("giving up on tile %s" % (box,))


def main(argv):
    if len(argv) < 6:
        print(__doc__)
        return 2
    min_lon, min_lat, max_lon, max_lat = (float(v) for v in argv[1:5])
    output = argv[5]
    tile = float(argv[6]) if len(argv) > 6 else 0.008
    nodes, ways, relations = {}, {}, {}
    pending = []
    lon = min_lon
    while lon < max_lon:
        lat = min_lat
        while lat < max_lat:
            pending.append((lon, lat, min(lon + tile, max_lon),
                            min(lat + tile * 0.7, max_lat)))
            lat += tile * 0.7
        lon += tile
    print("%d tiles" % len(pending))
    done = 0
    while pending:
        box = pending.pop()
        root = fetch(box)
        time.sleep(1.0)
        if root is None:  # dense: split in four
            mx, my = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            pending += [(box[0], box[1], mx, my), (mx, box[1], box[2], my),
                        (box[0], my, mx, box[3]), (mx, my, box[2], box[3])]
            continue
        for element in root:
            table = {"node": nodes, "way": ways,
                     "relation": relations}.get(element.tag)
            if table is not None:
                table.setdefault(element.get("id"), element)
        done += 1
        print("tile %d done, %d nodes %d ways" % (done, len(nodes), len(ways)),
              flush=True)
    out = ET.Element("osm", {"version": "0.6", "generator": "fetch_osm.py"})
    ET.SubElement(out, "bounds", {"minlat": "%.6f" % min_lat,
                                  "minlon": "%.6f" % min_lon,
                                  "maxlat": "%.6f" % max_lat,
                                  "maxlon": "%.6f" % max_lon})
    for table in (nodes, ways, relations):
        out.extend(table.values())
    ET.ElementTree(out).write(output, encoding="utf-8", xml_declaration=True)
    print("wrote %s: %d nodes, %d ways, %d relations" %
          (output, len(nodes), len(ways), len(relations)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
