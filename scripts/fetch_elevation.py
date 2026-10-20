#!/usr/bin/env python3
"""Downloads ground elevations for a bounding box as a lat/lon lattice.

    scripts/fetch_elevation.py MIN_LAT MIN_LON MAX_LAT MAX_LON OUTPUT [SPACING_M]

Samples the EU-DEM 25 m model (Copernicus, free to use with attribution)
through the public Open Topo Data API, 100 points per request at one
request a second, and writes the lattice the city importer reads with
city-import --elevation. Standard library only.
"""
import json
import math
import sys
import time
import urllib.request

API = "https://api.opentopodata.org/v1/eudem25m"
BATCH = 100
MARGIN_M = 100.0


def main(argv):
    if len(argv) < 6:
        print(__doc__)
        return 2
    min_lat, min_lon, max_lat, max_lon = (float(v) for v in argv[1:5])
    output = argv[5]
    spacing = float(argv[6]) if len(argv) > 6 else 30.0
    metres_per_degree_lat = 111320.0
    mid_lat = 0.5 * (min_lat + max_lat)
    metres_per_degree_lon = 111320.0 * math.cos(math.radians(mid_lat))
    dlat = spacing / metres_per_degree_lat
    dlon = spacing / metres_per_degree_lon
    north = max_lat + MARGIN_M / metres_per_degree_lat
    west = min_lon - MARGIN_M / metres_per_degree_lon
    south = min_lat - MARGIN_M / metres_per_degree_lat
    east = max_lon + MARGIN_M / metres_per_degree_lon
    cols = int(math.ceil((east - west) / dlon)) + 1
    rows = int(math.ceil((north - south) / dlat)) + 1
    points = [(north - j * dlat, west + i * dlon)
              for j in range(rows) for i in range(cols)]
    print("%d x %d samples, %d requests" % (cols, rows, math.ceil(len(points) / BATCH)))
    values = []
    for start in range(0, len(points), BATCH):
        batch = points[start:start + BATCH]
        locations = "|".join("%.6f,%.6f" % p for p in batch)
        request = urllib.request.Request(
            API + "?locations=" + locations,
            headers={"User-Agent": "gta-like-prototype/0.1"})
        for attempt in range(5):
            try:
                with urllib.request.urlopen(request, timeout=60) as response:
                    reply = json.load(response)
                break
            except Exception as error:  # network hiccup or rate limit
                print("retrying after", error)
                time.sleep(3 + 3 * attempt)
        else:
            print("giving up")
            return 1
        values.extend(r["elevation"] if r["elevation"] is not None else 0.0
                      for r in reply["results"])
        time.sleep(1.1)
    with open(output, "w") as out:
        out.write("# city-elevation 1\n")
        out.write("source EU-DEM v1.1 25 m via api.opentopodata.org\n")
        out.write("licence Copernicus EU-DEM, free with attribution\n")
        out.write("grid %.7f %.7f %.9f %.9f %d %d\n" % (north, west, dlat, dlon, cols, rows))
        for j in range(rows):
            out.write("row " + " ".join("%.2f" % v for v in values[j * cols:(j + 1) * cols]) + "\n")
    print("wrote", output, "elevations", min(values), "to", max(values), "m")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
