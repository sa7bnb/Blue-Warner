#!/usr/bin/env python3
"""
Fartkameror i Sverige med hastighetsgräns - allt i ett program.

  1. Laddar ner Sverige-utdraget av OpenStreetMap från Geofabrik
  2. Plockar ut alla fartkameror (highway=speed_camera)
  3. Kopplar varje kamera till en hastighetsgräns
  4. Skapar speed_cameras_with_limits.h (och speed_cameras.csv)

Hastigheten tas i den här ordningen:
  1. maxspeed på själva kameran
  2. maxspeed på en enforcement-relation som kameran ingår i
  3. maxspeed på närmaste bilväg inom SOKRADIE_M meter
  4. (valfritt) hastigheten i en gammal speed_cameras_with_limits.h
  5. annars OKAND (0)

Kör:
  python3 fartkameror.py
  python3 fartkameror.py gammal_speed_cameras_with_limits.h   (med reserv)
  python3 fartkameror.py --ny                                 (tvinga ny nedladdning)

Första körningen skapar ~/osmvenv och installerar pyosmium där automatiskt.
OSM-filen (sweden-latest.osm.pbf) sparas i mappen och återanvänds om den är
yngre än MAX_ALDER_DAGAR dagar. Behöver ungefär 2 GB ledigt RAM.

Data: (c) OpenStreetMap-bidragsgivare, licens ODbL.
"""
import csv
import math
import os
import re
import subprocess
import sys
import time
import urllib.request

PBF_URL = "https://download.geofabrik.de/europe/sweden-latest.osm.pbf"
PBF_FIL = "sweden-latest.osm.pbf"
UT_H = "speed_cameras_with_limits.h"
UT_CSV = "speed_cameras.csv"

MAX_ALDER_DAGAR = 7   # ladda ner OSM-filen på nytt om den är äldre
VENV = os.path.expanduser("~/osmvenv")

SOKRADIE_M = 40       # hur långt från kameran en väg får ligga
GAMMAL_RADIE_M = 30   # max avstånd vid matchning mot gammal fil
OKAND = 0             # skrivs när ingen hastighet hittas
INTE_BILVAG = {"footway", "cycleway", "path", "pedestrian", "steps",
               "track", "bridleway", "corridor", "platform", "construction"}

CELL_LAT, CELL_LON = 0.005, 0.01   # ca 550 x 550 m i Sverige


# ---------- inläsning av .h-filer ----------

def las_gammal(fil):
    m = re.compile(r"\{\s*(-?\d+\.\d+)\s*,\s*(-?\d+\.\d+)\s*,\s*(\d+)\s*\}")
    with open(fil, encoding="utf-8", errors="replace") as f:
        return [(float(a), float(b), int(c)) for a, b, c in m.findall(f.read())]


# ---------- geometri ----------

def meter(lat1, lon1, lat2, lon2):
    return math.hypot((lon2 - lon1) * 111320 * math.cos(math.radians(lat1)),
                      (lat2 - lat1) * 110540)


def avstand_punkt_segment(lat, lon, a, b):
    kx = 111320 * math.cos(math.radians(lat))
    ky = 110540
    ax, ay = (a[1] - lon) * kx, (a[0] - lat) * ky
    bx, by = (b[1] - lon) * kx, (b[0] - lat) * ky
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, (-ax * dx - ay * dy) / l2))
    return math.hypot(ax + t * dx, ay + t * dy)


def cell(lat, lon):
    return (int(lat / CELL_LAT), int(lon / CELL_LON))


def till_kmh(text):
    """'70' -> 70, '50 mph' -> 80, 'SE:urban' -> None"""
    if not text:
        return None
    t = text.split(";")[0].strip().lower()
    try:
        if "mph" in t:
            return round(float(t.replace("mph", "")) * 1.609)
        return int(float(t.replace("km/h", "").strip()))
    except ValueError:
        return None


def tal(x):
    return f"{x:.6f}".rstrip("0").rstrip(".")


# ---------- nedladdning ----------

def hamta_pbf(tvinga):
    if os.path.exists(PBF_FIL) and not tvinga:
        dagar = (time.time() - os.path.getmtime(PBF_FIL)) / 86400
        if dagar <= MAX_ALDER_DAGAR:
            print(f"Använder befintlig {PBF_FIL} "
                  f"({os.path.getsize(PBF_FIL) / 1e6:.0f} MB, {dagar:.0f} dagar gammal)")
            return
        print(f"{PBF_FIL} är {dagar:.0f} dagar gammal, hämtar ny.")
    print(f"Laddar ner {PBF_URL} ...")
    tmp = PBF_FIL + ".del"

    def framsteg(block, storlek, totalt):
        if totalt > 0:
            klart = min(block * storlek, totalt)
            print(f"\r  {klart / 1e6:6.0f} / {totalt / 1e6:.0f} MB", end="", flush=True)

    urllib.request.urlretrieve(PBF_URL, tmp, framsteg)
    os.replace(tmp, PBF_FIL)
    print("\n  klart.")


def ordna_osmium():
    """Ser till att pyosmium finns. Saknas det skapas ~/osmvenv och
    programmet startas om därifrån."""
    try:
        import osmium  # noqa: F401
        return
    except ImportError:
        pass
    vpy = os.path.join(VENV, "bin", "python")
    if os.path.realpath(sys.prefix) == os.path.realpath(VENV):
        sys.exit("pyosmium gick inte att importera trots installation i " + VENV)
    try:
        if not os.path.exists(vpy):
            print(f"Skapar Python-miljö i {VENV} ...")
            subprocess.check_call([sys.executable, "-m", "venv", VENV])
        if subprocess.call([vpy, "-c", "import osmium"],
                           stderr=subprocess.DEVNULL) != 0:
            print("Installerar pyosmium (görs bara första gången) ...")
            subprocess.check_call([vpy, "-m", "pip", "install", "-q", "osmium"])
    except (subprocess.CalledProcessError, OSError) as e:
        sys.exit(f"Kunde inte förbereda Python-miljön: {e}\n"
                 "Saknas venv? Kör:  sudo apt install python3-venv\n"
                 f"Ta sedan bort {VENV} och försök igen.")
    os.execv(vpy, [vpy, os.path.abspath(__file__)] + sys.argv[1:])


# ---------- huvudprogram ----------

def main():
    arg = [a for a in sys.argv[1:] if a != "--ny"]
    tvinga = "--ny" in sys.argv[1:]
    ordna_osmium()
    import osmium

    gamla = las_gammal(arg[0]) if arg else []
    if arg:
        print(f"Läste {len(gamla)} kameror med hastighet från {arg[0]}")

    hamta_pbf(tvinga)

    nara = None        # rutor nära någon kamera, byggs när noderna är lästa
    osm_kameror = {}   # nod-id -> (lat, lon, hastighet eller None)
    vagar = []         # (hastighet, [(lat, lon), ...])
    rel_hast = {}      # nod-id -> hastighet

    print("Läser OSM-filen (tar någon minut) ...")
    fp = (osmium.FileProcessor(PBF_FIL)
          .with_locations()
          .with_filter(osmium.filter.KeyFilter("highway", "type")))
    for o in fp:
        if o.is_node():
            if o.tags.get("highway") == "speed_camera":
                osm_kameror[o.id] = (o.location.lat, o.location.lon,
                                     till_kmh(o.tags.get("maxspeed")))
        elif o.is_way():
            if nara is None:
                nara = set()
                for la, lo, _ in osm_kameror.values():
                    cy, cx = cell(la, lo)
                    nara.update((cy + dy, cx + dx)
                                for dy in (-1, 0, 1) for dx in (-1, 0, 1))
            if o.tags.get("highway") in INTE_BILVAG:
                continue
            v = till_kmh(o.tags.get("maxspeed"))
            if v is None:
                continue
            punkter = [(n.lat, n.lon) for n in o.nodes if n.location.valid()]
            if any(cell(la, lo) in nara for la, lo in punkter):
                vagar.append((v, punkter))
        elif o.is_relation():
            if o.tags.get("type") != "enforcement":
                continue
            v = till_kmh(o.tags.get("maxspeed"))
            if v:
                for m in o.members:
                    if m.type == "n":
                        rel_hast[m.ref] = v
    print(f"  {len(osm_kameror)} kameror, {len(vagar)} vägar nära kameror, "
          f"{len(rel_hast)} relationskopplingar")
    if not osm_kameror:
        sys.exit("Hittade inga kameror i OSM-filen - avbryter utan att skriva något.")

    # index: ruta -> vägar
    vag_idx = {}
    for i, (v, punkter) in enumerate(vagar):
        for c in {cell(la, lo) for la, lo in punkter}:
            vag_idx.setdefault(c, set()).add(i)

    statistik = {"kamera": 0, "relation": 0, "väg": 0, "gammal fil": 0, "okänd": 0}
    rader = []
    for nid, (lat, lon, v) in sorted(osm_kameror.items()):
        cy, cx = cell(lat, lon)
        grannar = [(cy + dy, cx + dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1)]
        kalla = "kamera" if v is not None else None

        # 2: enforcement-relation
        if v is None and nid in rel_hast:
            v, kalla = rel_hast[nid], "relation"

        # 3: närmaste väg
        if v is None:
            kandidater = set()
            for c in grannar:
                kandidater |= vag_idx.get(c, set())
            narmast = (SOKRADIE_M + 1, None)
            for i in kandidater:
                wv, p = vagar[i]
                if len(p) < 2:
                    continue
                d = min(avstand_punkt_segment(lat, lon, p[j], p[j + 1])
                        for j in range(len(p) - 1))
                if d < narmast[0]:
                    narmast = (d, wv)
            if narmast[1] is not None:
                v, kalla = narmast[1], "väg"

        # 4: gammal fil
        if v is None and gamla:
            d, gv = min((meter(lat, lon, g[0], g[1]), g[2]) for g in gamla)
            if d <= GAMMAL_RADIE_M:
                v, kalla = gv, "gammal fil"

        if v is None:
            v, kalla = OKAND, "okänd"
        statistik[kalla] += 1
        rader.append((lat, lon, v))

    with open(UT_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["lat", "lon", "speed"])
        for lat, lon, v in rader:
            w.writerow([tal(lat), tal(lon), v])

    with open(UT_H, "w") as f:
        f.write("// Fartkameror med hastighetsgränser\n")
        f.write("// Genererad från OpenStreetMap (ODbL) via Geofabrik\n\n")
        f.write("#ifndef SPEED_CAMERAS_WITH_LIMITS_H\n")
        f.write("#define SPEED_CAMERAS_WITH_LIMITS_H\n\n")
        f.write("struct SpeedCamera {\n    float latitude;\n    float longitude;\n"
                "    int speedLimit;\n};\n\n")
        f.write("const SpeedCamera speedCameras[] = {\n")
        for lat, lon, v in rader:
            f.write(f"    {{{tal(lat)}, {tal(lon)}, {v}}},\n")
        f.write("};\n\n")
        f.write(f"const int numCameras = {len(rader)};\n\n#endif\n")

    print("Hastighet hämtad från:", statistik)
    print(f"Klart! Sparade {len(rader)} kameror i {UT_H}")


if __name__ == "__main__":
    main()
