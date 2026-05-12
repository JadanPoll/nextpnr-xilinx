#!/usr/bin/env python3
import sys, json, os

# EWS-specific paths
sys.path.insert(0, os.path.expanduser("~/prjxray"))
import prjxray.db

db_path = os.path.expanduser("~/nextpnr-xilinx/xilinx/external/prjxray-db/spartan7")
db = prjxray.db.Database(db_path, "xc7s50csga324-1")
grid = db.grid()
tiles = list(grid.tiles())

print("Extracting FASM map (this will take a minute or two)...")
with open("xc7s50_fasm_map.json", "w") as f:
    f.write('{"map":{')
    first = True
    for idx, tile_name in enumerate(tiles):
        if idx % 1000 == 0: print(f"  {idx}/{len(tiles)} tiles processed...")
        loc = grid.loc_of_tilename(tile_name)
        gi = grid.gridinfo_at_loc(loc)
        sb = db.get_tile_segbits(gi.tile_type)
        if not sb.segbits: continue
        for block_type, features in sb.segbits.items():
            if block_type not in gi.bits: continue
            base_addr = gi.bits[block_type].base_address
            for feature_name, bits in features.items():
                coords = []
                for bit in bits:
                    if bit.isset:
                        coords.append([
                            base_addr + bit.word_column,
                            bit.word_bit // 32,
                            bit.word_bit % 32
                        ])
                if coords:
                    suffix = feature_name.split('.', 1)[1]
                    key = f"{tile_name}.{suffix}"
                    val = coords[0] if len(coords) == 1 else coords
                    entry = f'"{key}":{json.dumps(val)}'
                    if not first: f.write(',')
                    f.write(entry)
                    first = False
    f.write('}}')
print("Done! Map saved to xc7s50_fasm_map.json")
