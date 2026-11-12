import re, sys, base64, zlib, struct, numpy as np, xml.etree.ElementTree as ET
def arr(da, header_compressed):
    fmt = da.get("format"); typ = da.get("type"); nc = int(da.get("NumberOfComponents", 1))
    dt = {"Float32": "<f4", "Float64": "<f8", "Int32": "<i4", "Int64": "<i8", "UInt8": "u1", "UInt32": "<u4", "UInt64": "<u8"}[typ]
    if fmt == "ascii":
        a = np.array(da.text.split(), dtype=float)
    else:
        raw = base64.b64decode("".join(da.text.split()))
        if header_compressed:
            nb, bs, lbs = struct.unpack("<3I", raw[:12]); cs = struct.unpack(f"<{nb}I", raw[12:12 + 4 * nb])
            # header itself is base64 separately in VTK; fall back to decoding header-length prefix
            off = 12 + 4 * nb; out = b""
            for c in cs: out += zlib.decompress(raw[off:off + c]); off += c
            a = np.frombuffer(out, dt)
        else:
            a = np.frombuffer(raw[4:], dt)
    return a.reshape(-1, nc) if nc > 1 else a
def read(path):
    r = ET.parse(path).getroot(); comp = r.get("compressor") is not None
    p = r.find(".//Piece")
    pts = arr(p.find("Points/DataArray"), comp).astype(float)
    polys = p.find("Polys")
    conn = arr(polys.find("DataArray[@Name='connectivity']"), comp).astype(int)
    offs = arr(polys.find("DataArray[@Name='offsets']"), comp).astype(int)
    tris = []; s = 0
    for e in offs:
        f = conn[s:e]; s = e
        for k in range(1, len(f) - 1): tris.append((f[0], f[k], f[k + 1]))
    return pts, np.array(tris)
if __name__ == "__main__":
    G = r"C:\OpenSim 4.5\Geometry\\"
    for n in sys.argv[1:]:
        pts, tri = read(G + n + ".vtp")
        print(f"{n:12s} pts {len(pts):6d} tris {len(tri):6d} centre {np.round(pts.mean(0),4)} size {np.round(np.ptp(pts,0),4)}")
