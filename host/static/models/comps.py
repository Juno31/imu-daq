from vtpread import read
import numpy as np, sys
def comps(pts, tri):
    n = len(pts); par = np.arange(n)
    def f(a):
        while par[a] != a: par[a] = par[par[a]]; a = par[a]
        return a
    for a, b, c in tri:
        for u, v in ((a, b), (a, c)):
            ru, rv = f(u), f(v)
            if ru != rv: par[ru] = rv
    # weld duplicate points too
    key = {}
    for i, p in enumerate(np.round(pts, 6)):
        k = tuple(p)
        if k in key:
            ru, rv = f(i), f(key[k]); par[ru] = rv
        else: key[k] = i
    roots = np.array([f(i) for i in range(n)])
    return roots
if __name__ == "__main__":
    pts, tri = read("C:/OpenSim 4.5/Geometry/" + sys.argv[1] + ".vtp")
    r = comps(pts, tri); u = np.unique(r)
    out = sorted([(pts[r == k].mean(0), (r == k).sum(), np.ptp(pts[r == k], 0)) for k in u], key=lambda x: x[0][1])
    print(len(u), "components")
    for c, n, s in out: print(np.round(c, 3), n, np.round(s, 3))
