"""Read-only geometry validation; writes results/previews under validation/."""
from pathlib import Path
import json, hashlib
import numpy as np
import cadquery as cq
from manifold3d import Manifold, Mesh
import imu_jig as j

OUT = j.HERE / 'validation'
OUT.mkdir(exist_ok=True)

def stl(path):
    raw = path.read_bytes()
    n = int.from_bytes(raw[80:84], 'little')
    dt = np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
    assert len(raw) == 84 + n * 50
    a = np.frombuffer(raw[84:],dtype=dt)['vertices']
    v, ix = np.unique(a.reshape(-1,3),axis=0,return_inverse=True)
    m = Manifold(Mesh(v.astype(np.float32),ix.reshape(-1,3).astype(np.uint32)))
    return m, a

def solid_mesh(s):
    v,t = s.tessellate(0.005,0.1)
    mesh = Mesh(np.array([p.toTuple() for p in v],dtype=np.float32),np.array(t,dtype=np.uint32))
    mesh.merge()
    m = Manifold(mesh)
    if str(m.status()) != 'Error.NoError':
        raise RuntimeError(str(m.status()))
    return m

actual, triangles = stl(j.HERE/'imu_jig.stl')
generated = j.build_jig()
shape = cq.importers.importStep(str(j.STEP_FILE)).val()
solids = shape.Solids()
pcb = max(solids,key=lambda s:s.BoundingBox().xlen*s.BoundingBox().ylen)
b = pcb.BoundingBox()
origin = np.array([b.xmin,b.ymin,b.zmin])
parts = [solid_mesh(s).translate((-origin).tolist()) for s in solids]
idx,x,y = next(j.pocket_positions())
offset = np.array([x+j.ORG[0],y+j.ORG[1],j.BODY_Z-j.RECESS])
assembled = Manifold.batch_boolean(parts, __import__('manifold3d').OpType.Add)
placed = assembled.translate(offset.tolist())
overlap = actual ^ placed
seated_volume = overlap.volume()
insertion = []
for dz in [0,0.1,0.5,1.0,1.5,2.0,2.5,3.0]:
    insertion.append({'lift_mm':dz,'rigid_overlap_mm3':(actual ^ placed.translate([0,0,dz])).volume()})
rotated = assembled.translate([-j.BOARD_X/2,-j.BOARD_Y/2,0]).rotate([0,0,180]).translate([j.BOARD_X/2,j.BOARD_Y/2,0]).translate(offset.tolist())
v,t = j.mesh_arrays(actual)
dv,dt = j.mesh_arrays(placed)
# Detail crop: clipping geometric solid rather than a whole-jig preview.
clip = j.box(x-4,y-4,34,x+j.PX+4,y+j.PY+4,45)
detail = actual ^ clip
cv,ct = j.mesh_arrays(detail)
j.render([(cv[ct],(0.65,0.68,0.73)),(dv[dt],(0.15,0.55,0.35))],OUT/'fit_detail.png',50,-60,'Actual STL + official Adafruit STEP: seated position')

# Section through mounting hole makes the snap shoulder and axial gap visible.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
hx,hy = j.HOLES[0]
cx,cy = offset[0]+hx,offset[1]+hy
section = actual.slice(float(offset[2]+0.8))
fig,ax = plt.subplots(figsize=(7,6),dpi=150)
for poly in section.to_polygons():
    a=np.asarray(poly)
    if len(a):ax.plot(a[:,0]-cx,a[:,1]-cy,color='gray',linewidth=0.8)
theta=np.linspace(0,2*np.pi,200)
ax.plot(1.25*np.cos(theta),1.25*np.sin(theta),color='green',label='PCB hole diameter 2.50 mm')
ax.set(xlim=(-2,2),ylim=(-2,2),xlabel='X from hole centre (mm)',ylabel='Y from hole centre (mm)',title='Actual STL section through seated PCB')
ax.set_aspect('equal');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(OUT/'pin_section.png');plt.close(fig)
result = {
    'stl_sha256':hashlib.sha256((j.HERE/'imu_jig.stl').read_bytes()).hexdigest(),
    'mesh_status':str(actual.status()),'mesh_bounds_mm':[triangles.min((0,1)).tolist(),triangles.max((0,1)).tolist()],
    'source_vs_STL_symmetric_difference_mm3':(actual-generated).volume()+(generated-actual).volume(),
    'board':{k:v for k,v in j.B.items() if k!='mesh'},
    'pocket_mm':[j.PX,j.PY], 'nominal_side_clearance_mm':[(j.PX-j.BOARD_X)/2,(j.PY-j.BOARD_Y)/2],
    'seated_intersection_mm3':seated_volume,'rotated_180_intersection_mm3':(actual^rotated).volume(),
    'all_pockets_seated_intersections_mm3':[
        {'pocket':i,'volume':(actual ^ assembled.translate([px+j.ORG[0],py+j.ORG[1],j.BODY_Z-j.RECESS])).volume()}
        for i,px,py in j.pocket_positions()],
    'insertion_rigid_intersections':insertion,'board_bottom_z_mm':float(offset[2]),
    'board_highest_z_mm':float(offset[2]+j.B['H']), 'top_reference_z_mm':j.BODY_Z,
    'pin_diameter_mm':j.PIN_D,'barb_diameter_mm':j.BARB_D,'axial_retention_gap_mm':0.1,
    'negative_wall_datum_shift_mm':[-j.ORG[0],-j.ORG[1]],
    'negative_wall_datum_intersection_mm3':(actual ^ placed.translate([-j.ORG[0],-j.ORG[1],0])).volume(),
    'cross_check':j.cross_check(j.B,j._load_brd_json(j.DIM_DIR/'pcb_geometry.json')),
    'limitations':['Rigid geometry only; no material deformation/FEA or physical insertion test','No mating cable plugs or user-installed headers in official board STEP','STL interpreted as mm; source generator confirms units']}
(OUT/'fit_results.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ['board','cross_check']},indent=2))
