#!/usr/bin/env python3
"""Build a Y-up OBJ circuit from the included traced sketch boundaries.

Requires Python 3, NumPy and SciPy. Matplotlib is needed only for --preview.
Example: python build_track.py --length 30 --output . --preview
"""
from __future__ import annotations
import argparse
import json
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial import Delaunay, cKDTree


def area(p):
    return .5 * np.sum(p[:, 0] * np.roll(p[:, 1], -1) - p[:, 1] * np.roll(p[:, 0], -1))


def contains(points, polygon):
    """Even-odd point-in-polygon, used for triangulation filtering."""
    points = np.atleast_2d(points)
    x, y = points.T
    inside = np.zeros(len(points), dtype=bool)
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        if abs(b[1] - a[1]) < 1e-12:
            continue
        cross = ((a[1] > y) != (b[1] > y)) & (x < (b[0]-a[0]) * (y-a[1]) / (b[1]-a[1]) + a[0])
        inside ^= cross
    return inside


def resample(p, spacing):
    p = np.vstack([p, p[0]])
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    t = np.linspace(0, d[-1], max(32, int(np.ceil(d[-1] / spacing))), endpoint=False)
    out = np.column_stack([np.interp(t, d, p[:, i]) for i in range(2)])
    return out if area(out) > 0 else out[::-1]


def check_simple(p):
    """Reject proper intersections between non-adjacent boundary segments."""
    def orient(a, b, c):
        return (b[..., 0]-a[..., 0])*(c[..., 1]-a[..., 1]) - (b[..., 1]-a[..., 1])*(c[..., 0]-a[..., 0])
    for i in range(len(p)):
        j = np.arange(i+2, len(p))
        if i == 0:
            j = j[j != len(p)-1]
        if not len(j):
            continue
        a, b = p[i], p[(i+1) % len(p)]
        c, d = p[j], p[(j+1) % len(p)]
        cross = (orient(a,b,c)*orient(a,b,d) < -1e-12) & (orient(c,d,a)*orient(c,d,b) < -1e-12)
        if np.any(cross):
            raise ValueError('Self-intersecting boundary')


def triangulate_ring(outer, inner):
    """Create a conforming Delaunay mesh by splitting missing constraint edges.

    Every outer/inner segment must exist in the triangulation before filtering;
    this prevents triangles bridging over the island or leaving the track.
    """
    loops = [outer.copy(), inner.copy()]
    for iteration in range(12):
        points = np.vstack(loops)
        tri = Delaunay(points)
        edges = {tuple(sorted((int(a), int(b)))) for f in tri.simplices for a,b in zip(f,np.roll(f,-1))}
        missing = []
        offset = 0
        for p in loops:
            missing.append([i for i in range(len(p)) if tuple(sorted((offset+i,offset+(i+1)%len(p)))) not in edges])
            offset += len(p)
        if not any(missing):
            centroids = points[tri.simplices].mean(axis=1)
            faces = tri.simplices[contains(centroids,loops[0]) & ~contains(centroids,loops[1])]
            expected = area(loops[0]) - area(loops[1])
            t = points[faces]
            actual = .5*np.abs((t[:,1,0]-t[:,0,0])*(t[:,2,1]-t[:,0,1])-(t[:,1,1]-t[:,0,1])*(t[:,2,0]-t[:,0,0])).sum()
            if abs(actual-expected) > max(1e-7, expected*1e-8):
                raise ValueError(f'Triangulation area mismatch: {actual} vs {expected}')
            return points, faces, loops, expected
        for k,p in enumerate(loops):
            ids = set(missing[k])
            loops[k] = np.array([q for i in range(len(p)) for q in ([p[i],(p[i]+p[(i+1)%len(p)])/2] if i in ids else [p[i]])])
    raise ValueError('Could not recover all boundary constraints')


def solid_floor(points, top_faces, thickness):
    n = len(points)
    top = np.column_stack([points[:,0],np.zeros(n),points[:,1]])
    vertices = np.vstack([top, top + [0,-thickness,0]])
    # Delaunay triangles are counterclockwise in XZ, which points downward in XYZ.
    faces = []
    for f in top_faces:
        a,b,c = map(int,f)
        normal = np.cross(top[b]-top[a],top[c]-top[a])
        faces.append([a,c,b] if normal[1] < 0 else [a,b,c])
    edge_counts = Counter(tuple(sorted((a,b))) for f in faces for a,b in zip(f,np.roll(f,-1)))
    top_oriented = np.asarray(faces,dtype=int)
    faces += [[int(c+n),int(b+n),int(a+n)] for a,b,c in top_oriented]
    for f in top_oriented:
        for a,b in zip(f,np.roll(f,-1)):
            if edge_counts[tuple(sorted((a,b)))] == 1:
                faces += [[int(b),int(a),int(a+n)],[int(b),int(a+n),int(b+n)]]
    return dict(name='Track_Surface',material='Asphalt',vertices=vertices,faces=np.asarray(faces),normals=None)


def tube(p, radius, sides, name):
    tangent = np.roll(p,-1,axis=0)-np.roll(p,1,axis=0)
    tangent /= np.linalg.norm(tangent,axis=1)[:,None]
    horizontal = np.column_stack([-tangent[:,1],np.zeros(len(p)),tangent[:,0]])
    theta = np.arange(sides)*2*np.pi/sides
    normals = horizontal[:,None,:]*np.cos(theta)[None,:,None] + np.array([0,1,0])[None,None,:]*np.sin(theta)[None,:,None]
    center = np.column_stack([p[:,0],np.full(len(p),radius),p[:,1]])
    vertices = (center[:,None,:]+radius*normals).reshape(-1,3)
    faces=[]
    for i in range(len(p)):
        for j in range(sides):
            a=i*sides+j; b=((i+1)%len(p))*sides+j
            c=((i+1)%len(p))*sides+(j+1)%sides; d=i*sides+(j+1)%sides
            faces += [[a,b,c],[a,c,d]]
    faces=np.asarray(faces)
    normal_test=np.cross(vertices[faces[0,1]]-vertices[faces[0,0]],vertices[faces[0,2]]-vertices[faces[0,0]])
    if np.dot(normal_test,normals.reshape(-1,3)[faces[0]].mean(axis=0))<0:
        faces=faces[:,::-1]
    return dict(name=name,material='Barrier_Red',vertices=vertices,faces=faces,normals=normals.reshape(-1,3))


def validate(mesh):
    v,f = mesh['vertices'],mesh['faces']
    if not np.isfinite(v).all() or f.min()<0 or f.max()>=len(v):
        raise ValueError('Invalid OBJ geometry')
    face_cross = np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
    if np.min(np.linalg.norm(face_cross,axis=1))<1e-12:
        raise ValueError('Degenerate triangle')
    edges=Counter(tuple(sorted((int(a),int(b)))) for face in f for a,b in zip(face,np.roll(face,-1)))
    directed=Counter((int(a),int(b)) for face in f for a,b in zip(face,np.roll(face,-1)))
    if any(c!=2 for c in edges.values()):
        raise ValueError('Mesh is not watertight')
    if any(directed[(b,a)]!=c for (a,b),c in directed.items()):
        raise ValueError('Inconsistent winding')
    volume = float(np.einsum('ij,ij->i',v[f[:,0]],np.cross(v[f[:,1]],v[f[:,2]])).sum()/6)
    if volume<=0:
        raise ValueError('Mesh points inward')
    return dict(vertices=len(v),triangles=len(f),watertight=True,consistent_winding=True,volume_m3=round(volume,6))


def write_obj(path,meshes):
    with path.open('w') as out:
        out.write('# Sketch circuit. Units: metres. Y up. Road top Y=0. Sketch top is -Z.\n')
        out.write('# Three independently closed meshes: road slab and two tube barriers.\nmtllib sketch_track.mtl\n')
        voff=noff=0
        for mesh in meshes:
            v,f=mesh['vertices'],mesh['faces']
            out.write(f"\no {mesh['name']}\nusemtl {mesh['material']}\n")
            for q in v:
                out.write('v %.7f %.7f %.7f\n'%tuple(q))
            n=mesh['normals']
            if n is None:
                n=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
                n/=np.linalg.norm(n,axis=1)[:,None]
            for q in n:
                out.write('vn %.7f %.7f %.7f\n'%tuple(q))
            out.write('s 1\n' if mesh['normals'] is not None else 's off\n')
            for i,face in enumerate(f):
                normalids=face+noff+1 if mesh['normals'] is not None else [i+noff+1]*3
                out.write('f '+' '.join(f'{int(vi)+voff+1}//{int(ni)}' for vi,ni in zip(face,normalids))+'\n')
            voff+=len(v); noff+=len(n)
    path.with_suffix('.mtl').write_text('''# Simple diffuse materials; no texture files required.
newmtl Asphalt
Ka 0.08 0.09 0.11
Kd 0.20 0.23 0.27
Ks 0.05 0.05 0.05
Ns 12
d 1.0
illum 2

newmtl Barrier_Red
Ka 0.18 0.04 0.03
Kd 0.88 0.22 0.13
Ks 0.18 0.18 0.18
Ns 30
d 1.0
illum 2
''')


def render(output,meshes,metadata):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from matplotlib.collections import PolyCollection
    fig=plt.figure(figsize=(13,10.5),facecolor='#f3f2ef')
    fig.text(.055,.94,'YOUR SKETCH, IN 3D',fontsize=23,weight='bold',color='#1e2936')
    fig.text(.055,.903,f"{metadata['length_m']:.1f} m long  /  {metadata['width_m']:.1f} m wide  /  two continuous barriers",fontsize=12,color='#536170')
    ax=fig.add_axes([.04,.12,.35,.73]);ax.set_facecolor('#f3f2ef')
    floor=meshes[0];v=floor['vertices'];f=floor['faces']
    top=np.all(np.abs(v[f,1])<1e-8,axis=1)
    ax.add_collection(PolyCollection(v[f[top]][:,:,[0,2]],facecolor='#333d47',edgecolor='none',antialiaseds=False,rasterized=True))
    for m in meshes[1:]:
        # Project the actual tube triangle mesh rather than a substituted outline.
        vv=m['vertices'];ff=m['faces'];polys=vv[ff][:,:,[0,2]]
        ax.add_collection(PolyCollection(polys,facecolor='#df5037',edgecolor='none',antialiaseds=False,rasterized=True))
    bounds=np.vstack([m['vertices'] for m in meshes])
    ax.set_xlim(bounds[:,0].min()-1,bounds[:,0].max()+1)
    ax.set_ylim(bounds[:,2].max()+1,bounds[:,2].min()-1);ax.set_aspect('equal');ax.axis('off')
    ax.text(.5,-.045,'TOP VIEW',transform=ax.transAxes,ha='center',fontsize=10,color='#536170',weight='bold')
    a3=fig.add_axes([.38,.16,.60,.66],projection='3d',computed_zorder=False)
    a3.set_facecolor('#f3f2ef')
    light=np.array([-.45,-.35,.82]);light/=np.linalg.norm(light)
    elev,azim=np.deg2rad([51,-64])
    view_direction=np.array([np.cos(elev)*np.cos(azim),np.cos(elev)*np.sin(azim),np.sin(elev)])
    for m in meshes:
        vv=m['vertices'][:,[0,2,1]];ff=m['faces'];polys=vv[ff]
        normal=np.cross(polys[:,1]-polys[:,0],polys[:,2]-polys[:,0]);normal/=np.linalg.norm(normal,axis=1)[:,None]
        # Axis permutation reverses handedness relative to OBJ coordinates.
        normal=-normal
        # Back-face culling avoids painter-order artifacts from the road underside.
        visible=normal@view_direction>0
        polys=polys[visible];normal=normal[visible]
        intensity=.54+.46*np.clip(normal@light,0,1)
        base=np.array([.22,.26,.30] if m['name']=='Track_Surface' else [.88,.28,.18])
        a3.add_collection3d(Poly3DCollection(polys,facecolors=np.clip(intensity[:,None]*base,0,1),edgecolors='none',linewidths=0,antialiaseds=False,zorder=2 if m['name']=='Track_Surface' else 3))
    width=metadata['width_m'];length=metadata['length_m']
    a3.set_xlim(-width/2-1,width/2+1);a3.set_ylim(-length/2-1,length/2+1);a3.set_zlim(-.1,1)
    a3.set_box_aspect([width+2,length+2,1.1]);a3.view_init(elev=51,azim=-64)
    a3.set_axis_off();a3.set_proj_type('ortho')
    fig.text(.68,.135,'PERSPECTIVE VIEW',ha='center',fontsize=10,color='#536170',weight='bold')
    fig.text(.055,.061,'Scale is assumed; the sketch supplied no dimensions. The small top gap has been closed.',fontsize=10,color='#536170')
    fig.text(.055,.036,'Rendered from the generated mesh. AutoDRIVE / Unity integration has not been run here.',fontsize=9,color='#6c7680')
    fig.savefig(output/'sketch_track_preview.png',dpi=150,facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--length',type=float,default=30,help='Outer barrier centerline extent in metres (default 30).')
    parser.add_argument('--barrier-diameter',type=float,default=.1524)
    parser.add_argument('--floor-thickness',type=float,default=.10)
    parser.add_argument('--spacing',type=float,default=.12,help='Boundary sampling in metres.')
    parser.add_argument('--sides',type=int,default=16)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent)
    parser.add_argument('--preview',action='store_true')
    args=parser.parse_args()
    if min(args.length,args.barrier_diameter,args.floor_thickness,args.spacing)<=0 or args.sides<8:
        parser.error('Dimensions must be positive and --sides must be at least 8.')
    source=json.loads(Path(__file__).with_name('track_boundaries.json').read_text())
    outer=np.array(source['outer_pixels']);inner=np.array(source['inner_pixels'])
    origin=(outer.min(axis=0)+outer.max(axis=0))/2
    scale=args.length/np.ptp(outer[:,1])
    outer=resample((outer-origin)*scale,args.spacing)
    inner=resample((inner-origin)*scale,args.spacing)
    check_simple(outer);check_simple(inner)
    if not contains(inner,outer).all():
        raise ValueError('Inner island leaves the outer boundary.')
    radius=args.barrier_diameter/2
    min_separation=cKDTree(outer).query(inner)[0].min()
    if min_separation<3*args.barrier_diameter:
        raise ValueError('Barrier diameter is too large for this circuit scale.')
    points,faces,loops,road_area=triangulate_ring(outer,inner)
    meshes=[solid_floor(points,faces,args.floor_thickness),tube(outer,radius,args.sides,'Outer_Barrier'),tube(inner,radius,args.sides,'Inner_Barrier')]
    checks={m['name']:validate(m) for m in meshes}
    metadata={
        'name':'Sketch Track','units':'metres','up_axis':'Y','image_top_direction':'-Z',
        'road_top_y':0,'length_m':round(float(np.ptp(outer[:,1])),3),'width_m':round(float(np.ptp(outer[:,0])),3),
        'overall_length_with_barriers_m':round(float(np.ptp(outer[:,1])+args.barrier_diameter),3),
        'barrier_diameter_m':args.barrier_diameter,'floor_thickness_m':args.floor_thickness,
        'road_area_to_barrier_centrelines_m2':round(float(road_area),3),
        'minimum_sampled_boundary_separation_m':round(float(min_separation),3),
        'minimum_sampled_clearance_after_barriers_m':round(float(min_separation-args.barrier_diameter),3),
        'approximate':True,'scale_assumption':f'{args.length:g} m length; source sketch has no dimensions.',
        'edits':'Closed the outer stroke at the top, removed the short pen overshoot, and smoothed pixel-level irregularities.',
        'unity_runtime_tested':False,'objects':checks,
        'track_boundaries':{'outer_xz':outer.round(6).tolist(),'inner_xz':inner.round(6).tolist()}
    }
    args.output.mkdir(parents=True,exist_ok=True)
    write_obj(args.output/'sketch_track.obj',meshes)
    (args.output/'track_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    if args.preview:
        render(args.output,meshes,metadata)
    print(json.dumps({k:v for k,v in metadata.items() if k!='track_boundaries'},indent=2))


if __name__=='__main__':
    main()
