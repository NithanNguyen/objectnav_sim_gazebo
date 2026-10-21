import sys,re,math,numpy as np,trimesh,yaml,matplotlib,xml.etree.ElementTree as ET
matplotlib.use('Agg'); import matplotlib.pyplot as plt
root,surf,out=sys.argv[1:4]
trials=sys.argv[4] if len(sys.argv)>4 else None
def unit(p):
    m=re.search(r'<unit[^>]*meter="([\d.eE+-]+)"',open(p,errors='ignore').read(5000)); return float(m.group(1)) if m else 1.0
def T(pose):
    r,p,y=pose[3:]; cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    M=np.eye(4); M[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]; M[:3,3]=pose[:3]; return M
W=ET.parse(root+'/worlds/small_house.world').getroot().find('world')
cache={}
def wmesh(mm):
    base=mm.find('include/uri').text.strip().replace('model://aws_robomaker_residential_','')
    import glob
    f=glob.glob(f"{root}/models/aws_robomaker_residential_{base}/meshes/*visual*.DAE")
    if not f: return base,None
    if f[0] not in cache:
        g=trimesh.util.concatenate(trimesh.load(f[0],force='scene').dump()); g.apply_scale(unit(f[0])); cache[f[0]]=g
    g=cache[f[0]].copy(); g.apply_transform(T([float(v) for v in mm.find('pose').text.split()])); return base,g
fig,ax=plt.subplots(figsize=(12,7.6),dpi=130)
SECT=('HouseWallB','RoomWall','Door_01','FoldingDoor','RoomWindow')
FOOT=('KitchenCabinet','Refrigerator','Wardrobe','CookingBench','FitnessEquipment','TV_0')
for mm in W.findall('model'):
    if mm.find('include/uri') is None: continue
    base,g=wmesh(mm)
    if g is None: continue
    if base.startswith(SECT):
        sec=g.section(plane_origin=[0,0,1.0],plane_normal=[0,0,1])
        if sec is None: continue
        c='#222' if base.startswith(('HouseWall','RoomWall')) else '#8ab'
        for e in sec.discrete: ax.plot(e[:,0],e[:,1],color=c,lw=1.6 if c=='#222' else 1.0)
    elif base.startswith(FOOT):
        lo,hi=g.bounds; ax.add_patch(plt.Rectangle(lo[:2],*(hi-lo)[:2],fc='none',ec='#888',lw=.8,ls='--'))
        ax.text(*(lo[:2]+hi[:2])/2,base.replace('_01','').replace('_02',''),fontsize=6,color='#666',ha='center',va='center')
S=yaml.safe_load(open(surf))['surfaces']
cols={'Bed_01':'#4C78A8','SofaC_01':'#F58518','CoffeeTable_01':'#54A24B','KitchenTable_01':'#E45756','NightStand_01':'#72B7B2','TVCabinet_01':'#B279A2','ReadingDesk_01':'#9D755D','ChairA_01':'#BAB0AC','ChairD_01':'#BAB0AC','BalconyTable_01':'#EECA3B','ShoeRack_01':'#FF9DA6'}
for s in S:
    x0,x1=s['x'];y0,y1=s['y']
    ax.add_patch(plt.Rectangle((x0,y0),x1-x0,y1-y0,fc=cols.get(s['type'],'#ccc'),ec='k',lw=.5,alpha=.7))
    if 'Chair' not in s['type']:
        ax.text((x0+x1)/2,(y0+y1)/2,f"{s['type'].replace('_01','')}\n{s['z']:.2f} m",ha='center',va='center',fontsize=7)
for name,(x,y) in {'Tablet_01':(1.57,-1.78),'Tableware_01':(7.15,0.98),'KitchenUtensils_01':(9.29,-5.02)}.items():
    ax.plot(x,y,'kx',ms=8,mew=2); ax.text(x+.12,y-.35,name,fontsize=7)
ROOMS={'Phong ngu':(-6.0,0.2),'Phong nho (TV)':(-6.0,-3.2),'Phong khach':(1.0,0.2),'Phong kinh':(1.1,5.1),'Bep + ban an':(7.0,-1.2)}
for k,(x,y) in ROOMS.items(): ax.text(x,y,k,fontsize=10,color='#1f4e79',ha='center',fontweight='bold',alpha=.85)
if trials:
    T=yaml.safe_load(open(trials))
    for s0 in T['start_poses']:
        ax.add_patch(plt.Circle((s0['x'],s0['y']),T.get('robot_radius_m',0.3),fc='#2ca02c',alpha=.35,ec='#2ca02c'))
        ax.annotate('',xy=(s0['x']+0.5*math.cos(s0['yaw']),s0['y']+0.5*math.sin(s0['yaw'])),xytext=(s0['x'],s0['y']),arrowprops=dict(arrowstyle='->',color='#2ca02c',lw=1.5))
        ax.text(s0['x'],s0['y']-0.6,s0['id'],ha='center',fontsize=9,color='#2ca02c',fontweight='bold')
    for q in T['placements']:
        c='#d62728' if q['visibility']=='visible' else '#7f7f7f'
        ax.plot(q['x'],q['y'],'o',ms=6,mfc=c,mec='w',mew=.8)
        ax.text(q['x']+0.12,q['y']+0.12,q['id'],fontsize=7,color=c,fontweight='bold')
ax.set_aspect('equal'); ax.set_xlim(-10,10); ax.set_ylim(-6.2,6.2); ax.grid(alpha=.25)
ax.set_xlabel('x (m, world)'); ax.set_ylabel('y (m, world)')
ax.set_title('AWS Small House (world frame). Tuong den, cua/cua so xanh (cat o z = 1,0 m); o mau = be mat dat vat + do cao;\n'
             'x = vat gay nhieu AWS; cham do = vi tri de thay, cham xam = bi che (P01-P10); vong xanh la = vi tri xuat phat S1, S2',fontsize=9)
plt.tight_layout(); plt.savefig(out); print('ok')
