#!/usr/bin/env python3
"""Reproduce report diagrams from saved map/config; no ROS graph or inference."""
from pathlib import Path
import argparse, json, shutil, math
import yaml
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def main():
 p=argparse.ArgumentParser();p.add_argument('--package',type=Path, default=Path(__file__).resolve().parent.parent);p.add_argument('--video', type=Path, help='Optional original local demo video');a=p.parse_args()
 root=a.package;out=Path(__file__).resolve().parent/'figures';out.mkdir(exist_ok=True)
 plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.titleweight':'bold','savefig.facecolor':'white'})
 meta=yaml.safe_load((root/'maps/map.yaml').read_text());im=Image.open(root/'maps'/meta['image'])
 x0,y0,_=meta['origin'];res=meta['resolution'];extent=[x0,x0+im.width*res,y0,y0+im.height*res]
 goals=yaml.safe_load((root/'config/waypoints.yaml').read_text())['goals']
 objects=yaml.safe_load((root/'config/objects.yaml').read_text())['objects']
 fig,ax=plt.subplots(figsize=(9.5,8.0),layout='constrained')
 ax.imshow(im,cmap='gray',vmin=0,vmax=255,extent=extent,origin='upper',interpolation='nearest')
 palette=['#279968','#dd842c','#4477aa','#4477aa','#aa4499','#cc4455','#279968','#4477aa']
 for i,(o,c) in enumerate(zip(objects,palette),1):
  x,y=o['pose'][:2];dx,dy,_=o['collision_size'];r=max(o['footprint_radius'],math.hypot(dx,dy)/2)
  ax.add_patch(Circle((x,y),r,facecolor=c,edgecolor=c,alpha=.30,lw=1.2))
  ax.text(x,y,str(i),ha='center',va='center',weight='bold',fontsize=10,color='black',bbox={'boxstyle':'circle,pad=.13','fc':'white','ec':c,'lw':1.2})
 for i,g in enumerate(goals,1):
  x,y,t=g['x'],g['y'],g['yaw'];ax.scatter(x,y,marker='*',s=190,c='#004e87',edgecolors='white',linewidths=.5,zorder=4)
  ax.arrow(x,y,.7*math.cos(t),.7*math.sin(t),width=.035,head_width=.17,color='#004e87',length_includes_head=True,zorder=3)
  off={1:(-.1,-.70),2:(-.15,-.68),3:(.35,.35)}[i]
  ax.text(x+off[0],y+off[1],f'G{i}',color='#004e87',weight='bold',fontsize=12,bbox={'fc':'white','ec':'none','pad':1.2})
 ax.set(xlim=(-5.5,8.3),ylim=(-5.0,7.4),xlabel='x (m) — frame map',ylabel='y (m) — frame map')
 ax.set_aspect('equal');ax.grid(alpha=.14,lw=.5)
 fig.savefig(out/'map_goals_objects.pdf');fig.savefig(out/'map_goals_objects.png',dpi=150);plt.close(fig)
 for source,name in [('scattered_rviz_during_navigation.png','rviz_navigation.png'),('camera_person_horse_chair.png','yolo_observed.png')]:
  shutil.copy2(root/'validation'/source,out/name)
 # Preserve the actual historical trajectory image exactly; captions identify its old scene.
 shutil.copy2(root/'validation/actual_trajectory.png',out/'trajectory_historical.png')
 summary=json.loads((root/'validation/scattered_demo_summary.json').read_text())
 counts=summary['counts'];fig,ax=plt.subplots(figsize=(7.5,3.6),layout='constrained')
 cats=['Ảnh annotated','Header khớp','Inference khi di chuyển'];nums=[counts['annotated'],counts['header_matches'],counts['inference_while_moving']]
 bars=ax.barh(cats,nums,color=['#4477aa','#279968','#dd842c'],height=.58)
 for bar,n in zip(bars,nums):ax.text(n+9,bar.get_y()+bar.get_height()/2,str(n),va='center',weight='bold')
 ax.set_xlim(0,830);ax.set_xlabel('Số mẫu observer ghi nhận — lượt lịch sử 15/10/2026');ax.invert_yaxis();ax.spines[['top','right']].set_visible(False);ax.grid(axis='x',alpha=.15);ax.set_axisbelow(True)
 fig.savefig(out/'parallel_evidence.pdf');plt.close(fig)
 if a.video:
  import cv2
  cap=cv2.VideoCapture(str(a.video))
  if not cap.isOpened():raise ValueError('Cannot open local video')
  for seconds,filename,crop in [(484.6666666666667,'video_navigation.jpg',None),(949.9466666666667,'video_bicycle.jpg',(83,780,454,1070))]:
   cap.set(cv2.CAP_PROP_POS_MSEC,seconds*1000);ok,frame=cap.read()
   if not ok:raise ValueError(f'Missing video frame at {seconds}')
   image=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
   if crop:image=image.crop(crop)
   image.save(out/filename,quality=97 if crop else 94)
  cap.release()
 print('Report figures generated:',out)
if __name__=='__main__':main()
