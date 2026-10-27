#!/usr/bin/env python3
"""Check the whole object footprints; optionally plot placement on the saved map."""
import argparse
import json
from pathlib import Path
import yaml
from turtlebot4_lab05.scene_layout import validate_scene


def main():
    share = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--plot', type=Path)
    args = parser.parse_args()
    config = yaml.safe_load((share/'config/objects.yaml').read_text())
    try:
        from ament_index_python.packages import get_package_share_directory
        world = Path(get_package_share_directory('turtlebot4_ignition_bringup'))/'worlds/maze.sdf'
        if not world.is_file():
            raise FileNotFoundError
    except Exception:
        world = share.parents[1]/'.cache_lab05/worlds/maze.sdf'
    report = validate_scene(config, share/'maps/map.yaml', world)
    print(json.dumps(report, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
        from PIL import Image
        meta = yaml.safe_load((share/'maps/map.yaml').read_text())
        image = Image.open(share/'maps'/meta['image'])
        x, y, angle = meta['origin']
        if angle:
            raise ValueError('Plot only supports this zero-yaw maze map')
        colors = dict(person='#2ca02c', horse='#ff7f0e', chair='#1f77b4', bicycle='#9467bd', oak_tree='#2ca02c', fire_hydrant='#d62728', stop_sign='#7f7f7f')
        fig, ax = plt.subplots(figsize=(11, 8))
        ax.imshow(image, cmap='gray', vmin=0, vmax=255, origin='upper',
                  extent=(x,x+image.width*meta['resolution'],y,y+image.height*meta['resolution']))
        for o in report['objects']:
            color = colors.get(o['asset'], '#17becf')
            ax.add_patch(Circle((o['x'],o['y']),o['radius'],color=color,alpha=.3))
            ax.scatter(o['x'],o['y'],color=color,s=30)
            ax.annotate(o['name'].replace('lab05_',''),(o['x'],o['y']),xytext=(6,7),textcoords='offset points',fontsize=9)
        goals=yaml.safe_load((share/'config/waypoints.yaml').read_text())['goals']
        for i,g in enumerate(goals,1):
            ax.scatter(g['x'],g['y'],marker='*',s=130,c='red')
            ax.annotate('Goal '+str(i),(g['x'],g['y']),xytext=(6,-12),textcoords='offset points',weight='bold')
        ax.set(xlim=(-5.5,8.7),ylim=(-5,7),xlabel='map/world x (m)',ylabel='map/world y (m)',title=f'{len(report["objects"])} scattered demo objects; circles enclose body footprints')
        ax.set_aspect('equal');ax.grid(alpha=.12);fig.tight_layout()
        args.plot.parent.mkdir(parents=True,exist_ok=True)
        fig.savefig(args.plot,dpi=150);plt.close(fig)


if __name__ == '__main__':
    main()
