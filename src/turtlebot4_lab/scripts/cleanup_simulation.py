#!/usr/bin/env python3
"""List/stop only Ignition processes with this lab's explicit transport partition."""
import argparse
import os
import signal
import time
from pathlib import Path


def matching():
    matches=[]
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():
            continue
        try:
            command=item.joinpath('cmdline').read_bytes().replace(b'\0',b' ').decode().strip()
            env=item.joinpath('environ').read_bytes().split(b'\0')
            if command.startswith('ign gazebo') and b'IGN_PARTITION=turtlebot4_lab_64' in env and b'ROS_DOMAIN_ID=64' in env:
                matches.append((int(item.name),command))
        except (OSError,UnicodeError):
            pass
    return matches


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stop',action='store_true',help='Stop listed lab processes after Ctrl+C on lab launch terminals')
    args=p.parse_args()
    processes=matching()
    for pid,command in processes:
        print(pid,command,flush=True)
    if not args.stop:
        print('Inspection only. Use --stop after stopping simulation launch.')
        return 0
    for sig,delay in [(signal.SIGINT,2),(signal.SIGTERM,2),(signal.SIGKILL,0.5)]:
        for pid,command in processes:
            # Recheck partition, domain and command each time (avoid PID reuse).
            if (pid,command) in matching():
                try:
                    os.kill(pid,sig)
                except ProcessLookupError:
                    pass
        time.sleep(delay)
    residual=matching()
    print('Remaining lab Ignition processes:',len(residual))
    return 1 if residual else 0


if __name__=='__main__':
    raise SystemExit(main())
