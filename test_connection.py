#!/usr/bin/env python3
from __future__ import annotations
import argparse
def main():
    p=argparse.ArgumentParser(); p.add_argument('--port'); p.add_argument('--baud',type=int); p.add_argument('--camera-only',action='store_true'); p.add_argument('--no-depth',action='store_true'); a=p.parse_args(); ok=True
    if not a.camera_only:
        try:
            from serial_comm import FlightController
            fc=FlightController(a.port,a.baud); print(f'[OK] MAVLink heartbeat on {fc.port}'); print(f'[INFO] GPS fix={fc.get_gps_status()} battery={fc.get_battery_voltage():.2f}V'); fc.close()
        except Exception as exc: print(f'[FAIL] FC: {exc}'); ok=False
    try:
        from vision import VisionProcessor
        vp=VisionProcessor(); f=vp.read_bottom_frame()
        if f is None: raise RuntimeError('bottom camera opened but no frame')
        print(f'[OK] bottom camera: {f.shape[1]}x{f.shape[0]}')
        if not a.no_depth: print(f'[OK] depth backend active; obstacle={vp.detect_dynamic_obstacle()}')
        vp.close()
    except Exception as exc: print(f'[FAIL] vision/depth: {exc}'); ok=False
    return 0 if ok else 1
if __name__=='__main__': raise SystemExit(main())
