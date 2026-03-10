"""Calibration validation script.

Compares converted robot poses against the recorded robot poses in a CSV file.
Position error is reported as Euclidean distance in mm.
Rotation error is reported as geodesic distance between orientations in degrees.
"""

import argparse
import csv
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

from src.calib.navigator import Navigator


def load_csv(csv_path: str) -> list:
    rows = []
    with open(csv_path, newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def wrap_deg(delta: float) -> float:
    return (delta + 180.0) % 360.0 - 180.0


def rotation_error_deg(pred, true_row) -> float:
    r_pred = R.from_euler('ZYX', [pred['w'], pred['v'], pred['u']], degrees=True)
    r_true = R.from_euler('ZYX', [
        float(true_row['w']), float(true_row['v']), float(true_row['u'])
    ], degrees=True)
    r_err = r_pred.as_matrix().T @ r_true.as_matrix()
    angle = np.arccos(np.clip((np.trace(r_err) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.degrees(angle))


def compare_poses(csv_path: str, calib_path: str):
    rows = load_csv(csv_path)

    try:
        nav = Navigator(calib_path=calib_path)
    except FileNotFoundError:
        print(f"[ERROR] Calibration file not found: {calib_path}")
        sys.exit(1)

    print(f"Calibration loaded (method: {nav.method}, unit: {nav.unit})")
    print(f"CSV loaded ({len(rows)} poses)\n")

    width = 11
    sep = '=' * 118
    thin = '-' * 118

    print(sep)
    print(f"{'Pose':^6} | {'Item':^10} | "
          f"{'x (mm)':>{width}} {'y (mm)':>{width}} {'z (mm)':>{width}} "
          f"{'u (deg)':>{width}} {'v (deg)':>{width}} {'w (deg)':>{width}}")
    print(sep)

    errors_pos = []
    errors_rot = []

    def fmt(value):
        return f"{value:>{width}.4f}"

    for row in rows:
        pose_id = row['pose_id']
        pred = nav.compute(
            float(row['q0']), float(row['qx']), float(row['qy']), float(row['qz']),
            float(row['tx']), float(row['ty']), float(row['tz'])
        )

        rx = float(row['x'])
        ry = float(row['y'])
        rz = float(row['z'])
        ru = float(row['u'])
        rv = float(row['v'])
        rw = float(row['w'])

        dx = pred['x'] - rx
        dy = pred['y'] - ry
        dz = pred['z'] - rz
        du = wrap_deg(pred['u'] - ru)
        dv = wrap_deg(pred['v'] - rv)
        dw = wrap_deg(pred['w'] - rw)

        pos_err = float(np.linalg.norm([dx, dy, dz]))
        rot_err = rotation_error_deg(pred, row)
        errors_pos.append(pos_err)
        errors_rot.append(rot_err)

        print(f"{pose_id:^6} | {'pred':^10} | "
              f"{fmt(pred['x'])} {fmt(pred['y'])} {fmt(pred['z'])} "
              f"{fmt(pred['u'])} {fmt(pred['v'])} {fmt(pred['w'])}")
        print(f"{'':^6} | {'true':^10} | "
              f"{fmt(rx)} {fmt(ry)} {fmt(rz)} {fmt(ru)} {fmt(rv)} {fmt(rw)}")
        print(f"{'':^6} | {'delta':^10} | "
              f"{fmt(dx)} {fmt(dy)} {fmt(dz)} {fmt(du)} {fmt(dv)} {fmt(dw)}")
        print(f"{'':^6} | {'metric':^10} | position={pos_err:.4f} mm   rotation={rot_err:.4f} deg")
        print(thin)

    arr_pos = np.array(errors_pos, dtype=float)
    arr_rot = np.array(errors_rot, dtype=float)

    print()
    print('=' * 56)
    print('Summary')
    print('=' * 56)
    print(f"Position mean: {arr_pos.mean():.4f} mm")
    print(f"Position min : {arr_pos.min():.4f} mm")
    print(f"Position max : {arr_pos.max():.4f} mm")
    print(f"Position std : {arr_pos.std():.4f} mm")
    print()
    print(f"Rotation mean: {arr_rot.mean():.4f} deg")
    print(f"Rotation min : {arr_rot.min():.4f} deg")
    print(f"Rotation max : {arr_rot.max():.4f} deg")
    print(f"Rotation std : {arr_rot.std():.4f} deg")
    print('=' * 56)


def main():
    parser = argparse.ArgumentParser(description='Validate NDI-to-robot conversion results.')
    parser.add_argument(
        '--csv', default='./dataset/calibration/calibration_data_test.csv',
        help='Validation CSV path'
    )
    parser.add_argument(
        '--calib', default='./dataset/results/calibration_result_broad.json',
        help='Calibration result JSON path'
    )
    args = parser.parse_args()

    compare_poses(csv_path=args.csv, calib_path=args.calib)


if __name__ == '__main__':
    main()
