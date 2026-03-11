r"""
calibration_test.py – 캘리브레이션 성능 검증 스크립트

이 파일 위치: src/tests/calibration_test.py
프로젝트 루트의 config.json을 자동으로 탐색합니다.
--calib, --csv, --out 경로는 프로젝트 루트 기준 상대경로 또는 절대경로를 모두 허용합니다.

[자세 오차 계산 방식]
  Euler angle 단순 차이 대신 회전행렬 기반 geodesic distance 를 사용합니다.
    rot_err = arccos( (trace(R_pred.T @ R_real) - 1) / 2 )  [단위: deg]
  이 방식은 gimbal lock / angle wrapping 에 무관하게 항상 올바른 회전 오차를 반환합니다.
  Euler 축별 차이(du/dv/dw)는 참고용으로만 출력하며 오차 통계에는 사용하지 않습니다.

[모드]
  1) offline  : 기존 CSV 파일을 읽어 pose별 좌표 변환 → 실측값과 비교 출력
  2) online   : 로봇을 실제로 이동시키면서 실시간으로 마커 pose를 취득 → 변환 → 비교

[실행 위치]
  프로젝트 루트(config.json 이 있는 폴더) 또는 src/tests/ 어디서든 실행 가능합니다.

  cd C:\Users\...\AutoCalibration
  python src/tests/calibration_test.py offline
  python src/tests/calibration_test.py online

[사용법 예시]
  ## offline – 기본값
  python src/tests/calibration_test.py offline

  ## offline – 파일 직접 지정
  python src/tests/calibration_test.py offline ^
      --csv   dataset/calibration/calibration_data_test.csv ^
      --calib dataset/results/calibration_result_broad.json ^
      --out   dataset/calibration/offline_result.csv

  ## online – 기본값
  python src/tests/calibration_test.py online

  ## online – 캘리브 파일 지정
  python src/tests/calibration_test.py online ^
      --calib dataset/results/calibration_result_narrow.json ^
      --out   dataset/calibration/online_narrow_result.csv
"""

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R


# ══════════════════════════════════════════════════════════════════════════════
# 프로젝트 루트 탐색 (config.json 기준)
# ══════════════════════════════════════════════════════════════════════════════

def _find_project_root() -> Path:
    here = Path(__file__).resolve().parent
    # config.json 우선 탐색
    for candidate in [here] + [here.parents[i] for i in range(5)]:
        if (candidate / "config.json").exists():
            return candidate
    # fallback: src/ 디렉토리 기준
    for candidate in [here] + [here.parents[i] for i in range(5)]:
        if (candidate / "src").is_dir():
            return candidate
    return Path.cwd()


PROJECT_ROOT = _find_project_root()

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.calib.navigator import Navigator
from src.utils.logger import get_logger, configure_logging
from src.utils.io import load_config

configure_logging()
log = get_logger(__name__)


def _resolve(path_str: str) -> str:
    p = Path(path_str)
    if p.is_absolute():
        return str(p)
    return str(PROJECT_ROOT / p)

def _append_csv(path: str, row: dict, write_header: bool = False):
    """CSV 파일에 한 행 append."""
    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=RT_CSV_HEADER)

        if write_header:
            writer.writeheader()

        writer.writerow(row)

# ── 결과 CSV 헤더 ──────────────────────────────────────────────────────────────
RT_CSV_HEADER = [
    "timestamp", "pose_id",
    "q0", "qx", "qy", "qz", "tx", "ty", "tz", "ndi_error_mm",
    # 예측값 (Navigator 출력)
    "pred_x", "pred_y", "pred_z", "pred_u", "pred_v", "pred_w",
    # 실측값 (로봇)
    "real_x", "real_y", "real_z", "real_u", "real_v", "real_w",
    # 위치 오차
    "err_x", "err_y", "err_z",
    "pos_err_mm",
    # 자세 오차 – Euler 축별 참고값 + geodesic (올바른 오차)
    "err_u_wrap", "err_v_wrap", "err_w_wrap",   # wrapping 보정 Euler 차이 (참고용)
    "rot_err_geodesic_deg",                      # 회전행렬 기반 geodesic (신뢰값)
    # gimbal lock 경고
    "gimbal_lock_warning",
]


# ══════════════════════════════════════════════════════════════════════════════
# 회전 오차 계산 유틸
# ══════════════════════════════════════════════════════════════════════════════

def _euler_indy_to_rotmat(u_deg: float, v_deg: float, w_deg: float) -> np.ndarray:
    """Neuromeka INDY Euler(u=Rx, v=Ry, w=Rz, extrinsic XYZ = intrinsic ZYX) → 회전행렬."""
    return R.from_euler('ZYX', [w_deg, v_deg, u_deg], degrees=True).as_matrix()


def _geodesic_rotation_error_deg(R_pred: np.ndarray, R_real: np.ndarray) -> float:
    """
    두 회전행렬 사이의 geodesic distance (각도 오차, 단위: deg).
    공식: theta = arccos( (trace(R_pred.T @ R_real) - 1) / 2 )

    - Euler angle wrapping 에 무관
    - Gimbal lock 상태에도 올바른 값 반환
    - 항상 0 ~ 180 deg 범위
    """
    R_diff = R_pred.T @ R_real
    # 수치 오차로 인해 ±1 범위를 살짝 벗어날 수 있으므로 clamp
    cos_val = np.clip((np.trace(R_diff) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_val)))


def _wrap_angle(deg: float) -> float:
    """각도를 -180 ~ +180 범위로 wrapping."""
    return (deg + 180.0) % 360.0 - 180.0


def _is_gimbal_lock(v_deg: float, threshold_deg: float = 85.0) -> bool:
    """v(Ry) 가 ±90° 근처이면 ZYX Euler 에서 gimbal lock 발생."""
    return abs(abs(v_deg) - 90.0) < (90.0 - threshold_deg)


def _compute_errors(pred: dict, real_xyzuvw: list) -> dict:
    """
    위치 오차 + 자세 오차(geodesic) 계산.

    반환값:
        err_x/y/z          : 위치 축별 오차 (mm)
        pos_err_mm         : 위치 오차 norm (mm)
        err_u/v/w_wrap     : Euler 축별 오차, -180~+180 wrapping 보정 (참고용)
        rot_err_geodesic_deg : 회전행렬 geodesic distance (deg) ← 신뢰값
        gimbal_lock_warning: 예측값 또는 실측값에 gimbal lock 의심 여부
    """
    rx, ry, rz, ru, rv, rw = real_xyzuvw
    dx = pred["x"] - rx
    dy = pred["y"] - ry
    dz = pred["z"] - rz

    # Euler 축별 차이 (wrapping 보정)
    du_wrap = _wrap_angle(pred["u"] - ru)
    dv_wrap = _wrap_angle(pred["v"] - rv)
    dw_wrap = _wrap_angle(pred["w"] - rw)

    # 회전행렬 기반 geodesic 오차
    R_pred = _euler_indy_to_rotmat(pred["u"], pred["v"], pred["w"])
    R_real = _euler_indy_to_rotmat(ru, rv, rw)
    geodesic = _geodesic_rotation_error_deg(R_pred, R_real)

    # gimbal lock 경고 (v가 ±85° 이상이면 ZYX Euler 불안정)
    gimbal = _is_gimbal_lock(pred["v"]) or _is_gimbal_lock(rv)

    return dict(
        err_x=dx, err_y=dy, err_z=dz,
        pos_err_mm=float(np.linalg.norm([dx, dy, dz])),
        err_u_wrap=du_wrap, err_v_wrap=dv_wrap, err_w_wrap=dw_wrap,
        rot_err_geodesic_deg=geodesic,
        gimbal_lock_warning=gimbal,
    )


# ══════════════════════════════════════════════════════════════════════════════
# 출력 헬퍼
# ══════════════════════════════════════════════════════════════════════════════

def _print_pose_block(pose_id, pred: dict, real_xyzuvw: list,
                      ndi_err: float, ts_str: str = ""):
    rx, ry, rz, ru, rv, rw = real_xyzuvw
    errs = _compute_errors(pred, real_xyzuvw)

    width = 11
    sep  = '=' * 118
    thin = '-' * 118

    def fmt(value):
        return f"{value:>{width}.4f}"

    dx = errs['err_x']
    dy = errs['err_y']
    dz = errs['err_z']
    du = errs['err_u_wrap']
    dv = errs['err_v_wrap']
    dw = errs['err_w_wrap']

    print(f"{pose_id:^6} | {'pred':^10} | "
          f"{fmt(pred['x'])} {fmt(pred['y'])} {fmt(pred['z'])} "
          f"{fmt(pred['u'])} {fmt(pred['v'])} {fmt(pred['w'])}")
    print(f"{'':^6} | {'true':^10} | "
          f"{fmt(rx)} {fmt(ry)} {fmt(rz)} {fmt(ru)} {fmt(rv)} {fmt(rw)}")
    print(f"{'':^6} | {'delta':^10} | "
          f"{fmt(dx)} {fmt(dy)} {fmt(dz)} {fmt(du)} {fmt(dv)} {fmt(dw)}")
    print(f"{'':^6} | {'metric':^10} | position={errs['pos_err_mm']:.4f} mm   rotation={errs['rot_err_geodesic_deg']:.4f} deg")
    print(thin)


def _print_summary(pos_errs: list, rot_errs_geodesic: list):
    arr_p = np.array(pos_errs)
    arr_r = np.array(rot_errs_geodesic)

    print()
    print('=' * 56)
    print('Summary')
    print('=' * 56)
    print(f"Position mean: {arr_p.mean():.4f} mm")
    print(f"Position min : {arr_p.min():.4f} mm")
    print(f"Position max : {arr_p.max():.4f} mm")
    print(f"Position std : {arr_p.std():.4f} mm")
    print()
    print(f"Rotation mean: {arr_r.mean():.4f} deg")
    print(f"Rotation min : {arr_r.min():.4f} deg")
    print(f"Rotation max : {arr_r.max():.4f} deg")
    print(f"Rotation std : {arr_r.std():.4f} deg")
    print('=' * 56)



# ══════════════════════════════════════════════════════════════════════════════
# config.json 로드
# ══════════════════════════════════════════════════════════════════════════════

def _load_project_config() -> dict:
    return load_config("config.json", base_dir=PROJECT_ROOT)


# ══════════════════════════════════════════════════════════════════════════════
# 1. OFFLINE MODE
# ══════════════════════════════════════════════════════════════════════════════

def run_offline(csv_path: str, calib_path: str, out_csv: Optional[str] = None):
    csv_path   = _resolve(csv_path)
    calib_path = _resolve(calib_path)
    if out_csv:
        out_csv = _resolve(out_csv)

    log.section("OFFLINE 검증 모드")
    log.info(f"프로젝트 루트 : {PROJECT_ROOT}")
    log.info(f"CSV           : {csv_path}")
    log.info(f"Calib         : {calib_path}")

    if not os.path.exists(csv_path):
        log.error(f"CSV 파일이 없습니다: {csv_path}")
        sys.exit(1)

    rows = []
    with open(csv_path, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            rows.append(row)
    log.info(f"CSV 로드 완료 – {len(rows)} 행")

    try:
        nav = Navigator(calib_path=calib_path)
        log.info(f"Navigator 로드 완료 (method: {nav.method}, unit: {nav.unit})")
    except FileNotFoundError:
        log.error(f"캘리브레이션 파일을 찾을 수 없습니다: {calib_path}")
        sys.exit(1)

    need_header = True
    if out_csv:
        os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)
        if os.path.exists(out_csv):
            os.remove(out_csv)
        log.info(f"결과 CSV 저장 위치: {out_csv}")

    pos_errs, rot_errs_geo = [], []

    width = 11
    sep = '=' * 118
    thin = '-' * 118

    print(sep)
    print(f"{'Pose':^6} | {'Item':^10} | "
          f"{'x (mm)':>{width}} {'y (mm)':>{width}} {'z (mm)':>{width}} "
          f"{'u (deg)':>{width}} {'v (deg)':>{width}} {'w (deg)':>{width}}")
    print(sep)

    for row in rows:
        pose_id = row["pose_id"]
        q0  = float(row["q0"]);  qx = float(row["qx"])
        qy  = float(row["qy"]);  qz = float(row["qz"])
        tx  = float(row["tx"]);  ty = float(row["ty"]);  tz = float(row["tz"])
        ndi_err = float(row.get("error", 0.0))
        ts_str  = row.get("timestamp", "")

        pred = nav.compute(q0, qx, qy, qz, tx, ty, tz)
        real = [float(row["x"]), float(row["y"]), float(row["z"]),
                float(row["u"]), float(row["v"]), float(row["w"])]

        errs = _compute_errors(pred, real)
        pos_errs.append(errs["pos_err_mm"])
        rot_errs_geo.append(errs["rot_err_geodesic_deg"])

        _print_pose_block(pose_id, pred, real, ndi_err, ts_str)

        if out_csv:
            _append_csv(out_csv, dict(
                timestamp=ts_str, pose_id=pose_id,
                q0=q0, qx=qx, qy=qy, qz=qz, tx=tx, ty=ty, tz=tz,
                ndi_error_mm=ndi_err,
                pred_x=pred["x"], pred_y=pred["y"], pred_z=pred["z"],
                pred_u=pred["u"], pred_v=pred["v"], pred_w=pred["w"],
                real_x=real[0], real_y=real[1], real_z=real[2],
                real_u=real[3], real_v=real[4], real_w=real[5],
                **errs,
            ), write_header=need_header)
            need_header = False

    _print_summary(pos_errs, rot_errs_geo)
    if out_csv:
        log.success(f"결과 저장 완료: {out_csv}")


# ══════════════════════════════════════════════════════════════════════════════
# 2. ONLINE MODE
# ══════════════════════════════════════════════════════════════════════════════

def run_online(calib_path: str, out_csv: str):
    calib_path = _resolve(calib_path)
    out_csv    = _resolve(out_csv)

    log.section("ONLINE 실시간 검증 모드")
    log.info(f"프로젝트 루트 : {PROJECT_ROOT}")
    log.info(f"Calib         : {calib_path}")

    try:
        config = _load_project_config()
    except Exception as e:
        log.error(f"config.json 로드 실패: {e}")
        sys.exit(1)

    hostname  = config["ndi"]["hostname"]
    tools     = config["ndi"]["tools"]
    rom_dir   = config["ndi"]["rom_dir"]
    encrypted = config["ndi"]["encrypted"]
    cipher    = config["ndi"]["cipher"]
    robot_ip  = config["robot"]["ip"]
    pose_file = config["test"]["robot_pose_file"]
    log.info(f"Pose file     : {pose_file}")

    try:
        nav = Navigator(calib_path=calib_path)
        log.info(f"Navigator 로드 완료 (method: {nav.method}, unit: {nav.unit})")
    except FileNotFoundError:
        log.error(f"캘리브레이션 파일을 찾을 수 없습니다: {calib_path}")
        sys.exit(1)

    try:
        from src.robot.controller import RobotController
        robot = RobotController(robot_ip=robot_ip)
        robot.indy.get_control_state()
        log.success(f"로봇 연결 성공 ({robot_ip})")
    except Exception as e:
        log.error(f"로봇 연결 실패: {e}")
        sys.exit(1)

    try:
        import src.ndi.tracker as nd
        api = nd.connect_and_setup_calibration_tools(
            hostname, tools, rom_dir, encrypted, cipher
        )
        log.success("NDI 연결 및 트래킹 시작")
    except Exception as e:
        log.error(f"NDI 연결 실패: {e}")
        sys.exit(1)

    if not os.path.exists(pose_file):
        log.error(f"pose 파일 없음: {pose_file}")
        api.stopTracking()
        sys.exit(1)

    with open(pose_file, "r", encoding="utf-8") as f:
        pose_list = sorted(json.load(f), key=lambda x: x["sample_number"])
    log.info(f"pose 리스트 로드 완료 – {len(pose_list)} poses")

    os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)
    if os.path.exists(out_csv):
        os.remove(out_csv)
    log.info(f"결과 CSV 저장 위치: {out_csv}")

    pos_errs, rot_errs_geo = [], []
    need_header = True

    width = 11
    sep = '=' * 118
    thin = '-' * 118

    robot.move_to_home()

    print(sep)
    print(f"{'Pose':^6} | {'Item':^10} | "
          f"{'x (mm)':>{width}} {'y (mm)':>{width}} {'z (mm)':>{width}} "
          f"{'u (deg)':>{width}} {'v (deg)':>{width}} {'w (deg)':>{width}}")
    print(sep)

    try:
        for item in pose_list:
            pose_id    = item["sample_number"]
            target_pos = item["pose"]

            log.info(f"[Pose {pose_id}] 이동 중... {target_pos}")
            robot.movel_to_pose(target_pos, vel_ratio=10, acc_ratio=10, timeout=60)
            time.sleep(1.0)

            real = robot.get_current_pose()

            log.info(f"[Pose {pose_id}] NDI 마커 취득 중...")
            # calibration mode와 동일하게 핸들 필터링 없이(None) 첫 번째 유효 포즈 획득
            raw_pose, reason = nd.get_latest_valid_pose(api, None, timeout_sec=5.0)

            if raw_pose is None:
                log.warning(f"[Pose {pose_id}] 마커 취득 실패: {reason} → 스킵")
                continue

            q0  = raw_pose["q0"];  qx = raw_pose["qx"]
            qy  = raw_pose["qy"];  qz = raw_pose["qz"]
            tx  = raw_pose["tx"];  ty = raw_pose["ty"];  tz = raw_pose["tz"]
            ndi_err = raw_pose.get("err", 0.0)
            ts_str  = raw_pose.get("ts_str",
                                   datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"))

            pred = nav.compute(q0, qx, qy, qz, tx, ty, tz)
            errs = _compute_errors(pred, real)
            pos_errs.append(errs["pos_err_mm"])
            rot_errs_geo.append(errs["rot_err_geodesic_deg"])

            _print_pose_block(pose_id, pred, real, ndi_err, ts_str)

            _append_csv(out_csv, dict(
                timestamp=ts_str, pose_id=pose_id,
                q0=q0, qx=qx, qy=qy, qz=qz, tx=tx, ty=ty, tz=tz,
                ndi_error_mm=ndi_err,
                pred_x=pred["x"], pred_y=pred["y"], pred_z=pred["z"],
                pred_u=pred["u"], pred_v=pred["v"], pred_w=pred["w"],
                real_x=real[0], real_y=real[1], real_z=real[2],
                real_u=real[3], real_v=real[4], real_w=real[5],
                **errs,
            ), write_header=need_header)
            need_header = False

    except KeyboardInterrupt:
        log.warning("사용자가 중단했습니다.")

    finally:
        api.stopTracking()
        robot.move_to_home()
        log.info("트래킹 중지 / 홈 복귀 완료")

    if pos_errs:
        _print_summary(pos_errs, rot_errs_geo)
        log.success(f"결과 저장 완료: {out_csv}")
    else:
        log.warning("수집된 데이터가 없습니다.")


# ══════════════════════════════════════════════════════════════════════════════
# CLI
# ══════════════════════════════════════════════════════════════════════════════

def _build_parser():
    parser = argparse.ArgumentParser(
        description="캘리브레이션 성능 검증 (offline / online)",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    sub = parser.add_subparsers(dest="mode", required=True)

    p_off = sub.add_parser("offline", help="기존 CSV를 읽어 오프라인 검증")
    p_off.add_argument("--csv",
        default="dataset/calibration/calibration_data_test.csv",
        help="검증용 CSV 파일 경로 (루트 기준)\n기본: dataset/calibration/calibration_data_test.csv")
    p_off.add_argument("--calib",
        default="dataset/results/calibration_result_broad.json",
        help="캘리브레이션 결과 JSON 경로 (루트 기준)\n기본: dataset/results/calibration_result_broad.json")
    p_off.add_argument("--out", default=None,
        help="결과 CSV 저장 경로 (생략 시 저장 안 함)\n예) dataset/calibration/offline_result.csv")

    p_on = sub.add_parser("online", help="로봇 + NDI 연동 실시간 검증")
    p_on.add_argument("--calib",
        default="dataset/results/calibration_result_broad.json",
        help="캘리브레이션 결과 JSON 경로 (루트 기준)\n기본: dataset/results/calibration_result_broad.json")
    p_on.add_argument("--out",
        default="dataset/calibration/calibration_data_test.csv",
        help="결과 CSV 저장 위치 (루트 기준)\n기본: dataset/calibration/calibration_data_test.csv")

    return parser


def main():
    parser = _build_parser()
    args = parser.parse_args()

    if args.mode == "offline":
        run_offline(csv_path=args.csv, calib_path=args.calib, out_csv=args.out)
    elif args.mode == "online":
        run_online(calib_path=args.calib, out_csv=args.out)


if __name__ == "__main__":
    main()