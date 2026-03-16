"""
online_estimation.py – 실시간 마커 기반 로봇 위치 추정

[개요]
  NDI 마커를 연속적으로 트래킹하면서 로봇의 EE 위치를 실시간으로 추정하고,
  실제 로봇 위치(ground truth)와 비교하여 오차를 계산·경고·기록한다.

[파이프라인 (while loop 기반)]
  NDI 트래킹 → Navigator 좌표 변환 → 로봇 ground truth 취득
  → 오차 계산 → 콘솔 출력 → CSV 저장

[실행 예시]
  python online_estimation.py
  python online_estimation.py --calib dataset/results/calibration_result_broad.json
                              --out   dataset/logs/online_log.csv
                              --pos-threshold 5.0
                              --rot-threshold 3.0
[종료]
  Ctrl+C 로 루프를 중단하고 통계 요약을 출력한다.
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R


# ══════════════════════════════════════════════════════════════════════════════
# 프로젝트 루트 자동 탐색 (config.json 기준)
# ══════════════════════════════════════════════════════════════════════════════

def _find_project_root() -> Path:
    """config.json 또는 src/ 디렉토리가 있는 프로젝트 루트를 탐색한다."""
    here = Path(__file__).resolve().parent
    for candidate in [here] + [here.parents[i] for i in range(5)]:
        if (candidate / "config.json").exists():
            return candidate
    for candidate in [here] + [here.parents[i] for i in range(5)]:
        if (candidate / "src").is_dir():
            return candidate
    return Path.cwd()


PROJECT_ROOT = _find_project_root()

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# ── 프로젝트 내부 모듈 ───────────────────────────────────────────────────────
from src.calib.navigator import Navigator
from src.utils.logger import get_logger, configure_logging
from src.utils.io import load_config

configure_logging()
log = get_logger(__name__)


# ══════════════════════════════════════════════════════════════════════════════
# CSV 로그 헤더 정의
# ══════════════════════════════════════════════════════════════════════════════

LOG_CSV_HEADER = [
    # 시간 정보
    "timestamp",
    # NDI 원시 측정값 (쿼터니언 + 위치)
    "q0", "qx", "qy", "qz",
    "tx", "ty", "tz",
    "ndi_error_mm",
    # 추정 위치 (Navigator 출력)
    "pred_x", "pred_y", "pred_z",
    "pred_u", "pred_v", "pred_w",
    # 실제 로봇 위치 ground truth (controller.get_current_pose())
    "real_x", "real_y", "real_z",
    "real_u", "real_v", "real_w",
    # 위치 오차 (mm)
    "err_x", "err_y", "err_z",
    "pos_err_mm",
    # 자세 오차 (deg)
    "err_u_wrap", "err_v_wrap", "err_w_wrap",   # Euler 축별 참고값
    "rot_err_geodesic_deg",                      # geodesic distance (신뢰값)
    # 기타
    "gimbal_lock_warning",
]


# ══════════════════════════════════════════════════════════════════════════════
# 오차 계산 모듈 (navigator_test.py 기반)
# ══════════════════════════════════════════════════════════════════════════════

def _euler_indy_to_rotmat(u_deg: float, v_deg: float, w_deg: float) -> np.ndarray:
    """
    Neuromeka INDY Euler → 회전행렬.
    컨벤션: u=Rx, v=Ry, w=Rz, extrinsic XYZ = intrinsic ZYX
    """
    return R.from_euler('ZYX', [w_deg, v_deg, u_deg], degrees=True).as_matrix()


def _geodesic_rotation_error_deg(R_pred: np.ndarray, R_real: np.ndarray) -> float:
    """
    두 회전행렬 사이의 geodesic distance (단위: deg).
    공식: theta = arccos( (trace(R_pred.T @ R_real) - 1) / 2 )

    - Euler angle wrapping / gimbal lock 에 무관
    - 항상 0 ~ 180 deg 범위 반환
    """
    R_diff  = R_pred.T @ R_real
    cos_val = np.clip((np.trace(R_diff) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_val)))


def _wrap_angle(deg: float) -> float:
    """각도를 -180 ~ +180 범위로 wrapping한다."""
    return (deg + 180.0) % 360.0 - 180.0


def _is_gimbal_lock(v_deg: float, threshold_deg: float = 85.0) -> bool:
    """v(Ry) 가 ±threshold 이상이면 ZYX Euler 에서 gimbal lock 발생 가능."""
    return abs(abs(v_deg) - 90.0) < (90.0 - threshold_deg)


def compute_errors(pred: dict, real_xyzuvw: list) -> dict:
    """
    추정 포즈와 실측 포즈 사이의 위치·자세 오차를 계산한다.

    Parameters
    ----------
    pred         : Navigator.compute() 반환 dict (x, y, z, u, v, w 포함)
    real_xyzuvw  : [x, y, z, u, v, w] – 로봇 실제 EE 좌표

    Returns
    -------
    dict 키:
        err_x/y/z            위치 축별 오차 (mm)
        pos_err_mm           Euclidean 위치 오차 norm (mm)
        err_u/v/w_wrap       Euler 축별 오차, wrapping 보정 (참고용)
        rot_err_geodesic_deg 회전행렬 geodesic 오차 (deg) ← 신뢰값
        gimbal_lock_warning  gimbal lock 의심 여부 (bool)
    """
    rx, ry, rz, ru, rv, rw = real_xyzuvw

    # 위치 오차: 예측 - 실측
    dx = pred["x"] - rx
    dy = pred["y"] - ry
    dz = pred["z"] - rz

    # 자세 오차: Euler wrapping 보정 (참고용)
    du_wrap = _wrap_angle(pred["u"] - ru)
    dv_wrap = _wrap_angle(pred["v"] - rv)
    dw_wrap = _wrap_angle(pred["w"] - rw)

    # 자세 오차: 회전행렬 geodesic distance (신뢰값)
    R_pred   = _euler_indy_to_rotmat(pred["u"], pred["v"], pred["w"])
    R_real   = _euler_indy_to_rotmat(ru, rv, rw)
    geodesic = _geodesic_rotation_error_deg(R_pred, R_real)

    # gimbal lock 경고 여부
    gimbal = _is_gimbal_lock(pred["v"]) or _is_gimbal_lock(rv)

    return dict(
        err_x=dx, err_y=dy, err_z=dz,
        pos_err_mm=float(np.linalg.norm([dx, dy, dz])),
        err_u_wrap=du_wrap, err_v_wrap=dv_wrap, err_w_wrap=dw_wrap,
        rot_err_geodesic_deg=geodesic,
        gimbal_lock_warning=gimbal,
    )


# ══════════════════════════════════════════════════════════════════════════════
# CSV 로거
# ══════════════════════════════════════════════════════════════════════════════

class CsvLogger:
    """
    실시간 로그를 CSV 파일에 한 행씩 append하는 래퍼 클래스.
    인스턴스 생성 시 헤더를 작성하고 기존 파일을 덮어쓴다(새 세션).
    """

    def __init__(self, path: str):
        self.path = path
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        # 기존 파일 덮어쓰기
        with open(self.path, "w", newline="", encoding="utf-8") as f:
            csv.DictWriter(f, fieldnames=LOG_CSV_HEADER).writeheader()
        log.info(f"CSV 로그 초기화: {self.path}")

    def write(self, row: dict):
        """한 행을 CSV 파일에 append한다."""
        with open(self.path, "a", newline="", encoding="utf-8") as f:
            csv.DictWriter(f, fieldnames=LOG_CSV_HEADER,
                           extrasaction='ignore').writerow(row)


# ══════════════════════════════════════════════════════════════════════════════
# 콘솔 출력 헬퍼
# ══════════════════════════════════════════════════════════════════════════════

_W = 10  # 숫자 필드 폭


def _fmt(v: float) -> str:
    return f"{v:>{_W}.4f}"


def print_frame(frame_idx: int, ts_str: str, pred: dict, real: list,
                errs: dict, ndi_err: float,
                pos_threshold: float, rot_threshold: float):
    """
    한 프레임의 추정·실측 포즈와 오차를 콘솔에 출력한다.
    오차가 임계값을 초과하면 WARNING 배너를 추가 출력한다.
    """
    rx, ry, rz, ru, rv, rw = real
    thin = '-' * 110

    print(f"\n[Frame {frame_idx:>5}]  {ts_str}  |  NDI sensor err: {ndi_err:.3f} mm")
    print(thin)
    print(f"  {'':6}  {'x (mm)':>{_W}} {'y (mm)':>{_W}} {'z (mm)':>{_W}}"
          f"  {'u (deg)':>{_W}} {'v (deg)':>{_W}} {'w (deg)':>{_W}}")
    print(f"  {'pred':6}  {_fmt(pred['x'])} {_fmt(pred['y'])} {_fmt(pred['z'])}"
          f"  {_fmt(pred['u'])} {_fmt(pred['v'])} {_fmt(pred['w'])}")
    print(f"  {'true':6}  {_fmt(rx)} {_fmt(ry)} {_fmt(rz)}"
          f"  {_fmt(ru)} {_fmt(rv)} {_fmt(rw)}")
    print(f"  {'delta':6}  {_fmt(errs['err_x'])} {_fmt(errs['err_y'])} {_fmt(errs['err_z'])}"
          f"  {_fmt(errs['err_u_wrap'])} {_fmt(errs['err_v_wrap'])} {_fmt(errs['err_w_wrap'])}")
    print(f"  {'metric':6}  pos_err={errs['pos_err_mm']:.4f} mm  "
          f"rot_err={errs['rot_err_geodesic_deg']:.4f} deg"
          + ("  [GIMBAL LOCK WARNING]" if errs['gimbal_lock_warning'] else ""))

    # 임계값 초과 시 강조 경고 출력
    if errs['pos_err_mm'] > pos_threshold:
        print(f"  >>> ⚠  POSITION ERROR {errs['pos_err_mm']:.4f} mm "
              f"> threshold {pos_threshold} mm <<<")
    if errs['rot_err_geodesic_deg'] > rot_threshold:
        print(f"  >>> ⚠  ROTATION ERROR {errs['rot_err_geodesic_deg']:.4f} deg "
              f"> threshold {rot_threshold} deg <<<")


def print_summary(pos_errs: list, rot_errs: list):
    """세션 종료 시 위치·자세 오차 통계 요약을 출력한다."""
    if not pos_errs:
        log.warning("수집된 데이터가 없어 요약을 출력할 수 없습니다.")
        return

    arr_pos = np.array(pos_errs, dtype=float)
    arr_rot = np.array(rot_errs, dtype=float)
    sep = '=' * 56

    print(f"\n{sep}")
    print("  세션 요약")
    print(sep)
    print(f"  수집 프레임  : {len(pos_errs)}")
    print(f"\n  Position Error (mm)")
    print(f"    mean : {arr_pos.mean():.4f}")
    print(f"    min  : {arr_pos.min():.4f}")
    print(f"    max  : {arr_pos.max():.4f}")
    print(f"    std  : {arr_pos.std():.4f}")
    print(f"\n  Rotation Error (deg, geodesic)")
    print(f"    mean : {arr_rot.mean():.4f}")
    print(f"    min  : {arr_rot.min():.4f}")
    print(f"    max  : {arr_rot.max():.4f}")
    print(f"    std  : {arr_rot.std():.4f}")
    print(sep)


# ══════════════════════════════════════════════════════════════════════════════
# 실시간 추정 메인 루프
# ══════════════════════════════════════════════════════════════════════════════

def run_online_estimation(
    calib_path:      str,
    out_csv:         str,
    pos_threshold:   float = 5.0,
    rot_threshold:   float = 3.0,
    poll_interval:   float = 0.05,
    ndi_timeout_sec: float = 0.5,
):
    """
    실시간 마커 기반 로봇 위치 추정 메인 루프.

    Parameters
    ----------
    calib_path      : 캘리브레이션 결과 JSON 경로 (절대 or 루트 기준 상대)
    out_csv         : 로그 CSV 저장 경로
    pos_threshold   : 위치 오차 경고 임계값 (mm)
    rot_threshold   : 자세 오차 경고 임계값 (deg)
    poll_interval   : 루프 주기 (초) – 실제 처리 시간이 이보다 길면 건너뜀
    ndi_timeout_sec : 단일 NDI 포즈 획득 타임아웃 (초)
    """

    # 경로 절대화
    if not os.path.isabs(calib_path):
        calib_path = str(PROJECT_ROOT / calib_path)
    if not os.path.isabs(out_csv):
        out_csv = str(PROJECT_ROOT / out_csv)

    log.section("ONLINE 실시간 위치 추정 모드")
    log.info(f"프로젝트 루트  : {PROJECT_ROOT}")
    log.info(f"Calib JSON     : {calib_path}")
    log.info(f"CSV 로그       : {out_csv}")
    log.info(f"Pos threshold  : {pos_threshold} mm")
    log.info(f"Rot threshold  : {rot_threshold} deg")
    log.info(f"Poll interval  : {poll_interval} s  (~{1/poll_interval:.0f} Hz)")

    # ── config 로드 ──────────────────────────────────────────────────────────
    try:
        config = load_config("config.json", base_dir=PROJECT_ROOT)
    except Exception as e:
        log.error(f"config.json 로드 실패: {e}")
        sys.exit(1)

    hostname  = config["ndi"]["hostname"]
    tools     = config["ndi"]["tools"]      # 기본: config["ndi"]["tools"] 추적
    rom_dir   = config["ndi"]["rom_dir"]
    encrypted = config["ndi"]["encrypted"]
    cipher    = config["ndi"]["cipher"]
    robot_ip  = config["robot"]["ip"]

    # ── Navigator 초기화 ─────────────────────────────────────────────────────
    try:
        nav = Navigator(calib_path=calib_path)
        log.success(f"Navigator 로드 완료 (method: {nav.method}, unit: {nav.unit})")
    except FileNotFoundError:
        log.error(f"캘리브레이션 파일을 찾을 수 없습니다: {calib_path}")
        sys.exit(1)

    # ── 로봇 컨트롤러 초기화 ─────────────────────────────────────────────────
    # controller.py 의 RobotController 사용
    # get_current_pose() → indy.get_control_state()['p'] → [x,y,z,u,v,w]
    try:
        from src.robot.controller import RobotController
        robot = RobotController(robot_ip=robot_ip)
        robot.indy.get_control_state()   # 연결 확인
        log.success(f"로봇 연결 성공 ({robot_ip})")
    except Exception as e:
        log.error(f"로봇 연결 실패: {e}")
        sys.exit(1)

    # ── NDI 연결 + 툴 로드 + 트래킹 시작 ────────────────────────────────────
    # config["ndi"]["tools"] 에 등록된 마커를 기본으로 추적
    try:
        import src.ndi.tracker as nd
        api = nd.connect_and_setup_calibration(
            hostname, tools, rom_dir, encrypted, cipher
        )
        log.success("NDI 트래킹 시작 완료")
    except Exception as e:
        log.error(f"NDI 연결 실패: {e}")
        sys.exit(1)

    # ── CSV 로거 초기화 ──────────────────────────────────────────────────────
    csv_logger = CsvLogger(out_csv)

    # ── 통계 누적 버퍼 ───────────────────────────────────────────────────────
    pos_errs: list = []
    rot_errs: list = []
    frame_idx: int = 0

    log.info("실시간 추정 루프 시작. 종료: Ctrl+C\n")

    # ══════════════════════════════════════════════════════════════════════════
    # 메인 while loop 파이프라인
    # ══════════════════════════════════════════════════════════════════════════
    try:
        while True:
            loop_start = time.time()

            # ── Step 1: NDI 마커 포즈 획득 ────────────────────────────────────
            # ttool_handle=None → 등록된 첫 번째 유효 마커를 사용
            # 로봇이 continuous하게 움직이는 상황이므로 타임아웃을 짧게 유지
            raw_pose, reason = nd.get_latest_valid_pose(
                api, ttool_handle=None, timeout_sec=ndi_timeout_sec
            )

            if raw_pose is None:
                # 마커 미인식: 경고 후 다음 루프로 넘어감
                log.warning(f"마커 미인식: {reason}")
                time.sleep(poll_interval)
                continue

            # ── Step 2: 타임스탬프 ────────────────────────────────────────────
            ts_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            # ── Step 3: NDI 원시값 파싱 ──────────────────────────────────────
            q0  = raw_pose["q0"];  qx = raw_pose["qx"]
            qy  = raw_pose["qy"];  qz = raw_pose["qz"]
            tx  = raw_pose["tx"];  ty = raw_pose["ty"];  tz = raw_pose["tz"]
            ndi_err = raw_pose.get("err", 0.0)

            # ── Step 4: Navigator 좌표 변환 ───────────────────────────────────
            # NDI 마커 측정값 → 로봇 EE 추정 좌표 (x,y,z,u,v,w)
            pred = nav.compute(q0, qx, qy, qz, tx, ty, tz)

            # ── Step 5: 로봇 ground truth 취득 ───────────────────────────────
            # controller.get_current_pose() = indy.get_control_state()['p']
            # 로봇 모션은 외부에서 제어되며 여기서는 상태만 읽는다
            try:
                real: list = robot.get_current_pose()
            except Exception as e:
                log.error(f"로봇 상태 취득 실패: {e}")
                time.sleep(poll_interval)
                continue

            # ── Step 6: 오차 계산 ────────────────────────────────────────────
            errs = compute_errors(pred, real)

            # ── Step 7: 통계 누적 ────────────────────────────────────────────
            pos_errs.append(errs["pos_err_mm"])
            rot_errs.append(errs["rot_err_geodesic_deg"])
            frame_idx += 1

            # ── Step 8: 콘솔 출력 + 임계값 초과 경고 ─────────────────────────
            print_frame(
                frame_idx, ts_str, pred, real, errs, ndi_err,
                pos_threshold, rot_threshold
            )

            # ── Step 9: CSV 로그 저장 ─────────────────────────────────────────
            csv_logger.write({
                "timestamp": ts_str,
                "q0": q0, "qx": qx, "qy": qy, "qz": qz,
                "tx": tx, "ty": ty, "tz": tz,
                "ndi_error_mm": ndi_err,
                "pred_x": pred["x"], "pred_y": pred["y"], "pred_z": pred["z"],
                "pred_u": pred["u"], "pred_v": pred["v"], "pred_w": pred["w"],
                "real_x": real[0],   "real_y": real[1],   "real_z": real[2],
                "real_u": real[3],   "real_v": real[4],   "real_w": real[5],
                **errs,
            })

            # ── Step 10: 루프 주기 조정 ──────────────────────────────────────
            elapsed = time.time() - loop_start
            sleep_t = max(0.0, poll_interval - elapsed)
            time.sleep(sleep_t)

    except KeyboardInterrupt:
        log.warning("\n사용자가 중단했습니다 (Ctrl+C).")

    finally:
        # NDI 트래킹 정지
        try:
            api.stopTracking()
            log.info("NDI 트래킹 정지 완료.")
        except Exception:
            pass

        # 세션 종료 통계 출력
        print_summary(pos_errs, rot_errs)
        log.success(f"CSV 로그 저장 완료: {out_csv}")


# ══════════════════════════════════════════════════════════════════════════════
# CLI 인터페이스
# ══════════════════════════════════════════════════════════════════════════════

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="실시간 마커 기반 로봇 위치 추정",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument("--calib",
        default="dataset/results/calibration_result_broad.json",
        help="캘리브레이션 결과 JSON (루트 기준 상대경로 또는 절대경로)\n"
             "기본: dataset/results/calibration_result_broad.json")
    p.add_argument("--out",
        default="dataset/logs/online_log.csv",
        help="CSV 로그 저장 경로\n기본: dataset/logs/online_log.csv")
    p.add_argument("--pos-threshold", type=float, default=5.0,
        help="위치 오차 경고 임계값 mm (기본: 5.0)")
    p.add_argument("--rot-threshold", type=float, default=3.0,
        help="자세 오차 경고 임계값 deg (기본: 3.0)")
    p.add_argument("--poll-interval", type=float, default=0.05,
        help="루프 폴링 간격 초 (기본: 0.05 → 약 20 Hz)")
    p.add_argument("--ndi-timeout", type=float, default=0.5,
        help="단일 NDI 포즈 획득 타임아웃 초 (기본: 0.5)")
    return p


def main():
    args = _build_parser().parse_args()
    run_online_estimation(
        calib_path      = args.calib,
        out_csv         = args.out,
        pos_threshold   = args.pos_threshold,
        rot_threshold   = args.rot_threshold,
        poll_interval   = args.poll_interval,
        ndi_timeout_sec = args.ndi_timeout,
    )


if __name__ == "__main__":
    main()