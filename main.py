# /src/main.py
import sys
import os
import time
import json
import numpy as np
from pathlib import Path

import src.ndi.tracker as ndi
from src.robot.controller import RobotController
from src.calib.calibration import HandEyeCalibration
from src.calib.navigator import Navigator
import src.utils.io as io
from src.utils.logger import get_logger
log = get_logger(__name__)

# ===========================
# State machine
# ===========================
class State:
    INIT          = "INIT"
    TRACKING      = "TRACKING"
    CALIBRATION   = "CALIBRATION"
    TELEOPERATION = "TELEOPERATION"
    EXIT          = "EXIT"

# ===========================
# Helper
# ===========================
# def is_valid_pose(pos, euler_deg, pos_limit=5000.0, euler_limit=360.0):
#     """기본 범위 체크로 비정상 포즈 필터링"""
#     if np.any(np.abs(pos) > pos_limit):
#         return False
#     if np.any(np.abs(euler_deg) > euler_limit):
#         return False
#     if np.any(np.isnan(pos)) or np.any(np.isnan(euler_deg)):
#         return False
#     return True

# ===========================
# CALIBRATION MODE
# ===========================
def run_calibration_mode(robot_controller, hostname, tools, rom_dir, encrypted, cipher,
                         robot_pose_file, dataset_root,
                         duration_sec, samples,
                         ):

    csv_file = io.delete_calibration_csv(robot_pose_file, dataset_root)

    robot_controller.move_to_home()

    # 로봇 포즈 JSON 로드 및 sample_number 기준 정렬
    with open(robot_pose_file, "r", encoding="utf-8") as f:
        pose_list = sorted(json.load(f), key=lambda x: x["sample_number"])

    api = ndi.connect_and_setup_calibration(hostname, tools, rom_dir, encrypted, cipher)

    try:
        for pose in pose_list:
            pose_id    = pose["sample_number"]
            target_pos = pose["pose"]

            # print(f"\n[CALIB] Pose {pose_id}: Collecting {samples} samples "
            #       f"(timeout: {duration_sec}s)...")
            log.info(f"\n[CALIB] Pose {pose_id}: Collecting {samples} samples "
                  f"(timeout: {duration_sec}s)...")
            
            robot_controller.movel_to_pose(target_pos, vel_ratio=10, acc_ratio=10, timeout=60)
            time.sleep(1)

            def on_sample(full_data):
                pos = full_data["position"]
                q   = full_data["quaternion"]
                err = full_data["error_mm"]

                tool_data = {
                    "q0": q["w"], "qx": q["x"],
                    "qy": q["y"], "qz": q["z"],
                    "tx": pos["x"], "ty": pos["y"], "tz": pos["z"],
                    "error": err,
                }

                pose_state = robot_controller.get_current_pose()
                robot_data = {
                    "x": pose_state[0], "y": pose_state[1], "z": pose_state[2],
                    "u": pose_state[3], "v": pose_state[4], "w": pose_state[5],
                }

                io.save_data_to_csv(csv_file, full_data["timestamp"], pose_id, tool_data, robot_data=robot_data)

            collected = ndi.collect_marker_samples(api, samples, duration_sec, pose_id, on_sample)

            # print(f"\n[INFO] Pose {pose_id} saved. samples={len(collected)}")
            log.info(f"\nPose {pose_id} saved. samples={len(collected)}")

            if len(collected) == 0:
                # print(f"[ERROR] Pose {pose_id}: NO VALID DATA!", flush=True)
                log.error(f"Pose {pose_id}: NO VALID DATA!")

            elif len(collected) < samples:
                # print(f"[WARNING] Pose {pose_id}: Only {len(collected)}/{samples} samples (timeout).", flush=True)
                log.warning(f"[WARNING] Pose {pose_id}: Only {len(collected)}/{samples} samples (timeout).")

    finally:
        api.stopTracking()
        robot_controller.move_to_home()
        # print("Calibration finished.", flush=True)
        log.info("Calibration finished.")

# ===========================
# TELEOPERATION MODE
# ===========================
def run_teleoperation_mode(robot_controller, hostname, ttool, rom_dir,
                           encrypted, cipher, calib_json_path,
                           x_offset, y_offset, z_offset):
    """
    Teleoperation 모드:
      1. ttool 마커 인식 → Navigator 로 로봇 EE 목표 좌표 변환
      2. Offset 적용 좌표 출력
      3. Enter 입력 → 로봇 이동 실행
      4. 이동 완료 → 완료 메시지 출력 후 다시 대기
      5. 'q' 입력 → INIT 상태로 복귀
      6. Ctrl+C → 강제 종료 후 INIT 복귀
    """

    print("\n" + "=" * 70)
    print("[TELEOPERATION] ttool 마커 인식 → 로봇 EE 이동 모드")
    print("  마커 인식 후 목표 좌표 출력 → Enter: 이동 / 'q': 종료")
    print("=" * 70)

    if not os.path.exists(calib_json_path):
        print(f"[ERROR] Calibration result not found: {calib_json_path}")
        return

    # Navigator 초기화
    try:
        nav = Navigator(calib_path=calib_json_path)
        print(f"[INFO] Navigator loaded (method: {nav.method}, unit: {nav.unit})")
    except Exception as e:
        print(f"[ERROR] Navigator init failed: {e}")
        return

    # ── NDI 연결 + 툴 로드 + 트래킹 시작 ─────────────────────────────
    try:
        api, ttool_handle = ndi.connect_and_setup_teleoperation(
            hostname, ttool, rom_dir, encrypted, cipher
        )
        # api, enabled_tools = ndi._connect_and_load_tools(hostname, [ttool], rom_dir, encrypted, cipher)
        # ttool_handle = f"{enabled_tools[0].transform.toolHandle:02X}"
    except RuntimeError as e:
        print(f"[ERROR] {e}")
        return

    if ttool_handle is None:
        return

    try:
        while True:

            print("-" * 70)
            print("[WAIT] 마커 인식 중...")

            raw_pose, reason = ndi.get_latest_valid_pose(
                api, ttool_handle, timeout_sec=10.0
            )

            if raw_pose is None:
                print(f"[WARNING] {reason}")
                sel = input("  재시도(Enter) / 종료(q): ").strip().lower()
                if sel == 'q':
                    break
                continue

            # Navigator 좌표 변환
            q0, qx, qy, qz = raw_pose['q0'], raw_pose['qx'], raw_pose['qy'], raw_pose['qz']
            tx, ty, tz = raw_pose['tx'], raw_pose['ty'], raw_pose['tz']

            result = nav.compute(q0, qx, qy, qz, tx, ty, tz)

            pos_arr = np.array([result['x'], result['y'], result['z']])
            euler_arr = np.array([result['u'], result['v'], result['w']])

            # if not is_valid_pose(pos_arr, euler_arr):
            #     print("[WARNING] 변환된 포즈가 유효 범위를 벗어났습니다. 재인식합니다.")
            #     continue

            pose = {**raw_pose, **result}

            # ───────────── Offset 적용 ─────────────
            x = pose['x'] + x_offset
            y = pose['y'] + y_offset
            z = pose['z'] + z_offset

            u = pose['u']
            v = pose['v']
            w = pose['w']

            target_pose = [x, y, z, u, v, w]

            # ───────────── 출력 ─────────────

            print(f"\n[DETECTED] {pose['ts_str']} | NDI Error: {pose['err']:.3f} mm")

            print("\n  [NDI Raw]")
            print(f"    Pos  x={pose['tx']:10.3f}  y={pose['ty']:10.3f}  z={pose['tz']:10.3f} (mm)")
            print(f"    Quat w={pose['q0']:.5f}  x={pose['qx']:.5f}  y={pose['qy']:.5f}  z={pose['qz']:.5f}")

            print("\n  [Navigator Result (Offset 미적용)]")
            print(f"    x={pose['x']:10.4f}  y={pose['y']:10.4f}  z={pose['z']:10.4f} (mm)")
            print(f"    u={pose['u']:10.4f}  v={pose['v']:10.4f}  w={pose['w']:10.4f} (deg)")

            print("\n  [Robot Target (Offset 적용, 실제 이동 좌표)]")
            print(f"    x={x:10.4f}  y={y:10.4f}  z={z:10.4f} (mm)")
            print(f"    u={u:10.4f}  v={v:10.4f}  w={w:10.4f} (deg)")

            print(f"\n  [INDY 포맷]")
            print(f"    [{x:.4f}, {y:.4f}, {z:.4f}, {u:.4f}, {v:.4f}, {w:.4f}]")

            print()

            # ───────────── 사용자 입력 ─────────────
            sel = input("  이동하려면 Enter / 재인식(r) / 종료(q): ").strip().lower()

            if sel == 'q':
                print("[INFO] Teleoperation mode 종료.")
                break

            elif sel == 'r':
                print("[INFO] 마커 재인식합니다.")
                continue

            # ───────────── 로봇 이동 ─────────────
            print("\n[MOVING] 로봇 이동 시작...")
            print(f"  목표: {target_pose}")

            try:
                robot_controller.movel_to_pose(
                    target_pose,
                    vel_ratio=10,
                    acc_ratio=10,
                    timeout=60
                )

                print("[DONE] 로봇 이동 완료.")

            except Exception as e:
                print(f"[ERROR] 로봇 이동 실패: {e}")

            print()

    except KeyboardInterrupt:

        print("\n[INFO] Teleoperation mode interrupted by user.")

    finally:

        api.stopTracking()
        print("[INFO] Tracking stopped.")

# ===========================
# MAIN
# ===========================
def main():
    STATE  = State.INIT
    config = io.load_config("config.json", base_dir=Path(__file__).resolve().parent)

    hostname  = config["ndi"]["hostname"]
    tools     = config["ndi"]["tools"]
    rom_dir   = config["ndi"]["rom_dir"]
    encrypted = config["ndi"]["encrypted"]
    cipher    = config["ndi"]["cipher"]

    robot_ip = config["robot"]["ip"]

    dataset_root    = config["dataset"]["dataset_root"]
    robot_pose_file = config["dataset"]["robot_pose_file"]

    ttool = config["teleoperation"]["ttool"]
    x_offset = config["teleoperation"]["x_offset"]
    y_offset = config["teleoperation"]["y_offset"]
    z_offset = config["teleoperation"]["z_offset"]

    paths = io.get_calibration_filepaths(robot_pose_file, dataset_root)

    while True:
        if STATE == State.INIT:
            print("\n" + "=" * 40)
            print("State: INIT")
            print("  1: Tracking Mode")
            print("  2: Calibration Mode")
            print("  3: Teleoperation Mode")
            print("  Q: Exit")
            print("=" * 40)
            sel = input("Select: ").strip()

            if sel == "1":
                STATE = State.TRACKING
            elif sel == "2":
                STATE = State.CALIBRATION
            elif sel == "3":
                STATE = State.TELEOPERATION
            elif sel.lower() == "q":
                STATE = State.EXIT
            else:
                print("[WARNING] Invalid selection.")

        elif STATE == State.TRACKING:
            ndi.run_tracking(hostname, tools, rom_dir, encrypted, cipher, print_tracking_data=ndi.print_tracking_data)
            STATE = State.INIT

        elif STATE == State.CALIBRATION:
            robot_controller = RobotController(robot_ip=robot_ip)
            try:
                robot_controller.indy.get_control_state()
            except Exception as e:
                print("[ERROR] Robot not connected:", e)
                STATE = State.INIT
                continue

            run_calibration_mode(
                robot_controller,
                hostname, tools, rom_dir, encrypted, cipher,
                robot_pose_file, dataset_root,
                duration_sec=config["calibration"]["duration_sec"],
                samples=config["calibration"]["samples"]
            )
            calib = HandEyeCalibration(csv_path=paths["csv"])
            calib.run()
            STATE = State.INIT

        elif STATE == State.TELEOPERATION:
            calib_json = paths["json"]

            robot_controller = RobotController(robot_ip=robot_ip)
            try:
                robot_controller.indy.get_control_state()
            except Exception as e:
                print("[ERROR] Robot not connected:", e)
                STATE = State.INIT
                continue

            run_teleoperation_mode(
                robot_controller=robot_controller,
                hostname=hostname,
                ttool=ttool, tools=tools, rom_dir=rom_dir,
                encrypted=encrypted, cipher=cipher,
                calib_json_path=calib_json,
                x_offset= x_offset, y_offset= y_offset, z_offset= z_offset
            )
            STATE = State.INIT

        elif STATE == State.EXIT:
            print("Exit.")
            break

if __name__ == "__main__":
    main()