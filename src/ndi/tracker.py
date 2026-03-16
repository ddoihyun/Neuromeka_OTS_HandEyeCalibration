import sys
import os
import time

from src.utils.logger import get_logger
log = get_logger(__name__)

# lib 폴더를 Python 경로에 추가 (ndi_vega_api 모듈 import를 위해)
script_dir   = os.path.dirname(os.path.abspath(__file__))  # → src/ndi/
project_root = os.path.dirname(os.path.dirname(script_dir)) # → AutoCalibration/
lib_path     = os.path.join(project_root, 'lib')

if os.path.exists(lib_path):
    sys.path.insert(0, lib_path)
    log.debug(f"Added Library to Python path: {lib_path}")
else:
    log.warning(f"lib directory not found at {lib_path}")

import ndi_vega_api
import argparse


# ===========================
# NDI 유틸
# ===========================

def on_error_print_debug_message(method_name, error_code):
    """
    API 함수 실행 후 반환된 error_code가 0이 아니면 
    해당 메서드 이름과 에러 메시지를 로그로 출력하는 디버그 함수
    """
    if isinstance(error_code, int) and error_code != 0:
        # print(f"{method_name} failed: {ndi_vega_api.CombinedApi.errorToString(error_code)}")
        log.error(f"{method_name} failed: {ndi_vega_api.CombinedApi.errorToString(error_code)}")
        return True
    return False

def validate_rom_files(tools, rom_dir):
    """연결 전 ROM 파일 존재 검증. 실패 시 None 반환."""
    validated = []
    for tool in filter(None, [t.strip() for t in tools]):
        tool_path = os.path.join(rom_dir, tool)
        if not os.path.isfile(tool_path):
            log.error(f"Cannot access ROM file: {tool_path}")
            return None
        log.success(f"Found ROM file: {tool_path}")
        validated.append(tool)
    return validated

def load_tool(api, tool_definition_file_path):
    """
    NDI Vega 장비에서 tool SROM 파일을 로드하고 사용 가능한 포트를 반환하는 초기화 함수
    """
    # portHandleRequest(hardwareType, systemType, toolType, portNumber, dummyTool)
    port_handle = api.portHandleRequest("********", "*", "1", "00", "**")
    
    if port_handle is None or port_handle < 0:
        # print("api.portHandleRequest failed")
        # log.error("api.portHandleRequest failed")
        on_error_print_debug_message("api.portHandleRequest", port_handle)
        return -1
    
    result = api.loadSromToPort(tool_definition_file_path, port_handle)

    # if result is not None and result != 0:
    #     # log.error(f"api.loadSromToPort failed: {ndi_vega_api.CombinedApi.errorToString(result)}") 
    #     return -1
    if on_error_print_debug_message("api.loadSromToPort", result):
        return -1
    
    # print(f"Loaded ROM to port {port_handle} successfully")
    log.success(f"Loaded ROM to port {port_handle} successfully")
    
    return port_handle

def get_tool_info(api, port_handle):
    """툴 정보 가져오기 (ID + Serial Number)"""
    info = api.portHandleInfo(f"{port_handle:02X}")
    return f"{info.getToolId()}.sn{info.getSerialNumber()}"

def initialize_and_enable_tools(api, enabled_tools):
    """로드된 툴들을 초기화하고 활성화"""
    # print("\nInitializing and enabling tools...")
    log.info("Initializing and enabling tools...")

    port_handles = api.portHandleSearchRequest(ndi_vega_api.PortHandleSearchRequestOption_NotInit)
    for port_handle_info in port_handles:
        port_handle = port_handle_info.getPortHandle()
        on_error_print_debug_message("api.portHandleInitialize", api.portHandleInitialize(port_handle))
        on_error_print_debug_message("api.portHandleEnable",     api.portHandleEnable(port_handle))

    port_handles = api.portHandleSearchRequest(ndi_vega_api.PortHandleSearchRequestOption_Enabled)
    for port_handle_info in port_handles:
        # print(port_handle_info.toString())
        log.debug(port_handle_info.toString()) 
        tool_data = ndi_vega_api.ToolData()
        port_handle_str = port_handle_info.getPortHandle()
        tool_data.transform.toolHandle = api.stringToInt(port_handle_str)
        tool_data.toolInfo = get_tool_info(api, tool_data.transform.toolHandle)
        enabled_tools.append(tool_data)

def decode_transform_status(t):
    """
    NDI Vega Transform 객체의 16-bit status 값을 해석하여 
    상태 정보를 사람이 읽기 쉬운 dict 형태로 반환

    Transform.status의 비트 구조:
        - Bits 0~7   : error code
        - Bit 8      : missing flag (툴이 시야에 없으면 True)
        - Bits 13~15 : face orientation (0~7)
    """
    status     = t.status
    error_code = status & 0x00FF
    missing    = bool(status & 0x0100)
    face       = (status & 0xE000) >> 12

    try:
        error_str = ndi_vega_api.TransformStatus_toString(error_code)
    except Exception:
        error_str = f"UNKNOWN({error_code:#02x})"

    return {
        "raw":        f"0x{status:04X}",
        "error_code": error_code,
        "error":      error_str,
        "missing":    missing,
        "face":       face,
    }

def extract_full_data_dict(tool_data, timestamp=None):
    """NDI Vega ToolData 객체의 status와 Tool 정보를 Python dict로 변환"""
    t    = tool_data.transform
    
    t_s  = tool_data.timespec_s
    t_ns = tool_data.timespec_ns
    precise_timestamp = t_s + t_ns * 1e-9

    info = decode_transform_status(t)

    return {
        "timestamp":   timestamp if timestamp is not None else precise_timestamp,
        "tool_handle": f"{t.toolHandle:02X}",
        "position":    {"x": t.tx,  "y": t.ty,  "z": t.tz},
        "quaternion":  {"w": t.q0,  "x": t.qx,  "y": t.qy,  "z": t.qz},
        "error_mm":    t.error,
        "status_raw":  info["raw"],
        "error_code":  info["error_code"],
        "error_str":   info["error"],
        "missing":     info["missing"],
        "face":        info["face"],
    }

def is_missing_frame(full_data: dict) -> bool:
    """NDI 프레임이 유효하지 않은 상태인지 판별"""
    return full_data["missing"]

# ===========================
# 공통 연결 헬퍼
# ===========================
def connect_and_setup(hostname, tools, rom_dir, encrypted, cipher):
    """
    NDI 연결 + ROM 로드 + 툴 초기화/활성화까지 공통 처리.

    Returns
    -------
    api : ndi_vega_api.CombinedApi 
        연결된 API 객체 (tracking 시작 전 상태)
    enabled_tools: list[ToolData]
        활성화된 툴 목록

    Raises
    ------
    RuntimeError : 연결 실패 시
    """
    validated = validate_rom_files(tools, rom_dir)
    if validated is None:
        raise RuntimeError("ROM file validation failed.")
    
    api      = ndi_vega_api.CombinedApi()
    protocol = ndi_vega_api.Protocol.SecureTCP if encrypted else ndi_vega_api.Protocol.TCP

    if api.connect(hostname, protocol, cipher) != 0:
        raise RuntimeError("NDI connection failed.")

    api.initialize()

    if tools:
        for tool in tools:
            tool_path = os.path.join(rom_dir, tool)
            load_tool(api, tool_path)

    enabled_tools = []
    initialize_and_enable_tools(api, enabled_tools)
    return api, enabled_tools

# ===========================
# Calibration 모드
# ===========================
def connect_and_setup_calibration(hostname, tools, rom_dir, encrypted, cipher):
    """
    캘리브레이션 모드용 NDI 연결 + 툴 로드 + 트래킹 시작.

    Returns
    -------
    api : ndi_vega_api.CombinedApi  (startTracking() 완료)
    """
    api, _ = connect_and_setup(hostname, tools, rom_dir, encrypted, cipher)
    api.startTracking()
    return api

def collect_marker_samples(api, samples, duration_sec, pose_id, on_sample):
    """
    캘리브레이션 모드에서 NDI 트래킹 데이터를 수집한다.

    Parameters
    ----------
    api          : ndi_vega_api.CombinedApi  (이미 startTracking() 된 상태)
    samples      : int   – 수집할 샘플 수
    duration_sec : float – 타임아웃(초)
    pose_id      : any   – 로그 출력용 포즈 ID
    on_sample    : callable(full_data) – 유효 샘플 1개당 호출되는 콜백.
                   False 반환 시 수집 중단.

    Returns
    -------
    collected : list[dict]
    """
    collected           = []
    start               = time.time()
    consecutive_missing = 0

    while len(collected) < samples:
        tool_data_list = api.getTrackingDataBX2()
        timestamp      = time.time()

        if not tool_data_list:
            time.sleep(0.05)
            continue

        for td in tool_data_list:
            full_data = extract_full_data_dict(td, timestamp)

            if is_missing_frame(full_data):
                consecutive_missing += 1
                if consecutive_missing >= 20:
                    # print(
                    #     f"\n[WARNING] Pose {pose_id}: Marker missing for "
                    #     f"{consecutive_missing} frames! Check marker visibility.",
                    #     flush=True,
                    # )
                    log.warning(
                        f"Pose {pose_id}: Marker missing for "
                        f"{consecutive_missing} frames! Check marker visibility."
                    )
                    consecutive_missing = 0
                continue

            consecutive_missing = 0

            pos = full_data["position"]
            q   = full_data["quaternion"]
            err = full_data["error_mm"]
            ts  = time.strftime("%H:%M:%S", time.localtime(full_data["timestamp"]))

            # print(
            #     f"{ts} | Pos(x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f}) | "
            #     f"Quat(w={q['w']:.4f}, x={q['x']:.4f}, y={q['y']:.4f}, z={q['z']:.4f}) | "
            #     f"Err={err:.4f}mm",
            #     flush=True,
            # )
            log.debug(
                f"{ts} | Pos(x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f}) | "
                f"Quat(w={q['w']:.4f}, x={q['x']:.4f}, y={q['y']:.4f}, z={q['z']:.4f}) | "
                f"Err={err:.4f}mm"
            )
            collected.append(full_data)

            if on_sample(full_data) is False:
                return collected

            # print(
            #     f"[PROGRESS] Pose {pose_id}: {len(collected)}/{samples} samples ",
            #     end='\r', flush=True,
            # )
            log.debug(f"Pose {pose_id}: {len(collected)}/{samples} samples collected")

            if len(collected) >= samples:
                break

        if time.time() - start > duration_sec:
            break

    return collected


# ===========================
# Teleoperation 모드
# ===========================
def connect_and_setup_tools(hostname, ttool, rom_dir, encrypted, cipher):
    """
    Teleoperation 모드용 NDI 연결 + 단일 툴(ttool) 로드 + 트래킹 시작.
 
    Returns
    -------
    (api, ttool_handle : str)  – 정상
    (api, None)                – 툴 활성화 실패 또는 핸들 미확정 시
    """

    try:
        api, enabled_tools = connect_and_setup(hostname, [ttool], rom_dir, encrypted, cipher)
    except RuntimeError as e:
        raise
 
    if not enabled_tools:
        log.error("No tools enabled. Check ROM file or marker connection.")
        return api, None
 
    ttool_key = os.path.splitext(ttool)[0].lower()  # '8700449.rom' → '8700449'
    matched = [
        t for t in enabled_tools
        if ttool_key in t.toolInfo.lower()
    ]
 
    if not matched:
        if len(enabled_tools) == 1:
            log.warning(
                f"toolInfo 매칭 실패({ttool_key!r}). "
                f"enabled_tools 가 1개뿐이므로 해당 툴을 ttool 로 간주합니다."
            )
            matched = enabled_tools
        else:
            log.error(
                f"ttool({ttool!r}) 핸들을 enabled_tools 에서 찾지 못했습니다. "
                f"toolInfo 목록: {[t.toolInfo for t in enabled_tools]}"
            )
            return api, None
 
    ttool_handle = f"{matched[0].transform.toolHandle:02X}"
    log.info(f"ttool handle: {ttool_handle!r}")
 
    api.startTracking()
    log.success(f"Tracking started. Tool: {ttool}")
    log.info("마커를 NDI 시야 내에 위치시키세요.")
 
    return api, ttool_handle

def get_latest_valid_pose(api, ttool_handle, timeout_sec=5.0):
    """
    NDI 트래킹 데이터에서 ttool_handle에 해당하는 유효한 최신 포즈를 획득한다.
    timeout_sec 내에 유효 포즈를 찾지 못하면 (None, reason)을 반환한다.

    Parameters
    ----------
    api          : ndi_vega_api.CombinedApi  (이미 startTracking() 된 상태)
    ttool_handle : str | None  – 필터링할 툴 핸들 문자열. None이면 전체 대상.
    timeout_sec  : float

    Returns
    -------
    (dict, None)  : 정상 포즈 딕셔너리
    (None, str)   : 실패 – reason에 원인 설명
    """
    deadline        = time.time() + timeout_sec
    seen_any_frame  = False
    seen_other_tool = False

    while time.time() < deadline:
        tool_data_list = api.getTrackingDataBX2()
        timestamp      = time.time()
        ts_str         = time.strftime("%H:%M:%S", time.localtime(timestamp))

        if not tool_data_list:
            time.sleep(0.05)
            continue

        seen_any_frame = True

        for td in tool_data_list:
            full_data   = extract_full_data_dict(td, timestamp)
            seen_handle = full_data.get("tool_handle")

            if ttool_handle is not None and seen_handle != ttool_handle:
                seen_other_tool = True
                continue

            if is_missing_frame(full_data):
                continue

            pos = full_data["position"]
            q   = full_data["quaternion"]

            return {
                "q0": q["w"], "qx": q["x"], "qy": q["y"], "qz": q["z"],
                "tx": pos["x"], "ty": pos["y"], "tz": pos["z"],
                "err":       full_data["error_mm"],
                "ts_str":    ts_str,
                "full_data": full_data,
            }, None

        time.sleep(0.05)

    # timeout
    if not seen_any_frame:
        reason = "NDI 트래킹 데이터가 수신되지 않습니다."
    elif seen_other_tool and ttool_handle is not None:
        reason = (f"ttool(handle={ttool_handle!r}) 이 NDI 시야에 없습니다. "
                  f"다른 마커만 감지되었습니다.")
    else:
        reason = "마커를 인식하지 못했습니다 (시야 밖이거나 missing 상태)."

    return None, reason


# ===========================
# Tracking 모드
# ===========================
def print_tracking_data(data):
    """트래킹 데이터 기본 출력 콜백 (CLI 및 Tracking 모드)."""
    pos = data["position"]
    q   = data["quaternion"]
    ts  = time.strftime("[%H:%M:%S]", time.localtime(data["timestamp"]))
    print(
        f"{ts} P{data['tool_handle']}: "
        f"Pos({pos['x']:.1f},{pos['y']:.1f},{pos['z']:.1f}) "
        f"Q({q['w']:.3f},{q['x']:.3f},{q['y']:.3f},{q['z']:.3f}) "
        f"E={data['error_mm']:.2f}mm | "
        f"raw={data['status_raw']} error={data['error_str']} "
        f"missing={data['missing']} face={data['face']}"
    )

def run_tracking(hostname, tools, rom_dir, encrypted, cipher, print_tracking_data):
    """
    Tracking 모드: 연결 → 트래킹 시작 → print_tracking_data 콜백으로 데이터 전달 → 종료.

    Parameters
    ----------
    print_tracking_data : callable(full_data) – missing이 아닌 프레임마다 호출
    """
    api, _ = connect_and_setup(hostname, tools, rom_dir, encrypted, cipher)
    api.startTracking()

    try:
        while True:
            tool_data_list = api.getTrackingDataBX2()
            # timestamp      = time.time()

            for td in tool_data_list:
                full_data = extract_full_data_dict(td)
                if not is_missing_frame(full_data):
                    print_tracking_data(full_data)

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        api.stopTracking()

def main():
    script_dir      = os.path.dirname(os.path.abspath(__file__))
    project_root    = os.path.dirname(os.path.dirname(script_dir))
    default_rom_dir = os.path.join(project_root, 'sroms')

    parser = argparse.ArgumentParser(description='NDI Vega Tracking')
    parser.add_argument('hostname', type=str,
                        help='Device hostname or IP (e.g., 169.254.7.57)')
    parser.add_argument('--tools', type=str, default='',
                        help='Comma-separated ROM files (e.g., 8700449.rom,8700339.rom)')
    parser.add_argument('--rom-dir', type=str, default=default_rom_dir,
                        help='ROM file directory (default: sroms/)')
    parser.add_argument('--encrypted', action='store_true',
                        help='Use encrypted connection')
    parser.add_argument('--cipher', type=str, default='',
                        help='TLS cipher suite')
    args = parser.parse_args()

    """
    # ROM 파일 추출 및 Tracking 실행
        1) 입력된 ROM 파일 이름들을 ','로 나누고 공백 제거
        2) 각 파일이 지정한 ROM 디렉토리에 실제로 존재하는지 확인
        3) 존재하면 tools 리스트에 추가, 없으면 에러 메시지 출력 후 종료
    """
    tools = validate_rom_files(args.tools.split(','), args.rom_dir)
    if tools is None:
        return -1

    # print(f"Connecting to {args.hostname}  (ROM dir: {args.rom_dir})")
    log.info(f"Connecting to {args.hostname}  (ROM dir: {args.rom_dir})")

    run_tracking(args.hostname, tools, args.rom_dir, args.encrypted, args.cipher, print_tracking_data)
    return 0

if __name__ == "__main__":
    try:
        exit(main())
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        exit(-1)