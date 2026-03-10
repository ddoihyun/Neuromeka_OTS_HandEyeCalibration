"""
motion.py – 로봇 동작 유틸 함수 모음 (Neuromeka IndyDCP)

IndyDCP3 인스턴스(indy)를 직접 받아 동작하는 함수들만 정의한다.
클래스 수준의 제어 로직은 controller.py 의 RobotController 를 사용할 것.
"""

import time
import json

from neuromeka import IndyDCP3, TaskBaseType
from typing import List
from src.utils.logger import get_logger
log = get_logger(__name__)


def wait_until_reached(indy: IndyDCP3, timeout: float = 15.0, poll_interval: float = 0.1) -> bool:
    """
    로봇이 목표 위치에 도달할 때까지 폴링 대기.

    Parameters
    ----------
    indy          : IndyDCP3
    timeout       : float  – 최대 대기 시간 (초)
    poll_interval : float  – 폴링 간격 (초)

    Returns
    -------
    True : 도달 성공

    Raises
    ------
    TimeoutError : timeout 초과 시
    """
    start = time.time()
    while True:
        if indy.get_motion_data().get('is_target_reached', False):
            return True
        if time.time() - start > timeout:
            raise TimeoutError("Motion timeout exceeded")
        time.sleep(poll_interval)


def movej_and_wait(indy: IndyDCP3, target_joint: List[float],
                   vel_ratio: int = 10, acc_ratio: int = 10, timeout: float = 60.0):
    """movej 명령 실행 후 도달 대기."""
    indy.movej(jtarget=target_joint, vel_ratio=vel_ratio, acc_ratio=acc_ratio)
    wait_until_reached(indy, timeout=timeout)


def movel_and_wait(indy: IndyDCP3, target_pos: List[float],
                   vel_ratio: int = 10, acc_ratio: int = 10, timeout: float = 60.0):
    """movel 명령 실행 후 도달 대기."""
    indy.movel(ttarget=target_pos, vel_ratio=vel_ratio, acc_ratio=acc_ratio,
               base_type=TaskBaseType.ABSOLUTE)
    wait_until_reached(indy, timeout=timeout)


def movel_from_json(indy: IndyDCP3, json_path: str,
                    vel_ratio: int = 10, acc_ratio: int = 10, timeout: float = 60):
    """
    robot_pose.json 을 읽어 sample_number 순서대로 movel 이동.

    JSON 포맷 예시:
        [{"sample_number": 1, "pose": [x, y, z, u, v, w]}, ...]
    """
    with open(json_path, 'r') as f:
        pose_list = sorted(json.load(f), key=lambda x: x['sample_number'])

    for item in pose_list:
        sample_no  = item['sample_number']
        target_pos = item['pose']
        # print(f"\n[{sample_no}] Moving to pose: {target_pos}")
        log.info(f"[{sample_no}] Moving to pose: {target_pos}")
        movel_and_wait(indy, target_pos, vel_ratio=vel_ratio, acc_ratio=acc_ratio, timeout=timeout)
        # print(f"✓ Reached sample {sample_no}")
        log.success(f"Reached sample {sample_no}")