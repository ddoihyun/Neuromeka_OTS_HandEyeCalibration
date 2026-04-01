"""
controller.py – 로봇 제어 클래스 (Neuromeka IndyDCP)

motion.py 의 유틸을 사용하여 RobotController 를 구현한다.
main.py 를 비롯한 상위 모듈은 이 클래스만 import 하면 된다.

단독 실행 시:
    python controller.py                          # 홈 이동만
    python controller.py --json dataset/poses/robot_pose_broad.json  # json 경로 지정
    python controller.py --pose 597 -143 225 79 -176 80              # 단일 movel
"""

import time
import argparse
import sys
import os
from typing import List
from src.utils.logger import get_logger
log = get_logger(__name__)

from neuromeka import IndyDCP3

# ── import 경로 (단독 실행 / 패키지 실행 모두 대응) ──────────────────
try:
    from src.robot.motion import movej_and_wait, movel_and_wait, movel_from_json
except ImportError:
    # 단독 실행 시 프로젝트 루트를 sys.path에 추가
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
    from src.robot.motion import movej_and_wait, movel_and_wait, movel_from_json


class RobotController:
    """
    Neuromeka Indy 로봇 제어 클래스.
    연결 확인, 홈 이동, movej/movel 이동을 담당한다.
    """

    # 캘리브레이션 기준 홈 포지션 (조인트각, deg)
    # EE 좌표: x=597.54, y=-143.74, z=225.75, u=79.39, v=-176.49, w=80.84
    DEFAULT_HOME_JOINT: List[float] = [
        -25.289349, 38.83128, 116.39529, -18.511345, -55.676815, -85.79714
    ]

    def __init__(self, robot_ip: str = '192.168.0.129', index: int = 0):
        """
        Parameters
        ----------
        robot_ip : str – 로봇 IP 주소
        index    : int – IndyDCP3 인덱스

        Raises
        ------
        ConnectionError : 연결 또는 상태 조회 실패 시
        """
        self.indy = IndyDCP3(robot_ip=robot_ip, index=index)

        try:
            self.indy.get_control_state()
        except Exception as e:
            raise ConnectionError(f"Robot connection failed: {e}")

        self.home_joint = list(self.DEFAULT_HOME_JOINT)

    # ── 상태 조회 ────────────────────────────────────────────────

    def get_current_pose(self) -> List[float]:
        """현재 EE 태스크 좌표 반환 [x, y, z, u, v, w]."""
        return self.indy.get_control_state()['p']

    # ── 이동 명령 ─────────────────────────────────────────────────

    def move_to_home(self, vel_ratio: int = 10, acc_ratio: int = 10, timeout: float = 60.0):
        """홈 포지션으로 movej 이동."""
        # print("\n[홈 포지션 이동 중...]")
        log.info("홈 포지션 이동 중...")
        movej_and_wait(self.indy, self.home_joint, vel_ratio=vel_ratio, acc_ratio=acc_ratio, timeout=timeout)
        # print("✓ 홈 포지션 도착")
        log.success("홈 포지션 도착")

    def movej_to_pose(self, offset: List[float],
                      vel_ratio: int = 10, acc_ratio: int = 10,
                      timeout: float = 60.0) -> List[float]:
        """
        홈 기준 오프셋 조인트 이동 (movej).

        Parameters
        ----------
        offset : List[float] – 홈 대비 각 조인트 오프셋 (deg)

        Returns
        -------
        현재 EE 태스크 좌표 [x, y, z, u, v, w]
        """
        target_joints = [h + o for h, o in zip(self.home_joint, offset)]
        movej_and_wait(self.indy, target_joints,
                       vel_ratio=vel_ratio, acc_ratio=acc_ratio, timeout=timeout)
        time.sleep(0.5)
        return self.get_current_pose()

    def movel_to_pose(self, target_pos: List[float],
                      vel_ratio: int = 10, acc_ratio: int = 10,
                      timeout: float = 60.0) -> List[float]:
        """
        절대 좌표로 movel 이동.

        Parameters
        ----------
        target_pos : List[float] – 목표 EE 태스크 좌표 [x, y, z, u, v, w]

        Returns
        -------
        도달 후 실제 EE 태스크 좌표 [x, y, z, u, v, w]
        """
        # print(f"\n[Moving to pose] {target_pos}")
        log.info(f"Moving to pose: {target_pos}")
        movel_and_wait(self.indy, target_pos,
                       vel_ratio=vel_ratio, acc_ratio=acc_ratio, timeout=timeout)
        # print("✓ Reached")
        log.success("Reached")
        return self.get_current_pose()

    def run_from_json(self, json_path: str,
                      vel_ratio: int = 10, acc_ratio: int = 10, timeout: float = 60.0):
        """
        robot_pose.json을 읽어 순서대로 movel 이동.

        Parameters
        ----------
        json_path : str – pose JSON 파일 경로
        """
        # print(f"\n[JSON 포즈 시퀀스 실행] {json_path}")
        log.info(f"JSON 포즈 시퀀스 실행: {json_path}")
        movel_from_json(self.indy, json_path,
                        vel_ratio=vel_ratio, acc_ratio=acc_ratio, timeout=timeout)


# ── 단독 실행 엔트리포인트 ───────────────────────────────────────────────

def _parse_args():
    parser = argparse.ArgumentParser(
        description="RobotController 단독 실행",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument('--ip', type=str, default='192.168.0.129',
                        help='로봇 IP 주소 (default: 192.168.0.129)')
    parser.add_argument('--json', type=str, default=None,
                        help='실행할 robot_pose JSON 파일 경로\n'
                             '예) dataset/poses/robot_pose_broad.json')
    parser.add_argument('--pose', type=float, nargs=6, default=None,
                        metavar=('X', 'Y', 'Z', 'U', 'V', 'W'),
                        help='단일 movel 목표 좌표 (6개 값)\n'
                             '예) --pose 597 -143 225 79 -176 80')
    parser.add_argument('--vel', type=int, default=10,
                        help='속도 비율 (default: 10)')
    parser.add_argument('--acc', type=int, default=10,
                        help='가속도 비율 (default: 10)')
    return parser.parse_args()


if __name__ == "__main__":
    from src.utils.logger import configure_logging
    configure_logging()
    args = _parse_args()

    # print(f"[연결 중] Robot IP: {args.ip}")
    log.info(f"연결 중 – Robot IP: {args.ip}")
    robot = RobotController(robot_ip=args.ip)

    # 1) 홈으로 이동
    robot.move_to_home(vel_ratio=args.vel, acc_ratio=args.acc)

    # 2-a) JSON 포즈 시퀀스 실행
    if args.json:
        robot.run_from_json(args.json, vel_ratio=args.vel, acc_ratio=args.acc)

    # 2-b) 단일 movel 목표 좌표
    elif args.pose:
        pose = robot.movel_to_pose(args.pose, vel_ratio=args.vel, acc_ratio=args.acc)
        # print(f"[도달 좌표] {pose}")
        log.info(f"도달 좌표: {pose}")


    # 3) 완료 후 홈 복귀
    robot.move_to_home(vel_ratio=args.vel, acc_ratio=args.acc)
    # print("\n[완료] controller.py 단독 실행 종료")
    log.success("controller.py 단독 실행 종료")
