"""NDI 마커 관측 → 로봇 EE 목표 좌표 변환.

사용법:
    # 직접 실행 (대화형)
    python3 navigate.py

    # 인자로 NDI 측정값 전달 (q0 qx qy qz tx ty tz)
    python3 navigate.py 0.123 -0.456 0.789 0.321 100.0 -200.0 -1500.0

    # 다른 캘리브레이션 결과 파일 사용
    python3 navigate.py --calib ./other_result.json 0.123 -0.456 0.789 0.321 100.0 -200.0 -1500.0

    # 모듈로 import
    from navigate import Navigator
    nav = Navigator()
    result = nav.compute(q0, qx, qy, qz, tx, ty, tz)
"""

import numpy as np
import json
import sys
from scipy.spatial.transform import Rotation as R


class Navigator:
    """캘리브레이션 결과를 이용한 NDI→로봇 좌표 변환기.

    네비게이션 공식:
        T_base_ee_target = inv(T_ndi_base) @ T_ndi_marker_new @ inv(T_ee_marker)
    """

    def __init__(self, calib_path='../../dataset/results/calibration_result.json'):
        with open(calib_path, 'r') as f:
            calib = json.load(f)

        self.T_ndi_base = np.array(calib['T_ndi_base']['matrix'])
        self.T_ee_marker = np.array(calib['T_ee_marker']['matrix'])
        self.unit = calib.get('translation_unit', 'mm')
        self.method = calib.get('method', 'unknown')
        self.ndi_position_bias = np.array(calib.get('ndi_position_bias', [0.0, 0.0, 0.0]))
        self.ndi_axis_scale = np.array(calib.get('ndi_axis_scale', [1.0, 1.0, 1.0]))

        # 미리 역행렬 계산 (반복 호출 시 효율)
        self.T_base_ndi = np.linalg.inv(self.T_ndi_base)
        self.T_marker_ee = np.linalg.inv(self.T_ee_marker)

    def compute(self, q0, qx, qy, qz, tx, ty, tz):
        """NDI 마커 관측값 → 로봇 EE 목표 좌표.

        Args:
            q0, qx, qy, qz: NDI 쿼터니언 (q0=scalar)
            tx, ty, tz: NDI 위치 (mm)

        Returns:
            dict with keys:
                'T': 4x4 homogeneous matrix (T_base_ee_target)
                'x', 'y', 'z': EE 목표 위치 (mm)
                'u', 'v', 'w': EE 목표 자세 (degrees, Neuromeka INDY 컨벤션)
        """
        # T_ndi_marker 구성
        R_ndi = R.from_quat([qx, qy, qz, q0]).as_matrix()
        T_ndi_marker = np.eye(4)
        T_ndi_marker[:3, :3] = R_ndi
        T_ndi_marker[:3, 3] = [tx, ty, tz]
        T_ndi_marker[:3, 3] = self.ndi_axis_scale * T_ndi_marker[:3, 3] + self.ndi_position_bias

        # 네비게이션 공식
        T_target = self.T_base_ndi @ T_ndi_marker @ self.T_marker_ee

        # Euler 추출 (INDY: extrinsic XYZ = intrinsic ZYX)
        zyx = R.from_matrix(T_target[:3, :3]).as_euler('ZYX', degrees=True)
        u, v, w = zyx[2], zyx[1], zyx[0]  # u=Rx, v=Ry, w=Rz

        return {
            'T': T_target,
            'x': T_target[0, 3],
            'y': T_target[1, 3],
            'z': T_target[2, 3],
            'u': u, 'v': v, 'w': w
        }

    def print_result(self, result):
        """결과를 보기 좋게 출력."""
        print(f"\n  로봇 EE 목표 좌표:")
        print(f"    위치: x={result['x']:.4f}, y={result['y']:.4f}, z={result['z']:.4f} {self.unit}")
        print(f"    자세: u={result['u']:.4f}°, v={result['v']:.4f}°, w={result['w']:.4f}°")
        print(f"    (Neuromeka INDY 포맷: [{result['x']:.4f}, {result['y']:.4f}, "
              f"{result['z']:.4f}, {result['u']:.4f}, {result['v']:.4f}, {result['w']:.4f}])")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='NDI 마커 → 로봇 EE 좌표 변환')
    parser.add_argument('--calib', default='../../dataset/results/calibration_result.json',
                        help='캘리브레이션 결과 JSON 경로')
    parser.add_argument('values', nargs='*', type=float,
                        help='q0 qx qy qz tx ty tz (7개 값)')
    args = parser.parse_args()

    nav = Navigator(calib_path=args.calib)
    print(f"캘리브레이션 로드 완료 (method: {nav.method}, unit: {nav.unit})")

    if len(args.values) == 7:
        # 명령줄 인자로 전달된 경우
        q0, qx, qy, qz, tx, ty, tz = args.values
        print(f"\n  NDI 입력: q0={q0}, qx={qx}, qy={qy}, qz={qz}, "
              f"tx={tx}, ty={ty}, tz={tz}")
        result = nav.compute(q0, qx, qy, qz, tx, ty, tz)
        nav.print_result(result)
    elif len(args.values) == 0:
        # 대화형 모드
        print("\nNDI 마커 측정값을 입력하세요 (종료: q)")
        print("형식: q0 qx qy qz tx ty tz\n")
        while True:
            try:
                line = input("> ").strip()
                if line.lower() in ('q', 'quit', 'exit', ''):
                    break
                vals = [float(v) for v in line.split()]
                if len(vals) != 7:
                    print("  7개 값을 입력하세요: q0 qx qy qz tx ty tz")
                    continue
                q0, qx, qy, qz, tx, ty, tz = vals
                result = nav.compute(q0, qx, qy, qz, tx, ty, tz)
                nav.print_result(result)
                print()
            except ValueError:
                print("  숫자를 입력하세요.")
            except (EOFError, KeyboardInterrupt):
                break
        print("\n종료.")
    else:
        print(f"오류: 7개 값이 필요합니다 (현재 {len(args.values)}개)")
        print("형식: q0 qx qy qz tx ty tz")
        sys.exit(1)


if __name__ == '__main__':
    main()
