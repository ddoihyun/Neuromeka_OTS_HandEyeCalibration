import numpy as np
import pandas as pd
import json
import cv2
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as R
from scipy.linalg import svd, lstsq
from scipy.optimize import least_squares
from pathlib import Path
from src.utils.logger import get_logger
log = get_logger(__name__)

try:
    from thc_calibration.rotation_utils import average_SE3
except Exception:
    average_SE3 = None

# ── 단위 설정 ──────────────────────────────────────
ROBOT_UNIT = "mm"   # 로봇 x,y,z 단위
NDI_UNIT   = "mm"   # NDI tx,ty,tz 단위
# ───────────────────────────────────────────────────


class HandEyeCalibration:
    """Eye-to-hand 캘리브레이션 시스템.

    시스템 구성:
        - NDI: 외부 고정 광학 센서 (카메라)
        - 마커: 로봇 EE에 부착
        - 목표: T_ndi_base, T_ee_marker를 구해서
                 임의의 NDI 마커 위치 → 로봇 EE 목표 좌표로 변환

    운동학적 관계:
        T_ndi_marker = T_ndi_base @ T_base_ee @ T_ee_marker

    네비게이션 공식:
        T_base_ee_target = inv(T_ndi_base) @ T_ndi_marker_new @ inv(T_ee_marker)
    """

    def __init__(self, csv_path='./calibration_data_broad.csv',
                 result_json_path=None, result_png_path=None):
        self.csv_path = csv_path
        self.result_json_path, self.result_png_path = self._resolve_output_paths(
            csv_path, result_json_path, result_png_path)
        self.raw_data = None
        self.averaged_data = None
        self.all_data = None
        self.T_ndi_base = None
        self.T_ee_marker = None
        self.best_method = None
        self.all_results = None
        self.ndi_position_bias = np.zeros(3)
        self.ndi_axis_scale = np.ones(3)

    def _resolve_output_paths(self, csv_path, result_json_path, result_png_path):
        csv_path = Path(csv_path)
        results_dir = Path('./dataset/results')
        results_dir.mkdir(parents=True, exist_ok=True)

        if result_json_path is None:
            suffix = csv_path.stem.replace('calibration_data', '')
            result_json_path = results_dir / f'calibration_result{suffix}.json'
        else:
            result_json_path = Path(result_json_path)

        if result_png_path is None:
            suffix = csv_path.stem.replace('calibration_data', '')
            result_png_path = results_dir / f'calibration_result{suffix}.png'
        else:
            result_png_path = Path(result_png_path)

        return Path(result_json_path), Path(result_png_path)

    # ──────────────────────────────────────────────
    # 1. 데이터 로딩 및 전처리
    # ──────────────────────────────────────────────
    def load_and_preprocess_data(self):
        print("=" * 60)
        print("1) 데이터 로딩 및 전처리")
        print("=" * 60)
        print(f"  로봇 단위: {ROBOT_UNIT}, NDI 단위: {NDI_UNIT}")

        self.raw_data = pd.read_csv(self.csv_path)
        print(f"전체 데이터 수: {len(self.raw_data)} rows")

        grouped = self.raw_data.groupby('pose_id')
        averaged_list = []
        total_removed = 0
        for pose_id, group in grouped:
            inlier_group = self._filter_pose_group_outliers(group)
            total_removed += len(group) - len(inlier_group)
            avg_T = self._average_pose_transform(inlier_group)
            avg_quat = R.from_matrix(avg_T[:3, :3]).as_quat()
            avg_data = {
                'pose_id': pose_id,
                'q0': avg_quat[3], 'qx': avg_quat[0],
                'qy': avg_quat[1], 'qz': avg_quat[2],
                'tx': avg_T[0, 3],
                'ty': avg_T[1, 3],
                'tz': avg_T[2, 3],
                'x':  inlier_group['x'].mean(),
                'y':  inlier_group['y'].mean(),
                'z':  inlier_group['z'].mean(),
                'u':  inlier_group['u'].mean(),
                'v':  inlier_group['v'].mean(),
                'w':  inlier_group['w'].mean(),
                'samples_raw': len(group),
                'samples_inlier': len(inlier_group),
            }
            averaged_list.append(avg_data)

        self.averaged_data = pd.DataFrame(averaged_list).reset_index(drop=True)
        print(f"??? ? pose ?: {len(self.averaged_data)}")
        if total_removed:
            print(f"  Pose ?? outlier ??: {total_removed} samples")

        if 'error' in self.raw_data.columns:
            err = self.raw_data['error']
            print(f"  NDI 측정 오차: 평균={err.mean():.4f}, "
                  f"max={err.max():.4f}, std={err.std():.4f}")

        print(f"\n  [데이터 범위 확인]")
        print(f"  로봇 xyz 범위: "
              f"x=[{self.averaged_data['x'].min():.1f}, {self.averaged_data['x'].max():.1f}], "
              f"y=[{self.averaged_data['y'].min():.1f}, {self.averaged_data['y'].max():.1f}], "
              f"z=[{self.averaged_data['z'].min():.1f}, {self.averaged_data['z'].max():.1f}]")
        print(f"  NDI txyz 범위: "
              f"tx=[{self.averaged_data['tx'].min():.1f}, {self.averaged_data['tx'].max():.1f}], "
              f"ty=[{self.averaged_data['ty'].min():.1f}, {self.averaged_data['ty'].max():.1f}], "
              f"tz=[{self.averaged_data['tz'].min():.1f}, {self.averaged_data['tz'].max():.1f}]")
        print()

        self.all_data = self.averaged_data.copy()
        print(f"학습 데이터: {len(self.all_data)} poses\n")

    def _average_pose_transform(self, group):
        transforms = np.array([
            self.get_ndi_transform(row) for _, row in group.iterrows()
        ])
        if average_SE3 is not None and len(transforms) > 0:
            return np.asarray(average_SE3(transforms), dtype=float)

        quats = group[['qx', 'qy', 'qz', 'q0']].values
        avg_quat = self._average_quaternions(quats)
        avg_R = R.from_quat(avg_quat).as_matrix()
        avg_t = group[['tx', 'ty', 'tz']].mean().to_numpy(dtype=float)
        return self.create_homogeneous_matrix(avg_R, avg_t)

    def _filter_pose_group_outliers(self, group):
        if len(group) <= 4:
            return group

        transforms = [self.get_ndi_transform(row) for _, row in group.iterrows()]
        mean_T = self._average_pose_transform(group)
        trans_err = []
        rot_err = []
        for T in transforms:
            trans_err.append(np.linalg.norm(T[:3, 3] - mean_T[:3, 3]))
            R_err = mean_T[:3, :3].T @ T[:3, :3]
            angle = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))
            rot_err.append(np.degrees(angle))

        trans_err = np.asarray(trans_err, dtype=float)
        rot_err = np.asarray(rot_err, dtype=float)

        def robust_limit(values, floor):
            med = np.median(values)
            mad = np.median(np.abs(values - med))
            sigma = 1.4826 * mad
            return max(floor, med + 3.0 * sigma)

        trans_limit = robust_limit(trans_err, floor=0.25)
        rot_limit = robust_limit(rot_err, floor=0.2)
        mask = (trans_err <= trans_limit) & (rot_err <= rot_limit)

        if 'error' in group.columns:
            ndi_err = group['error'].to_numpy(dtype=float)
            ndi_limit = robust_limit(ndi_err, floor=float(np.median(ndi_err) + 0.02))
            mask &= ndi_err <= ndi_limit

        min_keep = max(4, int(np.ceil(len(group) * 0.6)))
        if mask.sum() < min_keep:
            score = trans_err / max(np.median(trans_err), 1e-6)
            score += rot_err / max(np.median(rot_err), 1e-6)
            if 'error' in group.columns:
                ndi_err = group['error'].to_numpy(dtype=float)
                score += ndi_err / max(np.median(ndi_err), 1e-6)
            keep_idx = np.argsort(score)[:min_keep]
            mask = np.zeros(len(group), dtype=bool)
            mask[keep_idx] = True

        return group.iloc[np.where(mask)[0]].copy()

    def _average_quaternions(self, quats_xyzw: np.ndarray) -> np.ndarray:
        """Markley eigendecomposition 쿼터니언 평균"""
        M = np.zeros((4, 4))
        for q in quats_xyzw:
            q = q / np.linalg.norm(q)
            q_wxyz = np.array([q[3], q[0], q[1], q[2]])
            M += np.outer(q_wxyz, q_wxyz)
        M /= len(quats_xyzw)
        eigvals, eigvecs = np.linalg.eigh(M)
        avg_wxyz = eigvecs[:, np.argmax(eigvals)]
        return np.array([avg_wxyz[1], avg_wxyz[2], avg_wxyz[3], avg_wxyz[0]])

    # ──────────────────────────────────────────────
    # 2. 변환 행렬 생성
    # ──────────────────────────────────────────────
    def quaternion_to_rotation_matrix(self, q0, qx, qy, qz):
        return R.from_quat([qx, qy, qz, q0]).as_matrix()

    def euler_to_rotation_matrix(self, u_deg, v_deg, w_deg):
        """Neuromeka INDY: u=Rx, v=Ry, w=Rz, extrinsic XYZ = intrinsic ZYX"""
        return R.from_euler('ZYX', [w_deg, v_deg, u_deg], degrees=True).as_matrix()

    def create_homogeneous_matrix(self, R_mat, t):
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = np.asarray(t).flatten()
        return T

    def get_ndi_transform(self, row):
        """T_ndi_marker: 마커를 NDI 카메라 좌표계로 표현"""
        R_mat = self.quaternion_to_rotation_matrix(
            row['q0'], row['qx'], row['qy'], row['qz'])
        t = np.array([row['tx'], row['ty'], row['tz']])
        return self.create_homogeneous_matrix(R_mat, t)

    def get_ndi_transform_with_bias(self, row, ndi_position_bias=None,
                                    ndi_axis_scale=None):
        """NDI 위치 보정(축별 scale + bias)을 적용한 T_ndi_marker.

        t_corr = diag(scale) @ t_raw + bias
        """
        T_ndi = self.get_ndi_transform(row)
        t = T_ndi[:3, 3].copy()
        if ndi_axis_scale is not None:
            t = np.asarray(ndi_axis_scale).reshape(3) * t
        if ndi_position_bias is not None:
            t = t + np.asarray(ndi_position_bias).reshape(3)
        T_ndi[:3, 3] = t
        return T_ndi

    def get_robot_transform(self, row):
        """T_base_ee: EE를 로봇 Base 좌표계로 표현"""
        R_mat = self.euler_to_rotation_matrix(row['u'], row['v'], row['w'])
        t = np.array([row['x'], row['y'], row['z']])
        return self.create_homogeneous_matrix(R_mat, t)

    # ──────────────────────────────────────────────
    # 3. A-B 쌍 계산
    # ──────────────────────────────────────────────
    def compute_AB_pairs(self, data, use_all_pairs=False):
        """AX=XB 상대변환 쌍.

        use_all_pairs=True: 모든 (i,j) 조합 (OpenCV 내부와 동일 수준)
        use_all_pairs=False: 연속 (i,i+1) 쌍 (SVD solver 안정성용)
        """
        A_list, B_list = [], []
        n = len(data)
        if use_all_pairs:
            for i in range(n):
                T_r_i = self.get_robot_transform(data.iloc[i])
                T_n_i = self.get_ndi_transform(data.iloc[i])
                for j in range(i + 1, n):
                    T_r_j = self.get_robot_transform(data.iloc[j])
                    T_n_j = self.get_ndi_transform(data.iloc[j])
                    A_list.append(np.linalg.inv(T_r_i) @ T_r_j)
                    B_list.append(np.linalg.inv(T_n_i) @ T_n_j)
        else:
            for i in range(n - 1):
                T_r_i = self.get_robot_transform(data.iloc[i])
                T_r_j = self.get_robot_transform(data.iloc[i + 1])
                T_n_i = self.get_ndi_transform(data.iloc[i])
                T_n_j = self.get_ndi_transform(data.iloc[i + 1])
                A_list.append(np.linalg.inv(T_r_i) @ T_r_j)
                B_list.append(np.linalg.inv(T_n_i) @ T_n_j)
        return A_list, B_list

    # ──────────────────────────────────────────────
    # 4. SVD Solver (AX=XB → T_ee_marker)
    # ──────────────────────────────────────────────
    def solve_hand_eye_svd(self, A_list, B_list):
        """SVD 기반 AX=XB 풀이. X = T_ee_marker."""
        n = len(A_list)
        M = np.zeros((9 * n, 9))
        for i in range(n):
            Ra = A_list[i][:3, :3]
            Rb = B_list[i][:3, :3]
            M[9*i:9*(i+1), :] = np.kron(Ra, np.eye(3)) - np.kron(np.eye(3), Rb.T)

        _, S, Vt = svd(M)
        R_X = Vt[-1].reshape(3, 3)
        U_r, _, Vt_r = svd(R_X)
        R_X = U_r @ Vt_r
        if np.linalg.det(R_X) < 0:
            Vt_r[-1] *= -1
            R_X = U_r @ Vt_r

        M_t, b_t = [], []
        for i in range(n):
            Ra = A_list[i][:3, :3]
            ta = A_list[i][:3, 3]
            tb = B_list[i][:3, 3]
            M_t.append(Ra - np.eye(3))
            b_t.append(R_X @ tb - ta)
        t_X, _, _, _ = lstsq(np.vstack(M_t), np.concatenate(b_t))
        return self.create_homogeneous_matrix(R_X, t_X)

    # ──────────────────────────────────────────────
    # 5. calibrateRobotWorldHandEye (eye-to-hand 전용)
    # ──────────────────────────────────────────────
    def solve_robot_world_hand_eye(self, data, method):
        """T_ndi_base와 T_ee_marker 동시 계산.

        OpenCV 매핑:
            R_world2cam = inv(T_ndi_marker)
            R_base2gripper = inv(T_base_ee)
        출력:
            R_base2world → T_ndi_base
            R_gripper2cam → T_marker_ee → inv() → T_ee_marker
        """
        R_w2c, t_w2c, R_b2g, t_b2g = [], [], [], []
        for _, row in data.iterrows():
            T_ndi = self.get_ndi_transform(row)
            T_robot = self.get_robot_transform(row)
            T_ndi_inv = np.linalg.inv(T_ndi)
            T_robot_inv = np.linalg.inv(T_robot)
            R_w2c.append(T_ndi_inv[:3, :3])
            t_w2c.append(T_ndi_inv[:3, 3].reshape(3, 1))
            R_b2g.append(T_robot_inv[:3, :3])
            t_b2g.append(T_robot_inv[:3, 3].reshape(3, 1))

        R_b2w, t_b2w, R_g2c, t_g2c = cv2.calibrateRobotWorldHandEye(
            R_world2cam=R_w2c, t_world2cam=t_w2c,
            R_base2gripper=R_b2g, t_base2gripper=t_b2g,
            method=method)

        T_ndi_base = self.create_homogeneous_matrix(R_b2w, t_b2w.flatten())
        T_marker_ee = self.create_homogeneous_matrix(R_g2c, t_g2c.flatten())
        T_ee_marker = np.linalg.inv(T_marker_ee)
        return T_ndi_base, T_ee_marker

    # ──────────────────────────────────────────────
    # 6. calibrateHandEye eye-to-hand 래퍼
    # ──────────────────────────────────────────────
    def solve_handeye_for_T_ee_marker(self, data, method):
        """T_base_ee as g2b, inv(T_ndi_marker) as t2c → T_ee_marker."""
        R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
        for _, row in data.iterrows():
            T_robot = self.get_robot_transform(row)
            T_ndi_inv = np.linalg.inv(self.get_ndi_transform(row))
            R_g2b.append(T_robot[:3, :3])
            t_g2b.append(T_robot[:3, 3].reshape(3, 1))
            R_t2c.append(T_ndi_inv[:3, :3])
            t_t2c.append(T_ndi_inv[:3, 3].reshape(3, 1))
        R_out, t_out = cv2.calibrateHandEye(
            R_gripper2base=R_g2b, t_gripper2base=t_g2b,
            R_target2cam=R_t2c, t_target2cam=t_t2c,
            method=method)
        return self.create_homogeneous_matrix(R_out, t_out.flatten())

    # ──────────────────────────────────────────────
    # 7. Two-step: T_ee_marker → T_ndi_base
    # ──────────────────────────────────────────────
    def compute_T_ndi_base_from_T_ee_marker(self, data, T_ee_marker):
        """T_ndi_base = avg( T_ndi_marker_i @ inv(T_ee_marker) @ inv(T_base_ee_i) )"""
        transforms = []
        T_ee_marker_inv = np.linalg.inv(T_ee_marker)
        for _, row in data.iterrows():
            T_ndi = self.get_ndi_transform(row)
            T_robot = self.get_robot_transform(row)
            T_est = T_ndi @ T_ee_marker_inv @ np.linalg.inv(T_robot)
            transforms.append(T_est)

        transforms = np.asarray(transforms, dtype=float)
        if average_SE3 is not None and len(transforms) > 0:
            return np.asarray(average_SE3(transforms), dtype=float)

        quats = [R.from_matrix(T[:3, :3]).as_quat() for T in transforms]
        avg_quat = self._average_quaternions(np.array(quats))
        avg_R = R.from_quat(avg_quat).as_matrix()
        avg_t = np.mean(transforms[:, :3, 3], axis=0)
        return self.create_homogeneous_matrix(avg_R, avg_t)

    # ──────────────────────────────────────────────
    # 8. Point Registration (Arun's SVD method)
    # ──────────────────────────────────────────────
    def solve_point_registration(self, data, T_ee_marker=None):
        """SVD rigid body registration: p_ndi = R @ p_base + t → T_ndi_base"""
        p_ndi_list, p_base_list = [], []
        for _, row in data.iterrows():
            T_ndi = self.get_ndi_transform(row)
            T_robot = self.get_robot_transform(row)
            p_ndi_list.append(T_ndi[:3, 3])
            if T_ee_marker is not None:
                p_base_list.append((T_robot @ T_ee_marker)[:3, 3])
            else:
                p_base_list.append(T_robot[:3, 3])

        p_ndi = np.array(p_ndi_list)
        p_base = np.array(p_base_list)

        centroid_ndi = p_ndi.mean(axis=0)
        centroid_base = p_base.mean(axis=0)
        H = (p_base - centroid_base).T @ (p_ndi - centroid_ndi)
        U, S, Vt = np.linalg.svd(H)
        d = np.linalg.det(Vt.T @ U.T)
        R_reg = Vt.T @ np.diag([1, 1, d]) @ U.T
        t_reg = centroid_ndi - R_reg @ centroid_base

        T_ndi_base = self.create_homogeneous_matrix(R_reg, t_reg)
        p_pred = (R_reg @ p_base.T).T + t_reg
        residuals = np.linalg.norm(p_pred - p_ndi, axis=1)
        return T_ndi_base, residuals

    # ──────────────────────────────────────────────
    # 9. 비선형 최적화 (joint refinement)
    # ──────────────────────────────────────────────
    def _rotvec_t_to_matrix(self, rotvec, t):
        """rotation vector + translation → 4x4 homogeneous"""
        R_mat = R.from_rotvec(rotvec).as_matrix()
        return self.create_homogeneous_matrix(R_mat, t)

    def refine_nonlinear(self, data, T_ndi_base_init, T_ee_marker_init,
                         pos_weight=1.0, rot_weight=20.0,
                         exclude_indices=None):
        """비선형 최적화로 T_ndi_base, T_ee_marker 동시 정밀화.

        Args:
            pos_weight: 위치 오차 가중치
            rot_weight: 회전 오차 가중치 (mm 등가 스케일)
            exclude_indices: 제외할 포즈 인덱스 set
        """
        T_ndi_list, T_robot_list = [], []
        indices = []
        for idx, (_, row) in enumerate(data.iterrows()):
            if exclude_indices and idx in exclude_indices:
                continue
            T_ndi_list.append(self.get_ndi_transform(row))
            T_robot_list.append(self.get_robot_transform(row))
            indices.append(idx)

        rv_ndi = R.from_matrix(T_ndi_base_init[:3, :3]).as_rotvec()
        t_ndi = T_ndi_base_init[:3, 3]
        rv_ee = R.from_matrix(T_ee_marker_init[:3, :3]).as_rotvec()
        t_ee = T_ee_marker_init[:3, 3]
        x0 = np.concatenate([rv_ndi, t_ndi, rv_ee, t_ee])

        def residual_fn(x):
            T_nb = self._rotvec_t_to_matrix(x[0:3], x[3:6])
            T_em = self._rotvec_t_to_matrix(x[6:9], x[9:12])
            T_bn = np.linalg.inv(T_nb)
            T_me = np.linalg.inv(T_em)

            residuals = []
            for T_ndi, T_robot in zip(T_ndi_list, T_robot_list):
                T_pred = T_bn @ T_ndi @ T_me
                pos_err = (T_pred[:3, 3] - T_robot[:3, 3]) * pos_weight
                residuals.extend(pos_err)
                R_err = T_pred[:3, :3] @ T_robot[:3, :3].T
                rv_err = R.from_matrix(R_err).as_rotvec() * rot_weight
                residuals.extend(rv_err)
            return np.array(residuals)

        result = least_squares(residual_fn, x0, method='lm', max_nfev=20000)
        T_ndi_base_opt = self._rotvec_t_to_matrix(result.x[0:3], result.x[3:6])
        T_ee_marker_opt = self._rotvec_t_to_matrix(result.x[6:9], result.x[9:12])
        return T_ndi_base_opt, T_ee_marker_opt, result.cost

    def refine_nonlinear_with_ndi_bias(self, data, T_ndi_base_init, T_ee_marker_init,
                                       ndi_position_bias_init=None,
                                       pos_weight=1.0, rot_weight=20.0,
                                       bias_reg_weight=0.02,
                                       exclude_indices=None):
        """NDI 위치 편향(bias)까지 포함한 joint LM 최적화."""
        T_ndi_list, T_robot_list = [], []
        for idx, (_, row) in enumerate(data.iterrows()):
            if exclude_indices and idx in exclude_indices:
                continue
            T_ndi_list.append(self.get_ndi_transform(row))
            T_robot_list.append(self.get_robot_transform(row))

        rv_ndi = R.from_matrix(T_ndi_base_init[:3, :3]).as_rotvec()
        t_ndi = T_ndi_base_init[:3, 3]
        rv_ee = R.from_matrix(T_ee_marker_init[:3, :3]).as_rotvec()
        t_ee = T_ee_marker_init[:3, 3]
        b0 = np.zeros(3) if ndi_position_bias_init is None else np.asarray(ndi_position_bias_init).reshape(3)
        x0 = np.concatenate([rv_ndi, t_ndi, rv_ee, t_ee, b0])

        def residual_fn(x):
            T_nb = self._rotvec_t_to_matrix(x[0:3], x[3:6])
            T_em = self._rotvec_t_to_matrix(x[6:9], x[9:12])
            b_ndi = x[12:15]
            T_bn = np.linalg.inv(T_nb)
            T_me = np.linalg.inv(T_em)

            residuals = []
            for T_ndi_raw, T_robot in zip(T_ndi_list, T_robot_list):
                T_ndi = T_ndi_raw.copy()
                T_ndi[:3, 3] += b_ndi
                T_pred = T_bn @ T_ndi @ T_me
                pos_err = (T_pred[:3, 3] - T_robot[:3, 3]) * pos_weight
                residuals.extend(pos_err)
                R_err = T_pred[:3, :3] @ T_robot[:3, :3].T
                rv_err = R.from_matrix(R_err).as_rotvec() * rot_weight
                residuals.extend(rv_err)

            residuals.extend(bias_reg_weight * b_ndi)
            return np.array(residuals)

        result = least_squares(residual_fn, x0, method='lm', max_nfev=30000)
        T_ndi_base_opt = self._rotvec_t_to_matrix(result.x[0:3], result.x[3:6])
        T_ee_marker_opt = self._rotvec_t_to_matrix(result.x[6:9], result.x[9:12])
        ndi_bias_opt = result.x[12:15]
        return T_ndi_base_opt, T_ee_marker_opt, ndi_bias_opt, result.cost


    def refine_nonlinear_with_ndi_axis_scale(self, data, T_ndi_base_init, T_ee_marker_init,
                                             ndi_position_bias_init=None,
                                             ndi_axis_scale_init=None,
                                             pos_weight=1.0, rot_weight=20.0,
                                             bias_reg_weight=0.02,
                                             scale_reg_weight=10.0,
                                             exclude_indices=None):
        """NDI 축별 scale + bias까지 포함한 joint LM 최적화.

        scale는 1.0 근처로 정규화해 과적합을 억제한다.
        """
        T_ndi_list, T_robot_list = [], []
        for idx, (_, row) in enumerate(data.iterrows()):
            if exclude_indices and idx in exclude_indices:
                continue
            T_ndi_list.append(self.get_ndi_transform(row))
            T_robot_list.append(self.get_robot_transform(row))

        rv_ndi = R.from_matrix(T_ndi_base_init[:3, :3]).as_rotvec()
        t_ndi = T_ndi_base_init[:3, 3]
        rv_ee = R.from_matrix(T_ee_marker_init[:3, :3]).as_rotvec()
        t_ee = T_ee_marker_init[:3, 3]
        b0 = np.zeros(3) if ndi_position_bias_init is None else np.asarray(ndi_position_bias_init).reshape(3)
        s0 = np.ones(3) if ndi_axis_scale_init is None else np.asarray(ndi_axis_scale_init).reshape(3)
        x0 = np.concatenate([rv_ndi, t_ndi, rv_ee, t_ee, b0, s0])

        def residual_fn(x):
            T_nb = self._rotvec_t_to_matrix(x[0:3], x[3:6])
            T_em = self._rotvec_t_to_matrix(x[6:9], x[9:12])
            b_ndi = x[12:15]
            s_ndi = x[15:18]
            T_bn = np.linalg.inv(T_nb)
            T_me = np.linalg.inv(T_em)

            residuals = []
            for T_ndi_raw, T_robot in zip(T_ndi_list, T_robot_list):
                T_ndi = T_ndi_raw.copy()
                T_ndi[:3, 3] = s_ndi * T_ndi[:3, 3] + b_ndi
                T_pred = T_bn @ T_ndi @ T_me
                pos_err = (T_pred[:3, 3] - T_robot[:3, 3]) * pos_weight
                residuals.extend(pos_err)
                R_err = T_pred[:3, :3] @ T_robot[:3, :3].T
                rv_err = R.from_matrix(R_err).as_rotvec() * rot_weight
                residuals.extend(rv_err)

            residuals.extend(bias_reg_weight * b_ndi)
            residuals.extend(scale_reg_weight * (s_ndi - np.ones(3)))
            return np.array(residuals)

        result = least_squares(residual_fn, x0, method='lm', max_nfev=40000)
        T_ndi_base_opt = self._rotvec_t_to_matrix(result.x[0:3], result.x[3:6])
        T_ee_marker_opt = self._rotvec_t_to_matrix(result.x[6:9], result.x[9:12])
        ndi_bias_opt = result.x[12:15]
        ndi_scale_opt = result.x[15:18]
        return T_ndi_base_opt, T_ee_marker_opt, ndi_bias_opt, ndi_scale_opt, result.cost

    def refine_with_outlier_rejection(self, data, T_ndi_base_init, T_ee_marker_init,
                                      n_iterations=3, outlier_ratio=0.15):
        """반복적 아웃라이어 제거 + LM 최적화.

        각 반복에서 상위 outlier_ratio 만큼의 포즈를 제거하고 재최적화.
        """
        T_nb = T_ndi_base_init.copy()
        T_em = T_ee_marker_init.copy()
        exclude = set()
        n = len(data)

        for it in range(n_iterations):
            # LM 최적화
            T_nb, T_em, cost = self.refine_nonlinear(
                data, T_nb, T_em, pos_weight=1.0, rot_weight=20.0,
                exclude_indices=exclude)

            # 전체 포즈에 대해 잔차 계산
            pe, _ = self.evaluate_absolute_position(data, T_nb, T_em)

            # 상위 outlier_ratio 포즈를 제외 (최소 포즈 수 보장)
            max_exclude = max(0, n - 16)  # 최소 16포즈 유지
            n_exclude = min(int(n * outlier_ratio), max_exclude)
            if n_exclude > 0:
                worst_indices = np.argsort(pe)[-n_exclude:]
                exclude = set(worst_indices)

        # 최종 최적화 (아웃라이어 제외)
        T_nb, T_em, cost = self.refine_nonlinear(
            data, T_nb, T_em, pos_weight=1.0, rot_weight=20.0,
            exclude_indices=exclude)
        return T_nb, T_em, exclude

    # ──────────────────────────────────────────────
    # 10. 절대 위치 오차 평가 (핵심 메트릭)
    # ──────────────────────────────────────────────
    def evaluate_absolute_position(self, data, T_ndi_base, T_ee_marker,
                                   ndi_position_bias=None, ndi_axis_scale=None):
        """절대 EE 위치 예측 오차.

        예측: T_base_ee_pred = inv(T_ndi_base) @ T_ndi_marker @ inv(T_ee_marker)
        """
        pos_errors, rot_errors = [], []
        T_base_ndi = np.linalg.inv(T_ndi_base)
        T_marker_ee = np.linalg.inv(T_ee_marker)

        for _, row in data.iterrows():
            T_ndi = self.get_ndi_transform_with_bias(row, ndi_position_bias, ndi_axis_scale)
            T_robot_true = self.get_robot_transform(row)
            T_robot_pred = T_base_ndi @ T_ndi @ T_marker_ee

            pos_errors.append(
                np.linalg.norm(T_robot_pred[:3, 3] - T_robot_true[:3, 3]))
            R_err = T_robot_pred[:3, :3].T @ T_robot_true[:3, :3]
            angle = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))
            rot_errors.append(np.degrees(angle))

        return np.array(pos_errors), np.array(rot_errors)

    # ──────────────────────────────────────────────
    # 11. AX=XB 잔차 (보조 메트릭)
    # ──────────────────────────────────────────────
    def evaluate_handeye_residual(self, A_list, B_list, X):
        trans_errors, rot_errors = [], []
        for A, B in zip(A_list, B_list):
            AX = A @ X
            XB = X @ B
            t_err = np.linalg.norm(AX[:3, 3] - XB[:3, 3])
            R_err = AX[:3, :3].T @ XB[:3, :3]
            angle = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))
            trans_errors.append(t_err)
            rot_errors.append(np.degrees(angle))
        return np.mean(trans_errors), np.mean(rot_errors)

    # ──────────────────────────────────────────────
    # 12. 캘리브레이션 메인
    # ──────────────────────────────────────────────
    def calibrate(self):
        print("=" * 60)
        print("2) Eye-to-Hand Calibration 수행")
        print("=" * 60)
        data = self.all_data
        print(f"전체 데이터 사용: {len(data)} poses\n")

        results = []

        # ── Phase 1: calibrateRobotWorldHandEye ──
        print("-" * 60)
        print("Phase 1: calibrateRobotWorldHandEye")
        print("-" * 60)
        for name, flag in [
            ('RW-SHAH', cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH),
            ('RW-LI',   cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI),
        ]:
            try:
                T_nb, T_em = self.solve_robot_world_hand_eye(data, flag)
                pe, re = self.evaluate_absolute_position(data, T_nb, T_em)
                results.append({
                    'method_name': name, 'T_ndi_base': T_nb,
                    'T_ee_marker': T_em, 'pos_mean': np.mean(pe),
                    'pos_max': np.max(pe), 'pos_std': np.std(pe),
                    'rot_mean': np.mean(re), 'pos_errors': pe,
                    'rot_errors': re,
                    'ndi_position_bias': np.zeros(3),
                    'ndi_axis_scale': np.ones(3),
                })
                print(f"  {name}: 위치 평균={np.mean(pe):.4f}, "
                      f"최대={np.max(pe):.4f}{ROBOT_UNIT}")
            except Exception as e:
                print(f"  {name}: 실패 - {e}")
        print()

        # ── Phase 2: calibrateHandEye eye-to-hand ──
        print("-" * 60)
        print("Phase 2: calibrateHandEye → T_ee_marker → two-step T_ndi_base")
        print("-" * 60)
        he_T_ee_markers = {}  # Hybrid 접근법용 저장
        for name, flag in [
            ('HE-TSAI',       cv2.CALIB_HAND_EYE_TSAI),
            ('HE-PARK',       cv2.CALIB_HAND_EYE_PARK),
            ('HE-HORAUD',     cv2.CALIB_HAND_EYE_HORAUD),
            ('HE-ANDREFF',    cv2.CALIB_HAND_EYE_ANDREFF),
            ('HE-DANIILIDIS', cv2.CALIB_HAND_EYE_DANIILIDIS),
        ]:
            try:
                T_em = self.solve_handeye_for_T_ee_marker(data, flag)
                he_T_ee_markers[name] = T_em
                T_nb = self.compute_T_ndi_base_from_T_ee_marker(data, T_em)
                pe, re = self.evaluate_absolute_position(data, T_nb, T_em)
                results.append({
                    'method_name': name, 'T_ndi_base': T_nb,
                    'T_ee_marker': T_em, 'pos_mean': np.mean(pe),
                    'pos_max': np.max(pe), 'pos_std': np.std(pe),
                    'rot_mean': np.mean(re), 'pos_errors': pe,
                    'rot_errors': re,
                    'ndi_position_bias': np.zeros(3),
                    'ndi_axis_scale': np.ones(3),
                })
                print(f"  {name}: 위치 평균={np.mean(pe):.4f}, "
                      f"최대={np.max(pe):.4f}{ROBOT_UNIT}")
            except Exception as e:
                print(f"  {name}: 실패 - {e}")
        print()

        # ── Phase 3: Hybrid (HE T_ee_marker + PointReg T_ndi_base) ──
        print("-" * 60)
        print("Phase 3: Hybrid (calibrateHandEye T_ee_marker + PointReg T_ndi_base)")
        print("-" * 60)
        for he_name, T_em in he_T_ee_markers.items():
            try:
                T_nb_pr, res = self.solve_point_registration(data, T_ee_marker=T_em)
                pe, re = self.evaluate_absolute_position(data, T_nb_pr, T_em)
                hybrid_name = f"Hybrid-{he_name.replace('HE-','')}"
                results.append({
                    'method_name': hybrid_name, 'T_ndi_base': T_nb_pr,
                    'T_ee_marker': T_em, 'pos_mean': np.mean(pe),
                    'pos_max': np.max(pe), 'pos_std': np.std(pe),
                    'rot_mean': np.mean(re), 'pos_errors': pe,
                    'rot_errors': re,
                    'ndi_position_bias': np.zeros(3),
                    'ndi_axis_scale': np.ones(3),
                })
                print(f"  {hybrid_name}: 위치 평균={np.mean(pe):.4f}, "
                      f"최대={np.max(pe):.4f}{ROBOT_UNIT}")
            except Exception as e:
                print(f"  Hybrid-{he_name}: 실패 - {e}")
        print()

        # ── Phase 4: Point Registration ──
        print("-" * 60)
        print("Phase 4: Point Registration 교차검증")
        print("-" * 60)
        T_nb_approx, res_approx = self.solve_point_registration(data)
        print(f"  근사 (T_ee_marker=I): 잔차 평균={np.mean(res_approx):.4f}{ROBOT_UNIT}")
        if results:
            best_so_far = min(results, key=lambda r: r['pos_mean'])
            T_nb_ref, res_ref = self.solve_point_registration(
                data, T_ee_marker=best_so_far['T_ee_marker'])
            print(f"  보정: 잔차 평균={np.mean(res_ref):.4f}{ROBOT_UNIT}")
        print()

        # ── Phase 5: 비선형 최적화 ──
        print("-" * 60)
        print("Phase 5: LM 비선형 최적화")
        print("-" * 60)
        results_sorted = sorted(results, key=lambda r: (r['pos_mean'], r['pos_max']))
        n_refine = min(3, len(results_sorted))
        for i in range(n_refine):
            src = results_sorted[i]
            try:
                # LM (위치+회전 동시 최적화)
                T_nb_opt, T_em_opt, _ = self.refine_nonlinear(
                    data, src['T_ndi_base'], src['T_ee_marker'],
                    pos_weight=1.0, rot_weight=20.0)
                pe, re = self.evaluate_absolute_position(data, T_nb_opt, T_em_opt)
                opt_name = f"{src['method_name']}+LM"
                results.append({
                    'method_name': opt_name, 'T_ndi_base': T_nb_opt,
                    'T_ee_marker': T_em_opt, 'pos_mean': np.mean(pe),
                    'pos_max': np.max(pe), 'pos_std': np.std(pe),
                    'rot_mean': np.mean(re), 'pos_errors': pe,
                    'rot_errors': re,
                    'ndi_position_bias': np.zeros(3),
                    'ndi_axis_scale': np.ones(3),
                })
                print(f"  {opt_name}: 위치 평균={np.mean(pe):.4f}, "
                      f"최대={np.max(pe):.4f}{ROBOT_UNIT}")

                # LM + 아웃라이어 제거
                T_nb_rob, T_em_rob, excluded = self.refine_with_outlier_rejection(
                    data, src['T_ndi_base'], src['T_ee_marker'])
                pe2, re2 = self.evaluate_absolute_position(data, T_nb_rob, T_em_rob)
                rob_name = f"{src['method_name']}+LM+OR"
                results.append({
                    'method_name': rob_name, 'T_ndi_base': T_nb_rob,
                    'T_ee_marker': T_em_rob, 'pos_mean': np.mean(pe2),
                    'pos_max': np.max(pe2), 'pos_std': np.std(pe2),
                    'rot_mean': np.mean(re2), 'pos_errors': pe2,
                    'rot_errors': re2,
                    'ndi_position_bias': np.zeros(3),
                    'ndi_axis_scale': np.ones(3),
                })
                excl_str = ','.join(str(x) for x in sorted(excluded)) if excluded else 'none'
                print(f"  {rob_name}: 위치 평균={np.mean(pe2):.4f}, "
                      f"최대={np.max(pe2):.4f}{ROBOT_UNIT} "
                      f"(제외: {excl_str})")

                # LM + NDI 위치 bias 보정
                T_nb_bias, T_em_bias, ndi_bias, _ = self.refine_nonlinear_with_ndi_bias(
                    data, src['T_ndi_base'], src['T_ee_marker'],
                    ndi_position_bias_init=np.zeros(3),
                    pos_weight=1.0, rot_weight=20.0, bias_reg_weight=0.02)
                pe3, re3 = self.evaluate_absolute_position(
                    data, T_nb_bias, T_em_bias, ndi_position_bias=ndi_bias)
                bias_name = f"{src['method_name']}+LM+NDI-BIAS"
                results.append({
                    'method_name': bias_name, 'T_ndi_base': T_nb_bias,
                    'T_ee_marker': T_em_bias, 'pos_mean': np.mean(pe3),
                    'pos_max': np.max(pe3), 'pos_std': np.std(pe3),
                    'rot_mean': np.mean(re3), 'pos_errors': pe3,
                    'rot_errors': re3, 'ndi_position_bias': ndi_bias,
                    'ndi_axis_scale': np.ones(3),
                })
                print(f"  {bias_name}: 위치 평균={np.mean(pe3):.4f}, "
                      f"최대={np.max(pe3):.4f}{ROBOT_UNIT}, "
                      f"bias=[{ndi_bias[0]:.4f}, {ndi_bias[1]:.4f}, {ndi_bias[2]:.4f}] {ROBOT_UNIT}")

                # LM + NDI 축별 scale/bias 보정 (비등방성 대응)
                T_nb_ax, T_em_ax, ndi_bias_ax, ndi_scale_ax, _ = self.refine_nonlinear_with_ndi_axis_scale(
                    data, src['T_ndi_base'], src['T_ee_marker'],
                    ndi_position_bias_init=np.zeros(3),
                    ndi_axis_scale_init=np.ones(3),
                    pos_weight=1.0, rot_weight=20.0,
                    bias_reg_weight=0.02, scale_reg_weight=10.0)
                pe4, re4 = self.evaluate_absolute_position(
                    data, T_nb_ax, T_em_ax,
                    ndi_position_bias=ndi_bias_ax,
                    ndi_axis_scale=ndi_scale_ax)
                axis_name = f"{src['method_name']}+LM+NDI-AXIS"
                results.append({
                    'method_name': axis_name, 'T_ndi_base': T_nb_ax,
                    'T_ee_marker': T_em_ax, 'pos_mean': np.mean(pe4),
                    'pos_max': np.max(pe4), 'pos_std': np.std(pe4),
                    'rot_mean': np.mean(re4), 'pos_errors': pe4,
                    'rot_errors': re4,
                    'ndi_position_bias': ndi_bias_ax,
                    'ndi_axis_scale': ndi_scale_ax,
                })
                print(f"  {axis_name}: 위치 평균={np.mean(pe4):.4f}, "
                      f"최대={np.max(pe4):.4f}{ROBOT_UNIT}, "
                      f"bias=[{ndi_bias_ax[0]:.4f}, {ndi_bias_ax[1]:.4f}, {ndi_bias_ax[2]:.4f}] {ROBOT_UNIT}, "
                      f"scale=[{ndi_scale_ax[0]:.6f}, {ndi_scale_ax[1]:.6f}, {ndi_scale_ax[2]:.6f}]")
            except Exception as e:
                print(f"  {src['method_name']}+LM: 실패 - {e}")
        print()

        # ── 최종 결과 ──
        if not results:
            print("모든 방법 실패")
            return

        results.sort(key=lambda r: (r['pos_mean'], r['pos_max']))
        self.all_results = results

        print("=" * 90)
        print(f"전체 결과 비교 (절대 위치 오차, 단위: {ROBOT_UNIT})")
        print("=" * 90)
        print(f"{'순위':<5} {'방법':<22} {'평균':<10} {'최대':<10} "
              f"{'std':<10} {'회전(°)':<10}")
        print("-" * 70)
        for i, res in enumerate(results):
            print(f"{i+1:<5} {res['method_name']:<22} {res['pos_mean']:<10.4f} "
                  f"{res['pos_max']:<10.4f} {res['pos_std']:<10.4f} "
                  f"{res['rot_mean']:<10.4f}")
        print()

        best = results[0]
        self.T_ndi_base = best['T_ndi_base']
        self.T_ee_marker = best['T_ee_marker']
        self.ndi_position_bias = np.array(best.get('ndi_position_bias', np.zeros(3)))
        self.ndi_axis_scale = np.array(best.get('ndi_axis_scale', np.ones(3)))
        self.best_method = best['method_name']

        print(f"최적 방법: {self.best_method}")
        print(f"  위치 오차: 평균={best['pos_mean']:.4f}, 최대={best['pos_max']:.4f}{ROBOT_UNIT}")
        print(f"  회전 오차: 평균={best['rot_mean']:.4f}°\n")

        euler_ndi = R.from_matrix(self.T_ndi_base[:3, :3]).as_euler('ZYX', degrees=True)
        euler_ee = R.from_matrix(self.T_ee_marker[:3, :3]).as_euler('ZYX', degrees=True)
        print("T_ndi_base (NDI → 로봇 베이스):")
        print(f"  t=[{self.T_ndi_base[0,3]:.4f}, {self.T_ndi_base[1,3]:.4f}, "
              f"{self.T_ndi_base[2,3]:.4f}]{ROBOT_UNIT}")
        print(f"  R(ZYX)=[{euler_ndi[0]:.4f}°, {euler_ndi[1]:.4f}°, {euler_ndi[2]:.4f}°]")
        print("T_ee_marker (EE → 마커):")
        print(f"  t=[{self.T_ee_marker[0,3]:.4f}, {self.T_ee_marker[1,3]:.4f}, "
              f"{self.T_ee_marker[2,3]:.4f}]{ROBOT_UNIT}")
        print(f"  R(ZYX)=[{euler_ee[0]:.4f}°, {euler_ee[1]:.4f}°, {euler_ee[2]:.4f}°]")
        print(f"NDI 위치 bias (카메라 좌표계): [{self.ndi_position_bias[0]:.4f}, "
              f"{self.ndi_position_bias[1]:.4f}, {self.ndi_position_bias[2]:.4f}] {ROBOT_UNIT}")
        print(f"NDI 축별 scale: [{self.ndi_axis_scale[0]:.6f}, "
              f"{self.ndi_axis_scale[1]:.6f}, {self.ndi_axis_scale[2]:.6f}]")

        t_ee = np.linalg.norm(self.T_ee_marker[:3, 3])
        t_ndi = np.linalg.norm(self.T_ndi_base[:3, 3])
        print(f"\n  [Sanity] ||t_ee_marker||={t_ee:.2f}{ROBOT_UNIT}, "
              f"||t_ndi_base||={t_ndi:.2f}{ROBOT_UNIT}")
        print(f"  [Sanity] det(R_ndi)={np.linalg.det(self.T_ndi_base[:3,:3]):.6f}, "
              f"det(R_ee)={np.linalg.det(self.T_ee_marker[:3,:3]):.6f}\n")

        self.save_calibration_result()

    # ──────────────────────────────────────────────
    # 13. 검증
    # ──────────────────────────────────────────────
    def validate_transform_chain(self, data, T_ndi_base, T_ee_marker,
                                 ndi_position_bias=None, ndi_axis_scale=None):
        """좌표계 체인 일관성 검증.

        정방향 모델(eye-to-hand):
            T_ndi_marker ≈ T_ndi_base @ T_base_ee @ T_ee_marker

        역방향(잘못된 inversion 컨벤션이 섞인 경우 감지 목적):
            T_ndi_marker ≈ inv(T_ndi_base) @ T_base_ee @ inv(T_ee_marker)

        Returns:
            dict: forward/inverse 가정의 평균 위치/회전 잔차
        """
        fw_pos, fw_rot = [], []
        inv_pos, inv_rot = [], []

        T_base_ndi = np.linalg.inv(T_ndi_base)
        T_marker_ee = np.linalg.inv(T_ee_marker)

        for _, row in data.iterrows():
            T_ndi_true = self.get_ndi_transform_with_bias(row, ndi_position_bias, ndi_axis_scale)
            T_base_ee = self.get_robot_transform(row)

            T_ndi_pred_fw = T_ndi_base @ T_base_ee @ T_ee_marker
            T_ndi_pred_inv = T_base_ndi @ T_base_ee @ T_marker_ee

            fw_pos.append(np.linalg.norm(T_ndi_pred_fw[:3, 3] - T_ndi_true[:3, 3]))
            inv_pos.append(np.linalg.norm(T_ndi_pred_inv[:3, 3] - T_ndi_true[:3, 3]))

            R_fw = T_ndi_pred_fw[:3, :3].T @ T_ndi_true[:3, :3]
            R_inv = T_ndi_pred_inv[:3, :3].T @ T_ndi_true[:3, :3]

            fw_rot.append(np.degrees(np.arccos(np.clip((np.trace(R_fw) - 1) / 2, -1, 1))))
            inv_rot.append(np.degrees(np.arccos(np.clip((np.trace(R_inv) - 1) / 2, -1, 1))))

        return {
            'forward_pos_mean': float(np.mean(fw_pos)),
            'forward_rot_mean': float(np.mean(fw_rot)),
            'inverse_pos_mean': float(np.mean(inv_pos)),
            'inverse_rot_mean': float(np.mean(inv_rot)),
        }

    def diagnose_root_causes(self, data, T_ndi_base, T_ee_marker,
                             ndi_position_bias=None, ndi_axis_scale=None):
        """잔차 패턴 기반 근본 원인 진단.

        Returns:
            dict with correlation/anisotropy indicators.
        """
        T_base_ndi = np.linalg.inv(T_ndi_base)
        T_marker_ee = np.linalg.inv(T_ee_marker)

        pos_vecs = []
        pos_norms = []
        rot_norms = []
        ndi_dists = []
        robot_dists = []
        idxs = []

        for idx, (_, row) in enumerate(data.iterrows()):
            T_ndi = self.get_ndi_transform_with_bias(row, ndi_position_bias, ndi_axis_scale)
            T_robot_true = self.get_robot_transform(row)
            T_robot_pred = T_base_ndi @ T_ndi @ T_marker_ee

            e_pos = T_robot_pred[:3, 3] - T_robot_true[:3, 3]
            pos_vecs.append(e_pos)
            pos_norms.append(np.linalg.norm(e_pos))

            R_err = T_robot_pred[:3, :3] @ T_robot_true[:3, :3].T
            rv_err = R.from_matrix(R_err).as_rotvec()
            rot_norms.append(np.linalg.norm(rv_err))

            ndi_dists.append(np.linalg.norm(T_ndi[:3, 3]))
            robot_dists.append(np.linalg.norm(T_robot_true[:3, 3]))
            idxs.append(idx)

        pos_vecs = np.array(pos_vecs)
        pos_norms = np.array(pos_norms)
        rot_norms = np.array(rot_norms)
        ndi_dists = np.array(ndi_dists)
        robot_dists = np.array(robot_dists)
        idxs = np.array(idxs, dtype=float)

        def _corr(a, b):
            if len(a) < 2:
                return 0.0
            if np.std(a) < 1e-12 or np.std(b) < 1e-12:
                return 0.0
            return float(np.corrcoef(a, b)[0, 1])

        axis_std = np.std(pos_vecs, axis=0)
        axis_mean_abs = np.mean(np.abs(pos_vecs), axis=0)
        dominant_axis = int(np.argmax(axis_mean_abs))
        anisotropy = float((np.max(axis_std) + 1e-12) / (np.min(axis_std) + 1e-12))

        # 시간/순번 경향은 동기화·드리프트 의심 신호
        corr_idx = _corr(idxs, pos_norms)
        corr_ndi_dist = _corr(ndi_dists, pos_norms)
        corr_robot_dist = _corr(robot_dists, pos_norms)
        corr_rot_pos = _corr(rot_norms, pos_norms)

        suggestions = []
        if abs(corr_idx) > 0.35:
            suggestions.append('시간/순번 대비 오차 상관이 큼: 타임스탬프 동기화, 드리프트, 체결 미세 슬립 점검 권장')
        if abs(corr_ndi_dist) > 0.35:
            suggestions.append('카메라 원점 거리와 오차 상관이 큼: 작업거리/시야각/광학 왜곡/가림 영향 점검 권장')
        if anisotropy > 1.8:
            suggestions.append('축별 비등방성 오차가 큼: 베이스-카메라 축정의, TCP 방향성 편향, 로봇 축별 강성 점검 권장')
        if abs(corr_rot_pos) > 0.35:
            suggestions.append('회전오차와 위치오차 결합이 큼: tool-tip 오프셋, EE-마커 lever-arm, pivot 보정 재점검 권장')
        if not suggestions:
            suggestions.append('뚜렷한 단일 패턴 없음: 데이터 다양성 확대(거리/각도/영역) 후 재추정 권장')

        return {
            'corr_pose_index_vs_pos': corr_idx,
            'corr_ndi_distance_vs_pos': corr_ndi_dist,
            'corr_robot_distance_vs_pos': corr_robot_dist,
            'corr_rot_vs_pos': corr_rot_pos,
            'axis_std_xyz': axis_std.tolist(),
            'axis_mean_abs_xyz': axis_mean_abs.tolist(),
            'anisotropy_ratio': anisotropy,
            'dominant_axis': ['x', 'y', 'z'][dominant_axis],
            'suggestions': suggestions,
        }

    def validate_all_data(self):
        print("=" * 60)
        print(f"3) 전체 데이터 검증 (절대 위치 오차, 단위: {ROBOT_UNIT})")
        print("=" * 60)

        pe, re = self.evaluate_absolute_position(
            self.all_data, self.T_ndi_base, self.T_ee_marker,
            ndi_position_bias=self.ndi_position_bias,
            ndi_axis_scale=self.ndi_axis_scale)

        print(f"\n{'Pose':<8} {'위치오차(mm)':<15} {'회전오차(°)':<15}")
        print("-" * 38)
        for i, (p, r_) in enumerate(zip(pe, re)):
            flag = " ***" if p > 1.0 else ""
            print(f"{i:<8} {p:<15.4f} {r_:<15.4f}{flag}")

        print(f"\n위치: 평균={np.mean(pe):.4f}, std={np.std(pe):.4f}, "
              f"min={np.min(pe):.4f}, max={np.max(pe):.4f}{ROBOT_UNIT}")
        print(f"회전: 평균={np.mean(re):.4f}°, std={np.std(re):.4f}°")

        chain = self.validate_transform_chain(
            self.all_data, self.T_ndi_base, self.T_ee_marker,
            ndi_position_bias=self.ndi_position_bias,
            ndi_axis_scale=self.ndi_axis_scale)
        print("\n[좌표계 체인 검증]")
        print(f"  정방향(T_ndi_base @ T_base_ee @ T_ee_marker)"
              f": pos={chain['forward_pos_mean']:.4f}{ROBOT_UNIT}, "
              f"rot={chain['forward_rot_mean']:.4f}°")
        print(f"  역방향(inv(T_ndi_base) @ T_base_ee @ inv(T_ee_marker))"
              f": pos={chain['inverse_pos_mean']:.4f}{ROBOT_UNIT}, "
              f"rot={chain['inverse_rot_mean']:.4f}°")

        diag = self.diagnose_root_causes(
            self.all_data, self.T_ndi_base, self.T_ee_marker,
            ndi_position_bias=self.ndi_position_bias,
            ndi_axis_scale=self.ndi_axis_scale)
        print("\n[근본 원인 진단 힌트]")
        print(f"  corr(pose_index, pos_err)={diag['corr_pose_index_vs_pos']:.3f}")
        print(f"  corr(||t_ndi||, pos_err)={diag['corr_ndi_distance_vs_pos']:.3f}")
        print(f"  corr(||t_robot||, pos_err)={diag['corr_robot_distance_vs_pos']:.3f}")
        print(f"  corr(rot_err, pos_err)={diag['corr_rot_vs_pos']:.3f}")
        print(f"  axis_std xyz={np.array(diag['axis_std_xyz'])}")
        print(f"  anisotropy={diag['anisotropy_ratio']:.3f}, dominant_axis={diag['dominant_axis']}")
        for s_msg in diag['suggestions']:
            print(f"   - {s_msg}")

        target_met = np.mean(pe) < 1.0
        print(f"\n목표 (평균 < 1mm): {'YES' if target_met else 'NO'} "
              f"(현재: {np.mean(pe):.4f}{ROBOT_UNIT})\n")
        return pe, re

    # ──────────────────────────────────────────────
    # 14. 저장
    # ──────────────────────────────────────────────
    def save_calibration_result(self):
        output_dir = self.result_json_path.parent
        output_dir.mkdir(parents=True, exist_ok=True)

        euler_ndi = R.from_matrix(
            self.T_ndi_base[:3, :3]).as_euler('ZYX', degrees=True)
        euler_ee = R.from_matrix(
            self.T_ee_marker[:3, :3]).as_euler('ZYX', degrees=True)

        result = {
            'T_ndi_base': {
                'matrix': self.T_ndi_base.tolist(),
                'translation_vector': self.T_ndi_base[:3, 3].tolist(),
                'euler_ZYX_deg': {
                    'Z': float(euler_ndi[0]), 'Y': float(euler_ndi[1]),
                    'X': float(euler_ndi[2]),
                },
                'description': 'NDI camera frame -> Robot base frame',
            },
            'T_ee_marker': {
                'matrix': self.T_ee_marker.tolist(),
                'translation_vector': self.T_ee_marker[:3, 3].tolist(),
                'euler_ZYX_deg': {
                    'Z': float(euler_ee[0]), 'Y': float(euler_ee[1]),
                    'X': float(euler_ee[2]),
                },
                'description': 'Robot EE frame -> Marker frame',
            },
            'ndi_position_bias': self.ndi_position_bias.tolist(),
            'ndi_axis_scale': self.ndi_axis_scale.tolist(),
            'translation_unit': ROBOT_UNIT,
            'method': self.best_method,
            'num_poses': len(self.all_data),
            'navigation_formula':
                'T_base_ee_target = inv(T_ndi_base) @ T_ndi_marker_new @ inv(T_ee_marker)',
        }
        output_path = self.result_json_path
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(result, f, indent=2)
        print(f"Saved result: {output_path}\n")

        legacy_path = output_dir / 'calibration_result.json'
        if legacy_path != output_path:
            shutil.copyfile(output_path, legacy_path)
            print(f"Synced default path: {legacy_path}\n")

    # ──────────────────────────────────────────────
    # 15. 네비게이션 헬퍼
    # ──────────────────────────────────────────────
    def predict_ee_from_ndi(self, T_ndi_marker_new):
        """NDI 마커 관측 → 로봇 EE 목표 좌표.

        Returns: (T_base_ee_target, p_target [x,y,z], euler_target [u,v,w])
        """
        T_base_ndi = np.linalg.inv(self.T_ndi_base)
        T_marker_ee = np.linalg.inv(self.T_ee_marker)
        T_ndi_marker_corr = T_ndi_marker_new.copy()
        T_ndi_marker_corr[:3, 3] = self.ndi_axis_scale * T_ndi_marker_corr[:3, 3] + self.ndi_position_bias
        T_target = T_base_ndi @ T_ndi_marker_corr @ T_marker_ee

        p = T_target[:3, 3]
        zyx = R.from_matrix(T_target[:3, :3]).as_euler('ZYX', degrees=True)
        euler = np.array([zyx[2], zyx[1], zyx[0]])  # [u, v, w]
        return T_target, p, euler

    # ──────────────────────────────────────────────
    # 16. 시각화
    # ──────────────────────────────────────────────
    def visualize_results(self, pos_errors, rot_errors):
        print("=" * 60)
        print("4) 결과 시각화")
        print("=" * 60)

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        ax = axes[0, 0]
        ax.bar(range(len(pos_errors)), pos_errors, alpha=0.7, color='steelblue')
        ax.axhline(np.mean(pos_errors), color='red', linestyle='--',
                   label=f'Mean: {np.mean(pos_errors):.4f} {ROBOT_UNIT}')
        ax.axhline(1.0, color='orange', linestyle=':', label='Target: 1.0 mm')
        ax.set_xlabel('Pose Index')
        ax.set_ylabel(f'Position Error ({ROBOT_UNIT})')
        ax.set_title('Absolute Position Error per Pose')
        ax.legend(); ax.grid(True, alpha=0.3)

        ax = axes[0, 1]
        ax.bar(range(len(rot_errors)), rot_errors, alpha=0.7, color='seagreen')
        ax.axhline(np.mean(rot_errors), color='red', linestyle='--',
                   label=f'Mean: {np.mean(rot_errors):.4f}°')
        ax.set_xlabel('Pose Index')
        ax.set_ylabel('Rotation Error (°)')
        ax.set_title('Absolute Rotation Error per Pose')
        ax.legend(); ax.grid(True, alpha=0.3)

        ax = axes[1, 0]
        ax.hist(pos_errors, bins=10, color='steelblue', edgecolor='black', alpha=0.7)
        ax.axvline(1.0, color='orange', linestyle=':', label='Target: 1.0 mm')
        ax.set_xlabel(f'Position Error ({ROBOT_UNIT})')
        ax.set_ylabel('Frequency')
        ax.set_title('Position Error Distribution')
        ax.legend(); ax.grid(True, alpha=0.3)

        ax = axes[1, 1]
        if self.all_results:
            names = [r['method_name'] for r in self.all_results]
            means = [r['pos_mean'] for r in self.all_results]
            colors = ['gold' if i == 0 else 'lightgray' for i in range(len(names))]
            ax.barh(range(len(names)), means, color=colors, edgecolor='black', alpha=0.8)
            ax.set_yticks(range(len(names)))
            ax.set_yticklabels(names, fontsize=8)
            ax.set_xlabel(f'Mean Position Error ({ROBOT_UNIT})')
            ax.set_title('Method Comparison')
            ax.axvline(1.0, color='orange', linestyle=':', label='Target: 1.0 mm')
            ax.legend(); ax.grid(True, alpha=0.3)
            ax.invert_yaxis()

        config = (f"Method: {self.best_method} | Poses: {len(self.all_data)} | "
                  f"Pos mean: {np.mean(pos_errors):.4f}{ROBOT_UNIT} | "
                  f"Rot mean: {np.mean(rot_errors):.4f}°")
        fig.text(0.5, 0.01, config, ha='center', fontsize=10,
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout(rect=[0, 0.04, 1, 1])
        out = self.result_png_path
        out.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(out, dpi=150, bbox_inches='tight')
        print(f"시각화 저장: {out}")
        plt.close()
        print()

    # ──────────────────────────────────────────────
    # 17. 전체 실행
    # ──────────────────────────────────────────────
    def run(self):
        self.load_and_preprocess_data()
        self.calibrate()
        pe, re = self.validate_all_data()
        self.visualize_results(pe, re)
        print("=" * 60)
        print("캘리브레이션 완료!")
        print("=" * 60)


if __name__ == "__main__":
    import sys
    csv_path = sys.argv[1] if len(sys.argv) > 1 else './dataset/calibration/calibration_data_broad.csv'
    print(f"입력 파일: {csv_path}\n")
    calibration = HandEyeCalibration(csv_path=csv_path)
    calibration.run()
