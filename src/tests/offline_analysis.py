"""
offline_analysis.py – CSV 로그 기반 Trajectory 분석 및 시각화

[위치]
  src/tests/offline_analysis.py

[개요]
  online_estimation.py 가 저장한 CSV 로그를 불러와 아래 분석을 수행한다.
    1. Trajectory comparison    – 추정·실측 궤적 3D / 2D 비교
    2. Position error plot       – 시간축 위치 오차 (mm)
    3. Rotation error plot       – 시간축 자세 오차 (deg, geodesic)
    4. Axis-wise error           – x/y/z 축별 오차 time-series
    5. Error distribution        – 위치·자세 오차 히스토그램
    6. Dashboard                 – 위 항목을 하나의 이미지로 통합

[CSV 컬럼 규격]
  추정값 컬럼: pred_x, pred_y, pred_z, pred_u, pred_v, pred_w
  실측값 컬럼: real_x, real_y, real_z, real_u, real_v, real_w
  오차   컬럼: pos_err_mm, rot_err_geodesic_deg, err_x, err_y, err_z

  ※ 구버전(est_x / est_y …) CSV 도 자동으로 호환 처리한다.

[실행 예시]
  cd C:\\Users\\...\\HandEye_Calibration

  # 기본값
  python -m src.tests.offline_analysis

  # 파일 직접 지정
  python -m src.tests.offline_analysis ^
      --csv  dataset/logs/online_log.csv ^
      --out  dataset/logs/analysis ^
      --pos-thr 5.0 ^
      --rot-thr 3.0 ^
      --show
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ══════════════════════════════════════════════════════════════════════════════
# 프로젝트 루트 탐색
# ══════════════════════════════════════════════════════════════════════════════

def _find_project_root() -> Path:
    here = Path(__file__).resolve().parent
    for candidate in [here] + [here.parents[i] for i in range(5)]:
        if (candidate / "config.json").exists():
            return candidate
    return Path.cwd()


PROJECT_ROOT = _find_project_root()

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.utils.logger import get_logger, configure_logging

configure_logging()
log = get_logger(__name__)


# ══════════════════════════════════════════════════════════════════════════════
# 컬럼명 자동 감지 (버전 호환)
# ══════════════════════════════════════════════════════════════════════════════

def detect_prefixes(df: pd.DataFrame) -> tuple:
    """
    CSV 헤더를 검사하여 (pred_prefix, real_prefix) 를 반환한다.

    지원 형식:
      - online_estimation.py 저장 형식: pred_x / real_x  → ("pred", "real")
      - 구버전 형식:                    est_x  / real_x  → ("est",  "real")
    """
    if "pred_x" in df.columns:
        return "pred", "real"
    elif "est_x" in df.columns:
        log.warning("CSV 컬럼이 구버전 형식(est_x)입니다. 자동 호환 처리합니다.")
        return "est", "real"
    else:
        raise KeyError(
            "CSV 에서 추정값 컬럼(pred_x 또는 est_x)을 찾을 수 없습니다.\n"
            f"  실제 컬럼 목록: {list(df.columns)}"
        )


# ══════════════════════════════════════════════════════════════════════════════
# 오차 컬럼명 – online_estimation.py 저장 기준
# ══════════════════════════════════════════════════════════════════════════════

COL_POS_ERR = "pos_err_mm"
COL_ROT_ERR = "rot_err_geodesic_deg"
COL_ERR_X   = "err_x"
COL_ERR_Y   = "err_y"
COL_ERR_Z   = "err_z"


# ══════════════════════════════════════════════════════════════════════════════
# 데이터 로드 & 전처리
# ══════════════════════════════════════════════════════════════════════════════

def load_log(csv_path: str) -> pd.DataFrame:
    """
    CSV 로그 파일을 로드하고 기본 전처리를 수행한다.

    추가 컬럼:
        time_sec  : 첫 행 기준 경과 시간 (초)
        frame_idx : 0-based 행 인덱스
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV 파일을 찾을 수 없습니다: {csv_path}")

    df = pd.read_csv(csv_path, encoding="utf-8-sig")
    log.info(f"CSV 로드 완료: {len(df)} 행  ({csv_path})")

    # timestamp → 경과 시간(초)
    try:
        df["_dt"] = pd.to_datetime(df["timestamp"])
        t0 = df["_dt"].iloc[0]
        df["time_sec"] = (df["_dt"] - t0).dt.total_seconds()
    except Exception:
        df["time_sec"] = df.index.astype(float)

    df["frame_idx"] = df.index

    # 오차 컬럼 없으면 0 으로 대체 (구버전 CSV 호환)
    for col in [COL_POS_ERR, COL_ROT_ERR, COL_ERR_X, COL_ERR_Y, COL_ERR_Z]:
        if col not in df.columns:
            df[col] = 0.0
            log.warning(f"컬럼 '{col}' 없음 → 0 으로 대체")

    valid = df[COL_POS_ERR].notna().sum()
    log.info(f"오차 분석 가능 행: {valid} 행")
    return df


# ══════════════════════════════════════════════════════════════════════════════
# 기술 통계
# ══════════════════════════════════════════════════════════════════════════════

def summary_stats(df: pd.DataFrame,
                  pos_threshold: float,
                  rot_threshold: float) -> dict:
    pos  = df[COL_POS_ERR].dropna()
    rot  = df[COL_ROT_ERR].dropna()
    high = ((pos > pos_threshold) | (rot > rot_threshold)).sum()
    return dict(
        n_frames=len(df),
        n_high=int(high),
        pct_high=100.0 * high / max(len(df), 1),
        pos_mean=pos.mean(),   pos_median=pos.median(),
        pos_min=pos.min(),     pos_max=pos.max(),    pos_std=pos.std(),
        rot_mean=rot.mean(),   rot_median=rot.median(),
        rot_min=rot.min(),     rot_max=rot.max(),    rot_std=rot.std(),
    )


def print_summary(stats: dict):
    sep = '=' * 60
    print(f"\n{sep}")
    print("  Trajectory 오차 분석 요약")
    print(sep)
    print(f"  총 프레임 수          : {stats['n_frames']}")
    print(f"  고오차 구간 프레임    : {stats['n_high']} ({stats['pct_high']:.1f}%)")
    print(f"  [위치 오차 (mm)]")
    print(f"    평균    : {stats['pos_mean']:.4f}")
    print(f"    중앙값  : {stats['pos_median']:.4f}")
    print(f"    최솟값  : {stats['pos_min']:.4f}")
    print(f"    최댓값  : {stats['pos_max']:.4f}")
    print(f"    표준편차: {stats['pos_std']:.4f}")
    print(f"  [회전 오차 (deg)]")
    print(f"    평균    : {stats['rot_mean']:.4f}")
    print(f"    중앙값  : {stats['rot_median']:.4f}")
    print(f"    최솟값  : {stats['rot_min']:.4f}")
    print(f"    최댓값  : {stats['rot_max']:.4f}")
    print(f"    표준편차: {stats['rot_std']:.4f}")
    print(sep)


# ══════════════════════════════════════════════════════════════════════════════
# 고오차 구간 리포트
# ══════════════════════════════════════════════════════════════════════════════

def print_high_error_report(df: pd.DataFrame, real_pfx: str,
                             pos_threshold: float, rot_threshold: float):
    """임계값을 초과하는 프레임 목록을 콘솔에 출력한다."""
    mask = (df[COL_POS_ERR] > pos_threshold) | (df[COL_ROT_ERR] > rot_threshold)
    seg  = df[mask]

    print(f"\n{'='*70}")
    print(f"  고오차 구간 리포트  "
          f"(pos > {pos_threshold} mm  또는  rot > {rot_threshold} deg)")
    print(f"{'='*70}")

    if seg.empty:
        print("  임계값을 초과하는 구간이 없습니다.")
        return

    print(f"  총 {len(seg)} 프레임 / 전체 {len(df)} 프레임 "
          f"({100*len(seg)/len(df):.1f}%)\n")
    print(f"  {'Frame':>6}  {'Time(s)':>8}  {'pos_err':>9}  {'rot_err':>9}  "
          f"{'real_x':>9}  {'real_y':>9}  {'real_z':>9}")
    print(f"  {'-'*68}")

    for _, row in seg.iterrows():
        print(f"  {int(row['frame_idx']):>6}  "
              f"{row['time_sec']:>8.3f}  "
              f"{row[COL_POS_ERR]:>9.4f}  "
              f"{row[COL_ROT_ERR]:>9.4f}  "
              f"{row[f'{real_pfx}_x']:>9.2f}  "
              f"{row[f'{real_pfx}_y']:>9.2f}  "
              f"{row[f'{real_pfx}_z']:>9.2f}")


# ══════════════════════════════════════════════════════════════════════════════
# 시각화 공통 유틸
# ══════════════════════════════════════════════════════════════════════════════

_COLOR_PRED = "#2196F3"
_COLOR_REAL = "#FF5722"
_COLOR_ERR  = "#9C27B0"
_COLOR_WARN = "#F44336"


def _shade_regions(ax, t: np.ndarray, mask: np.ndarray,
                   color: str, alpha: float = 0.2, label: str = ""):
    """mask 가 True 인 연속 구간에 배경 음영을 채운다."""
    labeled = False
    in_seg  = False
    t_start = 0.0
    for i, (ti, mi) in enumerate(zip(t, mask)):
        if mi and not in_seg:
            t_start = ti; in_seg = True
        elif not mi and in_seg:
            kw = {"label": label} if not labeled else {}
            ax.axvspan(t_start, t[i - 1], color=color, alpha=alpha, **kw)
            labeled = True; in_seg = False
    if in_seg:
        kw = {"label": label} if not labeled else {}
        ax.axvspan(t_start, t[-1], color=color, alpha=alpha, **kw)


def _save(fig: plt.Figure, path: str, show: bool):
    """그래프를 파일로 저장하고 필요 시 화면에 표시한다."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    log.success(f"저장: {path}")
    if show:
        plt.show()
    plt.close(fig)


# ══════════════════════════════════════════════════════════════════════════════
# 시각화 함수
# ══════════════════════════════════════════════════════════════════════════════

def plot_trajectory_3d(df: pd.DataFrame, pred_pfx: str, real_pfx: str,
                       out_path: str, show: bool = False):
    """추정·실측 궤적을 3D 공간에 함께 그린다."""
    fig = plt.figure(figsize=(10, 8))
    ax  = fig.add_subplot(111, projection='3d')

    px, py, pz = f"{pred_pfx}_x", f"{pred_pfx}_y", f"{pred_pfx}_z"
    rx, ry, rz = f"{real_pfx}_x", f"{real_pfx}_y", f"{real_pfx}_z"

    ax.plot(df[px], df[py], df[pz],
            color=_COLOR_PRED, linewidth=1.2, label="Estimated", alpha=0.85)
    ax.scatter(df[px].iloc[0],  df[py].iloc[0],  df[pz].iloc[0],
               color=_COLOR_PRED, s=60, marker='o', zorder=5)
    ax.scatter(df[px].iloc[-1], df[py].iloc[-1], df[pz].iloc[-1],
               color=_COLOR_PRED, s=60, marker='^', zorder=5)

    ax.plot(df[rx], df[ry], df[rz],
            color=_COLOR_REAL, linewidth=1.2, label="Ground Truth",
            alpha=0.85, linestyle="--")
    ax.scatter(df[rx].iloc[0],  df[ry].iloc[0],  df[rz].iloc[0],
               color=_COLOR_REAL, s=60, marker='o', zorder=5)
    ax.scatter(df[rx].iloc[-1], df[ry].iloc[-1], df[rz].iloc[-1],
               color=_COLOR_REAL, s=60, marker='^', zorder=5)

    ax.set_xlabel("X (mm)"); ax.set_ylabel("Y (mm)"); ax.set_zlabel("Z (mm)")
    ax.set_title("3D Trajectory Comparison  (○=start  △=end)")
    ax.legend()
    fig.tight_layout()
    _save(fig, out_path, show)


def plot_trajectory_2d(df: pd.DataFrame, pred_pfx: str, real_pfx: str,
                       out_path: str, show: bool = False):
    """XY / XZ / YZ 세 평면에 추정·실측 궤적을 나란히 그린다."""
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    planes = [
        ("X (mm)", "Y (mm)", f"{pred_pfx}_x", f"{pred_pfx}_y",
                              f"{real_pfx}_x",  f"{real_pfx}_y"),
        ("X (mm)", "Z (mm)", f"{pred_pfx}_x", f"{pred_pfx}_z",
                              f"{real_pfx}_x",  f"{real_pfx}_z"),
        ("Y (mm)", "Z (mm)", f"{pred_pfx}_y", f"{pred_pfx}_z",
                              f"{real_pfx}_y",  f"{real_pfx}_z"),
    ]
    for ax, (xl, yl, px, py, rx, ry) in zip(axes, planes):
        ax.plot(df[px], df[py], color=_COLOR_PRED, linewidth=1.0,
                label="Estimated", alpha=0.85)
        ax.plot(df[rx], df[ry], color=_COLOR_REAL, linewidth=1.0,
                linestyle="--", label="Ground Truth", alpha=0.85)
        ax.set_xlabel(xl); ax.set_ylabel(yl)
        ax.set_title(f"{xl[0]}-{yl[0]} Plane")
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    fig.suptitle("2D Trajectory Comparison", fontsize=13, y=1.01)
    fig.tight_layout()
    _save(fig, out_path, show)

# ══════════════════════════════════════════════════════════════════════════════
# Plotly 통합 궤적 시각화 (인터랙티브 HTML)
# ══════════════════════════════════════════════════════════════════════════════

def plot_trajectory_plotly(df: pd.DataFrame, pred_pfx: str, real_pfx: str,
                           out_path: str, show: bool = False):
    """
    실제 로봇(Ground Truth)과 추정 위치(Estimated)의 이동 경로를
    Plotly 로 한꺼번에 시각화한다.

    레이아웃:
    - 상단: 3D Trajectory (추정 / 실측 동시 표시)
    - 하단 좌: XY 평면 투영
    - 하단 중: XZ 평면 투영
    - 하단 우: YZ 평면 투영

    출력: HTML (인터랙티브) + PNG (정적 스냅샷)
    """
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        log.error("plotly 가 설치되어 있지 않습니다. `pip install plotly` 후 재실행하세요.")
        return

    px_col = f"{pred_pfx}_x"; py_col = f"{pred_pfx}_y"; pz_col = f"{pred_pfx}_z"
    rx_col = f"{real_pfx}_x"; ry_col = f"{real_pfx}_y"; rz_col = f"{real_pfx}_z"

    # ── 컬러 설정 ─────────────────────────────────────────────────────────────
    C_PRED = "#2196F3"   # 파란색 – 추정값
    C_REAL = "#FF5722"   # 주황색 – 실측값

    # ── 서브플롯 구성: 3D(상단 전체) + 2D x 3(하단) ───────────────────────
    fig = make_subplots(
        rows=2, cols=3,
        specs=[
            [{"type": "scene", "colspan": 3}, None, None],
            [{"type": "xy"},   {"type": "xy"}, {"type": "xy"}],
        ],
        subplot_titles=(
            "3D Trajectory (Estimated vs Ground Truth)",
            "", "", "",
            "XY Plane", "XZ Plane", "YZ Plane",
        ),
        vertical_spacing=0.12,
        horizontal_spacing=0.06,
    )

    frame_idx = df["frame_idx"].values

    # ── 3D 궤적 ──────────────────────────────────────────────────────────────
    # 추정 궤적
    fig.add_trace(go.Scatter3d(
        x=df[px_col], y=df[py_col], z=df[pz_col],
        mode="lines+markers",
        name="Estimated",
        line=dict(color=C_PRED, width=3),
        marker=dict(size=2, color=C_PRED),
        customdata=frame_idx,
        hovertemplate=(
            "Frame: %{customdata}<br>"
            "X: %{x:.2f} mm<br>Y: %{y:.2f} mm<br>Z: %{z:.2f} mm<extra>Estimated</extra>"
        ),
    ), row=1, col=1)

    # 시작/끝 마커 (추정)
    for i, symbol, label in [(0, "circle", "Est. Start"), (-1, "diamond", "Est. End")]:
        fig.add_trace(go.Scatter3d(
            x=[df[px_col].iloc[i]], y=[df[py_col].iloc[i]], z=[df[pz_col].iloc[i]],
            mode="markers",
            name=label,
            marker=dict(size=8, color=C_PRED, symbol=symbol,
                        line=dict(color="white", width=1)),
            showlegend=True,
        ), row=1, col=1)

    # 실측 궤적
    fig.add_trace(go.Scatter3d(
        x=df[rx_col], y=df[ry_col], z=df[rz_col],
        mode="lines+markers",
        name="Ground Truth",
        line=dict(color=C_REAL, width=3, dash="dash"),
        marker=dict(size=2, color=C_REAL),
        customdata=frame_idx,
        hovertemplate=(
            "Frame: %{customdata}<br>"
            "X: %{x:.2f} mm<br>Y: %{y:.2f} mm<br>Z: %{z:.2f} mm<extra>Ground Truth</extra>"
        ),
    ), row=1, col=1)

    # 시작/끝 마커 (실측)
    for i, symbol, label in [(0, "circle", "GT Start"), (-1, "diamond", "GT End")]:
        fig.add_trace(go.Scatter3d(
            x=[df[rx_col].iloc[i]], y=[df[ry_col].iloc[i]], z=[df[rz_col].iloc[i]],
            mode="markers",
            name=label,
            marker=dict(size=8, color=C_REAL, symbol=symbol,
                        line=dict(color="white", width=1)),
            showlegend=True,
        ), row=1, col=1)

    # ── 연결선 (각 프레임의 추정↔실측 오차 표시) ─────────────────────────
    # 위치 오차가 큰 프레임만 연결선 표시 (전체 표시 시 너무 지저분해짐)
    pos_err = df[COL_POS_ERR].values
    high_mask = pos_err > pos_err.mean() + pos_err.std()
    for idx in df.index[high_mask]:
        fig.add_trace(go.Scatter3d(
            x=[df[px_col].iloc[idx], df[rx_col].iloc[idx]],
            y=[df[py_col].iloc[idx], df[ry_col].iloc[idx]],
            z=[df[pz_col].iloc[idx], df[rz_col].iloc[idx]],
            mode="lines",
            line=dict(color="rgba(200,50,50,0.4)", width=1),
            showlegend=False,
            hoverinfo="skip",
        ), row=1, col=1)

    # ── 2D 평면 투영 ─────────────────────────────────────────────────────────
    planes_2d = [
        # (row, col, x_pred, y_pred, x_real, y_real, xlabel, ylabel)
        (2, 1, px_col, py_col, rx_col, ry_col, "X (mm)", "Y (mm)"),
        (2, 2, px_col, pz_col, rx_col, rz_col, "X (mm)", "Z (mm)"),
        (2, 3, py_col, pz_col, ry_col, rz_col, "Y (mm)", "Z (mm)"),
    ]

    for row, col, xp, yp, xr, yr, xl, yl in planes_2d:
        # 추정
        fig.add_trace(go.Scatter(
            x=df[xp], y=df[yp],
            mode="lines",
            name="Estimated",
            line=dict(color=C_PRED, width=1.5),
            customdata=frame_idx,
            hovertemplate=f"Frame: %{{customdata}}<br>{xl}: %{{x:.2f}}<br>{yl}: %{{y:.2f}}<extra>Estimated</extra>",
            showlegend=(col == 1),  # 첫 번째 패널에서만 범례 표시
        ), row=row, col=col)
        # 실측
        fig.add_trace(go.Scatter(
            x=df[xr], y=df[yr],
            mode="lines",
            name="Ground Truth",
            line=dict(color=C_REAL, width=1.5, dash="dash"),
            customdata=frame_idx,
            hovertemplate=f"Frame: %{{customdata}}<br>{xl}: %{{x:.2f}}<br>{yl}: %{{y:.2f}}<extra>Ground Truth</extra>",
            showlegend=(col == 1),
        ), row=row, col=col)

        # 축 레이블
        axis_idx = (col - 1) + (0 if row == 1 else 3)  # subplot 인덱싱
        xaxis_key = f"xaxis{axis_idx + 2}" if axis_idx > 0 else "xaxis2"
        yaxis_key = f"yaxis{axis_idx + 2}" if axis_idx > 0 else "yaxis2"
        fig.update_layout(**{
            xaxis_key: dict(title=xl),
            yaxis_key: dict(title=yl),
        })

    # ── 3D 축 레이블 ─────────────────────────────────────────────────────────
    fig.update_scenes(
        xaxis_title="X (mm)",
        yaxis_title="Y (mm)",
        zaxis_title="Z (mm)",
    )

    # ── 전체 레이아웃 ─────────────────────────────────────────────────────────
    fig.update_layout(
        title=dict(
            text="Trajectory Comparison – Estimated vs Ground Truth",
            font=dict(size=16),
            x=0.5,
        ),
        height=900,
        template="plotly_white",
        legend=dict(
            x=1.02, y=0.95,
            bgcolor="rgba(255,255,255,0.8)",
            bordercolor="#cccccc",
            borderwidth=1,
        ),
        margin=dict(l=40, r=160, t=80, b=40),
    )

    # ── 저장 ─────────────────────────────────────────────────────────────────
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)

    # HTML (인터랙티브)
    html_path = out_path.replace(".png", ".html")
    fig.write_html(html_path, include_plotlyjs="cdn")
    log.success(f"저장 (인터랙티브 HTML): {html_path}")

    if show:
        fig.show()


def plot_position_error(df: pd.DataFrame, out_path: str,
                        pos_threshold: float, show: bool = False):
    """시간축 위치 오차 + 임계값 초과 구간 음영."""
    fig, ax = plt.subplots(figsize=(14, 4))
    t = df["time_sec"].values
    e = df[COL_POS_ERR].values
    ax.plot(t, e, color=_COLOR_ERR, linewidth=0.9, label="pos_err (mm)")
    ax.axhline(pos_threshold, color=_COLOR_WARN, linestyle="--",
               linewidth=1.2, label=f"threshold {pos_threshold} mm")
    _shade_regions(ax, t, e > pos_threshold, color=_COLOR_WARN,
                   alpha=0.15, label="above threshold")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Position Error (mm)")
    ax.set_title("Position Error over Time")
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    _save(fig, out_path, show)


def plot_rotation_error(df: pd.DataFrame, out_path: str,
                        rot_threshold: float, show: bool = False):
    """시간축 자세 오차(geodesic) + 임계값 초과 구간 음영."""
    fig, ax = plt.subplots(figsize=(14, 4))
    t = df["time_sec"].values
    e = df[COL_ROT_ERR].values
    ax.plot(t, e, color="#009688", linewidth=0.9, label="rot_err_geodesic (deg)")
    ax.axhline(rot_threshold, color=_COLOR_WARN, linestyle="--",
               linewidth=1.2, label=f"threshold {rot_threshold} deg")
    _shade_regions(ax, t, e > rot_threshold, color=_COLOR_WARN,
                   alpha=0.15, label="above threshold")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Rotation Error (deg)")
    ax.set_title("Rotation Error over Time  (geodesic distance)")
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    _save(fig, out_path, show)


def plot_axis_error(df: pd.DataFrame, out_path: str, show: bool = False):
    """x/y/z 축별 위치 오차 time-series (3행 subplot)."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    t = df["time_sec"].values
    for ax, col, color, label in zip(
        axes,
        [COL_ERR_X, COL_ERR_Y, COL_ERR_Z],
        ["#E91E63", "#4CAF50", "#2196F3"],
        ["err_x (mm)", "err_y (mm)", "err_z (mm)"],
    ):
        ax.plot(t, df[col].values, color=color, linewidth=0.9, label=label)
        ax.axhline(0, color="gray", linewidth=0.6, linestyle=":")
        ax.set_ylabel(label); ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Axis-wise Position Error  (x / y / z)", fontsize=13)
    fig.tight_layout()
    _save(fig, out_path, show)


def plot_error_distribution(df: pd.DataFrame, out_path: str,
                             pos_threshold: float, rot_threshold: float,
                             show: bool = False):
    """위치·자세 오차 분포 히스토그램 (나란히 2개)."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    pos_data = df[COL_POS_ERR].dropna()
    rot_data = df[COL_ROT_ERR].dropna()

    ax1.hist(pos_data, bins=40, color=_COLOR_ERR, edgecolor="white", alpha=0.8)
    ax1.axvline(pos_threshold, color=_COLOR_WARN, linestyle="--",
                label=f"threshold {pos_threshold} mm")
    ax1.axvline(pos_data.mean(), color="black", linestyle="-.",
                label=f"mean {pos_data.mean():.3f} mm")
    ax1.set_xlabel("Position Error (mm)"); ax1.set_ylabel("Count")
    ax1.set_title("Position Error Distribution")
    ax1.legend(); ax1.grid(True, alpha=0.3)

    ax2.hist(rot_data, bins=40, color="#009688", edgecolor="white", alpha=0.8)
    ax2.axvline(rot_threshold, color=_COLOR_WARN, linestyle="--",
                label=f"threshold {rot_threshold} deg")
    ax2.axvline(rot_data.mean(), color="black", linestyle="-.",
                label=f"mean {rot_data.mean():.3f} deg")
    ax2.set_xlabel("Rotation Error (deg)"); ax2.set_ylabel("Count")
    ax2.set_title("Rotation Error Distribution")
    ax2.legend(); ax2.grid(True, alpha=0.3)

    fig.suptitle("Error Distribution", fontsize=13)
    fig.tight_layout()
    _save(fig, out_path, show)


def plot_dashboard(df: pd.DataFrame, pred_pfx: str, real_pfx: str,
                   out_path: str,
                   pos_threshold: float, rot_threshold: float,
                   show: bool = False):
    """
    종합 대시보드 (2행 × 3열):
      [0,0] 위치 오차 time-series   [0,1] 자세 오차 time-series
      [0,2] 위치 오차 히스토그램    [1,0~2] x/y/z 축별 오차
    """
    fig = plt.figure(figsize=(18, 9))
    gs  = gridspec.GridSpec(2, 3, figure=fig, hspace=0.48, wspace=0.32)

    t  = df["time_sec"].values
    pe = df[COL_POS_ERR].values
    re = df[COL_ROT_ERR].values

    ax0 = fig.add_subplot(gs[0, 0])
    ax0.plot(t, pe, color=_COLOR_ERR, linewidth=0.9)
    ax0.axhline(pos_threshold, color=_COLOR_WARN, linestyle="--", linewidth=1.0)
    _shade_regions(ax0, t, pe > pos_threshold, color=_COLOR_WARN, alpha=0.15)
    ax0.set_title("Position Error (mm)"); ax0.set_xlabel("Time (s)")
    ax0.grid(True, alpha=0.3)

    ax1 = fig.add_subplot(gs[0, 1])
    ax1.plot(t, re, color="#009688", linewidth=0.9)
    ax1.axhline(rot_threshold, color=_COLOR_WARN, linestyle="--", linewidth=1.0)
    _shade_regions(ax1, t, re > rot_threshold, color=_COLOR_WARN, alpha=0.15)
    ax1.set_title("Rotation Error (deg)"); ax1.set_xlabel("Time (s)")
    ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(gs[0, 2])
    ax2.hist(pe, bins=30, color=_COLOR_ERR, edgecolor="white", alpha=0.8)
    ax2.axvline(pos_threshold, color=_COLOR_WARN, linestyle="--",
                label=f"thr {pos_threshold}")
    ax2.axvline(pe.mean(), color="black", linestyle="-.",
                label=f"mean {pe.mean():.3f}")
    ax2.set_title("Pos Error Dist."); ax2.set_xlabel("mm")
    ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)

    for col_idx, (col, color, title) in enumerate([
        (COL_ERR_X, "#E91E63", "err_x (mm)"),
        (COL_ERR_Y, "#4CAF50", "err_y (mm)"),
        (COL_ERR_Z, "#2196F3", "err_z (mm)"),
    ]):
        ax = fig.add_subplot(gs[1, col_idx])
        ax.plot(t, df[col].values, color=color, linewidth=0.9)
        ax.axhline(0, color="gray", linewidth=0.6, linestyle=":")
        ax.set_title(title); ax.set_xlabel("Time (s)")
        ax.grid(True, alpha=0.3)

    fig.suptitle("Online Estimation – Error Dashboard", fontsize=14, y=1.01)
    _save(fig, out_path, show)


# ══════════════════════════════════════════════════════════════════════════════
# 메인 분석 함수
# ══════════════════════════════════════════════════════════════════════════════

def run_offline_analysis(
    csv_path:      str,
    out_dir:       str,
    pos_threshold: float = 5.0,
    rot_threshold: float = 3.0,
    show:          bool  = False,
    use_plotly: bool = True,
):
    """
    CSV 로그를 로드하고 전체 trajectory 분석·시각화를 수행한다.

    Parameters
    ----------
    csv_path      : 분석할 CSV 로그 파일 경로
    out_dir       : 그래프 이미지를 저장할 디렉토리
    pos_threshold : 위치 오차 경고 임계값 (mm)
    rot_threshold : 자세 오차 경고 임계값 (deg)
    show          : True 이면 화면에 그래프 표시
    """
    # ── 1. 데이터 로드 ────────────────────────────────────────────────────────
    df = load_log(csv_path)

    # ── 2. 컬럼 prefix 자동 감지 (pred / est 모두 지원) ──────────────────────
    pred_pfx, real_pfx = detect_prefixes(df)

    os.makedirs(out_dir, exist_ok=True)
    def _p(name): return os.path.join(out_dir, name)

    # ── 3. 기술 통계 출력 ─────────────────────────────────────────────────────
    stats = summary_stats(df, pos_threshold, rot_threshold)
    print_summary(stats)

    # ── 4. 고오차 구간 콘솔 리포트 ───────────────────────────────────────────
    print_high_error_report(df, real_pfx, pos_threshold, rot_threshold)

    # ── 5. 그래프 생성·저장 ───────────────────────────────────────────────────
    log.info("종합 분석 차트 생성 중...")

    plot_trajectory_3d(df, pred_pfx, real_pfx, _p("trajectory_3d.png"), show)
    plot_trajectory_2d(df, pred_pfx, real_pfx, _p("trajectory_2d.png"), show)
    plot_position_error(df, _p("position_error.png"), pos_threshold, show)
    plot_rotation_error(df, _p("rotation_error.png"), rot_threshold, show)
    plot_axis_error(df, _p("axis_error.png"), show)
    plot_error_distribution(df, _p("error_distribution.png"),
                            pos_threshold, rot_threshold, show)
    plot_dashboard(df, pred_pfx, real_pfx, _p("dashboard.png"),
                   pos_threshold, rot_threshold, show)
    if use_plotly:
        plot_trajectory_plotly(df, pred_pfx, real_pfx,
                               _p("trajectory_plotly.html"), show)
        
    log.success(f"모든 그래프 저장 완료 → {out_dir}")


# ══════════════════════════════════════════════════════════════════════════════
# CLI
# ══════════════════════════════════════════════════════════════════════════════

def _default_csv() -> str:
    try:
        import json
        with open(PROJECT_ROOT / "config.json", encoding="utf-8") as f:
            cfg = json.load(f)
        return cfg.get("logs", {}).get("online_log", "dataset/logs/online_log.csv")
    except Exception:
        return "dataset/logs/online_log.csv"


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="CSV 로그 기반 오프라인 Trajectory 분석",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument("--csv",
        default=_default_csv(),
        help="분석할 CSV 로그 경로 (루트 기준 상대경로 또는 절대경로)\n"
             "기본: dataset/logs/online_log.csv")
    p.add_argument("--out",
        default="dataset/logs/analysis",
        help="그래프 이미지 저장 디렉토리\n기본: dataset/logs/analysis")
    p.add_argument("--pos-thr", type=float, default=5.0,
        help="위치 오차 강조 임계값 mm (기본: 5.0)")
    p.add_argument("--rot-thr", type=float, default=3.0,
        help="자세 오차 강조 임계값 deg (기본: 3.0)")
    p.add_argument("--show", action="store_true",
        help="그래프를 화면에 표시 (기본: 파일 저장만)")
    p.add_argument("--no-plotly", action="store_true",
               help="Plotly 인터랙티브 궤적 시각화를 건너뜁니다.")

    return p


def main():
    args = _build_parser().parse_args()

    if args.show:
        try:
            matplotlib.use('TkAgg')
        except Exception:
            pass


    csv_path = args.csv if os.path.isabs(args.csv) \
               else str(PROJECT_ROOT / args.csv)
    out_dir  = args.out  if os.path.isabs(args.out)  \
               else str(PROJECT_ROOT / args.out)

    run_offline_analysis(
        csv_path      = csv_path,
        out_dir       = out_dir,
        pos_threshold = args.pos_thr,
        rot_threshold = args.rot_thr,
        show          = args.show,
        use_plotly    = True,
    )


if __name__ == "__main__":
    main()