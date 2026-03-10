import csv
import json
import os
from pathlib import Path

CSV_HEADER = [
    "timestamp", "pose_id",
    "q0", "qx", "qy", "qz", "tx", "ty", "tz", "error",
    "x", "y", "z", "u", "v", "w"
]

def save_data_to_csv(filename, timestamp, pose_id, tool_data, robot_data=None):
    file_exists = os.path.exists(filename)
    with open(filename, mode="a", newline="", encoding='utf-8') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(CSV_HEADER)
        writer.writerow([
            timestamp, pose_id,
            tool_data["q0"], tool_data["qx"], tool_data["qy"], tool_data["qz"],
            tool_data["tx"], tool_data["ty"], tool_data["tz"],
            tool_data["error"],
            robot_data["x"], robot_data["y"], robot_data["z"],
            robot_data["u"], robot_data["v"], robot_data["w"],
        ])


def load_config(path="config.json", base_dir=None):
    if base_dir is None:
        base_dir = Path.cwd()
    else:
        base_dir = Path(base_dir).resolve()

    config_path = base_dir / path

    if not config_path.exists():
        raise FileNotFoundError(f"{config_path} not found.")
    with open(config_path, "r", encoding="utf-8") as f:
        config = json.load(f)

    if config["ndi"]["rom_dir"] is not None:
        config["ndi"]["rom_dir"] = str((base_dir / config["ndi"]["rom_dir"]).resolve())

    config["dataset"]["robot_pose_file"] = str((base_dir / config["dataset"]["robot_pose_file"]).resolve())

    return config


def get_calibration_filepaths(robot_pose_file, dataset_root):
    base = os.path.basename(robot_pose_file)
    name = os.path.splitext(base)[0]
    suffix = name.replace("robot_pose_", "")
    suffix = f"_{suffix}" if suffix else ""  # suffix가 비어있으면 _ 제거

    calib_dir = os.path.join(dataset_root, "calibration")
    results_dir = os.path.join(dataset_root, "results")
    os.makedirs(calib_dir, exist_ok=True)
    os.makedirs(results_dir, exist_ok=True)

    return {
        "csv": os.path.join(calib_dir, f"calibration_data{suffix}.csv"),
        "json": os.path.join(results_dir, f"calibration_result{suffix}.json"),
        "png": os.path.join(results_dir, f"calibration_result{suffix}.png"),
    }
