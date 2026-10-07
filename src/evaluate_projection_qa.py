"""LiDAR-Camera Projection QA & Calibration Drift Evaluation Tool.

Module thực hiện:
1. Đo độ nhạy của phép chiếu LiDAR-Camera khi Extrinsic bị trôi dạt (Yaw, Pitch, Translation).
2. Tính toán Point-in-Box Retention Ratio cho vật thể gần (<20m) và xa (>=20m).
3. Đo Latency chuẩn p50/p95 (bỏ lần đầu, lặp lại >= 20 lần).
4. So sánh trên cả 2 dataset (KITTI 64-beam vs nuScenes 32-beam).
5. Xuất báo cáo CSV, biểu đồ phân tích và ảnh minh hoạ failure cases.
"""
from __future__ import annotations

import argparse
import copy
import platform
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cv2
import matplotlib.pyplot as plt
import numpy as np

from starter.datasets import dataset_type, load_frame
from starter.kitti_io import KittiCalib, KittiObject
from starter.projection import perturb_extrinsic, project_velo_to_image, overlay_points, draw_box2d


def compute_point_in_box_retention(
    frame: dict,
    perturbed_calib: KittiCalib,
) -> dict[str, float]:
    """Tính toán tỷ lệ điểm giữ lại trong bounding box 2D khi calibration bị lệch."""
    pts = frame["points"]
    img_shape = frame["image"].shape
    orig_calib = frame["calib"]
    labels: list[KittiObject] = frame["labels"]

    # 1. Chiếu với calib gốc (chuẩn)
    uv_orig, depth_orig, mask_orig = project_velo_to_image(pts, orig_calib, img_shape)
    
    # 2. Chiếu với calib bị perturb
    uv_pert, depth_pert, mask_pert = project_velo_to_image(pts, perturbed_calib, img_shape)

    # Lấy chỉ số gốc của các điểm chiếu thành công
    valid_orig_idx = np.where(mask_orig)[0]
    valid_pert_idx = np.where(mask_pert)[0]
    pert_idx_to_pos = {idx: pos for pos, idx in enumerate(valid_pert_idx)}

    total_orig_pts_in_box = 0
    total_retained_in_box = 0

    near_orig_pts = 0
    near_retained = 0

    far_orig_pts = 0
    far_retained = 0

    for obj in labels:
        if obj.type.lower() in ["dontcare"]:
            continue
        x1, y1, x2, y2 = obj.bbox
        obj_dist = float(obj.location[2]) if obj.location is not None else 10.0

        # Tìm các điểm chiếu gốc nằm trong box
        in_box_mask = (
            (uv_orig[:, 0] >= x1) & (uv_orig[:, 0] <= x2) &
            (uv_orig[:, 1] >= y1) & (uv_orig[:, 1] <= y2)
        )
        box_pts_indices = valid_orig_idx[in_box_mask]
        n_in_box = len(box_pts_indices)
        if n_in_box == 0:
            continue

        total_orig_pts_in_box += n_in_box
        is_near = obj_dist < 20.0
        if is_near:
            near_orig_pts += n_in_box
        else:
            far_orig_pts += n_in_box

        # Kiểm tra điểm sau khi perturb
        retained_count = 0
        for pt_idx in box_pts_indices:
            if pt_idx in pert_idx_to_pos:
                u_p, v_p = uv_pert[pert_idx_to_pos[pt_idx]]
                if x1 <= u_p <= x2 and y1 <= v_p <= y2:
                    retained_count += 1

        total_retained_in_box += retained_count
        if is_near:
            near_retained += retained_count
        else:
            far_retained += retained_count

    retention_overall = (total_retained_in_box / total_orig_pts_in_box * 100.0) if total_orig_pts_in_box > 0 else 100.0
    retention_near = (near_retained / near_orig_pts * 100.0) if near_orig_pts > 0 else 100.0
    retention_far = (far_retained / far_orig_pts * 100.0) if far_orig_pts > 0 else 100.0
    fov_ratio = float(mask_pert.mean() * 100.0)

    return {
        "fov_ratio": fov_ratio,
        "retention_overall": retention_overall,
        "retention_near": retention_near,
        "retention_far": retention_far,
        "pts_in_box_orig": total_orig_pts_in_box,
    }


def benchmark_latency(frame: dict, iterations: int = 30) -> dict[str, float]:
    """Đo thời gian chạy (latency) p50/p95 của phép chiếu (bỏ lần đầu)."""
    pts = frame["points"]
    calib = frame["calib"]
    shape = frame["image"].shape

    # Lần chạy warmup
    _ = project_velo_to_image(pts, calib, shape)

    times_ms = []
    for _ in range(iterations):
        t0 = time.perf_counter()
        _ = project_velo_to_image(pts, calib, shape)
        times_ms.append((time.perf_counter() - t0) * 1000.0)

    times_ms = np.array(times_ms)
    p50 = float(np.percentile(times_ms, 50))
    p95 = float(np.percentile(times_ms, 95))
    mean = float(np.mean(times_ms))
    return {"p50_ms": p50, "p95_ms": p95, "mean_ms": mean}


def run_experiment(
    data_root: str,
    frames: list[str],
    yaw_sweep: list[float],
    ty_sweep: list[float],
) -> list[dict]:
    """Chạy toàn bộ thí nghiệm quét yaw drift và translation drift."""
    results = []
    is_nuscenes = (dataset_type(data_root) == "nuscenes")

    for f_id in frames:
        kwargs = {"use_ego_motion": True} if is_nuscenes else {}
        fr = load_frame(data_root, f_id, **kwargs)
        lat = benchmark_latency(fr)

        # 1. Quét Yaw Drift
        for yaw in yaw_sweep:
            calib_p = perturb_extrinsic(fr["calib"], yaw_deg=yaw)
            metrics = compute_point_in_box_retention(fr, calib_p)
            results.append({
                "dataset": "nuscenes" if is_nuscenes else "kitti",
                "frame": f_id,
                "perturb_type": "yaw_drift",
                "perturb_value": yaw,
                "unit": "deg",
                "fov_ratio_pct": round(metrics["fov_ratio"], 2),
                "retention_overall_pct": round(metrics["retention_overall"], 2),
                "retention_near_pct": round(metrics["retention_near"], 2),
                "retention_far_pct": round(metrics["retention_far"], 2),
                "latency_p50_ms": round(lat["p50_ms"], 2),
                "latency_p95_ms": round(lat["p95_ms"], 2),
            })

        # 2. Quét Translation Ty Drift
        for ty in ty_sweep:
            calib_p = perturb_extrinsic(fr["calib"], t_xyz_m=(0.0, ty, 0.0))
            metrics = compute_point_in_box_retention(fr, calib_p)
            results.append({
                "dataset": "nuscenes" if is_nuscenes else "kitti",
                "frame": f_id,
                "perturb_type": "ty_drift",
                "perturb_value": ty,
                "unit": "meter",
                "fov_ratio_pct": round(metrics["fov_ratio"], 2),
                "retention_overall_pct": round(metrics["retention_overall"], 2),
                "retention_near_pct": round(metrics["retention_near"], 2),
                "retention_far_pct": round(metrics["retention_far"], 2),
                "latency_p50_ms": round(lat["p50_ms"], 2),
                "latency_p95_ms": round(lat["p95_ms"], 2),
            })

    return results


def save_csv(results: list[dict], out_path: Path) -> None:
    """Lưu kết quả ra file CSV."""
    import csv
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not results:
        return
    keys = list(results[0].keys())
    with open(out_path, mode="w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(results)
    print(f"[OK] Đã lưu bảng số liệu vào {out_path}")


def generate_plots(results: list[dict], out_path: Path) -> None:
    """Vẽ 3 biểu đồ phân tích độ nhạy drift và so sánh kết quả."""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 3, figsize=(18, 5), dpi=150)

    # Lọc kết quả KITTI
    kitti_yaw = [r for r in results if r["dataset"] == "kitti" and r["perturb_type"] == "yaw_drift"]
    kitti_ty = [r for r in results if r["dataset"] == "kitti" and r["perturb_type"] == "ty_drift"]

    # 1. Subplot 1: Yaw Drift vs Retention (Near vs Far vs Overall)
    if kitti_yaw:
        yaw_vals = sorted(list(set(r["perturb_value"] for r in kitti_yaw)))
        near_means = [np.mean([r["retention_near_pct"] for r in kitti_yaw if r["perturb_value"] == y]) for y in yaw_vals]
        far_means = [np.mean([r["retention_far_pct"] for r in kitti_yaw if r["perturb_value"] == y]) for y in yaw_vals]
        all_means = [np.mean([r["retention_overall_pct"] for r in kitti_yaw if r["perturb_value"] == y]) for y in yaw_vals]

        axes[0].plot(yaw_vals, near_means, marker="o", color="#2ca02c", linewidth=2.5, label="Near (<20m)")
        axes[0].plot(yaw_vals, all_means, marker="s", color="#1f77b4", linewidth=2.5, label="Overall")
        axes[0].plot(yaw_vals, far_means, marker="^", color="#d62728", linewidth=2.5, label="Far (>=20m)")
        axes[0].axhline(y=50, color="gray", linestyle="--", alpha=0.7, label="50% Threshold")
        axes[0].set_title("LiDAR Point Retention vs Yaw Drift (KITTI)", fontsize=12, fontweight="bold")
        axes[0].set_xlabel("Yaw Misalignment (degrees)", fontsize=11)
        axes[0].set_ylabel("Point-in-Box Retention (%)", fontsize=11)
        axes[0].grid(True, linestyle=":", alpha=0.6)
        axes[0].legend(fontsize=10)
        axes[0].set_ylim(-5, 105)

    # 2. Subplot 2: Translation Ty Drift vs Retention
    if kitti_ty:
        ty_vals = sorted(list(set(r["perturb_value"] for r in kitti_ty)))
        near_means = [np.mean([r["retention_near_pct"] for r in kitti_ty if r["perturb_value"] == t]) for t in ty_vals]
        far_means = [np.mean([r["retention_far_pct"] for r in kitti_ty if r["perturb_value"] == t]) for t in ty_vals]
        all_means = [np.mean([r["retention_overall_pct"] for r in kitti_ty if r["perturb_value"] == t]) for t in ty_vals]

        axes[1].plot(ty_vals, near_means, marker="o", color="#2ca02c", linewidth=2.5, label="Near (<20m)")
        axes[1].plot(ty_vals, all_means, marker="s", color="#1f77b4", linewidth=2.5, label="Overall")
        axes[1].plot(ty_vals, far_means, marker="^", color="#d62728", linewidth=2.5, label="Far (>=20m)")
        axes[1].axhline(y=50, color="gray", linestyle="--", alpha=0.7, label="50% Threshold")
        axes[1].set_title("LiDAR Point Retention vs Translation Ty Drift", fontsize=12, fontweight="bold")
        axes[1].set_xlabel("Translation Ty Drift (meters)", fontsize=11)
        axes[1].set_ylabel("Point-in-Box Retention (%)", fontsize=11)
        axes[1].grid(True, linestyle=":", alpha=0.6)
        axes[1].legend(fontsize=10)
        axes[1].set_ylim(-5, 105)

    # 3. Subplot 3: Dataset Comparison (KITTI 64-beam vs nuScenes 32-beam)
    nusc_yaw = [r for r in results if r["dataset"] == "nuscenes" and r["perturb_type"] == "yaw_drift"]
    if kitti_yaw and nusc_yaw:
        common_yaws = sorted(list(set(r["perturb_value"] for r in kitti_yaw) & set(r["perturb_value"] for r in nusc_yaw)))
        kitti_means = [np.mean([r["retention_overall_pct"] for r in kitti_yaw if r["perturb_value"] == y]) for y in common_yaws]
        nusc_means = [np.mean([r["retention_overall_pct"] for r in nusc_yaw if r["perturb_value"] == y]) for y in common_yaws]

        axes[2].bar([y - 0.08 for y in common_yaws], kitti_means, width=0.16, color="#1f77b4", label="KITTI (64-beam)")
        axes[2].bar([y + 0.08 for y in common_yaws], nusc_means, width=0.16, color="#ff7f0e", label="nuScenes (32-beam)")
        axes[2].set_title("Sensor Comparison: Retention vs Yaw Drift", fontsize=12, fontweight="bold")
        axes[2].set_xlabel("Yaw Misalignment (degrees)", fontsize=11)
        axes[2].set_ylabel("Overall Point Retention (%)", fontsize=11)
        axes[2].set_xticks(common_yaws)
        axes[2].grid(True, linestyle=":", alpha=0.6)
        axes[2].legend(fontsize=10)
        axes[2].set_ylim(0, 110)

    plt.tight_layout()
    plt.savefig(str(out_path), bbox_inches="tight")
    plt.close()
    print(f"[OK] Đã lưu biểu đồ phân tích vào {out_path}")


def create_failure_visualizations(kitti_root: str, out_dir: Path) -> None:
    """Tạo các ảnh minh hoạ Failure Case khi calibration drift."""
    out_dir.mkdir(parents=True, exist_ok=True)
    fr = load_frame(kitti_root, "000011")

    # 1. Failure Case 1: Yaw drift 2.0° khiến điểm của xe ở xa rơi hoàn toàn ra ngoài box
    calib_fail1 = perturb_extrinsic(fr["calib"], yaw_deg=2.0)
    uv1, depth1, _ = project_velo_to_image(fr["points"], calib_fail1, fr["image"].shape)
    vis1 = overlay_points(fr["image"], uv1, depth1, radius=2)
    for obj in fr["labels"]:
        vis1 = draw_box2d(vis1, obj.bbox, label=f"{obj.type} z={obj.location[2]:.1f}m", color=(0, 0, 255))
    cv2.putText(vis1, "FAILURE CASE: Yaw Drift 2.0 deg -> Far Objects Misaligned", (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    fail_path1 = out_dir / "fail_01_yaw_drift_far_objects.png"
    cv2.imwrite(str(fail_path1), vis1)
    print(f"[OK] Đã tạo failure visual 1: {fail_path1}")

    # 2. Failure Case 2: Yaw drift 1.5° kết hợp cự ly người đi bộ
    calib_fail2 = perturb_extrinsic(fr["calib"], yaw_deg=1.5)
    uv2, depth2, _ = project_velo_to_image(fr["points"], calib_fail2, fr["image"].shape)
    vis2 = overlay_points(fr["image"], uv2, depth2, radius=2)
    for obj in fr["labels"]:
        if obj.type.lower() in ["pedestrian", "cyclist"]:
            vis2 = draw_box2d(vis2, obj.bbox, label=f"{obj.type} (Dislocated Points)", color=(0, 255, 255))
    cv2.putText(vis2, "FAILURE CASE: Pedestrian Point Cloud Dislocation at 1.5 deg Yaw", (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    fail_path2 = out_dir / "fail_02_yaw_drift_pedestrian.png"
    cv2.imwrite(str(fail_path2), vis2)
    print(f"[OK] Đã tạo failure visual 2: {fail_path2}")


def main() -> None:
    if sys.stdout and hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if sys.stderr and hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")

    parser = argparse.ArgumentParser(description="LiDAR-Camera Projection QA Benchmark CLI")
    parser.add_argument("--kitti-root", default="data/kitti_mini", help="Thư mục KITTI")
    parser.add_argument("--nusc-root", default="data/nuscenes_mini_subset", help="Thư mục nuScenes")
    parser.add_argument("--kitti-frames", nargs="+", default=["000011", "000021", "000049"], help="Các frame KITTI")
    parser.add_argument("--nusc-frames", nargs="+", default=["scene-0103_010"], help="Các frame nuScenes")
    parser.add_argument("--out-csv", default="results/calibration_drift_sweep.csv", help="Đường dẫn file CSV")
    parser.add_argument("--out-fig", default="results/figures/drift_analysis.png", help="Đường dẫn file biểu đồ")
    parser.add_argument("--out-figures-dir", default="results/figures", help="Thư mục lưu ảnh demo/failure")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    args = parser.parse_args()

    np.random.seed(args.seed)

    yaw_sweep = [0.0, 0.5, 1.0, 1.5, 2.0, 3.0]
    ty_sweep = [0.0, 0.05, 0.10, 0.15, 0.20, 0.30]

    print(f"=== Bắt đầu chạy Benchmark LiDAR-Camera Projection QA ===")
    print(f"Hệ điều hành: {platform.system()} {platform.release()}, CPU: {platform.processor()}")

    all_results = []
    # 1. Chạy trên KITTI
    print(f"\n[1/3] Đang xử lý tập dữ liệu KITTI ({len(args.kitti_frames)} frames)...")
    kitti_res = run_experiment(args.kitti_root, args.kitti_frames, yaw_sweep, ty_sweep)
    all_results.extend(kitti_res)

    # 2. Chạy trên nuScenes
    print(f"\n[2/3] Đang xử lý tập dữ liệu nuScenes ({len(args.nusc_frames)} frames)...")
    nusc_res = run_experiment(args.nusc_root, args.nusc_frames, yaw_sweep, ty_sweep)
    all_results.extend(nusc_res)

    # 3. Lưu kết quả CSV & Biểu đồ
    print(f"\n[3/3] Xuất báo cáo, biểu đồ và failure cases...")
    save_csv(all_results, Path(args.out_csv))
    generate_plots(all_results, Path(args.out_fig))
    create_failure_visualizations(args.kitti_root, Path(args.out_figures_dir))

    print("\n=== Hoàn thành toàn bộ Benchmark CP3 & CP4 thành công! ===")


if __name__ == "__main__":
    main()
