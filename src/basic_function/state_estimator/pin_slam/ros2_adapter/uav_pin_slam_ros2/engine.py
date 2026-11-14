import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class FrameResult:
    pose: np.ndarray
    tracking_ok: bool
    loop_corrected: bool
    neural_points: Optional[np.ndarray]
    processed_frame: int


class PinSlamEngine:
    """Thin live-data composition of the locked upstream PIN-SLAM classes."""

    def __init__(self, upstream_root: str, config_path: str, output_root: str) -> None:
        root = Path(upstream_root).resolve()
        if not (root / "utils" / "config.py").is_file():
            raise RuntimeError(f"PIN_SLAM_ROOT is not a PIN-SLAM checkout: {root}")
        sys.path.insert(0, str(root))

        import torch
        from dataset.slam_dataset import SLAMDataset
        from model.decoder import Decoder
        from model.neural_points import NeuralPoints
        from utils.config import Config
        from utils.loop_detector import NeuralPointMapContextManager
        from utils.mapper import Mapper
        from utils.mesher import Mesher
        from utils.pgo import PoseGraphManager
        from utils.tracker import Tracker

        self._torch = torch
        self._root = root
        self.config = Config()
        self.config.load(config_path)
        # The upstream CLI initializes dtype through utils.tools.setup_config;
        # the ROS adapter constructs the pipeline directly, so preserve that
        # required initialization before creating NeuralPoints.
        self.config.setup_dtype()
        self.config.run_with_ros = True
        self.config.output_root = output_root
        self.config.run_path = str(Path(output_root).resolve())
        Path(self.config.run_path).mkdir(parents=True, exist_ok=True)
        for child in ("map", "mesh"):
            Path(self.config.run_path, child).mkdir(exist_ok=True)
        if self.config.deskew:
            raise RuntimeError("AirSim PointCloud2 has no per-point time; deskew must be false")
        if self.config.device != "cuda" or not torch.cuda.is_available():
            raise RuntimeError("PIN-SLAM acceptance runtime requires CUDA")

        torch.manual_seed(self.config.seed)
        np.random.seed(self.config.seed)
        self.geo_mlp = Decoder(self.config, self.config.geo_mlp_hidden_dim, self.config.geo_mlp_level, 1)
        self.sem_mlp = (
            Decoder(self.config, self.config.sem_mlp_hidden_dim, self.config.sem_mlp_level,
                    self.config.sem_class_count + 1)
            if self.config.semantic_on else None
        )
        self.color_mlp = (
            Decoder(self.config, self.config.color_mlp_hidden_dim, self.config.color_mlp_level,
                    self.config.color_channel)
            if self.config.color_on else None
        )
        self.mlp_dict = {"sdf": self.geo_mlp, "semantic": self.sem_mlp, "color": self.color_mlp}
        self.neural_points = NeuralPoints(self.config)
        self.dataset = SLAMDataset(self.config)
        self.tracker = Tracker(self.config, self.neural_points, self.mlp_dict)
        self.mapper = Mapper(self.config, self.dataset, self.neural_points, self.mlp_dict)
        self.mesher = Mesher(self.config, self.neural_points, self.mlp_dict)
        self.pgm = PoseGraphManager(self.config)
        if self.config.pgo_on:
            self.pgm.add_pose_prior(0, np.eye(4), fixed=True)
        self.lcd_npmc = NeuralPointMapContextManager(self.config)
        self.loop_reg_failed_count = 0

    def process(
        self, cloud_message, map_max_points: int, publish_map: bool = True
    ) -> FrameResult:
        from utils.tools import freeze_decoders

        frame_id = self.dataset.processed_frame
        self.dataset.read_frame_ros(cloud_message)
        valid_frame = self.dataset.preprocess_frame()
        if not valid_frame:
            self.dataset.processed_frame += 1
            return FrameResult(np.array(self.dataset.cur_pose_ref), False, False, None, frame_id)

        tracking_ok = True
        if frame_id > 0:
            pose, _, _, tracking_ok = self.tracker.tracking(
                self.dataset.cur_source_points,
                self.dataset.cur_pose_guess_torch,
                self.dataset.cur_source_colors,
                self.dataset.cur_source_normals,
            )
            self.dataset.lose_track = not tracking_ok
            self.dataset.update_odom_pose(pose)

        travel_dist = self.dataset.travel_dist[:frame_id + 1]
        self.neural_points.travel_dist = self._torch.tensor(
            travel_dist, device=self.config.device, dtype=self.config.dtype)

        loop_corrected = self._detect_correct_loop(travel_dist) if self.config.pgo_on else False
        if tracking_ok and not self.dataset.stop_status:
            self.mapper.process_frame(
                self.dataset.cur_point_cloud_torch,
                self.dataset.cur_sem_labels_torch,
                self.dataset.cur_pose_torch,
                frame_id,
            )
        else:
            self.neural_points.reset_local_map(
                self.dataset.cur_pose_torch[:3, 3], None, frame_id)

        if frame_id == self.config.freeze_after_frame:
            freeze_decoders(self.mlp_dict, self.config)
            self.config.decoder_freezed = True
            self.neural_points.compute_feature_principle_components(down_rate=17)
        if self.config.ba_freq_frame > 0 and (frame_id + 1) % self.config.ba_freq_frame == 0:
            self.mapper.bundle_adjustment(self.config.ba_iters, self.config.ba_frame)
        if frame_id % self.config.mapping_freq_frame == 0:
            iterations = self.config.iters * self.config.init_iter_ratio if frame_id == 0 else self.config.iters
            if self.config.adaptive_iters and self.dataset.stop_status:
                iterations = max(1, iterations - 10)
            self.mapper.mapping(iterations)

        points = None
        if publish_map and tracking_ok and self.neural_points.count() > 0:
            count = self.neural_points.count()
            stride = max(1, int(np.ceil(count / max(1, map_max_points))))
            points = self.neural_points.neural_points[::stride].detach().cpu().numpy().astype(np.float32)
        pose = np.array(self.dataset.cur_pose_ref, dtype=np.float64)
        self.dataset.processed_frame += 1
        return FrameResult(pose, bool(tracking_ok), loop_corrected, points, frame_id)

    def _detect_correct_loop(self, travel_dist: np.ndarray) -> bool:
        from utils.loop_detector import detect_local_loop
        from utils.tools import transform_torch

        torch = self._torch
        frame_id = self.dataset.processed_frame
        valid_mapping = not self.dataset.lose_track and not self.dataset.stop_status
        if self.config.global_loop_on:
            if self.config.local_map_context and frame_id >= self.config.local_map_context_latency:
                local_id = frame_id - self.config.local_map_context_latency
                local_pose = torch.tensor(
                    self.dataset.pgo_poses[local_id], device=self.config.device, dtype=torch.float64)
                if self.config.local_map_context_latency > 0:
                    self.neural_points.reset_local_map(
                        local_pose[:3, 3], None, local_id,
                        self.config.loop_local_map_by_travel_dist,
                        self.config.loop_local_map_time_window)
                context = transform_torch(
                    self.neural_points.local_neural_points.detach(), torch.linalg.inv(local_pose))
                feature = (
                    self.neural_points.local_geo_features[:-1].detach()
                    if self.config.loop_with_feature else None)
                self.lcd_npmc.add_node(local_id, context, feature, valid_flag=valid_mapping)
            else:
                self.lcd_npmc.add_node(
                    frame_id, self.dataset.cur_point_cloud_torch, valid_flag=valid_mapping)

        self.pgm.add_frame_node(frame_id, self.dataset.pgo_poses[frame_id])
        self.pgm.init_poses = self.dataset.pgo_poses[:frame_id + 1]
        if frame_id == 0:
            return False
        self.pgm.add_odometry_factor(
            frame_id, frame_id - 1, self.dataset.last_odom_tran)
        self.pgm.estimate_drift(travel_dist, frame_id, correct_ratio=0.01)
        if self.config.pgo_with_pose_prior:
            self.pgm.add_pose_prior(frame_id, self.dataset.pgo_poses[frame_id])
        if frame_id - self.pgm.last_loop_idx <= self.config.pgo_freq or self.dataset.stop_status:
            return False

        candidate_mask = (
            travel_dist[-1] - travel_dist
            > self.config.min_loop_travel_dist_ratio * self.config.local_map_radius)
        if not np.any(candidate_mask):
            return False
        loop_id, _, loop_transform = detect_local_loop(
            self.dataset.pgo_poses[:frame_id + 1], candidate_mask,
            self.pgm.drift_radius, frame_id, self.loop_reg_failed_count,
            self.config.local_loop_dist_thre, self.config.local_loop_dist_thre * 3.0,
            self.config.silence)
        local_context_loop = False
        if loop_id is None and self.config.global_loop_on:
            loop_id, _, loop_transform, local_context_loop = self.lcd_npmc.detect_global_loop(
                self.dataset.pgo_poses[:frame_id + 1],
                self.pgm.drift_radius * self.config.loop_dist_drift_ratio_thre,
                candidate_mask, self.neural_points)
        if loop_id is None or not self.lcd_npmc.valid_flags[loop_id]:
            return False
        if self.config.loop_z_check_on and abs(loop_transform[2, 3]) > self.config.voxel_size_m * 4.0:
            return False

        initial = torch.tensor(
            self.dataset.pgo_poses[loop_id] @ loop_transform,
            device=self.config.device, dtype=torch.float64)
        self.neural_points.recreate_hash(initial[:3, 3], None, True, True, loop_id)
        refined, covariance, _, valid = self.tracker.tracking(
            self.dataset.cur_source_points.clone(), initial, loop_reg=True)
        if valid:
            loop_transform = np.linalg.inv(self.dataset.pgo_poses[loop_id]) @ refined.detach().cpu().numpy()
            valid = self.pgm.add_loop_factor(
                frame_id, loop_id, loop_transform,
                cov=covariance if self.config.use_reg_cov_mat else None)
        if not valid:
            self.neural_points.recreate_hash(
                self.dataset.cur_pose_torch[:3, 3], None, True, True, frame_id)
            self.loop_reg_failed_count += 1
            return False

        self.pgm.optimize_pose_graph()
        visual_id = frame_id - self.config.local_map_context_latency if local_context_loop else frame_id
        self.pgm.loop_edges_vis.append(np.array([loop_id, visual_id], dtype=np.uint32))
        self.pgm.loop_edges.append(np.array([loop_id, frame_id], dtype=np.uint32))
        self.pgm.loop_trans.append(loop_transform)
        pose_difference = torch.tensor(
            self.pgm.get_pose_diff(), device=self.config.device, dtype=self.config.dtype)
        self.dataset.cur_pose_torch = torch.tensor(
            self.pgm.cur_pose, device=self.config.device, dtype=self.config.dtype)
        self.dataset.cur_pose_ref = self.pgm.cur_pose.copy()
        self.neural_points.adjust_map(pose_difference)
        self.neural_points.recreate_hash(
            self.dataset.cur_pose_torch[:3, 3], None,
            not self.config.pgo_merge_map, self.config.rehash_with_time, frame_id)
        self.mapper.transform_data_pool(pose_difference)
        self.dataset.update_poses_after_pgo(self.pgm.pgo_poses)
        self.pgm.last_loop_idx = frame_id
        self.pgm.min_loop_idx = min(self.pgm.min_loop_idx, loop_id)
        self.loop_reg_failed_count = 0
        return True

    def save_results(self) -> None:
        from utils.tools import save_implicit_map

        import open3d as o3d

        self.dataset.write_results()
        point_cloud = self.neural_points.get_neural_points_o3d(query_global=True, color_mode=0)
        o3d.io.write_point_cloud(str(Path(self.config.run_path, "map", "neural_points.ply")), point_cloud)
        save_implicit_map(self.config.run_path, self.neural_points, self.mlp_dict)

    def save_mesh(self) -> None:
        from utils.tools import split_chunks

        cloud = self.neural_points.get_neural_points_o3d(query_global=True, random_down_ratio=23)
        if not cloud.has_points():
            raise RuntimeError("neural map is empty")
        bounds = cloud.get_axis_aligned_bounding_box()
        chunks = split_chunks(cloud, bounds, self.config.mc_res_m * 300)
        path = str(Path(self.config.run_path, "mesh", f"mesh_{self.dataset.processed_frame}.ply"))
        self.mesher.recon_aabb_collections_mesh(
            chunks, self.config.mc_res_m, path, False,
            self.config.semantic_on, self.config.color_on,
            filter_isolated_mesh=True, mesh_min_nn=self.config.mesh_min_nn)
