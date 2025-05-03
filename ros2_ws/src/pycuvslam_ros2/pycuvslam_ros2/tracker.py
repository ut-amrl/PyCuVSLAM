import os
import time
import bisect
from pathlib import Path

import yaml
import numpy as np
from loguru import logger
from scipy.spatial.transform import Rotation as R

import cuvslam

_DISTORTION_MODEL_MAP = {m.name.lower(): m for m in cuvslam.DistortionModel}

CAM2ENU = np.array(
    [
        [0, 0, 1, 0],
        [-1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1],
    ]
)
ENU2CAM = np.linalg.inv(CAM2ENU)


class VisualWheelOdometry:

    def __init__(self, config_path):
        self.tracker = PyCuVSLAM_MonoTracker(config_path)
        self.wheel_odometry = WheelOdometry(config_path)

        self.last_vo_SE3 = np.eye(4)
        self.last_ts = 0
        self.trajectory = []

    def update_wheel(self, ts, rpm_left, rpm_right):
        self.wheel_odometry.update(ts, rpm_left, rpm_right)

    def update_frame(self, ts, image):
        pose = self.tracker.track(ts, image)
        if pose is None:
            return self.trajectory[-1] if len(self.trajectory) > 0 else None

        # get relative pose (T^{t-1}_{t})
        T_curr = np.eye(4)
        T_curr[:3, :3] = R.from_quat(pose[3:]).as_matrix()
        T_curr[:3, 3] = pose[:3]
        T_curr = CAM2ENU @ T_curr @ ENU2CAM
        logger.debug(T_curr[:3, 3])

        if len(self.trajectory) == 0:
            self.trajectory.append(np.zeros(3))
            self.last_vo_SE3 = T_curr
            self.last_ts = ts
            return self.trajectory[0]

        T_prev = self.last_vo_SE3
        T_rel = np.linalg.inv(T_prev) @ T_curr
        vo_dist = np.linalg.norm(T_rel[:3, 3])
        logger.debug(f"t: {ts} - VO distance: {vo_dist}")

        # get wheel-odom distance
        wo_delta = self.wheel_odometry.get_rel_pose(self.last_ts, ts)
        logger.debug(f"t: {ts} - Wheel odometry delta: {wo_delta}")

        # get scale factor
        if vo_dist > 1e-6:
            s = np.linalg.norm(wo_delta[:2]) / vo_dist
        else:
            s = 0.0
        logger.debug(f"t: {ts} - Scale factor: {s}")

        # action (SE2)
        dx, dy = T_rel[:2, 3] * s
        dy = 0  # NOTE: this is a hack to ignore y motion
        dr = np.arctan2(T_rel[1, 0], T_rel[0, 0])

        # contradictory motion
        if dx * wo_delta[0] < 0:
            dx, dy = 0, 0

        # update pose (SE2)
        x, y, angle = self.trajectory[-1]
        c_angle, s_angle = np.cos(angle), np.sin(angle)
        x_new = x + dx * c_angle - dy * s_angle
        y_new = y + dx * s_angle + dy * c_angle
        angle_new = (angle + dr + np.pi) % (2 * np.pi) - np.pi

        new_pose = np.array([x_new, y_new, angle_new])
        self.trajectory.append(new_pose)

        # update history
        self.last_vo_SE3 = T_curr
        self.last_ts = ts
        return new_pose


class PyCuVSLAM_MonoTracker:
    def __init__(self, config_path):
        # configure tracker
        cfg = cuvslam.TrackerConfig()
        cfg.odometry_mode = cuvslam.TrackerOdometryMode.Mono
        cfg.async_sba = False
        cfg.enable_observations_export = True

        # get camera rig
        cam = self.get_camera(config_path)
        rig = cuvslam.Rig([cam])

        # initialize tracker
        self.pycuvslam_tracker = cuvslam.Tracker(rig, cfg)

        self.frame_id = 0
        self.trajectory = []

    def get_camera(self, config_path):
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file path does not exist: {config_path}")

        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
            logger.info(f"Config: {config}")

        cam = cuvslam.Camera()
        cam.distortion = cuvslam.Distortion(
            _DISTORTION_MODEL_MAP[config["distortion_model"].lower()],
            config["distortion_coefficients"],
        )
        cam.focal = [config["fx"], config["fy"]]
        cam.principal = [config["cx"], config["cy"]]
        cam.size = config["image_size"]
        cam.rig_from_camera = cuvslam.Pose(rotation=[0, 0, 0, 1], translation=[0, 0, 0])
        return cam

    def track(self, ts, image):
        result = self.pycuvslam_tracker.track(int(ts * 1e9), [image])

        pose = None
        if result.is_valid:
            pose = [*result.pose.translation, *result.pose.rotation]
        else:
            logger.warning(f"Frame {self.frame_id} - Pose: Invalid")

        self.trajectory.append(pose)
        self.frame_id += 1
        return pose


class EMA:
    def __init__(self, alpha):
        self.alpha = alpha
        self.x = None

    def __call__(self, z):
        self.x = z if self.x is None else self.alpha * z + (1 - self.alpha) * self.x
        return self.x


def to_SE2(x, y, th):
    return np.array(
        [
            [np.cos(th), -np.sin(th), x],
            [np.sin(th), np.cos(th), y],
            [0, 0, 1],
        ]
    )


class WheelOdometry:
    def __init__(self, config_path):
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        self.R = config["wheel_radius"]
        self.L = config["wheel_distance"]

        self.timestamps = []
        self.poses = []

    def update(self, ts, rpm_left, rpm_right):
        logger.debug(f"t: {ts} - Wheel RPM: {rpm_left}, {rpm_right}")
        if len(self.timestamps) == 0:
            self.timestamps.append(ts)
            self.poses.append(np.zeros(3))
            return

        dt = ts - self.timestamps[-1]
        logger.debug(f"t: {ts} - dt: {dt}")
        if dt <= 0:
            logger.warning("Non-positive time step, skipping update.")
            return

        # Convert RPM to radians per second
        w_left = rpm_left * (2 * np.pi / 60)
        w_right = rpm_right * (2 * np.pi / 60)

        # Calculate the linear and angular velocities
        v = 0.5 * self.R * (w_left + w_right)
        w = self.R * (w_right - w_left) / self.L

        # Update the pose
        x, y, theta = self.poses[-1]
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += w * dt
        theta = (theta + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]

        self.timestamps.append(ts)
        self.poses.append(np.array([x, y, theta]))
        logger.debug(f"Pose: {self.poses[-1]}")

    def _interp_pose(self, t):
        if t < self.timestamps[0] or t > self.timestamps[-1]:
            logger.warning(f"Timestamp {t} out of range")
            return None

        # Find index i such that timestamps[i] <= t < timestamps[i+1]
        idx = bisect.bisect_right(self.timestamps, t) - 1
        t0, t1 = self.timestamps[idx], self.timestamps[idx + 1]
        p0, p1 = self.poses[idx], self.poses[idx + 1]
        alpha = (t - t0) / (t1 - t0)
        # linear interp in XY and spherical‐linear (approx) for theta
        xy = (1 - alpha) * p0[:2] + alpha * p1[:2]
        # for heading, interp on unit‐circle
        c = (1 - alpha) * np.cos(p0[2]) + alpha * np.cos(p1[2])
        s = (1 - alpha) * np.sin(p0[2]) + alpha * np.sin(p1[2])
        theta = np.arctan2(s, c)
        return np.array([xy[0], xy[1], theta])

    def get_rel_pose(self, t0, t1):
        if t1 < t0:
            logger.error("t1 must be greater than t0")
            return np.zeros(3)

        if len(self.timestamps) < 2:
            logger.warning("Not enough data to compute scale")
            return np.zeros(3)

        p0 = self._interp_pose(t0)
        p1 = self._interp_pose(t1)
        if p0 is None or p1 is None:
            return np.zeros(3)

        T0 = to_SE2(*p0)
        T1 = to_SE2(*p1)
        T10 = np.linalg.inv(T0) @ T1
        dx = T10[0, 2]
        dy = T10[1, 2]
        dr = np.arctan2(T10[1, 0], T10[0, 0])
        return np.array([dx, dy, dr])


if __name__ == "__main__":
    import os
    import time
    import argparse

    import cv2
    import rerun as rr
    import rerun.blueprint as rrb

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ride_path",
        type=str,
        default="frodobots8k/output_rides_30/ride_73849_c8033c_20240801164812",
    )
    parser.add_argument(
        "--config_path", type=str, default="ros2_ws/src/pycuvslam_ros2/config/zero.yaml"
    )
    args = parser.parse_args()

    ride_path = Path(args.ride_path)

    # load recording
    cam_path = ride_path / "front_camera.mp4"
    cam_ts_path = str(next(ride_path.glob("front_camera_timestamps_*.csv"), None))
    cam_timestamps = np.genfromtxt(cam_ts_path, delimiter=",", skip_header=1)[:, 1]

    cap = cv2.VideoCapture(cam_path)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open recording {cam_path}")

    # control data (start with "control_data_*"
    ctrl_path = str(next(ride_path.glob("control_data_*.csv"), None))
    ctrl_path = np.genfromtxt(ctrl_path, delimiter=",", skip_header=1)
    ctrl_timestamps = ctrl_path[:, 6]
    ctrl_rpm_left = (ctrl_path[:, 2] + ctrl_path[:, 4]) / 2
    ctrl_rpm_right = (ctrl_path[:, 3] + ctrl_path[:, 5]) / 2

    def color_from_id(identifier):
        return [
            (identifier * 17) % 256,
            (identifier * 31) % 256,
            (identifier * 47) % 256,
        ]

    # # Initialize Rerun
    # rr.init("Frodo Tracker", spawn=True)
    # rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)
    # rr.send_blueprint(
    #     rrb.Blueprint(
    #         rrb.TimePanel(state="collapsed"),
    #         rrb.Horizontal(
    #             column_shares=[0.5, 0.5],
    #             contents=[
    #                 rrb.Spatial2DView(origin="world/camera_0"),
    #                 rrb.Spatial3DView(origin="world"),
    #             ],
    #         ),
    #     )
    # )

    # # Initialize Tracker
    # vo_tracker = PyCuVSLAM_MonoTracker(args.config_path)

    # for ts in cam_timestamps:
    #     ret, frame = cap.read()
    #     if not ret:
    #         break

    #     # Convert frame to RGB
    #     frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #     image = frame.copy()

    #     # Track frame
    #     pose = vo_tracker.track(ts, image)
    #     if pose is None:
    #         continue

    #     time.sleep(0.1)

    #     # Log pose to Rerun
    #     rr.set_time_sequence("frame", vo_tracker.frame_id)
    #     rr.log(
    #         "world/trajectory",
    #         rr.LineStrips3D([pose[:3] for pose in vo_tracker.trajectory]),
    #         static=True,
    #     )
    #     rr.log(
    #         "world/camera_0",
    #         rr.Transform3D(translation=pose[:3], quaternion=pose[3:]),
    #         rr.Arrows3D(
    #             vectors=np.eye(3) * 0.2,
    #             colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],  # RGB for XYZ axes
    #         ),
    #     )
    #     current_observations_main_cam = (
    #         vo_tracker.pycuvslam_tracker.get_last_observations(0)
    #     )
    #     points = np.array([[obs.u, obs.v] for obs in current_observations_main_cam])
    #     colors = np.array(
    #         [color_from_id(obs.id) for obs in current_observations_main_cam]
    #     )
    #     rr.log(
    #         "world/camera_0/observations",
    #         rr.Points2D(positions=points, colors=colors, radii=5.0),
    #         rr.Image(image).compress(jpeg_quality=80),
    #     )

    # cap.release()

    ## Visual Wheel Odometry

    # Initialize Rerun
    rr.init("Frodo Tracker", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.TimePanel(state="collapsed"),
            rrb.Horizontal(
                column_shares=[0.5, 0.5],
                contents=[
                    rrb.Spatial2DView(origin="world/camera_0"),
                    rrb.Spatial3DView(origin="world"),
                ],
            ),
        )
    )

    vwo = VisualWheelOdometry(args.config_path)

    ctrl_idx = 0
    last_ctrl_ts = 0

    for ts in cam_timestamps:
        ret, frame = cap.read()
        if not ret:
            break

        while ctrl_idx < len(ctrl_timestamps) and last_ctrl_ts < ts:
            # update wheel odometry
            vwo.update_wheel(
                ctrl_timestamps[ctrl_idx],
                ctrl_rpm_left[ctrl_idx],
                ctrl_rpm_right[ctrl_idx],
            )
            last_ctrl_ts = ctrl_timestamps[ctrl_idx]
            ctrl_idx += 1
        logger.debug(f"t: {ts} - Wheel odometry: {vwo.wheel_odometry.poses[-1]}")

        # Convert frame to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = frame.copy()

        # logger.debug(f"t: {ts} - Image shape: {image.shape}")

        # Track frame
        pose = vwo.update_frame(ts, image)
        logger.debug(f"t: {ts} - Pose: {pose}")
        if pose is None:
            continue


        # Log pose to Rerun
        traj = [np.concatenate((xyr[:2], [0])) for xyr in vwo.trajectory]
        rr.set_time_sequence("frame", vwo.tracker.frame_id)
        rr.log("world/trajectory", rr.LineStrips3D(traj), static=True)
        quat = R.from_euler("z", pose[2], degrees=False).as_quat()
        rr.log(
            "world/camera_0",
            rr.Transform3D(translation=[pose[0], pose[1], 0], quaternion=quat),
            rr.Arrows3D(
                vectors=np.eye(3) * 0.2,
                colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],  # RGB for XYZ axes
            ),
        )

        current_observations_main_cam = (
            vwo.tracker.pycuvslam_tracker.get_last_observations(0)
        )
        points = np.array([[obs.u, obs.v] for obs in current_observations_main_cam])
        colors = np.array(
            [color_from_id(obs.id) for obs in current_observations_main_cam]
        )
        rr.log(
            "world/camera_0/observations",
            rr.Points2D(positions=points, colors=colors, radii=5.0),
            rr.Image(image).compress(jpeg_quality=80),
        )

        time.sleep(0.05)

    cap.release()
