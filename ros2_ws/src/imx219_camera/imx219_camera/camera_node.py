"""IMX219 CSI camera -> ROS 2 sensor_msgs/Image.

Captures through the Jetson ISP (nvarguscamerasrc -> nvvidconv) with OpenCV's
GStreamer backend, converts with cv_bridge and publishes bgr8 frames.
The /dev/video0 V4L2 node only exposes raw RG10 Bayer, so the Argus path is
required to get debayered, auto-exposed colour images.

White balance: on L4T R39 the Argus auto-WB (wbmode=1) for IMX219 gives a
strong magenta cast (measured R/G=1.68, B/G=1.60 on a white-lit scene). The
'fluorescent' preset (wbmode=3) is close to neutral (R/G=1.09, B/G=1.01), so
it is the default, and a software gray-world AWB trims the remaining cast and
follows lighting changes.
"""
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image


def build_pipeline(sensor_id, width, height, fps, flip_method, wbmode, saturation):
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} wbmode={wbmode} '
        f'saturation={saturation} ! '
        f'video/x-raw(memory:NVMM),width={width},height={height},'
        f'framerate={fps}/1,format=NV12 ! '
        f'nvvidconv flip-method={flip_method} ! video/x-raw,format=BGRx ! '
        'videoconvert ! video/x-raw,format=BGR ! '
        'appsink drop=true max-buffers=1 sync=false'
    )


class GrayWorldAWB:
    """Gray-world white balance with temporal smoothing and clamped gains.

    Gains are estimated on a downscaled frame, ignoring clipped highlights and
    near-black pixels, normalised so the smallest gain is 1 (boost-only) and
    low-pass filtered so the image
    does not flicker. Gains are applied with one cv2.transform (uint8 saturating).
    """

    def __init__(self, alpha=0.1, max_gain=2.0, every_n=3):
        self.alpha = alpha
        self.max_gain = max_gain
        self.every_n = max(1, every_n)
        self.gains = np.ones(3, np.float32)  # B, G, R
        self._n = 0

    def _estimate(self, frame):
        small = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA)
        px = small.reshape(-1, 3).astype(np.float32)
        mx, mn = px.max(1), px.min(1)
        px = px[(mx < 235) & (mn > 15)]
        if len(px) < 200:
            return None
        mean = np.maximum(px.mean(0), 1e-3)
        # Normalise to the strongest channel so every gain is >= 1: a gain < 1
        # would pull clipped (255) channels down and tint blown highlights
        # (e.g. lamps turn green/cyan). Boost-only keeps saturated white white.
        return np.clip(mean.max() / mean, 1.0, self.max_gain)

    def __call__(self, frame):
        if self._n % self.every_n == 0:
            target = self._estimate(frame)
            if target is not None:
                self.gains += self.alpha * (target - self.gains)
        self._n += 1
        return apply_gains(frame, self.gains)


def apply_gains(frame, gains_bgr):
    return cv2.transform(frame, np.diag(np.asarray(gains_bgr, np.float32)))


class Imx219CameraNode(Node):
    def __init__(self):
        super().__init__('imx219_camera')
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('flip_method', 0)   # nvvidconv: 0 none, 2 rotate-180
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('best_effort', False)  # True for SensorData-style QoS
        # --- colour ---
        self.declare_parameter('wbmode', 3)          # Argus WB preset, 3 = fluorescent
        self.declare_parameter('saturation', 1.0)    # Argus saturation 0..2
        self.declare_parameter('awb', 'grayworld')   # 'grayworld' | 'manual' | 'none'
        self.declare_parameter('awb_gains', [1.0, 1.0, 1.0])  # B,G,R for 'manual'
        self.declare_parameter('awb_alpha', 0.1)     # gray-world smoothing (0..1)
        self.declare_parameter('awb_max_gain', 2.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.frame_id = p('frame_id')
        self.width, self.height = p('width'), p('height')

        awb = p('awb')
        if awb == 'grayworld':
            self.post = GrayWorldAWB(alpha=p('awb_alpha'), max_gain=p('awb_max_gain'))
        elif awb == 'manual':
            gains = list(p('awb_gains'))
            self.post = lambda f: apply_gains(f, gains)
        elif awb == 'none':
            self.post = None
        else:
            raise ValueError(f"awb must be grayworld|manual|none, got '{awb}'")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=(ReliabilityPolicy.BEST_EFFORT if p('best_effort')
                         else ReliabilityPolicy.RELIABLE))
        self.image_pub = self.create_publisher(Image, 'image_raw', qos)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', qos)
        self.bridge = CvBridge()

        pipeline = build_pipeline(p('sensor_id'), self.width, self.height, p('fps'),
                                  p('flip_method'), p('wbmode'), p('saturation'))
        self.get_logger().info(f'GStreamer pipeline: {pipeline}')
        self.get_logger().info(f'software AWB: {awb}')
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError('Failed to open camera pipeline '
                               '(is nvargus-daemon running / camera busy?)')

        self._running = True
        self._frames = 0
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        self.create_timer(5.0, self._report)

    def _capture_loop(self):
        while self._running and rclpy.ok():
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn('Frame grab failed', throttle_duration_sec=2.0)
                continue
            stamp = self.get_clock().now().to_msg()
            if self.post is not None:
                frame = self.post(frame)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            self.image_pub.publish(msg)

            info = CameraInfo()  # uncalibrated: size only
            info.header = msg.header
            info.width, info.height = frame.shape[1], frame.shape[0]
            self.info_pub.publish(info)
            self._frames += 1

    def _report(self):
        msg = f'publishing {self._frames / 5.0:.1f} fps'
        if isinstance(self.post, GrayWorldAWB):
            b, g, r = self.post.gains
            msg += f' | AWB gains B={b:.2f} G={g:.2f} R={r:.2f}'
        self.get_logger().info(msg)
        self._frames = 0

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Imx219CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
