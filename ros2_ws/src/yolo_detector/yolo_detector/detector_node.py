"""YOLO detector on the IMX219 stream.

Subscribes  : /camera/image_raw            (sensor_msgs/Image, bgr8)
Publishes   : /yolo/image_annotated        (sensor_msgs/Image, input frame + boxes)
              /yolo/detections             (vision_msgs/Detection2DArray, raw boxes)
              /yolo/timing                 (std_msgs/Float64MultiArray, per frame, ms)
                  [sof->recv, imgmsg->cv2, predict, sof->detections, sof->annotated]
                  'sof' = header.stamp = sensor start-of-frame from imx219_camera

Both outputs carry the *input* image header, so detections can be matched to
the frame (and to vehicle state) by timestamp. Box coordinates are pixels in
the input image: bbox.center = box centre, size_x/size_y = width/height,
results[0].hypothesis = (class name, confidence).

Needs Ultralytics, which lives in ~/yolo-venv (system site-packages enabled,
so rclpy/cv_bridge are visible there). The launch file runs the node with that
interpreter; with `ros2 run`, use `--prefix ~/yolo-venv/bin/python`.
"""
import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


def default_model_path():
    """Packaged TensorRT engine if it was built on this machine, else the PyTorch weights.

    best.engine is tied to this GPU + TensorRT version and is not in git; rebuild with
    `yolo export model=best.pt format=engine half=True imgsz=384,640` (see README).
    """
    d = os.path.join(get_package_share_directory('yolo_detector'), 'models')
    engine = os.path.join(d, 'best.engine')
    return engine if os.path.exists(engine) else os.path.join(d, 'best.pt')


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('model', '')            # '' -> models/best.engine, else best.pt
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('imgsz', 640)           # .pt only; an engine uses its build size
        self.declare_parameter('device', 'cuda:0')     # 'cpu' to force CPU
        self.declare_parameter('half', True)           # FP16 on GPU (Ultralytics quantize=16)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('best_effort', False)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        model_path = p('model') or default_model_path()
        self.conf, self.iou, self.imgsz = p('conf'), p('iou'), p('imgsz')
        self.device = p('device')
        self.quantize = 16 if p('half') and self.device != 'cpu' else 32
        self.publish_annotated = p('publish_annotated')

        from ultralytics import YOLO  # imported here so the error is clear if venv is missing
        self.get_logger().info(f'loading model {model_path} on {self.device}')
        self.model = YOLO(model_path, task='detect')
        self.is_engine = model_path.endswith('.engine')
        # Warm-up so the first real frame does not pay CUDA init / autotune cost.
        # A TensorRT engine has a fixed input shape: run once to read it from its metadata.
        self.model.predict(np.zeros((720, 1280, 3), np.uint8), imgsz=self.imgsz,
                           device=self.device, quantize=self.quantize, verbose=False)
        if self.is_engine:
            self.imgsz = list(self.model.predictor.model.imgsz)
        self.names = self.model.names
        backend = 'TensorRT' if self.is_engine else f'PyTorch fp{self.quantize}'
        self.get_logger().info(f'model ready ({backend}, imgsz {self.imgsz}), classes: {self.names}')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=(ReliabilityPolicy.BEST_EFFORT if p('best_effort')
                         else ReliabilityPolicy.RELIABLE))
        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(Detection2DArray, '/yolo/detections', qos)
        self.img_pub = self.create_publisher(Image, '/yolo/image_annotated', qos)
        self.timing_pub = self.create_publisher(Float64MultiArray, '/yolo/timing', 10)
        self.create_subscription(Image, p('image_topic'), self.on_image, qos)

        self._n, self._t_inf, self._n_det = 0, 0.0, 0
        self._e2e = []
        self.create_timer(5.0, self._report)

    def _age_ms(self, stamp_ns):
        return (self.get_clock().now().nanoseconds - stamp_ns) / 1e6

    def on_image(self, msg: Image):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age_recv = self._age_ms(stamp_ns)
        t0 = time.perf_counter()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        t1 = time.perf_counter()
        r = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf, iou=self.iou,
                               device=self.device, quantize=self.quantize, verbose=False)[0]
        t2 = time.perf_counter()
        self._t_inf += t2 - t1
        self._n += 1

        boxes = r.boxes
        xyxy = boxes.xyxy.cpu().numpy() if len(boxes) else np.empty((0, 4))
        confs = boxes.conf.cpu().numpy() if len(boxes) else np.empty(0)
        clss = boxes.cls.cpu().numpy().astype(int) if len(boxes) else np.empty(0, int)
        self._n_det += len(xyxy)

        out = Detection2DArray()
        out.header = msg.header
        for i, ((x1, y1, x2, y2), c, k) in enumerate(zip(xyxy, confs, clss)):
            d = Detection2D()
            d.header = msg.header
            d.id = str(i)
            d.bbox.center.position.x = float((x1 + x2) / 2)
            d.bbox.center.position.y = float((y1 + y2) / 2)
            d.bbox.size_x = float(x2 - x1)
            d.bbox.size_y = float(y2 - y1)
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = str(self.names.get(int(k), int(k)))
            h.hypothesis.score = float(c)
            d.results.append(h)
            out.detections.append(d)
        self.det_pub.publish(out)
        age_det = self._age_ms(stamp_ns)
        self._e2e.append(age_det)

        age_img = float('nan')
        if self.publish_annotated:
            vis = frame.copy()
            for (x1, y1, x2, y2), c, k in zip(xyxy.astype(int), confs, clss):
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
                label = f'{self.names.get(int(k), k)} {c:.2f}'
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                ty = max(y1, th + 6)
                cv2.rectangle(vis, (x1, ty - th - 6), (x1 + tw + 4, ty), (0, 0, 255), -1)
                cv2.putText(vis, label, (x1 + 2, ty - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.drawMarker(vis, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 12, 2)
            img = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            img.header = msg.header
            self.img_pub.publish(img)
            age_img = self._age_ms(stamp_ns)

        self.timing_pub.publish(Float64MultiArray(data=[
            age_recv, 1000 * (t1 - t0), 1000 * (t2 - t1), age_det, age_img]))

    def _report(self):
        if self._n:
            self.get_logger().info(
                f'{self._n / 5.0:.1f} fps | predict {1000 * self._t_inf / self._n:.1f} ms '
                f'| SOF->detections p50 {np.median(self._e2e):.1f} ms '
                f'| detections {self._n_det}')
        else:
            self.get_logger().warn('no images received', throttle_duration_sec=10.0)
        self._n, self._t_inf, self._n_det = 0, 0.0, 0
        self._e2e = []


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
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
