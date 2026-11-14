import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.conf = 0.4
        self.imgsz = 640
        self.device = 'cpu'
        self.track = True
        self.tracker = 'bytetrack.yaml'
        self.min_track_age = 3
        self.every_n = 1
        
        self.classes = None  # None = detect all classes; or e.g. [0] for persons
        
        self.model_name = 'yolo11n.pt'
        self.model = YOLO(self.model_name, task='detect')
        self.br = CvBridge()
        
        self.frame_count = 0
        self.track_age = {}
        
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, 'video_frames', self.callback, qos)
        
        self.pub_img = self.create_publisher(Image, 'yolo/annotated', 1)
        self.pub_det = self.create_publisher(Detection2DArray, 'yolo/detections', 10)
        self.pub_conf = self.create_publisher(Detection2DArray, 'yolo/confirmed', 10)
        
        self.get_logger().info(
            f'YOLO detector ready (track={self.track}, every_n={self.every_n}, '
            f'min_track_age={self.min_track_age}, classes={self.classes})')
        
    def make_detection(self, header, box, names):
        cx, cy, w, h = box.xywh[0].tolist()
        d = Detection2D()
        d.header = header
        d.bbox.size_x, d.bbox.size_y = w, h
        c = d.bbox.center
        if hasattr(c, 'position'):
            c.position.x, c.position.y = cx, cy
        else:
            c.x, c.y = cx, cy
            
        if box.id is not None:
            d.id = str(int(box.id))
        
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = names[int(box.cls)]
        hyp.hypothesis.score = float(box.conf)
        d.results.append(hyp)
        return d
        
    def callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.every_n:
            return
        
        frame = self.br.imgmsg_to_cv2(msg, 'bgr8')
        kwargs = dict(
            conf=self.conf,
            imgsz=self.imgsz,
            device=self.device,
            classes=self.classes,
            verbose=False
        )
        if self.track:
            # persist=True keeps tracker state between calls
            result = self.model.track(
                frame, persist=True, tracker=self.tracker, **kwargs)[0]
        else:
            result = self.model.predict(frame, **kwargs)[0]
 
        all_dets = Detection2DArray()
        all_dets.header = msg.header
        confirmed = Detection2DArray()
        confirmed.header = msg.header
        seen = set()
 
        for box in result.boxes:
            det = self.make_detection(msg.header, box, result.names)
            all_dets.detections.append(det)
 
            if not self.track:
                confirmed.detections.append(det)
                continue
 
            if box.id is None:          # not assigned to a track yet
                continue
            tid = int(box.id)
            seen.add(tid)
            age = self.track_age.get(tid, 0) + 1
            self.track_age[tid] = age
            if age >= self.min_track_age:
                confirmed.detections.append(det)
 
        if self.track:
            # forget tracks that were not seen in this frame, so "age" means
            # consecutive frames and ghost IDs don't accumulate
            self.track_age = {t: a for t, a in self.track_age.items() if t in seen}
 
        self.pub_det.publish(all_dets)
        self.pub_conf.publish(confirmed)
 
        # rendering the overlay costs CPU, so skip it if nobody is watching
        if self.pub_img.get_subscription_count() > 0:
            annotated = self.br.cv2_to_imgmsg(result.plot(), 'bgr8')
            annotated.header = msg.header
            self.pub_img.publish(annotated)
        
def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()