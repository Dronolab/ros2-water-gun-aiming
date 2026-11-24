from ultralytics import YOLO # type: ignore
import cv2
import numpy as np
import os

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from drone_interfaces.msg import UnitVector
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory



# Load YOLO model once
MODEL_PATH = os.path.join(
    get_package_share_directory("drone_image_detection"),
    "models",
    "targets.pt"
)

model = YOLO(MODEL_PATH)


# Calibration file
CALIB_PATH = os.path.join(
    get_package_share_directory("drone_image_detection"),
    "config",
    "siyi_a8_calib.npz"
)


# Load calibration once at startup
def load_calibration():
    if not os.path.exists(CALIB_PATH):
        raise FileNotFoundError(
            f"Calibration file not found: {CALIB_PATH}"
        )

    data = np.load(CALIB_PATH)

    mtx = data["mtx"]
    dist = data["dist"]

    h = 1080
    w = 1920

    new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(
        mtx,
        dist,
        (w, h),
        1,
        (w, h)
    )

    return mtx, dist, new_camera_mtx


# Load calibration once
try:
    mtx, dist, new_camera_mtx = load_calibration()
except Exception as e:
    print(f"Error loading calibration: {e}", flush=True)
    raise

K_INV = np.linalg.inv(new_camera_mtx)

# ÉTAPE 3: output le unit vector
def uni_vector_from_pixel(u, v):
   

    pixel_point = np.array([u, v, 1.0])
    p_3d = K_INV.dot(pixel_point)

    unit_vector = p_3d / np.linalg.norm(p_3d)

    return unit_vector


# Étape 2: undistort l'image
def undistort_image(u, v):
    src_point = np.array([[[u, v]]], dtype=np.float32)

    undistorted_point = cv2.undistortPoints(
        src_point,
        mtx,
        dist,
        None,
        new_camera_mtx
    )

    undistorted_x, undistorted_y = undistorted_point[0][0]

    return uni_vector_from_pixel(
        undistorted_x,
        undistorted_y,
    )


class YoloNode(Node):

    def __init__(self):
        super().__init__("yolo_node")

        self.declare_parameter("mode", "dev")
        self.mode = self.get_parameter("mode").value

        if self.mode not in ("dev", "prod"):
            raise ValueError(
                f"Invalid mode '{self.mode}'. Expected 'dev' or 'prod'."
            )

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        self.unit_vector_publisher = self.create_publisher(
            UnitVector,
            "/yolo/unit_vector",
            10
        )

        self.get_logger().info(
            f"YOLO node started in {self.mode} mode. "
            "Waiting for /camera/image_raw..."
        )


    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

            # Calibration is currently for 1920x1080
            frame = cv2.resize(frame, (1920, 1080))

            result = model.predict(
                source=frame,
                conf=0.5,
                show=False,
                verbose=False,
                save=False,
                imgsz=640
            )

            if (
                result[0].boxes is not None
                and len(result[0].boxes) > 0
            ):
                names = result[0].names
                class_ids = result[0].boxes.cls.tolist()

                cx, cy, _, _ = (
                    result[0].boxes.xywh[0].tolist()
                )

                # Nom de l'objet détecté
                self.get_logger().info(
                    names[int(class_ids[0])]
                )

                # Coordonnées centrales du bounding box
                self.get_logger().info(
                    f"{cx} {cy}"
                )

                # Vecteur unitaire
                unit_vector = undistort_image(
                    int(cx),
                    int(cy)
                )

                self.get_logger().info(
                    f"Unit vector: {unit_vector}"
                )

                msg = UnitVector()
                msg.x = unit_vector[0]
                msg.y = unit_vector[1]
                msg.z = unit_vector[2]

                self.unit_vector_publisher.publish(msg)
                
                # Allows visualization of the annotations
                if self.mode == "dev":
                    # Temporary visualization
                    annotated_frame = result[0].plot()

                    cv2.imwrite(
                        "./webcam_frame_pred.png",
                        annotated_frame
                    )

        except Exception as e:
            self.get_logger().error(
                f"Error processing image: {e}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
