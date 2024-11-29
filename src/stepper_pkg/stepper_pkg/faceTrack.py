import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class FaceTrackingNode(Node):
    def __init__(self):
        super().__init__('face_tracking_node')
        self.get_logger().info('Face tracking node has started.')

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            return

        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        self.image_pub = self.create_publisher(Image, '/face_image', 10)

        self.timer = self.create_timer(0.1, self.process_image)

    def process_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab frame")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

        cv2.imshow("Face Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()

    def destroy(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = FaceTrackingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
