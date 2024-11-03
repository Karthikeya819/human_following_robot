import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2 as cv
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
from datetime import datetime

# PID Control Variables
time = 0
integral = 0
time_prev = -1e-6
e_prev = 0

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('Follow_Human')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cam = cv.VideoCapture('/dev/video2')
        package_share_directory = get_package_share_directory('human_following_robot')  # Replace with your package name
        model_path = os.path.join(package_share_directory, 'models', 'yolov8n.pt')  # Adjust the path as needed
        self.model = YOLO(model_path)
        # Initialize Message
        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0

        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = 0.0

    def timer_callback(self):
        ret, frame = self.cam.read()
        if not ret:
            return
        results = self.model.predict(frame, conf=0.5)

        # Filter out Person class detections
        person_boxes = [box for box in results[0].boxes if box.cls == 0.0]

        # If persons are not Found Do Nothing
        if len(person_boxes) <= 0:
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.publisher_.publish(self.msg)
            return

        # Commands Bot
        origin_y,origin_x = frame.shape[:2]
        origin_x /= 2

        x1, y1, x2, y2 = person_boxes[0].xyxy[0]
        Iorigin_x = (x2 + x1)/2

        self.msg.angular.z = float(PID(1,0,0,origin_x,Iorigin_x))/300

        # shift_x = Iorigin_x - origin_x
        # print(shift_x)
        # if abs(shift_x) > 25:
        #     if shift_x > 0:
        #         print("Move Right")
        #         self.msg.angular.z = -0.3
        #     elif shift_x < 0:
        #         print("Move Left")
        #         self.msg.angular.z = 0.3
        # else:
        #     self.msg.linear.x = 0.0
        #     self.msg.angular.z = 0.0
        
        # publish msg
        self.publisher_.publish(self.msg)

        for person in person_boxes:
            x1, y1, x2, y2 = person.xyxy[0]
            cv.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
            cv.putText(frame, f'Class: person, Conf: {person.conf[0]:.2f}', (int(x1), int(y1) - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv.circle(frame,(int(frame.shape[1]/2),int(frame.shape[0]/2)),5,(255,0,0),-1)
            cv.circle(frame,(int((x1+x2)/2),int((y1+y2)/2)),5,(255,0,0),-1)

        cv.imshow('YOLO Detection', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            pass


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

def PID(Kp, Ki, Kd, setpoint, measurement):
    global time, integral, time_prev, e_prev  # Value of offset - when the error is equal zero
    dtime = datetime.now()
    time = dtime.second + 0.000001 * dtime.microsecond
    offset = 0
    # PID calculations
    e = setpoint - measurement 
    P = Kp*e
    integral = integral + Ki*e*(time - time_prev)
    D = Kd*(e - e_prev)/(time - time_prev)  # calculate manipulated variable - MV 
    MV = offset + P + integral + D 
    # update stored data for next iteration
    e_prev = e
    time_prev = time
    return MV

if __name__ == '__main__':
    main()