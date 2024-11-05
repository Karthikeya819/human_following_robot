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
time1 = 0
integral = 0
integral1 = 0
time_prev = -1e-6
time_prev1 = -1e-6
e_prev = 0
e_prev1 = 0

# Tracking Particular Human
REGISTERED = False
REGISTERED_ID = None
REGISTERED_AREA = None

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('Follow_Human')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cam = cv.VideoCapture('/dev/video2')
        package_share_directory = get_package_share_directory('human_following_robot')
        model_path = os.path.join(package_share_directory, 'models', 'yolov8n.pt')
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
        global REGISTERED, REGISTERED_ID, REGISTERED_AREA
        ret, frame = self.cam.read()
        if not ret:
            return
        results = self.model.track(frame, conf=0.5)

        # Filter out Person class detections
        person_boxes = [box for box in results[0].boxes if box.cls == 0.0]

        # If persons are not Found Do Nothing
        if len(person_boxes) <= 0:
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.publisher_.publish(self.msg)
            #REGISTERED = False
            return
        
        if not REGISTERED:
            for person in person_boxes:
                if person.id != None:
                    REGISTERED = True
                    REGISTERED_ID = int(person.id)
                    x1, y1, x2, y2 = person.xyxy[0]
                    REGISTERED_AREA = int((x2-x1)*(y2-y1))
                    break
        
        print(REGISTERED,REGISTERED_ID,REGISTERED_AREA)

        if REGISTERED:
            for person in person_boxes:
                if person.id == REGISTERED_ID:
                    # Commands Bot
                    origin_y,origin_x = frame.shape[:2]
                    origin_x /= 2
                    origin_y /= 2
            
                    x1, y1, x2, y2 = person.xyxy[0]
                    Iorigin_x = (x2 + x1)/2
                    Iorigin_y = (y2 + y1)/2

                    Area = int((x2-x1)*(y2-y1))
            
                    self.msg.angular.z = float(PID(1, 0, 0, origin_x, Iorigin_x)) / 300
                    self.msg.linear.x = float(PIDL(1, 0, 0, REGISTERED_AREA, Area)) / 70000

                    #print("REGIS: "+str(REGISTERED_AREA)+", Current: "+str(Area))

                    # Shift_Area = REGISTERED_AREA - Area
                    # if abs(Shift_Area) > 3000:
                    #     if Shift_Area < 0:
                    #         self.msg.linear.x = -0.2
                    #     else:
                    #         self.msg.linear.x = +0.2
                    # else:
                    #     self.msg.linear.x = 0.0
                    
                    # publish msg
                    self.publisher_.publish(self.msg)
                    break

        for person in person_boxes:
            person_id = "Null" if person.id == None else int(person.id)
            x1, y1, x2, y2 = person.xyxy[0]
            cv.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
            cv.putText(frame, f'Id: {person_id} Class: person, Conf: {person.conf[0]:.2f}', (int(x1), int(y1) - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
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
    global time, integral, time_prev, e_prev
    dtime = datetime.now()
    time = dtime.second + 0.000001 * dtime.microsecond
    # SetPoint1
    offset = 0
    e = setpoint - measurement 
    P = Kp*e
    integral = integral + Ki*e*(time - time_prev)
    D = Kd*(e - e_prev)/(time - time_prev)
    MV = offset + P + integral + D 
    # update stored data for next iteration
    e_prev = e
    time_prev = time
    return MV

# PID Linear Motion
def PIDL(Kp, Ki, Kd, setpoint, measurement):
    global time1, integral1, time_prev1, e_prev1
    dtime = datetime.now()
    time1 = dtime.second + 0.000001 * dtime.microsecond
    # SetPoint2
    offset = 0
    e = setpoint - measurement 
    print(setpoint, setpoint - measurement)
    P = Kp*e
    integral1 = integral1 + Ki*e*(time1 - time_prev1)
    D = Kd*(e - e_prev1)/(time1 - time_prev1)
    MV = offset + P + integral + D 
    # update stored data for next iteration
    e_prev1 = e
    time_prev1 = time1
    return MV

if __name__ == '__main__':
    main()