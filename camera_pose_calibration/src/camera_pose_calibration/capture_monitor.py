#!/usr/bin/env python

import roslib; roslib.load_manifest('camera_pose_calibration')
import cv
from cv_bridge import CvBridge, CvBridgeError
import rospy
import threading
import numpy
from calibration_msgs.msg import Interval
from calibration_msgs.msg import CalibrationPattern
from sensor_msgs.msg import Image
from camera_pose_calibration.msg import RobotMeasurement


def beep(frequency=440, amplitude=63, duration=0.5):
    try:
        sample = 8000
        half_period = int(sample/frequency/2)
        beep = chr(amplitude)*half_period+chr(0)*half_period
        beep *= int(duration*frequency)
        audio = file('/dev/audio', 'wb')
        audio.write(beep)
        audio.close()
    except:
        print "Beep beep"



class ImageRenderer:
    def __init__(self, ns):
        self.lock = threading.Lock()
        self.image_time = rospy.Time(0)
        self.image = None
        self.interval = 0
        self.features = None
        self.bridge = CvBridge()
        self.ns = ns
        self.max_interval = 1.0

        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.30, 1.5, thickness = 2)
        self.image_sub = rospy.Subscriber(ns+'/image_throttle', Image, self.image_cb)
        self.interval_sub = rospy.Subscriber(ns+'/settled_interval', Interval, self.interval_cb)
        self.features_sub = rospy.Subscriber(ns+'/features', CalibrationPattern, self.features_cb)

    def image_cb(self, msg):
        with self.lock:
            self.image_time = rospy.Time.now()
            self.image = msg

    def interval_cb(self, msg):
        with self.lock:
            self.interval = (msg.end - msg.start).to_sec()

    def features_cb(self, msg):
        with self.lock:
            self.features = msg


    def render(self, window):
        with self.lock:
            if self.image and self.image_time + rospy.Duration(2.0) > rospy.Time.now():
                cv.Resize(self.bridge.imgmsg_to_cv(self.image, 'rgb8'), window)
                interval = min(1,(self.interval / self.max_interval))
                cv.Rectangle(window,
                             (int(0.05*window.width), int(window.height*0.9)),
                             (int(interval*window.width*0.9+0.05*window.width), int(window.height*0.95)),
                             (0, interval*255, (1-interval)*255), thickness=-1)
                cv.Rectangle(window,
                             (int(0.05*window.width), int(window.height*0.9)),
                             (int(window.width*0.9+0.05*window.width), int(window.height*0.95)),
                             (0, interval*255, (1-interval)*255))
                if self.features:
                    w_scaling =  float (window.width) / self.image.width
                    h_scaling =  float (window.height) / self.image.height
                    if self.features.success:
                        corner_color = (0,255,0)
                    else:
                        corner_color = (0,0,255)
                    for cur_pt in self.features.image_points:
                        cv.Circle(window, (int(cur_pt.x*w_scaling), int(cur_pt.y*h_scaling)), int(w_scaling*5), corner_color)
            else:
                # Generate random white noise (for fun)
                noise = numpy.random.rand(window.height, window.width)*256
                numpy.asarray(window)[:,:,0] = noise;
                numpy.asarray(window)[:,:,1] = noise;
                numpy.asarray(window)[:,:,2] = noise;
                cv.PutText(window, self.ns, (int(window.width * .05), int(window.height * .95)), self.font, (0,0,255))

class Aggregator:
    def __init__(self, ns_list):
        print "Creating aggregator for ", ns_list

        self.lock = threading.Lock()
        self.capture_time = rospy.Time(0)
        self.captured_sub = rospy.Subscriber('robot_measurement', RobotMeasurement, self.captured_cb)

        # image
        w = 640
        h = 480
        self.image_out = cv.CreateMat(h, w, cv.CV_8UC3)
        self.pub = rospy.Publisher('aggregated_image', Image)
        self.bridge = CvBridge()
        self.image_success = cv.CreateMat(h, w, cv.CV_8UC3)
        text = "Successfully captured checkerboard"
        font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.30, 1.5, thickness = 2)
        ((text_w, text_h), _) = cv.GetTextSize(text, font)
        cv.PutText(self.image_success, text, (w/2-text_w/2, h/2-text_h/2), font, (0,255,0))

        # create render windows
        layouts = [ (1,1), (2,2), (2,2), (2,2), (3,3), (3,3), (3,3), (3,3), (3,3) ]
        layout = layouts[len(ns_list)-1]
        sub_w = w / layout[0]
        sub_h = h / layout[1]
        self.windows = []
        for j in range(layout[1]):
            for i in range(layout[0]):
                self.windows.append( cv.GetSubRect(self.image_out, (i*sub_w, j*sub_h, sub_w, sub_h) ) )

        # create renderers
        self.renderer_list = []
        for ns in ns_list:
            self.renderer_list.append(ImageRenderer(ns))

    def captured_cb(self, msg):
        with self.lock:
            self.capture_time = rospy.Time.now()
        beep()

    def loop(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            r.sleep()
            with self.lock:
                for window, render in zip(self.windows, self.renderer_list):
                    render.render(window)

                if self.capture_time+rospy.Duration(2.0) > rospy.Time.now():
                    self.pub.publish(self.bridge.cv_to_imgmsg(self.image_success, encoding="passthrough"))
                else:
                    self.pub.publish(self.bridge.cv_to_imgmsg(self.image_out, encoding="passthrough"))



def main():
    rospy.init_node('capture_monitor')
    args = rospy.myargv()

    a = Aggregator(args[1:])
    a.loop()



if __name__ == '__main__':
    main()