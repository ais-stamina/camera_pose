#!/usr/bin/python



import roslib; roslib.load_manifest('camera_pose_calibration')
import rospy
import threading
from calibration_msgs.msg import Interval, CalibrationPattern
import actionlib
from camera_pose_calibration.srv import *

def diff(f1, f2):
    if not f1 or not f2:
        return 999999
    return pow(f1.x - f2.x, 2) + pow(f1.y - f2.y, 2)


class FilterIntervals:
    def __init__(self):
        self.min_duration = rospy.Duration(rospy.get_param('~min_duration', 0.5))
        self.min_motion = rospy.get_param('~min_motion', 5.0)
        self.lock = threading.Lock()
        self.feature = None
        self.last_feature = None
        self.enabled = 0

        self.pub = rospy.Publisher('interval_filtered', Interval, queue_size=1)
        self.sub_intervals = rospy.Subscriber('interval', Interval, self.interval_cb)
        self.sub_features = rospy.Subscriber('features', CalibrationPattern, self.feature_cb)
        self._srv = rospy.Service('calibrate_switch', CalibrateSwitch, self.switch_on)

    def switch_on(self, goal):
        self.enabled = goal.switch
        return CalibrateSwitchResponse(self.enabled)

    def interval_cb(self, msg):
        with self.lock:
            duration = msg.end - msg.start
            
            if self.enabled and self.feature and duration > self.min_duration:
                self.last_feature = self.feature
                self.pub.publish(msg)
                self.enabled = 0


    def feature_cb(self, msg):
        with self.lock:
            if len(msg.image_points) > 0:
                self.feature = msg.image_points[0]


def main():
    rospy.init_node('filter_intervals')
    f = FilterIntervals()
    rospy.spin()



if __name__ == '__main__':
    main()
