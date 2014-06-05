#!/usr/bin/env python  
import roslib
roslib.load_manifest('camera_pose_calibration')
import rospy
import math
import threading
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from tf_conversions import posemath
from geometry_msgs.msg import TransformStamped
from camera_pose_calibration.msg import CameraCalibration

class CameraPublisher:
    def __init__(self, pose, child_frame_id, tf_listener):
        self.camera_id = child_frame_id.replace("_rgb_optical_frame", "")
        self.lock = threading.Lock()
        self.tf_listener = tf_listener
        #self.pub = tf2_ros.TransformBroadcaster()
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        #self.pose = pose
        self.set_pose(pose, child_frame_id)
        

    def set_pose(self, pose, child_frame_id):
        with self.lock:
            self.transform = TransformStamped()
            self.transform.header.frame_id = 'base_link'
            self.transform.child_frame_id = child_frame_id
            self.transform.transform.translation.x = pose.position.x
            self.transform.transform.translation.y = pose.position.y
            self.transform.transform.translation.z = pose.position.z
            self.transform.transform.rotation.x = pose.orientation.x
            self.transform.transform.rotation.y = pose.orientation.y
            self.transform.transform.rotation.z = pose.orientation.z
            self.transform.transform.rotation.w = pose.orientation.w
            self.pose = posemath.fromTf(((pose.position.x, pose.position.y, pose.position.z), \
                                         (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)))

    def publish(self):
        with self.lock:
            #rospy.loginfo("Trying to get the transforms")
            # publishing the transform base -> link
            frame_to_optical_frame = self._get_transform(self.camera_id + "_rgb_frame", \
                                                    self.camera_id + "_rgb_optical_frame");
            if frame_to_optical_frame is None:
                return

            link_to_frame = self._get_transform(self.camera_id + "_link", \
                                           self.camera_id + "_rgb_frame");
                                           
            if link_to_frame is None:
                return
            
            optical_frame_to_base_link = self.pose#self._get_transform("base_link", \
                                         #               self.camera_id + "_rgb_optical_frame");
            
            if optical_frame_to_base_link is None:
                return
        
            # Transform base -> _link = base -> _rgb_optical_frame * _rgb_optical_frame -> _rgb_frame * _rgb_frame -> _link
            #rospy.loginfo("Got all transforms. Computing and publishing...")
        
            transform = optical_frame_to_base_link * frame_to_optical_frame.Inverse() * link_to_frame.Inverse()
            (trans, rot) = posemath.toTf(transform)
            self.tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), self.camera_id + "_link", "base_link")
            
            #self.transform.header.stamp = rospy.Time.now() + rospy.Duration(0.5)
            #self.pub.sendTransform(self.transform)
    
    def _get_transform(self, from_, to_):
         try:
             #rospy.loginfo("Waiting for the transform %s -> %s" % (from_, to_))
             self.tf_listener.waitForTransform(from_, to_, rospy.Time(0), rospy.Duration(5))
             return posemath.fromTf( self.tf_listener.lookupTransform(from_, to_, rospy.Time(0)) )
         except (tf.Exception):
             rospy.logdebug("Transform lookup from %s to %s failed. Retrying..." % (from_, to_))
        
         return None


class CalibrationPublishManager:
    def __init__(self):
    
        self.lock = threading.Lock()
        self.publish_list = {}
        self.sub = rospy.Subscriber('camera_calibration', CameraCalibration, self.cal_cb)
        
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

    def cal_cb(self, msg):
        with self.lock:
            self.publish_list = {}
            for pose, camera in zip(msg.camera_pose, msg.camera_id):
                self.publish_list[camera] = CameraPublisher(pose, camera, self.tf_listener)


    def publish(self):
        with self.lock:
            for name, pub in self.publish_list.iteritems():
                pub.publish()


def main():
    rospy.init_node('stamina_calibration_tf_publisher')
    r = rospy.Rate(5)
    
    #cam_param = "/stamina_camera_driver/cameras"
    
    #try:
    #    cameras = rospy.get_param(cam_param)
    #except KeyError:
    #    rospy.logerror("Parameter %s is not set. Make sure the driver is loaded." % cam_param)
    #    return
    
    c = CalibrationPublishManager()
    
    while not rospy.is_shutdown():
        c.publish()
        
        try:
            r.sleep()
        except:
            rospy.loginfo("Shutting down")


if __name__ == '__main__':
    main()
