#!/usr/bin/env python  
import roslib
roslib.load_manifest('camera_pose_calibration')
import rospy
import math
import threading
import tf2_ros
import tf
from tf_conversions import posemath
from geometry_msgs.msg import TransformStamped
from camera_pose_calibration.msg import CameraCalibration
from calibration_estimation.single_transform import quat_to_rpy
from calibration_estimation.urdf_params import UrdfParams
from update_urdf import update_urdf


class CameraTransformator:
    '''
    Given a new pose of the camera in the world frame compute the 
    transformation between the camera link and its parent link
    '''
        
    def __init__(self, joint, chain, tf_listener, world_frame = "base_link"):

        self.joint = joint
        self.chain = chain
        self.world_frame = world_frame
        self.lock = threading.Lock()
        
        self.tf_listener = tf_listener
        #self.tf_broadcaster = tf.TransformBroadcaster()
        self.precompute()
        

    def set_pose(self, pose):
        with self.lock:
            self.pose = posemath.fromTf(((pose.position.x, pose.position.y, pose.position.z), \
                                         (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)))

    def precompute(self):
        '''
        Precomputed the transformation chain from the optical frame to the camera link
        since it's assumed to be static
        '''
        # calculating the chain of transformations 
        # (presumably from the optical frame (to which we have the world frame transform)
        # to the child link of the joint in question
        transformations = list()
        for i in xrange(len(self.chain)-1, 0, -2):
            from_link = self.chain[i]
            to_link = self.chain[i - 2] # assuming alternating link/joint
                
            transform = self._get_transform(from_link, to_link);
            if transform is None:
                rospy.logerror("Unable to get the transform %s -> %s", from_link, to_link)
                return
                
            transformations.append(transform)
         
        self.optframe_camframe = reduce(lambda a, b: b*a, transformations)
         
        # computing the transform: joint's parent link -> world frame 
        self.lparent_world = self._get_transform(self.joint.parent, self.world_frame);
        if self.lparent_world is None:
            rospy.logerror("Unable to get transform %s -> %s", self.joint.parent, self.world_frame)
            return 
        
        
    def get_transform(self):
        with self.lock:
            # computing the transform: joint's child link -> world frame
            lchild_world = self.pose.Inverse() * self.optframe_camframe
            
            # finally the transform: parent_link -> child_link
            t = self.lparent_world * lchild_world.Inverse()
            
            rot = t.M.GetRPY()
            return (t.p.x(), t.p.y(), t.p.z(), rot[0], rot[1], rot[2])
            
            #transform = optical_frame_to_base_link * frame_to_optical_frame.Inverse() * link_to_frame.Inverse()
            #(trans, rot) = posemath.toTf(transform)
            #self.tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), self.camera_id + "_link", "base_link")
            
            #self.transform.header.stamp = rospy.Time.now() + rospy.Duration(0.5)
            #self.pub.sendTransform(self.transform)
    
    def _get_transform(self, from_, to_):
         try:
             #rospy.loginfo("Waiting for the transform %s -> %s" % (from_, to_))
             self.tf_listener.waitForTransform(from_, to_, rospy.Time(0), rospy.Duration(5))
             return posemath.fromTf( self.tf_listener.lookupTransform(from_, to_, rospy.Time(0)) )
         except (tf.Exception):
             rospy.logdebug("Transform lookup from %s to %s failed." % (from_, to_))
         
         return None


class CalibrationManager:
    def __init__(self, robot_description, urdf_xml, joint_names):
    
        self.lock = threading.Lock()
        self.urdf_xml = urdf_xml
        self.publish_list = {}
        self.tf_listener = tf.TransformListener()
        self.config = {'sensors': {'rectified_cams': {}, 'chains': {}, 'tilting_lasers': {}}, 'checkerboards': {}, 'transforms': {}}
        
        self.joint_names = joint_names
        self.robot_params = UrdfParams(robot_description, self.config)
        self.updated = False
        self.sub = rospy.Subscriber('camera_calibration', CameraCalibration, self.cal_cb)
    
    def is_updated(self):
        return self.updated

    def cal_cb(self, msg):
        with self.lock:
            self.publish_list = {}
            for pose, camera in zip(msg.camera_pose, msg.camera_id):
                camera = camera.strip("/")
                if not camera in self.publish_list and camera in self.joint_names:
                    joint = self.robot_params.urdf.joint_map[self.joint_names[camera]]
                    chain = self.robot_params.urdf.get_chain(joint.child, camera.strip("/"))
                    self.publish_list[camera] = CameraTransformator(joint, chain, self.tf_listener)
                
                self.publish_list[camera].set_pose(pose)
                self.updated = True

    def update(self):
        with self.lock:
            transforms = {}
            for link_name, joint_name in self.joint_names.iteritems():
                transforms[joint_name] = self.publish_list[link_name].get_transform()
            
            self.config['transforms'] = transforms
            self.robot_params.configure(self.robot_params.get_clean_urdf(), self.config)
            new_urdf = update_urdf(self.robot_params.get_clean_urdf(), self.robot_params)
        
            # write out to URDF
            outfile = open(self.urdf_xml, 'w')
            rospy.loginfo('Writing model updates to %s', self.urdf_xml)
            outfile.write( new_urdf.to_xml_string() )
            outfile.close()
            
            # updating the description
            rospy.set_param("robot_description", new_urdf.to_xml_string())
            
            self.updated = False


def usage():
    return "Usage: <path_to_urdf>, <link>:<joint> [<link>:<joint> [...]]"

def main():
    rospy.init_node('stamina_calibration_tf_publisher')
    robot_description = rospy.get_param("robot_description")
    r = rospy.Rate(5)
    
    if (len(rospy.myargv()) < 3):
        rospy.logerror(usage())
        return
    else:
        # the keys are the optical frames of the calibrated cameras
        # the values are the fixed joints of the cameras and their base
        cam_dict = {}
        urdf_xml = rospy.myargv()[1]
        try:
            for i in xrange(2, len(rospy.myargv())):
                [cam_link, joint] = rospy.myargv()[i].split(':')
                cam_dict[cam_link] = joint
        except:
            rospy.logerror(usage())
            return
    
    print cam_dict
    c = CalibrationManager(robot_description, urdf_xml, cam_dict)
    
    while not rospy.is_shutdown():
        if c.is_updated():
            c.update()
        
        try:
            r.sleep()
        except:
            rospy.loginfo("Shutting down")


if __name__ == '__main__':
    main()
