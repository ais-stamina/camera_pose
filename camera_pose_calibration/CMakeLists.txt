cmake_minimum_required(VERSION 2.4.6)
project(camera_pose_calibration)

find_package(catkin REQUIRED COMPONENTS rospy
										geometry_msgs
										calibration_msgs
										sensor_msgs
										image_cb_detector
										monocam_settler
										interval_intersection
										joint_states_settler
										#orocos_kdl
										tf_conversions
										tf2_ros
										tf2_kdl
										message_filters)

add_message_files(
   FILES
   CalibrationEstimate.msg
   CameraCalibration.msg
   CameraMeasurement.msg
   CameraPose.msg
   RobotMeasurement.msg
)

generate_messages(DEPENDENCIES geometry_msgs calibration_msgs sensor_msgs)

catkin_package()
