cmake_minimum_required(VERSION 3.8)
project(pi05_piper_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StateSnapshot.msg"
  "msg/CanFrame.msg"
  "msg/CameraFrameMetadata.msg"
  "msg/PolicyChunk.msg"
  "msg/TargetPreview.msg"
  "msg/CommandProposal.msg"
  "msg/CommandReceipt.msg"
  "msg/EpisodeEvent.msg"
  "srv/SetJointTarget.srv"
  "srv/SetGripper.srv"
  "srv/SetControlMode.srv"
  "srv/SetMotorEnabled.srv"
  "srv/HomeArm.srv"
  "action/RunTask.action"
  DEPENDENCIES std_msgs sensor_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
