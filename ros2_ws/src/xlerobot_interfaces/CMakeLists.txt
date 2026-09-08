cmake_minimum_required(VERSION 3.8)
project(xlerobot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CapabilityError.msg"
  "msg/CapabilityProgress.msg"
  "msg/CalibrationTargetObservation.msg"
  "msg/PolicyJointChunk.msg"
  "msg/PolicyStartContext.msg"
  "msg/PerceptionObservation.msg"
  "msg/ScanMapConsistency.msg"
  "msg/ServoCalibrationJoint.msg"
  "msg/TaskEvent.msg"
  "msg/TopGraspPlan.msg"
  "action/ApproachTarget.action"
  "action/AutoLocalize.action"
  "action/CalibrationJob.action"
  "action/CollectEpisode.action"
  "action/DetectObject.action"
  "action/ExecuteTask.action"
  "action/ExecutePolicyStream.action"
  "action/ExecuteLearnedPolicy.action"
  "action/GraspObject.action"
  "action/VerifyGrasp.action"
  "action/HandoverObject.action"
  "action/NavigateToNamedPlace.action"
  "action/PrepareGrasp.action"
  "action/RecordEpisode.action"
  "action/ScanForPerson.action"
  "action/SetMaintenancePreset.action"
  "action/MoveCalibrationPose.action"
  "action/SpeakText.action"
  "srv/ActivateArtifact.srv"
  "srv/CaptureCalibrationSample.srv"
  "srv/ImportCalibrationResult.srv"
  "srv/FinalizeEpisode.srv"
  "srv/BeginEpisode.srv"
  "srv/MarkEpisodeEvent.srv"
  "srv/ReviewEpisode.srv"
  "srv/RecordSiteValidation.srv"
  "srv/RemoveNamedPlace.srv"
  "srv/SaveSiteMap.srv"
  "srv/SaveBaseGeometryCalibration.srv"
  "srv/SolveCalibrationSamples.srv"
  "srv/ServoCalibrationStep.srv"
  "srv/SetNamedPlace.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
