#include <tablebot_moveit_config/common_actions.h>

using namespace std;

/** Adjust angle to equivalent angle on [-pi, pi]
*  @param angle the angle to be simplified (-inf, inf)
*  @return the simplified angle on [-pi, pi]
*/
static inline double simplify_angle(double angle)
{
  double previous_rev = floor(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double next_rev = ceil(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double current_rev;
  if (fabs(angle - previous_rev) < fabs(angle - next_rev))
    return angle - previous_rev;
  return angle - next_rev;
}

CommonActions::CommonActions() : pnh("~"),
    moveToJointPoseClient("tablebot_moveit_wrapper/move_to_joint_pose"),
    moveToPoseClient("tablebot_moveit_wrapper/move_to_pose"),
    gripperClient("gripper_actions/gripper_manipulation"),
    liftClient("tablebot_moveit/common_actions/lift"),
    verifyClient("gripper_actions/verify_grasp"),
    liftServer(n, "tablebot_moveit/common_actions/lift", boost::bind(&CommonActions::liftArm, this, _1), false),
    armServer(n, "tablebot_moveit/common_actions/arm_action", boost::bind(&CommonActions::executeArmAction, this, _1), false),
    pickupServer(n, "tablebot_moveit/common_actions/pickup", boost::bind(&CommonActions::executePickup, this, _1), false),
    storeServer(n, "tablebot_moveit/common_actions/store", boost::bind(&CommonActions::executeStore, this, _1), false)
{
  //setup home position
  homePosition.resize(NUM_JACO_JOINTS);
  homePosition[0] = -1.4776;
  homePosition[1] = 2.9246;
  homePosition[2] = 1.002;
  homePosition[3] = -2.0793;
  homePosition[4] = 1.4455;
  homePosition[5] = 1.3156;

  defaultRetractPosition.resize(NUM_JACO_JOINTS);
  defaultRetractPosition[0] = -2.9388;
  defaultRetractPosition[1] = 2.8894;
  defaultRetractPosition[2] = 0.5216;
  defaultRetractPosition[3] = -3.1342;
  defaultRetractPosition[4] = 1.0143;
  defaultRetractPosition[5] = 1.1296;

  angularCmdPublisher = n.advertise<wpi_jaco_msgs::AngularCommand>("jaco_arm/angular_cmd", 1);

  eraseTrajectoriesClient = n.serviceClient<std_srvs::Empty>("jaco_arm/erase_trajectories");
  cartesianPathClient = n.serviceClient<rail_manipulation_msgs::CartesianPath>("tablebot_moveit_wrapper/cartesian_path");
  jacoPosClient = n.serviceClient<wpi_jaco_msgs::GetAngularPosition>("jaco_arm/get_angular_position");
  attachClosestObjectClient = n.serviceClient<std_srvs::Empty>("tablebot_moveit_wrapper/attach_closest_object");
  detachObjectsClient = n.serviceClient<std_srvs::Empty>("tablebot_moveit_wrapper/detach_objects");
  prepareGraspClient = n.serviceClient<rail_manipulation_msgs::PrepareGrasp>("tablebot_moveit_wrapper/prepare_grasp");
  reactivateGraspCollisionClient = n.serviceClient<std_srvs::Empty>("tablebot_moveit_wrapper/reactivate_grasp_collision");

  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/common_actions/approach_angle_pose_test", 1000);

  //start action server
  liftServer.start();
  armServer.start();
  pickupServer.start();
  storeServer.start();
}

void CommonActions::executePickup(const rail_manipulation_msgs::PickupGoalConstPtr &goal)
{
  ROS_INFO("###########################received pickup goal");
  rail_manipulation_msgs::PickupFeedback feedback;
  rail_manipulation_msgs::PickupResult result;
  result.success = false;
  stringstream ss;

  //make sure pose is in the table_base_link frame
  geometry_msgs::PoseStamped graspPose, approachAnglePose;
  graspPose.header.frame_id = "table_base_link";
  if (goal->pose.header.frame_id != "table_base_link")
    tfListener.transformPose("table_base_link", goal->pose, graspPose);
  else
    graspPose = goal->pose;

  tf::Transform graspTransform;
  graspTransform.setOrigin(tf::Vector3(graspPose.pose.position.x, graspPose.pose.position.y, graspPose.pose.position.z));
  graspTransform.setRotation(tf::Quaternion(graspPose.pose.orientation.x, graspPose.pose.orientation.y, graspPose.pose.orientation.z, graspPose.pose.orientation.w));
  ros::Time now = ros::Time::now();
  tfBroadcaster.sendTransform(tf::StampedTransform(graspTransform, now, "table_base_link", "grasp_frame"));
  tfListener.waitForTransform("grasp_frame", "table_base_link", now, ros::Duration(5.0));  

  approachAnglePose.header.frame_id = "grasp_frame";
  // originally -0.1 is too close to the object. unable to plan to.
  approachAnglePose.pose.position.x = -0.2;
  approachAnglePose.pose.orientation.w = 1.0;

  geometry_msgs::PoseStamped newGraspPose;
  newGraspPose.header.frame_id = "grasp_frame";
  newGraspPose.pose.position.x = -0.07;
  newGraspPose.pose.orientation.w = 1.0;
  graspPose = newGraspPose;
  pose_pub.publish(graspPose);
  //approachAnglePose.pose.position.x = 0;
  //approachAnglePose.pose.position.y = 0;
  //approachAnglePose.pose.position.z = 0.3;
  //approachAnglePose.pose.orientation = graspPose.pose.orientation;

  //move to approach angle
  ss.str("");
  ss << "Attempting to move gripper to approach angle...";
  ROS_INFO("###########################Attempting to move gripper to approach angle...");
  feedback.message = ss.str();
  pickupServer.publishFeedback(feedback);

  rail_manipulation_msgs::MoveToPoseGoal approachAngleGoal;
  approachAngleGoal.pose = approachAnglePose;
  approachAngleGoal.planningTime = 3.0;
  moveToPoseClient.sendGoal(approachAngleGoal);
  moveToPoseClient.waitForResult(ros::Duration(30.0));
  if (!moveToPoseClient.getResult()->success)
  {
    ss.str("");
    ss << "Moving to approach angle failed for this grasp.";
    ROS_INFO("###########################Moving to approach angle failed for this grasp.");
    feedback.message = ss.str();
    pickupServer.publishFeedback(feedback);
    result.executionSuccess = false;
    pickupServer.setAborted(result, "Unable to move to approach angle.");
    ROS_INFO("###########################Unable to move to approach angle.");
  }
  ROS_INFO("###########################moved to approach angle.");

  //open gripper
  ss.str("");
  ss << "Opening gripper...";
  ROS_INFO("###########################opening gripper.");
  feedback.message = ss.str();
  pickupServer.publishFeedback(feedback);

  rail_manipulation_msgs::GripperGoal gripperGoal;
  gripperGoal.close = false;
  gripperClient.sendGoal(gripperGoal);
  gripperClient.waitForResult(ros::Duration(10.0));
  if (!gripperClient.getResult()->success)
  {
    ROS_INFO("Opening gripper failed.");
    result.executionSuccess = false;
    pickupServer.setAborted(result, "Unable to open gripper.");
    return;
  }
  ROS_INFO("###########################opened gripper.");


  rail_manipulation_msgs::PrepareGrasp prepareGrasp;
  prepareGrasp.request.graspPose = graspPose;
  prepareGraspClient.call(prepareGrasp);

  //plan to grasp pose
  ss.str("");
  ss << "Moving to grasp pose...";
  ROS_INFO("###########################moving to grasp pose.");
  feedback.message = ss.str();
  pickupServer.publishFeedback(feedback);

  //TODO: possible change: if failed, use the linear method of the old executePickup ???
  rail_manipulation_msgs::MoveToPoseGoal graspPoseGoal;
  graspPoseGoal.pose = graspPose;
  graspPoseGoal.jointStateDifference = M_PI/2.0;
  graspPoseGoal.planningTime = 3.0;
  moveToPoseClient.sendGoal(graspPoseGoal);
  moveToPoseClient.waitForResult(ros::Duration(30.0));
  if (!moveToPoseClient.getResult()->success) //TODO: fix other bug here
  {
    //reactivate grasp collisions
    std_srvs::Empty emptySrv;
    reactivateGraspCollisionClient.call(emptySrv);

    ss.str("");
    ss << "Moving to grasp pose failed for this grasp.";
    ROS_INFO("###########################Moving to grasp pose failed for this grasp.");
    feedback.message = ss.str();
    pickupServer.publishFeedback(feedback);
    result.executionSuccess = false;
    pickupServer.setAborted(result, "Unable to move to approach angle.");
    ROS_INFO("###########################Unable to move to approach angle.");
    return;
  }

  //close gripper
  ss.str("");
  ss << "Closing gripper...";
  feedback.message = ss.str();
  pickupServer.publishFeedback(feedback);

  gripperGoal.close = true;
  gripperClient.sendGoal(gripperGoal);
  gripperClient.waitForResult(ros::Duration(10.0));
  if (!gripperClient.getResult()->success)
  {
    ROS_INFO("Closing gripper failed.");
    result.executionSuccess = false;
    pickupServer.setAborted(result, "Unable to close gripper.");
    return;
  }

  ROS_INFO("Passed threshold for execution success.");
  result.executionSuccess = true;

  //attach scene object to gripper
  std_srvs::Empty emptySrv;
  if (!attachClosestObjectClient.call(emptySrv))
  {
    ROS_INFO("No scene object to attach...");
  }

  if (goal->lift)
  {
    //lift hand
    ss.str("");
    ss << "Lifting hand...";
    feedback.message = ss.str();
    pickupServer.publishFeedback(feedback);

    rail_manipulation_msgs::LiftGoal liftGoal;
    liftClient.sendGoal(liftGoal);
    liftClient.waitForResult(ros::Duration(10.0));
  }

  if (goal->verify)
  {
    ss.str("");
    ss << "Verifying grasp...";
    feedback.message = ss.str();
    pickupServer.publishFeedback(feedback);

    rail_manipulation_msgs::VerifyGraspGoal verifyGoal;
    verifyClient.sendGoal(verifyGoal);
    verifyClient.waitForResult(ros::Duration(10.0));
    if (!(verifyClient.getResult()->success && verifyClient.getResult()->grasping))
    {
      ROS_INFO("Grasp evaluated as unsuccessful.");
      result.success = false;
      std_srvs::Empty detachSrv;
      detachObjectsClient.call(detachSrv);
      pickupServer.setAborted(result, "Grasp evaluated as unsuccessful.");
      return;
    }
    else
      result.success = true;
  }
  else
  {
    result.success = true;
  }

  pickupServer.setSucceeded(result);
}

/* Old method involving linear pickup trajectory... remove if new method proves sufficient
void CommonActions::executePickup(const rail_manipulation_msgs::PickupGoalConstPtr &goal)
{
  rail_manipulation_msgs::PickupFeedback feedback;
  rail_manipulation_msgs::PickupResult result;
  result.success = false;
  stringstream ss;

  //make sure pose is in the table_base_link frame
  geometry_msgs::PoseStamped graspPose, approachAnglePose;
  graspPose.header.frame_id = "table_base_link";
  if (goal->pose.header.frame_id != "table_base_link")
    tfListener.transformPose("table_base_link", goal->pose, graspPose);
  else
    graspPose = goal->pose;

  tf::Transform graspTransform;
  graspTransform.setOrigin(tf::Vector3(graspPose.pose.position.x, graspPose.pose.position.y, graspPose.pose.position.z));
  graspTransform.setRotation(tf::Quaternion(graspPose.pose.orientation.x, graspPose.pose.orientation.y, graspPose.pose.orientation.z, graspPose.pose.orientation.w));
  ros::Time now = ros::Time::now();
  tfBroadcaster.sendTransform(tf::StampedTransform(graspTransform, now, "table_base_link", "grasp_frame"));
  tfListener.waitForTransform("grasp_frame", "table_base_link", now, ros::Duration(5.0));

  approachAnglePose.header.frame_id = "grasp_frame";
  approachAnglePose.pose.position.x = -0.1;
  approachAnglePose.pose.orientation.w = 1.0;

  //move to approach angle
  ss.str("");
  ss << "Attempting to move gripper to approach angle...";
  feedback.message = ss.str();
  pickupServer.publishFeedback(feedback);

  rail_manipulation_msgs::MoveToPoseGoal approachAngleGoal;
  approachAngleGoal.pose = approachAnglePose;
  moveToPoseClient.sendGoal(approachAngleGoal);
  moveToPoseClient.waitForResult(ros::Duration(30.0));
  if (!moveToPoseClient.getResult()->success)
  {
    ss.str("");
    ss << "Moving to approach angle failed for this grasp.";
    feedback.message = ss.str();
    pickupServer.publishFeedback(feedback);
    result.executionSuccess = false;
    pickupServer.setAborted(result, "Unable to move to approach angle.");
    return;
  }

  //open gripper
  ss.str("");
  ss << "Opening gripper...";
  feedback.message = ss.str();
  pickupServer.publishFeedback(feedback);

  rail_manipulation_msgs::GripperGoal gripperGoal;
  gripperGoal.close = false;
  gripperClient.sendGoal(gripperGoal);
  gripperClient.waitForResult(ros::Duration(10.0));
  if (!gripperClient.getResult()->success)
  {
    ROS_INFO("Opening gripper failed.");
    result.executionSuccess = false;
    pickupServer.setAborted(result, "Unable to open gripper.");
    return;
  }

  //follow approach angle to grasp
  ss.str("");
  ss << "Moving along approach angle...";
  feedback.message = ss.str();
  pickupServer.publishFeedback(feedback);

  rail_manipulation_msgs::CartesianPath srv;
  geometry_msgs::PoseStamped cartesianPose;
  cartesianPose.header.frame_id = "table_base_link";
  tfListener.transformPose("table_base_link", graspPose, cartesianPose);
  srv.request.waypoints.push_back(cartesianPose.pose);
  srv.request.avoidCollisions = false;
  if (!cartesianPathClient.call(srv))
  {
    ROS_INFO("Could not call Jaco Cartesian path service.");
    result.executionSuccess = false;
    pickupServer.setAborted(result, "Could not call Jaco Cartesian path service.");
    return;
  }

  //close gripper
  ss.str("");
  ss << "Closing gripper...";
  feedback.message = ss.str();
  pickupServer.publishFeedback(feedback);

  gripperGoal.close = true;
  gripperClient.sendGoal(gripperGoal);
  gripperClient.waitForResult(ros::Duration(10.0));
  if (!gripperClient.getResult()->success)
  {
    ROS_INFO("Closing gripper failed.");
    result.executionSuccess = false;
    pickupServer.setAborted(result, "Unable to close gripper.");
    return;
  }

  result.executionSuccess = true;

  //attach scene object to gripper
  std_srvs::Empty emptySrv;
  if (!attachClosestObjectClient.call(emptySrv))
  {
    ROS_INFO("No scene object to attach...");
  }

  if (goal->lift)
  {
    //lift hand
    ss.str("");
    ss << "Lifting hand...";
    feedback.message = ss.str();
    pickupServer.publishFeedback(feedback);

    rail_manipulation_msgs::LiftGoal liftGoal;
    liftClient.sendGoal(liftGoal);
    liftClient.waitForResult(ros::Duration(10.0));
  }

  if (goal->verify)
  {
    ss.str("");
    ss << "Verifying grasp...";
    feedback.message = ss.str();
    pickupServer.publishFeedback(feedback);

    rail_manipulation_msgs::VerifyGraspGoal verifyGoal;
    verifyClient.sendGoal(verifyGoal);
    verifyClient.waitForResult(ros::Duration(10.0));
    if (!(verifyClient.getResult()->success && verifyClient.getResult()->grasping))
    {
      ROS_INFO("Grasp evaluated as unsuccessful.");
      result.success = false;
      pickupServer.setAborted(result, "Grasp evaluated as unsuccessful.");
      return;
    }
    else
      result.success = true;
  }
  else
  {
    result.success = true;
  }

  pickupServer.setSucceeded(result);
}
*/

void CommonActions::executeStore(const rail_manipulation_msgs::StoreGoalConstPtr &goal)
{
  rail_manipulation_msgs::StoreFeedback feedback;
  rail_manipulation_msgs::StoreResult result;
  stringstream ss;

  // store the wrist angle so the lift action doesn't have to rotate back
  float wristAngle;

  //make sure pose is in the table_base_link frame
  /*
  geometry_msgs::PoseStamped storePose;
  storePose.header.frame_id = "table_base_link";
  storePose.pose.position.x = .047;
  storePose.pose.position.y = -.191;
  storePose.pose.position.z = .831;
  storePose.pose.orientation.x = .699;
  storePose.pose.orientation.y = -.294;
  storePose.pose.orientation.z = -.620;
  storePose.pose.orientation.w = -.202;
   */

  //move above store position
  ss.str("");
  ss << "Moving gripper above store position...";
  feedback.message = ss.str();
  storeServer.publishFeedback(feedback);

  rail_manipulation_msgs::MoveToPoseGoal approachAngleGoal;
  approachAngleGoal.pose = goal->store_pose;
  approachAngleGoal.pose.pose.position.z += .1;
  approachAngleGoal.planningTime = 3.0;
  moveToPoseClient.sendGoal(approachAngleGoal);
  moveToPoseClient.waitForResult(ros::Duration(30.0));
  if (!moveToPoseClient.getResult()->success)
  {
    ss.str("");
    ss << "Moving gripper above store position failed.";
    feedback.message = ss.str();
    storeServer.publishFeedback(feedback);
    result.success = false;
    storeServer.setAborted(result, "Unable to move gripper above store position.");
    return;
  }

  //lower gripper
  ss.str("");
  ss << "Lowering gripper...";
  feedback.message = ss.str();
  storeServer.publishFeedback(feedback);

  rail_manipulation_msgs::CartesianPath srv;
  geometry_msgs::PoseStamped cartesianPose;
  approachAngleGoal.pose.pose.position.z -= .1;
  cartesianPose.header.frame_id = "table_base_link";
  tfListener.transformPose("table_base_link", approachAngleGoal.pose, cartesianPose);
  srv.request.waypoints.push_back(cartesianPose.pose);
  srv.request.avoidCollisions = false;
  if (!cartesianPathClient.call(srv))
  {
    ROS_INFO("Could not call Jaco Cartesian path service.");
    result.success = false;
    storeServer.setAborted(result, "Could not call Jaco Cartesian path service.");
    return;
  }

  //open gripper
  ss.str("");
  ss << "Opening gripper...";
  feedback.message = ss.str();
  storeServer.publishFeedback(feedback);

  rail_manipulation_msgs::GripperGoal gripperGoal;
  gripperGoal.close = false;
  gripperClient.sendGoal(gripperGoal);
  gripperClient.waitForResult(ros::Duration(10.0));
  if (!gripperClient.getResult()->success)
  {
    ROS_INFO("Opening gripper failed.");
    result.success = false;
    storeServer.setAborted(result, "Unable to open gripper.");
    return;
  }

  //close gripper
  ss.str("");
  ss << "Closing gripper to check grasp...";
  feedback.message = ss.str();
  storeServer.publishFeedback(feedback);

  gripperGoal.close = true;
  gripperGoal.force = 100;
  gripperClient.sendGoal(gripperGoal);
  gripperClient.waitForResult(ros::Duration(10.0));
  if (!gripperClient.getResult()->success)
  {
    ROS_INFO("Closing gripper failed.");
    result.success = false;
    storeServer.setAborted(result, "Unable to close gripper.");
    return;
  }

  ROS_INFO("Done closing");

  //verify that the object was dropped
  ss << "Checking if the object was stored...";
  ROS_INFO("Checking if the object was stored...");
  feedback.message = ss.str();
  storeServer.publishFeedback(feedback);

  rail_manipulation_msgs::VerifyGraspGoal verifyGoal;
  verifyClient.sendGoal(verifyGoal);
  verifyClient.waitForResult(ros::Duration(10.0));

  ROS_INFO("First verify is %d, %d", verifyClient.getResult()->success, verifyClient.getResult()->grasping);

  if (verifyClient.getResult()->success && verifyClient.getResult()->grasping)
  {
    int attempts = 3;
    bool dropSuccessful = false;

    while(attempts > 0 && !dropSuccessful)
    {
      ROS_INFO("Storing object failed. Retrying...");
      ss << "Storing object failed. Retrying...";
      feedback.message = ss.str();
      storeServer.publishFeedback(feedback);

      // Open gripper again
      gripperGoal.close = false;
      gripperClient.sendGoal(gripperGoal);
      gripperClient.waitForResult(ros::Duration(10.0));
      if (!gripperClient.getResult()->success)
      {
        ROS_INFO("Opening gripper failed.");
        result.success = false;
        storeServer.setAborted(result, "Unable to open gripper.");
        return;
      }

      // Rotate wrist 180 degrees to try and free stuck objects

      // Get current joint positions
      wpi_jaco_msgs::GetAngularPosition::Request req;
      wpi_jaco_msgs::GetAngularPosition::Response res;
      if (!jacoPosClient.call(req, res))
      {
        ROS_INFO("Could not call Jaco joint position service.");
        result.success = false;
        storeServer.setAborted(result, "Unable to retry store action.");
        return;
      }

      wpi_jaco_msgs::AngularCommand cmd;
      cmd.position = true;
      cmd.armCommand = true;
      cmd.fingerCommand = false;
      cmd.repeat = false;
      cmd.joints.resize(res.pos.size());
      for (unsigned int i = 0; i < res.pos.size(); i++)
      {
        // rotate wrist by 180 degrees
        if (i == 5)
        {
          cmd.joints[i] = simplify_angle(res.pos[i] + M_PI);
        }
        else
        {
          cmd.joints[i] = res.pos[i];
        }
      }
      angularCmdPublisher.publish(cmd);

      ros::Duration(5.0).sleep();

      // Open gripper again
      gripperGoal.close = false;
      gripperClient.sendGoal(gripperGoal);
      gripperClient.waitForResult(ros::Duration(10.0));
      if (!gripperClient.getResult()->success)
      {
        ROS_INFO("Opening gripper failed.");
        result.success = false;
        storeServer.setAborted(result, "Unable to open gripper.");
        return;
      }

      //close gripper to verify
      ss.str("");
      ss << "Closing gripper...";
      feedback.message = ss.str();
      storeServer.publishFeedback(feedback);

      rail_manipulation_msgs::GripperGoal gripperGoal;
      gripperGoal.close = true;
      gripperClient.sendGoal(gripperGoal);
      gripperClient.waitForResult(ros::Duration(10.0));
      if (!gripperClient.getResult()->success)
      {
        ROS_INFO("Closing gripper failed.");
        result.success = false;
        storeServer.setAborted(result, "Unable to open gripper.");
        return;
      }

      //verify that the object was dropped
      ss << "Checking if the object was stored...";
      ROS_INFO("Checking if the object was stored...");
      feedback.message = ss.str();
      storeServer.publishFeedback(feedback);

      verifyClient.sendGoal(verifyGoal);
      verifyClient.waitForResult(ros::Duration(10.0));

      dropSuccessful = (verifyClient.getResult()->success && !verifyClient.getResult()->grasping);
      attempts--;
    }

    if (attempts == 0 && !dropSuccessful)
    {
      ROS_INFO("Store failed.");
      result.success = false;
      storeServer.setAborted(result, "Unable to store object.");
      return;
    }

    ROS_INFO("Object successfully stored.");
    ss << "Object successfully stored.";
  }
  else
  {
    ROS_INFO("Object successfully stored.");
    ss << "Object successfully stored.";
    feedback.message = ss.str();
    storeServer.publishFeedback(feedback);
  }

  //detach any attached collision objects
  std_srvs::Empty emptySrv;
  if (!detachObjectsClient.call(emptySrv))
  {
    ROS_INFO("Couldn't call detach objects service.");
  }

  //raise gripper
  ss.str("");
  ss << "Raising gripper...";
  feedback.message = ss.str();
  storeServer.publishFeedback(feedback);

  cartesianPose.pose.position.z += .05;
  srv.request.waypoints.clear();
  srv.request.waypoints.push_back(cartesianPose.pose);
  srv.request.avoidCollisions = false;
  if (!cartesianPathClient.call(srv))
  {
    ROS_INFO("Could not call Jaco Cartesian path service.");
    result.success = false;
    storeServer.setAborted(result, "Could not call Jaco Cartesian path service.");
    return;
  }

  result.success = true;
  storeServer.setSucceeded(result);
}

void CommonActions::executeArmAction(const rail_manipulation_msgs::ArmGoalConstPtr &goal)
{
  rail_manipulation_msgs::ArmFeedback feedback;
  rail_manipulation_msgs::ArmResult result;

  switch (goal->action) {
    case rail_manipulation_msgs::ArmGoal::READY:
      feedback.message = "Readying arm...";
      armServer.publishFeedback(feedback);
    break;
    case rail_manipulation_msgs::ArmGoal::RETRACT:
      feedback.message = "Retracting arm...";
      armServer.publishFeedback(feedback);
    break;
    default:
      feedback.message = "Executing arm action...";
      armServer.publishFeedback(feedback);
    break;
  }

  if (goal->action == rail_manipulation_msgs::ArmGoal::READY)
  {
    if (isArmAtPosition(homePosition))
    {
      feedback.message = "Arm is already ready.";
      armServer.publishFeedback(feedback);

      result.success = true;
      armServer.setSucceeded(result);
      return;
    }
  }
  else if (goal->action == rail_manipulation_msgs::ArmGoal::RETRACT)
  {
    if (isArmAtPosition(defaultRetractPosition))
    {
      feedback.message = "Arm is already retracted.";
      armServer.publishFeedback(feedback);

      result.success = true;
      armServer.setSucceeded(result);
      return;
    }
  }

  rail_manipulation_msgs::MoveToJointPoseGoal jointPoseGoal;

  vector<float> baseJointPoseGoal;
  if (goal->action == rail_manipulation_msgs::ArmGoal::READY)
  {
    baseJointPoseGoal.resize(homePosition.size());
    for (unsigned int i = 0; i < baseJointPoseGoal.size(); i ++)
    {
      baseJointPoseGoal[i] = homePosition[i];
    }
  }
  else if (goal->action == rail_manipulation_msgs::ArmGoal::RETRACT)
  {
    baseJointPoseGoal.resize(defaultRetractPosition.size());
    for (unsigned int i = 0; i < baseJointPoseGoal.size(); i ++)
    {
      baseJointPoseGoal[i] = defaultRetractPosition[i];
    }
  }
  jointPoseGoal.joints.resize(baseJointPoseGoal.size());
  for (unsigned int i = 0; i < jointPoseGoal.joints.size(); i ++)
  {
    jointPoseGoal.joints[i] = baseJointPoseGoal[i];
  }
  bool succeeded = false;
  int counter = 0;
  int attempts = MAX_MOVE_ATTEMPTS;
  while (!succeeded && counter < attempts)
  {
    stringstream ss;
    ss << "Planning and moving arm to requested arm position (attempt " << counter + 1 << "/" << attempts << ")";
    feedback.message = ss.str();
    armServer.publishFeedback(feedback);

    ROS_INFO("Move arm attempt %d", counter);

    moveToJointPoseClient.sendGoal(jointPoseGoal);
    ROS_INFO("Moving arm to given position...");
    while (!moveToJointPoseClient.getState().isDone())
    {
      if (armServer.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("Arm action preempted.");
        moveToJointPoseClient.cancelAllGoals();
        result.success = false;
        armServer.setPreempted(result);
        return;
      }
    }

    rail_manipulation_msgs::MoveToJointPoseResultConstPtr readyResult = moveToJointPoseClient.getResult();

    succeeded = readyResult->success;
    counter ++;

    //slightly vary joint goal and retry planning
    if (!succeeded && counter < attempts)
    {
      ROS_INFO("Moving arm failed, resampling goal for another attempt...");
      for (unsigned int i = 0; i < jointPoseGoal.joints.size(); i ++)
      {
        jointPoseGoal.joints[i] = baseJointPoseGoal[i] + (rand() % 700 - 350) / 10000;  //vary by up to ~2 degrees
      }
    }
  }

  if (!succeeded)
  {
    feedback.message = "Failed to perform requested arm action.";
    armServer.publishFeedback(feedback);

    ROS_INFO("Plan and move to requested arm position failed.");
    result.success = false;
    armServer.setSucceeded(result);
    return;
  }

  switch (goal->action)
  {
    case rail_manipulation_msgs::ArmGoal::READY:
      feedback.message = "Ready arm completed.";
      armServer.publishFeedback(feedback);
    break;
    case rail_manipulation_msgs::ArmGoal::RETRACT:
      feedback.message = "Retract arm completed.";
      armServer.publishFeedback(feedback);
    break;
    default:
      feedback.message = "Arm action completed.";
      armServer.publishFeedback(feedback);
    break;
  }

  result.success = succeeded;
  armServer.setSucceeded(result);
}

void CommonActions::liftArm(const rail_manipulation_msgs::LiftGoalConstPtr &goal)
{
  rail_manipulation_msgs::LiftResult result;

  rail_manipulation_msgs::CartesianPath srv;
  tf::StampedTransform currentEefTransform;
  tfListener.waitForTransform("tablebot_ee_link", "table_base_link", ros::Time::now(), ros::Duration(1.0));
  tfListener.lookupTransform("table_base_link", "tablebot_ee_link", ros::Time(0), currentEefTransform);
  geometry_msgs::Pose liftPose;
  liftPose.position.x = currentEefTransform.getOrigin().x();
  liftPose.position.y = currentEefTransform.getOrigin().y();
  liftPose.position.z = currentEefTransform.getOrigin().z() + .1;
  liftPose.orientation.x = currentEefTransform.getRotation().x();
  liftPose.orientation.y = currentEefTransform.getRotation().y();
  liftPose.orientation.z = currentEefTransform.getRotation().z();
  liftPose.orientation.w = currentEefTransform.getRotation().w();
  srv.request.waypoints.push_back(liftPose);
  srv.request.avoidCollisions = false;

  if (!cartesianPathClient.call(srv))
  {
    ROS_INFO("Could not call Jaco Cartesian path service.");
    result.success = false;
    liftServer.setAborted(result, "Could not call Jaco Cartesian path service.");
    return;
  }

  result.success = srv.response.success;
  liftServer.setSucceeded(result);
}

bool CommonActions::isArmAtPosition(const vector<float> &pos)
{
  float dstFromRetract = 0;

  //get joint positions
  wpi_jaco_msgs::GetAngularPosition::Request req;
  wpi_jaco_msgs::GetAngularPosition::Response res;
  if(!jacoPosClient.call(req, res))
  {
    ROS_INFO("Could not call Jaco joint position service.");
    return false;
  }

  for (unsigned int i = 0; i < pos.size(); i ++)
  {
    dstFromRetract += fabs(pos[i] - res.pos[i]);
  }

  if (dstFromRetract > 0.175)
    return false;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "common_actions");

  CommonActions ca;

  ros::spin();
}
