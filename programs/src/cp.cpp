#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string.h>

// To spawn Stl files in moveit env
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

//Global Variables
moveit_msgs::AttachedCollisionObject attached_objects[6]; 

double ObjectCoordinates[6][7] = {
  {-0.23, -0.16, 0.36,  0.707388, -0.706825, 0.0005629, 0.0005633}, {-0.430, -0.13875, 0.4646,  0, 0, 0, 1},
  {-0.27, 0.23, 0.09,  0, 0, 0.7071068, -0.7071068}, {-0.23, -0.13, 0.1,  0, 0, 0,1}, 
  {-0.27, -0.305, 0.08,  0, 0, -0.7071068, 0.7071068},     {-0.31, -0.27, 0.16,  0, 0, -0.7071068, 0.7071068}};
  
std::string ObjectNames[6] = {"PlungerPlate","PressureLid","output_plate","filter_plate", "Waste_plate", "Sample_rack"};





//function for moving robot
void movebot(moveit::planning_interface::MoveGroupInterface& move_group,
  double x,double y,double z,double rx,double ry,double rz,double rq)
{

    //move_group.setPlannerId("PRM");
    //move_group.setPlanningTime(15);
    //move_group.setNumPlanningAttempts(100);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group.getCurrentPose("gripper");
    geometry_msgs::Pose target_pose1;


    target_pose1.position.x = x;
    target_pose1.position.y = y;
    target_pose1.position.z = z;
    target_pose1.orientation.x = rx;
    target_pose1.orientation.y = ry;
    target_pose1.orientation.z = rz;
    target_pose1.orientation.w = rq;

    move_group.setPoseTarget(target_pose1);
    bool success = (move_group.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ros::WallDuration(1).sleep();
    
    ROS_INFO_NAMED("tutorial", "Path Planning %s", success ? "" : "FAILED");
    ros::WallDuration(1).sleep();
    move_group.move();
    ros::WallDuration(1).sleep();
}



//Function for operating gripper
void gripper(moveit::planning_interface::MoveGroupInterface& grip, std::string status)
{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  grip.setJointValueTarget(grip.getNamedTargetValues(status));
  ROS_INFO("Gripper Target Value Reached");
  grip.move();

  ros::WallDuration(1).sleep();
    
}


void addobj(ros::Publisher& diff, moveit_msgs::PlanningScene& planning_scene)
{

  shape_msgs::Mesh mesh[6];
  shapes::ShapeMsg mesh_msg[6];
  shapes::Mesh* m[6]; 
    
  for (int i=0; i<6;i++)
  {
    std::string filename = ObjectNames[i];
    std::string path ="package://withgripper/objects/";
    std::string extension = ".stl";
    std::string pathname= path+ObjectNames[i]+extension;
    

    attached_objects[i].object.header.frame_id = "world";
    attached_objects[i].object.id = ObjectNames[i];

    Eigen::Vector3d b(1, 1, 1);
    m[i] = shapes::createMeshFromResource(pathname, b);
    shapes::constructMsgFromShape(m[i],mesh_msg[i]);
    mesh[i] = boost::get<shape_msgs::Mesh>(mesh_msg[i]);

    geometry_msgs::Pose pose;

    pose.position.x = ObjectCoordinates[i][0];
    pose.position.y = ObjectCoordinates[i][1];
    pose.position.z = ObjectCoordinates[i][2];
    pose.orientation.x = ObjectCoordinates[i][3];
    pose.orientation.y = ObjectCoordinates[i][4];
    pose.orientation.z = ObjectCoordinates[i][5];
    pose.orientation.w = ObjectCoordinates[i][6];

    attached_objects[i].object.meshes.push_back(mesh[i]);
    attached_objects[i].object.mesh_poses.push_back(pose);
    attached_objects[i].object.operation = attached_objects[i].object.ADD;

    planning_scene.world.collision_objects.push_back(attached_objects[i].object);
    planning_scene.is_diff = true;
    diff.publish(planning_scene);
  
  }
  

  ROS_INFO("All Object Added");
 

  
}

void attachobj(ros::Publisher& diff, moveit_msgs::PlanningScene& planning_scene, int ObjectId)
{
  

  attached_objects[ObjectId].link_name = "gripper";
  attached_objects[ObjectId].touch_links = std::vector<std::string>{ "clawl","clawr","gripper" };
  
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = ObjectNames[ObjectId];
  remove_object.header.frame_id = "world";
  remove_object.operation = remove_object.REMOVE;

  ROS_INFO("Attaching the object to the hand and removing it from the world.");
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene.robot_state.attached_collision_objects.push_back(attached_objects[ObjectId]);
  planning_scene.robot_state.is_diff = true;
  diff.publish(planning_scene);
}

void detachobj(ros::Publisher& diff, moveit_msgs::PlanningScene& planning_scene, int ObjectId)
{
  moveit_msgs::AttachedCollisionObject detach;
  detach.object.id = ObjectNames[ObjectId];
  detach.link_name = "clawl";
  detach.object.operation = attached_objects[ObjectId].object.REMOVE;
  
  ROS_INFO("detach object from robot frame");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach);
  planning_scene.world.collision_objects.clear();
  planning_scene.robot_state.is_diff = true;
  diff.publish(planning_scene);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle n;
  ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene",1);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(planning_scene_diff_publisher.getNumSubscribers()<1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP_ARM = "robot";
  static const std::string PLANNING_GROUP_GRIPPER = "hand";
  static const std::string PLANNING_GROUP_ROBOT = "robot_hand";
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
  moveit::planning_interface::MoveGroupInterface move_group_interface_robot(PLANNING_GROUP_ROBOT);
  moveit_msgs::PlanningScene ps;



  addobj(planning_scene_diff_publisher, ps);  
  
  //gripper(move_group_interface_gripper, "open");
  
  //movebot(move_group_interface_arm, -0.36625, 0.15825, 0.508, 0.5, -0.5, 0.5, -0.5);
  //movebot(move_group_interface_arm, -0.3755, 0.15825, 0.4845, 0.5, -0.5, 0.5, -0.5);
  //attachobj(planning_scene_diff_publisher,ps,0);
  //gripper(move_group_interface_gripper, "close");
  //movebot(move_group_interface_arm, -0.3755, 0.15825, 0.508, 0.5, -0.5, 0.5, -0.5);
  //move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
  //move_group_interface_arm.move();
  //movebot(move_group_interface_arm, -0.22972, -0.211, 0.348506, 0, 0.7071068, 0.7071068, 0);
  

  ros::WallDuration(1).sleep();

  //movebot(move_group_interface_arm, -0.22881, -0.20941,0.180, 0, 0.7071068, 0.7071068, 0);
  //movebot(move_group_interface_arm, -0.22881, -0.20941,0.173, 0, 0.7071068, 0.7071068, 0);
  //detachobj(planning_scene_diff_publisher,ps, 0);

  ros::WallDuration(1).sleep();

  ros::shutdown();
  return 0;
}
/*
cd ros/mammacheck/
source devel/setup.bash
killall -9 rosmaster; killall -9 rviz; killall gzserver; killall gzclient
*/