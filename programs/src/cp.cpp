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
moveit_msgs::AttachedCollisionObject attached_objects[2]; 

double ObjectCoordinates[2][7] = {
  {-0.23, -0.16, 0.36,  0.707388, -0.706825, 0.0005629, 0.0005633}, {-0.430, -0.13875, 0.4646,  0, 0, 0, 1}};
  
std::string ObjectNames[2] = {"box1","box2"};





//function for moving robot
void movebot(moveit::planning_interface::MoveGroupInterface& move_group,
  double x,double y,double z,double rx,double ry,double rz,double rq)
{

    //move_group.setPlannerId("PRM");
    //move_group.setPlanningTime(15);
    //move_group.setNumPlanningAttempts(100);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group.getCurrentPose("wrist2");
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



//Function for adding collision objects to the world
void addobj(ros::Publisher& diff, moveit_msgs::PlanningScene& planning_scene)
{

  
  for (int i=0; i<2;i++)
  {

    attached_objects[i].object.header.frame_id = "world";
    attached_objects[i].object.id = ObjectNames[i];

    geometry_msgs::Pose pose;
    pose.position.x = ObjectCoordinates[i][0];
    pose.position.y = ObjectCoordinates[i][1];
    pose.position.z = ObjectCoordinates[i][2];
    pose.orientation.x = ObjectCoordinates[i][3];
    pose.orientation.y = ObjectCoordinates[i][4];
    pose.orientation.z = ObjectCoordinates[i][5];
    pose.orientation.w = ObjectCoordinates[i][6];

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.02;
    primitive.dimensions[1] = 0.02;
    primitive.dimensions[2] = 0.02;

    
    attached_objects[i].object.primitives.push_back(primitive);
    attached_objects[i].object.primitive_poses.push_back(pose);
    attached_objects[i].object.operation = attached_objects[i].object.ADD;

    planning_scene.world.collision_objects.push_back(attached_objects[i].object);
    planning_scene.is_diff = true;
    diff.publish(planning_scene);
  
  }
  

  ROS_INFO("All Object Added");  
}





//Function for attaching collision objects to the robot
void attachobj(ros::Publisher& diff, moveit_msgs::PlanningScene& planning_scene, int ObjectId)
{
  

  attached_objects[ObjectId].link_name = "wrist2";
  attached_objects[ObjectId].touch_links = std::vector<std::string>{ "wrist2" };
  
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


//Function for deattaching collision objects from the robot
void detachobj(ros::Publisher& diff, moveit_msgs::PlanningScene& planning_scene, int ObjectId)
{
  moveit_msgs::AttachedCollisionObject detach;
  detach.object.id = ObjectNames[ObjectId];
  detach.link_name = "wrist2";
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
  static const std::string PLANNING_GROUP_ARM = "mrdummy";
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
  moveit_msgs::PlanningScene ps;



  addobj(planning_scene_diff_publisher, ps);  
  
  
  movebot(move_group_interface_arm, 0.079115, 0.0562584, 0.155379, 0.026604, 0.0127926, 0.43747, 0.898747);
  //attachobj(planning_scene_diff_publisher,ps,0);
  

  ros::WallDuration(1).sleep();

  //detachobj(planning_scene_diff_publisher,ps, 0);

  ros::WallDuration(1).sleep();

  ros::shutdown();
  return 0;
}
