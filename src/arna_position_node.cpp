/*
 * Author: Das Summit
 * Co Author: Jordan Dowdy, RE2
 * Date: 17/08/22
 * Version: 2.0
 */


#include "ros/ros.h"
#include <iostream>
#include <string>

// ROS messages
#include "sensor_msgs/JointState.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "actionlib/client/simple_action_client.h"
#include "hebiros/TrajectoryAction.h"
#include "sensor_msgs/Joy.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl_conversions/kdl_msg.h>
#include "kdl/frames.hpp"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf_conversions/tf_kdl.h>
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include <kdl/frames_io.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

//Eigen
#include <Eigen/Core>
#include <Eigen/LU>

using namespace hebiros;
#define NUMBER_OF_JOINTS    6

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef TWO_REV
#define TWO_REV (4.0 * M_PI)
#endif

const int NUM_OF_JOINTS {6};
sensor_msgs::JointState jointFeedback_g;
sensor_msgs::Joy joyCommandMsg;
sensor_msgs::JointState joint_command_msg;
static bool receivedjointFeedback_g = false;
static bool achievedJointGoal_g = false;

 
float voltageOne {0.0}; //+z^
float voltageTwo {0.0};//-z^
float voltageThree {0.0}; //-y^
float voltageFour {0.0}; //+y^
KDL::Vector tfForceOne, tfForceTwo, tfForceThree,tfForceFour;
const float GAIN {0.00045}; //temp



// does transformations when we get voltage from command feedback
void transform_data(tf::TransformListener &listener, tf::StampedTransform &lk2base_transform ){

    //ROS_INFO("Function Called");
    // our voltage needs to be turned into a vector to describe the direction of force

    // if you notice that the vector doesnt match the axis this is because
    // the stepper is already rotated compared by 90 degrees. This makes it so
    // the actual sensor controls the correct axis.
    KDL::Vector localForceOne (0, voltageOne, 0);
    KDL::Vector localForceTwo (0, voltageTwo, 0); //this one was wired backwords so its number is already negative.
    KDL::Vector localForceThree (0,0, -voltageThree); 
    KDL::Vector localForceFour (0, 0, voltageFour);
    //ROS_INFO("Vector Created");
    KDL::Rotation rot;
    //ROS_INFO("Matrix Created");
    // generates the rotation matrix from frame link_6 (position of sensors) to refrance frame base_link
    tf::quaternionTFToKDL(lk2base_transform.getRotation(),rot);
    //ROS_INFO("Matrix Populated");
    //returns a new 1x3 matrix that is our localForce# vector rotated
    tfForceOne = (rot*localForceOne);
    tfForceTwo = (rot*localForceTwo);
    tfForceThree = (rot*localForceThree);
    tfForceFour = (rot*localForceFour);
    //ROS_INFO("Matrix Multiplication Completed");
    float effortJointOne = GAIN * (/*tfForceOne(1) + tfForceTwo(1) + */ tfForceThree(1) + tfForceFour(1));
    float effortJointTwo = GAIN * (/*tfForceOne(2) + tfForceTwo(1) + */ tfForceThree(2) + tfForceFour(2));
    float rollJointOne = GAIN *1.5* voltageOne;
    float rollJointTwo = GAIN *1.5 * voltageTwo;
    //ROS_INFO("Effort Calculated");
    //std::cout << "Force" << tfForceOne << ", " << tfForceFour << std::endl;
    //std::cout << "Effort" << effortJointOne << ", " << effortJointTwo << std::endl;
    
    if (jointFeedback_g.position[0] > -12.567 && jointFeedback_g.position[0] < 12.567)
    {    
        //joint_command_msg.effort[0] =  effortJointOne;
        joint_command_msg.position[0] = joint_command_msg.position[0] + effortJointOne;
        

    }
    
    if (jointFeedback_g.position[1] > -3.49066 && jointFeedback_g.position[1] < 0.785)
    {
        //joint_command_msg.effort[1] = effortJointOne;
        joint_command_msg.position[1] = joint_command_msg.position[1] + effortJointTwo;
        //ROS_INFO("Effort Assigned");
    }
    
    if (jointFeedback_g.position[4] > -12.567 && jointFeedback_g.position[4] < 12.567)
    {    
        //joint_command_msg.effort[0] =  effortJointOne;
        joint_command_msg.position[4] = joint_command_msg.position[4] + rollJointOne + rollJointTwo;
        

    }
    

    /*
     *
     * This is for cartisian space and not joint space
    KDL::ChainIkSolverVel_pinv vel_ik_solver(re2_chain, 0.0001, 1000);
    KDL::ChainIkSolverPos_NR ik_solver(ur5_chain, fk_solver, vel_ik_solver, 1000);

    KDL::Frame goal_pose(rot, tfForce1);

    KLD::JntArray init_joint_pose (jointFeedback_g.position[0],jointFeedback_g.position[1],jointFeedback_g.position[2],
                                   jointFeedback_g.position[3],jointFeedback_g.position[4],jointFeedback_g.position[5]);

    KDL::JntArray jnt_pose_goal(num_joints);
    ik_solver.CartToJnt(init_jnt_pose, goal_pose, jnt_pose_goal);

    joint_command_msg.position[0] = jnt_pose_goal[0];
    joint_command_msg.position[1] = jnt_pose_goal[1];
    joint_command_msg.position[2] = jnt_pose_goal[2];
    joint_command_msg.position[3] = jnt_pose_goal[3];
    joint_command_msg.position[4] = jnt_pose_goal[4];
    joint_command_msg.position[5] = jnt_pose_goal[5];

    */

}

// Callback when we get joint state feedback
void joint_callback(
        sensor_msgs::JointState data)
{
    receivedjointFeedback_g = true;
    jointFeedback_g = data;

}


// Command Callback
void command_callback(
        sensor_msgs::Joy data )
{
    voltageOne = data.axes[0] *5/1024;
    voltageTwo = data.axes[1] *5/1024;
    voltageThree = data.axes[2] *5/1024;
    voltageFour = data.axes[3] *5/1024;
    std::cout << data.axes[0] << std::endl;
    std::cout << voltageOne << ", " << voltageTwo << ", " << voltageThree << ", " << voltageFour << ", " << std::endl; 
    /*
	ROS_INFO(
        "Joints [0]:%f [1]:%f [2]:%f [3]:%f [4]:%f [5]:%f",
        data.axes[0],
        data.axes[1],
        data.axes[2],
        data.axes[3],
        data.axes[4],
        data.axes[5] );

    joint_command_msg.position[0] = data.axes[0] * M_PI;
    joint_command_msg.position[1] = data.axes[1] * M_PI;
    joint_command_msg.position[2] = data.axes[3] * M_PI;
    joint_command_msg.position[3] = data.axes[4] * M_PI;
    joint_command_msg.position[4] = joint_command_msg.position[4];
    joint_command_msg.position[5] = joint_command_msg.position[5];
    */


}



// Callback when trajectory action goal becomes active
void trajectory_active_callback()
{
    ROS_INFO(
            "Goal just went active" );
}


// Callback when feedback is received for the trajectory action goal
void trajectory_feedback_callback(
        const TrajectoryFeedbackConstPtr& feedback )
{
    ROS_INFO(
            "Trajectory percent completion: %f",
            feedback->percent_complete );
}


//Callback which is called once when the trajectory action goal completes
void trajectory_done_callback(
        const actionlib::SimpleClientGoalState& state,
        const TrajectoryResultConstPtr& result )
{
    ROS_INFO(
            "Final state: %s",
            state.toString().c_str() );

    for (int i = 0; i < result->final_state.name.size(); i++)
    {
        ROS_INFO(
                "\n%s:\n  Position:%f\n  Velocity%f\n  Effort:%f",
                result->final_state.name[i].c_str(),
                result->final_state.position[i],
                result->final_state.velocity[i],
                result->final_state.effort[i] );
    }

    achievedJointGoal_g = true;
}


// Format the goal information properly for HEBI's interface
void construct_goal(
        TrajectoryGoal& goal,
        int num_joints,
        int num_waypoints,
        std::vector<double> times,
        std::vector<std::string> names,
        std::vector<std::vector<double>> positions,
        std::vector<std::vector<double>> velocities,
        std::vector<std::vector<double>> accelerations )
{
    goal.times.resize( num_waypoints );
    goal.waypoints.resize( num_waypoints );

    WaypointMsg waypoint;
    waypoint.names.resize( num_joints );
    waypoint.positions.resize( num_joints );
    waypoint.velocities.resize( num_joints );
    waypoint.accelerations.resize( num_joints );

    //Construct the goal using the TrajectoryGoal format
    for (int i = 0; i < num_waypoints; i++)
    {
        for (int j = 0; j < num_joints; j++)
        {
            waypoint.names[j] = names[j];
            waypoint.positions[j] = positions[j][i];
            waypoint.velocities[j] = velocities[j][i];
            waypoint.accelerations[j] = accelerations[j][i];
        }
        goal.times[i] = times[i];
        goal.waypoints[i] = waypoint;
    }
}



int main( int argc, char **argv )
{
    // Setup -----------------------------------------------------------------

    ros::init( argc, argv, "arna_position_example" );
    ros::NodeHandle n;
    ros::Rate loop_rate( 100 ); // Hz
    
    // Note: Do not command the gripper faster than 20 Hz.
    // If you do, it will behave erratically.
    ros::Rate gripperLoop( 20 ); // Hz

 
    // Probably don't need a full 5 seconds, but need some delay here.
    ROS_WARN("Sleeping for 5 seconds for HEBI node and joint_state_pub to fully boot up");
    ros::Duration(5.0).sleep();
    
    tf::TransformListener listener;
    tf::StampedTransform lk2base_transform;
    

    /*
     *
     * do not need if using joint space (what is currently in use)
    KDL::Tree  re2_tree;
    if(!kdl_parser::treeFromFile("octocandemo_ws/src/arna_example/arna.urdf", re2_tree)){
        ROS_ERROR("URDF could not be parsed");
    }


    KDL::Chain re2_chain;
    re2_tree.getChain("ARNA/Shoulder_Yaw", "ARNA/Wrist_Pitch", re2_chain);

     */
	

    

    // Use this group name for all of our HEBI topics and services.
    std::string group_name = "arna_manipulator";

    // Create a joint state feedback subscriber
    ros::Subscriber feedback_subscriber = n.subscribe(
            "/hebiros/" + group_name + "/feedback/joint_state",
            100, // Buffer up to 100 messages
            joint_callback);
    
    jointFeedback_g.position.reserve( NUMBER_OF_JOINTS ); // Expect position reports
    // Use the add_group_from_names service to create a group
    ros::ServiceClient add_group_client =
            n.serviceClient<AddGroupFromNamesSrv>( "/hebiros/add_group_from_names" );
    bool groupCreated = false;
    AddGroupFromNamesSrv add_group_srv;
    add_group_srv.request.group_name = group_name;
    // These are the names that the joints should have.
    add_group_srv.request.families = {"ARNA"};
    add_group_srv.request.names = {
            "Shoulder_Yaw",
            "Shoulder_Pitch",
            "Upper_Arm_Roll",
            "Elbow_Pitch",
            "Wrist_Roll",
            "Wrist_Pitch" };
    groupCreated = add_group_client.call( add_group_srv );
    if ( true == groupCreated )
    {
        // Specific topics and services will now be available under this group's namespace
        ROS_INFO( "Created group: %s", group_name.c_str() );
    }
    else
    {
        ROS_FATAL( "Failed to create group: %s", group_name.c_str() );
        return -1;
    }

    // Now that there's a group, it should be getting feedback.  We're going
    // to want that feedback for our starting point of our trajectory.
    while( ros::ok() && (false == receivedjointFeedback_g) )
    {
        ROS_INFO( "Waiting for joint feedback" );
        ros::spinOnce();
        loop_rate.sleep();
    }


    // Create an action client for executing a trajectory
    actionlib::SimpleActionClient<TrajectoryAction> trajectoryClient(
            "/hebiros/" + group_name + "/trajectory",
            true );

    //Wait for the action server corresponding to the action client
    trajectoryClient.waitForServer();

    // First, let's send everything to zero position. ------------------------
    double distanceToTarget = 0.0;
    while( ros::ok() && (distanceToTarget > 0.05) );

    // For the joints, construct a trajectory to be sent as an action goal.
    // The HEBI joints will accept non-trajectory position commands, but the
    // motion is pretty abrupt.  The trajectory interface creates
    // much smoother motion.

    // Set the times to reach each waypoint in seconds
    std::vector<double> times = {0, 5};

    std::vector<std::string> names = {
            "ARNA/Shoulder_Yaw",
            "ARNA/Shoulder_Pitch",
            "ARNA/Upper_Arm_Roll",
            "ARNA/Elbow_Pitch",
            "ARNA/Wrist_Roll",
            "ARNA/Wrist_Pitch" };

    // Set positions, velocities, and accelerations for each waypoint and each joint
    // The following vectors have one joint per row and one waypoint per column
    std::vector<std::vector<double>> positions =
            {{jointFeedback_g.position[0], 0},
             {jointFeedback_g.position[1], 0},
             {jointFeedback_g.position[2], 0},
             {jointFeedback_g.position[3], 0},
             {jointFeedback_g.position[4], 0},
             {jointFeedback_g.position[5], 0}};
    std::vector<std::vector<double>> velocities =
            {{0, 0},
             {0, 0},
             {0, 0},
             {0, 0},
             {0, 0},
             {0, 0}};
    std::vector<std::vector<double>> accelerations =
            {{0, 0},
             {0, 0},
             {0, 0},
             {0, 0},
             {0, 0},
             {0, 0}};

    // Send the goal, executing the trajectory
    TrajectoryGoal goal;
    construct_goal(
            goal,
            NUMBER_OF_JOINTS,
            2,
            times,
            names,
            positions,
            velocities,
            accelerations );
    trajectoryClient.sendGoal(
            goal,
            &trajectory_done_callback,
            &trajectory_active_callback,
            &trajectory_feedback_callback );

    while( ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();

        if ( true == achievedJointGoal_g )
        {
            achievedJointGoal_g = false;
            break;
        }
    }
    
    // Once the arm is at its goal position, we have two options.  We can
    // move on to the next trajectory, or hold position.  To hold position, we
    // need to keep periodically commanding the arm to a position, or it will
    // time out and slump.  We're going to be moving the gripper around next,
    // so let's command the arm to hold a position, then give it another
    // trajectory later.

    // Create a joint state command publisher
    ros::Publisher joint_command_publisher = n.advertise<sensor_msgs::JointState>(
            "/hebiros/" + group_name + "/command/joint_state",
            10 ); // Buffer up to 10 messages

    // Create a JointState that we'll use to command the HEBI modules

    joint_command_msg.name.push_back( "ARNA/Shoulder_Yaw" );   // [0]
    joint_command_msg.name.push_back( "ARNA/Shoulder_Pitch" ); // [1]
    joint_command_msg.name.push_back( "ARNA/Upper_Arm_Roll" ); // [2]
    joint_command_msg.name.push_back( "ARNA/Elbow_Pitch" );    // [3]
    joint_command_msg.name.push_back( "ARNA/Wrist_Roll" );     // [4]
    joint_command_msg.name.push_back( "ARNA/Wrist_Pitch" );    // [5]

    joint_command_msg.position.resize( NUMBER_OF_JOINTS ); // Use position commands

    joint_command_msg.position[0] = jointFeedback_g.position[0];
    joint_command_msg.position[1] = jointFeedback_g.position[1];
    joint_command_msg.position[2] = jointFeedback_g.position[2];
    joint_command_msg.position[3] = jointFeedback_g.position[3];
    joint_command_msg.position[4] = jointFeedback_g.position[4];
    joint_command_msg.position[5] = jointFeedback_g.position[5];

    //Subscriber for External Msgs
    ros::Subscriber control_command_subscriber = n.subscribe("/joy", 100, command_callback);


    // Now let's send each joint to its limits then back to zero. ------------

    // Note, it's possible to move the gripper and the joints at the same
    // time.  We're just moving them one at a time here for simplicity's sake.
    // You can command the joints at 200 Hz, but remember not to command the
    // gripper any faster than 20 Hz.


    // Now that we're done sending the gripper through its range of motion
    // we can construct a trajectory to send the arm through its
    // range of motion.
    std::vector<double> times2 = {
            0, 20, 40, 60, 80, 100, 120, 140, 160, 180,
            200, 220, 240, 260, 280, 300, 320, 340, 360 };

    std::vector<std::string> names2 = {
            "ARNA/Shoulder_Yaw",
            "ARNA/Shoulder_Pitch",
            "ARNA/Upper_Arm_Roll",
            "ARNA/Elbow_Pitch",
            "ARNA/Wrist_Roll",
            "ARNA/Wrist_Pitch" };

    // Set positions, velocities, and accelerations for each waypoint and each joint.
    // The following vectors have one joint per row and one waypoint per column.
    // Use nan to indicate "don't care" for a given value in the HEBI interface.
    // The roll joints are all continuous, so just put them through 2 revolutions in each direction.
    // The other limits are from arna_example/urdf/arna.urdf


    double nan = std::numeric_limits<float>::quiet_NaN();
    std::vector<std::vector<double>> positions2 =
            {{jointFeedback_g.position[0], -TWO_REV, TWO_REV, 0.0,        0.0,      0.0, 0.0,      0.0,     0.0, 0.0,       0.0,      0.0, 0.0,      0.0,     0.0, 0.0,       0.0,      0.0, 0.0},
             {jointFeedback_g.position[1],   1.5708,  1.5708, 0.0,  -0.785398,  3.49066, 0.0,      0.0,     0.0, 0.0,       0.0,      0.0, 0.0,      0.0,     0.0, 0.0,       0.0,      0.0, 0.0},
             {jointFeedback_g.position[2],      0.0,     0.0, 0.0,        0.0,      0.0, 0.0, -TWO_REV, TWO_REV, 0.0,       0.0,      0.0, 0.0,      0.0,     0.0, 0.0,       0.0,      0.0, 0.0},
             {jointFeedback_g.position[3],      0.0,     0.0, 0.0,   1.570796,-1.570796, 0.0,      0.0,     0.0, 0.0, -1.570796, 1.570796, 0.0,      0.0,     0.0, 0.0,       0.0,      0.0, 0.0},
             {jointFeedback_g.position[4],      0.0,     0.0, 0.0,        0.0,      0.0, 0.0,      0.0,     0.0, 0.0,       0.0,      0.0, 0.0, -TWO_REV, TWO_REV, 0.0,       0.0,      0.0, 0.0},
             {jointFeedback_g.position[5],      0.0,     0.0, 0.0,        0.0,      0.0, 0.0,      0.0,     0.0, 0.0,  1.570796,-1.570796, 0.0,      0.0,     0.0, 0.0, -1.570796, 1.570796, 0.0}};
    std::vector<std::vector<double>> velocities2 =
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::vector<std::vector<double>> accelerations2 =
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};


    

    // Send the goal, executing the trajectory

    construct_goal(
            goal,
            NUMBER_OF_JOINTS,
            19, // number of waypoints
            times2,
            names2,
            positions2,
            velocities2,
            accelerations2 );

    /*trajectoryClient.sendGoal(
        goal,
        &trajectory_done_callback,
        &trajectory_active_callback,
        &trajectory_feedback_callback );*/
    
    try{
	listener.waitForTransform("/base_link", "/link_6",
                                 ros::Time(0), ros::Duration(2.0));
        
        listener.lookupTransform("/base_link", "/link_6",
                                 ros::Time(0), lk2base_transform);
        
        }
        catch (...){
           ROS_INFO("Transform Failed");
	   
        }
   
    while( ros::ok() )
    {
       ros::spinOnce();
       loop_rate.sleep();
       
	try{
	
        listener.lookupTransform("/base_link", "/link_6",
                                 ros::Time(0), lk2base_transform);
        
        }
        catch (...){
           ROS_INFO("Transform Failed");
	   joint_command_publisher.publish(joint_command_msg);
           continue;
        }
        transform_data(listener, lk2base_transform);
       
        //FOR TESTING SENSOR AND STEPPER MOVEMENT
        /*	
        if(i == 120){
           if(j == 0){
              j++;
              voltageOne = 0.0; //+z^ 
              voltageTwo = 0.7;//-z^
              voltageThree = 0.3; //-y^
              voltageFour = 0.0; //+y^
            }
           else{
              j=0;
              voltageOne = 0.7; //+z^
              voltageTwo = 0.0;//-z^
              voltageThree = 0.0; //-y^
              voltageFour = 0.3; //+y^
           }
           i=0;
           }
        }

        */
        joint_command_publisher.publish(joint_command_msg);

        /*if ( true == achievedJointGoal_g )
        {
            // Our example is done.
            ROS_WARN( "End of example.  Shutting down." );
            ros::shutdown();
        }*/
    }

    return 0;
}
