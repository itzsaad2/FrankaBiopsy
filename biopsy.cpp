// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <thread>
#include "ik.h"
#include <chrono>
#include <string>
#include <sstream>
#include "examples_common.h"



// Namespace for robot context - contains pointers to robot, gripper, and model for global access
namespace robotContext {
    franka::Robot *robot;
    franka::Gripper *gripper;
    franka::Model *model;

}

// Global variables for tracking end-effector poses and state
namespace global_variable{
  std::array< double, 16 > current_ee_pose; 
  bool flag_done_collecting; 
  std::vector<std::array< double, 16 >> collected_ee_poses;
}

// Function to apply a transformation to a point in space using rotation and translation
Eigen::Vector3d applyTransformation(const Eigen::Vector3d &point, const Eigen::Matrix3d &R, const Eigen::Vector3d &t) {
  Eigen::Vector3d rotatedPoint = R * point;
  Eigen::Vector3d transformedPoint = rotatedPoint + t;
  return transformedPoint;
}

// Function to convert a 3D point to a homogeneous transformation matrix
Eigen::Matrix4d convertToHomogeneousMatrix(const Eigen::Vector3d &point) {
  
  Eigen::Matrix4d homogeneous_matrix;
  homogeneous_matrix << 1, 0, 0, point(0),
                         0, -1, 0, point(1),
                         0, 0, -1, point(2),
                         0, 0, 0, 1;

  return homogeneous_matrix;
}

// Thread function to record poses. It prompts user input to collect n_poses end-effector poses
std::vector<std::array< double, 16 >> record_pose_thread(int n_poses=3){
  int collected_poses = 0; 
  std::string my_string = "";
  
  while(collected_poses < n_poses){
    std::cout << "Press ENTER to collect current pose, anything else to quit data collection" << std::endl;
    std::getline(std::cin, my_string);
    if(my_string.length()==0){
      global_variable::collected_ee_poses.push_back(global_variable::current_ee_pose);
      std::cout << "Pose:"; 
      std::cout << collected_poses << std::endl;
      std::cout << Eigen::Matrix4d::Map(global_variable::current_ee_pose.data()) << std::endl;
      collected_poses++;
    }
    else{
      std::cout << "Exiting data collection"<<std::endl; 
      global_variable::flag_done_collecting = true; 
      break;
    }
  }
  global_variable::flag_done_collecting = true; 
}

// Function to calculate Euclidean distance between two points
double distance(const Eigen::Vector3d &v1,const Eigen::Vector3d &v2){
  Eigen::Vector3d diff = v1 - v2;
  return diff.norm();
}

// Main function starts here
int main(int argc, char** argv) {
  global_variable::flag_done_collecting = false; 

  std::vector<std::array< double, 16 >> ee_poses; 
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  InverseKinematics ik_controller(1, IKType::M_P_PSEUDO_INVERSE); // Initialize inverse kinematics controller
  try {
     // Setup robot and model
    franka::Robot robot(argv[1]);
    robotContext::robot = &robot;
    franka::Model model = robot.loadModel();
    robotContext::model = &model;

    // Move robot to a starting configuration
    std::array<double, 7> q_goal = {{0, 0, 0, -2 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.25, q_goal);
    robot.control(motion_generator);

    // Prompt for user input to choose operation mode
    std::cout << "Enter 1 for one voxel space pose \nEnter 2 for two voxel space poses (start and target image pose) \nEnter 3 for regular collection method \nEnter 4 for impedence biopsy demo \nEnter 5 for feedback biopsy demo" << std::endl;
    int input;
    std::cin >> input;

    // Definitions for transformation from image space to robot space
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    // Hardcoded transformation matrix and translation vector
    R << -0.99993103,  0.01168232,  0.00120587,
        0.01168708,  0.99992361,  0.00402341,
        -0.00115878,  0.00403722, -0.99999118;

    t << 0.55575213, -0.14195533, -0.00889236;

    // Cases based on user input for different functionalities
    if (input == 1) {

      double x, y, z;
       std::cout << "x coordinates voxel units:";
       std::cin >> x;
       std::cout << "y coordinates voxel units:";
       std::cin >> y;
       std::cout << "z coordinates voxel units:";
       std::cin >> z;

       x = x*0.8/100;
       y = y*0.8/100;
       z = z*0.32/100;

       Eigen::Vector3d imageSpacePoint(x, y, z);

        Eigen::Vector3d transformedPoint = applyTransformation(imageSpacePoint, R, t);

        Eigen::Matrix4d transformedMatrix = convertToHomogeneousMatrix(transformedPoint);
  
        std::array<double, 16> transformedArray; // We need to convert Eigen::Matrix4d to std::array
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(transformedArray.data()) = transformedMatrix;
        global_variable::collected_ee_poses.push_back(transformedArray);


        for (auto &ee_pose: global_variable::collected_ee_poses){
          Eigen::Matrix4d pose = Eigen::Matrix4d::Map(ee_pose.data());
          std::cout<<pose<<"\n"; 

          double time = 0.0;
          robot.control([&time, &pose, &ik_controller](const franka::RobotState& robot_state,
                                              franka::Duration period) -> franka::JointVelocities {
            time += period.toSec();
            franka::JointVelocities output_velocities = ik_controller(robot_state, period, pose);
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> output_eigen_velocities(robot_state.dq.data());
            Eigen::Vector3d current_position(robot_state.O_T_EE[12],robot_state.O_T_EE[13],robot_state.O_T_EE[14]); 
            Eigen::Vector3d desired_position(pose(0,3), pose(1,3), pose(2,3)) ;
            double dist = distance(current_position, desired_position);

            if (time >= 15.0 || (output_eigen_velocities.norm() < 0.0005 && dist < 0.0005) ) {
              output_velocities = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
              return franka::MotionFinished(output_velocities);
            }
            
            return output_velocities;
          });
          std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }

    } if (input == 2) {

      double x_start, y_start, z_start;
      double x_target, y_target, z_target;

      std::cout << "Enter start x coordinate in voxel space: ";
      std::cin >> x_start;
      std::cout << "Enter start y coordinate in voxel space: ";
      std::cin >> y_start;
      std::cout << "Enter start z coordinate in voxel space: ";
      std::cin >> z_start;

      std::cout << "Enter target x coordinate in voxel space: ";
      std::cin >> x_target;
      std::cout << "Enter target y coordinate in voxel space: ";
      std::cin >> y_target;
      std::cout << "Enter target z coordinate in voxel space: ";
      std::cin >> z_target;


      // Convert coordinates from voxel units to meters (assuming similar conversion factor as before)
      Eigen::Vector3d startSpacePoint(x_start*0.8/100, y_start*0.8/100, z_start*0.32/100);
      Eigen::Vector3d targetSpacePoint(x_target*0.8/100, y_target*0.8/100, z_target*0.32/100);

      // Apply transformation for start and target points
      Eigen::Vector3d transformedStartPoint = applyTransformation(startSpacePoint, R, t);
      Eigen::Vector3d transformedTargetPoint = applyTransformation(targetSpacePoint, R, t);

      // Convert transformed points to homogeneous matrix format and add to collected_ee_poses
      Eigen::Matrix4d transformedStartMatrix = convertToHomogeneousMatrix(transformedStartPoint);
      Eigen::Matrix4d transformedTargetMatrix = convertToHomogeneousMatrix(transformedTargetPoint);

      std::array<double, 16> transformedStartArray;
      std::array<double, 16> transformedTargetArray;
      
      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(transformedStartArray.data()) = transformedStartMatrix;
      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(transformedTargetArray.data()) = transformedTargetMatrix;
      
      global_variable::collected_ee_poses.push_back(transformedStartArray);
      global_variable::collected_ee_poses.push_back(transformedTargetArray);


      for (auto &ee_pose: global_variable::collected_ee_poses){
      Eigen::Matrix4d pose = Eigen::Matrix4d::Map(ee_pose.data());
      std::cout<<pose<<"\n"; 

      double time = 0.0;
      robot.control([&time, &pose, &ik_controller](const franka::RobotState& robot_state,
                                          franka::Duration period) -> franka::JointVelocities {
        time += period.toSec();
        franka::JointVelocities output_velocities = ik_controller(robot_state, period, pose);
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> output_eigen_velocities(robot_state.dq.data());
        Eigen::Vector3d current_position(robot_state.O_T_EE[12],robot_state.O_T_EE[13],robot_state.O_T_EE[14]); 
        Eigen::Vector3d desired_position(pose(0,3), pose(1,3), pose(2,3)) ;
        double dist = distance(current_position, desired_position);

        if (time >= 15.0 || (output_eigen_velocities.norm() < 0.0005 && dist < 0.0005) ) {
          output_velocities = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
          return franka::MotionFinished(output_velocities);
        }
        
        return output_velocities;
      });
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      }

    } if (input == 3) {

      std::thread t1(record_pose_thread, 15);
      try{
        robot.control([](const franka::RobotState& robot_state,
                        franka::Duration period) -> franka::Torques {

          franka::Torques output_torques = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
          global_variable::current_ee_pose =  robot_state.O_T_EE;
          if (global_variable::flag_done_collecting)
            return franka::MotionFinished(output_torques);  
          else
            return output_torques;
        });
      }catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
      }
      t1.join();

      std::cout << "Done collecting" << std::endl;


      for (auto &ee_pose: global_variable::collected_ee_poses){
      Eigen::Matrix4d pose = Eigen::Matrix4d::Map(ee_pose.data());
      std::cout<<pose<<"\n"; 

      double time = 0.0;
      robot.control([&time, &pose, &ik_controller](const franka::RobotState& robot_state,
                                          franka::Duration period) -> franka::JointVelocities {
        time += period.toSec();
        franka::JointVelocities output_velocities = ik_controller(robot_state, period, pose);
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> output_eigen_velocities(robot_state.dq.data());
        Eigen::Vector3d current_position(robot_state.O_T_EE[12],robot_state.O_T_EE[13],robot_state.O_T_EE[14]); 
        Eigen::Vector3d desired_position(pose(0,3), pose(1,3), pose(2,3)) ;
        double dist = distance(current_position, desired_position);

        if (time >= 15.0 || (output_eigen_velocities.norm() < 0.0005 && dist < 0.0005) ) {
          output_velocities = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
          return franka::MotionFinished(output_velocities);
        }
        
        return output_velocities;
      });
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      }


    } if (input == 4) {

      // Case 4: Impedance biopsy demo
      double x_start, y_start, z_start;
      double x_target, y_target, z_target;

      std::cout << "Enter target x coordinate in voxel space: ";
      std::cin >> x_target;
      std::cout << "Enter target y coordinate in voxel space: ";
      std::cin >> y_target;
      std::cout << "Enter target z coordinate in voxel space: ";
      std::cin >> z_target;

      x_start = x_target;
      y_start = y_target;
      z_start = z_target - 100;


      // Convert coordinates from voxel units to meters (assuming similar conversion factor as before)
      Eigen::Vector3d startSpacePoint(x_start*0.8/100, y_start*0.8/100, z_start*0.32/100);

      // Apply transformation for start and target points
      Eigen::Vector3d transformedStartPoint = applyTransformation(startSpacePoint, R, t);

      // Convert transformed points to homogeneous matrix format and add to collected_ee_poses
      Eigen::Matrix4d transformedStartMatrix = convertToHomogeneousMatrix(transformedStartPoint);

      std::array<double, 16> transformedStartArray;
      
      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(transformedStartArray.data()) = transformedStartMatrix;
      
      global_variable::collected_ee_poses.push_back(transformedStartArray);

      for (auto &ee_pose: global_variable::collected_ee_poses){
      Eigen::Matrix4d pose = Eigen::Matrix4d::Map(ee_pose.data());
      std::cout<<pose<<"\n"; 

      double time = 0.0;
      robot.control([&time, &pose, &ik_controller](const franka::RobotState& robot_state,
                                          franka::Duration period) -> franka::JointVelocities {
        time += period.toSec();
        franka::JointVelocities output_velocities = ik_controller(robot_state, period, pose);
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> output_eigen_velocities(robot_state.dq.data());
        Eigen::Vector3d current_position(robot_state.O_T_EE[12],robot_state.O_T_EE[13],robot_state.O_T_EE[14]); 
        Eigen::Vector3d desired_position(pose(0,3), pose(1,3), pose(2,3)) ;
        double dist = distance(current_position, desired_position);

        if (time >= 15.0 || (output_eigen_velocities.norm() < 0.0005 && dist < 0.0005) ) {
          output_velocities = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
          return franka::MotionFinished(output_velocities);
        }
        
        return output_velocities;
      });
      }

      // Compliance parameters

      // Change this value to change the resistance translationaly
      const double translational_stiffness{140.0}; 

      // Change this value to change the resistance rotationaly
      const double rotational_stiffness{80.0}; 

      Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
      stiffness.setZero();
      stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
      stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
      damping.setZero();
      damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                        Eigen::MatrixXd::Identity(3, 3);
      damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                            Eigen::MatrixXd::Identity(3, 3); // Damping is dependant on the rotational and translational stiffness

      
      franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.rotation());

    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});  // Changing these values will change the maximum torque threshold before the robot's safety emergency stop kicks in.



    double initial_joint1_position = robot.readOnce().q[0];

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration /*duration*/) -> franka::Torques {
      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);


      double current_joint1_position = robot_state.q[0];
      double joint1_position_change = current_joint1_position - initial_joint1_position;

      double delta_z = 0.25 * joint1_position_change;

      Eigen::Vector3d position_d_dynamic = position_d;
      position_d_dynamic.z() += delta_z;

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.rotation());

      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << position - position_d_dynamic;

      // orientation error
      // "difference" quaternion
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }
      // "difference" quaternion
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << -transform.rotation() * error.tail(3);

      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);

      // Spring damper system with damping ratio=1
      tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
      tau_d << tau_task + coriolis;

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
      return tau_d_array;
    };

    // start real-time control loop
    robot.control(impedance_control_callback);

    } if (input == 5) {

      // Case 5: Feedback biopsy demo


      // Input for target coordinates in voxel space, which will be converted to real-world coordinates for the robot to reach.
      // This setup simulates planning a path to a target location based on medical imaging, where the biopsy needle should be directed.

      double x_start, y_start, z_start;
      double x_target, y_target, z_target;

      std::cout << "Enter target x coordinate in voxel space: ";
      std::cin >> x_target;
      std::cout << "Enter target y coordinate in voxel space: ";
      std::cin >> y_target;
      std::cout << "Enter target z coordinate in voxel space: ";
      std::cin >> z_target;

      // Assuming the biopsy target is at a certain depth (z_target), we start slightly above this point (z_start) to simulate the needle insertion.
      x_start = x_target;
      y_start = y_target;
      z_start = z_target - 100; // Example offset, adjust based on actual application needs.


      // Convert coordinates from voxel units to meters, based on the imaging scale and robot's operational space.
      Eigen::Vector3d startSpacePoint(x_start*0.8/100, y_start*0.8/100, z_start*0.32/100);

      // Transform the point from the imaging coordinate system to the robot's coordinate system using a pre-defined transformation.
      Eigen::Vector3d transformedStartPoint = applyTransformation(startSpacePoint, R, t);

      // Convert the transformed start point into a homogeneous transformation matrix for use with the robot.
      Eigen::Matrix4d transformedStartMatrix = convertToHomogeneousMatrix(transformedStartPoint);

      std::array<double, 16> transformedStartArray;
      
      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(transformedStartArray.data()) = transformedStartMatrix;
      
      // Add the transformed start position to the list of poses to be reached.
      global_variable::collected_ee_poses.push_back(transformedStartArray);

      // Iterate through the collected end-effector poses, instructing the robot to move to each position.
      // This part simulates approaching the biopsy site.
      for (auto &ee_pose: global_variable::collected_ee_poses){
      Eigen::Matrix4d pose = Eigen::Matrix4d::Map(ee_pose.data());
      std::cout<<pose<<"\n"; 

      double time = 0.0;
      robot.control([&time, &pose, &ik_controller](const franka::RobotState& robot_state,
                                          franka::Duration period) -> franka::JointVelocities {
        time += period.toSec();
        franka::JointVelocities output_velocities = ik_controller(robot_state, period, pose);
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> output_eigen_velocities(robot_state.dq.data());
        Eigen::Vector3d current_position(robot_state.O_T_EE[12],robot_state.O_T_EE[13],robot_state.O_T_EE[14]); 
        Eigen::Vector3d desired_position(pose(0,3), pose(1,3), pose(2,3)) ;
        double dist = distance(current_position, desired_position);

        if (time >= 15.0 || (output_eigen_velocities.norm() < 0.0005 && dist < 0.0005) ) {
          output_velocities = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
          return franka::MotionFinished(output_velocities);
        }
        
        return output_velocities;
      });
      }

      // At this point in the biopsy demo (input = 5), the robot is positioned and ready for the biopsy procedure. 
      // The following steps would simulate or execute the biopsy needle's advancement into the target tissue, 
      // adjusting in real-time based on feedback from force sensors or similar devices.

    robot.setCollisionBehavior(
        {{0.5, 0.43, 0.5, 0.41, 0.5, 0.45, 0.5}},
        {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},
        {{5.0, 5.0, 1.0, 5.0, 5.0, 5.0}},
        {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}});

    std::array<double, 16> initial_pose;
    std::array<double, 2> initial_elbow;
    double time = 0.0;
    std::array<double, 16> original_pose;
    
    time = 0.0;

    // Initialize variables for feedback control loop
    bool keepinloop=true; // Control flag for the main feedback loop
    bool hitting = false; // Count iterations to prevent infinite loops
    int count = 0;

    // Start a feedback control loop to simulate or perform the biopsy procedure
    while (keepinloop == true) {

      double torque_on_joint1;// Variable to store the torque on joint 1, indicating force feedback
      std::array<double, 6> contact; // Array to store contact information in Cartesian space
      std::array <double, 7> joint_torque; // Array to store contact information in joint space

      // Read current robot state, including torque and contact information
      size_t nan = 0;
      robot.read([&nan, &torque_on_joint1, &original_pose, &contact, &joint_torque](const franka::RobotState& robot_state) {
        torque_on_joint1 = robot_state.tau_ext_hat_filtered[0]; // Read external torque on joint 1
        original_pose = robot_state.O_T_EE_c; 
        contact = robot_state.cartesian_contact; // Read Cartesian contact information
        joint_torque = robot_state.joint_contact; // Read joint contact information
      return nan++ < 100;
      });

    // Theses line are used to confirm the values of the contact and joint torque arrays have been read correctly, can be used during procedure if uncommented

    // std::cout << contact[0] << " " << contact[1] << " " << contact[2] << " " << contact[3] << " " << contact[4] << " " << contact[5] << " " << std::endl;
    // std::cout << joint_torque[0] << " " << joint_torque[1] << " " << joint_torque[2] << " " << joint_torque[3] << " " << joint_torque[4] << " " << joint_torque[5] << " " << joint_torque[6] << " " << std::endl;

    // Evaluate if the needle is hitting an obstacle based on joint torque and contact information
    if (joint_torque[1] == 1 && joint_torque[3] == 1 && contact[2] == 1){
      hitting = true;
    } else {
      hitting = false;
    }

    std::cout << hitting << std::endl;

    // Variables for torque limits and movement distance, adjusted based on whether the needle is hitting an obstacle
    int torque_lim;
    double distance;

    if (hitting == false){
      torque_lim = 1;
      distance = 250.0;
    } else if (hitting == true){
      torque_lim = 5;
      distance = 500.0;
    }


    // React to the force feedback by adjusting the movement of the robot
    if (torque_on_joint1 > torque_lim) {

      time = 0.0;

      // Adjust the robot's position, primarily focusing on Z-axis movement to simulate needle advancement or retraction
      robot.control(
        [&time, &initial_pose, &initial_elbow, &original_pose, &keepinloop, &count, &distance](const franka::RobotState& robot_state,
                                               franka::Duration period) -> franka::CartesianPose {
          

          if (time == 0.0) {
            initial_pose = robot_state.O_T_EE_c;
            initial_elbow = robot_state.elbow_c;
          }

          time += period.toSec();
          
          double angle = M_PI / distance * (1.0 - std::cos(M_PI / 0.5 * time));

          auto elbow = initial_elbow;
          elbow[0] += angle;

          // Calculate the new Z position based on the current feedback and intended movement distance
          double change_in_z = M_PI / distance * (1.0 - std::cos(M_PI / 0.5 * time));
          initial_pose[14] = original_pose[14] + change_in_z;

          if (time >= 0.5) {
            std::cout << std::endl << "Finished motion" << std::endl;
            if (count > 1000){
              keepinloop = false;
            }
            count += 1;
            return franka::MotionFinished({initial_pose, elbow});
          }

          return {initial_pose, elbow};
        });

    }
      // React to the force feedback by adjusting the movement of the robot
      if (torque_on_joint1 < -torque_lim) {

      time = 0.0;

      // Adjust the robot's position, primarily focusing on Z-axis movement to simulate needle advancement or retraction
      robot.control(
        [&time, &initial_pose, &initial_elbow, &original_pose, &keepinloop, &count, &distance](const franka::RobotState& robot_state,
                                               franka::Duration period) -> franka::CartesianPose {
          

          if (time == 0.0) {
            initial_pose = robot_state.O_T_EE_c;
            initial_elbow = robot_state.elbow_c;
          }

          time += period.toSec();

          double angle = M_PI / distance * (1.0 - std::cos(M_PI / 0.5 * time));

          auto elbow = initial_elbow;
          elbow[0] -= angle;

          // Calculate the new Z position based on the current feedback and intended movement distance
          double change_in_z = M_PI / distance * (1.0 - std::cos(M_PI / 0.5 * time));
          initial_pose[14] = original_pose[14] - change_in_z;

          if (time >= 0.5) {
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            if (count > 1000){
              keepinloop = false;
            }
            count += 1;
            return franka::MotionFinished({initial_pose, elbow});
          }

          return {initial_pose, elbow};
        });

    }
    }
    }

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
