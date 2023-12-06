// Copyright 2023 AIT Austrian Institute of Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "test_pid_trajectory.hpp"

TEST_F(PidTrajectoryTest, TestEmptySetup)
{
  std::shared_ptr<TestableJointTrajectoryControllerPlugin> traj_contr =
    std::make_shared<TestableJointTrajectoryControllerPlugin>();

  ASSERT_FALSE(traj_contr->initialize(node_, std::vector<std::string>()));
}

TEST_F(PidTrajectoryTest, TestSingleJoint)
{
  std::shared_ptr<TestableJointTrajectoryControllerPlugin> traj_contr =
    std::make_shared<TestableJointTrajectoryControllerPlugin>();

  std::vector<std::string> joint_names = {"joint1"};
  auto joint_names_paramv = rclcpp::ParameterValue(joint_names);

  // override read_only parameter
  node_->declare_parameter("joints", joint_names_paramv);

  ASSERT_TRUE(traj_contr->initialize(node_, joint_names));

  // set dynamic parameters
  traj_contr->trigger_declare_parameters();
  node_->set_parameter(rclcpp::Parameter("gains.joint1.p", 1.0));

  ASSERT_TRUE(traj_contr->computeGains(trajectory_msgs::msg::JointTrajectory()));

  trajectory_msgs::msg::JointTrajectoryPoint traj_msg;
  traj_msg.positions.push_back(0.0);
  traj_msg.velocities.push_back(0.0);
  std::vector<double> tmp_command(1, 0.0);
  const rclcpp::Time time;
  const rclcpp::Duration period(1, 0);

  ASSERT_NO_THROW(
    traj_contr->computeCommands(tmp_command, traj_msg, traj_msg, traj_msg, time, period));
}

TEST_F(PidTrajectoryTest, TestMultipleJoints)
{
  std::shared_ptr<TestableJointTrajectoryControllerPlugin> traj_contr =
    std::make_shared<TestableJointTrajectoryControllerPlugin>();

  std::vector<std::string> joint_names = {"joint1", "joint2", "joint3"};
  auto joint_names_paramv = rclcpp::ParameterValue(joint_names);

  // override read_only parameter
  node_->declare_parameter("joints", joint_names_paramv);

  ASSERT_TRUE(traj_contr->initialize(node_, joint_names));

  // set dynamic parameters
  traj_contr->trigger_declare_parameters();
  node_->set_parameter(rclcpp::Parameter("gains.joint1.p", 1.0));
  node_->set_parameter(rclcpp::Parameter("gains.joint2.p", 1.0));
  node_->set_parameter(rclcpp::Parameter("gains.joint3.p", 1.0));

  ASSERT_TRUE(traj_contr->computeGains(trajectory_msgs::msg::JointTrajectory()));

  trajectory_msgs::msg::JointTrajectoryPoint traj_msg;
  traj_msg.positions.push_back(0.0);
  traj_msg.positions.push_back(0.0);
  traj_msg.positions.push_back(0.0);
  traj_msg.velocities.push_back(0.0);
  traj_msg.velocities.push_back(0.0);
  traj_msg.velocities.push_back(0.0);
  std::vector<double> tmp_command(3, 0.0);
  const rclcpp::Time time;
  const rclcpp::Duration period(1, 0);

  ASSERT_NO_THROW(
    traj_contr->computeCommands(tmp_command, traj_msg, traj_msg, traj_msg, time, period));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
