// Copyright 2023 University of Leeds.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include <gtest/gtest.h>
#include "mock_communications.hpp"
#include "motor_node.hpp"

/**
 * @file Test the MotorNode class.
 *
*/

// This delay is set to the smallest value that allows the tests to pass every time.
static const std::chrono::milliseconds kConnectionDelayMs(250);

/**
 * @brief Test class implementation.
 */
class TestMotorNode : public ::testing::Test
{
protected:
  void SetUp();
  void spin();
  void TearDown();
  std::shared_ptr<MotorNode> node_;
  rclcpp::executors::SingleThreadedExecutor * exec_;
  std::thread * thread_;
};

void TestMotorNode::SetUp()
{
  // Set up rclcpp stuff.
  rclcpp::init(0, nullptr);
  rcutils_ret_t result = rcutils_logging_set_logger_level(
    "TestMotorNode", RCUTILS_LOG_SEVERITY_DEBUG);
  (void) result;
  // Create node to test.
  rclcpp::NodeOptions options;
  node_ = std::make_shared<MotorNode>(options);
  // Inject fake comms class.
  auto comms = std::make_shared<MockCommunications>();
  node_->AddComms(comms);
  // Add the node to the executor.
  exec_ = new rclcpp::executors::SingleThreadedExecutor();
  exec_->add_node(node_);
  // Create thread so that the node runs while the tests are taking place.
  thread_ = new std::thread(&TestMotorNode::spin, this);
  // Wait for the node to get started.
  std::this_thread::sleep_for(kConnectionDelayMs);
}

void TestMotorNode::spin()
{
  exec_->spin();
}

void TestMotorNode::TearDown()
{
  RCUTILS_LOG_INFO_NAMED("TestMotorNode", "%s", __FUNCTION__);
  // Stop node.
  exec_->cancel();
  thread_->join();
  RCUTILS_LOG_INFO_NAMED("TestMotorNode", "%s joined", __FUNCTION__);
  delete exec_;
  delete thread_;
  // Destroy node.
  node_ = nullptr;
  rclcpp::shutdown();
  RCUTILS_LOG_INFO_NAMED("TestMotorNode", "%s done", __FUNCTION__);
}

/// @test Verify that the node can be started and stopped.
TEST_F(TestMotorNode, Smoke) {
  RCUTILS_LOG_INFO_NAMED("TestMotorNode", "%s: started", __FUNCTION__);
  SUCCEED();
}

/// @test Verify that all publishers and subscribers offered by the node are
/// present.
TEST_F(TestMotorNode, PubsAndSubs) {
  RCUTILS_LOG_INFO_NAMED("TestMotorNode", "%s: started", __FUNCTION__);
  // Provide publishers and verify that the publisher has been able to connect
  // to a subscriber offered by the MotorNode.
  SUCCEED();
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
