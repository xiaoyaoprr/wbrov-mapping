/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <QTimer>
#include "startup_render_guard.hpp"

#include <QApplication>  // NOLINT: cpplint is unable to handle the include order here

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "rviz_common/visualizer_app.hpp"
#include "rviz_rendering/render_system.hpp"

int main(int argc, char ** argv)
{
  const unsigned attempt = wbrov::startupAttempt(std::getenv("WBROV_RVIZ_RENDER_ATTEMPT"));
  std::vector<std::string> originalArgs(argv, argv + argc);
  std::string executable;
  bool retryRequested = false;
  int result = 1;
  std::atomic<bool> contextFailed{false};
  {
  // remove ROS arguments before passing to QApplication
  std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> non_ros_args_c_strings;
  for (auto & arg : non_ros_args) {
    non_ros_args_c_strings.push_back(&arg.front());
  }
  int non_ros_argc = static_cast<int>(non_ros_args_c_strings.size());

  QApplication qapp(non_ros_argc, non_ros_args_c_strings.data());
  executable = QCoreApplication::applicationFilePath().toStdString();

  // TODO(wjwwood): use node's logger here in stead
  auto logger = rclcpp::get_logger("rviz2");
  // install logging handlers to route logging through ROS's logging system
  rviz_common::set_logging_handlers(
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_DEBUG(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_INFO(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_WARN(logger, "%s", msg.c_str());
    },
    [logger, &contextFailed](const std::string & msg, const std::string &, size_t) {
      if (wbrov::isContextCreationFailure(msg)) contextFailed.store(true);
      RCLCPP_ERROR(logger, "%s", msg.c_str());
    }
  );

  rviz_rendering::RenderSystem::forceNoStereo();
  rviz_common::VisualizerApp vapp(
    std::make_unique<rviz_common::ros_integration::RosClientAbstraction>());
  vapp.setApp(&qapp);
  if (vapp.init(argc, argv)) {
    // Only initialization may restart the display. The mapping process and
    // camera are separate, and the RViz PID remains unchanged across exec.
    QTimer::singleShot(2000, &qapp, [&]() {
      if (contextFailed.load()) {
        retryRequested = attempt < wbrov::maxRenderRetries;
        RCLCPP_ERROR(logger, "[WBROV_RVIZ_RENDER_FAILED] attempt=%u retry=%d", attempt, retryRequested);
        qapp.exit(75);
      } else {
        RCLCPP_INFO(logger, "[WBROV_RVIZ_RENDER_READY] attempt=%u", attempt);
      }
    });
    result = qapp.exec();
  }
  }  // Shut down the ROS context and Qt windows before replacing this process.
  if (!retryRequested) return result;
  const std::string nextAttempt = std::to_string(attempt + 1);
  if (::setenv("WBROV_RVIZ_RENDER_ATTEMPT", nextAttempt.c_str(), 1) != 0) return 75;
  std::vector<char *> args;
  for (auto & arg : originalArgs) args.push_back(arg.data());
  args.push_back(nullptr);
  ::execv(executable.c_str(), args.data());
  std::cerr << "RViz initialization retry could not execute the verified entry point\n";
  return 75;
}
