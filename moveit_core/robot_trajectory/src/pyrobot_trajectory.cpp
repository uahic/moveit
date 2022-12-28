/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Martin Schulze
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * The name of Peter Mitrano may not be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Martin Schulze */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <moveit/python/pybind_rosmsg_typecasters.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model/robot_model.h>

namespace py = pybind11;
using namespace robot_trajectory;

void def_robot_trajectory_bindings(py::module& m)
{
  m.doc() = "";

  py::class_<RobotTrajectory, RobotTrajectoryPtr>(m, "RobotTrajectory")
      .def(py::init<const robot_model::RobotModelConstPtr&, const std::string&>(), py::arg("robot_model"),
           py::arg("group"))
      .def(py::init<const robot_model::RobotModelConstPtr&, const robot_model::JointModelGroup*>(),
           py::arg("robot_model"), py::arg("group"))
      .def("getRobotModel", &RobotTrajectory::getRobotModel)
      .def("getGroup", &RobotTrajectory::getGroup)
      .def("getGroupName", &RobotTrajectory::getGroupName)
      .def("setGroupName", &RobotTrajectory::setGroupName, py::arg("group_name"))
      .def("getWayPointCount", &RobotTrajectory::getWayPointCount)
      .def("getWayPoint", &RobotTrajectory::getWayPoint)
      .def("getLastWayPoint", &RobotTrajectory::getLastWayPoint)
      .def("getFirstWayPoint", &RobotTrajectory::getFirstWayPoint)
      .def("getWayPointDurations", &RobotTrajectory::getWayPointDurations)
      .def("getWayPointDurationFromStart", &RobotTrajectory::getWayPointDurationFromStart, py::arg("index"))
      .def("getWayPointDurationFromPrevious", &RobotTrajectory::getWayPointDurationFromPrevious, py::arg("index"))
      .def("setWayPointDurationFromPrevious", &RobotTrajectory::setWayPointDurationFromPrevious, py::arg("index"),
           py::arg("value"))
      .def("empty", &RobotTrajectory::empty)
      .def("addSuffixWayPoint",
           py::overload_cast<const robot_state::RobotState&, double>(&RobotTrajectory::addSuffixWayPoint),
           py::arg("state"), py::arg("dt"))
      .def("addSuffixWayPoint",
           py::overload_cast<const robot_state::RobotStatePtr&, double>(&RobotTrajectory::addSuffixWayPoint))

      .def("addPrefixWayPoint",
           py::overload_cast<const robot_state::RobotState&, double>(&RobotTrajectory::addPrefixWayPoint),
           py::arg("state"), py::arg("dt"))
      .def("addPrefixWayPoint",
           py::overload_cast<const robot_state::RobotStatePtr&, double>(&RobotTrajectory::addPrefixWayPoint),
           py::arg("state"), py::arg("dt"))
      .def("insertWayPoint",
           py::overload_cast<std::size_t, const robot_state::RobotState&, double>(&RobotTrajectory::insertWayPoint),
           py::arg("index"), py::arg("state"), py::arg("dt"))
      .def("insertWayPoint",
           py::overload_cast<std::size_t, const robot_state::RobotStatePtr&, double>(&RobotTrajectory::insertWayPoint),
           py::arg("index"), py::arg("state"), py::arg("dt"))
      .def("append", &RobotTrajectory::append, py::arg("source"), py::arg("dt"), py::arg("start_index") = 0,
           py::arg("end_index") = std::numeric_limits<std::size_t>::max())
      .def("swap", &RobotTrajectory::swap, py::arg("other"))
      .def("clear", &RobotTrajectory::clear)
      .def("getDuration", &RobotTrajectory::getDuration)
      .def("getAverageSegmentDuration", &RobotTrajectory::getAverageSegmentDuration)
      .def("getRobotTrajectoryMsg", &RobotTrajectory::getRobotTrajectoryMsg, py::arg("trajectory"),
           py::arg("joint_filter") = std::vector<std::string>())
      .def("setRobotTrajectoryMsg",
           py::overload_cast<const robot_state::RobotState&, const trajectory_msgs::JointTrajectory&>(
               &RobotTrajectory::setRobotTrajectoryMsg),
           py::arg("reference_state"), py::arg("trajectory"))
      .def("setRobotTrajectoryMsg",
           py::overload_cast<const robot_state::RobotState&, const moveit_msgs::RobotTrajectory&>(
               &RobotTrajectory::setRobotTrajectoryMsg),
           py::arg("reference_state"), py::arg("trajectory"))
      .def("setRobotTrajectoryMsg",
           py::overload_cast<const robot_state::RobotState&, const moveit_msgs::RobotState&,
                             const moveit_msgs::RobotTrajectory&>(&RobotTrajectory::setRobotTrajectoryMsg),
           py::arg("reference_state"), py::arg("state"), py::arg("trajectory"))
      .def("reverse", &RobotTrajectory::reverse)
      .def("unwind", py::overload_cast<>(&RobotTrajectory::unwind))
      .def("unwind", py::overload_cast<const robot_state::RobotState&>(&RobotTrajectory::unwind), py::arg("state"))
      .def("findWayPointIndicesForDurationAfterStart", &RobotTrajectory::findWayPointIndicesForDurationAfterStart,
           py::arg("duration"), py::arg("before"), py::arg("after"), py::arg("blend"))
      .def("getStateAtDurationFromStart", &RobotTrajectory::getStateAtDurationFromStart, py::arg("request_duration"),
           py::arg("output_state"))
      //
      ;
}