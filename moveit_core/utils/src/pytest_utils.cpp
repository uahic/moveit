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

#include <srdfdom/srdf_writer.h>
#include <urdf_model/model.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>
#include <moveit/utils/robot_model_test_utils.h>

namespace py = pybind11;
using namespace moveit::core;

void def_test_utils_bindings(py::module& m)
{
  m.doc() = "Moveit testing utility functions";

  m.def("loadTestingRobotModel", &loadTestingRobotModel, py::arg("robot_name"));
  m.def("loadModelInterface", &loadModelInterface, py::arg("robot_name"));
  m.def("loadSRDFModel", &loadSRDFModel, py::arg("robot_name"));

  py::class_<RobotModelBuilder, std::shared_ptr<RobotModelBuilder>>(m, "RobotModelBuilder")
      .def(py::init<const std::string&, const std::string&>(), py::arg("name"), py::arg("base_link_name"))
      .def("addChain", &RobotModelBuilder::addChain, py::arg("section"), py::arg("type"),
           py::arg("joint_origins") = std::vector<geometry_msgs::Pose>(),
           py::arg("joint_axis") = std::array<double, 3>{ 1.0, 0.0, 0.0 })
      .def("addCollisionMesh", &RobotModelBuilder::addCollisionMesh, py::arg("link_name"), py::arg("filename"),
           py::arg("origin"))
      .def("addCollisionBox", &RobotModelBuilder::addCollisionBox, py::arg("link_name"), py::arg("dims"),
           py::arg("origin"))
      .def("addVisualBox", &RobotModelBuilder::addVisualBox, py::arg("link_name"), py::arg("size"), py::arg("origin"))
      .def("addInertial", &RobotModelBuilder::addInertial, py::arg("link_name"), py::arg("mass"), py::arg("origin"),
           py::arg("ixx"), py::arg("ixy"), py::arg("ixz"), py::arg("iyy"), py::arg("iyz"), py::arg("izz"))
      .def("addVirtualJoint", &RobotModelBuilder::addVirtualJoint, py::arg("parent_frame"), py::arg("child_link"),
           py::arg("type"), py::arg("name") = std::string(""))
      .def("addGroupChain", &RobotModelBuilder::addGroupChain, py::arg("base_link"), py::arg("tip_link"),
           py::arg("name") = std::string(""))
      .def("addGroup", &RobotModelBuilder::addGroup, py::arg("links"), py::arg("joints"), py::arg("name"))
      .def("addEndEffector", &RobotModelBuilder::addEndEffector, py::arg("name"), py::arg("parent_link"),
           py::arg("parent_group") = std::string(""), py::arg("component_group") = std::string(""))
      .def("isValid", &RobotModelBuilder::isValid)
      .def("build", &RobotModelBuilder::build)
      //
      ;
}