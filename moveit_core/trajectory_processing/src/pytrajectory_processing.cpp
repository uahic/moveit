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

#include <urdf_model/model.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
// #include <moveit/trajectory_processing/ruckig_traj_smoothing.h>

namespace py = pybind11;
using namespace trajectory_processing;

void def_trajectory_processing_bindings(py::module& m)
{
  m.doc() =
      "Time re-parametrization functions for creating smooth joint trajectories with respect to velocity and "
      "acceleration bounds. Jerks are discontinuous.";

  // Path
  py::class_<Path, std::shared_ptr<Path>>(m, "Path")
      .def(py::init<const std::list<Eigen::VectorXd>&, double>(), py::arg("path"), py::arg("max_deviation") = 0.0)
      .def(py::init<const Path&>(), py::arg("path"))
      .def("getLength", &Path::getLength)
      .def("getConfig", &Path::getConfig, py::arg("s"))
      .def("getTangent", &Path::getTangent, py::arg("s"))
      .def("getCurvature", &Path::getCurvature, py::arg("s"))
      .def("getNextSwitchingPoint", &Path::getNextSwitchingPoint, py::arg("s"), py::arg("discontinuity"))
      .def("getSwitchingPoints", &Path::getSwitchingPoints)
      //
      ;

  // Trajectory
  py::class_<Trajectory, std::shared_ptr<Trajectory>>(m, "Trajectory")
      .def(py::init<const Path&, const Eigen::VectorXd&, const Eigen::VectorXd&, double>(), py::arg("path"),
           py::arg("max_velocity"), py::arg("max_acceleration"), py::arg("time_step") = 0.001)
      .def("isValid", &Trajectory::isValid)
      .def("getDuration", &Trajectory::getDuration)
      .def("getPosition", &Trajectory::getPosition, py::arg("time"))
      .def("getVelocity", &Trajectory::getVelocity, py::arg("time"))
      .def("getAcceleration", &Trajectory::getAcceleration, py::arg("time"))
      .def("resample",
           [](const Trajectory& trajectory, double resample_dt) {
             size_t sample_count = std::ceil(trajectory.getDuration() / resample_dt);
             std::list<Eigen::VectorXd> positions;
             std::list<Eigen::VectorXd> velocities;
             std::list<Eigen::VectorXd> accelerations;
             std::list<double> times;
             //    std::tuple<std::list<Eigen::VectorXd>, std::list<Eigen::VectorXd>, std::list<Eigen::VectorXd>,
             //    std::list<double>>
             //        ret_val;

             for (size_t sample = 0; sample <= sample_count; ++sample)
             {
               // always sample the end of the trajectory as well
               double t = std::min(trajectory.getDuration(), sample * resample_dt);
               Eigen::VectorXd position = trajectory.getPosition(t);
               Eigen::VectorXd velocity = trajectory.getVelocity(t);
               Eigen::VectorXd acceleration = trajectory.getAcceleration(t);
               positions.push_back(position);
               velocities.push_back(velocity);
               accelerations.push_back(acceleration);
               times.push_back(t);
             }
             return std::make_tuple(positions, velocities, accelerations, times);
           })
      //
      ;

  // TimeOptimalTrajectoryGeneration
  py::class_<TimeOptimalTrajectoryGeneration, TimeOptimalTrajectoryGenerationPtr>(m, "TimeOptimalTrajectoryGeneration")
      .def(py::init<const double, const double, const double>(), py::arg("path_tolerance") = 0.1,
           py::arg("resample_dt") = 0.1, py::arg("min_angle_change") = 0.001)
      .def("computeTimeStamps", &TimeOptimalTrajectoryGeneration::computeTimeStamps, py::arg("trajectory"),
           py::arg("max_velocity_scaling_factor") = 1.0, py::arg("max_acceleration_scaling_factor") = 1.0)
      //
      ;

  // IterativeParabolicTimeParameterization
  py::class_<IterativeParabolicTimeParameterization, IterativeParabolicTimeParameterizationPtr>(m,
                                                                                                "IterativeParabolicTime"
                                                                                                "Parameterization")
      .def(py::init<unsigned int, double>(), py::arg("max_iterations") = 100, py::arg("max_time_change_per_it") = 0.01)
      .def("computeTimeStamps", &IterativeParabolicTimeParameterization::computeTimeStamps, py::arg("trajectory"),
           py::arg("max_velocity_scaling_factor") = 1.0, py::arg("max_acceleration_scaling_factor") = 1.0)
      //
      ;

  // IterativeSplineParameterization
  py::class_<IterativeSplineParameterization, std::shared_ptr<IterativeSplineParameterization>>(m,
                                                                                                "IterativeSplineParamet"
                                                                                                "erization")
      .def(py::init<bool>(), py::arg("add_points") = true)
      .def("computeTimeStamps", &IterativeSplineParameterization::computeTimeStamps, py::arg("trajectory"),
           py::arg("max_velocity_scaling_factor") = 1.0, py::arg("max_acceleration_scaling_factor") = 1.0)
      //
      ;

  // RuckigSmoothing
  //   py::class_<RuckigSmoothing, std::shared_ptr<RuckigSmoothing>>(m, "RuckigSmoothing")
  //       .def(py::init<>())
  //       .def_static("applySmoothing",
  //                   py::overload_cast<robot_trajectory::RobotTrajectory&, const std::unordered_map<std::string,
  //                   double>&,
  //                                     const std::unordered_map<std::string, double>&,
  //                                     const std::unordered_map<std::string,
  //                                     double>&>(&RuckigSmoothing::applySmoothing))
  //       .def_static("applySmoothing", py::overload_cast<robot_trajectory::RobotTrajectory&, const double, const
  //       double>(
  //                                         &RuckigSmoothing::applySmoothing))
  //       //
  //       ;
}
