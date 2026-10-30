// SPDX-License-Identifier: Apache-2.0
#include <onero_a1_kinematics/profile.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
namespace py=pybind11;
void bindNativePlanning(py::module_& module);
PYBIND11_MODULE(_core,module)
{
  using namespace onero_a1_kinematics;
  py::class_<Profile>(module,"Profile")
    .def(py::init<Eigen::VectorXd,Eigen::VectorXd>())
    .def_readonly("reference",&Profile::reference).def_readonly("weights",&Profile::weights)
    .def_readwrite("enabled",&Profile::enabled).def_readonly("max_speed",&Profile::max_speed)
    .def_readonly("planning_candidates",&Profile::planning_candidates)
    .def_readonly("continuity_weight",&Profile::continuity_weight)
    .def_readonly("path_weight",&Profile::path_weight)
    .def_readonly("max_wrist_roll_deviation",&Profile::max_wrist_roll_deviation)
    .def("set_bounds",&Profile::setBounds).def("cost",&Profile::cost)
    .def("gradient",&Profile::gradient).def("path_cost",&Profile::pathCost)
    .def("nullspace_velocity",&Profile::nullspaceVelocity,py::arg("q"),py::arg("jacobian"),py::arg("scale")=1.);
  bindNativePlanning(module);
  module.def("load_profile",&loadProfile);
  module.def("project_nullspace",&projectNullspace);
}
