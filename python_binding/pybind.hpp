#pragma once

#include <pybind11/pybind11.h>

namespace py = pybind11;

void export_ego_parameters(py::module &module);

void export_extraction_result(py::module &module);

void export_extraction_interface(py::module &module);
