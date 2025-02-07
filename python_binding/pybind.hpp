#pragma once

#include <nanobind/nanobind.h>

void export_propositions(const nanobind::module_ &module);

void export_ego_parameters(const nanobind::module_ &module);

void export_extraction_result(const nanobind::module_ &module);

void export_extraction_interface(const nanobind::module_ &module);
