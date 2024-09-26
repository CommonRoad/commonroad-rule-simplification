#pragma once

#include <nanobind/nanobind.h>

void export_propositions(nanobind::module_ &module);

void export_ego_parameters(nanobind::module_ &module);

void export_extraction_result(nanobind::module_ &module);

void export_extraction_interface(nanobind::module_ &module);
