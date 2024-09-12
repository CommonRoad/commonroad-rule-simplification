#include "pybind.hpp"

#include "helper/world.hpp"

#include "cr_knowledge_extraction/extraction_interface.hpp"

#include <pybind11/stl.h>

using knowledge_extraction::ego_behavior::EgoParameters;

PYBIND11_MODULE(cr_knowledge_extraction_core, module) {
    // Import the Python bindings of the curvilinear coordinate system to ensure that the necessary types are bound
    py::module_::import("commonroad_dc.pycrccosy");

    module.doc() = "C++ extension for commonroad-knowledge-extraction.";

    export_ego_parameters(module);
    export_extraction_result(module);
    export_extraction_interface(module);
}

void export_extraction_result(py::module &module) {
    py::class_<knowledge_extraction::ExtractionResult>(module, "ExtractionResult")
        .def_readonly("positive_propositions", &knowledge_extraction::ExtractionResult::positive_propositions)
        .def_readonly("negative_propositions", &knowledge_extraction::ExtractionResult::negative_propositions)
        .def_readonly("implications", &knowledge_extraction::ExtractionResult::implications)
        .def_readonly("equivalences", &knowledge_extraction::ExtractionResult::equivalences);
}

void export_extraction_interface(py::module &module) {
    py::class_<knowledge_extraction::ExtractionInterface>(module, "ExtractionInterface")
        .def(py::init([](const std::string &scenario_path, double dt, const EgoParameters &ego_params,
                         const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) {
                 auto world = pybind_helper::open_world(scenario_path, dt);
                 return knowledge_extraction::ExtractionInterface{std::move(world), ego_ccs, ego_params};
             }),
             py::arg("scenario_path"), py::arg("dt"), py::arg("ego_params"), py::arg("ego_ccs"))
        .def("extract", &knowledge_extraction::ExtractionInterface::extract);
}

void export_ego_parameters(py::module &module) {
    py::class_<knowledge_extraction::ego_behavior::EgoParameters>(module, "EgoParameters")
        .def(py::init([](std::optional<double> a_lon_min, std::optional<double> a_lon_max,
                         std::optional<double> a_lat_min, std::optional<double> a_lat_max,
                         std::optional<double> v_lon_min, std::optional<double> v_lon_max,
                         std::optional<double> v_lat_min, std::optional<double> v_lat_max,
                         std::optional<std::tuple<time_step_t, double, double, double, double, double>> initial_state,
                         std::optional<double> uncertainty_p_lon, std::optional<double> uncertainty_p_lat,
                         std::optional<double> uncertainty_v_lon, std::optional<double> uncertainty_v_lat,
                         std::optional<double> length, std::optional<double> width, std::optional<double> t_react) {
                 auto params = EgoParameters{};
                 if (a_lon_min.has_value()) {
                     params.a_lon_min = a_lon_min.value();
                 }
                 if (a_lon_max.has_value()) {
                     params.a_lon_max = a_lon_max.value();
                 }
                 if (a_lat_min.has_value()) {
                     params.a_lat_min = a_lat_min.value();
                 }
                 if (a_lat_max.has_value()) {
                     params.a_lat_max = a_lat_max.value();
                 }
                 if (v_lon_min.has_value()) {
                     params.v_lon_min = v_lon_min.value();
                 }
                 if (v_lon_max.has_value()) {
                     params.v_lon_max = v_lon_max.value();
                 }
                 if (v_lat_min.has_value()) {
                     params.v_lat_min = v_lat_min.value();
                 }
                 if (v_lat_max.has_value()) {
                     params.v_lat_max = v_lat_max.value();
                 }
                 if (initial_state.has_value()) {
                     auto [time_step, p_x, p_y, v, acc, phi] = initial_state.value();
                     params.initial_state = State{time_step, p_x, p_y, v, acc, phi};
                 }
                 if (uncertainty_p_lon.has_value()) {
                     params.uncertainty_p_lon = uncertainty_p_lon.value();
                 }
                 if (uncertainty_p_lat.has_value()) {
                     params.uncertainty_p_lat = uncertainty_p_lat.value();
                 }
                 if (uncertainty_v_lon.has_value()) {
                     params.uncertainty_v_lon = uncertainty_v_lon.value();
                 }
                 if (uncertainty_v_lat.has_value()) {
                     params.uncertainty_v_lat = uncertainty_v_lat.value();
                 }
                 if (length.has_value()) {
                     params.length = length.value();
                 }
                 if (width.has_value()) {
                     params.width = width.value();
                 }
                 if (t_react.has_value()) {
                     params.t_react = t_react.value();
                 }
                 return params;
             }),
             py::arg("a_lon_min") = std::nullopt, py::arg("a_lon_max") = std::nullopt,
             py::arg("a_lat_min") = std::nullopt, py::arg("a_lat_max") = std::nullopt,
             py::arg("v_lon_min") = std::nullopt, py::arg("v_lon_max") = std::nullopt,
             py::arg("v_lat_min") = std::nullopt, py::arg("v_lat_max") = std::nullopt,
             py::arg("initial_state") = std::nullopt, py::arg("uncertainty_p_lon") = std::nullopt,
             py::arg("uncertainty_p_lat") = std::nullopt, py::arg("uncertainty_v_lon") = std::nullopt,
             py::arg("uncertainty_v_lat") = std::nullopt, py::arg("length") = std::nullopt,
             py::arg("width") = std::nullopt, py::arg("t_react") = std::nullopt)
        .def_readwrite("a_lon_min", &EgoParameters::a_lon_min)
        .def_readwrite("a_lon_max", &EgoParameters::a_lon_max)
        .def_readwrite("a_lat_min", &EgoParameters::a_lat_min)
        .def_readwrite("a_lat_max", &EgoParameters::a_lat_max)
        .def_readwrite("v_lon_min", &EgoParameters::v_lon_min)
        .def_readwrite("v_lon_max", &EgoParameters::v_lon_max)
        .def_readwrite("v_lat_min", &EgoParameters::v_lat_min)
        .def_readwrite("v_lat_max", &EgoParameters::v_lat_max)
        .def_property(
            "initial_state",
            [](const EgoParameters &params) {
                const auto &initial_state = params.initial_state;
                return std::make_tuple(initial_state.getTimeStep(), initial_state.getXPosition(),
                                       initial_state.getYPosition(), initial_state.getVelocity(),
                                       initial_state.getAcceleration(), initial_state.getGlobalOrientation());
            },
            [](EgoParameters &params, const std::tuple<time_step_t, double, double, double, double, double> &state) {
                auto [time_step, p_x, p_y, v, acc, phi] = state;
                params.initial_state = State{time_step, p_x, p_y, v, acc, phi};
            })
        .def_readwrite("uncertainty_p_lon", &EgoParameters::uncertainty_p_lon)
        .def_readwrite("uncertainty_p_lat", &EgoParameters::uncertainty_p_lat)
        .def_readwrite("uncertainty_v_lon", &EgoParameters::uncertainty_v_lon)
        .def_readwrite("uncertainty_v_lat", &EgoParameters::uncertainty_v_lat)
        .def_readwrite("length", &EgoParameters::length)
        .def_readwrite("width", &EgoParameters::width)
        .def_readwrite("t_react", &EgoParameters::t_react);
}
