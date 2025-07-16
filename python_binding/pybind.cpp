#include "pybind.hpp"

#include "cr_knowledge_extraction/extraction_interface.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>

using knowledge_extraction::Proposition;
using knowledge_extraction::ego_behavior::EgoParameters;

namespace nb = nanobind;
using namespace nb::literals;

namespace {
std::optional<nb::module_> try_import(const char *name) {
    try {
        return nb::module_::import_(name);
    } catch (nb::python_error &e) {
        if (e.matches(nb::builtins()["ModuleNotFoundError"])) {
            const auto py_warn = nb::module_::import_("warnings");
            py_warn.attr("warn")(
                "The module '" + std::string(name) +
                "' is not found. Some signatures might be incorrect and some features may not work as expected.");
            return std::nullopt;
        }
        throw;
    }
}
} // namespace

NB_MODULE(knowledge_extraction_core, module) {
    module.doc() = "C++ extension for knowledge extraction from CommonRoad scenarios.";
    // Import the Python bindings of the CLCS and environment model to ensure that the necessary types are bound
    try_import("commonroad_clcs.pycrccosy");
    try_import("crcpp");

    export_propositions(module);
    export_ego_parameters(module);
    export_extraction_result(module);
    export_extraction_interface(module);
}

void export_extraction_result(const nb::module_ &module) {
    nb::class_<knowledge_extraction::ExtractionResult>(module, "ExtractionResult")
        .def_ro("positive_propositions", &knowledge_extraction::ExtractionResult::positive_propositions)
        .def_ro("negative_propositions", &knowledge_extraction::ExtractionResult::negative_propositions)
        .def_ro("implications", &knowledge_extraction::ExtractionResult::implications)
        .def_ro("equivalences", &knowledge_extraction::ExtractionResult::equivalences);
}

void export_extraction_interface(const nb::module_ &module) {
    nb::class_<knowledge_extraction::ExtractionInterface>(module, "ExtractionInterface")
        .def(nb::init<std::shared_ptr<World>, std::shared_ptr<geometry::CurvilinearCoordinateSystem>, EgoParameters>(),
             "world"_a, "ego_ccs"_a, "ego_params"_a)
        .def("extract_all", &knowledge_extraction::ExtractionInterface::extract_all)
        .def("extract_all_but_implications", &knowledge_extraction::ExtractionInterface::extract_all_but_implications)
        .def("extract_kleene", nb::overload_cast<const std::unordered_map<time_step_t, std::vector<std::string>> &>(
                                   &knowledge_extraction::ExtractionInterface::extract_kleene))
        .def("extract_relationships",
             nb::overload_cast<const std::unordered_map<time_step_t, std::vector<std::string>> &>(
                 &knowledge_extraction::ExtractionInterface::extract_relationships))
        .def("extract_equivalences", &knowledge_extraction::ExtractionInterface::extract_equivalences)
        .def("extract_implications", &knowledge_extraction::ExtractionInterface::extract_implications);
}

void export_propositions(const nb::module_ &module) {
    auto prop = nb::enum_<Proposition>(module, "Proposition")
                    .value("IN_SAME_LANE", Proposition::IN_SAME_LANE)
                    .value("KEEPS_SAFE_DISTANCE_PREC", Proposition::KEEPS_SAFE_DISTANCE_PREC)
                    .value("CUT_IN", Proposition::CUT_IN)
                    .value("IN_FRONT_OF", Proposition::IN_FRONT_OF)
                    .value("ON_MAIN_CARRIAGEWAY", Proposition::ON_MAIN_CARRIAGEWAY)
                    .value("ON_MAIN_CARRIAGEWAY_RIGHT_LANE", Proposition::ON_MAIN_CARRIAGEWAY_RIGHT_LANE)
                    .value("ON_MAIN_CARRIAGEWAY_LEFT_LANE", Proposition::ON_MAIN_CARRIAGEWAY_LEFT_LANE)
                    .value("OTHER_ON_ACCESS_RAMP", Proposition::OTHER_ON_ACCESS_RAMP)
                    .value("OTHER_ON_MAIN_CARRIAGEWAY", Proposition::OTHER_ON_MAIN_CARRIAGEWAY)
                    .value("IN_INTERSECTION", Proposition::IN_INTERSECTION)
                    .value("AT_STOP_SIGN", Proposition::AT_STOP_SIGN)
                    .value("STOP_LINE_IN_FRONT", Proposition::STOP_LINE_IN_FRONT)
                    .value("RELEVANT_TRAFFIC_LIGHT", Proposition::RELEVANT_TRAFFIC_LIGHT)
                    .value("IN_STANDSTILL", Proposition::IN_STANDSTILL)
                    .value("ON_INCOMING_LEFT_OF", Proposition::ON_INCOMING_LEFT_OF)
                    .value("ON_ONCOMING_OF", Proposition::ON_ONCOMING_OF)
                    .value("IN_INTERSECTION_CONFLICT_AREA", Proposition::IN_INTERSECTION_CONFLICT_AREA)
                    .value("OTHER_IN_INTERSECTION_CONFLICT_AREA", Proposition::OTHER_IN_INTERSECTION_CONFLICT_AREA)
                    .value("CAUSES_BRAKING_INTERSECTION", Proposition::CAUSES_BRAKING_INTERSECTION)

                    .value("TURNING_LEFT", Proposition::TURNING_LEFT)
                    .value("TURNING_RIGHT", Proposition::TURNING_RIGHT)
                    .value("GOING_STRAIGHT", Proposition::GOING_STRAIGHT)
                    .value("OTHER_TURNING_LEFT", Proposition::OTHER_TURNING_LEFT)
                    .value("OTHER_TURNING_RIGHT", Proposition::OTHER_TURNING_RIGHT)
                    .value("OTHER_GOING_STRAIGHT", Proposition::OTHER_GOING_STRAIGHT)

                    .value("SAME_LEFT_LEFT_PRIORITY", Proposition::SAME_LEFT_LEFT_PRIORITY)
                    .value("SAME_LEFT_RIGHT_PRIORITY", Proposition::SAME_LEFT_RIGHT_PRIORITY)
                    .value("SAME_LEFT_STRAIGHT_PRIORITY", Proposition::SAME_LEFT_STRAIGHT_PRIORITY)
                    .value("SAME_RIGHT_LEFT_PRIORITY", Proposition::SAME_RIGHT_LEFT_PRIORITY)
                    .value("SAME_RIGHT_RIGHT_PRIORITY", Proposition::SAME_RIGHT_RIGHT_PRIORITY)
                    .value("SAME_RIGHT_STRAIGHT_PRIORITY", Proposition::SAME_RIGHT_STRAIGHT_PRIORITY)
                    .value("SAME_STRAIGHT_LEFT_PRIORITY", Proposition::SAME_STRAIGHT_LEFT_PRIORITY)
                    .value("SAME_STRAIGHT_RIGHT_PRIORITY", Proposition::SAME_STRAIGHT_RIGHT_PRIORITY)
                    .value("SAME_STRAIGHT_STRAIGHT_PRIORITY", Proposition::SAME_STRAIGHT_STRAIGHT_PRIORITY)

                    .value("HAS_LEFT_LEFT_PRIORITY", Proposition::HAS_LEFT_LEFT_PRIORITY)
                    .value("HAS_LEFT_RIGHT_PRIORITY", Proposition::HAS_LEFT_RIGHT_PRIORITY)
                    .value("HAS_LEFT_STRAIGHT_PRIORITY", Proposition::HAS_LEFT_STRAIGHT_PRIORITY)
                    .value("HAS_RIGHT_LEFT_PRIORITY", Proposition::HAS_RIGHT_LEFT_PRIORITY)
                    .value("HAS_RIGHT_RIGHT_PRIORITY", Proposition::HAS_RIGHT_RIGHT_PRIORITY)
                    .value("HAS_RIGHT_STRAIGHT_PRIORITY", Proposition::HAS_RIGHT_STRAIGHT_PRIORITY)
                    .value("HAS_STRAIGHT_LEFT_PRIORITY", Proposition::HAS_STRAIGHT_LEFT_PRIORITY)
                    .value("HAS_STRAIGHT_RIGHT_PRIORITY", Proposition::HAS_STRAIGHT_RIGHT_PRIORITY)
                    .value("HAS_STRAIGHT_STRAIGHT_PRIORITY", Proposition::HAS_STRAIGHT_STRAIGHT_PRIORITY)

                    .value("OTHER_HAS_LEFT_LEFT_PRIORITY", Proposition::OTHER_HAS_LEFT_LEFT_PRIORITY)
                    .value("OTHER_HAS_LEFT_RIGHT_PRIORITY", Proposition::OTHER_HAS_LEFT_RIGHT_PRIORITY)
                    .value("OTHER_HAS_LEFT_STRAIGHT_PRIORITY", Proposition::OTHER_HAS_LEFT_STRAIGHT_PRIORITY)
                    .value("OTHER_HAS_RIGHT_LEFT_PRIORITY", Proposition::OTHER_HAS_RIGHT_LEFT_PRIORITY)
                    .value("OTHER_HAS_RIGHT_RIGHT_PRIORITY", Proposition::OTHER_HAS_RIGHT_RIGHT_PRIORITY)
                    .value("OTHER_HAS_RIGHT_STRAIGHT_PRIORITY", Proposition::OTHER_HAS_RIGHT_STRAIGHT_PRIORITY)
                    .value("OTHER_HAS_STRAIGHT_LEFT_PRIORITY", Proposition::OTHER_HAS_STRAIGHT_LEFT_PRIORITY)
                    .value("OTHER_HAS_STRAIGHT_RIGHT_PRIORITY", Proposition::OTHER_HAS_STRAIGHT_RIGHT_PRIORITY)
                    .value("OTHER_HAS_STRAIGHT_STRAIGHT_PRIORITY", Proposition::OTHER_HAS_STRAIGHT_STRAIGHT_PRIORITY)

                    .def_static("to_string", &knowledge_extraction::proposition::to_string)
                    .def_static("from_string", &knowledge_extraction::proposition::from_string)
                    .def_static("proposition_to_string",
                                [](const Proposition &prop) {
                                    return knowledge_extraction::proposition::proposition_to_string.at(prop);
                                })
                    .def_static("string_to_proposition", [](const std::string &prop) {
                        return knowledge_extraction::proposition::string_to_proposition.at(prop);
                    });
    for (const auto &[prop_enum, prop_string] : knowledge_extraction::proposition::proposition_to_string) {
        prop.def_static(
            prop_string.c_str(),
            [prop_enum](std::optional<size_t> obstacle_id) {
                return knowledge_extraction::proposition::to_string(prop_enum, obstacle_id);
            },
            "obstacle_id"_a = std::nullopt);
    }
}

void export_ego_parameters(const nb::module_ &module) {
    nb::class_<EgoParameters>(module, "EgoParameters")
        .def(
            "__init__",
            [](EgoParameters *self, std::optional<double> a_lon_min, std::optional<double> a_lon_max,
               std::optional<double> a_lat_min, std::optional<double> a_lat_max, std::optional<double> v_lon_min,
               std::optional<double> v_lon_max, std::optional<double> v_lat_min, std::optional<double> v_lat_max,
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
                new (self) EgoParameters{params};
            },
            "a_lon_min"_a = std::nullopt, "a_lon_max"_a = std::nullopt, "a_lat_min"_a = std::nullopt,
            "a_lat_max"_a = std::nullopt, "v_lon_min"_a = std::nullopt, "v_lon_max"_a = std::nullopt,
            "v_lat_min"_a = std::nullopt, "v_lat_max"_a = std::nullopt, "initial_state"_a = std::nullopt,
            "uncertainty_p_lon"_a = std::nullopt, "uncertainty_p_lat"_a = std::nullopt,
            "uncertainty_v_lon"_a = std::nullopt, "uncertainty_v_lat"_a = std::nullopt, "length"_a = std::nullopt,
            "width"_a = std::nullopt, "t_react"_a = std::nullopt)
        .def_rw("a_lon_min", &EgoParameters::a_lon_min)
        .def_rw("a_lon_max", &EgoParameters::a_lon_max)
        .def_rw("a_lat_min", &EgoParameters::a_lat_min)
        .def_rw("a_lat_max", &EgoParameters::a_lat_max)
        .def_rw("v_lon_min", &EgoParameters::v_lon_min)
        .def_rw("v_lon_max", &EgoParameters::v_lon_max)
        .def_rw("v_lat_min", &EgoParameters::v_lat_min)
        .def_rw("v_lat_max", &EgoParameters::v_lat_max)
        .def_prop_rw(
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
        .def_rw("uncertainty_p_lon", &EgoParameters::uncertainty_p_lon)
        .def_rw("uncertainty_p_lat", &EgoParameters::uncertainty_p_lat)
        .def_rw("uncertainty_v_lon", &EgoParameters::uncertainty_v_lon)
        .def_rw("uncertainty_v_lat", &EgoParameters::uncertainty_v_lat)
        .def_rw("length", &EgoParameters::length)
        .def_rw("width", &EgoParameters::width)
        .def_rw("t_react", &EgoParameters::t_react);
}
