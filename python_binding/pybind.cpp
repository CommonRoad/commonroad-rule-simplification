#include "pybind.hpp"

#include "helper/world.hpp"

#include "cr_knowledge_extraction/extraction_interface.hpp"
#include "cr_knowledge_extraction/hello.hpp"

#include <pybind11/stl.h>

PYBIND11_MODULE(cr_knowledge_extraction_core, module) {
    // Import the Python bindings of the curvilinear coordinate system to ensure that the necessary types are bound
    py::module_::import("commonroad_dc.pycrccosy");

    module.doc() = "C++ extension for commonroad-knowledge-extraction.";

    module.def("hello", &knowledge_extraction::hello);
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
        .def(py::init([](const std::string &scenario_path,
                         const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) {
                 auto world = pybind_helper::open_world(scenario_path);
                 return knowledge_extraction::ExtractionInterface{std::move(world), ego_ccs};
             }),
             py::arg("scenario_path"), py::arg("ego_ccs"))
        .def("extract", &knowledge_extraction::ExtractionInterface::extract);
}
