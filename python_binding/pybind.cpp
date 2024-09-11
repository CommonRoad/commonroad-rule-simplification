#include "pybind.hpp"

#include "cr_knowledge_extraction/hello.hpp"

PYBIND11_MODULE(cr_knowledge_extraction_core, module) {
    // Import the Python bindings of the curvilinear coordinate system to ensure that the necessary types are bound
    py::module_::import("commonroad_dc.pycrccosy");

    module.doc() = "C++ extension for commonroad-knowledge-extraction.";

    module.def("hello", &knowledge_extraction::hello);
}
