include(utils/EnsureStatic)

# Sanity check: Ensure we are building all dependencies as static libraries
if (SKBUILD)
    if (SKBUILD_SELF_CONTAINED)
        # If we are building all dependencies ourselves, then they should
        # all be static.

        ensure_all_static(cr_knowledge_extraction)
    else ()
        # If we are partially using system libraries, then only the cr_knowledge_extraction library
        # itself has to be static.

        ensure_static(cr_knowledge_extraction)
    endif ()
endif ()

if (NOT SKBUILD)
    find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)
    message(STATUS "Adding Python interface for compilation only")
endif ()

set(python_module_name cr_knowledge_extraction_core)
nanobind_add_module(${python_module_name} MODULE
        pybind.hpp
        pybind.cpp
)

target_precompile_headers(nanobind-static
        PUBLIC
        <nanobind/nanobind.h>
)

nanobind_add_stub(
        ${python_module_name}_stub
        MODULE ${python_module_name}
        OUTPUT ${python_module_name}.pyi
        PYTHON_PATH $<TARGET_FILE_DIR:${python_module_name}>
        DEPENDS ${python_module_name}
        MARKER_FILE py.typed
        VERBOSE
)

install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${python_module_name}.pyi
        ${CMAKE_CURRENT_BINARY_DIR}/py.typed
        DESTINATION cr_knowledge_extraction
        COMPONENT PythonModules
)

target_link_libraries(${python_module_name} PRIVATE cr_knowledge_extraction)

install(TARGETS ${python_module_name}
        LIBRARY DESTINATION cr_knowledge_extraction
        COMPONENT ${python_module_name}
)

add_custom_target(install-python-modules-${python_module_name}
        ${CMAKE_COMMAND}
        -DCMAKE_INSTALL_COMPONENT=${python_module_name}
        -P "${PROJECT_BINARY_DIR}/cmake_install.cmake"
        DEPENDS ${python_module_name}
)
