set(CR_KNOWLEDGE_EXTRACTION_SRC_FILES
        src/extraction_interface.cpp
        src/hello.cpp
        src/proposition.cpp

        src/relationship/equivalence/in_same_lane_equiv_extractor.cpp
        src/relationship/implication/in_front_of_impl_extractor.cpp
)

set(CR_KNOWLEDGE_EXTRACTION_HDR_FILES
        include/cr_knowledge_extraction/hello.hpp
        include/cr_knowledge_extraction/proposition.hpp

        include/cr_knowledge_extraction/kleene/kleene_extractor.hpp

        include/cr_knowledge_extraction/relationship/relationship_extractor.hpp
        include/cr_knowledge_extraction/relationship/equivalence/in_same_lane_equiv_extractor.hpp
        include/cr_knowledge_extraction/relationship/implication/in_front_of_impl_extractor.hpp
)

add_library(cr_knowledge_extraction ${CR_KNOWLEDGE_EXTRACTION_SRC_FILES})

target_sources(cr_knowledge_extraction PRIVATE ${CR_KNOWLEDGE_EXTRACTION_HDR_FILES})

if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.23.0)
    target_sources(
            cr_knowledge_extraction
            INTERFACE
            FILE_SET cr_knowledge_extraction_headers
            TYPE HEADERS
            BASE_DIRS include
            FILES ${CR_KNOWLEDGE_EXTRACTION_HDR_FILES}
    )
endif ()

# select C++ standard for this target and all dependent targets
target_compile_features(cr_knowledge_extraction PUBLIC cxx_std_20)

# build position independent code so that we can link this in the Python bindings (builds shared lib)
set_property(TARGET cr_knowledge_extraction PROPERTY POSITION_INDEPENDENT_CODE ON)


target_include_directories(cr_knowledge_extraction
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/cpp/include>
)

target_link_libraries(cr_knowledge_extraction
        PUBLIC
        crccosy
        EnvironmentModel::env_model
        Eigen3::Eigen
)

add_library(CommonRoadKnowledgeExtraction::cr_knowledge_extraction ALIAS cr_knowledge_extraction)

add_executable(cr_knowledge_extraction_executable src/main.cpp)
target_link_libraries(cr_knowledge_extraction_executable PRIVATE cr_knowledge_extraction)

foreach (target IN ITEMS cr_knowledge_extraction)
    # this adds many warnings to the build. They usually help to find some bugs
    # TODO: Check whether each warning flag is actually supported by the compiler before adding it
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wconversion
                -pedantic -Wfatal-errors -Wno-unused-parameter)
    endif ()
    if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        target_compile_options(${target} PRIVATE /W3)
    endif ()
endforeach ()
