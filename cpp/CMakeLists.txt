set(CR_KNOWLEDGE_EXTRACTION_SRC_FILES
        src/extraction_interface.cpp
        src/proposition.cpp

        src/ego_behavior/behavior_overapproximation.cpp

        src/env_model/env_model.cpp

        src/kleene/braking/safe_distance_extractor.cpp
        src/kleene/ego_independent/ego_independent_extractor.cpp
        src/kleene/general/cut_in_extractor.cpp
        src/kleene/intersection/on_incoming_left_of_extractor.cpp
        src/kleene/intersection/other_turning_extractor.cpp
        src/kleene/position/at_traffic_sign_extractor.cpp
        src/kleene/position/in_front_of_extractor.cpp
        src/kleene/position/in_same_lane_extractor.cpp
        src/kleene/position/on_lanelet_with_type_extractor.cpp
        src/kleene/position/on_main_carriageway_right_lane_extractor.cpp
        src/kleene/position/relevant_traffic_light_extractor.cpp
        src/kleene/regulatory/priority_extractor.cpp

        src/relationship/equivalence/in_intersection_conflict_area_equiv_extractor.cpp
        src/relationship/equivalence/in_same_lane_equiv_extractor.cpp
        src/relationship/implication/in_front_of_impl_extractor.cpp
        src/relationship/implication/safe_distance_impl_extractor.cpp

        src/road_network/curvilinear_road_network.cpp
)

set(CR_KNOWLEDGE_EXTRACTION_HDR_FILES
        include/cr_knowledge_extraction/proposition.hpp

        include/cr_knowledge_extraction/env_model/env_model.hpp

        include/cr_knowledge_extraction/ego_behavior/behavior_overapproximation.hpp
        include/cr_knowledge_extraction/ego_behavior/ego_params.hpp
        include/cr_knowledge_extraction/ego_behavior/sets/box.hpp

        include/cr_knowledge_extraction/kleene/kleene_extractor.hpp
        include/cr_knowledge_extraction/kleene/braking/safe_distance_extractor.hpp
        include/cr_knowledge_extraction/kleene/ego_independent/ego_independent_extractor.hpp
        include/cr_knowledge_extraction/kleene/general/cut_in_extractor.hpp
        include/cr_knowledge_extraction/kleene/intersection/on_incoming_left_of_extractor.hpp
        include/cr_knowledge_extraction/kleene/intersection/other_turning_extractor.hpp
        include/cr_knowledge_extraction/kleene/position/at_traffic_sign_extractor.hpp
        include/cr_knowledge_extraction/kleene/position/in_front_of_extractor.hpp
        include/cr_knowledge_extraction/kleene/position/in_same_lane_extractor.hpp
        include/cr_knowledge_extraction/kleene/position/on_lanelet_with_type_extractor.hpp
        include/cr_knowledge_extraction/kleene/position/on_main_carriageway_right_lane_extractor.hpp
        include/cr_knowledge_extraction/kleene/position/relevant_traffic_light_extractor.hpp
        include/cr_knowledge_extraction/kleene/regulatory/priority_extractor.hpp

        include/cr_knowledge_extraction/relationship/relationship_extractor.hpp
        include/cr_knowledge_extraction/relationship/equivalence/in_intersection_conflict_area_equiv_extractor.hpp
        include/cr_knowledge_extraction/relationship/equivalence/in_same_lane_equiv_extractor.hpp
        include/cr_knowledge_extraction/relationship/implication/in_front_of_impl_extractor.hpp
        include/cr_knowledge_extraction/relationship/implication/safe_distance_impl_extractor.hpp

        include/cr_knowledge_extraction/road_network/curvilinear_lanelet.hpp
        include/cr_knowledge_extraction/road_network/curvilinear_road_network.hpp
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
        PRIVATE
        spdlog::spdlog
)

target_link_libraries(cr_knowledge_extraction
        PUBLIC
        crccosy
        EnvironmentModel::env_model
        Eigen3::Eigen
)

add_library(CommonRoadKnowledgeExtraction::cr_knowledge_extraction ALIAS cr_knowledge_extraction)

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
