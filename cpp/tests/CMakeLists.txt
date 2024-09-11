set(CR_KNOWLEDGE_EXTRACTION_TEST_SRC_FILES
        test_hello.cpp

        relationship/implication/test_in_front_of_impl_extractor.cpp

        test_envs/test_envs.cpp
)

add_executable(cr_knowledge_extraction_test
        ${CR_KNOWLEDGE_EXTRACTION_TEST_SRC_FILES}
        all_tests.cpp
)

target_include_directories(cr_knowledge_extraction_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(cr_knowledge_extraction_test PRIVATE
        cr_knowledge_extraction
        GTest::gtest
        GTest::gmock
)

target_precompile_headers(cr_knowledge_extraction_test PRIVATE
        <gtest/gtest.h>
)

include(GoogleTest)
# queries the compiled executable for tests, this requires the executable to be runnable. if you are
# cross compiling, make sure to properly set CMAKE_CROSSCOMPILING_EMULATOR.
gtest_discover_tests(
        cr_knowledge_extraction_test
        # TEST_PREFIX "./"
        # increase the discovery timeout for `cr_knowledge_extraction_test --gtest_list_tests`
        DISCOVERY_TIMEOUT 60
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports
)
