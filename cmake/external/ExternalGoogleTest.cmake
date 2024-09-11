include(FetchContent)
include(utils/FetchContentHelper)

FetchContent_Declare_Fallback(
        googletest

        SYSTEM

        # NOTE - URL download should be preferred:
        # HTTP downloads are faster than Git clones and therefore reduce configuration time

        # Git version for reference
        #GIT_REPOSITORY https://github.com/google/googletest.git
        #GIT_TAG        release-1.12.1

        URL https://github.com/google/googletest/archive/refs/tags/release-1.12.1.tar.gz
        URL_HASH SHA256=81964fe578e9bd7c94dfdb09c8e4d6e6759e19967e397dbea48d1c10e45d0df2

        FIND_PACKAGE_ARGS NAMES GTest
)
set(INSTALL_GTEST OFF)
FetchContent_MakeAvailable(googletest)

if (NOT googletest_FOUND AND NOT GTest_FOUND)
    # Add google-test
    mark_as_advanced(
            BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
            gmock_build_tests gtest_build_samples gtest_build_tests
            gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
    )

    if (TARGET gtest AND TARGET gtest_main AND TARGET gmock AND TARGET gmock_main)
        set_target_properties(gtest PROPERTIES FOLDER extern)
        set_target_properties(gtest_main PROPERTIES FOLDER extern)
        set_target_properties(gmock PROPERTIES FOLDER extern)
        set_target_properties(gmock_main PROPERTIES FOLDER extern)
    else ()
        message(AUTHOR_WARNING "Could not find local GTest targets, please report this")
    endif ()
endif ()
