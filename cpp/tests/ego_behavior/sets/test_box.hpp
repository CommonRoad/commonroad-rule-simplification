#pragma once

#include "cr_knowledge_extraction/ego_behavior/sets/box.hpp"

#include <gtest/gtest.h>

class BoxTest : public testing::Test {
  protected:
    knowledge_extraction::ego_behavior::sets::Box<2> box2d{{10, 10}, {1, 2}};
    knowledge_extraction::ego_behavior::sets::Box<4> box4d{{10, 10, 10, 10}, {1, 2, 3, 4}};
};
