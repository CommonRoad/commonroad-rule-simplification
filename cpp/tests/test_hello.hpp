#pragma once

#include <gtest/gtest.h>

class HelloTest : public testing::Test {
  protected:
    std::string substr{"Hello"};
};
