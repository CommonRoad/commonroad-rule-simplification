#include "test_hello.hpp"

#include "cr_knowledge_extraction/hello.hpp"

TEST_F(HelloTest, Substring) {
    auto str = knowledge_extraction::hello();
    EXPECT_TRUE(str.find(substr) != std::string::npos);
}
