#include "test_box.hpp"

using namespace knowledge_extraction::ego_behavior::sets;

TEST_F(BoxTest, InitFromBounds2D) {
    auto box = Box<2>::from_bounds({9, 8}, {11, 12});
    EXPECT_EQ(box, box2d);
}

TEST_F(BoxTest, Bounds2D) {
    auto [min, max] = box2d.bounds();
    EXPECT_EQ(min, (Eigen::Vector2d{{9, 8}}));
    EXPECT_EQ(max, (Eigen::Vector2d{{11, 12}}));
}

TEST_F(BoxTest, Sum2D) {
    auto other_box = Box<2>{{1, 1}, {2, 2}};
    auto sum = box2d.sum(other_box);
    auto expected = Box<2>{{11, 11}, {3, 4}};
    EXPECT_EQ(sum, expected);
}

TEST_F(BoxTest, Shrink2D) {
    auto shrunk = box2d.shrink(3);
    auto expected = Box<2>{{10, 10}, {0, 0.5}};
    EXPECT_EQ(shrunk, expected);
}

TEST_F(BoxTest, LinearMapPositive2D) {
    auto box = Box<2>{{1, 1}, {1, 1}};
    auto matrix = Eigen::Matrix<double, 2, 2>{{1, 2}, {0, 1}};
    auto mapped = box.linear_map_positive(matrix);
    auto expected = Box<2>{{3, 1}, {3, 1}};
    EXPECT_EQ(mapped, expected);
}

TEST_F(BoxTest, InitFromBounds4D) {
    auto box = Box<4>::from_bounds({9, 8, 7, 6}, {11, 12, 13, 14});
    EXPECT_EQ(box, box4d);
}

TEST_F(BoxTest, Bounds4D) {
    auto [min, max] = box4d.bounds();
    EXPECT_EQ(min, (Eigen::Vector4d{{9, 8, 7, 6}}));
    EXPECT_EQ(max, (Eigen::Vector4d{{11, 12, 13, 14}}));
}

TEST_F(BoxTest, Sum4D) {
    auto other_box = Box<4>{{1, 1, 1, 1}, {2, 2, 2, 2}};
    auto sum = box4d.sum(other_box);
    auto expected = Box<4>{{11, 11, 11, 11}, {3, 4, 5, 6}};
    EXPECT_EQ(sum, expected);
}

TEST_F(BoxTest, Shrink4D) {
    auto shrunk = box4d.shrink(3);
    auto expected = Box<4>{{10, 10, 10, 10}, {0, 0.5, 1.5, 2.5}};
    EXPECT_EQ(shrunk, expected);
}

TEST_F(BoxTest, LinearMapPositive4D) {
    auto box = Box<4>{{1, 1, 2, 2}, {1, 1, 2, 2}};
    auto matrix = Eigen::Matrix<double, 4, 4>{{1, 2, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 2}, {0, 0, 0, 1}};
    auto mapped = box.linear_map_positive(matrix);
    auto expected = Box<4>{{3, 1, 6, 2}, {3, 1, 6, 2}};
    EXPECT_EQ(mapped, expected);
}
