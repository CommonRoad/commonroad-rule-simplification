#include "test_in_same_lane_equiv_extractor.hpp"

#include "cr_knowledge_extraction/relationship/equivalence/in_same_lane_equiv_extractor.hpp"

#include <gmock/gmock.h>

using namespace knowledge_extraction::relationship::equivalence;
using knowledge_extraction::relationship::RelationshipType;

using testing::UnorderedElementsAreArray;

TEST_F(InSameLaneEquivExtractorTest, InterstateSimple) {
    auto extractor = InSameLaneEquivExtractor{test_envs.two_lanes};
    auto relevant_obstacle_ids_over_time = std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>{
        {0, {7, 8, 9}},
        {1, {7, 9}},
        {2, {7, 8}},
    };
    auto implications_over_time = extractor.extract(relevant_obstacle_ids_over_time);

    EXPECT_THAT(implications_over_time.at(0), UnorderedElementsAreArray({
                                                  std::tuple{RelationshipType::EQUIVALENCE, 7, 8},
                                              }));
    EXPECT_TRUE(implications_over_time.at(1).empty());
    EXPECT_THAT(implications_over_time.at(2), UnorderedElementsAreArray({
                                                  std::tuple{RelationshipType::EQUIVALENCE, 7, 8},
                                              }));
    EXPECT_TRUE(implications_over_time[3].empty());
}
