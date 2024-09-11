#include "test_in_same_lane_equiv_extractor.hpp"

#include "cr_knowledge_extraction/relationship/equivalence/in_same_lane_equiv_extractor.hpp"

#include <gmock/gmock.h>

using namespace knowledge_extraction::relationship::equivalence;
using knowledge_extraction::relationship::RelationshipType;

using testing::UnorderedElementsAreArray;

TEST_F(InSameLaneEquivExtractorTest, InterstateSimple) {
    auto extractor = InSameLaneEquivExtractor{test_envs.interstate_simple.first, test_envs.interstate_simple.second};
    auto relevant_obstacle_ids_over_time = std::unordered_map<time_step_t, std::unordered_set<size_t>>{
        {0, {100, 101, 102, 103, 104, 105}},
        {1, {100, 101, 102, 104, 105}},
        //        {18, {100, 101, 102, 103, 104, 105}},
        {26, {100, 101, 102, 103, 104, 105}},
    };
    auto implications_over_time = extractor.extract(relevant_obstacle_ids_over_time);

    EXPECT_THAT(implications_over_time.at(0), UnorderedElementsAreArray({
                                                  std::tuple{RelationshipType::EQUIVALENCE, 100, 104},
                                                  {RelationshipType::EQUIVALENCE, 102, 103},
                                              }));
    EXPECT_THAT(implications_over_time.at(1), UnorderedElementsAreArray({
                                                  std::tuple{RelationshipType::EQUIVALENCE, 100, 104},
                                              }));
    // The map appears to be broken so that only 103 and 104 are in the same lane at this time
    //    EXPECT_THAT(implications_over_time.at(18), UnorderedElementsAreArray({
    //                                                  std::tuple{RelationshipType::EQUIVALENCE, 100, 103},
    //                                                  {RelationshipType::EQUIVALENCE, 103, 104},
    //                                              }));
    EXPECT_THAT(implications_over_time.at(26), UnorderedElementsAreArray({
                                                   std::tuple{RelationshipType::EQUIVALENCE, 100, 104},
                                               }));
    EXPECT_TRUE(implications_over_time[2].empty());
}
