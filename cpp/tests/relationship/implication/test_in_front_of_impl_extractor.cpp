#include "test_in_front_of_impl_extractor.hpp"

#include "cr_knowledge_extraction/relationship/implication/in_front_of_impl_extractor.hpp"

#include <gmock/gmock.h>

using namespace knowledge_extraction::relationship::implication;
using knowledge_extraction::relationship::RelationshipType;

using testing::UnorderedElementsAreArray;

TEST_F(InFrontOfImplExtractorTest, InterstateSimple) {
    auto extractor = InFrontOfImplExtractor{test_envs.interstate_simple};
    auto relevant_obstacle_ids_over_time = std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>{
        {0, {100, 101, 102, 103, 104, 105}},
        {1, {100, 101, 102, 104, 105}},
        {39, {100, 101, 102, 103, 104, 105}},
    };
    auto implications_over_time = extractor.extract(relevant_obstacle_ids_over_time);

    EXPECT_THAT(implications_over_time.at(0), UnorderedElementsAreArray({
                                                  std::tuple{RelationshipType::IMPLICATION, 100, 101},
                                                  {RelationshipType::IMPLICATION, 101, 102},
                                                  {RelationshipType::IMPLICATION, 102, 103},
                                                  {RelationshipType::IMPLICATION, 103, 104},
                                                  {RelationshipType::IMPLICATION, 104, 105},
                                              }));
    EXPECT_THAT(implications_over_time.at(1), UnorderedElementsAreArray({
                                                  std::tuple{RelationshipType::IMPLICATION, 100, 101},
                                                  {RelationshipType::IMPLICATION, 101, 102},
                                                  {RelationshipType::IMPLICATION, 102, 104},
                                                  {RelationshipType::IMPLICATION, 104, 105},
                                              }));
    EXPECT_THAT(implications_over_time.at(39), UnorderedElementsAreArray({
                                                   std::tuple{RelationshipType::IMPLICATION, 100, 102},
                                                   {RelationshipType::IMPLICATION, 102, 101},
                                                   {RelationshipType::IMPLICATION, 101, 103},
                                                   {RelationshipType::IMPLICATION, 103, 104},
                                                   {RelationshipType::IMPLICATION, 104, 105},
                                               }));
    EXPECT_TRUE(implications_over_time[2].empty());
}
