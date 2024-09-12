#pragma once

#include <optional>
#include <string>
#include <unordered_map>

namespace knowledge_extraction {
enum class Proposition {
    IN_SAME_LANE,
    KEEPS_SAFE_DISTANCE_PREC,
    IN_FRONT_OF,
};

namespace proposition {
constexpr auto in_same_lane = "InSameLane";
constexpr auto keeps_safe_distance_prec = "KeepsSafeDistancePrec";
constexpr auto in_front_of = "InFrontOf";

const std::unordered_map<Proposition, std::string> proposition_to_string = {
    {Proposition::IN_SAME_LANE, in_same_lane},
    {Proposition::KEEPS_SAFE_DISTANCE_PREC, keeps_safe_distance_prec},
    {Proposition::IN_FRONT_OF, in_front_of},
};

const std::unordered_map<std::string, Proposition> string_to_proposition = {
    {in_same_lane, Proposition::IN_SAME_LANE},
    {keeps_safe_distance_prec, Proposition::KEEPS_SAFE_DISTANCE_PREC},
    {in_front_of, Proposition::IN_FRONT_OF},
};

std::string to_string(knowledge_extraction::Proposition proposition, std::optional<size_t> parameter);

std::pair<Proposition, std::optional<size_t>> from_string(const std::string &proposition);
} // namespace proposition
} // namespace knowledge_extraction
