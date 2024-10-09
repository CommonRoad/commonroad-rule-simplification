#pragma once

#include <optional>
#include <string>
#include <unordered_map>

namespace knowledge_extraction {
enum class Proposition {
    IN_SAME_LANE,
    KEEPS_SAFE_DISTANCE_PREC,
    CUT_IN,
    IN_FRONT_OF,
    ON_MAIN_CARRIAGEWAY,
    ON_MAIN_CARRIAGEWAY_RIGHT_LANE,
    OTHER_ON_ACCESS_RAMP,
    OTHER_ON_MAIN_CARRIAGEWAY,
    IN_INTERSECTION,
    AT_STOP_SIGN,
    STOP_LINE_IN_FRONT,
    RELEVANT_TRAFFIC_LIGHT,
    IN_STANDSTILL,
    ON_INCOMING_LEFT_OF,
    ON_ONCOMING_OF,
    TURNING_LEFT,
    TURNING_RIGHT,
    GOING_STRAIGHT,
    OTHER_TURNING_LEFT,
    OTHER_TURNING_RIGHT,
    OTHER_GOING_STRAIGHT,
};

namespace proposition {
constexpr auto in_same_lane = "InSameLane";
constexpr auto keeps_safe_distance_prec = "KeepsSafeDistancePrec";
constexpr auto cut_in = "CutIn";
constexpr auto in_front_of = "InFrontOf";
constexpr auto on_main_carriageway = "OnMainCarriageway";
constexpr auto on_main_carriageway_right_lane = "OnMainCarriagewayRightLane";
constexpr auto other_on_access_ramp = "OtherOnAccessRamp";
constexpr auto other_on_main_carriageway = "OtherOnMainCarriageway";
constexpr auto in_intersection = "InIntersection";
constexpr auto at_stop_sign = "AtStopSign";
constexpr auto stop_line_in_front = "StopLineInFront";
constexpr auto relevant_traffic_light = "RelevantTrafficLight";
constexpr auto in_standstill = "InStandstill";
constexpr auto on_incoming_left_of = "OnIncomingLeftOf";
constexpr auto on_oncoming_of = "OnOncomingOf";
constexpr auto turning_left = "TurningLeft";
constexpr auto turning_right = "TurningRight";
constexpr auto going_straight = "GoingStraight";
constexpr auto other_turning_left = "OtherTurningLeft";
constexpr auto other_turning_right = "OtherTurningRight";
constexpr auto other_going_straight = "OtherGoingStraight";

const std::unordered_map<Proposition, std::string> proposition_to_string = {
    {Proposition::IN_SAME_LANE, in_same_lane},
    {Proposition::KEEPS_SAFE_DISTANCE_PREC, keeps_safe_distance_prec},
    {Proposition::CUT_IN, cut_in},
    {Proposition::IN_FRONT_OF, in_front_of},
    {Proposition::ON_MAIN_CARRIAGEWAY, on_main_carriageway},
    {Proposition::ON_MAIN_CARRIAGEWAY_RIGHT_LANE, on_main_carriageway_right_lane},
    {Proposition::OTHER_ON_ACCESS_RAMP, other_on_access_ramp},
    {Proposition::OTHER_ON_MAIN_CARRIAGEWAY, other_on_main_carriageway},
    {Proposition::IN_INTERSECTION, in_intersection},
    {Proposition::AT_STOP_SIGN, at_stop_sign},
    {Proposition::STOP_LINE_IN_FRONT, stop_line_in_front},
    {Proposition::RELEVANT_TRAFFIC_LIGHT, relevant_traffic_light},
    {Proposition::IN_STANDSTILL, in_standstill},
    {Proposition::ON_INCOMING_LEFT_OF, on_incoming_left_of},
    {Proposition::ON_ONCOMING_OF, on_oncoming_of},
    {Proposition::TURNING_LEFT, turning_left},
    {Proposition::TURNING_RIGHT, turning_right},
    {Proposition::GOING_STRAIGHT, going_straight},
    {Proposition::OTHER_TURNING_LEFT, other_turning_left},
    {Proposition::OTHER_TURNING_RIGHT, other_turning_right},
    {Proposition::OTHER_GOING_STRAIGHT, other_going_straight},
};

const std::unordered_map<std::string, Proposition> string_to_proposition = {
    {in_same_lane, Proposition::IN_SAME_LANE},
    {keeps_safe_distance_prec, Proposition::KEEPS_SAFE_DISTANCE_PREC},
    {cut_in, Proposition::CUT_IN},
    {in_front_of, Proposition::IN_FRONT_OF},
    {on_main_carriageway, Proposition::ON_MAIN_CARRIAGEWAY},
    {on_main_carriageway_right_lane, Proposition::ON_MAIN_CARRIAGEWAY_RIGHT_LANE},
    {other_on_access_ramp, Proposition::OTHER_ON_ACCESS_RAMP},
    {other_on_main_carriageway, Proposition::OTHER_ON_MAIN_CARRIAGEWAY},
    {in_intersection, Proposition::IN_INTERSECTION},
    {at_stop_sign, Proposition::AT_STOP_SIGN},
    {stop_line_in_front, Proposition::STOP_LINE_IN_FRONT},
    {relevant_traffic_light, Proposition::RELEVANT_TRAFFIC_LIGHT},
    {in_standstill, Proposition::IN_STANDSTILL},
    {on_incoming_left_of, Proposition::ON_INCOMING_LEFT_OF},
    {on_oncoming_of, Proposition::ON_ONCOMING_OF},
    {turning_left, Proposition::TURNING_LEFT},
    {turning_right, Proposition::TURNING_RIGHT},
    {going_straight, Proposition::GOING_STRAIGHT},
    {other_turning_left, Proposition::OTHER_TURNING_LEFT},
    {other_turning_right, Proposition::OTHER_TURNING_RIGHT},
    {other_going_straight, Proposition::OTHER_GOING_STRAIGHT},
};

std::string to_string(knowledge_extraction::Proposition proposition, std::optional<size_t> parameter);

std::pair<Proposition, std::optional<size_t>> from_string(const std::string &proposition);
} // namespace proposition
} // namespace knowledge_extraction
