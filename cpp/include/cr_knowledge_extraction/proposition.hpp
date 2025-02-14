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
    IN_INTERSECTION_CONFLICT_AREA,
    OTHER_IN_INTERSECTION_CONFLICT_AREA,
    CAUSES_BRAKING_INTERSECTION,

    TURNING_LEFT,
    TURNING_RIGHT,
    GOING_STRAIGHT,
    OTHER_TURNING_LEFT,
    OTHER_TURNING_RIGHT,
    OTHER_GOING_STRAIGHT,

    SAME_LEFT_LEFT_PRIORITY,
    SAME_LEFT_RIGHT_PRIORITY,
    SAME_LEFT_STRAIGHT_PRIORITY,
    SAME_RIGHT_LEFT_PRIORITY,
    SAME_RIGHT_RIGHT_PRIORITY,
    SAME_RIGHT_STRAIGHT_PRIORITY,
    SAME_STRAIGHT_LEFT_PRIORITY,
    SAME_STRAIGHT_RIGHT_PRIORITY,
    SAME_STRAIGHT_STRAIGHT_PRIORITY,

    HAS_LEFT_LEFT_PRIORITY,
    HAS_LEFT_RIGHT_PRIORITY,
    HAS_LEFT_STRAIGHT_PRIORITY,
    HAS_RIGHT_LEFT_PRIORITY,
    HAS_RIGHT_RIGHT_PRIORITY,
    HAS_RIGHT_STRAIGHT_PRIORITY,
    HAS_STRAIGHT_LEFT_PRIORITY,
    HAS_STRAIGHT_RIGHT_PRIORITY,
    HAS_STRAIGHT_STRAIGHT_PRIORITY,

    OTHER_HAS_LEFT_LEFT_PRIORITY,
    OTHER_HAS_LEFT_RIGHT_PRIORITY,
    OTHER_HAS_LEFT_STRAIGHT_PRIORITY,
    OTHER_HAS_RIGHT_LEFT_PRIORITY,
    OTHER_HAS_RIGHT_RIGHT_PRIORITY,
    OTHER_HAS_RIGHT_STRAIGHT_PRIORITY,
    OTHER_HAS_STRAIGHT_LEFT_PRIORITY,
    OTHER_HAS_STRAIGHT_RIGHT_PRIORITY,
    OTHER_HAS_STRAIGHT_STRAIGHT_PRIORITY,
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
constexpr auto in_intersection_conflict_area = "InIntersectionConflictArea";
constexpr auto other_in_intersection_conflict_area = "OtherInIntersectionConflictArea";
constexpr auto causes_braking_intersection = "CausesBrakingIntersection";

constexpr auto turning_left = "TurningLeft";
constexpr auto turning_right = "TurningRight";
constexpr auto going_straight = "GoingStraight";
constexpr auto other_turning_left = "OtherTurningLeft";
constexpr auto other_turning_right = "OtherTurningRight";
constexpr auto other_going_straight = "OtherGoingStraight";

constexpr auto same_left_left_priority = "SameLeftLeftPriority";
constexpr auto same_left_right_priority = "SameLeftRightPriority";
constexpr auto same_left_straight_priority = "SameLeftStraightPriority";
constexpr auto same_right_left_priority = "SameRightLeftPriority";
constexpr auto same_right_right_priority = "SameRightRightPriority";
constexpr auto same_right_straight_priority = "SameRightStraightPriority";
constexpr auto same_straight_left_priority = "SameStraightLeftPriority";
constexpr auto same_straight_right_priority = "SameStraightRightPriority";
constexpr auto same_straight_straight_priority = "SameStraightStraightPriority";

constexpr auto has_left_left_priority = "HasLeftLeftPriority";
constexpr auto has_left_right_priority = "HasLeftRightPriority";
constexpr auto has_left_straight_priority = "HasLeftStraightPriority";
constexpr auto has_right_left_priority = "HasRightLeftPriority";
constexpr auto has_right_right_priority = "HasRightRightPriority";
constexpr auto has_right_straight_priority = "HasRightStraightPriority";
constexpr auto has_straight_left_priority = "HasStraightLeftPriority";
constexpr auto has_straight_right_priority = "HasStraightRightPriority";
constexpr auto has_straight_straight_priority = "HasStraightStraightPriority";

constexpr auto other_has_left_left_priority = "OtherHasLeftLeftPriority";
constexpr auto other_has_left_right_priority = "OtherHasLeftRightPriority";
constexpr auto other_has_left_straight_priority = "OtherHasLeftStraightPriority";
constexpr auto other_has_right_left_priority = "OtherHasRightLeftPriority";
constexpr auto other_has_right_right_priority = "OtherHasRightRightPriority";
constexpr auto other_has_right_straight_priority = "OtherHasRightStraightPriority";
constexpr auto other_has_straight_left_priority = "OtherHasStraightLeftPriority";
constexpr auto other_has_straight_right_priority = "OtherHasStraightRightPriority";
constexpr auto other_has_straight_straight_priority = "OtherHasStraightStraightPriority";

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
    {Proposition::IN_INTERSECTION_CONFLICT_AREA, in_intersection_conflict_area},
    {Proposition::OTHER_IN_INTERSECTION_CONFLICT_AREA, other_in_intersection_conflict_area},
    {Proposition::CAUSES_BRAKING_INTERSECTION, causes_braking_intersection},

    {Proposition::TURNING_LEFT, turning_left},
    {Proposition::TURNING_RIGHT, turning_right},
    {Proposition::GOING_STRAIGHT, going_straight},
    {Proposition::OTHER_TURNING_LEFT, other_turning_left},
    {Proposition::OTHER_TURNING_RIGHT, other_turning_right},
    {Proposition::OTHER_GOING_STRAIGHT, other_going_straight},

    {Proposition::SAME_LEFT_LEFT_PRIORITY, same_left_left_priority},
    {Proposition::SAME_LEFT_RIGHT_PRIORITY, same_left_right_priority},
    {Proposition::SAME_LEFT_STRAIGHT_PRIORITY, same_left_straight_priority},
    {Proposition::SAME_RIGHT_LEFT_PRIORITY, same_right_left_priority},
    {Proposition::SAME_RIGHT_RIGHT_PRIORITY, same_right_right_priority},
    {Proposition::SAME_RIGHT_STRAIGHT_PRIORITY, same_right_straight_priority},
    {Proposition::SAME_STRAIGHT_LEFT_PRIORITY, same_straight_left_priority},
    {Proposition::SAME_STRAIGHT_RIGHT_PRIORITY, same_straight_right_priority},
    {Proposition::SAME_STRAIGHT_STRAIGHT_PRIORITY, same_straight_straight_priority},

    {Proposition::HAS_LEFT_LEFT_PRIORITY, has_left_left_priority},
    {Proposition::HAS_LEFT_RIGHT_PRIORITY, has_left_right_priority},
    {Proposition::HAS_LEFT_STRAIGHT_PRIORITY, has_left_straight_priority},
    {Proposition::HAS_RIGHT_LEFT_PRIORITY, has_right_left_priority},
    {Proposition::HAS_RIGHT_RIGHT_PRIORITY, has_right_right_priority},
    {Proposition::HAS_RIGHT_STRAIGHT_PRIORITY, has_right_straight_priority},
    {Proposition::HAS_STRAIGHT_LEFT_PRIORITY, has_straight_left_priority},
    {Proposition::HAS_STRAIGHT_RIGHT_PRIORITY, has_straight_right_priority},
    {Proposition::HAS_STRAIGHT_STRAIGHT_PRIORITY, has_straight_straight_priority},

    {Proposition::OTHER_HAS_LEFT_LEFT_PRIORITY, other_has_left_left_priority},
    {Proposition::OTHER_HAS_LEFT_RIGHT_PRIORITY, other_has_left_right_priority},
    {Proposition::OTHER_HAS_LEFT_STRAIGHT_PRIORITY, other_has_left_straight_priority},
    {Proposition::OTHER_HAS_RIGHT_LEFT_PRIORITY, other_has_right_left_priority},
    {Proposition::OTHER_HAS_RIGHT_RIGHT_PRIORITY, other_has_right_right_priority},
    {Proposition::OTHER_HAS_RIGHT_STRAIGHT_PRIORITY, other_has_right_straight_priority},
    {Proposition::OTHER_HAS_STRAIGHT_LEFT_PRIORITY, other_has_straight_left_priority},
    {Proposition::OTHER_HAS_STRAIGHT_RIGHT_PRIORITY, other_has_straight_right_priority},
    {Proposition::OTHER_HAS_STRAIGHT_STRAIGHT_PRIORITY, other_has_straight_straight_priority},
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
    {in_intersection_conflict_area, Proposition::IN_INTERSECTION_CONFLICT_AREA},
    {other_in_intersection_conflict_area, Proposition::OTHER_IN_INTERSECTION_CONFLICT_AREA},
    {causes_braking_intersection, Proposition::CAUSES_BRAKING_INTERSECTION},

    {turning_left, Proposition::TURNING_LEFT},
    {turning_right, Proposition::TURNING_RIGHT},
    {going_straight, Proposition::GOING_STRAIGHT},
    {other_turning_left, Proposition::OTHER_TURNING_LEFT},
    {other_turning_right, Proposition::OTHER_TURNING_RIGHT},
    {other_going_straight, Proposition::OTHER_GOING_STRAIGHT},

    {same_left_left_priority, Proposition::SAME_LEFT_LEFT_PRIORITY},
    {same_left_right_priority, Proposition::SAME_LEFT_RIGHT_PRIORITY},
    {same_left_straight_priority, Proposition::SAME_LEFT_STRAIGHT_PRIORITY},
    {same_right_left_priority, Proposition::SAME_RIGHT_LEFT_PRIORITY},
    {same_right_right_priority, Proposition::SAME_RIGHT_RIGHT_PRIORITY},
    {same_right_straight_priority, Proposition::SAME_RIGHT_STRAIGHT_PRIORITY},
    {same_straight_left_priority, Proposition::SAME_STRAIGHT_LEFT_PRIORITY},
    {same_straight_right_priority, Proposition::SAME_STRAIGHT_RIGHT_PRIORITY},
    {same_straight_straight_priority, Proposition::SAME_STRAIGHT_STRAIGHT_PRIORITY},

    {has_left_left_priority, Proposition::HAS_LEFT_LEFT_PRIORITY},
    {has_left_right_priority, Proposition::HAS_LEFT_RIGHT_PRIORITY},
    {has_left_straight_priority, Proposition::HAS_LEFT_STRAIGHT_PRIORITY},
    {has_right_left_priority, Proposition::HAS_RIGHT_LEFT_PRIORITY},
    {has_right_right_priority, Proposition::HAS_RIGHT_RIGHT_PRIORITY},
    {has_right_straight_priority, Proposition::HAS_RIGHT_STRAIGHT_PRIORITY},
    {has_straight_left_priority, Proposition::HAS_STRAIGHT_LEFT_PRIORITY},
    {has_straight_right_priority, Proposition::HAS_STRAIGHT_RIGHT_PRIORITY},
    {has_straight_straight_priority, Proposition::HAS_STRAIGHT_STRAIGHT_PRIORITY},

    {other_has_left_left_priority, Proposition::OTHER_HAS_LEFT_LEFT_PRIORITY},
    {other_has_left_right_priority, Proposition::OTHER_HAS_LEFT_RIGHT_PRIORITY},
    {other_has_left_straight_priority, Proposition::OTHER_HAS_LEFT_STRAIGHT_PRIORITY},
    {other_has_right_left_priority, Proposition::OTHER_HAS_RIGHT_LEFT_PRIORITY},
    {other_has_right_right_priority, Proposition::OTHER_HAS_RIGHT_RIGHT_PRIORITY},
    {other_has_right_straight_priority, Proposition::OTHER_HAS_RIGHT_STRAIGHT_PRIORITY},
    {other_has_straight_left_priority, Proposition::OTHER_HAS_STRAIGHT_LEFT_PRIORITY},
    {other_has_straight_right_priority, Proposition::OTHER_HAS_STRAIGHT_RIGHT_PRIORITY},
    {other_has_straight_straight_priority, Proposition::OTHER_HAS_STRAIGHT_STRAIGHT_PRIORITY},
};

/**
 * Convert a proposition to its string representation.
 *
 * @param proposition The proposition to convert.
 * @param parameter The optional predicate parameter.
 * @return The string representation of the proposition.
 */
std::string to_string(knowledge_extraction::Proposition proposition, std::optional<size_t> parameter);

/**
 * Parse a proposition with parameter from a string.
 *
 * @param proposition The proposition string.
 * @return The parsed proposition and its parameter (if it has one).
 */
std::pair<Proposition, std::optional<size_t>> from_string(const std::string &proposition);
} // namespace proposition
} // namespace knowledge_extraction
