#include "cr_knowledge_extraction/proposition.hpp"

#include <stdexcept>

using namespace knowledge_extraction;

std::pair<Proposition, std::optional<size_t>> proposition::from_string(const std::string &proposition) {
    // Find first opening parenthesis
    auto opening_parenthesis = proposition.find_first_of('(');
    auto has_parameter = opening_parenthesis != std::string::npos;
    if ((!has_parameter && proposition.ends_with(')')) || (has_parameter && !proposition.ends_with(')'))) {
        throw std::invalid_argument("Malformed parameters for proposition: " + proposition);
    }
    // Take substring from beginning to opening parenthesis
    auto proposition_name = has_parameter ? proposition.substr(0, opening_parenthesis) : proposition;
    if (!proposition::string_to_proposition.contains(proposition_name)) {
        throw std::logic_error("Unknown proposition: " + proposition);
    }
    auto proposition_enum = proposition::string_to_proposition.at(proposition_name);

    auto parameter = std::optional<size_t>{};
    if (has_parameter) {
        // Extract parameter from parentheses
        auto parameter_str = proposition.substr(opening_parenthesis + 1, proposition.size() - opening_parenthesis - 2);
        // Parse parameter as size_t
        size_t parameter_value;
        try {
            parameter_value = std::stoul(parameter_str);
        } catch (std::invalid_argument &e) {
            throw std::invalid_argument("Invalid parameter for proposition: " + proposition);
        }
        parameter = parameter_value;
    }

    return {proposition_enum, parameter};
}

std::string proposition::to_string(knowledge_extraction::Proposition proposition, std::optional<size_t> parameter) {
    auto proposition_name = proposition::proposition_to_string.at(proposition);
    if (parameter.has_value()) {
        return proposition_name + "(" + std::to_string(parameter.value()) + ")";
    }
    return proposition_name;
}
