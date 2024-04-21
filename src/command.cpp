#include "command.hpp"

using namespace Command;

std::unique_ptr<BaseCommand> BaseCommand::create_command(const std::string &type, const std::string &data)
{
    if (type == type_waypoint)
    {
        return std::make_unique<Waypoint>(data);
    }
    else if (type == type_detect_marker)
    {
        // TODO add detect marker class
    }
    else if (type == type_drop_payload)
    {
        // TODO add drop payload class
    }
    else if (type == type_end_mission)
    {
        // TODO add end mission class
    }

    throw std::runtime_error("Unknown type: " + type);
}

const nlohmann::json Waypoint::data_template = {};

void Waypoint::parse_json(const std::string &data_str)
{
    nlohmann::json candidate;

    // Parse string as JSON
    try
    {
        candidate = nlohmann::json::parse(data_str);
    }
    catch (const nlohmann::json::exception &e)
    {
        throw std::runtime_error("Waypoint::parse_json: JSON is invalid");
    }

    //for (auto& [key, val] : candidate.items()) {}
}
