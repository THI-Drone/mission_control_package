#pragma once

#include <string>
#include <stdexcept>
#include <memory>
#include <nlohmann/json.hpp>

namespace Command
{
    class BaseCommand
    {
    public:
        static constexpr const char *type_waypoint = "waypoint";
        static constexpr const char *type_detect_marker = "detect_marker";
        static constexpr const char *type_drop_payload = "drop_payload";
        static constexpr const char *type_end_mission = "end_mission";

    public:
        static std::unique_ptr<BaseCommand> create_command(const std::string &type, const std::string &data);

        virtual const char* get_type() = 0;
    
    protected:
        virtual void parse_json(const std::string &data_str) = 0;  /// Parse the json string and check for errors. If everything is fine, store json in data.
    };

    class Waypoint : public BaseCommand
    {
    private:
        nlohmann::json data;
        static const nlohmann::json data_template;

        void parse_json(const std::string &data_str) override;
        // TODO implement parser in CommonNode and use this one instead

    public:
        Waypoint(const std::string &data_str)
        {
            parse_json(data_str);
        }

        const char* get_type() override
        {
            return BaseCommand::type_waypoint;
        }
    };
}
