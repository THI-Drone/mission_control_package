#pragma once

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include <rclcpp/node_options.hpp>

#include "common_package/commands.hpp"

class MissionDefinitionReader
{
public:
    MissionDefinitionReader() = default;

    void read_file(std::string &file_path);
};
