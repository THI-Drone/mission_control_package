#pragma once

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>
#include <nlohmann/json.hpp>

#include "common_package/commands.hpp"

namespace mission_file_lib{
class MissionDefinitionReader
{
public:
    MissionDefinitionReader() = default;

    /**
     * @brief Reads a mission definition file and performs validation checks.
     *
     * This function reads a mission definition file located at the specified file path.
     * It performs various validation checks on the JSON content of the file to ensure
     * that it conforms to the expected structure and values.
     *
     * @param file_path The path to the mission definition file.
     * @param dry_run Flag indicating whether to perform a dry run without storing the result.
     *
     * @throws std::runtime_error if the file fails to open, parse, or if the JSON content is invalid.
     *
     * @note This function stores the result in an internal data structure for future use, unless it is a dry run.
     */
    void read_file(const std::string &file_path, const bool dry_run);
};
}
