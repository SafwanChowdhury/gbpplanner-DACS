/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <Globals.h>
#include <Utils.h>
#include "json.hpp"

/*****************************************************************/
// Simply reads the appropriate sections from the config.json
/*****************************************************************/
void Globals::parse_global_args(std::ifstream &config_file)
{

    // Basic parameters
    nlohmann::json j;
    config_file >> j;
    ASSETS_DIR = j["ASSETS_DIR"];

    // Display parameters
    DISPLAY = static_cast<bool>((int)j["DISPLAY"]);
    ;
    WORLD_SZ = j["WORLD_SZ"];
    SCREEN_SZ = j["SCREEN_SZ"];
    DRAW_INTERROBOT = static_cast<bool>((int)j["DRAW_INTERROBOT"]);
    DRAW_PATH = static_cast<bool>((int)j["DRAW_PATH"]);
    DRAW_WAYPOINTS = static_cast<bool>((int)j["DRAW_WAYPOINTS"]);

    // Simulation parameters
    SEED = j["SEED"];
    TIMESTEP = j["TIMESTEP"];
    MAX_TIME = j["MAX_TIME"];
    NUM_ROBOTS = j["NUM_ROBOTS"];
    T_HORIZON = j["T_HORIZON"];
    ROBOT_RADIUS = j["ROBOT_RADIUS"];
    COMMUNICATION_RADIUS = j["COMMUNICATION_RADIUS"];
    MAX_SPEED = j["MAX_SPEED"];
    COMMS_FAILURE_RATE = j["COMMS_FAILURE_RATE"];
    FORMATION = j["FORMATION"];
    OBSTACLE_FILE = j["OBSTACLE_FILE"];

    // GBP parameters
    SIGMA_FACTOR_DYNAMICS = j["SIGMA_FACTOR_DYNAMICS"];
    SIGMA_FACTOR_INTERROBOT = j["SIGMA_FACTOR_INTERROBOT"];
    SIGMA_FACTOR_OBSTACLE = j["SIGMA_FACTOR_OBSTACLE"];
    NUM_ITERS = j["NUM_ITERS"];

    USE_RADAR = j["USE_RADAR"];
    WAYPOINT_RADIUS = j["WAYPOINT_RADIUS"];
}

Globals::Globals() {};

/*****************************************************************/
// Allows for parsing of an external config file
/*****************************************************************/
int Globals::parse_global_args(int argc, char **argv)
{
    bool cfg_file_set = false;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--cfg" && i + 1 < argc)
        {
            this->CONFIG_FILE = argv[i + 1];
            cfg_file_set = true;
            ++i; // Skip the next argument as we've already processed it
        }
        else if (arg == "--radar-ip" && i + 1 < argc)
        {
            RADAR_IPS.push_back(argv[i + 1]);
            ++i; // Skip the next argument as we've already processed it
        }
    }

    if (!cfg_file_set)
    {
        std::cout << "Error: Config file not specified. Use --cfg <config_file_path>" << std::endl;
        return EXIT_FAILURE;
    }

    std::ifstream my_config_file(CONFIG_FILE);
    if (!my_config_file)
    {
        std::cout << "Error: Couldn't find the config file: " << CONFIG_FILE << std::endl;
        return EXIT_FAILURE;
    }

    parse_global_args(my_config_file);
    post_parsing();

    return EXIT_SUCCESS;
}

/*****************************************************************/
// Any checks on the input configs should go here.
/*****************************************************************/
void Globals::post_parsing()
{
    // Cap max speed, since it should be <= ROBOT_RADIUS/2.f / TIMESTEP:
    // In one timestep a robot should not move more than half of its radius
    // (since we plan for discrete timesteps)
    if (MAX_SPEED > ROBOT_RADIUS / 2.f / TIMESTEP)
    {
        MAX_SPEED = ROBOT_RADIUS / 2.f / TIMESTEP;
        print("Capping MAX_SPEED parameter at ", MAX_SPEED);
    }
    T0 = ROBOT_RADIUS / 2.f / MAX_SPEED; // Time between current state and next state of planned path

    if (!RADAR_IPS.empty())
    {
        print("Parsed Radar IPs:");
        for (const auto &ip : RADAR_IPS)
        {
            print(" - ", ip);
        }
    }
    else
    {
        print("No Radar IPs provided.");
    }
}
