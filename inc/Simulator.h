/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once

#include <map>
#include <memory>
#include <algorithm>
#include <Utils.h>
#include <tuple>
#include <gbp/GBPCore.h>
#include <Graphics.h>
#include <gbp/Variable.h>
#include <nanoflann.h>
#include <raylib.h>
#include <rlights.h>
#include <nanoflann.h>
#include <KDTreeMapOfVectorsAdaptor.h>
#include <random>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>
#include "Radar.h"
#include "PositionSender.h"

class Robot;
class Graphics;
class TreeOfRobots;
struct TruckData
{
    double x;
    double y;
    double vx;
    double vy;
};

/************************************************************************************/
// The main Simulator. This is where the magic happens.
/************************************************************************************/
class Simulator
{
public:
    friend class Robot;
    friend class Factor;

    // Constructor
    Simulator(const std::vector<std::string> &radarIPs = std::vector<std::string>());
    ~Simulator();

    // Pointer to Graphics class which hold all the camera, graphics and models for display
    Graphics *graphics;

    // kd-tree to store the positions of the robots at each timestep.
    // This is used for calculating the neighbours of robots blazingly fast.
    typedef KDTreeMapOfVectorsAdaptor<std::map<int, std::vector<double>>> KDTree;
    std::map<int, std::vector<double>> robot_positions_{{0, {0., 0.}}};
    KDTree *treeOfRobots_;

    // Image representing the obstacles in the environment
    Image obstacleImg;

    int next_rid_ = 0;                             // New robots will use this rid. It should be ++ incremented when this happens
    int next_vid_ = 0;                             // New variables will use this vid. It should be ++ incremented when this happens
    int next_fid_ = 0;                             // New factors will use this fid. It should be ++ incremented when this happens
    uint32_t clock_ = 0;                           // Simulation clock (timesteps)
    std::map<int, std::shared_ptr<Robot>> robots_; // Map containing smart pointers to all robots, accessed by their rid.
    bool new_robots_needed_ = true;                // Whether or not to create new robots. (Some formations are dynamicaly changing)
    bool leader_init_ = false;                     // Whether or not the leader has been initialised
    bool symmetric_factors = false;                // If true, when inter-robot factors need to be created between two robots,
                                                   // a pair of factors is created (one belonging to each robot). This becomes a redundancy.

    /*******************************************************************************/
    // Create new robots if needed. Handles deletion of robots out of bounds.
    // New formations must modify the vectors "robots to create" and optionally "robots_to_delete"
    // by appending (push_back()) a shared pointer to a Robot class.
    /*******************************************************************************/
    void createOrDeleteRobots();

    /*******************************************************************************/
    // Set a proportion of robots to not perform inter-robot communications
    /*******************************************************************************/
    void setCommsFailure(float failure_rate = globals.COMMS_FAILURE_RATE);

    /*******************************************************************************/
    // Timestep loop of simulator.
    /*******************************************************************************/
    void timestep();

    /*******************************************************************************/
    // Drawing graphics.
    /*******************************************************************************/
    void draw();

    /*******************************************************************************/
    // Read the coordinates of the robots from a file.
    /*******************************************************************************/
    void readCoordinatesFromFile();

    /*******************************************************************************/
    // Use a kd-tree to perform a radius search for neighbours of a robot within comms. range
    // (Updates the neighbours_ of a robot)
    /*******************************************************************************/
    void calculateRobotNeighbours(std::map<int, std::shared_ptr<Robot>> &robots);

    /*******************************************************************************/
    // Handles keypresses and mouse input, and updates camera.
    /*******************************************************************************/
    void eventHandler();

    /*******************************************************************************/
    // Update the position of the robot in the simulator's robot_positions_ map.
    /*******************************************************************************/
    void updateRobotPosition(int robotIndex, double x, double y, double vx, double vy);

    /*******************************************************************************/
    // Deletes the robot from the simulator's robots_, as well as any variable/factors associated.
    /*******************************************************************************/
    void deleteRobot(std::shared_ptr<Robot> robot);

    /***************************************************************************************************************/
    // RANDOM NUMBER GENERATOR.
    // Usage: random_number("normal", mean, sigma) or random_number("uniform", lower, upper)
    /***************************************************************************************************************/
    std::mt19937 gen_normal = std::mt19937(globals.SEED);
    std::mt19937 gen_uniform = std::mt19937(globals.SEED);
    std::mt19937 gen_uniform_int = std::mt19937(globals.SEED);
    template <typename T>
    T random_number(std::string distribution, T param1, T param2)
    {
        if (distribution == "normal")
            return std::normal_distribution<T>(param1, param2)(gen_normal);
        if (distribution == "uniform")
            return std::uniform_real_distribution<T>(param1, param2)(gen_uniform);
        return (T)0;
    }
    int random_int(int lower, int upper)
    {
        return std::uniform_int_distribution<int>(lower, upper)(gen_uniform_int);
    }

    Radar radar;
    PositionSender position_sender;

    void updateRobotsFromRadar();
    void handleWaypointsAndMergePoints(std::shared_ptr<Robot> &robot, int robotIndex);
    std::vector<std::tuple<double, double, double, double, double, double, double, std::string>> getIterationValues() const;
    void sendIterationValues(const std::vector<std::tuple<double, double, double, double, double, double, double, std::string>> &values);
    void updateReceivedTruckData(int rid, double x, double y, double vx, double vy);
    void initializeRobotMapping();
    int mapHostToRobot(const std::string &host_id);
    std::string getHostIdForRobot(int robot_id) const;
    void printRouteTimes();

    struct SimulationData
    {
        int time_step;
        std::vector<std::tuple<int, int, double, double, double>> robot_data;
    };

    std::vector<SimulationData> simulation_data;

    void collectSimulationData();
    void exportSimulationData();

private:
    std::map<int, TruckData> receivedTruckData;
    std::map<std::string, int> server_to_robot_map;
    std::map<int, std::string> robot_to_server_map;
    std::map<std::string, int> host_to_robot_map;
    std::map<int, std::string> robot_to_host_map;
    int next_robot_id = 1;
    bool override_cruise_control = false;
    std::set<int> missing_robots;
    std::map<int, Eigen::Vector4d> last_coords;
};
