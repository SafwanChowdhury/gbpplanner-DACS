/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <iostream>
#include <gbp/GBPCore.h>
#include <Simulator.h>
#include <Graphics.h>
#include <Robot.h>
#include <nanoflann.h>
#include <fstream>
#include <sstream>
#include "json.hpp"

/*******************************************************************************/
// Raylib setup
/*******************************************************************************/
Simulator::Simulator(const std::vector<std::string> &radarIPs)
    : radar(), position_sender()
{
    SetTraceLogLevel(LOG_ERROR);
    if (globals.USE_RADAR)
    {
        for (const auto &ip : radarIPs)
        {
            radar.addServer(ip, 39846);
        }
        radar.start();

        std::this_thread::sleep_for(std::chrono::seconds(2));

        initializeRobotMapping();
    }
    else
    {
        position_sender.loadPositions();
        position_sender.setStartingIndex(1, 70); // Set starting index for sending positions
        position_sender.setStartingIndex(2, 60); // Set starting index for sending positions

        position_sender.setRobotFailurePoint(2, 110); // Set failure points for specific robots if needed

        position_sender.startSendingPositions();
    }

    if (globals.DISPLAY)
    {
        SetTargetFPS(60);
        InitWindow(globals.SCREEN_SZ, globals.SCREEN_SZ, globals.WINDOW_TITLE);
    }

    // Initialise kdtree for storing robot positions (needed for nearest neighbour check)
    treeOfRobots_ = new KDTree(2, robot_positions_, 50);

    // For display only
    // User inputs an obstacle image where the obstacles are BLACK and background is WHITE.
    obstacleImg = LoadImage(globals.OBSTACLE_FILE.c_str());
    if (obstacleImg.width == 0)
        obstacleImg = GenImageColor(globals.WORLD_SZ, globals.WORLD_SZ, WHITE);

    // However for calculation purposes the image needs to be inverted.
    ImageColorInvert(&obstacleImg);
    graphics = new Graphics(obstacleImg);
};

/*******************************************************************************/
// Destructor
/*******************************************************************************/
Simulator::~Simulator()
{
    exportSimulationData();
    delete treeOfRobots_;
    int n = robots_.size();
    for (int i = 0; i < n; ++i)
        robots_.erase(i);
    if (globals.DISPLAY)
    {
        delete graphics;
        CloseWindow();
    }
    if (globals.USE_RADAR)
    {
        radar.stop();
    }
    else
    {
        position_sender.stopSendingPositions();
    }
};

/*******************************************************************************/
// Drawing graphics.
/*******************************************************************************/
void Simulator::draw()
{
    if (!globals.DISPLAY)
        return;

    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode3D(graphics->camera3d);
    // Draw Ground
    DrawModel(graphics->groundModel_, graphics->groundModelpos_, 1., WHITE);
    // Draw Robots
    for (auto [rid, robot] : robots_)
    {
        robot->draw();
        if (!robot->isMaster_)
        {
            auto master_robot = robots_.find(robot->master_id_);
            if (master_robot != robots_.end())
            {
                // Cast positions to float explicitly
                Vector3 slave_position = {
                    static_cast<float>(robot->position_(0)),
                    static_cast<float>(robot->height_3D_),
                    static_cast<float>(robot->position_(1))};
                Vector3 master_position = {
                    static_cast<float>(master_robot->second->position_(0)),
                    static_cast<float>(master_robot->second->height_3D_),
                    static_cast<float>(master_robot->second->position_(1))};
                // print distance
                // std::cout << "Distance between " << rid << " and " << robot->master_id_ << " is " << (robot->position_.head<2>() - master_robot->second->position_.head<2>()).norm() << std::endl;
                DrawLine3D(slave_position, master_position, DARKGRAY);
            }
        }
    }
    EndMode3D();
    draw_info(clock_);
    EndDrawing();
};

void Simulator::updateRobotPosition(int robotIndex, double x, double y, double vx, double vy)
{
    auto robotIt = robots_.find(robotIndex);
    if (robotIt == robots_.end())
    {
        return; // Robot not found, exit early
    }
    auto &robot = robotIt->second;

    // Check if the robot has failed
    if (!position_sender.isRobotFailed(robotIndex))
    {
        // Update position only if the robot hasn't failed
        robot->position_ = Eigen::Vector4d(x, y, vx, vy);
    }
}

void Simulator::handleWaypointsAndMergePoints(std::shared_ptr<Robot> &robot, int robotIndex)
{
    // Update first waypoint to current position
    if (!robot->waypoints_.empty())
    {
        robot->waypoints_.front() = robot->position_;
    }

    if (robot->group_id_ == 2 && robot->waypoints_.size() >= 2 && !robot->has_merged_)
    {
        // Check if Group 2 robot has reached its merge point (second waypoint)
        Eigen::Vector2d mergePoint = robot->waypoints_[1].head<2>();
        double distance = (robot->position_.head<2>() - mergePoint).norm();
        if (distance <= 10)
        {
            // Group 2 robot has reached the merge point
            robot->waypoints_.erase(robot->waypoints_.begin() + 1);
            robot->has_merged_ = true; // Set a flag to indicate that the robot has merged
            robot->override_cruise_control_ = true;
        }
    }

    // Special handling for robots after they have merged
    if (robot->has_merged_ && robot->master_id_ != -1)
    {
        auto masterIt = robots_.find(robot->master_id_);
        if (masterIt != robots_.end())
        {
            // Clear existing waypoints of the follower
            robot->waypoints_.clear();
            // Add the current position and master's position as waypoints
            robot->waypoints_.push_back(robot->position_);
            robot->waypoints_.push_back(masterIt->second->position_);
        }
    }
}

void Simulator::updateRobotsFromRadar()
{
    if (globals.USE_RADAR)
    {
        auto [coordinates, velocities, routeTimes, routeDistances] = radar.getLatestData();
        auto latencies = radar.getLatestLatencies();

        for (const auto &[host_id, coord] : coordinates)
        {
            int robot_id = mapHostToRobot(host_id);
            auto vel_it = velocities.find(host_id);
            auto latency_it = latencies.find(radar.getServerIdForHost(host_id));

            if (vel_it != velocities.end())
            {
                updateRobotPosition(robot_id, coord.x(), coord.y(), vel_it->second.x(), vel_it->second.y());
                last_coords[robot_id] = Eigen::Vector4d(coord.x(), coord.y(), vel_it->second.x(), vel_it->second.y());

                if (latency_it != latencies.end())
                {
                    // Log or use the latency information as needed
                    double latency = latency_it->second;

                    auto robot_it = robots_.find(robot_id);
                    if (robot_it != robots_.end())
                    {
                        robot_it->second->setLatency(latency);
                    }
                }
            }
            else
            {
                updateRobotPosition(robot_id, coord.x(), coord.y(), 0.0, 0.0);
                last_coords[robot_id] = Eigen::Vector4d(coord.x(), coord.y(), 0.0, 0.0);
            }
            std::string server_id = radar.getServerIdForHost(host_id);
        }
    }
    else
    {
        auto positions = position_sender.getLatestPositions();
        for (const auto &[robot_id, _] : robots_)
        {
            if (positions.find(robot_id) == positions.end())
            {
                missing_robots.insert(robot_id);
                continue;
            }
            auto it = positions.find(robot_id);
            if (it != positions.end())
            {
                const auto &position = it->second;
                updateRobotPosition(robot_id, position[0], position[1], position[2], position[3]);
                last_coords[robot_id] = Eigen::Vector4d(position[0], position[1], position[2], position[3]);
            }
        }
    }
}

void Simulator::initializeRobotMapping()
{
    auto server_order = radar.getServerOrder();
    for (const auto &server_id : server_order)
    {
        if (radar.hasReceivedHostId(server_id))
        {
            std::string host_id = radar.getHostIdForServer(server_id);
            if (host_to_robot_map.find(host_id) == host_to_robot_map.end())
            {
                host_to_robot_map[host_id] = next_robot_id;
                robot_to_host_map[next_robot_id] = host_id;
                next_robot_id++;
            }
        }
    }
}

int Simulator::mapHostToRobot(const std::string &host_id)
{
    auto it = host_to_robot_map.find(host_id);
    if (it != host_to_robot_map.end())
    {
        return it->second;
    }
    // If not found, create a new mapping
    int new_robot_id = next_robot_id++;
    host_to_robot_map[host_id] = new_robot_id;
    robot_to_host_map[new_robot_id] = host_id;
    return new_robot_id;
}

std::string Simulator::getHostIdForRobot(int robot_id) const
{
    auto it = robot_to_host_map.find(robot_id);
    return (it != robot_to_host_map.end()) ? it->second : "";
}

std::vector<std::tuple<double, double, double, double, double, double, double, std::string>> Simulator::getIterationValues() const
{
    std::vector<std::tuple<double, double, double, double, double, double, double, std::string>> values;
    for (const auto &[rid, robot] : robots_)
    {
        try
        {
            auto robotData = robot->getData();
            std::string host_id = getHostIdForRobot(rid);
            if (!host_id.empty())
            {
                double x, y, vx, vy;
                auto last_coord_it = last_coords.find(rid);
                if (last_coord_it != last_coords.end())
                {
                    const auto &last_coord = last_coord_it->second;
                    x = last_coord(0);
                    y = last_coord(1);
                    vx = last_coord(2);
                    vy = last_coord(3);
                }
                else
                {
                    // Fallback to robot's current position if not in last_coords
                    x = robot->position_(0);
                    y = robot->position_(1);
                    vx = robot->position_(2);
                    vy = robot->position_(3);
                }

                values.push_back(std::make_tuple(
                    x,                      // x position
                    y,                      // y position
                    vx,                     // x velocity
                    vy,                     // y velocity
                    std::get<0>(robotData), // last_acceleration_
                    std::get<1>(robotData), // last_turn_angle_
                    std::get<2>(robotData), // last_next_speed_
                    host_id                 // unique identifier (host ID)
                    ));
            }
        }
        catch (const std::exception &e)
        {
            continue;
        }
    }
    return values;
}

void Simulator::printRouteTimes()
{
    if (!globals.USE_RADAR)
    {
        return;
    }

    // Get the latest radar data
    auto [coordinates, velocities, routeTimes, routeDistances] = radar.getLatestData();

    for (const auto &[host_id, route_time] : routeTimes)
    {
        // Check if distance information is available
        if (routeDistances.find(host_id) != routeDistances.end())
        {
            double current_distance = routeDistances.at(host_id);
            double remaining_distance = 40890.0 - current_distance; // 40890 meters is the target distance
            double remaining_time = 1913.0 - route_time;            // 1913 seconds is the target time

            // Calculate the required speed in meters per second
            double required_speed_mps = remaining_distance / remaining_time;

            // Convert the required speed from meters per second to miles per hour
            double required_speed_mph = required_speed_mps * 2.23694;

            std::cout << "Truck " << host_id << ": " << route_time << " seconds, Distance: " << current_distance
                      << " meters -> Required Speed: " << required_speed_mph << " mph" << std::endl;
        }
        else
        {
            std::cerr << "No distance data available for truck " << host_id << std::endl;
        }
    }
}

void Simulator::sendIterationValues(const std::vector<std::tuple<double, double, double, double, double, double, double, std::string>> &values)
{
    auto servers = radar.getServers();
    size_t num_servers = servers.size();

    if (num_servers == 0 || values.empty())
    {
        return;
    }

    // Get the latest radar data
    auto [coordinates, velocities, routeTimes, routeDistances] = radar.getLatestData();

    // Prepare data for all trucks using only radar data
    nlohmann::json all_trucks_data;
    for (const auto &[host_id, coord] : coordinates)
    {
        auto vel_it = velocities.find(host_id);
        if (vel_it != velocities.end())
        {
            all_trucks_data.push_back({{"host_id", host_id},
                                       {"position", {{"x", coord.x()}, {"y", coord.y()}}},
                                       {"velocity", {{"x", vel_it->second.x()}, {"y", vel_it->second.y()}}},
                                       {"robot_id", mapHostToRobot(host_id)}});
        }
    }

    const double MERGE_DISTANCE = 40890.0;            // meters
    const double MERGE_TIME = 1913.0;                 // seconds
    const double TARGET_MERGE_SPEED = 60.0 * 0.44704; // 60 mph converted to m/s
    const double FOLLOWER_TIME_GAP = 2.0;             // 2 seconds gap for the follower

    // Set the leader's robot ID here
    const int LEADER_RID = 2; // Change this to the desired leader's robot ID
    std::string leader_host_id;
    double leader_distance = 0;

    // First pass: Identify the leader and its distance
    for (size_t i = 0; i < num_servers && i < values.size(); ++i)
    {
        const auto &[x, y, vx, vy, acceleration, turn_angle, next_speed, host_id] = values[i];
        if (mapHostToRobot(host_id) == LEADER_RID)
        {
            leader_host_id = host_id;
            leader_distance = routeDistances[host_id];
            break;
        }
    }

    // Second pass: Calculate and send data for each truck
    for (size_t i = 0; i < num_servers && i < values.size(); ++i)
    {
        const auto &[x, y, vx, vy, acceleration, turn_angle, next_speed, host_id] = values[i];
        const auto &server = servers[i];

        int robot_id = mapHostToRobot(host_id);
        auto robot_it = robots_.find(robot_id);
        bool robot_override_cruise_control = (robot_it != robots_.end()) ? robot_it->second->override_cruise_control_ : false;

        double current_distance = routeDistances[host_id];
        double remaining_distance = MERGE_DISTANCE - current_distance;
        double remaining_time = MERGE_TIME - routeTimes[host_id];

        double current_speed = std::sqrt(vx * vx + vy * vy);
        double required_speed_mps;

        bool is_leader = (host_id == leader_host_id);

        if (is_leader)
        {
            // Leader logic: aim to reach merge point at 60 mph
            if (remaining_distance > 0 && remaining_time > 0)
            {
                double t = current_distance / MERGE_DISTANCE;
                required_speed_mps = current_speed * (1 - t) + TARGET_MERGE_SPEED * t;

                double estimated_arrival_time = remaining_distance / required_speed_mps;
                if (estimated_arrival_time > remaining_time)
                {
                    required_speed_mps = remaining_distance / remaining_time;
                }
            }
            else
            {
                required_speed_mps = TARGET_MERGE_SPEED;
            }
        }
        else
        {
            // Follower logic: adjust speed to arrive FOLLOWER_TIME_GAP seconds after the leader
            double leader_remaining_distance = MERGE_DISTANCE - leader_distance;
            double time_to_match = remaining_time - FOLLOWER_TIME_GAP;
            if (time_to_match > 0)
            {
                required_speed_mps = remaining_distance / time_to_match;
            }
            else
            {
                required_speed_mps = current_speed; // Maintain current speed if we can't calculate
            }
        }

        double required_speed_mph = required_speed_mps * 2.23694;
        if (robot_override_cruise_control)
        {
            printf("Robot %s overriding cruise control with speed %f\n", host_id.c_str(), next_speed);
        }
        // Get the current time
        double sent_time = std::chrono::duration_cast<std::chrono::duration<double>>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();
        print("time step: ", clock_);
        nlohmann::json json_data = {
            {"iteration_data", {{"host_id", host_id}, {"position", {{"x", x}, {"y", y}}}, {"velocity", {{"x", vx}, {"y", vy}}}, {"acceleration", acceleration}, {"turn_angle", turn_angle}, {"next_speed", next_speed}, {"robot_id", robot_id}, {"timestep", clock_}, {"is_leader", is_leader}, {"override_cruise_control", robot_override_cruise_control}, {"sent_time", sent_time}}},
            {"all_trucks_data", all_trucks_data}};

        std::string json_string = json_data.dump() + "\n";
        radar.sendData(server, json_string);

        // Add this truck's data to all_trucks_data for the next iteration
        all_trucks_data.push_back({{"host_id", host_id},
                                   {"position", {{"x", x}, {"y", y}}},
                                   {"velocity", {{"x", vx}, {"y", vy}}},
                                   {"is_leader", is_leader},
                                   {"required_speed_mph", required_speed_mph}});
    }
}

/*******************************************************************************/
// Timestep loop of simulator.
/*******************************************************************************/
void Simulator::timestep()
{

    if (globals.SIM_MODE != Timestep)
        return;

    updateRobotsFromRadar(); // Update the robots' positions from the radar

    for (auto &[rid, robot] : robots_)
    {
        handleWaypointsAndMergePoints(robot, rid);
    }

    // printRouteTimes();

    // Create and/or destory factors depending on a robot's neighbours
    calculateRobotNeighbours(robots_);

    for (auto &[r_id, robot] : robots_)
    {

        if (robot) // Check if the robot pointer is valid
        {

            robot->updateInterrobotFactors();
            robot->updateMasterSlaveFactors();
        }
    }

    // Update planned paths for all robots
    for (auto &[rid, robot] : robots_)
    {
        if (robot && !position_sender.isRobotFailed(rid))
        {
            robot->updatePlannedPath();
        }
    }

    // If the communications failure rate is non-zero, activate/deactivate robot comms
    setCommsFailure(globals.COMMS_FAILURE_RATE);

    // Perform iterations of GBP. Ideally the internal and external iterations
    // should be interleaved better. Here it is assumed there are an equal number.
    for (int i = 0; i < globals.NUM_ITERS; i++)
    {
        iterateGBP(1, INTERNAL, robots_);
        iterateGBP(1, EXTERNAL, robots_);
    }

    // Update the robot current and horizon states by one timestep
    for (auto &[r_id, robot] : robots_)
    {
        if (robot) // Check if the robot pointer is valid
        {
            robot->updateHorizon();
            robot->updateCurrent();
        }
    }
    auto iterationValues = getIterationValues();
    if (globals.USE_RADAR)
    {
        sendIterationValues(iterationValues);
    }
    collectSimulationData();
    // Increase simulation clock by one timestep
    clock_++;
    if (clock_ >= globals.MAX_TIME)
        globals.RUN = false;
};

/*******************************************************************************/
// Use a kd-tree to perform a radius search for neighbours of a robot within comms. range
// (Updates the neighbours_ of a robot)
/*******************************************************************************/
void Simulator::calculateRobotNeighbours(std::map<int, std::shared_ptr<Robot>> &robots)
{
    for (auto [rid, robot] : robots)
    {
        robot_positions_.at(rid) = std::vector<double>{robot->position_(0), robot->position_(1)};
    }
    treeOfRobots_->index->buildIndex();

    for (auto [rid, robot] : robots)
    {
        // Find nearest neighbors in radius
        robot->neighbours_.clear();
        std::vector<double> query_pt = std::vector<double>{robots[rid]->position_(0), robots[rid]->position_(1)};
        const float search_radius = pow(globals.COMMUNICATION_RADIUS, 2.);
        std::vector<nanoflann::ResultItem<size_t, double>> matches;
        nanoflann::SearchParameters params;
        params.sorted = true;
        const size_t nMatches = treeOfRobots_->index->radiusSearch(&query_pt[0], search_radius, matches, params);
        for (size_t i = 0; i < nMatches; i++)
        {
            auto it = robots_.begin();
            std::advance(it, matches[i].first);
            if (it->first == rid)
                continue;
            robot->neighbours_.push_back(it->first);
        }
    }
};

/*******************************************************************************/
// Set a proportion of robots to not perform inter-robot communications
/*******************************************************************************/
void Simulator::setCommsFailure(float failure_rate)
{
    if (failure_rate == 0)
        return;
    // Get all the robot ids and then shuffle them
    std::vector<int> range{};
    for (auto &[rid, robot] : robots_)
        range.push_back(rid);
    std::shuffle(range.begin(), range.end(), gen_uniform);
    // Set a proportion of the robots as inactive using their interrobot_comms_active_ flag.
    int num_inactive = round(failure_rate * robots_.size());
    for (int i = 0; i < range.size(); i++)
    {
        robots_.at(range[i])->interrobot_comms_active_ = (i >= num_inactive);
    }
}

/*******************************************************************************/
// Handles keypresses and mouse input, and updates camera.
/*******************************************************************************/
void Simulator::eventHandler()
{
    // Deal with Keyboard key press
    int key = GetKeyPressed();
    switch (key)
    {
    case KEY_ESCAPE:
        globals.RUN = false;
        break;
    case KEY_H:
        globals.LAST_SIM_MODE = (globals.SIM_MODE == Help) ? globals.LAST_SIM_MODE : globals.SIM_MODE;
        globals.SIM_MODE = (globals.SIM_MODE == Help) ? globals.LAST_SIM_MODE : Help;
        break;
    case KEY_SPACE:
        graphics->camera_transition_ = !graphics->camera_transition_;
        break;
    case KEY_P:
        globals.DRAW_PATH = !globals.DRAW_PATH;
        break;
    case KEY_R:
        globals.DRAW_INTERROBOT = !globals.DRAW_INTERROBOT;
        break;
    case KEY_W:
        globals.DRAW_WAYPOINTS = !globals.DRAW_WAYPOINTS;
        break;
    case KEY_ENTER:
        globals.SIM_MODE = (globals.SIM_MODE == Timestep) ? SimNone : Timestep;
        break;
    case KEY_Z:
        radar.setZeroPoint();
    default:
        break;
    }

    // Mouse input handling
    Ray ray = GetMouseRay(GetMousePosition(), graphics->camera3d);
    Vector3 mouse_gnd = Vector3Add(ray.position, Vector3Scale(ray.direction, -ray.position.y / ray.direction.y));
    Vector2 mouse_pos{mouse_gnd.x, mouse_gnd.z}; // Position on the ground plane
    // Do stuff with mouse here using mouse_pos .eg:
    // if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)){
    //     do_code
    // }

    // Update the graphics if the camera has moved
    graphics->update_camera();
}

/*******************************************************************************/
// Create new robots if needed. Handles deletion of robots out of bounds.
// New formations must modify the vectors "robots to create" and optionally "robots_to_delete"
// by appending (push_back()) a shared pointer to a Robot class.
/*******************************************************************************/
void Simulator::createOrDeleteRobots()
{
    if (!new_robots_needed_)
        return;

    std::vector<std::shared_ptr<Robot>> robots_to_create{};
    std::vector<std::shared_ptr<Robot>> robots_to_delete{};
    Eigen::VectorXd starting, turning, ending; // Waypoints : [x,y,xdot,ydot].
    int num_robots = globals.USE_RADAR ? radar.getServerCount() : globals.NUM_ROBOTS;
    if (globals.FORMATION == "ets2")
    {
        new_robots_needed_ = false; // We only need to create the robots once
        if (robots_.empty())
        {
            for (int i = 1; i <= num_robots; ++i)
            {
                Eigen::VectorXd initialPosition(4);
                initialPosition << 0., 0., 0., 0.;
                Eigen::VectorXd waypoint(4);
                waypoint << 100., 0., 0., 0.;
                Eigen::VectorXd waypoint2(4);
                waypoint2 << 100., 15., 0., 0.;
                Eigen::VectorXd waypoint3(4);
                waypoint3 << -125., -30., 0., 0.;
                Eigen::VectorXd waypoint4(4);
                waypoint4 << -250., -175., 0., 0.;

                std::deque<Eigen::VectorXd> waypoints;
                waypoints.push_back(initialPosition);

                int master_id;
                bool isMaster;
                int group_id;

                if (i == 1)
                {
                    master_id = -1;
                    isMaster = true;
                    group_id = 1;
                }
                else
                {
                    master_id = i - 1;
                    isMaster = false;
                    group_id = 2;
                }

                if (group_id == 2)
                {
                    waypoints.push_back(initialPosition);
                    waypoints.push_back(waypoint2);
                }
                else
                {
                    waypoints.push_back(initialPosition);
                    waypoints.push_back(waypoint);
                    waypoints.push_back(waypoint3);
                    waypoints.push_back(waypoint4);
                }

                float robot_radius = globals.ROBOT_RADIUS;
                Color robot_color = (group_id == 2) ? DARKBROWN : DARKBLUE; // Group 2: DARKBROWN, Group 1: DARKBLUE

                robots_to_create.push_back(std::make_shared<Robot>(
                    this, i, waypoints, robot_radius, robot_color, isMaster, master_id, group_id));
            }
        }
    }
    else
    {
        print("Shouldn't reach here, formation not defined!");
        // Define new formations here!
    }
    // Create and/or delete the robots as necessary.
    for (auto robot : robots_to_create)
    {
        robot_positions_[robot->rid_] = std::vector<double>{robot->waypoints_[0](0), robot->waypoints_[0](1)};
        robots_[robot->rid_] = robot;
        if (!robot->isMaster_)
            robot->createMasterSlaveFactors();
    };
    for (auto robot : robots_to_delete)
    {
        deleteRobot(robot);
    };
};

/*******************************************************************************/
// Deletes the robot from the simulator's robots_, as well as any variable/factors associated.
/*******************************************************************************/
void Simulator::deleteRobot(std::shared_ptr<Robot> robot)
{
    auto connected_rids_copy = robot->connected_r_ids_;
    for (auto r : connected_rids_copy)
    {
        robot->deleteInterrobotFactors(robots_.at(r));
        robots_.at(r)->deleteInterrobotFactors(robot);
    }
    robots_.erase(robot->rid_);
    robot_positions_.erase(robot->rid_);
}

/*******************************************************************************/
// Testing Functions
/*******************************************************************************/
void Simulator::collectSimulationData()
{
    SimulationData data;
    data.time_step = clock_;

    for (const auto &[rid, robot] : robots_)
    {
        if (robot->master_id_ != -1)
        {
            auto master_robot = robots_.find(robot->master_id_);
            if (master_robot != robots_.end())
            {
                // Calculate distance
                double distance = (robot->position_.head<2>() - master_robot->second->position_.head<2>()).norm();

                // Collect data
                data.robot_data.push_back(std::make_tuple(
                    robot->master_id_,
                    rid,
                    master_robot->second->current_speed_,
                    robot->last_next_speed_,
                    distance));

                // std::cout << "Data collected for master-slave pair: " << robot->master_id_ << "-" << rid
                //           << ", Distance: " << distance << std::endl;
            }
        }
    }

    simulation_data.push_back(data);
    // std::cout << "collectSimulationData called. Time step: " << clock_
    //           << ", Number of master-slave pairs in data: " << data.robot_data.size() << std::endl;
}

void Simulator::exportSimulationData()
{
    auto formatDP = [](double value, int precision)
    {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    };

    // Get current time
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%m%d_%H%M");
    std::string timestamp = ss.str();

    std::string filename = "simulation_data_" + timestamp + "_" +
                           "SM" + formatDP(globals.SIGMA_FACTOR_MASTERSLAVE, 1) + "_" +
                           "MD" + formatDP(globals.MIN_DISTANCE, 0) + "_" +
                           "XD" + formatDP(globals.MAX_DISTANCE, 0) + ".csv";

    std::replace(filename.begin(), filename.end(), ' ', '_');
    std::replace(filename.begin(), filename.end(), ':', '_');
    std::replace(filename.begin(), filename.end(), ',', '_');

    std::ofstream file(filename, std::ios::out | std::ios::trunc);
    if (!file.is_open())
    {
        std::cerr << "Failed to create or open file: " << filename << std::endl;
        return;
    }
    auto replaceAll = [](std::string &str, const std::string &from, const std::string &to)
    {
        size_t startPos = 0;
        while ((startPos = str.find(from, startPos)) != std::string::npos)
        {
            str.replace(startPos, from.length(), to);
            startPos += to.length(); // Move past the last replacement
        }
    };
    auto removeChar = [](std::string &str, char charToRemove)
    {
        str.erase(std::remove(str.begin(), str.end(), charToRemove), str.end());
    };
    std::string name = std::string(filename);
    replaceAll(name, "simulation_data_", "");
    replaceAll(name, timestamp, "");
    removeChar(name, '_');
    replaceAll(name, ".csv", "");
    file << "TimeStep,MasterID,SlaveID,MasterNextSpeed,SlaveNextSpeed,Distance," << name << "\n";
    for (const auto &step_data : simulation_data)
    {
        for (const auto &[master_id, slave_id, master_next_speed, slave_next_speed, distance] : step_data.robot_data)
        {
            file << step_data.time_step << ","
                 << master_id << ","
                 << slave_id << ","
                 << master_next_speed << ","
                 << slave_next_speed << ","
                 << distance << "\n";
        }
    }
    file.close();
    std::cout << "Simulation data exported to " << filename << std::endl;
}