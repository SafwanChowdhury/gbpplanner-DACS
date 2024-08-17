#pragma once

#include <string>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <atomic>
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>

class WaypointSender
{
public:
    WaypointSender();
    ~WaypointSender();

    void loadWaypoints();
    void startSendingWaypoints();
    void stopSendingWaypoints();
    std::map<int, Eigen::Vector4d> getLatestWaypoints();
    void setRobotFailurePoint(int robot_id, int failure_point);
    void clearRobotFailurePoint(int robot_id);
    bool isRobotFailed(int robot_id) const;

private:
    const std::string TRUCK1_WAYPOINTS_FILE = "../assets/scripts/Truck1Waypoints.txt";
    const std::string TRUCK2_WAYPOINTS_FILE = "../assets/scripts/Truck2Waypoints.txt";

    std::map<int, std::vector<Eigen::VectorXd>> waypoints;
    std::map<int, int> failure_points;
    std::map<int, Eigen::Vector4d> latest_waypoints;
    std::map<int, bool> robot_failed;

    std::thread waypoint_thread;
    std::mutex waypoints_mutex;
    std::atomic<bool> sending_waypoints;

    void loadWaypointsFromFile(const std::string &filename, std::vector<Eigen::VectorXd> &robot_waypoints);
    void sendWaypointsThread();
    void updateLatestWaypoint(int robot_id, const Eigen::VectorXd &waypoint);
    bool hasFailurePoint(int robot_id) const;
};