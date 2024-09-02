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

class PositionSender
{
public:
    PositionSender();
    ~PositionSender();

    void loadPositions();
    void startSendingPositions();
    void stopSendingPositions();
    std::map<int, Eigen::Vector4d> getLatestPositions();
    void setRobotFailurePoint(int robot_id, int failure_point);
    void clearRobotFailurePoint(int robot_id);
    bool isRobotFailed(int robot_id) const;
    std::map<int, size_t> start_indices;
    void setStartingIndex(int robot_id, size_t starting_index);

private:
    const std::string TRUCK1_WAYPOINTS_FILE = "../assets/scripts/Truck1Waypoints.txt";
    const std::string TRUCK2_WAYPOINTS_FILE = "../assets/scripts/Truck2Waypoints.txt";

    std::map<int, std::vector<Eigen::VectorXd>> positions;
    std::map<int, int> failure_points;
    std::map<int, Eigen::Vector4d> latest_positions;
    std::map<int, bool> robot_failed;

    std::thread position_thread;
    std::mutex positions_mutex;
    std::atomic<bool> sending_positions;

    void loadPositionsFromFile(const std::string &filename, std::vector<Eigen::VectorXd> &robot_positions);
    void sendPositionsThread();
    void updateLatestPosition(int robot_id, const Eigen::VectorXd &position);
    bool hasFailurePoint(int robot_id) const;
};