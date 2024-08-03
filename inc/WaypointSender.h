#ifndef WAYPOINT_SENDER_H
#define WAYPOINT_SENDER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <map>
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

private:
    const std::string TRUCK1_WAYPOINTS_FILE = "../assets/scripts/Truck1Waypoints.txt";
    const std::string TRUCK2_WAYPOINTS_FILE = "../assets/scripts/Truck2Waypoints.txt";

    std::vector<Eigen::VectorXd> truck1_waypoints;
    std::vector<Eigen::VectorXd> truck2_waypoints;
    std::map<int, Eigen::Vector4d> latest_waypoints;
    std::atomic<bool> sending_waypoints;
    std::thread waypoint_thread;
    std::mutex waypoints_mutex;

    void sendWaypointsThread();
    void loadWaypointsFromFile(const std::string &filename, std::vector<Eigen::VectorXd> &waypoints);
    void updateLatestWaypoint(int truck_id, const Eigen::VectorXd &waypoint);
};

#endif // WAYPOINT_SENDER_H