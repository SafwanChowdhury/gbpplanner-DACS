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
    std::map<int, Eigen::Vector2d> getLatestWaypoints();

private:
    const std::string TRUCK1_WAYPOINTS_FILE = "../assets/scripts/Truck1Waypoints.txt";
    const std::string TRUCK2_WAYPOINTS_FILE = "../assets/scripts/Truck2Waypoints.txt";

    std::vector<Eigen::Vector3d> truck1_waypoints;
    std::vector<Eigen::Vector3d> truck2_waypoints;
    std::atomic<bool> sending_waypoints;
    std::thread waypoint_thread;
    std::map<int, Eigen::Vector2d> latest_waypoints;
    std::mutex waypoints_mutex;

    void sendWaypointsThread();
    void loadWaypointsFromFile(const std::string &filename, std::vector<Eigen::Vector3d> &waypoints);
    void updateLatestWaypoint(int truck_id, const Eigen::Vector3d &waypoint);
};

#endif // WAYPOINT_SENDER_H