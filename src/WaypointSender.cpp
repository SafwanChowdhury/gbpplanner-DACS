#include "WaypointSender.h"

WaypointSender::WaypointSender() : sending_waypoints(false) {}

WaypointSender::~WaypointSender()
{
    stopSendingWaypoints();
}

void WaypointSender::loadWaypoints()
{
    loadWaypointsFromFile(TRUCK1_WAYPOINTS_FILE, waypoints[1]);
    loadWaypointsFromFile(TRUCK2_WAYPOINTS_FILE, waypoints[2]);
}

void WaypointSender::loadWaypointsFromFile(const std::string &filename, std::vector<Eigen::VectorXd> &robot_waypoints)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return;
    }

    robot_waypoints.clear();
    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        int truck_id;
        double x, z, vx, vz;
        if (iss >> truck_id >> x >> z >> vx >> vz)
        {
            Eigen::VectorXd waypoint(5);
            waypoint << truck_id, x, z, vx, vz;
            robot_waypoints.push_back(waypoint);
        }
        else
        {
            std::cerr << "Error reading line: " << line << std::endl;
        }
    }

    std::cout << "Loaded " << robot_waypoints.size() << " waypoints from " << filename << std::endl;
}

void WaypointSender::startSendingWaypoints()
{
    if (sending_waypoints)
    {
        std::cout << "Already sending waypoints." << std::endl;
        return;
    }

    sending_waypoints = true;
    waypoint_thread = std::thread(&WaypointSender::sendWaypointsThread, this);
}

void WaypointSender::stopSendingWaypoints()
{
    sending_waypoints = false;
    if (waypoint_thread.joinable())
    {
        waypoint_thread.join();
    }
}

std::map<int, Eigen::Vector4d> WaypointSender::getLatestWaypoints()
{
    std::lock_guard<std::mutex> lock(waypoints_mutex);
    return latest_waypoints;
}

void WaypointSender::setRobotFailurePoint(int robot_id, int failure_point)
{
    failure_points[robot_id] = failure_point;
}

void WaypointSender::clearRobotFailurePoint(int robot_id)
{
    failure_points.erase(robot_id);
}

bool WaypointSender::hasFailurePoint(int robot_id) const
{
    return failure_points.find(robot_id) != failure_points.end();
}

void WaypointSender::sendWaypointsThread()
{
    std::map<int, size_t> indices;

    for (const auto &pair : waypoints)
    {
        indices[pair.first] = 0;
        robot_failed[pair.first] = false;
    }

    while (sending_waypoints)
    {
        bool all_finished = true;

        for (auto &pair : waypoints)
        {
            int robot_id = pair.first;
            auto &robot_waypoints = pair.second;

            if (indices[robot_id] < robot_waypoints.size() && !robot_failed[robot_id])
            {
                updateLatestWaypoint(robot_id, robot_waypoints[indices[robot_id]]);
                indices[robot_id]++;

                // Check if we've reached the failure point for this robot
                auto failure_it = failure_points.find(robot_id);
                if (failure_it != failure_points.end() && indices[robot_id] >= static_cast<size_t>(failure_it->second))
                {
                    std::cout << "Simulating communication failure for robot " << robot_id << " at waypoint " << indices[robot_id] << std::endl;
                    robot_failed[robot_id] = true;
                }

                all_finished = false;
            }
        }

        if (all_finished)
        {
            std::cout << "All waypoints sent or all robots failed. Stopping." << std::endl;
            sending_waypoints = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 20 Hz
    }
}

bool WaypointSender::isRobotFailed(int robot_id) const
{
    auto it = robot_failed.find(robot_id);
    return (it != robot_failed.end()) && it->second;
}

void WaypointSender::updateLatestWaypoint(int robot_id, const Eigen::VectorXd &waypoint)
{
    std::lock_guard<std::mutex> lock(waypoints_mutex);
    latest_waypoints[robot_id] = Eigen::Vector4d(waypoint[1], waypoint[2], waypoint[3], waypoint[4]);
}