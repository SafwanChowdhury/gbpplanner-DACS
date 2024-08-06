#include "WaypointSender.h"

WaypointSender::WaypointSender() : sending_waypoints(false), robot2_failure_point(-1), robot2_failed(false) {}

WaypointSender::~WaypointSender()
{
    stopSendingWaypoints();
}

void WaypointSender::loadWaypoints()
{
    loadWaypointsFromFile(TRUCK1_WAYPOINTS_FILE, truck1_waypoints);
    loadWaypointsFromFile(TRUCK2_WAYPOINTS_FILE, truck2_waypoints);
}

void WaypointSender::loadWaypointsFromFile(const std::string &filename, std::vector<Eigen::VectorXd> &waypoints)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return;
    }

    waypoints.clear();
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
            waypoints.push_back(waypoint);
        }
        else
        {
            std::cerr << "Error reading line: " << line << std::endl;
        }
    }

    std::cout << "Loaded " << waypoints.size() << " waypoints from " << filename << std::endl;
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

void WaypointSender::setRobot2FailurePoint(int failure_point)
{
    robot2_failure_point = failure_point;
}

void WaypointSender::sendWaypointsThread()
{
    size_t truck1_index = 0, truck2_index = 0;
    robot2_failed = false;
    while (sending_waypoints)
    {
        if (truck1_index < truck1_waypoints.size())
        {
            updateLatestWaypoint(1, truck1_waypoints[truck1_index]);
            truck1_index++;
        }

        if (truck2_index < truck2_waypoints.size() && !robot2_failed)
        {
            updateLatestWaypoint(2, truck2_waypoints[truck2_index]);
            truck2_index++;

            // Check if we've reached the failure point for robot 2
            if (robot2_failure_point >= 0 && truck2_index >= static_cast<size_t>(robot2_failure_point))
            {
                std::cout << "Simulating communication failure for robot 2 at waypoint " << truck2_index << std::endl;
                robot2_failed = true;
            }
        }

        if (truck1_index >= truck1_waypoints.size() && (truck2_index >= truck2_waypoints.size() || robot2_failed))
        {
            std::cout << "All waypoints sent or robot 2 failed. Stopping." << std::endl;
            sending_waypoints = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 20 Hz
    }
}

void WaypointSender::updateLatestWaypoint(int truck_id, const Eigen::VectorXd &waypoint)
{
    std::lock_guard<std::mutex> lock(waypoints_mutex);
    latest_waypoints[truck_id] = Eigen::Vector4d(waypoint[1], waypoint[2], waypoint[3], waypoint[4]);
}