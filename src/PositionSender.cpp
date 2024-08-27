#include "PositionSender.h"

PositionSender::PositionSender() : sending_positions(false) {}

PositionSender::~PositionSender()
{
    stopSendingPositions();
}

void PositionSender::loadPositions()
{
    loadPositionsFromFile(TRUCK1_WAYPOINTS_FILE, positions[1]);
    loadPositionsFromFile(TRUCK2_WAYPOINTS_FILE, positions[2]);
}

void PositionSender::loadPositionsFromFile(const std::string &filename, std::vector<Eigen::VectorXd> &robot_positions)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return;
    }

    robot_positions.clear();
    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        int truck_id;
        double x, z, vx, vz;
        if (iss >> truck_id >> x >> z >> vx >> vz)
        {
            Eigen::VectorXd position(5);
            position << truck_id, x, z, vx, vz;
            robot_positions.push_back(position);
        }
        else
        {
            std::cerr << "Error reading line: " << line << std::endl;
        }
    }

    std::cout << "Loaded " << robot_positions.size() << " positions from " << filename << std::endl;
}

void PositionSender::startSendingPositions()
{
    if (sending_positions)
    {
        std::cout << "Already sending positions." << std::endl;
        return;
    }

    sending_positions = true;
    position_thread = std::thread(&PositionSender::sendPositionsThread, this);
}

void PositionSender::stopSendingPositions()
{
    sending_positions = false;
    if (position_thread.joinable())
    {
        position_thread.join();
    }
}

std::map<int, Eigen::Vector4d> PositionSender::getLatestPositions()
{
    std::lock_guard<std::mutex> lock(positions_mutex);
    return latest_positions;
}

void PositionSender::setRobotFailurePoint(int robot_id, int failure_point)
{
    failure_points[robot_id] = failure_point;
}

void PositionSender::clearRobotFailurePoint(int robot_id)
{
    failure_points.erase(robot_id);
}

bool PositionSender::hasFailurePoint(int robot_id) const
{
    return failure_points.find(robot_id) != failure_points.end();
}

void PositionSender::sendPositionsThread()
{
    std::map<int, size_t> indices;

    for (const auto &pair : positions)
    {
        indices[pair.first] = 0;
        robot_failed[pair.first] = false;
    }

    while (sending_positions)
    {
        bool all_finished = true;

        for (auto &pair : positions)
        {
            int robot_id = pair.first;
            auto &robot_positions = pair.second;

            if (indices[robot_id] < robot_positions.size() && !robot_failed[robot_id])
            {
                updateLatestPosition(robot_id, robot_positions[indices[robot_id]]);
                indices[robot_id]++;

                // Check if we've reached the failure point for this robot
                auto failure_it = failure_points.find(robot_id);
                if (failure_it != failure_points.end() && indices[robot_id] >= static_cast<size_t>(failure_it->second))
                {
                    std::cout << "Simulating communication failure for robot " << robot_id << " at position " << indices[robot_id] << std::endl;
                    robot_failed[robot_id] = true;
                }

                all_finished = false;
            }
        }

        if (all_finished)
        {
            std::cout << "All positions sent or all robots failed. Stopping." << std::endl;
            sending_positions = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 10 Hz // 30 fps = 33ms
    }
}

bool PositionSender::isRobotFailed(int robot_id) const
{
    auto it = robot_failed.find(robot_id);
    return (it != robot_failed.end()) && it->second;
}

void PositionSender::updateLatestPosition(int robot_id, const Eigen::VectorXd &position)
{
    std::lock_guard<std::mutex> lock(positions_mutex);
    latest_positions[robot_id] = Eigen::Vector4d(position[1], position[2], position[3], position[4]);
}