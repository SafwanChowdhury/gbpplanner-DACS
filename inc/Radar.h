#pragma once

#include <vector>
#include <map>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <Eigen/Dense>
#include "json.hpp"
#include <iostream>

struct ServerInfo
{
    std::string ip;
    int port;
    int sockfd;
};

class Radar
{
public:
    Radar();
    ~Radar();

    void addServer(const std::string &server_ip, int server_port);
    void start();
    void stop();
    void setZeroPoint();
    std::map<std::string, Eigen::Vector2d> getLatestCoordinates();

private:
    void connectWebSocket(ServerInfo &server);
    void readWebSocket(ServerInfo &server);
    void processTruckData(const std::string &server_id, const nlohmann::json &data);
    std::string generateWebSocketKey();
    bool performWebSocketHandshake(int sockfd, const std::string &host, int port);
    void processCoordinates(const std::string &server_id, double originalX, double originalZ);
    void handleConnectionFailure(ServerInfo &server);

    std::vector<ServerInfo> servers;
    Eigen::Vector2d zero_point;
    std::map<std::string, Eigen::Vector2d> latest_coordinates;
    std::mutex data_mutex;
    std::atomic<bool> running;
    std::vector<std::thread> server_threads;
    std::map<std::string, Eigen::Vector2d> raw_coordinates;
    const double zoom_factor = 3.44; // new_image/old_image = 1000/688 = 1.453   old_scaling/new_scaling = 5/1.453 = 3.44
};