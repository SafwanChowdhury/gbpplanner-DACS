#include "Radar.h"
#include <cmath>
#include <chrono>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netdb.h>
#include <random>
#include "json.hpp"

Radar::Radar() : zero_point(-42698, -19546), running(false) {}

Radar::~Radar()
{
    stop();
}

void Radar::addServer(const std::string &server_ip, int server_port)
{
    std::string server_id = server_ip + ":" + std::to_string(server_port);
    servers.push_back({server_ip, server_port, -1});
    server_order.push_back(server_id);
}

std::vector<std::string> Radar::getServerOrder() const
{
    return server_order;
}

void Radar::mapHostIdToServer(const std::string &server_id, const std::string &host_id)
{
    server_to_host_id[server_id] = host_id;
    host_to_server_id[host_id] = server_id;
}

bool Radar::hasReceivedHostId(const std::string &server_id) const
{
    return server_to_host_id.find(server_id) != server_to_host_id.end();
}

size_t Radar::getServerCount() const
{
    return servers.size();
}

void Radar::start()
{
    running = true;
    for (auto &server : servers)
    {
        server_threads.emplace_back(&Radar::connectWebSocket, this, std::ref(server)); // Create a new thread for each server
    }
}

void Radar::stop()
{
    running = false;
    for (auto &server : servers)
    {
        if (server.sockfd != -1) // Close the socket if it's open
        {
            close(server.sockfd); // Close the socket
            server.sockfd = -1;   // Reset the socket file descriptor
        }
    }
    for (auto &thread : server_threads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }
    server_threads.clear();
}

void Radar::setZeroPoint()
{
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!raw_coordinates.empty())
    {
        // Set zero point from the first truck's position
        zero_point = raw_coordinates.begin()->second;
        std::cout << "Zero point set to: (" << zero_point.x() << ", " << zero_point.y() << ")" << std::endl;

        // Clear the processed coordinates as they need to be recalculated
        latest_coordinates.clear();
    }
}

std::tuple<std::map<std::string, Eigen::Vector2d>, std::map<std::string, Eigen::Vector2d>, std::map<std::string, double>, std::map<std::string, double>> Radar::getLatestData()
{
    std::lock_guard<std::mutex> lock(data_mutex);

    std::map<std::string, Eigen::Vector2d> host_coordinates;
    std::map<std::string, Eigen::Vector2d> host_velocities;
    std::map<std::string, double> host_route_times;
    std::map<std::string, double> host_route_distances;

    for (const auto &pair : latest_coordinates)
    {
        std::string host_id = getHostIdForServer(pair.first);
        if (!host_id.empty())
        {
            host_coordinates[host_id] = pair.second;
        }
    }

    for (const auto &pair : latest_velocities)
    {
        std::string host_id = getHostIdForServer(pair.first);
        if (!host_id.empty())
        {
            host_velocities[host_id] = pair.second;
        }
    }

    for (const auto &pair : latest_route_times)
    {
        std::string host_id = getHostIdForServer(pair.first);
        if (!host_id.empty())
        {
            host_route_times[host_id] = pair.second;
        }
    }

    for (const auto &pair : latest_route_distances)
    {
        std::string host_id = getHostIdForServer(pair.first);
        if (!host_id.empty())
        {
            host_route_distances[host_id] = pair.second;
        }
    }

    return std::make_tuple(host_coordinates, host_velocities, host_route_times, host_route_distances);
}

void Radar::connectWebSocket(ServerInfo &server)
{
    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    if (getaddrinfo(server.ip.c_str(), std::to_string(server.port).c_str(), &hints, &res) != 0)
    {
        std::cerr << "getaddrinfo failed for " << server.ip << std::endl;
        return;
    }

    server.sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (server.sockfd == -1)
    {
        std::cerr << "socket creation failed for " << server.ip << std::endl;
        freeaddrinfo(res);
        return;
    }

    if (connect(server.sockfd, res->ai_addr, res->ai_addrlen) == -1)
    {
        std::cerr << "connection failed for " << server.ip << std::endl;
        close(server.sockfd);
        freeaddrinfo(res);
        return;
    }

    freeaddrinfo(res);
    std::cout << "Connected to " << server.ip << ":" << server.port << std::endl;

    readWebSocket(server);
}

void Radar::readWebSocket(ServerInfo &server)
{
    const int BUFFER_SIZE = 4096;
    char buffer[BUFFER_SIZE];
    std::string accumulated_data;

    while (running)
    {
        int bytes_received = recv(server.sockfd, buffer, BUFFER_SIZE - 1, 0);
        if (bytes_received > 0)
        {
            buffer[bytes_received] = '\0';
            accumulated_data += buffer;

            size_t pos;
            while ((pos = accumulated_data.find('\n')) != std::string::npos)
            {
                std::string json_str = accumulated_data.substr(0, pos);
                accumulated_data.erase(0, pos + 1);

                try
                {
                    nlohmann::json j = nlohmann::json::parse(json_str);
                    processTruckData(server.ip + ":" + std::to_string(server.port), j);
                }
                catch (nlohmann::json::parse_error &e)
                {
                    std::cerr << "JSON parsing error: " << e.what() << std::endl;
                    std::cerr << "Problematic JSON string: " << json_str << std::endl;
                }
            }
        }
        else if (bytes_received == 0)
        {
            std::cout << "Server " << server.ip << " closed connection" << std::endl;
            handleConnectionFailure(server);
            break;
        }
        else
        {
            std::cerr << "Error receiving data from " << server.ip << std::endl;
            handleConnectionFailure(server);
            break;
        }
    }
}

void Radar::handleConnectionFailure(ServerInfo &server)
{
    std::string server_id = server.ip + ":" + std::to_string(server.port);
    std::lock_guard<std::mutex> lock(data_mutex);

    // Check if we have a last known coordinate for this server
    auto it = raw_coordinates.find(server_id);
    if (it != raw_coordinates.end())
    {
        // Use the last known coordinate
        double lastX = it->second.x();
        double lastZ = it->second.y();

        // Process and update the coordinates
        processCoordinates(server_id, lastX, lastZ);

        std::cout << "Using last known coordinate for " << server_id << std::endl;
    }
    else
    {
        std::cout << "No last known coordinate available for " << server_id << std::endl;
    }
}

void Radar::processTruckData(const std::string &server_id, const nlohmann::json &data)
{
    std::lock_guard<std::mutex> lock(data_mutex);
    try
    {
        if (data.contains("host_id"))
        {
            std::string host_id = data["host_id"];
            if (!hasReceivedHostId(server_id))
            {
                mapHostIdToServer(server_id, host_id);
                std::cout << "Mapped host_id: " << host_id << " to server_id: " << server_id << std::endl;
            }
        }
        if (data.contains("api") && data["api"].is_object() &&
            data["api"].contains("truckPlacement") && data["api"]["truckPlacement"].is_object())
        {
            auto &truckPlacement = data["api"]["truckPlacement"];
            if (truckPlacement.contains("coordinateX") && truckPlacement.contains("coordinateZ"))
            {
                // Extract original coordinates
                double originalX = truckPlacement["coordinateX"];
                double originalZ = truckPlacement["coordinateZ"];

                // Store raw coordinates
                raw_coordinates[server_id] = Eigen::Vector2d(originalX, originalZ);

                // Process coordinates
                processCoordinates(server_id, originalX, originalZ);
            }
            else
            {
                std::cerr << "Missing coordinate data in truckPlacement for " << server_id << std::endl;
            }
        }
        if (data["api"].contains("truckVector") && data["api"]["truckVector"].is_object())
        {
            auto &truckVector = data["api"]["truckVector"];
            if (truckVector.contains("velocityX") && truckVector.contains("velocityZ"))
            {
                double velocityX = truckVector["velocityX"];
                double velocityZ = truckVector["velocityZ"];
                processVelocity(server_id, velocityX, velocityZ);
            }
            else
            {
                std::cerr << "Missing velocity data in truckPlacement for " << server_id << std::endl;
            }
        }
        if (data["api"].contains("truckFloat") && data["api"]["truckFloat"].is_object())
        {
            auto &truckFloat = data["api"]["truckFloat"];
            if (truckFloat.contains("routeTime") && truckFloat.contains("routeDistance"))
            {
                double routeTime = truckFloat["routeTime"];
                double routeDistance = truckFloat["routeDistance"];
                latest_route_distances[server_id] = routeDistance;
                latest_route_times[server_id] = routeTime;
            }
            else
            {
                std::cerr << "Missing routeTime data in truckFloat for " << server_id << std::endl;
            }
        }
    }
    catch (const nlohmann::json::exception &e)
    {
        std::cerr << "JSON processing error for " << server_id << ": " << e.what() << std::endl;
    }
}

void Radar::printHostServerMappings() const
{
    std::cout << "Current host_id to server_id mappings:" << std::endl;
    for (const auto &pair : host_to_server_id)
    {
        std::cout << "host_id: " << pair.first << " -> server_id: " << pair.second << std::endl;
    }
}

void Radar::processCoordinates(const std::string &server_id, double originalX, double originalZ)
{
    // If zero point is not set, don't process further
    if (zero_point.isZero())
    {
        return;
    }

    // Calculate relative coordinates
    double relativeX = originalX - zero_point.x();
    double relativeZ = originalZ - zero_point.y();

    // Rotate 90 degrees clockwise
    double rotatedX = relativeZ;
    double rotatedY = -relativeX;

    // Store the processed coordinates
    latest_coordinates[server_id] = Eigen::Vector2d(rotatedX, rotatedY);
}

void Radar::processVelocity(const std::string &server_id, double velocityX, double velocityZ)
{
    // Rotate 90 degrees clockwise
    double rotatedVX = velocityZ;
    double rotatedVY = -velocityX;

    // Store the processed velocity
    latest_velocities[server_id] = Eigen::Vector2d(rotatedVX, rotatedVY);
}

bool Radar::performWebSocketHandshake(int sockfd, const std::string &host, int port)
{
    std::string handshake = "GET / HTTP/1.1\r\n"
                            "Host: " +
                            host + ":" + std::to_string(port) + "\r\n"
                                                                "Upgrade: websocket\r\n"
                                                                "Connection: Upgrade\r\n"
                                                                "Sec-WebSocket-Version: 13\r\n\r\n";

    if (send(sockfd, handshake.c_str(), handshake.length(), 0) == -1)
    {
        std::cerr << "Failed to send handshake" << std::endl;
        return false;
    }

    char buffer[1024];
    int bytes_received = recv(sockfd, buffer, sizeof(buffer), 0);
    if (bytes_received == -1)
    {
        std::cerr << "Failed to receive handshake response" << std::endl;
        return false;
    }

    std::string response(buffer, bytes_received);
    std::cout << "Handshake response: " << response << std::endl;
    return response.find("HTTP/1.1 101") != std::string::npos;
}

std::vector<ServerInfo> Radar::getServers() const
{
    return servers;
}

void Radar::sendData(const ServerInfo &server, const std::string &data)
{
    if (server.sockfd != -1)
    {
        ssize_t bytes_sent = send(server.sockfd, data.c_str(), data.length(), 0);
        if (bytes_sent == -1)
        {
            std::cerr << "Error sending data to " << server.ip << ":" << server.port << ": " << strerror(errno) << std::endl;
        }
        else if (static_cast<size_t>(bytes_sent) < data.length())
        {
            std::cerr << "Warning: Only " << bytes_sent << " of " << data.length() << " bytes sent to " << server.ip << ":" << server.port << std::endl;
        }
        // else
        // {
        //     std::cout << "Successfully sent " << bytes_sent << " bytes to " << server.ip << ":" << server.port << std::endl;
        // }
    }
    else
    {
        std::cerr << "Error: Socket not connected for " << server.ip << ":" << server.port << std::endl;
    }
}

std::string Radar::getServerIdForHost(const std::string &host_id)
{
    auto it = host_to_server_id.find(host_id);
    if (it != host_to_server_id.end())
    {
        return it->second;
    }
    return ""; // Return empty string if host_id is not found
}

std::string Radar::getHostIdForServer(const std::string &server_id) const
{
    for (const auto &pair : host_to_server_id)
    {
        if (pair.second == server_id)
        {
            return pair.first;
        }
    }
    return ""; // Return empty string if server_id is not found
}