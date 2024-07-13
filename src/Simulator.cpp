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

/*******************************************************************************/
// Raylib setup
/*******************************************************************************/
Simulator::Simulator()
{
    SetTraceLogLevel(LOG_ERROR);
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
    delete treeOfRobots_;
    int n = robots_.size();
    for (int i = 0; i < n; ++i)
        robots_.erase(i);
    if (globals.DISPLAY)
    {
        delete graphics;
        CloseWindow();
    }
};

/*******************************************************************************/
// Draw highway roads and lanes
/*******************************************************************************/
void Simulator::draw_highway()
{
    // Parameters for the highway
    int n_roads = 2;                                // Number of roads
    int n_lanes = 2;                                // Number of lanes per road
    double lane_width = 8.0 * globals.ROBOT_RADIUS; // Width of each lane
    double road_length = globals.WORLD_SZ;          // Length of each road
    double road_spacing = globals.WORLD_SZ / 4.6;   // Distance between roads
    double ramp_length = road_length / 3;           // Length of on/off ramps
    double ramp_angle = M_PI / 2;                   // Angle of the ramps to the road
    double ramp_length_off = road_length / 4;       // Length of on/off ramps
    double ramp_angle_off = M_PI / 2;               // Angle of the ramps to the road

    // Ramp positions as variables
    double on_ramp_start_pos = -road_length / 2;
    double on_ramp_end_pos = road_length / 40;
    double off_ramp_start_pos = -on_ramp_end_pos;
    double off_ramp_end_pos = road_length / 2;

    for (int road = 0; road < n_roads; ++road)
    {
        double road_v_offset = (road - 0.325) * road_spacing;
        bool flip_ramps = (road % 2 != 0); // Flip ramps on one road

        // Draw the main highway lanes
        for (int lane = 0; lane < n_lanes; ++lane)
        {
            double lane_v_offset = road_v_offset + (0.5 * (1 - 2.0 * n_lanes) + lane) * lane_width;

            Eigen::VectorXd start_point = Eigen::VectorXd{{-road_length / 2.0, lane_v_offset, 0.0, 0.0}};
            Eigen::VectorXd end_point = Eigen::VectorXd{{road_length / 2.0, lane_v_offset, 0.0, 0.0}};

            DrawLine3D(
                Vector3{static_cast<float>(start_point(0)), 0.0f, static_cast<float>(start_point(1))},
                Vector3{static_cast<float>(end_point(0)), 0.0f, static_cast<float>(end_point(1))},
                BLUE);
        }

        // Draw the on and off ramps for each road
        for (int lane = 0; lane < n_lanes; ++lane)
        {
            double lane_v_offset = road_v_offset + (0.5 * (1 - 2.0 * n_lanes) + lane) * lane_width;

            if (flip_ramps)
            {
                // On ramp (flipped horizontally)
                double on_ramp_start_x = on_ramp_start_pos - ramp_length * cos(ramp_angle);
                double on_ramp_start_y = lane_v_offset + ramp_length * sin(ramp_angle) - lane_width;
                double on_ramp_end_x = on_ramp_end_pos;
                double on_ramp_end_y = lane_v_offset;

                DrawLine3D(
                    Vector3{static_cast<float>(on_ramp_start_x), 0.0f, static_cast<float>(on_ramp_start_y)},
                    Vector3{static_cast<float>(on_ramp_end_x), 0.0f, static_cast<float>(on_ramp_end_y)},
                    GREEN);

                // Off ramp
                double off_ramp_start_x = off_ramp_start_pos;
                double off_ramp_start_y = lane_v_offset;
                double off_ramp_end_x = off_ramp_end_pos + ramp_length_off * cos(ramp_angle_off);
                double off_ramp_end_y = lane_v_offset + ramp_length_off * sin(ramp_angle_off) + lane_width;

                DrawLine3D(
                    Vector3{static_cast<float>(off_ramp_start_x), 0.0f, static_cast<float>(off_ramp_start_y)},
                    Vector3{static_cast<float>(off_ramp_end_x), 0.0f, static_cast<float>(off_ramp_end_y)},
                    GREEN);

                // Draw waypoints at on and off-ramp start and end positions
                DrawSphere(Vector3{static_cast<float>(on_ramp_start_x), 0.0f, static_cast<float>(on_ramp_start_y)}, 0.5f, RED);
                DrawSphere(Vector3{static_cast<float>(on_ramp_end_x), 0.0f, static_cast<float>(on_ramp_end_y)}, 0.5f, RED);
                DrawSphere(Vector3{static_cast<float>(off_ramp_start_x), 0.0f, static_cast<float>(off_ramp_start_y)}, 0.5f, RED);
                DrawSphere(Vector3{static_cast<float>(off_ramp_end_x), 0.0f, static_cast<float>(off_ramp_end_y)}, 0.5f, RED);
            }
            else
            {
                // On ramp (flipped to the other side)
                double on_ramp_start_x = on_ramp_start_pos - ramp_length * cos(ramp_angle);
                double on_ramp_start_y = lane_v_offset - ramp_length * sin(ramp_angle) + lane_width;
                double on_ramp_end_x = on_ramp_end_pos;
                double on_ramp_end_y = lane_v_offset;

                DrawLine3D(
                    Vector3{static_cast<float>(on_ramp_start_x), 0.0f, static_cast<float>(on_ramp_start_y)},
                    Vector3{static_cast<float>(on_ramp_end_x), 0.0f, static_cast<float>(on_ramp_end_y)},
                    GREEN);

                // Off ramp (flipped to the other side)
                double off_ramp_start_x = off_ramp_start_pos;
                double off_ramp_start_y = lane_v_offset;
                double off_ramp_end_x = off_ramp_end_pos + ramp_length_off * cos(ramp_angle_off);
                double off_ramp_end_y = lane_v_offset - ramp_length_off * sin(ramp_angle_off) - lane_width;

                DrawLine3D(
                    Vector3{static_cast<float>(off_ramp_start_x), 0.0f, static_cast<float>(off_ramp_start_y)},
                    Vector3{static_cast<float>(off_ramp_end_x), 0.0f, static_cast<float>(off_ramp_end_y)},
                    GREEN);

                // Draw waypoints at on and off-ramp start and end positions
                DrawSphere(Vector3{static_cast<float>(on_ramp_start_x), 0.0f, static_cast<float>(on_ramp_start_y)}, 0.5f, RED);
                DrawSphere(Vector3{static_cast<float>(on_ramp_end_x), 0.0f, static_cast<float>(on_ramp_end_y)}, 0.5f, RED);
                DrawSphere(Vector3{static_cast<float>(off_ramp_start_x), 0.0f, static_cast<float>(off_ramp_start_y)}, 0.5f, RED);
                DrawSphere(Vector3{static_cast<float>(off_ramp_end_x), 0.0f, static_cast<float>(off_ramp_end_y)}, 0.5f, RED);
            }
        }
    }
}

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

    // Draw highway roads and lanes if in highway configuration
    if (globals.FORMATION == "highway-pair")
    {
        draw_highway();
    }

    // Draw Robots and Lines between masters and slaves
    for (auto &[rid, robot] : robots_)
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
                // print(rid, robot->position_.transpose().eval(), master_robot->second->position_.transpose().eval());
                DrawLine3D(slave_position, master_position, DARKGRAY);
            }
        }
    }
    EndMode3D();
    draw_info(clock_);
    EndDrawing();
}

/*******************************************************************************/
// Timestep loop of simulator.
/*******************************************************************************/
void Simulator::timestep()
{

    if (globals.SIM_MODE != Timestep)
        return;

    // Create and/or destory factors depending on a robot's neighbours
    calculateRobotNeighbours(robots_);
    for (auto [r_id, robot] : robots_)
    {
        robot->updateInterrobotFactors();
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
    for (auto [r_id, robot] : robots_)
    {
        robot->updateHorizon();
        robot->updateCurrent();
    }

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

double random_double(double min, double max)
{
    double random = ((double)rand()) / (double)RAND_MAX;
    double diff = max - min;
    double r = random * diff;
    return min + r;
}

struct HighwayWaypoints
{
    Eigen::VectorXd lane_start;
    Eigen::VectorXd lane_end;
    Eigen::VectorXd on_ramp_start;
    Eigen::VectorXd on_ramp_merge;
    Eigen::VectorXd off_ramp_start;
    Eigen::VectorXd off_ramp_merge;
};

// Function to pre-calculate waypoints for the highway configuration
std::unordered_map<int, std::vector<std::pair<HighwayWaypoints, HighwayWaypoints>>> calculateHighwayWaypoints()
{
    std::unordered_map<int, std::vector<std::pair<HighwayWaypoints, HighwayWaypoints>>> highwayWaypoints;
    int n_roads = 2; // Number of roads in the highway configuration
    int n_lanes = 2;
    double lane_width = 8.0 * globals.ROBOT_RADIUS;
    double road_spacing = globals.WORLD_SZ / 4.6;
    double road_length = globals.WORLD_SZ;

    double ramp_length;
    double ramp_angle;
    double ramp_length_off;
    double ramp_angle_off;

    double on_ramp_start_pos;
    double on_ramp_end_pos;
    double off_ramp_start_pos;
    double off_ramp_end_pos;

    if (globals.FORMATION == "highway-pair")
    {
        ramp_length = road_length / 3;     // Length of on/off ramps
        ramp_angle = M_PI / 2;             // Angle of the ramps to the road
        ramp_length_off = road_length / 4; // Length of on/off ramps
        ramp_angle_off = M_PI / 2;         // Angle of the ramps to the road

        // Ramp positions as variables
        on_ramp_start_pos = -road_length / 2;
        on_ramp_end_pos = road_length / 40;
        off_ramp_start_pos = -on_ramp_end_pos;
        off_ramp_end_pos = road_length / 2;
    }
    else
    {
        ramp_length = road_length / 5.3;
        ramp_angle = M_PI / 3.1;
        ramp_length_off = road_length / 7.8;
        ramp_angle_off = M_PI / 4.5;

        on_ramp_start_pos = -road_length / 2.5;
        on_ramp_end_pos = -road_length / 3.7;
        off_ramp_start_pos = road_length / 3.7;
        off_ramp_end_pos = road_length / 2.5;
    }
    for (int group = 0; group < 4; ++group)
    {
        int road = group % 2;
        double road_v_offset = (road - 0.325) * road_spacing;

        for (int lane = 0; lane < n_lanes; ++lane)
        {
            double lane_v_offset = road_v_offset + (0.5 * (1 - 2.0 * n_lanes) + lane) * lane_width;

            double on_ramp_start_x, on_ramp_start_y, on_ramp_merge_x, on_ramp_merge_y;
            double off_ramp_start_x, off_ramp_start_y, off_ramp_merge_x, off_ramp_merge_y;

            // Normal waypoints
            on_ramp_start_x = on_ramp_start_pos - ramp_length * cos(ramp_angle);
            on_ramp_start_y = lane_v_offset - ramp_length * sin(ramp_angle) + lane_width;
            on_ramp_merge_x = on_ramp_end_pos;
            on_ramp_merge_y = lane_v_offset;

            off_ramp_start_x = off_ramp_start_pos;
            off_ramp_start_y = lane_v_offset;
            off_ramp_merge_x = off_ramp_end_pos + ramp_length_off * cos(ramp_angle_off);
            off_ramp_merge_y = lane_v_offset - ramp_length_off * sin(ramp_angle_off) - lane_width;

            HighwayWaypoints normal_waypoints = {
                Eigen::VectorXd{{-road_length / 2.0, lane_v_offset, globals.MAX_SPEED, 0.0}},
                Eigen::VectorXd{{road_length / 2.0, lane_v_offset, globals.MAX_SPEED, 0.0}},
                Eigen::VectorXd{{on_ramp_start_x, on_ramp_start_y, globals.MAX_SPEED, 0.0}},
                Eigen::VectorXd{{on_ramp_merge_x, on_ramp_merge_y, globals.MAX_SPEED, 0.0}},
                Eigen::VectorXd{{off_ramp_start_x, off_ramp_start_y, globals.MAX_SPEED, 0.0}},
                Eigen::VectorXd{{off_ramp_merge_x, off_ramp_merge_y, globals.MAX_SPEED, 0.0}}};

            // Flipped waypoints
            on_ramp_start_x = on_ramp_start_pos - ramp_length * cos(ramp_angle);
            on_ramp_start_y = lane_v_offset + ramp_length * sin(ramp_angle) - lane_width;
            on_ramp_merge_x = on_ramp_end_pos;
            on_ramp_merge_y = lane_v_offset;

            off_ramp_start_x = off_ramp_start_pos;
            off_ramp_start_y = lane_v_offset;
            off_ramp_merge_x = off_ramp_end_pos + ramp_length_off * cos(ramp_angle_off);
            off_ramp_merge_y = lane_v_offset + ramp_length_off * sin(ramp_angle_off) + lane_width;

            HighwayWaypoints flipped_waypoints = {
                Eigen::VectorXd{{-road_length / 2.0, lane_v_offset, globals.MAX_SPEED, 0.0}},
                Eigen::VectorXd{{road_length / 2.0, lane_v_offset, globals.MAX_SPEED, 0.0}},
                Eigen::VectorXd{{on_ramp_start_x, on_ramp_start_y, globals.MAX_SPEED, 0.0}},
                Eigen::VectorXd{{on_ramp_merge_x, on_ramp_merge_y, globals.MAX_SPEED, 0.0}},
                Eigen::VectorXd{{off_ramp_start_x, off_ramp_start_y, globals.MAX_SPEED, 0.0}},
                Eigen::VectorXd{{off_ramp_merge_x, off_ramp_merge_y, globals.MAX_SPEED, 0.0}}};

            highwayWaypoints[group].emplace_back(normal_waypoints, flipped_waypoints);
        }
    }

    return highwayWaypoints;
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

    if (globals.FORMATION == "circle")
    {
        new_robots_needed_ = false;
        float min_circumference_spacing = 5. * globals.ROBOT_RADIUS;
        double min_radius = 0.25 * globals.WORLD_SZ;
        double additional_radius = 5.0; // Define how much bigger the radius for odd robots should be
        Eigen::VectorXd centre{{0., 0., 0., 0.}};

        for (int i = 0; i < globals.NUM_ROBOTS; i++)
        {
            // Calculate radius of the circle for even robots and increase for odd robots
            float radius_circle = (globals.NUM_ROBOTS == 1) ? min_radius : std::max(min_radius, sqrt(min_circumference_spacing / (2. - 2. * cos(2. * PI / (double)globals.NUM_ROBOTS))));

            // Calculate offset from center
            Eigen::VectorXd offset_from_centre_inner = Eigen::VectorXd{{radius_circle * cos(2. * PI * (i / 2) / (float)(globals.NUM_ROBOTS / 2))},
                                                                       {radius_circle * sin(2. * PI * (i / 2) / (float)(globals.NUM_ROBOTS / 2))},
                                                                       {0.},
                                                                       {0.}};

            // Calculate offset from center
            Eigen::VectorXd offset_from_centre_outer = Eigen::VectorXd{{(radius_circle + additional_radius) * cos(2. * PI * (i / 2) / (float)(globals.NUM_ROBOTS / 2))},
                                                                       {(radius_circle + additional_radius) * sin(2. * PI * (i / 2) / (float)(globals.NUM_ROBOTS / 2))},
                                                                       {0.},
                                                                       {0.}};

            Eigen::VectorXd starting = (i % 2 == 0) ? centre + offset_from_centre_inner : centre + offset_from_centre_outer;
            Eigen::VectorXd ending = (i % 2 == 0) ? centre - offset_from_centre_outer : centre - offset_from_centre_inner;
            // Eigen::VectorXd ending = centre - offset_from_centre_outer;
            std::deque<Eigen::VectorXd>
                waypoints{starting, ending};

            // Define robot radius and colour here.
            float robot_radius = globals.ROBOT_RADIUS;
            // If i is even, robot is red, else blue.
            bool isMaster = (next_rid_ % 2 == 0); // for example, even ids are masters
            Color robot_color = isMaster ? DARKBROWN : DARKBLUE;
            int master_id = isMaster ? next_rid_ : next_rid_ - 1; // for example, each slave has the previous robot as master

            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints, robot_radius, robot_color, isMaster, master_id));
        }
        // print to termimal for each robot its data
        for (auto robot : robots_to_create)
        {
            print("Robot ID: ", robot->rid_, " Master ID: ", robot->master_id_, " Master: ", robot->isMaster_, " Position: ", robot->position_.transpose());
        }
    }
    else if (globals.FORMATION == "junction")
    {
        // Robots in a cross-roads style junction with master-slave functionality.
        new_robots_needed_ = true; // This is needed so that more robots can be created as the simulation progresses.
        if (clock_ % 20 == 0)
        { // Arbitrary condition on the simulation time to create new robots
            int n_roads = 2;
            int road = random_int(0, n_roads - 1);
            Eigen::Matrix4d rot;
            rot.setZero();
            rot.topLeftCorner(2, 2) << cos(PI / 2. * road), -sin(PI / 2. * road), sin(PI / 2. * road), cos(PI / 2. * road);
            rot.bottomRightCorner(2, 2) << cos(PI / 2. * road), -sin(PI / 2. * road), sin(PI / 2. * road), cos(PI / 2. * road);

            int n_lanes = 2;
            int lane = random_int(0, n_lanes - 1);
            double lane_width = 4. * globals.ROBOT_RADIUS;
            double lane_v_offset = (0.5 * (1 - n_lanes) + lane) * lane_width;
            double additional_distance = 0.5; // Define the additional distance behind the master

            // Define starting and ending points for master and slave robots
            Eigen::VectorXd starting_master = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2. + additional_distance, lane_v_offset, globals.MAX_SPEED, 0.}};
            Eigen::VectorXd ending_master = rot * Eigen::VectorXd{{(double)globals.WORLD_SZ / 2. + additional_distance, lane_v_offset, 0., 0.}};

            Eigen::VectorXd starting_slave = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2., lane_v_offset, globals.MAX_SPEED, 0.}};
            Eigen::VectorXd ending_slave = rot * Eigen::VectorXd{{(double)globals.WORLD_SZ / 2., lane_v_offset, 0., 0.}};

            // Create waypoints for master and slave robots
            std::deque<Eigen::VectorXd> waypoints_master{starting_master, ending_master};
            std::deque<Eigen::VectorXd> waypoints_slave{starting_slave, ending_slave};

            float robot_radius = globals.ROBOT_RADIUS;

            // Create master robot
            bool isMaster = true;
            Color robot_color_master = DARKBROWN;
            int master_id = next_rid_;
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints_master, robot_radius, robot_color_master, isMaster, master_id));

            // Create slave robot
            isMaster = false;
            Color robot_color_slave = DARKBLUE;
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints_slave, robot_radius, robot_color_slave, isMaster, master_id));
        }

        // Delete robots if out of bounds
        for (auto [rid, robot] : robots_)
        {
            if (abs(robot->position_(0)) > globals.WORLD_SZ / 2 || abs(robot->position_(1)) > globals.WORLD_SZ / 2)
            {
                robots_to_delete.push_back(robot);
            }
        }
    }
    else if (globals.FORMATION == "junction_twoway")
    {
        // Robots in a two-way junction, turning LEFT (RED), RIGHT (BLUE) or STRAIGHT (GREEN) with master-slave functionality.
        new_robots_needed_ = true; // This is needed so that more robots can be created as the simulation progresses.
        if (clock_ % 20 == 0)
        { // Arbitrary condition on the simulation time to create new robots
            int n_roads = 4;
            int road = random_int(0, n_roads - 1);
            // We will define one road (the one going left) and then we can rotate the positions for other roads.
            Eigen::Matrix4d rot;
            rot.setZero();
            rot.topLeftCorner(2, 2) << cos(PI / 2. * road), -sin(PI / 2. * road), sin(PI / 2. * road), cos(PI / 2. * road);
            rot.bottomRightCorner(2, 2) << cos(PI / 2. * road), -sin(PI / 2. * road), sin(PI / 2. * road), cos(PI / 2. * road);

            int n_lanes = 2;
            int lane = random_int(0, n_lanes - 1);
            int turn = random_int(0, 2);
            double lane_width = 4. * globals.ROBOT_RADIUS;
            double lane_v_offset = (0.5 * (1 - 2. * n_lanes) + lane) * lane_width;
            double lane_h_offset = (1 - turn) * (0.5 + lane - n_lanes) * lane_width;
            double additional_distance = 0.5; // Define the additional distance behind the master

            // Define starting, turning, and ending points for master robots
            Eigen::VectorXd starting_master = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2. + additional_distance, lane_v_offset, globals.MAX_SPEED, 0.}};
            Eigen::VectorXd turning_master = rot * Eigen::VectorXd{{lane_h_offset, lane_v_offset, (turn % 2) * globals.MAX_SPEED, (turn - 1) * globals.MAX_SPEED}};
            Eigen::VectorXd ending_master = rot * Eigen::VectorXd{{lane_h_offset + (turn % 2) * globals.WORLD_SZ * 1. + additional_distance, lane_v_offset + (turn - 1) * globals.WORLD_SZ * 1., 0., 0.}};

            // Define starting, turning, and ending points for slave robots
            Eigen::VectorXd starting_slave = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2., lane_v_offset, globals.MAX_SPEED, 0.}};
            Eigen::VectorXd turning_slave = turning_master; // Slave robots will have the same turning point
            Eigen::VectorXd ending_slave = rot * Eigen::VectorXd{{lane_h_offset + (turn % 2) * globals.WORLD_SZ * 1., lane_v_offset + (turn - 1) * globals.WORLD_SZ * 1., 0., 0.}};

            // Create waypoints for master and slave robots
            std::deque<Eigen::VectorXd> waypoints_master{starting_master, turning_master, ending_master};
            std::deque<Eigen::VectorXd> waypoints_slave{starting_slave, turning_slave, ending_slave};

            float robot_radius = globals.ROBOT_RADIUS;

            // Create master robot
            bool isMaster = true;
            Color robot_color_master = DARKBROWN;
            int master_id = next_rid_;
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints_master, robot_radius, robot_color_master, isMaster, master_id));

            // Create slave robot
            isMaster = false;
            Color robot_color_slave = DARKBLUE;
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints_slave, robot_radius, robot_color_slave, isMaster, master_id));
        }

        // Delete robots if out of bounds
        for (auto [rid, robot] : robots_)
        {
            if (abs(robot->position_(0)) > globals.WORLD_SZ / 2 || abs(robot->position_(1)) > globals.WORLD_SZ / 2)
            {
                robots_to_delete.push_back(robot);
            }
        }
    }
    else if (globals.FORMATION == "grid")
    {
        // Robots in a cross-roads style junction with master-slave functionality.
        new_robots_needed_ = true; // This is needed so that more robots can be created as the simulation progresses.
        if (clock_ % 20 == 0)
        { // Arbitrary condition on the simulation time to create new robots
            leader_init_ = true;
            int n_roads = 6; // Adjusted for 6 possible road positions
            int road = random_int(0, n_roads - 1);

            Eigen::Matrix4d rot;
            rot.setZero();

            // Define the rotation based on the selected road orientation
            switch (road)
            {
            case 0: // Horizontal road (top row)
                rot.topLeftCorner(2, 2) << 1, 0, 0, 1;
                rot.bottomRightCorner(2, 2) << 1, 0, 0, 1;
                break;
            case 1: // Horizontal road (middle row)
                rot.topLeftCorner(2, 2) << 1, 0, 0, 1;
                rot.bottomRightCorner(2, 2) << 1, 0, 0, 1;
                break;
            case 2: // Horizontal road (bottom row)
                rot.topLeftCorner(2, 2) << 1, 0, 0, 1;
                rot.bottomRightCorner(2, 2) << 1, 0, 0, 1;
                break;
            case 3: // Vertical road (left column)
                rot.topLeftCorner(2, 2) << 0, -1, 1, 0;
                rot.bottomRightCorner(2, 2) << 0, -1, 1, 0;
                break;
            case 4: // Vertical road (middle column)
                rot.topLeftCorner(2, 2) << 0, -1, 1, 0;
                rot.bottomRightCorner(2, 2) << 0, -1, 1, 0;
                break;
            case 5: // Vertical road (right column)
                rot.topLeftCorner(2, 2) << 0, -1, 1, 0;
                rot.bottomRightCorner(2, 2) << 0, -1, 1, 0;
                break;
            default:
                break;
            }

            int n_lanes = 2;
            int lane = random_int(0, n_lanes - 1);
            double lane_width = 4. * globals.ROBOT_RADIUS;
            double lane_v_offset = (0.5 * (1 - n_lanes) + lane) * lane_width;
            double additional_distance = -2; // Define the additional distance behind the master

            Eigen::VectorXd starting_master;
            Eigen::VectorXd ending_master;
            Eigen::VectorXd starting_slave;

            // Adjust the distance between pairs of roads by reducing the vertical/horizontal offsets
            double closer_offset = globals.WORLD_SZ / 3.5;           // This brings roads closer together in the grid
            double closer_offset_vertical = globals.WORLD_SZ / 3.25; // This brings roads closer together in the grid

            switch (road)
            {
            case 0:
                starting_master = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2., closer_offset + lane_v_offset, globals.MAX_SPEED, 0.}};
                starting_slave = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2. + additional_distance, closer_offset + lane_v_offset, globals.MAX_SPEED, 0.}};
                ending_master = rot * Eigen::VectorXd{{globals.WORLD_SZ / 2., closer_offset + lane_v_offset, 0., 0.}};
                break;
            case 1:
                starting_master = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2., 0. + lane_v_offset, globals.MAX_SPEED, 0.}};
                starting_slave = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2. + additional_distance, 0. + lane_v_offset, globals.MAX_SPEED, 0.}};
                ending_master = rot * Eigen::VectorXd{{globals.WORLD_SZ / 2., 0. + lane_v_offset, 0., 0.}};
                break;
            case 2:
                starting_master = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2., -closer_offset + lane_v_offset, globals.MAX_SPEED, 0.}};
                starting_slave = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2. + additional_distance, 0. + lane_v_offset, globals.MAX_SPEED, 0.}};
                ending_master = rot * Eigen::VectorXd{{globals.WORLD_SZ / 2., -closer_offset + lane_v_offset, 0., 0.}};
                break;
            case 3:
                starting_master = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2., closer_offset_vertical + lane_v_offset, globals.MAX_SPEED, 0.}};
                starting_slave = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2. + additional_distance, 0. + lane_v_offset, globals.MAX_SPEED, 0.}};
                ending_master = rot * Eigen::VectorXd{{globals.WORLD_SZ / 2., closer_offset_vertical + lane_v_offset, 0., 0.}};
                break;
            case 4:
                starting_master = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2., 0. + lane_v_offset, globals.MAX_SPEED, 0.}};
                starting_slave = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2. + additional_distance, 0. + lane_v_offset, globals.MAX_SPEED, 0.}};
                ending_master = rot * Eigen::VectorXd{{globals.WORLD_SZ / 2., 0. + lane_v_offset, 0., 0.}};
                break;
            case 5:
                starting_master = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2., -closer_offset_vertical + lane_v_offset, globals.MAX_SPEED, 0.}};
                starting_slave = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2. + additional_distance, 0. + lane_v_offset, globals.MAX_SPEED, 0.}};
                ending_master = rot * Eigen::VectorXd{{globals.WORLD_SZ / 2., -closer_offset_vertical + lane_v_offset, 0., 0.}};
                break;
            default:
                break;
            }

            // Create waypoints for master and slave robots
            std::deque<Eigen::VectorXd> waypoints_master{starting_master, ending_master};

            float robot_radius = globals.ROBOT_RADIUS;

            // Create master robot
            bool isMaster = true;
            Color robot_color_master = DARKBROWN;
            int master_id = next_rid_;
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints_master, robot_radius, robot_color_master, isMaster, master_id));

            // Create slave robot
            isMaster = false;
            Color robot_color_slave = DARKBLUE;
            master_id = next_rid_ - 1;
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints_master, robot_radius, robot_color_slave, isMaster, master_id));
        }

        // Delete robots if out of bounds
        for (auto [rid, robot] : robots_)
        {
            if (abs(robot->position_(0)) > globals.WORLD_SZ / 2 || abs(robot->position_(1)) > globals.WORLD_SZ / 2)
            {
                robots_to_delete.push_back(robot);
                leader_init_ = false;
            }
        }
    }
    else if (globals.FORMATION == "follow-leader")
    {
        new_robots_needed_ = true;
        // robot count and time
        if (clock_ % 20 == 0 && next_rid_ < globals.NUM_ROBOTS + 1) // Update condition to create 21 robots (1 leader + 20 followers)
        {
            // Create a leader robot
            std::cout << "Creating leader robot" << std::endl;
            starting = Eigen::VectorXd{{-globals.WORLD_SZ / 2., -globals.WORLD_SZ / 2., globals.MAX_SPEED, 0.}};

            // generate a set of random waypoints for the leader robot and insert it into the robots_ map
            std::deque<Eigen::VectorXd> waypoints{starting};
            for (int i = 0; i < 10; i++)
            {
                Eigen::VectorXd next_waypoint = Eigen::VectorXd{{random_double(-globals.WORLD_SZ / 2., globals.WORLD_SZ / 2.), random_double(-globals.WORLD_SZ / 2., globals.WORLD_SZ / 2.), 0., 0.}};
                waypoints.push_back(next_waypoint);
            }

            ending = Eigen::VectorXd{{(double)globals.WORLD_SZ, -globals.WORLD_SZ / 2., 0., 0.}};
            waypoints.push_back(ending);
            float robot_radius = globals.ROBOT_RADIUS;
            Color robot_color = DARKGREEN;
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints, robot_radius, robot_color, true, -1));

            leader_init_ = true;

            // Create follower robots
            float min_circumference_spacing = 5. * globals.ROBOT_RADIUS;
            double min_radius = 0.25 * globals.WORLD_SZ;
            Eigen::VectorXd centre{{0., 0., 0., 0.}};
            for (int i = 1; i <= globals.NUM_ROBOTS; i++) // Create 20 follower robots
            {
                // Select radius of large circle to be at least min_radius,
                // Also ensures that robots in the circle are at least min_circumference_spacing away from each other
                float radius_circle = (globals.NUM_ROBOTS == 1) ? min_radius : std::max(min_radius, sqrt(min_circumference_spacing / (2. - 2. * cos(2. * PI / globals.NUM_ROBOTS))));
                Eigen::VectorXd offset_from_centre = Eigen::VectorXd{{radius_circle * cos(2. * PI * i / globals.NUM_ROBOTS)},
                                                                     {radius_circle * sin(2. * PI * i / globals.NUM_ROBOTS)},
                                                                     {0.},
                                                                     {0.}};
                starting = centre + offset_from_centre;
                std::deque<Eigen::VectorXd> waypoints{starting, starting};

                // Define robot radius and colour here.
                float robot_radius = globals.ROBOT_RADIUS;
                Color robot_color = ColorFromHSV(i * 360. / globals.NUM_ROBOTS, 1., 0.75);
                robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints, robot_radius, robot_color, true, -1));
            }
        }

        if (!robots_.empty())
        {
            // Update the follower robots to follow the node in front of them
            for (int i = 1; i <= globals.NUM_ROBOTS; i++) // Update loop condition to follow 20 robots
            {
                auto target = robots_.at(i - 1);
                auto follower = robots_.at(i);
                Eigen::VectorXd offset_from_target = Eigen::VectorXd{{0., 0., 0., 0.}};
                follower->waypoints_[0] = target->position_ - offset_from_target;
            }
        }
    }
    else if (globals.FORMATION == "highway")
    {
        new_robots_needed_ = true; // This is needed so that more robots can be created as the simulation progresses.
        if (clock_ % 20 == 0 && next_rid_ < globals.NUM_ROBOTS)
        {
            int n_roads = 2; // Adjusted to match the highway configuration
            int road = random_int(0, n_roads - 1);
            int n_lanes = 2;
            double lane_width = 4.0 * globals.ROBOT_RADIUS;
            double road_length = globals.WORLD_SZ;        // Length of each road
            double road_spacing = globals.WORLD_SZ / 3.5; // Distance between roads
            double road_v_offset = (road - 0.375) * road_spacing;
            bool flip_ramps = (road % 2 != 0); // Flip ramps on one road

            int lane = random_int(0, n_lanes - 1);
            double lane_v_offset = road_v_offset + (0.5 * (1 - 2.0 * n_lanes) + lane) * lane_width;

            double ramp_length = road_length / 2.7;     // Length of on/off ramps
            double ramp_angle = M_PI / 2.9;             // Angle of the ramps to the road
            double ramp_length_off = road_length / 3.2; // Length of on/off ramps
            double ramp_angle_off = M_PI / 3.2;         // Angle of the ramps to the road

            // Ramp positions as variables
            double on_ramp_start_pos = -road_length / 3.1;
            double on_ramp_end_pos = -road_length / 6.;
            double off_ramp_start_pos = road_length / 6.0;
            double off_ramp_end_pos = road_length / 3.1;

            // Define ramp positions based on the flip condition
            double on_ramp_start_x, on_ramp_start_y, on_ramp_merge_x, on_ramp_merge_y;
            double off_ramp_start_x, off_ramp_start_y, off_ramp_merge_x, off_ramp_merge_y;

            if (flip_ramps)
            {
                on_ramp_start_x = on_ramp_start_pos - ramp_length * cos(ramp_angle);
                on_ramp_start_y = lane_v_offset + ramp_length * sin(ramp_angle) - lane_width;
                on_ramp_merge_x = on_ramp_end_pos;
                on_ramp_merge_y = lane_v_offset;

                off_ramp_start_x = off_ramp_start_pos;
                off_ramp_start_y = lane_v_offset;
                off_ramp_merge_x = off_ramp_end_pos + ramp_length_off * cos(ramp_angle_off);
                off_ramp_merge_y = lane_v_offset + ramp_length_off * sin(ramp_angle_off) + lane_width;
            }
            else
            {
                on_ramp_start_x = on_ramp_start_pos - ramp_length * cos(ramp_angle);
                on_ramp_start_y = lane_v_offset - ramp_length * sin(ramp_angle) + lane_width;
                on_ramp_merge_x = on_ramp_end_pos;
                on_ramp_merge_y = lane_v_offset;

                off_ramp_start_x = off_ramp_start_pos;
                off_ramp_start_y = lane_v_offset;
                off_ramp_merge_x = off_ramp_end_pos + ramp_length_off * cos(ramp_angle_off);
                off_ramp_merge_y = lane_v_offset - ramp_length_off * sin(ramp_angle_off) - lane_width;
            }

            // Define starting, turning, and ending points for master robots
            Eigen::VectorXd starting_master = Eigen::VectorXd{{on_ramp_start_x, on_ramp_start_y, globals.MAX_SPEED, 0.0}};
            Eigen::VectorXd turning_1 = Eigen::VectorXd{{on_ramp_merge_x, on_ramp_merge_y, globals.MAX_SPEED, 0.0}};
            Eigen::VectorXd turning_2 = Eigen::VectorXd{{off_ramp_start_x, off_ramp_start_y, globals.MAX_SPEED, 0.0}};
            Eigen::VectorXd ending_master = Eigen::VectorXd{{off_ramp_merge_x, off_ramp_merge_y, 0.0, 0.0}};

            // Define starting, turning, and ending points for slave robots
            Eigen::VectorXd starting_slave = Eigen::VectorXd{{on_ramp_start_x, on_ramp_start_y, globals.MAX_SPEED, 0.0}};
            Eigen::VectorXd ending_slave = Eigen::VectorXd{{off_ramp_merge_x, off_ramp_merge_y, 0.0, 0.0}};

            std::deque<Eigen::VectorXd> waypoints_master;
            std::deque<Eigen::VectorXd> waypoints_slave;

            // Create waypoints for master and slave robots
            if (!flip_ramps) // If ramps are not flipped, the master robot goes from left to right
            {
                waypoints_master = {starting_master, turning_1, turning_2, ending_master};
                waypoints_slave = {starting_slave, turning_1, turning_2, ending_slave};
            }
            else
            {
                waypoints_master = {ending_master, turning_2, turning_1, starting_master};
                waypoints_slave = {ending_slave, turning_2, turning_1, starting_slave};
            }

            float robot_radius = globals.ROBOT_RADIUS;

            // Create master robot
            bool isMaster = true;
            Color robot_color_master = DARKBROWN;
            int master_id = next_rid_;
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints_master, robot_radius, robot_color_master, isMaster, master_id));

            // Create slave robot
            isMaster = false;
            Color robot_color_slave = DARKBLUE;
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints_slave, robot_radius, robot_color_slave, isMaster, master_id));
        }

        // Delete robots if out of bounds
        for (auto [rid, robot] : robots_)
        {
            if (abs(robot->position_(0)) > globals.WORLD_SZ / 2 || abs(robot->position_(1)) > globals.WORLD_SZ / 2)
            {
                robots_to_delete.push_back(robot);
            }
        }
    }
    else if (globals.FORMATION == "highway-follow-leader")
    {
        new_robots_needed_ = true;
        // Robot count and time
        if (clock_ % 20 == 0 && next_rid_ < 4 * globals.NUM_ROBOTS + 4) // Create 2 leaders + globals.NUM_ROBOTS followers per group for 4 groups
        {
            int n_roads = 2; // Number of roads in the highway configuration
            int n_lanes = 2;
            double lane_width = 4.0 * globals.ROBOT_RADIUS;
            double road_spacing = globals.WORLD_SZ / 3.5;

            for (int group = 0; group < 4; ++group)
            {
                int road = group % 2; // Road is set to 0 for group 0 and 2, 1 for group 1 and 3
                double road_v_offset = (road - 0.375) * road_spacing;
                bool flip_ramps = (group == 1 || group == 3);        // Flip ramps for groups 1 and 3
                bool travel_on_highway = (group == 2 || group == 3); // Groups 2 and 3 travel on the main highway

                int lane = random_int(0, n_lanes - 1);
                double lane_v_offset = road_v_offset + (0.5 * (1 - 2.0 * n_lanes) + lane) * lane_width;
                if (group == 3)
                {
                    lane_v_offset = road_v_offset + (0.5 * (1 - 1.0 * n_lanes) + lane) * lane_width;
                }
                double road_length = globals.WORLD_SZ;      // Length of each road
                double ramp_length = road_length / 2.7;     // Length of on/off ramps
                double ramp_angle = M_PI / 2.9;             // Angle of the ramps to the road
                double ramp_length_off = road_length / 3.2; // Length of off ramps
                double ramp_angle_off = M_PI / 3.2;         // Angle of the off ramps to the road

                double on_ramp_start_pos = -road_length / 3.1;
                double on_ramp_end_pos = -road_length / 6.;
                double off_ramp_start_pos = road_length / 6.0;
                double off_ramp_end_pos = road_length / 3.1;

                // Define ramp positions based on the flip condition
                double on_ramp_start_x, on_ramp_start_y, on_ramp_merge_x, on_ramp_merge_y;
                double off_ramp_start_x, off_ramp_start_y, off_ramp_merge_x, off_ramp_merge_y;

                if (flip_ramps)
                {
                    on_ramp_start_x = on_ramp_start_pos - ramp_length * cos(ramp_angle);
                    on_ramp_start_y = lane_v_offset + ramp_length * sin(ramp_angle) - lane_width;
                    on_ramp_merge_x = on_ramp_end_pos;
                    on_ramp_merge_y = lane_v_offset;

                    off_ramp_start_x = off_ramp_start_pos;
                    off_ramp_start_y = lane_v_offset;
                    off_ramp_merge_x = off_ramp_end_pos + ramp_length_off * cos(ramp_angle_off);
                    off_ramp_merge_y = lane_v_offset + ramp_length_off * sin(ramp_angle_off) + lane_width;
                }
                else
                {
                    on_ramp_start_x = on_ramp_start_pos - ramp_length * cos(ramp_angle);
                    on_ramp_start_y = lane_v_offset - ramp_length * sin(ramp_angle) + lane_width;
                    on_ramp_merge_x = on_ramp_end_pos;
                    on_ramp_merge_y = lane_v_offset;

                    off_ramp_start_x = off_ramp_start_pos;
                    off_ramp_start_y = lane_v_offset;
                    off_ramp_merge_x = off_ramp_end_pos + ramp_length_off * cos(ramp_angle_off);
                    off_ramp_merge_y = lane_v_offset - ramp_length_off * sin(ramp_angle_off) - lane_width;
                }

                // Define waypoints for the leader robot
                std::deque<Eigen::VectorXd> waypoints_leader;

                if (travel_on_highway)
                {
                    if (group == 2) // Left to right on road 0
                    {
                        waypoints_leader.push_back(Eigen::VectorXd{{-road_length / 2.0, road_v_offset, globals.MAX_SPEED, 0.0}});
                        waypoints_leader.push_back(Eigen::VectorXd{{road_length / 2.0, road_v_offset, globals.MAX_SPEED, 0.0}});
                    }
                    else if (group == 3) // Right to left on road 1
                    {
                        waypoints_leader.push_back(Eigen::VectorXd{{road_length / 2.0, road_v_offset, globals.MAX_SPEED, 0.0}});
                        waypoints_leader.push_back(Eigen::VectorXd{{off_ramp_start_x, off_ramp_start_y, globals.MAX_SPEED, 0.0}});
                        waypoints_leader.push_back(Eigen::VectorXd{{on_ramp_merge_x, on_ramp_merge_y, globals.MAX_SPEED, 0.0}});
                        waypoints_leader.push_back(Eigen::VectorXd{{-road_length / 2.0, road_v_offset, globals.MAX_SPEED, 0.0}});
                    }
                }
                else
                {
                    if (!flip_ramps)
                    {
                        waypoints_leader.push_back(Eigen::VectorXd{{on_ramp_start_x, on_ramp_start_y, globals.MAX_SPEED, 0.0}});
                        waypoints_leader.push_back(Eigen::VectorXd{{on_ramp_merge_x, on_ramp_merge_y, globals.MAX_SPEED, 0.0}});
                        waypoints_leader.push_back(Eigen::VectorXd{{off_ramp_start_x, off_ramp_start_y, globals.MAX_SPEED, 0.0}});
                        waypoints_leader.push_back(Eigen::VectorXd{{off_ramp_merge_x, off_ramp_merge_y, 0.0, 0.0}});
                    }
                    else
                    {
                        waypoints_leader.push_back(Eigen::VectorXd{{off_ramp_merge_x, off_ramp_merge_y, globals.MAX_SPEED, 0.0}});
                        waypoints_leader.push_back(Eigen::VectorXd{{off_ramp_start_x, off_ramp_start_y, globals.MAX_SPEED, 0.0}});
                        waypoints_leader.push_back(Eigen::VectorXd{{on_ramp_merge_x, on_ramp_merge_y, globals.MAX_SPEED, 0.0}});
                        waypoints_leader.push_back(Eigen::VectorXd{{on_ramp_start_x, on_ramp_start_y, 0.0, 0.0}});
                    }
                }

                float robot_radius = globals.ROBOT_RADIUS;
                Color leader_color = (group < 2) ? (group == 0 ? DARKGREEN : RED) : (group == 2 ? BLUE : YELLOW);

                // Create the leader robot for each group
                robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints_leader, robot_radius, leader_color, true, -1));

                // Create follower robots
                for (int i = 1; i <= globals.NUM_ROBOTS; i++) // Create follower robots
                {
                    Eigen::VectorXd starting_position;
                    if (travel_on_highway)
                    {
                        if (group == 2)
                        {
                            starting_position = Eigen::VectorXd{{-road_length / 2.0, road_v_offset, globals.MAX_SPEED, 0.0}};
                        }
                        else
                        {
                            starting_position = Eigen::VectorXd{{road_length / 2.0, road_v_offset, globals.MAX_SPEED, 0.0}};
                        }
                    }
                    else
                    {
                        if (!flip_ramps)
                        {
                            starting_position = Eigen::VectorXd{{on_ramp_start_x, on_ramp_start_y, globals.MAX_SPEED, 0.0}};
                        }
                        else
                        {
                            starting_position = Eigen::VectorXd{{off_ramp_merge_x, off_ramp_merge_y, globals.MAX_SPEED, 0.0}};
                        }
                    }
                    std::deque<Eigen::VectorXd> waypoints{starting_position, starting_position};

                    // Define robot radius and colour
                    float follower_radius = globals.ROBOT_RADIUS;
                    Color follower_color = ColorFromHSV(i * 360.0 / globals.NUM_ROBOTS, 1.0, 0.75);
                    robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints, follower_radius, follower_color, true, -1));
                }
            }
        }

        if (!robots_.empty())
        {
            // Update the follower robots to follow the node in front of them
            for (int group = 0; group < 4; ++group)
            {
                if (random_double(0.0, 1.0) < 0.87) // 87% communication failure rate
                {
                    continue;
                }
                int start_index = group * (globals.NUM_ROBOTS + 1);
                for (int i = start_index + 1; i <= start_index + globals.NUM_ROBOTS; i++)
                {
                    auto target = robots_.at(i - 1);
                    auto follower = robots_.at(i);
                    Eigen::VectorXd offset_from_target = Eigen::VectorXd{{0.0, 0.0, 0.0, 0.0}};
                    follower->waypoints_[0] = target->position_ - offset_from_target;
                }
            }
        }
    }
    else if (globals.FORMATION == "highway-join-leave")
    {
        auto highwayWaypoints = calculateHighwayWaypoints();

        new_robots_needed_ = true;
        // Robot count and time
        if (clock_ % 20 == 0 && next_rid_ < 4 * globals.NUM_ROBOTS + 4) // Create 2 leaders + globals.NUM_ROBOTS followers per group for 4 groups
        {
            for (int group = 0; group < 4; ++group)
            {
                bool travel_on_highway = (group == 2 || group == 3);
                int lane = random_int(0, 2 - 1);
                bool flip_ramps = (group == 1);

                auto waypoints = highwayWaypoints[group][lane];
                auto &selected_waypoints = flip_ramps ? waypoints.second : waypoints.first;

                std::deque<Eigen::VectorXd> waypoints_leader;

                if (travel_on_highway)
                {
                    if (group == 2) // Left to right on road 0
                    {
                        waypoints_leader.push_back(selected_waypoints.lane_start);
                        waypoints_leader.push_back(selected_waypoints.on_ramp_merge);
                        selected_waypoints = highwayWaypoints[2][0].first;
                        waypoints_leader.push_back(selected_waypoints.off_ramp_start);
                        waypoints_leader.push_back(selected_waypoints.off_ramp_merge);
                    }
                    else if (group == 3) // Right to left on road 1
                    {
                        selected_waypoints = highwayWaypoints[3][1].second;
                        waypoints_leader.push_back(selected_waypoints.lane_end);
                        waypoints_leader.push_back(selected_waypoints.off_ramp_start);
                        waypoints_leader.push_back(selected_waypoints.on_ramp_merge);
                        selected_waypoints = highwayWaypoints[3][0].second;
                        waypoints_leader.push_back(selected_waypoints.lane_start);
                    }
                }
                else
                {
                    if (!flip_ramps)
                    {
                        waypoints_leader.push_back(selected_waypoints.on_ramp_start);
                        waypoints_leader.push_back(selected_waypoints.on_ramp_merge);
                        waypoints_leader.push_back(selected_waypoints.off_ramp_start);
                        waypoints_leader.push_back(selected_waypoints.off_ramp_merge);
                    }
                    else
                    {
                        waypoints_leader.push_back(selected_waypoints.off_ramp_merge);
                        waypoints_leader.push_back(selected_waypoints.off_ramp_start);
                        waypoints_leader.push_back(selected_waypoints.on_ramp_merge);
                        waypoints_leader.push_back(selected_waypoints.on_ramp_start);
                    }
                }

                float robot_radius = globals.ROBOT_RADIUS;
                Color leader_color;
                Color follower_color;

                // Assign distinct colors to each group
                switch (group)
                {
                case 0:
                    leader_color = GREEN;
                    follower_color = GREEN;
                    break;
                case 1:
                    leader_color = MAGENTA;
                    print("next_rid_: ", next_rid_);
                    follower_color = MAGENTA;
                    break;
                case 2:
                    leader_color = BLUE;
                    follower_color = BLUE;
                    break;
                case 3:
                    leader_color = RED;
                    follower_color = RED;
                    break;
                }
                // Create the leader robot for each group
                robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints_leader, robot_radius, leader_color, true, -1));

                // Create follower robots
                for (int i = 1; i <= globals.NUM_ROBOTS; i++) // Create follower robots
                {
                    Eigen::VectorXd starting_position;
                    if (travel_on_highway)
                    {
                        if (group == 2)
                        {
                            starting_position = selected_waypoints.lane_start;
                        }
                        else
                        {
                            starting_position = selected_waypoints.lane_end;
                        }
                    }
                    else
                    {
                        if (!flip_ramps)
                        {
                            starting_position = selected_waypoints.on_ramp_start;
                        }
                        else
                        {
                            starting_position = selected_waypoints.off_ramp_merge;
                        }
                    }
                    std::deque<Eigen::VectorXd> waypoints{starting_position, starting_position};
                    if (next_rid_ == 5)
                        follower_color = YELLOW;
                    // Create the follower robot with distinct color
                    robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints, robot_radius, follower_color, true, -1));
                }
            }
        }

        if (!robots_.empty())
        {
            // Update the follower robots to follow the node in front of them
            for (int group = 0; group < 4; ++group)
            {
                int start_index = group * (globals.NUM_ROBOTS + 1);
                for (int i = start_index + 1; i <= start_index + globals.NUM_ROBOTS; i++)
                {
                    auto target = robots_.at(i - 1);
                    auto follower = robots_.at(i);
                    Eigen::VectorXd offset_from_target = Eigen::VectorXd{{0.0, 0.0, 0.0, 0.0}};
                    follower->waypoints_[0] = target->position_ - offset_from_target;
                }
            }

            // Check if robot 3 has passed its second waypoint by checking its queue size
            auto leader = robots_.at(3); // Robot 3
            if (leader->waypoints_.size() < 2)
            {
                // Set robot 5 to follow robot 11
                auto follower = robots_.at(5);
                auto new_leader = robots_.at(11); // Robot 11
                follower->waypoints_.clear();     // Clear existing waypoints
                follower->waypoints_.push_back(new_leader->position_);

                // Update robot 5's following behavior to continue following robot 11
                for (int i = 6; i <= globals.NUM_ROBOTS; i++)
                {
                    auto target = robots_.at(i - 1);
                    auto follower = robots_.at(i);
                    Eigen::VectorXd offset_from_target = Eigen::VectorXd{{0.0, 0.0, 0.0, 0.0}};
                    follower->waypoints_[0] = target->position_ - offset_from_target;
                }
            }

            // Ensure robot 5 continues to follow robot 11 if the switch has already been made
            auto follower = robots_.at(5);
            if (follower->waypoints_.size() == 2 && follower->waypoints_[0] == robots_.at(11)->position_)
            {
                auto new_leader = robots_.at(11); // Robot 11
                Eigen::VectorXd offset_from_target = Eigen::VectorXd{{0.0, 0.0, 0.0, 0.0}};
                follower->waypoints_[0] = new_leader->position_ - offset_from_target;
            }
        }
    }
    else if (globals.FORMATION == "highway-rogue")
    {
        auto highwayWaypoints = calculateHighwayWaypoints();

        new_robots_needed_ = true;
        // Robot count and time
        if (clock_ % 20 == 0 && next_rid_ < 4 * globals.NUM_ROBOTS + 4) // Create 2 leaders + globals.NUM_ROBOTS followers per group for 4 groups
        {
            for (int group = 0; group < 4; ++group)
            {
                bool travel_on_highway = (group == 2 || group == 3);
                int lane = random_int(0, 2 - 1);
                bool flip_ramps = (group == 1);

                auto waypoints = highwayWaypoints[group][lane];
                auto &selected_waypoints = flip_ramps ? waypoints.second : waypoints.first;

                std::deque<Eigen::VectorXd> waypoints_leader;

                if (travel_on_highway)
                {
                    if (group == 2) // Left to right on road 0
                    {
                        waypoints_leader.push_back(selected_waypoints.lane_start);
                        waypoints_leader.push_back(selected_waypoints.on_ramp_merge);
                        selected_waypoints = highwayWaypoints[2][0].first;
                        waypoints_leader.push_back(selected_waypoints.off_ramp_start);
                        waypoints_leader.push_back(selected_waypoints.off_ramp_merge);
                    }
                    else if (group == 3) // Right to left on road 1
                    {
                        selected_waypoints = highwayWaypoints[3][1].second;
                        waypoints_leader.push_back(selected_waypoints.lane_end);
                        waypoints_leader.push_back(selected_waypoints.off_ramp_start);
                        waypoints_leader.push_back(selected_waypoints.on_ramp_merge);
                        selected_waypoints = highwayWaypoints[3][0].second;
                        waypoints_leader.push_back(selected_waypoints.lane_start);
                    }
                }
                else
                {
                    if (!flip_ramps)
                    {
                        waypoints_leader.push_back(selected_waypoints.on_ramp_start);
                        waypoints_leader.push_back(selected_waypoints.on_ramp_merge);
                        waypoints_leader.push_back(selected_waypoints.off_ramp_start);
                        waypoints_leader.push_back(selected_waypoints.off_ramp_merge);
                    }
                    else
                    {
                        waypoints_leader.push_back(selected_waypoints.off_ramp_merge);
                        waypoints_leader.push_back(selected_waypoints.off_ramp_start);
                        waypoints_leader.push_back(selected_waypoints.on_ramp_merge);
                        waypoints_leader.push_back(selected_waypoints.on_ramp_start);
                    }
                }

                float robot_radius = globals.ROBOT_RADIUS;
                Color leader_color;
                Color follower_color;

                // Assign distinct colors to each group
                switch (group)
                {
                case 0:
                    leader_color = GREEN;
                    follower_color = GREEN;
                    break;
                case 1:
                    leader_color = MAGENTA;
                    print("next_rid_: ", next_rid_);
                    follower_color = MAGENTA;
                    break;
                case 2:
                    leader_color = BLUE;
                    follower_color = BLUE;
                    break;
                case 3:
                    leader_color = RED;
                    follower_color = RED;
                    break;
                }
                // Create the leader robot for each group
                robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints_leader, robot_radius, leader_color, true, -1));

                // Create follower robots
                for (int i = 1; i <= globals.NUM_ROBOTS; i++) // Create follower robots
                {
                    Eigen::VectorXd starting_position;
                    if (travel_on_highway)
                    {
                        if (group == 2)
                        {
                            starting_position = selected_waypoints.lane_start;
                        }
                        else
                        {
                            starting_position = selected_waypoints.lane_end;
                        }
                    }
                    else
                    {
                        if (!flip_ramps)
                        {
                            starting_position = selected_waypoints.on_ramp_start;
                        }
                        else
                        {
                            starting_position = selected_waypoints.off_ramp_merge;
                        }
                    }
                    std::deque<Eigen::VectorXd> waypoints{starting_position, starting_position};
                    if (next_rid_ == 5)
                        follower_color = YELLOW;
                    // Create the follower robot with distinct color
                    robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints, robot_radius, follower_color, true, -1));
                }
            }
        }

        // Generate solo agents every few frames
        if (clock_ % 20 == 0) // Adjust the frequency as needed
        {
            int num_solo_agents = 1; // Number of solo agents to create
            for (int i = 0; i < num_solo_agents; ++i)
            {
                int direction = random_int(0, 1); // 0 for left to right, 1 for right to left
                int lane = random_int(0, 1);      // Randomly pick between lanes

                auto waypoints2 = (direction == 0) ? highwayWaypoints[direction][lane] : highwayWaypoints[direction + 2][lane];
                auto &selected_waypoints = waypoints2.first;

                Eigen::VectorXd starting_position;
                Eigen::VectorXd ending_position;

                if (direction == 0) // Left to right
                {
                    starting_position = selected_waypoints.lane_start;
                    ending_position = selected_waypoints.lane_end;
                }
                else // Right to left
                {
                    starting_position = selected_waypoints.lane_end;
                    ending_position = selected_waypoints.lane_start;
                }

                std::deque<Eigen::VectorXd> waypoints3{starting_position, ending_position};

                // Define solo agent radius and color
                float solo_robot_radius = globals.ROBOT_RADIUS;
                Color solo_color = BLACK;
                robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints3, solo_robot_radius, solo_color, true, -1));
            }
            // Delete robots if out of bounds
            for (auto [rid, robot] : robots_)
            {
                if ((abs(robot->position_(0)) > globals.WORLD_SZ / 2 || abs(robot->position_(1)) > globals.WORLD_SZ / 2) && robot->color_.r == 0 && robot->color_.g == 0 && robot->color_.b == 0)
                {
                    robots_to_delete.push_back(robot);
                }
            }
        }

        if (!robots_.empty())
        {
            // Update the follower robots to follow the node in front of them
            for (int group = 0; group < 4; ++group)
            {
                int start_index = group * (globals.NUM_ROBOTS + 1);
                for (int i = start_index + 1; i <= start_index + globals.NUM_ROBOTS; i++)
                {
                    auto target = robots_.at(i - 1);
                    auto follower = robots_.at(i);
                    Eigen::VectorXd offset_from_target = Eigen::VectorXd{{0.0, 0.0, 0.0, 0.0}};
                    follower->waypoints_[0] = target->position_ - offset_from_target;
                }
            }

            // Check if robot 3 has passed its second waypoint by checking its queue size
            auto leader = robots_.at(3); // Robot 3
            if (leader->waypoints_.size() < 2)
            {
                // Set robot 5 to follow robot 11
                auto follower = robots_.at(5);
                auto new_leader = robots_.at(11); // Robot 11
                follower->waypoints_.clear();     // Clear existing waypoints
                follower->waypoints_.push_back(new_leader->position_);

                // Update robot 5's following behavior to continue following robot 11
                for (int i = 6; i <= globals.NUM_ROBOTS; i++)
                {
                    auto target = robots_.at(i - 1);
                    auto follower = robots_.at(i);
                    Eigen::VectorXd offset_from_target = Eigen::VectorXd{{0.0, 0.0, 0.0, 0.0}};
                    follower->waypoints_[0] = target->position_ - offset_from_target;
                }
            }

            // Ensure robot 5 continues to follow robot 11 if the switch has already been made
            auto follower = robots_.at(5);
            if (follower->waypoints_.size() == 2 && follower->waypoints_[0] == robots_.at(11)->position_)
            {
                auto new_leader = robots_.at(11); // Robot 11
                Eigen::VectorXd offset_from_target = Eigen::VectorXd{{0.0, 0.0, 0.0, 0.0}};
                follower->waypoints_[0] = new_leader->position_ - offset_from_target;
            }
        }
    }
    else if (globals.FORMATION == "highway-pair")
    {
        auto highwayWaypoints = calculateHighwayWaypoints();
        // Add this counter at the beginning of your code
        new_robots_needed_ = true;
        // Robot count and time
        if (clock_ % 20 == 0 && next_rid_ < 3) // Create 1 leader, 1 follower, and 1 additional robot
        {
            bool travel_on_highway = false; // Assuming a single road without highway conditions
            int lane = random_int(0, 1);    // Assuming only one lane
            bool flip_ramps = false;        // No need to flip ramps for a single road

            auto waypoints = highwayWaypoints[0][lane];
            auto &selected_waypoints = waypoints.first; // Choose the first set of waypoints

            std::deque<Eigen::VectorXd> waypoints_leader;
            waypoints_leader.push_back(selected_waypoints.lane_start);
            waypoints_leader.push_back(selected_waypoints.on_ramp_merge);
            waypoints_leader.push_back(selected_waypoints.lane_end);

            float robot_radius = globals.ROBOT_RADIUS;
            Color leader_color = GREEN;          // Assign color for the leader
            Color follower_color = GREEN;        // Assign color for the follower
            Color additional_robot_color = BLUE; // Assign a distinct color for the additional robot

            // Create the leader robot
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints_leader, robot_radius, leader_color, true, -1));

            // Create the follower robot
            Eigen::VectorXd starting_position = selected_waypoints.lane_start; // Starting position for the follower
            std::deque<Eigen::VectorXd> waypoints_follower{starting_position, starting_position};
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints_follower, robot_radius, follower_color, true, -1));

            // Create the additional robot
            std::deque<Eigen::VectorXd> waypoints_additional_robot;
            waypoints_additional_robot.push_back(selected_waypoints.on_ramp_start);
            waypoints_additional_robot.push_back(selected_waypoints.on_ramp_merge);
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints_additional_robot, robot_radius, additional_robot_color, true, -1));
        }

        if (!robots_.empty())
        {
            // Update the follower robots to follow the node in front of them
            auto leader = robots_.at(0);           // Assuming the leader is at index 0
            auto follower = robots_.at(1);         // Assuming the follower is at index 1
            auto additional_robot = robots_.at(2); // Assuming the additional robot is at index 2

            Eigen::VectorXd offset_from_target = Eigen::VectorXd::Zero(4); // Adjust the dimension if necessary
            follower->waypoints_[0] = leader->position_ - offset_from_target;

            // Check if the leader has passed the second waypoint
            if (leader->waypoints_.size() < 2)
            {
                // Clear the additional robot's waypoints and make it follow the leader
                additional_robot->waypoints_.clear();
                additional_robot->waypoints_.push_back(leader->position_ - offset_from_target);

                // Update the additional robot's following behavior to continue following the leader
                additional_robot->waypoints_[0] = leader->position_ - offset_from_target;

                follower->waypoints_[0] = additional_robot->position_ - offset_from_target;
            }
        }
    }
    else
    {
        print("Shouldn't reach here, formation not defined!");
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
