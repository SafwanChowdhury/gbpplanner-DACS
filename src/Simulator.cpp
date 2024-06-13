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
        // Robots in a grid style city. There is only one-way traffic, and no turning.
        new_robots_needed_ = true; // This is needed so that more robots can be created as the simulation progresses.
        if (clock_ % 40 == 0)
        { // Arbitrary condition on the simulation time to create new robots
            // Assuming the grid is defined as a 2D array or list
            std::vector<std::vector<int>> grid = {
                {1, 1, 0, 0},
                {1, 0, 1, 1},
                {0, 1, 1, 0},
                {1, 1, 0, 1}};

            int n_roads = grid.size() * grid[0].size(); // Total possible roads
            int road_index = random_int(0, n_roads - 1);
            int row = road_index / grid[0].size();
            int col = road_index % grid[0].size();

            // Define cell size; this should be based on your grid configuration
            double cell_size = 10.0;               // Adjust cell size according to your grid dimensions
            int n_lanes = 2;                       // Define number of lanes
            int lane = random_int(0, n_lanes - 1); // Randomly choose a lane

            // Determine road orientation and position
            Eigen::Matrix4d rot;
            rot.setZero();
            if (grid[row][col] == 1)
            { // Horizontal road
                rot.topLeftCorner(2, 2) << 1, 0, 0, 1;
                rot.bottomRightCorner(2, 2) << 1, 0, 0, 1;
            }
            else
            { // Vertical road
                rot.topLeftCorner(2, 2) << 0, -1, 1, 0;
                rot.bottomRightCorner(2, 2) << 0, -1, 1, 0;
            }

            double lane_width = 4.0 * globals.ROBOT_RADIUS;
            double lane_v_offset = (0.5 * (1 - n_lanes) + lane) * lane_width;
            double additional_distance = 0.5; // Define the additional distance behind the master

            // Define starting and ending points for master and slave robots based on grid cell
            Eigen::VectorXd starting_master(4);
            Eigen::VectorXd ending_master(4);
            Eigen::VectorXd starting_slave(4);
            Eigen::VectorXd ending_slave(4);

            starting_master << -globals.WORLD_SZ / 2.0 + col * cell_size + additional_distance, row * cell_size + lane_v_offset, globals.MAX_SPEED, 0.0;
            ending_master << globals.WORLD_SZ / 2.0 + col * cell_size + additional_distance, row * cell_size + lane_v_offset, 0.0, 0.0;

            starting_slave << -globals.WORLD_SZ / 2.0 + col * cell_size, row * cell_size + lane_v_offset, globals.MAX_SPEED, 0.0;
            ending_slave << globals.WORLD_SZ / 2.0 + col * cell_size, row * cell_size + lane_v_offset, 0.0, 0.0;

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
    else
    {
        print("Shouldn't reach here, formation not defined!");
        // Define new formations here!
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
