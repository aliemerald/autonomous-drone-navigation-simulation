#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>

#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <map>
#include <limits> // For std::numeric_limits

using namespace webots;


#ifndef CLAMP
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#endif
#define M_PI 3.14159265358979323846

struct Waypoint { double x, y, z; };
enum GridCellState { FREE_SPACE = 0, OBSTACLE = 1, UNKNOWN = 2 };

enum DroneState {
    TAKEOFF,
    EXPLORING,
    PLANNING_TO_SUBGOAL,
    NAVIGATING_TO_SUBGOAL,
    REACHED_GOAL,
    STUCK
};

// Sub-states for the EXPLORING phase
enum ExplorationState {
    SCANNING,
    FINDING_FRONTIER,
    SELECTING_SUBGOAL
};

void print_log(double time, const std::string &message) {
    std::cout << std::fixed << std::setprecision(3) << "[" << time << "] " << message << std::endl;
}

namespace Pathfinder {
    struct Node {
        int y, x, parent_y, parent_x; // y is row, x is column
        double g_cost, h_cost, f_cost;
        bool operator>(const Node& other) const { return this->f_cost > other.f_cost; }
    };

    bool isValid(int x, int y, int width, int height) {
        return (x >= 0) && (x < width) && (y >= 0) && (y < height);
    }

    bool isTraversable(const std::vector<std::vector<int>>& grid, int x, int y) {
        return grid[y][x] == FREE_SPACE;
    }

    void inflateObstacleCircular(std::vector<std::vector<int>>& grid, int x_center, int y_center, double drone_radius_m, double grid_resolution, int width, int height) {
        int radius_cells = static_cast<int>(std::ceil(drone_radius_m / grid_resolution));
        for (int i = -radius_cells; i <= radius_cells; ++i) {
            for (int j = -radius_cells; j <= radius_cells; ++j) {
                if (std::sqrt(i * i + j * j) <= radius_cells) {
                    if (isValid(x_center + j, y_center + i, width, height)) {
                        grid[y_center + i][x_center + j] = OBSTACLE;
                    }
                }
            }
        }
    }

    void printGrid(const std::vector<std::vector<int>>& grid, int start_x, int start_y, int end_x, int end_y, const std::vector<Waypoint>& path_waypoints, const Waypoint& grid_origin, double grid_resolution, int final_goal_grid_x, int final_goal_grid_y, double time_current) {
        std::cout << "Grid Map at Time: " << std::fixed << std::setprecision(3) << time_current << " ---" << std::endl;
        std::cout << "  (World +Y / North is Up, World +X / East is Right)" << std::endl;
        std::vector<std::vector<char>> display_grid(grid.size(), std::vector<char>(grid[0].size()));
        for (int y = 0; y < (int)grid.size(); ++y) {
            for (int x = 0; x < (int)grid[y].size(); ++x) {
                if (grid[y][x] == OBSTACLE) display_grid[y][x] = '#';
                else if (grid[y][x] == FREE_SPACE) display_grid[y][x] = '.';
                else display_grid[y][x] = '?';
            }
        }

        for (const auto& wp : path_waypoints) {
            // World X increasing -> Grid X increasing
            // World Y increasing (North) -> Grid Y decreasing
            int path_grid_x = static_cast<int>((wp.x - grid_origin.x) / grid_resolution);
            int path_grid_y = grid.size() - 1 - static_cast<int>((wp.y - grid_origin.y) / grid_resolution);

            if (isValid(path_grid_x, path_grid_y, grid[0].size(), grid.size()) && display_grid[path_grid_y][path_grid_x] == '.') {
                display_grid[path_grid_y][path_grid_x] = '*';
            }
        }

        // Convert start, end, and final goal to grid coordinates for display
        if (isValid(start_x, start_y, grid[0].size(), grid.size())) display_grid[start_y][start_x] = 'S';
        if (isValid(end_x, end_y, grid[0].size(), grid.size()) && end_x != -1) display_grid[end_y][end_x] = 'E'; // Only show 'E' if a valid end_x is provided
        if (isValid(final_goal_grid_x, final_goal_grid_y, grid[0].size(), grid.size())) {
            display_grid[final_goal_grid_y][final_goal_grid_x] = 'F';
        }

        for (int y = 0; y < (int)grid.size(); ++y) {
            std::cout << std::setw(3) << y << "|"; // Grid Row
            for (int x = 0; x < (int)grid[y].size(); ++x) {
                std::cout << display_grid[y][x];
            }
            std::cout << '|' << std::endl;
        }
        std::cout << "-------------------" << std::endl;
    }

    double calculateHValue(int x, int y, int dest_x, int dest_y) { return sqrt(pow(x - dest_x, 2) + pow(y - dest_y, 2)); }

    std::vector<Waypoint> tracePath(const std::map<int, Node>& closed_list, int dest_x, int dest_y, int width, int height, double target_z, const Waypoint& grid_origin, double grid_resolution, double time_current) {
        std::vector<Waypoint> path;
        int current_x = dest_x, current_y = dest_y;
        
        print_log(time_current, "Tracing path from grid destination (" + std::to_string(dest_x) + ", " + std::to_string(dest_y) + ") back to start.");
        
        while (true) {
            auto it = closed_list.find(current_y * width + current_x);
            if (it == closed_list.end()) {
                print_log(time_current, "Error: Node (" + std::to_string(current_x) + ", " + std::to_string(current_y) + ") not found in closed list during trace.");
                break; // Should not happen for a valid path end node
            }
            Node current_node = it->second;

            // Convert grid coordinates to world coordinates
            double world_x = grid_origin.x + current_node.x * grid_resolution + grid_resolution / 2.0;
            double world_y = grid_origin.y + (height - 1 - current_node.y) * grid_resolution + grid_resolution / 2.0;

            path.push_back({world_x, world_y, target_z});
            
            if (current_node.parent_x == current_x && current_node.parent_y == current_y) {
                print_log(time_current, "Reached start node in path trace.");
                break; // Reached start node
            }
            int temp_x = current_node.parent_x, temp_y = current_node.parent_y;
            current_x = temp_x;
            current_y = temp_y;
        }
        std::reverse(path.begin(), path.end());
        print_log(time_current, "Path trace complete. Path has " + std::to_string(path.size()) + " waypoints.");
        return path;
    }

    std::vector<Waypoint> aStarSearch(const std::vector<std::vector<int>>& grid, int start_x, int start_y, int dest_x, int dest_y, double target_z, const Waypoint& grid_origin, double grid_resolution, double time_current) {
        int width = grid[0].size();
        int height = grid.size();

        print_log(time_current, "A* Search: Start Grid (" + std::to_string(start_x) + ", " + std::to_string(start_y) + "), Dest Grid (" + std::to_string(dest_x) + ", " + std::to_string(dest_y) + ")");

        if (!isValid(start_x, start_y, width, height) || !isValid(dest_x, dest_y, width, height)) {
            print_log(time_current, "A* Error: Start or Destination is out of bounds.");
            return {};
        }
        if (!isTraversable(grid, start_x, start_y) || !isTraversable(grid, dest_x, dest_y)) {
            print_log(time_current, "A* Warning: Start or Destination is not in traversable space. Start traversable: " + std::to_string(isTraversable(grid, start_x, start_y)) + ", Dest traversable: " + std::to_string(isTraversable(grid, dest_x, dest_y)));
            // If start/dest is blocked, A* cannot find a path -> select new subgoal
            return {};
        }

        std::vector<Node> open_list;
        std::map<int, Node> closed_list;
        Node start_node = {start_y, start_x, start_y, start_x, 0.0, calculateHValue(start_x, start_y, dest_x, dest_y), 0.0};
        start_node.f_cost = start_node.g_cost + start_node.h_cost;
        open_list.push_back(start_node);

        // 8-directional movement
        int dY[] = {-1, 1, 0, 0, -1, -1, 1, 1}; // N, S, E, W, NW, NE, SW, SE
        int dX[] = {0, 0, 1, -1, -1, 1, -1, 1};

        long long iteration_count = 0; // Prevent infinite loops

        while (!open_list.empty()) {
            iteration_count++;
            if (iteration_count > width * height * 10) { // Safety break for very large grids or complex paths
                std::cerr << "A* Error: Exceeded max iterations (" << iteration_count << "). Likely stuck or no path." << std::endl;
                return {};
            }

            auto current_it = std::min_element(open_list.begin(), open_list.end(), [](const Node& a, const Node& b) { return a.f_cost < b.f_cost; });
            Node current_node = *current_it;
            open_list.erase(current_it);
            closed_list[current_node.y * width + current_node.x] = current_node;

            if (current_node.y == dest_y && current_node.x == dest_x) {
                print_log(time_current, "A* Found path to destination after " + std::to_string(iteration_count) + " iterations.");
                return tracePath(closed_list, dest_x, dest_y, width, height, target_z, grid_origin, grid_resolution, time_current);
            }

            for (int i = 0; i < 8; ++i) {
                int newY = current_node.y + dY[i];
                int newX = current_node.x + dX[i];

                if (isValid(newX, newY, width, height) && isTraversable(grid, newX, newY) && closed_list.find(newY * width + newX) == closed_list.end()) {
                    double cost_to_move = (i < 4) ? 1.0 : std::sqrt(2.0); // 1.0 for cardinal, sqrt(2) for diagonal
                    double g_new = current_node.g_cost + cost_to_move;
                    double h_new = calculateHValue(newX, newY, dest_x, dest_y);
                    double f_new = g_new + h_new;

                    auto it = std::find_if(open_list.begin(), open_list.end(), [&](const Node& n) { return n.x == newX && n.y == newY; });
                    if (it != open_list.end()) {
                        if (it->f_cost > f_new) {
                            *it = {newY, newX, current_node.y, current_node.x, g_new, h_new, f_new};
                        }
                    } else {
                        open_list.push_back({newY, newX, current_node.y, current_node.x, g_new, h_new, f_new});
                    }
                }
            }
        }
        print_log(time_current, "A* No path found.");
        return {}; // No path found
    }
}

void updateMapFromLidarChunk(std::vector<std::vector<int>>& grid, webots::Lidar* lidar, const Waypoint& current_pos, double raw_ned_yaw, const Waypoint& grid_origin, double grid_resolution, double drone_radius, int start_index, int points_to_process, double time_current) {
    const int GRID_WIDTH = grid[0].size();
    const int GRID_HEIGHT = grid[0].size();
    const float* ranges = lidar->getRangeImage();
    
    if (!ranges) return;
    double drone_yaw_for_lidar = raw_ned_yaw; 

    int resolution = lidar->getHorizontalResolution();
    double fov = lidar->getFov();
    int end_index = std::min(start_index + points_to_process, resolution);

    for (int i = start_index; i < end_index; ++i) {
        double beam_angle_relative = fov / 2.0 - (double)i / (resolution - 1) * fov;
        double effective_world_angle = drone_yaw_for_lidar + beam_angle_relative; 

        double range = ranges[i];
        double max_lidar_range = lidar->getMaxRange();
        bool definite_hit = (std::isfinite(range) && range > 0.0 && range < max_lidar_range - grid_resolution);
        
        double range_limit = definite_hit ? range : max_lidar_range;
        
        for (double d = 0.2; d < range_limit; d += grid_resolution) {
            double x_world = current_pos.x + d * std::cos(effective_world_angle);
            double y_world = current_pos.y + d * std::sin(effective_world_angle);

            int grid_x = static_cast<int>((x_world - grid_origin.x) / grid_resolution);
            int grid_y = GRID_HEIGHT - 1 - static_cast<int>((y_world - grid_origin.y) / grid_resolution);

            if (Pathfinder::isValid(grid_x, grid_y, GRID_WIDTH, GRID_HEIGHT) && grid[grid_y][grid_x] == UNKNOWN) {
                grid[grid_y][grid_x] = FREE_SPACE;
            }
        }

        if (definite_hit) {
            double obs_x_world = current_pos.x + range * std::cos(effective_world_angle);
            double obs_y_world = current_pos.y + range * std::sin(effective_world_angle);

            int obs_grid_x = static_cast<int>((obs_x_world - grid_origin.x) / grid_resolution);
            int obs_grid_y = GRID_HEIGHT - 1 - static_cast<int>((obs_y_world - grid_origin.y) / grid_resolution);

            if (Pathfinder::isValid(obs_grid_x, obs_grid_y, GRID_WIDTH, GRID_HEIGHT)) {
                Pathfinder::inflateObstacleCircular(grid, obs_grid_x, obs_grid_y, drone_radius, grid_resolution, GRID_WIDTH, GRID_HEIGHT);
            }
        }
    }
}

void findFrontierCellsChunk(std::vector<std::pair<int, int>>& frontier, const std::vector<std::vector<int>>& grid, int start_y, int rows_to_process, double time_current) {
    int height = grid.size();
    int width = grid[0].size();
    int end_y = std::min(start_y + rows_to_process, height);
    int dY[] = {-1, 1, 0, 0}; // Cardinal directions
    int dX[] = {0, 0, 1, -1};

    for (int y = start_y; y < end_y; ++y) {
        for (int x = 0; x < width; ++x) {
            if (grid[y][x] == FREE_SPACE) {
                // Check if any neighbor is UNKNOWN
                for (int i = 0; i < 4; ++i) {
                    int ny = y + dY[i];
                    int nx = x + dX[i];
                    if (Pathfinder::isValid(nx, ny, width, height) && grid[ny][nx] == UNKNOWN) {
                        frontier.push_back({x, y}); // Store the FREE_SPACE cell that is adjacent to UNKNOWN
                        break; // Found a frontier for this cell, move to next cell
                    }
                }
            }
        }
    }
}

std::pair<int, int> findBestSubgoal(const std::vector<std::pair<int, int>>& frontier_cells, int final_goal_x_grid, int final_goal_y_grid, double time_current) {
    if (frontier_cells.empty()) {
        print_log(time_current, "No frontier cells available to select a subgoal from.");
        return {-1, -1};
    }
    double min_dist_sq = -1.0;
    std::pair<int, int> best_subgoal = {-1, -1};

    print_log(time_current, "Selecting best subgoal from " + std::to_string(frontier_cells.size()) + " frontier cells. Final goal grid: (" + std::to_string(final_goal_x_grid) + ", " + std::to_string(final_goal_y_grid) + ")");

    for (const auto& cell : frontier_cells) {
        // Calculate squared Euclidean distance to the final goal
        double dist_sq = pow(cell.first - final_goal_x_grid, 2) + pow(cell.second - final_goal_y_grid, 2);
        if (min_dist_sq < 0 || dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_subgoal = cell;
        }
    }
    print_log(time_current, "Best subgoal selected (grid): (" + std::to_string(best_subgoal.first) + ", " + std::to_string(best_subgoal.second) + "), distance: " + std::to_string(std::sqrt(min_dist_sq)));
    return best_subgoal;
}

int main() {
    Supervisor *robot = new Supervisor(); // Supervisor for waypoint marker manipulation & path visualisation
    int timeStep = (int)robot->getBasicTimeStep();

    InertialUnit *imu = robot->getInertialUnit("inertial unit"); imu->enable(timeStep);
    GPS *gps = robot->getGPS("gps"); gps->enable(timeStep);
    Gyro *gyro = robot->getGyro("gyro"); gyro->enable(timeStep);
    Lidar *lidar = robot->getLidar("lidar"); lidar->enable(timeStep); lidar->enablePointCloud();
    
    Motor *motors[4];
    const char *motor_names[] = {"front left propeller", "front right propeller", "rear left propeller", "rear right propeller"};
    for (int i = 0; i < 4; ++i) {
        motors[i] = robot->getMotor(motor_names[i]);
        if (!motors[i]) { // Add a check to confirm motor was found
            print_log(robot->getTime(), std::string("Error: Motor '") + motor_names[i] + "' not found. Check Webots model.");
            return 1; // Exit if a critical device is missing
        }
        motors[i]->setPosition(std::numeric_limits<double>::infinity());
        motors[i]->setVelocity(1.0);
    }

    const Waypoint FINAL_DESTINATION = {8, 10, 2.0};
    Waypoint current_subgoal_world = {0.0, 0.0, 0.0};
    std::vector<Waypoint> a_star_path;
    size_t current_path_index = 0;
    DroneState currentState = TAKEOFF;
    double takeoff_altitude = 2.0;

    const int GRID_WIDTH = 120;
    const int GRID_HEIGHT = 120;
    const double GRID_RESOLUTION = 0.25;
    const double DRONE_SAFETY_RADIUS_METERS = 0.75;
    std::vector<std::vector<int>> grid(GRID_HEIGHT, std::vector<int>(GRID_WIDTH, UNKNOWN));

    const Waypoint grid_origin = {
        -GRID_WIDTH * GRID_RESOLUTION / 2.0,
        -GRID_HEIGHT * GRID_RESOLUTION / 2.0,
        0.0
    };
    print_log(robot->getTime(), "Grid Origin (World): X=" + std::to_string(grid_origin.x) + ", Y=" + std::to_string(grid_origin.y));

    ExplorationState currentExplorationState = SCANNING;
    int lidar_scan_progress = 0;
    int frontier_search_progress_y = 0;
    std::vector<std::pair<int, int>> frontier_cells;

    const double k_max_yaw_disturbance = 1.5; 
    const double k_max_pos_disturbance = 8.0;
    const double k_vertical_thrust = 68.5, k_vertical_offset = 0.6, k_vertical_p = 3.0;
    const double k_roll_p = 50.0, k_pitch_p = 30.0;
    const double k_yaw_p = 1.0;
    const double k_pos_p = 0.4;
    const double k_pos_d = 0.6;
    
    Waypoint hover_position = {0.0, 0.0, 0.0};
    bool is_hover_pos_set = false;

    double last_x = 0.0, last_y = 0.0, last_time_pd = 0.0;
    double vx = 0.0, vy = 0.0;

    print_log(robot->getTime(), "Drone mission starting. State: TAKEOFF");

    Node *waypoint_markers_group = robot->getFromDef("WAYPOINT_MARKERS");
    Node *flight_path = robot->getFromDef("FLIGHT_PATH");
    Field *flight_path_coord = nullptr;
    Field *flight_path_coordIndex = nullptr;
    
    std::vector<Node*> waypoint_marker_nodes;
    std::vector<Field*> waypoint_marker_translations;
    
    if (flight_path) {
        Node *shape = flight_path->getField("children")->getMFNode(0);
        if (shape) {
            Node *geometry = shape->getField("geometry")->getSFNode();
            if (geometry) {
                flight_path_coord = geometry->getField("coord");
                flight_path_coordIndex = geometry->getField("coordIndex");
            }
        }
    }

    auto createWaypointMarkers = [&](const std::vector<Waypoint>& waypoints) {
        if (!waypoint_markers_group) return;
        
        // Clear existing markers
        Field *children_field = waypoint_markers_group->getField("children");
        int child_count = children_field->getCount();
        for (int i = child_count - 1; i >= 0; --i) {
            children_field->removeMF(i);
        }
        waypoint_marker_nodes.clear();
        waypoint_marker_translations.clear();
        
        // Create new markers for each waypoint
        for (size_t i = 0; i < waypoints.size(); ++i) {
            std::string marker_def = "WAYPOINT_MARKER_" + std::to_string(i);
            std::string marker_string = 
                "DEF " + marker_def + " Transform { "
                "translation " + std::to_string(waypoints[i].x) + " " + 
                                    std::to_string(waypoints[i].y) + " " + 
                                    std::to_string(waypoints[i].z) + " "
                "children [ "
                "Shape { "
                "appearance Appearance { "
                "material Material { diffuseColor 1 0 0 } "
                "} "
                "geometry Sphere { radius 0.1 } "
                "} ] }";
            
            children_field->importMFNodeFromString(i, marker_string);
            
            // Get the created node and its translation field
            Node *marker_node = children_field->getMFNode(i);
            if (marker_node) {
                waypoint_marker_nodes.push_back(marker_node);
                Field *translation = marker_node->getField("translation");
                waypoint_marker_translations.push_back(translation);
            }
        }
    };

    // Function to update progressive flight path
    std::vector<Waypoint> traveled_path;
    auto updateProgressiveFlightPath = [&](const std::vector<Waypoint>& path) {
        if (!flight_path_coord || !flight_path_coordIndex) return;
        
        // Only proceed if we have at least 2 points
        if (path.size() < 2) return;
        
        Node *coord_node = flight_path_coord->getSFNode();
        if (coord_node) {
            Field *point_field = coord_node->getField("point");
            
            // Get current point count
            int current_point_count = point_field->getCount();
            int new_points_needed = path.size() - current_point_count;
            
            // Only add new points, don't clear existing ones
            if (new_points_needed > 0) {
                for (int i = 0; i < new_points_needed; ++i) {
                    int point_index = current_point_count + i;
                    double point[3] = {path[point_index].x, path[point_index].y, path[point_index].z};
                    point_field->insertMFVec3f(point_index, point);
                }
                
                // Update coordIndex only when we add new points
                int current_coord_count = flight_path_coordIndex->getCount();
                
                // Remove the old terminator (-1) if it exists
                if (current_coord_count > 0) {
                    int last_index = flight_path_coordIndex->getMFInt32(current_coord_count - 1);
                    if (last_index == -1) {
                        flight_path_coordIndex->removeMF(current_coord_count - 1);
                        current_coord_count--;
                    }
                }
                
                // Add indices for new points
                for (int i = 0; i < new_points_needed; ++i) {
                    int coord_index = current_point_count + i;
                    flight_path_coordIndex->insertMFInt32(current_coord_count + i, coord_index);
                }
                
                // Add the terminator
                flight_path_coordIndex->insertMFInt32(current_coord_count + new_points_needed, -1);
            }
        }
    };

    auto hideWaypointMarker = [&](size_t index) {
        if (index < waypoint_marker_translations.size() && waypoint_marker_translations[index]) {
            double hidden_pos[3] = {1000.0, 1000.0, 1000.0};
            waypoint_marker_translations[index]->setSFVec3f(hidden_pos);
        }
    };


    while (robot->step(timeStep) != -1) {
        double time = robot->getTime();
        const double roll = imu->getRollPitchYaw()[0], pitch = imu->getRollPitchYaw()[1], yaw = imu->getRollPitchYaw()[2];
        const double* gps_values = gps->getValues();
        const Waypoint current_pos = {gps_values[0], gps_values[1], gps_values[2]};
        const double roll_velocity = gyro->getValues()[0], pitch_velocity = gyro->getValues()[1];

        double dt = time - last_time_pd;
        if (dt > 0.0) {
            vx = (current_pos.x - last_x) / dt;
            vy = (current_pos.y - last_y) / dt;
        }
        last_x = current_pos.x;
        last_y = current_pos.y;
        last_time_pd = time;
        
        double roll_disturbance = 0.0, pitch_disturbance = 0.0, yaw_disturbance = 0.0, vertical_input = 0.0;

        // Mark current drone position as FREE_SPACE
        // Convert current drone position to grid coordinates using the same logic as other conversions
        int drone_grid_x = static_cast<int>((current_pos.x - grid_origin.x) / GRID_RESOLUTION);
        int drone_grid_y = GRID_HEIGHT - 1 - static_cast<int>((current_pos.y - grid_origin.y) / GRID_RESOLUTION);
        if (Pathfinder::isValid(drone_grid_x, drone_grid_y, GRID_WIDTH, GRID_HEIGHT)) {
            grid[drone_grid_y][drone_grid_x] = FREE_SPACE;
        }

        switch (currentState) {
            case TAKEOFF: {
                vertical_input = k_vertical_p * std::pow(CLAMP(takeoff_altitude - current_pos.z + k_vertical_offset, -1.0, 1.0), 3.0);
                if (current_pos.z > takeoff_altitude - 0.1) {
                    currentState = EXPLORING;
                    currentExplorationState = SCANNING;
                    lidar_scan_progress = 0;
                    print_log(time, "Takeoff complete. State: EXPLORING (Sub-state: SCANNING)");
                }
                break;
            }

            case EXPLORING: {
                switch(currentExplorationState) {
                    case SCANNING: {
                        int lidar_resolution = lidar->getHorizontalResolution();
                        int points_per_step = 60;
                        updateMapFromLidarChunk(grid, lidar, current_pos, yaw, grid_origin, GRID_RESOLUTION, DRONE_SAFETY_RADIUS_METERS, lidar_scan_progress, points_per_step, time);
                        lidar_scan_progress += points_per_step;

                        if (lidar_scan_progress >= lidar_resolution) {
                            lidar_scan_progress = 0; // Reset for next full scan
                            currentExplorationState = FINDING_FRONTIER;
                            frontier_search_progress_y = 0; // Reset for full frontier search
                            frontier_cells.clear(); // Clear old frontier cells
                            print_log(time, "Scan complete. Sub-state: FINDING_FRONTIER");
                        }
                        break;
                    }
                    case FINDING_FRONTIER: {
                        int rows_per_step = 20; // Process 20 rows of the grid for frontiers per step
                        findFrontierCellsChunk(frontier_cells, grid, frontier_search_progress_y, rows_per_step, time);
                        frontier_search_progress_y += rows_per_step;

                        if (frontier_search_progress_y >= GRID_HEIGHT) {
                            frontier_search_progress_y = 0; // Reset for next frontier search
                            currentExplorationState = SELECTING_SUBGOAL;
                            print_log(time, "Frontier search complete. Found " + std::to_string(frontier_cells.size()) + " cells. Sub-state: SELECTING_SUBGOAL");
                        }
                        break;
                    }
                    case SELECTING_SUBGOAL: {
                        double dist_to_final = std::sqrt(pow(current_pos.x - FINAL_DESTINATION.x, 2) + pow(current_pos.y - FINAL_DESTINATION.y, 2));
                        if (dist_to_final < 1.0) { // If final destination is very close
                            currentState = REACHED_GOAL;
                            print_log(time, "Final destination is within reach. State: REACHED_GOAL");
                            break;
                        }
                        
                        if (frontier_cells.empty()) {
                            print_log(time, "No frontier found. Cannot explore further. State: STUCK");
                            currentState = STUCK;
                            break;
                        }

                        // Convert final destination to grid coordinates using the same logic as other conversions
                        int final_dest_grid_x = static_cast<int>((FINAL_DESTINATION.x - grid_origin.x) / GRID_RESOLUTION);
                        int final_dest_grid_y = GRID_HEIGHT - 1 - static_cast<int>((FINAL_DESTINATION.y - grid_origin.y) / GRID_RESOLUTION);
                        print_log(time, "Final Destination World: (" + std::to_string(FINAL_DESTINATION.x) + ", " + std::to_string(FINAL_DESTINATION.y) + ") -> Grid: (" + std::to_string(final_dest_grid_x) + ", " + std::to_string(final_dest_grid_y) + ")");

                        auto subgoal_grid = findBestSubgoal(frontier_cells, final_dest_grid_x, final_dest_grid_y, time);

                        if(subgoal_grid.first == -1) { // No valid subgoal found
                            print_log(time, "Could not determine a valid subgoal. State: STUCK");
                            currentState = STUCK;
                            break;
                        }

                        // Convert selected subgoal grid coordinates back to world coordinates
                        current_subgoal_world = {
                            grid_origin.x + subgoal_grid.first * GRID_RESOLUTION + GRID_RESOLUTION / 2.0,
                            grid_origin.y + (GRID_HEIGHT - 1 - subgoal_grid.second) * GRID_RESOLUTION + GRID_RESOLUTION / 2.0,
                            takeoff_altitude // Maintain current flight altitude
                        };

                        print_log(time, "Selected Subgoal Grid: (" + std::to_string(subgoal_grid.first) + ", " + std::to_string(subgoal_grid.second) + ")");
                        print_log(time, "New subgoal selected (World): (" + std::to_string(current_subgoal_world.x) + ", " + std::to_string(current_subgoal_world.y) + ", " + std::to_string(current_subgoal_world.z) + "). State: PLANNING_TO_SUBGOAL");
                        currentState = PLANNING_TO_SUBGOAL;
                        break;
                    }
                }
                vertical_input = k_vertical_p * std::pow(CLAMP(takeoff_altitude - current_pos.z + k_vertical_offset, -1.0, 1.0), 3.0);
                break;
            }

            case PLANNING_TO_SUBGOAL: {
                // Convert current drone position to grid coordinates
                int start_x = static_cast<int>((current_pos.x - grid_origin.x) / GRID_RESOLUTION);
                int start_y = GRID_HEIGHT - 1 - static_cast<int>((current_pos.y - grid_origin.y) / GRID_RESOLUTION);
                print_log(time, "Current Drone Position World: (" + std::to_string(current_pos.x) + ", " + std::to_string(current_pos.y) + ") -> Grid: (" + std::to_string(start_x) + ", " + std::to_string(start_y) + ")");


                // Convert subgoal world coordinates to grid coordinates
                int dest_x = static_cast<int>((current_subgoal_world.x - grid_origin.x) / GRID_RESOLUTION);
                int dest_y = GRID_HEIGHT - 1 - static_cast<int>((current_subgoal_world.y - grid_origin.y) / GRID_RESOLUTION);
                print_log(time, "Current Subgoal World: (" + std::to_string(current_subgoal_world.x) + ", " + std::to_string(current_subgoal_world.y) + ") -> Grid: (" + std::to_string(dest_x) + ", " + std::to_string(dest_y) + ")");


                // Ensure start and destination are marked as free for pathfinding
                if (Pathfinder::isValid(start_x, start_y, GRID_WIDTH, GRID_HEIGHT)) grid[start_y][start_x] = FREE_SPACE;
                if (Pathfinder::isValid(dest_x, dest_y, GRID_WIDTH, GRID_HEIGHT)) grid[dest_y][dest_x] = FREE_SPACE;

                int final_dest_grid_x = static_cast<int>((FINAL_DESTINATION.x - grid_origin.x) / GRID_RESOLUTION);
                int final_dest_grid_y = GRID_HEIGHT - 1 - static_cast<int>((FINAL_DESTINATION.y - grid_origin.y) / GRID_RESOLUTION);

                // Print grid BEFORE A* search
                print_log(time, "Grid state before A* search:");
                Pathfinder::printGrid(grid, start_x, start_y, dest_x, dest_y, {}, grid_origin, GRID_RESOLUTION, final_dest_grid_x, final_dest_grid_y, time);


                print_log(time, "Calculating path to subgoal with A*");
                a_star_path = Pathfinder::aStarSearch(grid, start_x, start_y, dest_x, dest_y, takeoff_altitude, grid_origin, GRID_RESOLUTION, time);

                if (!a_star_path.empty()) {
                    print_log(time, "Path to subgoal found! State: NAVIGATING_TO_SUBGOAL. Path length: " + std::to_string(a_star_path.size()) + " waypoints.");
                    current_path_index = 0;
                    currentState = NAVIGATING_TO_SUBGOAL;
                    // This printGrid call remains to show the final planned path
                    Pathfinder::printGrid(grid, start_x, start_y, dest_x, dest_y, a_star_path, grid_origin, GRID_RESOLUTION, final_dest_grid_x, final_dest_grid_y, time);
                    
                    // Create waypoint markers for all planned waypoints
                    createWaypointMarkers(a_star_path); 
                    
                    // Initialize traveled path with starting position
                    traveled_path.clear(); 
                    traveled_path.push_back({current_pos.x, current_pos.y, current_pos.z}); 
                    // Add the first waypoint immediately to ensure we have 2 points for line drawing
                    if (!a_star_path.empty()) { 
                        traveled_path.push_back(a_star_path[0]); 
                    }

                } else {
                    print_log(time, "Failed to find a path to subgoal. Marking subgoal as obstacle and re-exploring.");
                    if (Pathfinder::isValid(dest_x, dest_y, GRID_WIDTH, GRID_HEIGHT)) {
                            grid[dest_y][dest_x] = OBSTACLE; // Mark the unreachable subgoal as an obstacle
                    }
                    currentState = EXPLORING;
                    currentExplorationState = SCANNING; // Go back to scanning to update map
                    lidar_scan_progress = 0;
                }
                break;
            }

            case NAVIGATING_TO_SUBGOAL: {
                if (current_path_index >= a_star_path.size()) {
                    print_log(time, "Subgoal reached. Returning to EXPLORING to find next frontier.");
                    currentState = EXPLORING;
                    currentExplorationState = SCANNING; // Start new scan after reaching subgoal
                    lidar_scan_progress = 0;
                    break;
                }

                Waypoint target = a_star_path[current_path_index];
                
                double dx = target.x - current_pos.x;
                double dy = target.y - current_pos.y;
                double distance_to_wp = std::sqrt(dx * dx + dy * dy);
                double target_bearing = std::atan2(dy, dx);
                double heading_error = target_bearing - yaw;

                if (heading_error > M_PI) heading_error -= 2 * M_PI;
                if (heading_error < -M_PI) heading_error += 2 * M_PI;

                static double last_nav_print_time = 0.0;
                if (time - last_nav_print_time >= 0.5) { // Print navigation updates every 0.5 seconds
                    print_log(time, "Navigating to waypoint " + std::to_string(current_path_index) + ": Target World (" + std::to_string(target.x) + ", " + std::to_string(target.y) + ", " + std::to_string(target.z) + ")");
                    print_log(time, "Current Drone Position World: (" + std::to_string(current_pos.x) + ", " + std::to_string(current_pos.y) + ", " + std::to_string(current_pos.z) + ")");
                    print_log(time, "Distance to WP: " + std::to_string(distance_to_wp) + ", Heading Error: " + std::to_string(heading_error));
                    last_nav_print_time = time;
                }

                yaw_disturbance = CLAMP(k_yaw_p * heading_error, -k_max_yaw_disturbance, k_max_yaw_disturbance);

                double forward_error = dx * std::cos(yaw) + dy * std::sin(yaw);
                double sideways_error = -dx * std::sin(yaw) + dy * std::cos(yaw);

                pitch_disturbance = CLAMP(-k_pos_p * forward_error, -k_max_pos_disturbance, k_max_pos_disturbance);
                roll_disturbance = CLAMP(-k_pos_p * sideways_error, -k_max_pos_disturbance, k_max_pos_disturbance);

                static double last_path_update_time = 0.0; 
                static double last_recorded_x = current_pos.x; 
                static double last_recorded_y = current_pos.y; 
                
                if (time - last_path_update_time > 0.2) { 
                    double moved_distance = std::sqrt(std::pow(current_pos.x - last_recorded_x, 2) + 
                                                      std::pow(current_pos.y - last_recorded_y, 2)); 
                    
                    // Only add point if drone has moved at least 0.3 meters 
                    if (moved_distance > 0.3 || traveled_path.size() < 2) { 
                        traveled_path.push_back({current_pos.x, current_pos.y, current_pos.z}); 
                        
                        if (traveled_path.size() > 200) { 
                            traveled_path.erase(traveled_path.begin()); 
                        }
                        
                        updateProgressiveFlightPath(traveled_path); 
                        
                        last_recorded_x = current_pos.x; 
                        last_recorded_y = current_pos.y; 
                    }
                    last_path_update_time = time; 
                }

                if (distance_to_wp < 0.2) { // 0.2m tolerance
                    print_log(time, "Waypoint " + std::to_string(current_path_index) + " reached.");
                    hideWaypointMarker(current_path_index); 
                    current_path_index++;
                }
                
                vertical_input = k_vertical_p * std::pow(CLAMP(target.z - current_pos.z + k_vertical_offset, -1.0, 1.0), 3.0);
                
                break;
            }

            case REACHED_GOAL: {
                if (!is_hover_pos_set) {
                    hover_position = {current_pos.x, current_pos.y, current_pos.z};
                    is_hover_pos_set = true;
                    print_log(time, "Final Destination Reached! Holding position at (" + std::to_string(hover_position.x) + ", " + std::to_string(hover_position.y) + ")");
                }

                double dx = hover_position.x - current_pos.x;
                double dy = hover_position.y - current_pos.y;
                double p_forward_term = dx * std::cos(yaw) + dy * std::sin(yaw);
                double p_sideways_term = -dx * std::sin(yaw) + dy * std::cos(yaw);

                double d_forward_term = vx * std::cos(yaw) + vy * std::sin(yaw);
                double d_sideways_term = -vx * std::sin(yaw) + vy * std::cos(yaw);

                pitch_disturbance = CLAMP(-k_pos_p * p_forward_term - k_pos_d * d_forward_term, -k_max_pos_disturbance, k_max_pos_disturbance);
                roll_disturbance = CLAMP(-k_pos_p * p_sideways_term - k_pos_d * d_sideways_term, -k_max_pos_disturbance, k_max_pos_disturbance);
                
                vertical_input = k_vertical_p * std::pow(CLAMP(hover_position.z - current_pos.z + k_vertical_offset, -1.0, 1.0), 3.0);
                yaw_disturbance = 0;
                break;
            }
            case STUCK: {
                print_log(time, "Drone is stuck. Halting mission.");
                for (int i = 0; i < 4; ++i) {
                    motors[i]->setVelocity(0.0);
                }
                delete robot;
                return 0;
            }
        }

        double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
        double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;

        double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_disturbance;
        double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_disturbance;
        double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_disturbance;
        double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_disturbance;
 
        motors[0]->setVelocity(front_left_motor_input);
        motors[1]->setVelocity(-front_right_motor_input);
        motors[2]->setVelocity(-rear_left_motor_input);
        motors[3]->setVelocity(rear_right_motor_input);
    }

    print_log(robot->getTime(), "Mission ended.");
    for (int i = 0; i < 4; ++i) {
        motors[i]->setVelocity(0.0);
    }
    delete robot;
    return 0;
}
