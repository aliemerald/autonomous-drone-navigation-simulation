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

using namespace webots;

#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#define M_PI 3.14159265358979323846

void print_log(double time, const std::string &message) {
    std::cout << std::fixed << std::setprecision(3) << "[" << time << "] " << message << std::endl;
}

struct Waypoint { double x, y, z; };
enum DroneState { TAKEOFF, MAPPING, PLANNING, ALIGNING, NAVIGATING, MISSION_COMPLETE }; 

// --- Pathfinder & Debugger ---
namespace Pathfinder {
    struct Node {
        int y, x, parent_y, parent_x;
        double g_cost, h_cost, f_cost;
        bool operator>(const Node& other) const { return this->f_cost > other.f_cost; }
    };

    bool isValid(int x, int y, int width, int height) {
        return (x >= 0) && (x < width) && (y >= 0) && (y < height);
    }

    bool isUnblocked(const std::vector<std::vector<int>>& grid, int x, int y) { return grid[y][x] == 0; }
    
    void inflateObstacleCircular(std::vector<std::vector<int>>& grid, int x_center, int y_center, double drone_radius_m, double grid_resolution, int width, int height) {
        int radius_cells = static_cast<int>(std::ceil(drone_radius_m / grid_resolution));

        for (int i = -radius_cells; i <= radius_cells; ++i) {
            for (int j = -radius_cells; j <= radius_cells; ++j) {
                if (std::sqrt(i * i + j * j) <= radius_cells) {
                    if (isValid(x_center + j, y_center + i, width, height)) {
                        grid[y_center + i][x_center + j] = 1; // Mark as obstructed
                    }
                }
            }
        }
    }

    void printGrid(const std::vector<std::vector<int>>& grid, int start_x, int start_y, int end_x, int end_y, const std::vector<Waypoint>& path_waypoints, const Waypoint& grid_origin, double grid_resolution) {
        std::cout << "--- A* Grid Map ---" << std::endl;
        std::cout << "   "; 
        for (int x = 0; x < (int)grid[0].size(); ++x) {
            std::cout << (x % 10 == 0 ? std::to_string(x/10) : " "); 
        }
        std::cout << std::endl;
        std::cout << "   ";
        for (int x = 0; x < (int)grid[0].size(); ++x) {
            std::cout << (x % 10);
        }
        std::cout << std::endl;

        std::vector<std::vector<char>> display_grid(grid.size(), std::vector<char>(grid[0].size()));
        for (int y = 0; y < (int)grid.size(); ++y) {
            for (int x = 0; x < (int)grid[y].size(); ++x) {
                if (grid[y][x] == 1) display_grid[y][x] = '#';
                else display_grid[y][x] = '.';
            }
        }

        for (const auto& wp : path_waypoints) {
            int path_grid_x = static_cast<int>(std::floor((wp.x - grid_origin.x) / grid_resolution));
            int path_grid_y = static_cast<int>(std::floor((wp.y - grid_origin.y) / grid_resolution));

            if (isValid(path_grid_x, path_grid_y, grid[0].size(), grid.size()) && 
                display_grid[path_grid_y][path_grid_x] == '.') { 
                display_grid[path_grid_y][path_grid_x] = '*';
            }
        }

        if (isValid(start_x, start_y, grid[0].size(), grid.size())) display_grid[start_y][start_x] = 'S';
        if (isValid(end_x, end_y, grid[0].size(), grid.size())) display_grid[end_y][end_x] = 'E';


        for (int y = 0; y < (int)grid.size(); ++y) {
            std::cout << std::setw(2) << std::setfill(' ') << y << "|"; 
            for (int x = 0; x < (int)grid[y].size(); ++x) {
                std::cout << display_grid[y][x];
            }
            std::cout << '|' << std::endl;
        }
        std::cout << "-------------------" << std::endl;
    }

    double calculateHValue(int x, int y, int dest_x, int dest_y) { return sqrt(pow(x - dest_x, 2) + pow(y - dest_y, 2)); }

    std::vector<Waypoint> tracePath(const std::map<int, Node>& closed_list, int dest_x, int dest_y, int width, double target_z, const Waypoint& grid_origin, double grid_resolution) {
        std::vector<Waypoint> path;
        int current_x = dest_x, current_y = dest_y;
        while (true) {
            if (path.size() > 2 * (width * closed_list.size())) { 
                std::cerr << "TracePath Warning: Path tracing exceeding expected length. Breaking loop." << std::endl;
                break;
            }

            auto it = closed_list.find(current_y * width + current_x);
            if (it == closed_list.end()) {
                std::cerr << "TracePath Error: Current node (" << current_x << "," << current_y << ") not found in closed list." << std::endl;
                break; 
            }
            Node current_node = it->second;

            // CRITICAL CHANGE: Calculate waypoint at the CENTER of the grid cell
            path.push_back({grid_origin.x + current_x * grid_resolution + grid_resolution / 2.0, 
                            grid_origin.y + current_y * grid_resolution + grid_resolution / 2.0, 
                            target_z});
            
            if (current_node.parent_x == current_x && current_node.parent_y == current_y) break; 
            
            int temp_x = current_node.parent_x, temp_y = current_node.parent_y;
            current_x = temp_x; 
            current_y = temp_y;
        }
        std::reverse(path.begin(), path.end()); return path;
    }

    std::vector<Waypoint> aStarSearch(const std::vector<std::vector<int>>& grid, int start_x, int start_y, int dest_x, int dest_y, double target_z, const Waypoint& grid_origin, double grid_resolution) {
        int width = grid[0].size(); 
        int height = grid.size(); 
        
        if (!isValid(start_x, start_y, width, height)) {
            std::cerr << "A* Error: Start point (" << start_x << ", " << start_y << ") is out of grid bounds." << std::endl;
            return {};
        }
        if (!isValid(dest_x, dest_y, width, height)) {
            std::cerr << "A* Error: Destination point (" << dest_x << ", " << dest_y << ") is out of grid bounds." << std::endl;
            return {};
        }

        if (!isUnblocked(grid, start_x, start_y)) {
            std::cerr << "A* Warning: Start point (" << start_x << ", " << start_y << ") is in a mapped obstacle." << std::endl;
        }
        if (!isUnblocked(grid, dest_x, dest_y)) {
            std::cerr << "A* Warning: Destination (" << dest_x << ", " << dest_y << ") is in a mapped obstacle." << std::endl;
        }

        std::vector<Node> open_list;
        std::map<int, Node> closed_list; 

        Node start_node = {start_y, start_x, start_y, start_x, 0.0, calculateHValue(start_x, start_y, dest_x, dest_y), 0.0};
        start_node.f_cost = start_node.g_cost + start_node.h_cost;
        open_list.push_back(start_node);

        int dY[] = {-1, 1, 0, 0, -1, -1, 1, 1}; 
        int dX[] = {0, 0, 1, -1, 1, -1, 1, -1}; 

        long long iteration_count = 0; 

        while (!open_list.empty()) {
            iteration_count++;
            if (iteration_count > width * height * 10) { 
                 std::cerr << "A* Error: Exceeded max iterations (" << iteration_count << "). Likely stuck or no path." << std::endl;
                 return {};
            }

            auto current_it = std::min_element(open_list.begin(), open_list.end(), [](const Node& a, const Node& b) { return a.f_cost < b.f_cost; });
            Node current_node = *current_it;
            open_list.erase(current_it); 
            closed_list[current_node.y * width + current_node.x] = current_node; 

            if (current_node.y == dest_y && current_node.x == dest_x) {
                std::cout << "A* Success: Destination reached after " << iteration_count << " iterations." << std::endl;
                return tracePath(closed_list, dest_x, dest_y, width, target_z, grid_origin, grid_resolution);
            }

            for (int i = 0; i < 8; ++i) { 
                int newY = current_node.y + dY[i];
                int newX = current_node.x + dX[i];

                if (isValid(newX, newY, width, height) && isUnblocked(grid, newX, newY) && closed_list.find(newY * width + newX) == closed_list.end()) {
                    double cost_to_move = (i < 4) ? 1.0 : std::sqrt(2.0); 
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

        std::cerr << "A* Error: Failed to find a path after " << iteration_count << " iterations. Open list is empty." << std::endl;
        return {}; 
    }
}

void addKnownObstaclesToGrid(std::vector<std::vector<int>>& grid, const Waypoint& grid_origin, double grid_resolution, int GRID_WIDTH, int GRID_HEIGHT, double drone_safety_radius_meters) {
    std::vector<std::pair<double, double>> known_obstacle_coords = {
        {3.71, 2.5}, {7, -1.79}, {-2, 5.25}, {-4.52, -0.6}, {11.9, 3.41},
        {7.7, 8.01}, {13.5, 12.6}, {5.98, 16.4}, {0.85, 12.07} 
    };

    std::cout << "--- Manually Adding Known Obstacles to Grid (for Debugging) ---" << std::endl;
    for (const auto& obs_coord : known_obstacle_coords) {
        double obs_x_world = obs_coord.first;
        double obs_y_world = obs_coord.second;

        int grid_x = static_cast<int>((obs_x_world - grid_origin.x) / grid_resolution);
        int grid_y = static_cast<int>((obs_y_world - grid_origin.y) / grid_resolution);

        if (Pathfinder::isValid(grid_x, grid_y, GRID_WIDTH, GRID_HEIGHT)) {
            std::cout << "World Obstacle (" << std::fixed << std::setprecision(2) << obs_x_world << ", " << obs_y_world << ") -> Grid (" << grid_x << ", " << grid_y << ")" << std::endl;
            Pathfinder::inflateObstacleCircular(grid, grid_x, grid_y, drone_safety_radius_meters, grid_resolution, GRID_WIDTH, GRID_HEIGHT);
        } else {
            std::cout << "Warning: Known obstacle (" << obs_x_world << ", " << obs_y_world << ") is outside grid bounds. Mapped to (" << grid_x << ", " << grid_y << ")." << std::endl;
        }
    }
    std::cout << "------------------------------------------------------------" << std::endl;
}


int main() {
    Supervisor *robot = new Supervisor();  // Changed from Robot to Supervisor
    int timeStep = (int)robot->getBasicTimeStep();

    // Get visualization nodes
    Node *waypoint_markers_group = robot->getFromDef("WAYPOINT_MARKERS");  // Changed to group
    Node *flight_path = robot->getFromDef("FLIGHT_PATH");
    Field *flight_path_coord = nullptr;
    Field *flight_path_coordIndex = nullptr;
    
    // Store waypoint marker nodes and fields
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

    InertialUnit *imu = robot->getInertialUnit("inertial unit"); imu->enable(timeStep);
    GPS *gps = robot->getGPS("gps"); gps->enable(timeStep);
    Gyro *gyro = robot->getGyro("gyro"); gyro->enable(timeStep);
    Lidar *lidar = robot->getLidar("lidar"); lidar->enable(timeStep); lidar->enablePointCloud();
    Motor *motors[4];
    const char *motor_names[] = {"front left propeller", "front right propeller", "rear left propeller", "rear right propeller"};
    for (int i = 0; i < 4; ++i) {
        motors[i] = robot->getMotor(motor_names[i]);
        motors[i]->setPosition(INFINITY);
        motors[i]->setVelocity(1.0);
    }

    Waypoint target_destination = {8.0, 10.0, 2.0}; 
    std::vector<Waypoint> a_star_path;
    std::vector<Waypoint> traveled_path;  // New: store the actual traveled path
    size_t current_path_index = 0;
    DroneState currentState = TAKEOFF;
    double takeoff_altitude = 2.0;
    
    Waypoint hover_position = {0.0, 0.0, 0.0};
    bool is_hover_pos_set = false;
    
    // Assuming a 50x50 meter world, 0.25m resolution needs 200x200 cells
    const int GRID_WIDTH = 200; 
    const int GRID_HEIGHT = 200; 
    const double GRID_RESOLUTION = 0.25; 
    const double DRONE_SAFETY_RADIUS_METERS = 2.0; // AGGRESSIVELY INCREASED SAFETY RADIUS TO 2.0m (try 2.5 if needed)
    Waypoint grid_origin = { -GRID_WIDTH * GRID_RESOLUTION / 2.0, -GRID_HEIGHT * GRID_RESOLUTION / 2.0, 0.0 };
    std::vector<std::vector<int>> grid(GRID_HEIGHT, std::vector<int>(GRID_WIDTH, 0));
    double initial_yaw = 0.0;
    bool map_is_built = false;
    double last_print_time = 0.0;

    double last_x = 0.0, last_y = 0.0, last_time_pd = 0.0;
    double vx = 0.0, vy = 0.0;

    print_log(robot->getTime(), "Drone mission starting. State: TAKEOFF");

    const double k_vertical_thrust = 68.5, k_vertical_offset = 0.6, k_vertical_p = 3.0;
    const double k_roll_p = 50.0, k_pitch_p = 30.0, k_yaw_p = 1.0;
    const double k_pos_p = 0.4;
    const double k_pos_d = 0.6; 
    const double k_max_pos_disturbance = 2.0, k_max_yaw_disturbance = 1.5;

    // Function to create waypoint markers dynamically
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

// Replace the updateProgressiveFlightPath function with this fixed version:
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
    // Function to hide waypoint marker when reached
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
        const double current_x = gps_values[0], current_y = gps_values[1], altitude = gps_values[2];
        const double roll_velocity = gyro->getValues()[0], pitch_velocity = gyro->getValues()[1];
        
        double dt = time - last_time_pd;
        if (dt > 0.0) {
            vx = (current_x - last_x) / dt;
            vy = (current_y - last_y) / dt;
        }
        last_x = current_x;
        last_y = current_y;
        last_time_pd = time;
        
        double roll_disturbance = 0.0, pitch_disturbance = 0.0, yaw_disturbance = 0.0, vertical_input = 0.0;

        switch (currentState) {
            case TAKEOFF: {
                is_hover_pos_set = false;
                vertical_input = k_vertical_p * std::pow(CLAMP(takeoff_altitude - altitude + k_vertical_offset, -1.0, 1.0), 3.0);
                if (altitude > takeoff_altitude - 0.1) {
                    currentState = MAPPING;
                    initial_yaw = yaw;
                    print_log(time, "Takeoff complete. State: MAPPING");
                }
                break;
            }

            case MAPPING: {
                is_hover_pos_set = false;
                vertical_input = k_vertical_p * std::pow(CLAMP(takeoff_altitude - altitude + k_vertical_offset, -1.0, 1.0), 3.0);
                yaw_disturbance = 0.5; 
                const float* ranges = lidar->getRangeImage();
                if(ranges){
                    int resolution = lidar->getHorizontalResolution();
                    for(int i=0; i < resolution; ++i){
                        if(ranges[i] > 0.2 && ranges[i] < lidar->getMaxRange() * 0.95){
                             double angle = yaw + (i - resolution / 2.0) * (lidar->getFov() / resolution);
                             double obs_x = current_x + ranges[i] * cos(angle);
                             double obs_y = current_y + ranges[i] * sin(angle);
                             int grid_x = static_cast<int>((obs_x - grid_origin.x) / GRID_RESOLUTION);
                             int grid_y = static_cast<int>((obs_y - grid_origin.y) / GRID_RESOLUTION);
                             // Pathfinder::inflateObstacleCircular(grid, grid_x, grid_y, DRONE_SAFETY_RADIUS_METERS, GRID_RESOLUTION, GRID_WIDTH, GRID_HEIGHT); // STILL COMMENTED OUT
                        }
                    }
                }
                double yaw_diff = fmod(yaw - initial_yaw + 4 * M_PI, 2 * M_PI);
                if (map_is_built && yaw_diff < 0.1) {
                    currentState = PLANNING;
                    print_log(time, "Mapping complete. State: PLANNING (LIDAR mapping temporarily disabled)"); 
                }
                if (yaw_diff > M_PI) map_is_built = true; 
                break;
            }

            case PLANNING: {
                int start_x = static_cast<int>((current_x - grid_origin.x) / GRID_RESOLUTION);
                int start_y = static_cast<int>((current_y - grid_origin.y) / GRID_RESOLUTION);
                int dest_x = static_cast<int>((target_destination.x - grid_origin.x) / GRID_RESOLUTION);
                int dest_y = static_cast<int>((target_destination.y - grid_origin.y) / GRID_RESOLUTION);

                print_log(time, "DEBUG: Current Drone World Pos: (" + std::to_string(current_x) + ", " + std::to_string(current_y) + ")");
                print_log(time, "DEBUG: Grid Origin World Pos: (" + std::to_string(grid_origin.x) + ", " + std::to_string(grid_origin.y) + ")");
                print_log(time, "DEBUG: Grid Resolution: " + std::to_string(GRID_RESOLUTION));
                print_log(time, "DEBUG: Drone Safety Radius (Meters): " + std::to_string(DRONE_SAFETY_RADIUS_METERS));
                print_log(time, "DEBUG: Mapped Drone Grid Pos: (" + std::to_string(start_x) + ", " + std::to_string(start_y) + ")");
                print_log(time, "DEBUG: Target Destination World Pos: (" + std::to_string(target_destination.x) + ", " + std::to_string(target_destination.y) + ")");
                print_log(time, "DEBUG: Mapped Target Grid Pos: (" + std::to_string(dest_x) + ", " + std::to_string(dest_y) + ")");

                if (Pathfinder::isValid(start_x, start_y, GRID_WIDTH, GRID_HEIGHT)) grid[start_y][start_x] = 0;
                if (Pathfinder::isValid(dest_x, dest_y, GRID_WIDTH, GRID_HEIGHT)) grid[dest_y][dest_x] = 0;
                
                addKnownObstaclesToGrid(grid, grid_origin, GRID_RESOLUTION, GRID_WIDTH, GRID_HEIGHT, DRONE_SAFETY_RADIUS_METERS);
                
                Pathfinder::printGrid(grid, start_x, start_y, dest_x, dest_y, {}, grid_origin, GRID_RESOLUTION); 
                
                print_log(time, "Calculating path with A*...");
                a_star_path = Pathfinder::aStarSearch(grid, start_x, start_y, dest_x, dest_y, takeoff_altitude, grid_origin, GRID_RESOLUTION);
                
                if (!a_star_path.empty()) {
                    print_log(time, "Path found! " + std::to_string(a_star_path.size()) + " points. State: ALIGNING"); 
                    currentState = ALIGNING; 
                    Pathfinder::printGrid(grid, start_x, start_y, dest_x, dest_y, a_star_path, grid_origin, GRID_RESOLUTION); 
                    
                    // NEW: Create waypoint markers for all planned waypoints
                    createWaypointMarkers(a_star_path);
                    
                    // Initialize traveled path with starting position
                    traveled_path.clear();
                    traveled_path.push_back({current_x, current_y, altitude});
                    // Add the first waypoint immediately to ensure we have 2 points for line drawing
                    traveled_path.push_back(a_star_path[0]);
                } else {
                    print_log(time, "Failed to find a path. Switching to hover mode.");
                    currentState = MISSION_COMPLETE;
                }
                is_hover_pos_set = false;
                break;
            }

            case ALIGNING: { 
                is_hover_pos_set = false;
                if (a_star_path.empty()) { 
                    currentState = MISSION_COMPLETE;
                    print_log(time, "ALIGNING: No path to align to. Switching to MISSION_COMPLETE.");
                    break;
                }

                static Waypoint initial_align_pos;
                static bool initial_align_pos_set_for_state = false;
                if (!initial_align_pos_set_for_state) {
                    initial_align_pos = {current_x, current_y, altitude};
                    initial_align_pos_set_for_state = true;
                    print_log(time, "ALIGNING: Initial position captured for alignment: (" + std::to_string(initial_align_pos.x) + ", " + std::to_string(initial_align_pos.y) + ")");
                }
                 if (currentState != ALIGNING) { 
                    initial_align_pos_set_for_state = false;
                 }

                Waypoint first_waypoint = a_star_path[0];
                double dx = first_waypoint.x - current_x; 
                double dy = first_waypoint.y - current_y; 
                double target_bearing = std::atan2(dy, dx);
                double heading_error = target_bearing - yaw;
                if (heading_error > M_PI) heading_error -= 2 * M_PI; else if (heading_error < -M_PI) heading_error += 2 * M_PI;

                yaw_disturbance = CLAMP(k_yaw_p * heading_error, -k_max_yaw_disturbance, k_max_yaw_disturbance);
                
                vertical_input = k_vertical_p * std::pow(CLAMP(takeoff_altitude - altitude + k_vertical_offset, -1.0, 1.0), 3.0);
                
                roll_disturbance = 0.0; 
                pitch_disturbance = 0.0;

                if (std::abs(heading_error) < 0.05) { 
                    currentState = NAVIGATING;
                    print_log(time, "Aligned with first waypoint. State: NAVIGATING");
                    last_print_time = time; 
                    initial_align_pos_set_for_state = false; 
                }
                if (time - last_print_time > 0.5) { 
                     std::cout << std::fixed << std::setprecision(2) << "[" << time << "] " 
                               << "ALIGNING: Target Yaw: " << target_bearing * 180/M_PI 
                               << " deg, Current Yaw: " << yaw * 180/M_PI 
                               << " deg, Error: " << heading_error * 180/M_PI << " deg" << std::endl; 
                     last_print_time = time;
                }

                break;
            }

            // Replace the NAVIGATING case section with distance-based path updates:
            case NAVIGATING: {
                static bool initial_align_pos_set_for_state = false; 
                if (initial_align_pos_set_for_state) initial_align_pos_set_for_state = false;
            
                is_hover_pos_set = false;
                if (current_path_index >= a_star_path.size()) {
                    print_log(time, "Final destination reached. State: MISSION_COMPLETE");
                    currentState = MISSION_COMPLETE;
                    break;
                }
            
                // Use the proven waypoint navigation logic
                Waypoint target = a_star_path[current_path_index];
                
                double dx = target.x - current_x;
                double dy = target.y - current_y;
                double distance = std::sqrt(dx * dx + dy * dy);
                double target_bearing = std::atan2(dy, dx);
                
                double heading_error = target_bearing - yaw;
                if (heading_error > M_PI) heading_error -= 2 * M_PI;
                if (heading_error < -M_PI) heading_error += 2 * M_PI;
            
                // Waypoint navigation control (same as your working code)
                yaw_disturbance = CLAMP(k_yaw_p * heading_error, -k_max_yaw_disturbance, k_max_yaw_disturbance);
                
                double forward_error = dx * std::cos(yaw) + dy * std::sin(yaw);
                double sideways_error = -dx * std::sin(yaw) + dy * std::cos(yaw);
                
                pitch_disturbance = CLAMP(-k_pos_p * forward_error, -k_max_pos_disturbance, k_max_pos_disturbance);
                roll_disturbance = CLAMP(-k_pos_p * sideways_error, -k_max_pos_disturbance, k_max_pos_disturbance);
                
                vertical_input = k_vertical_p * std::pow(CLAMP(target.z - altitude + k_vertical_offset, -1.0, 1.0), 3.0);
            
                // NEW: Distance-based path updates - only add when drone has moved significantly
                static double last_path_update_time = 0.0;
                static double last_recorded_x = current_x;
                static double last_recorded_y = current_y;
                
                if (time - last_path_update_time > 0.2) { // Check every 200ms
                    double moved_distance = std::sqrt(std::pow(current_x - last_recorded_x, 2) + 
                                                    std::pow(current_y - last_recorded_y, 2));
                    
                    // Only add point if drone has moved at least 0.3 meters
                    if (moved_distance > 0.3 || traveled_path.size() < 2) {
                        traveled_path.push_back({current_x, current_y, altitude});
                        
                        // Limit the path length to prevent too many coordinates
                        if (traveled_path.size() > 200) {
                            traveled_path.erase(traveled_path.begin());
                        }
                        
                        // Update visualization
                        updateProgressiveFlightPath(traveled_path);
                        
                        last_recorded_x = current_x;
                        last_recorded_y = current_y;
                    }
                    last_path_update_time = time;
                }
                
                // Check if current waypoint is reached (same threshold as working code)
                if (distance < 0.2) {
                    print_log(time, "Waypoint " + std::to_string(current_path_index + 1) + " reached!");
                    
                    // NEW: Hide the reached waypoint marker
                    hideWaypointMarker(current_path_index);
                    
                    current_path_index++;
                }
                
                // Progress logging (same format as working code)
                if (time - last_print_time >= 0.5) {
                    last_print_time = time;
                    std::cout << std::fixed << std::setprecision(2);
                    std::cout << "Time: " << time << "s | ";
                    std::cout << "Waypoint: " << current_path_index + 1 << "/" << a_star_path.size() << " | ";
                    std::cout << "Current Pos: (" << current_x << ", " << current_y << ", Alt: " << altitude << ") | ";
                    std::cout << "Target Pos: (" << target.x << ", " << target.y << ") | ";
                    std::cout << "Dist: " << distance << "m | ";
                    std::cout << "Heading Error: " << heading_error << " rad" << std::endl;
                }
                break;
            }
            
            case MISSION_COMPLETE: {
                static bool initial_align_pos_set_for_state = false; 
                if (initial_align_pos_set_for_state) initial_align_pos_set_for_state = false;

                if (!is_hover_pos_set) {
                    hover_position = {current_x, current_y, altitude};
                    is_hover_pos_set = true;
                    print_log(time, "Holding position at (" + std::to_string(hover_position.x) + ", " + std::to_string(hover_position.y) + ")");
                }

                double dx = hover_position.x - current_x;
                double dy = hover_position.y - current_y;
                double p_forward_term = dx * std::cos(yaw) + dy * std::sin(yaw);
                double p_sideways_term = -dx * std::sin(yaw) + dy * std::cos(yaw);

                double d_forward_term = vx * std::cos(yaw) + vy * std::sin(yaw);
                double d_sideways_term = -vx * std::sin(yaw) + vy * std::cos(yaw);

                pitch_disturbance = CLAMP(-k_pos_p * p_forward_term - k_pos_d * d_forward_term, -k_max_pos_disturbance, k_max_pos_disturbance);
                roll_disturbance = CLAMP(-k_pos_p * p_sideways_term - k_pos_d * d_sideways_term, -k_max_pos_disturbance, k_max_pos_disturbance);
                
                vertical_input = k_vertical_p * std::pow(CLAMP(hover_position.z - altitude + k_vertical_offset, -1.0, 1.0), 3.0);
                yaw_disturbance = 0;
                break;
            }
        }

        double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
        double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
        double m1 = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_disturbance;
        double m2 = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_disturbance;
        double m3 = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_disturbance;
        double m4 = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_disturbance;
        motors[0]->setVelocity(m1); motors[1]->setVelocity(-m2); motors[2]->setVelocity(-m3); motors[3]->setVelocity(m4);
    }

    delete robot;
    return 0;
}