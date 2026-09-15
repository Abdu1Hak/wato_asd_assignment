#include "planner_core.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {
}

double PlannerCore::heuristic(int x1, int y1 , int x2, int y2) const {
    // Calculate the Euclidian Distance
    return std::hypot(x2 - x1, y2 - y1);
}

// Because the planner doesnt know the costmap of certain areas, unknown areas of -1 must have cost of 0 
// As the map updates, the path prefers curving around inflated obstacles
bool PlannerCore::isTraversable(const nav_msgs::msg::OccupancyGrid& grid, int x, int y) const {

    int width  = static_cast<int>(grid.info.width);
    int height = static_cast<int>(grid.info.height);
    if (x < 0 || x >= width || y < 0 || y >= height) return false;

    int idx = y * width + x;
    if (idx < 0 || idx >= static_cast<int>(grid.data.size())) return false; 

    int8_t raw = grid.data[idx];

    // Quick debug 
    if (raw != 0 && raw != -1){
        RCLCPP_INFO(logger_, "Cell (%d, %d) has raw value: %d", x, y, raw);
    }


    if (raw == -1) return true;
    // Must be mapped (val >= 0) and below the occupied threshold
    // Slight change: Unknown space is treated as traversable
    uint8_t cost = static_cast<uint8_t>(raw); 
    return cost < occupancy_threshold_;
}

int PlannerCore::cellCost(const nav_msgs::msg::OccupancyGrid& grid, int x, int y) const {
    int width  = static_cast<int>(grid.info.width);
    int height = static_cast<int>(grid.info.height);
    if (x < 0 || x >= width || y < 0 || y >= height) return 0;
    int idx = y * width + x; 
    if (idx < 0 || idx >= static_cast<int>(grid.data.size())) return 0; 

    int8_t val = grid.data[idx]; 
    // Unknown space has 0 cost - assumed free until sensed
    if (val == -1) return 10; // slight penalty cost to i guess prefer the longer route avoiding pink 
    return static_cast<uint8_t>(val); 
}

bool PlannerCore::canMoveBetween(const nav_msgs::msg::OccupancyGrid& grid, int fx, int fy, int tx, int ty) const {
    int dx = tx - fx; 
    int dy = ty - fy; 
    if (dx == 0 || dy == 0) return true; // Straight movement
    // Do not cut corners through occupied/unknown cells
    // checks if both adjacent cells are free
    return isTraversable(grid, fx + dx, fy) && isTraversable(grid, fx, fy + dy); 
}


nav_msgs::msg::Path PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid& grid, 
    const geometry_msgs::msg::Pose& start, 
    const geometry_msgs::msg::Pose& goal)
{
   // Enter A* Algorithm 
    // MORE DEBUG -------AAAA
    nav_msgs::msg::Path path;
    path.header.frame_id = grid.header.frame_id.empty() ? "sim_world" : grid.header.frame_id;
    
    int zero_count = 0;
    int minus_one_count = 0;
    int other_negative = 0;
    int positive_count = 0;

    for (int8_t v : grid.data) {
        if (v == 0) zero_count++;
        else if (v == -1) minus_one_count++;
        else if (v < -1) other_negative++;
        else positive_count++;
    }

    RCLCPP_INFO(logger_, "Zeros (free): %d | -1 (unknown): %d | Negative costs: %d | Positive costs: %d",
    zero_count, minus_one_count, other_negative, positive_count);
   
   // 1. World Cords to Grid Cells of the Robot Cell + Goal Cell
    int sx = static_cast<int>(std::floor((start.position.x - grid.info.origin.position.x) / grid.info.resolution)); 
    int sy = static_cast<int>(std::floor((start.position.y - grid.info.origin.position.y) / grid.info.resolution)); 
    int gx = static_cast<int>(std::floor((goal.position.x - grid.info.origin.position.x) / grid.info.resolution)); 
    int gy = static_cast<int>(std::floor((goal.position.y - grid.info.origin.position.y) / grid.info.resolution)); 


    int width  = static_cast<int>(grid.info.width);
    int height = static_cast<int>(grid.info.height);

    // Start and Goal sanity check -- debugging
    if (sx < 0 || sy < 0 || sx >= width || sy >= height ||
        gx < 0 || gy < 0 || gx >= width || gy >= height) {
        RCLCPP_WARN(logger_, "Start or goal outside grid; returning empty path.");
        return path;}
    if (!isTraversable(grid, sx, sy) || !isTraversable(grid, gx, gy)) {
        RCLCPP_WARN(logger_, "Goal cell is occupied; returning empty path.");
        return path;
    }

    // 2. Open List order by f(n)
    // Create a Lamda expression for given two nodes, put the one with lower f() first. 
    auto cmp = [](Node* a, Node* b) {return a->f() > b->f(); }; 
    // Creates a priority queue of Node* using cmp to decide priority
    // open_list = nodes waiting to be processed, with lowest f() first 
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open_list(cmp); 
    // Create a hashmap 
    std::unordered_set<int> closed_list; 
    std::unordered_map<int, double> best_g; 
    std::vector<Node*> allocated_nodes; 

    auto createNode = [&](int x, int y, double g, double h, Node* parent) -> Node* {
        Node* node = new Node{x,y,g,h,parent}; 
        allocated_nodes.push_back(node); 
        return node; 
    };

    int start_key = sy * width + sx;
    best_g[start_key] = 0.0;
    open_list.push(createNode(sx, sy, 0.0, heuristic(sx, sy, gx, gy), nullptr));

    Node* goal_node = nullptr; 
    const int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!open_list.empty()){
        Node* current = open_list.top(); 
        open_list.pop(); 
    
        int key = current->y * grid.info.width + current->x; 
        if (closed_list.count(key) > 0) continue; 
        closed_list.insert(key); 

        if (current->x == gx && current->y == gy){ // Youve reached the goal. Yippey!
            goal_node = current; 
            break; 
        }
        
        // In theory, from the current grid, move in all 8 directions, check if its occupied, if yes break. 
        for (int i=0; i < 8; i++){
            int nx = current->x + dx[i]; 
            int ny = current->y + dy[i]; 
            int nkey = ny * width + nx;
            
            if (closed_list.count(nkey) > 0) continue; 
            if (!isTraversable(grid, nx, ny)) continue; 
            if (!canMoveBetween(grid, current->x, current->y, nx, ny)) continue;

            double step_cost = std::hypot(dx[i], dy[i]); 
            int cell_cost_val = cellCost(grid, nx, ny); 
            double tentative_g = current->g + step_cost + cost_weight_ * cell_cost_val; 
            
            // Only add to this neighbor if the route is cheaper 
            auto it = best_g.find(nkey); 
            if (it == best_g.end() || tentative_g < it->second){
                best_g[nkey] = tentative_g; 
                open_list.push(createNode(nx, ny, tentative_g, heuristic(nx, ny, gx, gy), current));
            }

        }

    }
    // Topologically move through the Nodes to build the path, then reverse
    // This is in a nutshell the A* algorithm 
    if (goal_node != nullptr){
        for (Node* n = goal_node; n!=nullptr; n=n->parent){ 
            geometry_msgs::msg::PoseStamped pose; 
            pose.header.frame_id = path.header.frame_id;
            pose.pose.position.x = grid.info.origin.position.x + (static_cast<double>(n->x) + 0.5) * grid.info.resolution; 
            pose.pose.position.y = grid.info.origin.position.y + (static_cast<double>(n->y) + 0.5) * grid.info.resolution; 
            pose.pose.orientation.w = 1.0; 
            path.poses.push_back(pose);
        }
        std::reverse(path.poses.begin(), path.poses.end());
    }   
    
    for (Node* ptr : allocated_nodes){
        delete ptr; 
    }
    
    return path; 

}

} 
