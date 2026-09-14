#include "planner_core.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

double PlannerCore::heuristic(int x1, int y1 , int x2, int y2) const {
    // Calculate the Euclidian Distance
    return std::hypot(x2 - x1, y2 - y1);
}

bool PlannerCore::isOccupied(const nav_msgs::msg::OccupancyGrid& grid, int x, int y) const {
    int idx = y * grid.info.width + x;  // cell idx
    if (idx < 0 || idx >= static_cast<int>(grid.data.size())) return true; 
    return grid.data[idx] > 50; 
}

nav_msgs::msg::Path PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid& grid, 
    const geometry_msgs::msg::Pose& start, 
    const geometry_msgs::msg::Pose& goal)
{
   // Enter A* Algorithm 
   nav_msgs::msg::Path path; 
   path.header.frame_id = "map"; 
   
   // 1. World Cords to Grid Cells of the Robot Cell + Goal Cell 
   int sx = static_cast<int>((start.position.x - grid.info.origin.position.x) / grid.info.resolution); 
   int sy = static_cast<int>((start.position.y - grid.info.origin.position.y) / grid.info.resolution); 
   int gx = static_cast<int>((goal.position.x - grid.info.origin.position.x) / grid.info.resolution); 
   int gy = static_cast<int>((goal.position.y - grid.info.origin.position.y) / grid.info.resolution); 

    // 2. Open List order by f(n)
    // Create a Lamda expression for given two nodes, put the one with lower f() first. 
    auto cmp = [](Node* a, Node* b) {return a->f() > b->f(); }; 
    // Creates a priority queue of Node* using cmp to decide priority
    // open_list = nodes waiting to be processed, with lowest f() first 
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open_list(cmp); 
    // Create a hashmap 
    std::unordered_map<int, bool> closed_list; 

    Node* start_node = new Node{sx, sy, 0.0, heuristic(sx, sy, gx, gy), nullptr};
    open_list.push(start_node); // Create start node and push it into queue 

    Node* goal_node = nullptr; 
    // Four Possible Movement Directions
    const int dx[] = {1, -1, 0, 0}; 
    const int dy[] = {0, 0, 1, -1}; 

    while (!open_list.empty()){
        Node* current = open_list.top(); 
        open_list.pop(); 
    
        int key = current->y * grid.info.width + current->x; 
        if (closed_list[key]) continue; 
        closed_list[key] = true; 

        if (current->x == gx && current->y == gy){ // Youve reached the goal. Yippey!
            goal_node = current; 
            break; 
        }
        
        // In theory, from the current grid, move in all 4 directions, check if its occupied, if yes break. 
        for (int i=0; i < 4; i++){
            int nx = current->x + dx[i]; 
            int ny = current->y + dy[i]; 
            if (isOccupied(grid, nx, ny)) continue; 
            
            // Otherwise, push that node to the open_list, and run the algorithm again, in order to eventually inflate the region around the robot with g + h 
            Node* neighbor = new Node{nx, ny, current->g + 1.0, heuristic(nx, ny, gx, gy), current}; 
            open_list.push(neighbor); 
        }

    }
    // Topologically move through the Nodes to build the path, then reverse
    // This is in a nutshell the A* algorithm 
    for (Node* n = goal_node; n!=nullptr; n=n->parent){ 
        geometry_msgs::msg::PoseStamped pose; 
        pose.header.stamp = path.header.stamp; 
        pose.header.frame_id = "sim_world";
        pose.pose.position.x = n->x * grid.info.resolution + grid.info.origin.position.x; 
        pose.pose.position.y = n->y * grid.info.resolution + grid.info.origin.position.y; 
        path.poses.push_back(pose); 
    }
    std::reverse(path.poses.begin(), path.poses.end());
    
    return path; 

}

} 
