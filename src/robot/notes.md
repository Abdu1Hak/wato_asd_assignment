[Program Start]
       │
       ▼
 1. main()
    ├── rclcpp::init()
    │   └── Initializes the ROS 2 DDS middleware.
    │
    ├── std::make_shared<CostmapNode>()  ──► [Constructor Runs]
    │   ├── Creates CostmapCore object (allocates 200x200 grid).
    │   ├── lidar_sub: Subscribes to "/lidar" & registers laserCallback.
    │   └── costmap_pub: Creates publisher on topic "/costmap".
    │
    └── rclcpp::spin(node)
        └── Enters an idle wait loop (blocks here, waiting for incoming messages).
       │
═══════╪══════════════════════════════════════════════════════════════════════════
       │ (Hardware emits a LiDAR scan on "/lidar")
       ▼
 2. laserCallback(scan_msg)  ◄── [Triggered automatically by spin()]
    │
    ├── Step 1: costmap_.initializeCostmap()
    │   └── Wipes the 200x200 grid back to all 0s (free space).
    │
    ├── Step 2: Loop through scan_msg->ranges
    │   ├── Filters out invalid/noisy ranges (< min or > max).
    │   ├── laserToGridCord(range, angle) -> transforms Polar (r, θ) to Grid (x, y).
    │   └── markObstacles(x_grid, y_grid) -> sets matching cell to 100 (obstacle).
    │
    ├── Step 3: costmap_.inflateObstacles()
    │   └── Scans for cells with 100 and inflates a 1-meter linear cost buffer around them.
    │
    ├── Step 4: costmap_.getCostmapMsg()
    │   └── Fills in resolution, width, height, origin, flattens 2D grid into 1D, returns msg.
    │
    ├── Step 5: msg.header.stamp = this->now()
    │   └── Node stamps the message with current ROS 2 system time.
    │
    └── Step 6: costmap_pub->publish(msg)  ──► [Data sent over "/costmap" topic]
       │
═══════╪══════════════════════════════════════════════════════════════════════════
       │
       ▼
 3. Callback completes
    └── Node returns to rclcpp::spin() waiting for the next LiDAR scan.