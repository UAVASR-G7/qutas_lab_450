#include <ros/ros.h>
#include <area_map/area_map.h>
#include <nav_msgs/OccupancyGrid.h>
#include <stdlib.h> // srand, rand
#include <time.h>   // time
#include <cmath>    // For sqrt and pow
#include <vector>   // For storing available quadrants

// Function to check if two obstacles are too close to each other
bool isTooClose(const obstacles_t& obs1, const obstacles_t& obs2) {
    // Calculate Euclidean distance between the centers of two obstacles
    double distance = sqrt(pow(obs1.x - obs2.x, 2) + pow(obs1.y - obs2.y, 2));
    double min_distance = obs1.size + obs2.size; // Minimum distance based on the sizes

    return (distance < min_distance);
}

// Function to generate random positions within a specific quadrant
void generateRandomPositionInQuadrant(obstacles_t& obs, int quadrant, int map_width, int map_height, int axes_div) {
    double p = 2 * (((rand() % 101) / 100.0) - 0.5);  // Random offset for axis
    int half_width = map_width / 2;
    int half_height = map_height / 2;

    switch (quadrant) {
        case 1: // Top-right (+x, +y)
            obs.x = (rand() % (half_width / axes_div)) + half_width / axes_div;
            obs.y = (rand() % (half_height / axes_div)) + half_height / axes_div;
            break;
        case 2: // Top-left (-x, +y)
            obs.x = -(rand() % (half_width / axes_div)) - half_width / axes_div;
            obs.y = (rand() % (half_height / axes_div)) + half_height / axes_div;
            break;
        case 3: // Bottom-left (-x, -y)
            obs.x = -(rand() % (half_width / axes_div)) - half_width / axes_div;
            obs.y = -(rand() % (half_height / axes_div)) - half_height / axes_div;
            break;
        case 4: // Bottom-right (+x, -y)
            obs.x = (rand() % (half_width / axes_div)) + half_width / axes_div;
            obs.y = -(rand() % (half_height / axes_div)) - half_height / axes_div;
            break;
    }

    // Shift position to the center of the map
    obs.x += map_width / 2;
    obs.y += map_height / 2;
}

AreaMap::AreaMap() :
    nh_("~"),
    topic_map_("grid"),
    param_frame_id_("map"),
    param_map_width_(10),  // Default width (can be overwritten by YAML)
    param_map_height_(10),  // Default height (can be overwritten by YAML)
    param_map_resolution_(0.1),
    param_map_boarder_(false),
    param_num_obs_(0) {

    // Load node parameters
    nh_.param("frame_id", param_frame_id_, param_frame_id_);
    nh_.param("map/width", param_map_width_, param_map_width_);
    nh_.param("map/height", param_map_height_, param_map_height_);
    nh_.param("map/resolution", param_map_resolution_, param_map_resolution_);
    nh_.param("map/boarder", param_map_boarder_, param_map_boarder_);

    int i = 0;
    std::string obs_type;
    while (nh_.getParam("obstacles/obs_" + std::to_string(i) + "/type", obs_type)) {
        obstacles_t obs;

        ROS_INFO("Loading obstacle %i (%s)", i, obs_type.c_str());

        if (obs_type == "square") {
            obs.type = OBS_SQUARE;
        } else if (obs_type == "circle") {
            obs.type = OBS_CIRCLE;
        } else {
            obs.type = OBS_NONE;
            ROS_ERROR("Unknown obstacle type: %s", obs_type.c_str());
        }

        if ((obs.type != OBS_NONE) &&
            nh_.getParam("obstacles/obs_" + std::to_string(i) + "/size", obs.size) &&
            nh_.getParam("obstacles/obs_" + std::to_string(i) + "/position/x", obs.x) &&
            nh_.getParam("obstacles/obs_" + std::to_string(i) + "/position/y", obs.y)) {

            obs_.push_back(obs);
        }

        i++;
    }

    if (i == 0) {
        srand(time(NULL));  // Random seed initialization

        // List of quadrants to ensure unique quadrant assignment
        std::vector<int> available_quadrants = {1, 2, 3, 4};

        for (int j = 0; j < 2; j++) {  // Loop twice to generate two obstacles
            obstacles_t obs;
            obs.type = OBS_SQUARE;  // You can adjust this for different obstacle types (circle, etc.)
            obs.size = 3;
            int axes_div = 6;

            nh_.param("obstacles/size", obs.size, obs.size);  // Using the same size from the YAML file
            nh_.param("obstacles/divisor", axes_div, axes_div);

            // Randomly select a quadrant for this obstacle
            int quadrant_idx = rand() % available_quadrants.size();
            int quadrant = available_quadrants[quadrant_idx];
            available_quadrants.erase(available_quadrants.begin() + quadrant_idx);  // Remove quadrant from list

            // Generate random position in the selected quadrant
            generateRandomPositionInQuadrant(obs, quadrant, param_map_width_, param_map_height_, axes_div);

            // Ensure this obstacle is not too close to any previously generated obstacles
            bool valid_position = true;
            for (const auto& prev_obs : obs_) {
                if (isTooClose(obs, prev_obs)) {
                    valid_position = false;  // Too close to another obstacle, re-generate
                    break;
                }
            }

            // Add the obstacle to the list once it has a valid position
            if (valid_position) {
                ROS_INFO("Random obstacle %d placed in quadrant %d (s:%i;d:%i)", j + 1, quadrant, obs.size, axes_div);
                obs_.push_back(obs);
            }
        }
    }

    // Setup publisher
    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid", 1, true);

    // Generate Map
    generate_map();

    ROS_INFO("Generated map with %i obstacles", (int)obs_.size());

    // Publish map data (latched)
    pub_map_.publish(msg_out_);
}

AreaMap::~AreaMap() {}

void AreaMap::generate_map(void) {
    ros::Time stamp = ros::Time::now();

    msg_out_.header.frame_id = param_frame_id_;
    msg_out_.header.stamp = stamp;

    msg_out_.info.map_load_time = stamp;
    msg_out_.info.resolution = param_map_resolution_;
    msg_out_.info.width = param_map_width_;
    msg_out_.info.height = param_map_height_;

    msg_out_.info.origin.position.x = -param_map_resolution_ * (param_map_width_ / 2);
    msg_out_.info.origin.position.y = -param_map_resolution_ * (param_map_height_ / 2);
    msg_out_.info.origin.position.z = 0.0;
    msg_out_.info.origin.orientation.w = 1.0;
    msg_out_.info.origin.orientation.x = 0.0;
    msg_out_.info.origin.orientation.y = 0.0;
    msg_out_.info.origin.orientation.z = 0.0;

    // Load in base map
    for (int i = 0; i < param_map_width_; i++) {
        for (int j = 0; j < param_map_height_; j++) {
            int8_t spot = 0;

            // Draw in the border
            if (param_map_boarder_ &&
                ((i == 0) || (j == 0) || (i == (param_map_width_ - 1)) || (j == (param_map_height_ - 1)))) {
                spot = 100;
            }

            msg_out_.data.push_back(spot);
        }
    }

    // Load in obstacles
    for (int k = 0; k < obs_.size(); k++) {
        switch (obs_[k].type) {
            case OBS_SQUARE: {
                for (int y = -obs_[k].size; y < obs_[k].size + 1; y++) {
                    for (int x = -obs_[k].size; x < obs_[k].size + 1; x++) {
                        int dx = obs_[k].x + x;
                        int dy = obs_[k].y + y;

                        if ((dx >= 0) &&
                            (dx < param_map_width_) &&
                            (dy >= 0) &&
                            (dy < param_map_height_)) {
                            msg_out_.data[dx + (dy * param_map_width_)] = 100;
                        }
                    }
                }
                break;
            }
            case OBS_CIRCLE: {
                for (int y = -obs_[k].size; y < obs_[k].size + 1; y++) {
                    for (int x = -obs_[k].size; x < obs_[k].size + 1; x++) {
                        int dx = obs_[k].x + x;
                        int dy = obs_[k].y + y;
                        int r = obs_[k].size;

                        if ((dx >= 0) &&
                            (dx < param_map_width_) &&
                            (dy >= 0) &&
                            (dy < param_map_height_) &&
                            ((x * x + y * y) <= (r * r + r * 0.25))) {
                            msg_out_.data[dx + (dy * param_map_width_)] = 100;
                        }
                    }
                }
                break;
            }
            default: {
                ROS_ERROR("Cannot load obstacle: %i", k);
            }
        }
    }
}
