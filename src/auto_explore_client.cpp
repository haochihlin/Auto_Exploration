#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/footprint.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <frontier_exploration/geometry_tools.h>
#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/ExploreTaskActionGoal.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>

#include <boost/foreach.hpp>


namespace frontier_exploration{

/**
 * @brief Client for FrontierExplorationServer that receives map from topic, and autonomous creates boundary polygon for frontier exploration
 */
class FrontierExplorationClientAuto{

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber map_sub_;


    uint map_width_;
    uint map_height_;

    /**
     * @brief Build boundary polygon from map received through map topic.
     * @param map Received map from any type of mapping algorithm
     */
    void mapCB(const nav_msgs::OccupancyGridConstPtr& map)
    {
        if(map_width_ == map->info.width && map_height_ == map->info.height)
            return;
        
        // Get basic map info
        float resolution = map->info.resolution;
        map_width_ = map->info.width;
        map_height_ = map->info.height;
        if(map_width_ <= 20 || map_height_ <= 20)
        {
            ROS_WARN("The size of map is too small !! SKIP");
            return;
        }

        // Find the corner of map (boundary zone is 10 cell)
        float bound_zone = 10*resolution;
        float max_x = map_width_*resolution + map->info.origin.position.x - bound_zone;
        float max_y = map_height_*resolution + map->info.origin.position.y - bound_zone;
        float min_x = map->info.origin.position.x + bound_zone;
        float min_y = map->info.origin.position.y + bound_zone;

        // Push the points into the polygon vector
        geometry_msgs::PolygonStamped polygon_area; 
        polygon_area.polygon.points.clear();
        polygon_area.header = map->header;
        geometry_msgs::Point point; 
        point.z = 0;
        point.x = min_x;
        point.y = min_y;
        polygon_area.polygon.points.push_back(costmap_2d::toPoint32(point));
        point.x = max_x;
        polygon_area.polygon.points.push_back(costmap_2d::toPoint32(point));
        point.y = max_y;
        polygon_area.polygon.points.push_back(costmap_2d::toPoint32(point));
        point.x = min_x;
        polygon_area.polygon.points.push_back(costmap_2d::toPoint32(point));
        
        actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> exploreClient("explore_server", true);
        exploreClient.waitForServer();
        ROS_INFO("Sending goal");
        frontier_exploration::ExploreTaskGoal goal;
        goal.explore_center.header = map->header;
        point.x = 0.0;
        point.y = 0.0;
        goal.explore_center.point = point;
        goal.explore_boundary = polygon_area;
        exploreClient.sendGoal(goal);

    } //end of mapCB

public:

    /**
     * @brief Constructor for the client.
     */
    FrontierExplorationClientAuto() :
        nh_(),
        private_nh_("~"),
        map_width_(0),
        map_height_(0)
    {
        map_sub_ = nh_.subscribe("/map", 1, &FrontierExplorationClientAuto::mapCB, this);
        ROS_INFO("Start autonomous map exploration");
    }    

}; // end of class

}// end of namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_client_auto");

    frontier_exploration::FrontierExplorationClientAuto client_auto;
    ros::spin();
    return 0;
}
