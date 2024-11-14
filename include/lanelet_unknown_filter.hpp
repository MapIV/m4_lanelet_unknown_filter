#include <rclcpp/rclcpp.hpp>
#include <route_handler/route_handler.hpp>

#include <tier4_autoware_utils/ros/transform_listener.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/impl/Lanelet.h>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <tf2/utils.h>
#include <boost/geometry.hpp>

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <cmath>
#include <algorithm>


namespace lanelet_unknown_filter
{
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_mapping_msgs::msg::HADMapBin;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Pose;
using route_handler::RouteHandler;

using LaneletsData = std::vector<lanelet::Lanelet>;


class LaneletUnknownFilterNode : public rclcpp::Node
{
public:
  explicit LaneletUnknownFilterNode(const rclcpp::NodeOptions & node_options);

private:
  // ROS Publisher and Subscriber
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_;
  rclcpp::Subscription<DetectedObjects>::SharedPtr sub_objects_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
	tier4_autoware_utils::TransformListener transform_listener_{this};
	  // Lanelet Map Pointers
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
	// parameter
	double dist_th;
  int nearest_num;
  bool remove_all_unknown;

	void mapCallback(const HADMapBin::ConstSharedPtr msg);
  void objectsCallback(const DetectedObjects::ConstSharedPtr in_objects);

  // DetectedObject change_yaw(const DetectedObject & object);

	DetectedObject change_yaw_and_size(const lanelet::ConstLanelet & ll2, const DetectedObject & object, const geometry_msgs::msg::Point & obj_pos_geo);
	bool checkObjectCondition(
		const DetectedObject & object, const lanelet::BasicPoint2d & obj_pos, const lanelet::ConstLanelets & lanes_near_ego);
  lanelet::ConstLanelets getNextLanelets(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets laneIncludingPoint(double x, double y);
};
}