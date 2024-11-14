#include "lanelet_unknown_filter.hpp"

namespace lanelet_unknown_filter
{
LaneletUnknownFilterNode::LaneletUnknownFilterNode(const rclcpp::NodeOptions & node_options)
: Node("lanelet_unknown_filter", node_options)
{
	dist_th = static_cast<double>(this->declare_parameter("distance_threshold",1.0));
  remove_allunknown = static_cast<double>(this->declare_parameter("remove_allunknown",false));
	sub_objects_ = this->create_subscription<DetectedObjects>(
    "/input/objects", 1,
    std::bind(&LaneletUnknownFilterNode::objectsCallback, this, std::placeholders::_1));
  sub_map_ = this->create_subscription<HADMapBin>(
    "/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&LaneletUnknownFilterNode::mapCallback, this, std::placeholders::_1));
  pub_objects_ = this->create_publisher<DetectedObjects>("/output/objects", rclcpp::QoS{1});
}

void LaneletUnknownFilterNode::mapCallback(const HADMapBin::ConstSharedPtr msg){
  RCLCPP_INFO(get_logger(), "[LaneletUnknownFilter]: Start aloading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  RCLCPP_INFO(get_logger(), "[LaneletUnknownFilter]: Map is loaded");
}

void LaneletUnknownFilterNode::objectsCallback(const DetectedObjects::ConstSharedPtr in_objects){
	if (!lanelet_map_ptr_) {
    return;
  }
  RCLCPP_INFO(get_logger(), "[LaneletUnknownFilter]: 2");
  auto map2ego = transform_listener_.getTransform(
    "map",        // src
    "base_link",  // target
    in_objects->header.stamp, rclcpp::Duration::from_seconds(0.1));
  // 初期位置を設定しないとmapとbaselinkが別のtftreeとなる。その状態でmap2egoなどにアクセスするとセグフォが生じる
  if (!map2ego) return;
  RCLCPP_INFO(get_logger(), "[LaneletUnknownFilter]: 21");

	double ego_x = map2ego->transform.translation.x;
	double ego_y = map2ego->transform.translation.y;
  double ego_z = map2ego->transform.translation.z;
  Pose ego_pose_from_map;
  ego_pose_from_map.position.x = ego_x;
  ego_pose_from_map.position.y = ego_y;
  ego_pose_from_map.position.z = ego_z;

  auto quat_geo = map2ego->transform.rotation;
  tf2::Quaternion quat(quat_geo.x, quat_geo.y, quat_geo.z, quat_geo.w);
  double r,p,yaw;//出力値[rad]
  tf2::Matrix3x3(quat).getRPY(r, p, yaw);//クォータニオン→オイラー角

  lanelet::BasicPoint2d ego_point(ego_x, ego_y);
  lanelet::ConstLanelets lanes_near_ego;
  std::vector<std::pair<double, lanelet::Lanelet>> ego_lanelets =
  lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, ego_point, 10);
  if((int)ego_lanelets.size() == 0) return;
  for (const auto & lanelet_ : ego_lanelets) {
      if(lanelet_.first > 20) continue;
      lanelet::ConstLanelet lanelet = lanelet_.second;
      auto lanes = routing_graph_ptr_->following(lanelet);
      for (const auto & lane : lanes) {
        lanes_near_ego.push_back(lane);
        auto lanes2 = routing_graph_ptr_->following(lane);
        for (const auto & lane2 : lanes2) {
          lanes_near_ego.push_back(lane2);
        }
      }
    }
  DetectedObjects output;
  output.header = in_objects->header;
  RouteHandler rh = RouteHandler();
  for (const auto & object : in_objects->objects) {
    double obj_x_from_ego = object.kinematics.pose_with_covariance.pose.position.x;
    double obj_y_from_ego = object.kinematics.pose_with_covariance.pose.position.y;
	  double obj_x = ego_x + std::cos(yaw)*obj_x_from_ego - std::sin(yaw)*obj_y_from_ego;
	  double obj_y = ego_y + std::sin(yaw)*obj_x_from_ego - std::cos(yaw)*obj_y_from_ego;
    geometry_msgs::msg::Point obj_pos_geo;
    obj_pos_geo.x = obj_x;
    obj_pos_geo.y = obj_y;
    obj_pos_geo.z = map2ego->transform.translation.z;
	  lanelet::BasicPoint2d search_point(obj_x, obj_y);

    
    //lanelet::ConstLanelets ego_closest_lanelet;//, next_lanelet;
    

    std::vector<std::pair<double, lanelet::Lanelet>> near_obj_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 1);
    lanelet::ConstLanelet obj_closest_lanelet = near_obj_lanelets[0].second;
    //if(!(rh.getClosestLaneletWithinRoute(ego_pose_from_map, &ego_closest_lanelet))) {}

    bool should_be_changed = checkObjectCondition(object, search_point, lanes_near_ego);
    // RCLCPP_INFO(get_logger(), "[LaneletUnknownFilter]: ego_closest_lanelet=%d",(int)ego_closest_lanelet.size());
    RCLCPP_INFO(get_logger(), "[LaneletUnknownFilter]: lanes_near_ego=%d",(int)lanes_near_ego.size());

    // auto pub_obj = change_yaw(lanes_near_ego[nearest_num], object, obj_pos_geo);
    // auto pub_obj = change_yaw(object);
    // pub_obj.classification[0].label = 1;
		// output.objects.push_back(pub_obj);
    if(should_be_changed){
      RCLCPP_INFO(get_logger(), "[LaneletUnknownFilter]: 1");
			auto pub_obj = change_yaw_and_size(obj_closest_lanelet, object, obj_pos_geo);
      RCLCPP_INFO(get_logger(), "[LaneletUnknownFilter]: 11");
      pub_obj.classification[0].label = 1;
			output.objects.push_back(pub_obj);
		}
    else if(remove_allunknown){
      if((int)object.classification.front().label != 0) output.objects.push_back(object);
    }
    else output.objects.push_back(object);

  }
  if(output.objects.size()) pub_objects_->publish(output);
}

DetectedObject LaneletUnknownFilterNode::change_yaw_and_size(
	const lanelet::ConstLanelet & ll2, const DetectedObject & object, const geometry_msgs::msg::Point & obj_pos_geo){
	DetectedObject ret_obj = object;
	auto lane_yaw = lanelet::utils::getLaneletAngle(ll2, obj_pos_geo);
	double y = lane_yaw;//+M_PI/2;   
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, y); 
	ret_obj.kinematics.pose_with_covariance.pose.orientation.x = quaternion.x();
	ret_obj.kinematics.pose_with_covariance.pose.orientation.y = quaternion.y();
	ret_obj.kinematics.pose_with_covariance.pose.orientation.z = quaternion.z();
	ret_obj.kinematics.pose_with_covariance.pose.orientation.w = quaternion.w();
	ret_obj.kinematics.twist_with_covariance.twist.angular.x = 0;
	ret_obj.kinematics.twist_with_covariance.twist.angular.y = 0;
	ret_obj.kinematics.twist_with_covariance.twist.angular.z = 0;
  ret_obj.shape.dimensions.x = 3;
  ret_obj.shape.dimensions.y = 1;
	return(ret_obj);
}

bool LaneletUnknownFilterNode::checkObjectCondition(
		const DetectedObject & object, const lanelet::BasicPoint2d & search_point, const lanelet::ConstLanelets & lanes_near_ego){
	int MAX_DIST = 10000000;
	int condition_cnt = 0;
  if(lanes_near_ego.empty()) return false;
	if ((int)object.classification.front().label == 0) condition_cnt += 1;
	// if (std::abs(object.kinematics.twist_with_covariance.twist.linear.x) <= 0.1) condition_cnt += 10;
	// if (std::abs(object.kinematics.twist_with_covariance.twist.linear.y) <= 0.1) condition_cnt += 100;

	double dist_to_nearest_lanelet = MAX_DIST;
  // int itr = 0;
  for (const auto & lanelet : lanes_near_ego) {
    //double dist_to_llt = boost::geometry::distance(lanelet.polygon2d().basicPolygon(), search_point);
    auto dist_to_llt = lanelet::geometry::distanceToCenterline2d(lanelet, search_point);
    dist_to_nearest_lanelet = std::min(dist_to_nearest_lanelet,dist_to_llt);
    // if (dist_to_nearest_lanelet > dist_to_llt){
    //   dist_to_nearest_lanelet = dist_to_llt;
    //   // nearest_num = itr;
    // }
    // itr++;
  }
  RCLCPP_INFO(get_logger(), "[LaneletUnknownFilter]: dist_to_nearest_lanelet=%lf",dist_to_nearest_lanelet);
  RCLCPP_INFO(get_logger(), "[LaneletUnknownFilter]: dist_th=%lf",dist_th);
	if (dist_to_nearest_lanelet <= dist_th) condition_cnt += 1000;
  RCLCPP_INFO(get_logger(), "[LaneletUnknownFilter]: cnt=%d", condition_cnt);
	if (condition_cnt == 1001) {
    return true;
  }
	else return false;
}

lanelet::ConstLanelets LaneletUnknownFilterNode::getNextLanelets(const lanelet::ConstLanelet & lanelet) const
{
  return routing_graph_ptr_->following(lanelet);
}

lanelet::ConstLanelets LaneletUnknownFilterNode::laneIncludingPoint(double x, double y)
{
    // obstacle point
  lanelet::BasicPoint2d search_point(x, y);

  // nearest lanelet
  std::vector<std::pair<double, lanelet::Lanelet>> surrounding_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 10);

  {  // Step 1. Search same directional lanelets
    // No Closest Lanelets
    if (surrounding_lanelets.empty()) {
      return {};
    }
    lanelet::ConstLanelets including_lanelet;
    // std::optional<std::pair<double, lanelet::Lanelet>> closest_lanelet{std::nullopt};
    for (const auto & lanelet : surrounding_lanelets) {
      // Check if the close lanelets meet the necessary condition for start lanelets and
      if (lanelet.second.centerline().size() <= 1) {
        continue;
      } 
      // Check if similar lanelet is inside the object lanelet
      // if (isDuplicated(lanelet, including_lanelet)) {
      //   continue;
      // }
      // Check if the obstacle is inside of this lanelet
      constexpr double epsilon = 1e-3;
      if (lanelet.first < epsilon) {
        const lanelet::ConstLanelet ego_lanelet = lanelet.second;
        including_lanelet.push_back(ego_lanelet);
      }
    }
    if (!including_lanelet.empty()) {
      return including_lanelet;
    }
  }
  return lanelet::ConstLanelets{};
}

// bool LaneletUnknownFilterNode::isDuplicated(
//   const std::pair<double, lanelet::ConstLanelet> & target_lanelet, const lanelet::ConstLanelets & lanelets_data){
//   const double CLOSE_LANELET_THRESHOLD = 0.1;
//   for (const auto & lanelet_data : lanelets_data) {
//     const auto target_lanelet_end_p = target_lanelet.second.centerline2d().back();
//     const auto lanelet_end_p = lanelet_data.centerline2d().back();
//     const double dist = std::hypot(
//       target_lanelet_end_p.x() - lanelet_end_p.x(), target_lanelet_end_p.y() - lanelet_end_p.y());
//     if (dist < CLOSE_LANELET_THRESHOLD) {
//       return true;
//     }
//   }
//   return false;
// }
}
int main(int argc, char **argv){
    rclcpp::init(argc,argv); ////ROS2通信を初期化
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions node_options;

    auto node1 = std::make_shared<lanelet_unknown_filter::LaneletUnknownFilterNode>(node_options);
    exec.add_node(node1);

    
    exec.spin();
    return 0;
}