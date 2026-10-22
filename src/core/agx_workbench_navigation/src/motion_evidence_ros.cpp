#include <agx_workbench_navigation/motion_evidence_ros.hpp>
#include <agx_workbench_navigation/motion_policy_io.hpp>
#include <agx_workbench_navigation/motion_sensor_evidence.hpp>
#include <agx_workbench_navigation/LidarObstacleSnapshot.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <fstream>
#include <mutex>
#include <sstream>
#include <stdexcept>
namespace agx_workbench_navigation {
namespace {
void Require(bool condition,const char* reason) {if(!condition)throw std::invalid_argument(reason);}
bool Finite(double value) {return std::isfinite(value);}
std::string Read(const std::string& path,std::size_t bound) {
  std::ifstream stream(path,std::ios::binary|std::ios::ate);
  Require(static_cast<bool>(stream),"installed evidence source cannot be read");
  const auto size=stream.tellg();Require(size>0 && static_cast<std::size_t>(size)<=bound,"installed evidence source exceeds bound");
  std::string bytes(static_cast<std::size_t>(size),'\0');stream.seekg(0);stream.read(&bytes[0],bytes.size());
  Require(static_cast<bool>(stream),"installed evidence source read failed");return bytes;
}
std::string Token(std::istream& stream) {
  while(stream>>std::ws && stream.peek()=='#') {std::string comment;std::getline(stream,comment);}
  std::string token;stream>>token;return token;
}
std::size_t Dimension(const std::string& value) {
  Require(!value.empty() && value.size()<=8 && value.find_first_not_of("0123456789")==std::string::npos,"invalid PGM dimension");
  const auto result=std::stoul(value);Require(result>0 && result<=1000000,"PGM dimension exceeds bound");return result;
}
MotionCaptureTransform Transform(const geometry_msgs::Pose& pose) {
  MotionCaptureTransform result;
  const auto& p=pose.position;const auto& q=pose.orientation;
  result.position={{p.x,p.y,p.z}};result.orientation={{q.x,q.y,q.z,q.w}};
  for(const auto value:result.position)Require(Finite(value),"nonfinite pose");
  double norm=0;for(const auto value:result.orientation) {Require(Finite(value),"nonfinite quaternion");norm+=value*value;}
  Require(Finite(norm) && std::abs(norm-1)<.001,"invalid quaternion");
  const double scale=std::sqrt(norm);for(auto& value:result.orientation)value/=scale;
  return result;
}
MotionCaptureTransform Transform(const geometry_msgs::Transform& transform) {
  geometry_msgs::Pose pose;pose.position.x=transform.translation.x;
  pose.position.y=transform.translation.y;pose.position.z=transform.translation.z;pose.orientation=transform.rotation;
  return Transform(pose);
}
MotionPose Planar(const MotionCaptureTransform& pose) {
  const auto& q=pose.orientation;return {pose.position[0],pose.position[1],std::atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]))};
}
void Covariance(const boost::array<double,36>& cov) {
  for(const auto value:cov)Require(Finite(value),"nonfinite covariance");
  for(int i=0;i<6;++i)Require(cov[i*7]>=0,"negative covariance");
}
bool Nearby(const MotionCaptureTransform& first,const MotionCaptureTransform& second,double distance,double angle) {
  const auto a=Planar(first),b=Planar(second);
  return std::hypot(a.x_m-b.x_m,a.y_m-b.y_m)<=distance && std::abs(MotionAngleDifference(a.yaw_rad,b.yaw_rad))<=angle;
}
bool SameGridGeometry(const MotionGrid& a,const MotionGrid& b) {
  return a.width==b.width && a.height==b.height && std::abs(a.resolution_m-b.resolution_m)<=1e-6 &&
      std::abs(a.origin_x_m-b.origin_x_m)<=1e-9 && std::abs(a.origin_y_m-b.origin_y_m)<=1e-9;
}
// The monitor only emits dense FLOAT32 XYZ. Validate before iterator/allocation.
CapturedMotionCloud Decode(const LidarObstacleSnapshot& msg) {
  CapturedMotionCloud cloud;cloud.sequence=msg.cloud_sequence;cloud.source_stamp_s=msg.header.stamp.toSec();
  cloud.source_stamp_ns=msg.header.stamp.toNSec();
  cloud.transform_stamp_s=msg.transform_stamp.toSec();cloud.transform_static=msg.transform_static;
  cloud.receipt_steady_s=msg.receipt_steady_s;cloud.state=msg.state;
  cloud.source_frame=msg.source_frame;cloud.target_frame=msg.header.frame_id;cloud.pose_frame=msg.capture_pose.header.frame_id;
  cloud.model_digest=msg.policy_digest;cloud.filter_digest=msg.filter_digest;cloud.capture_pose=Transform(msg.capture_pose.pose);
  const auto& points=msg.obstacle_points;
  Require(msg.capture_pose.header.stamp==msg.header.stamp && points.header.stamp==msg.header.stamp &&
      points.header.frame_id=="base_link" && points.height==1 && points.width<=250000 &&
      !points.is_bigendian && points.point_step==12 && points.row_step==points.width*12 &&
      points.data.size()==points.row_step && points.fields.size()==3,"malformed obstacle transport");
  for(std::size_t i=0;i<3;++i) {
    const auto& field=points.fields[i];const std::string name=i==0?"x":(i==1?"y":"z");
    Require(field.name==name && field.offset==i*4 && field.datatype==sensor_msgs::PointField::FLOAT32 && field.count==1,
        "unsupported obstacle point layout");
  }
  if(points.width==0)return cloud;
  sensor_msgs::PointCloud2ConstIterator<float> x(points,"x"),y(points,"y"),z(points,"z");
  cloud.points.reserve(points.width);
  for(std::size_t i=0;i<points.width;++i,++x,++y,++z) {
    Require(Finite(*x) && Finite(*y) && Finite(*z),"nonfinite filtered obstacle");cloud.points.push_back({{*x,*y,*z}});
  }
  return cloud;
}
bool SameCapture(const LidarObstacleSnapshot& a,const LidarObstacleSnapshot& b) {
  // Health may expire and the ROS transport sequence may advance. Neither
  // permits changes to original receipt, geometry or source identity.
  return a.cloud_sequence==b.cloud_sequence && a.source_frame==b.source_frame &&
      a.header.frame_id==b.header.frame_id && a.receipt_steady_s==b.receipt_steady_s &&
      a.transform_stamp==b.transform_stamp && a.transform_static==b.transform_static &&
      a.policy_digest==b.policy_digest && a.filter_digest==b.filter_digest &&
      a.capture_pose==b.capture_pose && a.obstacle_points==b.obstacle_points;
}
}
class RosMotionEvidence::Impl {
 public:
  Impl(ros::NodeHandle node,std::string policy_path,std::string map_path,std::string registry_path,
      std::string lidar_path,GridSource costmap,std::function<double()> clock)
      :node_(std::move(node)),policy_path_(std::move(policy_path)),map_path_(std::move(map_path)),
       registry_path_(std::move(registry_path)),lidar_path_(std::move(lidar_path)),costmap_(std::move(costmap)),
       clock_(std::move(clock)),listener_(tf_),spinner_(2,&queue_) {
    Require(static_cast<bool>(clock_) && static_cast<bool>(costmap_),"evidence requires owned clock and costmap");
    model_digest_=MotionBytesDigest(Read(policy_path_,1024*1024));
    registry_digest_=MotionBytesDigest(Read(registry_path_,1024*1024));
    const auto filter_bytes=Read(lidar_path_,1024*1024);
    filter_digest_=MotionBytesDigest(filter_bytes);
    const auto filter=YAML::Load(filter_bytes);
    cloud_age_=filter["max_age_s"].as<double>();
    Require(Finite(cloud_age_) && cloud_age_>0 && cloud_age_<=1,"invalid installed sensor age limit");
    LoadMap();
    LoadDeparture(filter);
    node_.setCallbackQueue(&queue_);
    std::string pose_topic="/amcl_pose";node_.param("route_localization_pose_topic",pose_topic,pose_topic);
    map_sub_=node_.subscribe("/map",1,&Impl::Map,this);
    pose_sub_=node_.subscribe(pose_topic,1,&Impl::Pose,this);
    odom_sub_=node_.subscribe("/odom",1,&Impl::Odom,this);
    cloud_sub_=node_.subscribe("/workbench_nav/lidar_obstacle_snapshot",1,&Impl::Cloud,this);
    spinner_.start();
  }
  ~Impl() {map_sub_.shutdown();pose_sub_.shutdown();odom_sub_.shutdown();cloud_sub_.shutdown();queue_.disable();spinner_.stop();queue_.clear();}
  MotionPlanningEvidence Snapshot() {
    try {
      const double began=clock_();
      Require(!changed_,"installed route evidence changed; restart with matching configuration");
      if(MotionFileDigest(policy_path_)!=model_digest_ || MotionFileDigest(registry_path_)!=registry_digest_ ||
          MotionFileDigest(lidar_path_)!=filter_digest_ || MotionFileDigest(map_path_)!=yaml_digest_ ||
          MotionFileDigest(image_path_)!=image_digest_ ||
          (!departure_path_.empty() && MotionFileDigest(departure_path_)!=departure_digest_)) {
        changed_=true;throw std::invalid_argument("installed map, registry, policy or sensor filter changed");
      }
      geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose;nav_msgs::Odometry::ConstPtr odom;
      std::deque<std::pair<nav_msgs::Odometry::ConstPtr,double>> odom_samples;
      std::shared_ptr<const CapturedMotionCloud> cloud;double pose_receipt,odom_receipt;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        Require(map_valid_ && !map_conflict_,"live static map does not match installed map");
        Require(static_cast<bool>(pose_), ("localization evidence unavailable: "+pose_error_).c_str());
        Require(static_cast<bool>(odom_), ("odometry evidence unavailable: "+odom_error_).c_str());
        Require(static_cast<bool>(cloud_), ("captured obstacle evidence unavailable: "+cloud_error_).c_str());
        pose=pose_;odom_samples=odom_samples_;cloud=cloud_;pose_receipt=pose_receipt_;
      }
      Require(map_sub_.getNumPublishers()==1 && pose_sub_.getNumPublishers()==1 &&
          odom_sub_.getNumPublishers()==1 && cloud_sub_.getNumPublishers()==1,"route evidence source owner count mismatch");
      auto raw=costmap_();auto grid=std::move(raw.grid);
      const double copied=clock_();
      Require(Finite(raw.observed_steady_s) && raw.observed_steady_s>=0 && raw.observed_steady_s<=copied &&
          Finite(raw.deadline_steady_s) && raw.deadline_steady_s>copied &&
          raw.deadline_steady_s<=raw.observed_steady_s+.25,"raw costmap lease expired or invalid");
      Require(ValidMotionGrid(grid) && grid.cells.size()<=1000000 && SameGridGeometry(grid,expected_),"live costmap geometry mismatch");
      // Keep the verified original static map, including unknown cells. Raw
      // obstacle-layer NO_INFORMATION means no observation there; it cannot
      // erase map cells or introduce a blanket unknown mask over mapped space.
      for(std::size_t i=0;i<grid.cells.size();++i) {
        Require(grid.cells[i]==0 || grid.cells[i]==254,"inflated/soft cost presented as original occupancy");
        if(grid.cells[i]!=254)grid.cells[i]=expected_.cells[i];
      }
      const double now=clock_(),source_now=ros::Time::now().toSec();Require(now>=began,"route evidence clock regressed");
      // TF and odometry callbacks have independent transport order. Select the
      // newest original sample whose matching transforms have arrived; retain
      // its original receipt, and still apply the unchanged 0.25 s deadlines.
      for(auto candidate=odom_samples.rbegin();candidate!=odom_samples.rend();++candidate) {
        if(candidate->first->header.stamp.toSec()<=source_now &&
            tf_.canTransform("map","base_link",candidate->first->header.stamp) &&
            tf_.canTransform("odom","rslidar",candidate->first->header.stamp)) {
          odom=candidate->first;odom_receipt=candidate->second;break;
        }
      }
      Require(static_cast<bool>(odom),"odometry awaiting its original TF or source time");
      double pose_deadline=MotionEvidenceDeadline(now,source_now,pose->header.stamp.toSec(),pose_receipt,.75,.75);
      pose_deadline=std::min(pose_deadline,MotionEvidenceDeadline(now,source_now,odom->header.stamp.toSec(),odom_receipt,.25,.25));
      const auto base=tf_.lookupTransform("map","base_link",odom->header.stamp);
      const auto sensor=tf_.lookupTransform("map","rslidar",odom->header.stamp);
      ros::Time original_stamp;original_stamp.fromNSec(cloud->source_stamp_ns);
      const auto map_from_odom=tf_.lookupTransform("map","odom",original_stamp);
      Require(!base.header.stamp.isZero() && !sensor.header.stamp.isZero(),"current measured TF has no dynamic source stamp");
      pose_deadline=std::min(pose_deadline,MotionEvidenceDeadline(now,source_now,base.header.stamp.toSec(),odom_receipt,.75,.25));
      pose_deadline=std::min(pose_deadline,MotionEvidenceDeadline(now,source_now,sensor.header.stamp.toSec(),odom_receipt,.75,.25));
      const double pose_age=source_now-pose->header.stamp.toSec();
      Require(Nearby(Transform(sensor.transform),Transform(pose->pose.pose),.10+.5*pose_age,.10+.8*pose_age),
          "current map TF disagrees with localization evidence");
      const auto measured_odom=tf_.lookupTransform("odom","rslidar",odom->header.stamp);
      Require(Nearby(Transform(measured_odom.transform),Transform(odom->pose.pose),.01,.01),"odometry and its original TF disagree");
      const auto overlay=OverlayCapturedObstacles(std::move(grid),*cloud,Transform(map_from_odom.transform),source_now,now,
                                                model_digest_,filter_digest_,cloud_age_);
      MotionPlanningEvidence result;result.grid=overlay.grid;result.measured_pose=Planar(Transform(base.transform));
      result.frame_id="map";result.static_map_digest=map_digest_;result.registry_digest=registry_digest_;
      result.observed_at_s=now;result.pose_deadline_s=pose_deadline;
      result.obstacle_deadline_s=std::min(overlay.deadline_s,raw.deadline_steady_s);
      result.departure_policy=departure_policy_;result.registered_departures=departures_;
      std::vector<MotionDeparturePose> stopping_samples;
      for(const auto& sample:odom_samples)if(sample.first->header.stamp<=odom->header.stamp)
        stopping_samples.push_back({Planar(Transform(sample.first->pose.pose)),sample.first->header.stamp.toSec()});
      result.departure_stop_observed=ObservedDepartureStop(stopping_samples,odom->header.stamp.toSec());
      return result;
    } catch(const std::exception& error) {
      MotionPlanningEvidence unavailable;unavailable.unavailable_detail=std::string(error.what()).substr(0,256);return unavailable;
    }
  }
 private:
  void LoadDeparture(const YAML::Node& filter) {
    node_.param("route_docking_config_file",departure_path_,std::string{});
    if(departure_path_.empty())return; // No implicit phase qualification.
    const auto bytes=Read(departure_path_,1024*1024);departure_digest_=MotionBytesDigest(bytes);
    const auto controller=YAML::Load(bytes)["controller"];
    auto& policy=departure_policy_;
    policy.max_distance_m=controller["max_rollback_travel_m"].as<double>();
    policy.max_speed_mps=controller["max_rollback_speed_mps"].as<double>();
    policy.acceleration_mps2=controller["max_rollback_acceleration_mps2"].as<double>();
    policy.stop_dwell_s=controller["stop_dwell_s"].as<double>();
    policy.max_cross_track_m=controller["max_cross_track_m"].as<double>();
    policy.max_yaw_drift_rad=controller["max_yaw_drift_rad"].as<double>();
    const double rear=filter["footprint"]["rear_x_m"].as<double>();
    const double right=filter["footprint"]["right_y_m"].as<double>();
    const double left=filter["footprint"]["left_y_m"].as<double>();
    const double clearance=filter["static_clearance_m"].as<double>();
    const double latency=filter["latency"]["sensor_s"].as<double>()+
        filter["latency"]["command_s"].as<double>()+filter["latency"]["brake_s"].as<double>();
    const double braking=filter["minimum_braking_deceleration_mps2"].as<double>();
    Require(Finite(rear) && rear<0 && Finite(right) && right<0 && Finite(left) && left>0 &&
        Finite(clearance) && clearance>=0 && Finite(latency) && latency>=0 &&
        Finite(braking) && braking>0,"invalid installed departure sensor bounds");
    const double margin=clearance+policy.max_speed_mps*latency+
        policy.max_speed_mps*policy.max_speed_mps/(2*braking);
    policy.geometry=LoadInstalledMotionPolicy(policy_path_).preflight.geometry;
    policy.geometry.footprint={{{rear-margin,right}},{{0,right}},{{0,left}},{{rear-margin,left}}};
    policy.geometry.hard_padding_m=0; // Rear stopping allowance is directional.
    policy.geometry.integration_error_m=0;
    const auto registry=YAML::Load(Read(registry_path_,1024*1024));
    Require(registry["workbenches"].IsSequence() && registry["workbenches"].size()<=64,"departure station count exceeds bound");
    auto planar=[](const YAML::Node& pose) {
      Require(pose["frame"].as<std::string>()=="map" && pose["position_m"].size()==3 &&
          pose["orientation_xyzw"].size()==4,"invalid registered departure frame");
      const auto p=pose["position_m"],q=pose["orientation_xyzw"];
      const double x=q[0].as<double>(),y=q[1].as<double>(),z=q[2].as<double>(),w=q[3].as<double>();
      Require(Finite(x) && Finite(y) && Finite(z) && Finite(w) && std::abs(x)<1e-6 &&
          std::abs(y)<1e-6 && std::abs(z*z+w*w-1)<1e-6,"invalid registered departure orientation");
      MotionPose result{p[0].as<double>(),p[1].as<double>(),std::atan2(2*z*w,1-2*z*z)};
      Require(ValidMotionPose(result),"invalid registered departure pose");return result;
    };
    for(const auto& station:registry["workbenches"])
      departures_.push_back({station["id"].as<std::string>(),planar(station["map_pose"]),planar(station["staging_pose"])});
    std::string mode;node_.param("operation_mode",mode,std::string("hardware"));
    policy.enabled=mode=="simulation"; // Hardware phase qualification is separate.
  }
  void LoadMap() {
    const auto bytes=Read(map_path_,1024*1024);yaml_digest_=MotionBytesDigest(bytes);const auto yaml=YAML::Load(bytes);
    Require(yaml.IsMap() && yaml["origin"].IsSequence() && yaml["origin"].size()==3,"invalid installed map YAML");
    image_path_=yaml["image"].as<std::string>();Require(!image_path_.empty(),"missing map image");
    if(image_path_[0]!='/')image_path_=map_path_.substr(0,map_path_.find_last_of('/')+1)+image_path_;
    const auto image=Read(image_path_,16*1024*1024);image_digest_=MotionBytesDigest(image);
    // Decode these immutable bytes directly. Reopening the pathname in an image
    // library would let a file change bypass the checked dimensions/size bound.
    std::istringstream pgm(image);const auto magic=Token(pgm);Require(magic=="P5" || magic=="P2","route map image must be bounded PGM");
    const auto width=Dimension(Token(pgm)),height=Dimension(Token(pgm)),max_value=Dimension(Token(pgm));
    Require(width<=1000000/height && max_value==255,"map requires bounded 8-bit PGM cells");
    const double resolution=yaml["resolution"].as<double>(),occupied=yaml["occupied_thresh"].as<double>(),free=yaml["free_thresh"].as<double>();
    double origin[3]={yaml["origin"][0].as<double>(),yaml["origin"][1].as<double>(),yaml["origin"][2].as<double>()};
    const int negate=yaml["negate"].as<int>();
    Require(Finite(resolution) && resolution>0 && Finite(occupied) && Finite(free) && free>=0 && free<occupied && occupied<=1 &&
      Finite(origin[0]) && Finite(origin[1]) && Finite(origin[2]) && std::abs(origin[2])<1e-9 && (negate==0 || negate==1),"unsupported map geometry or threshold");
    Require(!yaml["mode"] || yaml["mode"].as<std::string>()=="trinary","route map requires trinary mode");
    expected_.width=width;expected_.height=height;expected_.resolution_m=resolution;expected_.origin_x_m=origin[0];expected_.origin_y_m=origin[1];
    expected_.cells.resize(width*height);
    std::size_t offset=0;
    if(magic=="P5") {
      const int separator=pgm.get();
      Require(separator==' ' || separator=='\t' || separator=='\n' || separator=='\r',"missing binary PGM separator");
      if(separator=='\r' && pgm.peek()=='\n')pgm.get();
      const auto position=pgm.tellg();Require(position>=0,"truncated PGM header");offset=static_cast<std::size_t>(position);
      Require(offset<=image.size() && image.size()-offset>=width*height,"truncated binary PGM pixels");
    }
    for(std::size_t i=0;i<width*height;++i) {
      unsigned int pixel=0;
      if(magic=="P5")pixel=static_cast<unsigned char>(image[offset+i]);
      else {
        const auto token=Token(pgm);Require(!token.empty() && token.size()<=3 &&
            token.find_first_not_of("0123456789")==std::string::npos,"invalid ASCII PGM pixel");
        pixel=std::stoul(token);Require(pixel<=255,"ASCII PGM pixel exceeds depth");
      }
      const double occupancy=negate?pixel/255.:(255-pixel)/255.;
      const auto index=(height-1-i/width)*width+i%width;
      expected_.cells[index]=occupancy>occupied?254:(occupancy<free?0:255);
    }
    Require(MotionFileDigest(map_path_)==yaml_digest_ && MotionFileDigest(image_path_)==image_digest_,"map changed while decoding");
    map_digest_=MotionBytesDigest(yaml_digest_.substr(7)+"|"+image_digest_.substr(7));
  }
  void Map(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    bool valid=false;
    try {
      MotionGrid geometry;geometry.width=msg->info.width;geometry.height=msg->info.height;geometry.resolution_m=msg->info.resolution;
      geometry.origin_x_m=msg->info.origin.position.x;geometry.origin_y_m=msg->info.origin.position.y;
      Require(msg->header.frame_id=="map" && SameGridGeometry(geometry,expected_) && msg->data.size()==expected_.cells.size(),"live map geometry mismatch");
      const auto origin=Transform(msg->info.origin);Require(std::abs(origin.position[2])<1e-9 &&
          std::abs(origin.orientation[0])+std::abs(origin.orientation[1])+std::abs(origin.orientation[2])<1e-9,"live map origin rotation unsupported");
      for(std::size_t i=0;i<msg->data.size();++i) {
        const auto cell=msg->data[i];Require(cell==-1 || cell==0 || cell==100,"nontrinary live map");
        Require(expected_.cells[i]==(cell<0?255:(cell==0?0:254)),"live map content mismatch");
      }
      valid=true;
    } catch(const std::exception&) {}
    std::lock_guard<std::mutex> lock(mutex_);if(!valid)map_conflict_=true;map_valid_=valid;
  }
  void Pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    const double receipt=clock_();std::lock_guard<std::mutex> lock(mutex_);pose_.reset();
    try {
      Require(msg->header.frame_id=="map" && msg->header.stamp>pose_stamp_ && receipt>=pose_receipt_,"invalid localization frame or order");
      MotionEvidenceDeadline(receipt,ros::Time::now().toSec(),msg->header.stamp.toSec(),receipt,.75,.75);
      const auto pose=Transform(msg->pose.pose);Covariance(msg->pose.covariance);
      if(last_pose_) {const double dt=std::min(.75,(msg->header.stamp-last_pose_->header.stamp).toSec());
        Require(Nearby(pose,Transform(last_pose_->pose.pose),.10+.5*dt,.10+.8*dt),"localization discontinuity");}
      pose_=last_pose_=msg;pose_stamp_=msg->header.stamp;pose_receipt_=receipt;pose_error_.clear();
    } catch(const std::exception& error) {pose_error_=std::string(error.what()).substr(0,256);}
  }
  void Odom(const nav_msgs::Odometry::ConstPtr& msg) {
    const double receipt=clock_();std::lock_guard<std::mutex> lock(mutex_);odom_.reset();
    try {
      Require(msg->header.frame_id=="odom" && msg->child_frame_id=="rslidar" && msg->header.stamp>odom_stamp_ && receipt>=odom_receipt_,"invalid odometry frame or order");
      MotionEvidenceDeadline(receipt,ros::Time::now().toSec(),msg->header.stamp.toSec(),receipt,.25,.25);
      Transform(msg->pose.pose);Covariance(msg->pose.covariance);Covariance(msg->twist.covariance);
      const auto& v=msg->twist.twist;for(double value:{v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z})Require(Finite(value),"nonfinite odometry velocity");
      odom_=msg;odom_stamp_=msg->header.stamp;odom_receipt_=receipt;odom_error_.clear();
      odom_samples_.emplace_back(msg,receipt);
      if(odom_samples_.size()>64)odom_samples_.pop_front();
    } catch(const std::exception& error) {odom_samples_.clear();odom_error_=std::string(error.what()).substr(0,256);}
  }
  void Cloud(const LidarObstacleSnapshot::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);cloud_.reset();
    try {
      Require(msg->header.stamp>=cloud_stamp_,"obstacle source stamp regressed");
      if(msg->header.stamp==cloud_stamp_ && last_cloud_)Require(SameCapture(*msg,*last_cloud_),"conflicting repeated obstacle capture");
      auto decoded=Decode(*msg);
      // Validation includes original receipt; timer publication cannot renew it.
      OverlayCapturedObstacles(expected_,decoded,MotionCaptureTransform{},ros::Time::now().toSec(),clock_(),model_digest_,filter_digest_,cloud_age_);
      Require(!last_cloud_ || decoded.receipt_steady_s>=last_cloud_->receipt_steady_s,"obstacle receipt regressed");
      cloud_=std::make_shared<const CapturedMotionCloud>(std::move(decoded));last_cloud_=msg;cloud_stamp_=msg->header.stamp;cloud_error_.clear();
    } catch(const std::exception& error) {cloud_error_=std::string(error.what()).substr(0,256);}
  }
  ros::CallbackQueue queue_;ros::NodeHandle node_;std::string policy_path_,map_path_,registry_path_,lidar_path_,image_path_;
  std::string model_digest_,registry_digest_,filter_digest_,yaml_digest_,image_digest_,map_digest_;
  std::string departure_path_,departure_digest_;
  MotionDeparturePolicy departure_policy_;
  std::vector<RegisteredMotionDeparture> departures_;
  std::string pose_error_,odom_error_,cloud_error_;
  MotionGrid expected_;GridSource costmap_;std::function<double()> clock_;double cloud_age_=.35;
  tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;ros::AsyncSpinner spinner_;
  ros::Subscriber map_sub_,pose_sub_,odom_sub_,cloud_sub_;
  std::mutex mutex_;bool map_valid_=false,map_conflict_=false;std::atomic<bool> changed_{false};
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose_,last_pose_;nav_msgs::Odometry::ConstPtr odom_;
  std::deque<std::pair<nav_msgs::Odometry::ConstPtr,double>> odom_samples_;
  LidarObstacleSnapshot::ConstPtr last_cloud_;std::shared_ptr<const CapturedMotionCloud> cloud_;
  ros::Time pose_stamp_,odom_stamp_,cloud_stamp_;double pose_receipt_=0,odom_receipt_=0;
};
RosMotionEvidence::RosMotionEvidence(ros::NodeHandle node,std::string policy_path,std::string map_path,
    std::string registry_path,std::string lidar_path,GridSource costmap,std::function<double()> clock)
    :impl_(new Impl(std::move(node),std::move(policy_path),std::move(map_path),std::move(registry_path),
                    std::move(lidar_path),std::move(costmap),std::move(clock))) {}
RosMotionEvidence::~RosMotionEvidence()=default;
MotionPlanningEvidence RosMotionEvidence::Snapshot() {return impl_->Snapshot();}
namespace {
std::mutex shared_evidence_mutex;
std::shared_ptr<const MotionPlanningEvidence> shared_evidence;
}
void PublishSharedMotionEvidence(MotionPlanningEvidence evidence) {
  auto immutable=std::make_shared<const MotionPlanningEvidence>(std::move(evidence));
  std::lock_guard<std::mutex> lock(shared_evidence_mutex);shared_evidence=std::move(immutable);
}
std::shared_ptr<const MotionPlanningEvidence> SharedMotionEvidence() {
  std::lock_guard<std::mutex> lock(shared_evidence_mutex);return shared_evidence;
}
}
