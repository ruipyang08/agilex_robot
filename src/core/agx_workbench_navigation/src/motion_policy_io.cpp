#include <agx_workbench_navigation/motion_policy_io.hpp>
#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

namespace agx_workbench_navigation {
namespace {
void Require(bool condition,const std::string& reason) {
  if(!condition)throw std::invalid_argument("motion policy: "+reason);
}
std::string Bytes(const std::string& path,std::size_t maximum) {
  std::ifstream stream(path,std::ios::binary | std::ios::ate);
  if(!stream)throw std::runtime_error("cannot read installed motion source: "+path);
  const auto size=stream.tellg();
  Require(size>0 && static_cast<std::size_t>(size)<=maximum,"source exceeds size bound");
  std::string result(static_cast<std::size_t>(size),'\0');stream.seekg(0);
  if(!stream.read(&result[0],size))throw std::runtime_error("cannot read complete installed motion source");
  return result;
}
void Keys(const YAML::Node& node,const std::set<std::string>& expected) {
  Require(node.IsMap(),"mapping required");std::set<std::string> actual;
  for(const auto& item:node) {
    Require(item.first.IsScalar() && actual.insert(item.first.as<std::string>()).second,"duplicate/non-text key");
  }
  Require(actual==expected,"missing or unsupported keys");
}
double FiniteNumber(const YAML::Node& node) {
  Require(node.IsScalar() && node.Tag()!="!" && node.Tag()!="tag:yaml.org,2002:str","number required");
  const auto value=node.as<double>();
  Require(std::isfinite(value),"finite number required");return value;
}
double Number(const YAML::Node& node,bool positive=false) {
  const auto value=FiniteNumber(node);
  Require(std::isfinite(value) && (positive?value>0:value>=0),"finite nonnegative/positive number required");
  return value;
}
std::string Text(const YAML::Node& node) {
  Require(node.IsScalar(),"text required");const auto value=node.as<std::string>();
  Require(!value.empty() && value.size()<=512,"bounded nonempty text required");return value;
}
}
std::string MotionBytesDigest(const std::string& bytes) {
  unsigned char digest[EVP_MAX_MD_SIZE];unsigned int size=0;
  if(EVP_Digest(bytes.data(),bytes.size(),digest,&size,EVP_sha256(),nullptr)!=1)
    throw std::runtime_error("motion source SHA256 failed");
  std::ostringstream out;out<<"sha256:"<<std::hex<<std::setfill('0');
  for(unsigned int i=0;i<size;++i)out<<std::setw(2)<<static_cast<int>(digest[i]);
  return out.str();
}
std::string MotionFileDigest(const std::string& path) {return MotionBytesDigest(Bytes(path,16*1024*1024));}
InstalledMotionPolicy LoadInstalledMotionPolicy(const std::string& path) {
  const auto bytes=Bytes(path,1024*1024);const auto documents=YAML::LoadAll(bytes);
  Require(documents.size()==1,"one YAML document required");const auto raw=documents.front();
  Require(raw.IsMap() && raw["schema_version"].IsScalar(),"versioned mapping required");
  const int version=raw["schema_version"].as<int>();Require(version==1 || version==2,"unsupported schema");
  std::set<std::string> keys{"schema_version","model_id","frame_id","execution_enabled","qualification",
      "footprint_xy_m","error_budget_m","soft_clearance_m","sampling","motion_limits","evidence_ids"};
  if(version==2)keys.insert("footprint_provenance");Keys(raw,keys);
  const auto enabled=raw["execution_enabled"];
  Require(enabled.IsScalar() && enabled.Tag()!="!" && enabled.Tag()!="tag:yaml.org,2002:str" &&
      (enabled.Scalar()=="false" || enabled.Scalar()=="true"),"execution boolean required");
  const bool execute=enabled.Scalar()=="true";
  Require(execute?(version==2 && Text(raw["qualification"])=="experimental_simulation"):
      Text(raw["qualification"])=="unqualified","unsupported execution qualification");
  Require(Text(raw["frame_id"])=="base_link","unsupported footprint frame");Text(raw["model_id"]);
  InstalledMotionPolicy policy;policy.source_path=path;policy.execution_enabled=execute;
  auto& config=policy.preflight;config.model_digest=MotionBytesDigest(bytes);config.frame_id="map";
  config.geometry.footprint.clear();const auto points=raw["footprint_xy_m"];
  Require(points.IsSequence() && points.size()>=3 && points.size()<=64,"bounded polygon required");
  for(const auto& point:points) {
    Require(point.IsSequence() && point.size()==2,"XY pair required");
    const double x=FiniteNumber(point[0]),y=FiniteNumber(point[1]);
    Require(std::isfinite(x) && std::isfinite(y) && std::hypot(x,y)<=1.,"finite supported footprint required");
    config.geometry.footprint.push_back({{x,y}});
  }
  Require(ValidMotionPolygon(config.geometry.footprint),"ordered convex origin-containing footprint required");
  if(version==1)Require(config.geometry.footprint==MotionGeometryPolicy().footprint,"schema one requires canonical rectangle");
  else {
    const auto provenance=raw["footprint_provenance"];Keys(provenance,{"scope","source_sha256"});
    Require(Text(provenance["scope"])=="diagnostic_candidate","diagnostic footprint scope required");
    const auto hashes=provenance["source_sha256"];Require(hashes.IsMap() && hashes.size()>0,"footprint sources required");
    std::set<std::string> paths;
    for(const auto& source:hashes) {
      const auto name=Text(source.first),hash=Text(source.second);
      Require(name.front()!='/' && name.find('\\')==std::string::npos && paths.insert(name).second &&
          name!=".." && name.compare(0,3,"../")!=0 && name.find("/../")==std::string::npos &&
          !(name.size()>=3 && name.compare(name.size()-3,3,"/..")==0),"portable unique footprint source required");
      Require(hash.size()==64 && hash.find_first_not_of("0123456789abcdef")==std::string::npos,"source SHA256 required");
    }
  }
  const auto budget=raw["error_budget_m"];Keys(budget,{"geometry","localization","tracking","reserve"});
  config.geometry.hard_padding_m=0;for(const auto& item:budget)config.geometry.hard_padding_m+=Number(item.second);
  Require(std::isfinite(config.geometry.hard_padding_m),"combined error budget must be finite");
  config.geometry.soft_clearance_m=Number(raw["soft_clearance_m"]);
  const auto sampling=raw["sampling"];Keys(sampling,{"max_translation_step_m","max_rotation_step_rad"});
  config.geometry.translation_step_m=Number(sampling["max_translation_step_m"],true);
  config.geometry.rotation_step_rad=Number(sampling["max_rotation_step_rad"],true);
  const auto limits=raw["motion_limits"];
  Keys(limits,{"max_speed_mps","acceleration_mps2","deceleration_mps2","max_yaw_rate_radps",
      "angular_acceleration_radps2","curvature_rate_per_m_s","lateral_acceleration_mps2","min_turn_radius_m",
      "steering_settle_timeout_s","stop_dwell_s"});
  auto& l=config.limits;
  l.max_speed_mps=Number(limits["max_speed_mps"],true);l.acceleration_mps2=Number(limits["acceleration_mps2"],true);
  l.deceleration_mps2=Number(limits["deceleration_mps2"],true);l.max_yaw_rate_radps=Number(limits["max_yaw_rate_radps"],true);
  l.angular_acceleration_radps2=Number(limits["angular_acceleration_radps2"],true);
  l.curvature_rate_per_m_s=Number(limits["curvature_rate_per_m_s"],true);
  l.lateral_acceleration_mps2=Number(limits["lateral_acceleration_mps2"],true);
  l.min_turn_radius_m=Number(limits["min_turn_radius_m"],true);
  l.steering_settle_s=Number(limits["steering_settle_timeout_s"],true);l.stop_dwell_s=Number(limits["stop_dwell_s"],true);
  Require(ValidateMotionLimits(l).empty(),"unsupported motion limits");
  Require(l.max_speed_mps<=.5 && l.acceleration_mps2<=.25 && l.deceleration_mps2<=.3 &&
      l.max_yaw_rate_radps<=kMotionYawRateCeilingRadps && l.angular_acceleration_radps2<=.4 && l.curvature_rate_per_m_s<=.5 &&
      l.lateral_acceleration_mps2<=.2 && l.min_turn_radius_m>=.4764 && l.steering_settle_s<=2. &&
      l.stop_dwell_s>=.12,"motion limits exceed preview capability");
  const auto evidence=raw["evidence_ids"];Require(evidence.IsSequence() && evidence.size()>0,"evidence IDs required");
  std::set<std::string> evidence_ids;
  for(const auto& id:evidence)Require(evidence_ids.insert(Text(id)).second,"unique evidence IDs required");
  return policy;
}
}  // namespace agx_workbench_navigation
