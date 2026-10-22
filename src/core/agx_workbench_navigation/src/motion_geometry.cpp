#include <agx_workbench_navigation/motion_geometry.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace agx_workbench_navigation {
namespace {
double Cross(const std::array<double,2>& a, const std::array<double,2>& b,
             const std::array<double,2>& p) {
  return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]);
}
bool Positive(double v) { return std::isfinite(v) && v>0; }
bool Nonnegative(double v) { return std::isfinite(v) && v>=0; }
bool Overlap(const MotionPolygon& a,const MotionPolygon& b,double padding) {
  for(const auto* axes:{&a,&b}) {
    for(std::size_t i=0;i<axes->size();++i) {
      const auto& p=(*axes)[i];const auto& q=(*axes)[(i+1)%axes->size()];
      const double nx=p[1]-q[1],ny=q[0]-p[0];
      const double extra=padding*std::hypot(nx,ny)+1e-12;
      double amin=INFINITY,amax=-INFINITY,bmin=INFINITY,bmax=-INFINITY;
      for(const auto& v:a) {const double d=nx*v[0]+ny*v[1];amin=std::min(amin,d);amax=std::max(amax,d);}
      for(const auto& v:b) {const double d=nx*v[0]+ny*v[1];bmin=std::min(bmin,d);bmax=std::max(bmax,d);}
      if(amax+extra<bmin || bmax+extra<amin)return false;
    }
  }
  return true;
}
}
bool ValidMotionPolygon(const MotionPolygon& poly) {
  if(poly.size()<3 || poly.size()>64)return false;
  for(const auto& p:poly)if(!std::isfinite(p[0]) || !std::isfinite(p[1]) ||
                           std::abs(p[0])>1e4 || std::abs(p[1])>1e4)return false;
  const double first=Cross(poly[0],poly[1],poly[2]);
  if(std::abs(first)<1e-12)return false;
  for(std::size_t i=0;i<poly.size();++i) {
    const auto& a=poly[i];const auto& b=poly[(i+1)%poly.size()];
    if(Cross(a,b,poly[(i+2)%poly.size()])*first<=0)return false;
    for(const auto& p:poly)if(Cross(a,b,p)*first < -1e-12)return false;
  }
  return true;
}
bool MotionPolygonContains(const MotionPolygon& outer,const MotionPolygon& inner) {
  if(!ValidMotionPolygon(outer) || !ValidMotionPolygon(inner))return false;
  const double sign=Cross(outer[0],outer[1],outer[2])>0?1:-1;
  for(std::size_t i=0;i<outer.size();++i)
    for(const auto& p:inner)
      if(sign*Cross(outer[i],outer[(i+1)%outer.size()],p)<-1e-12)return false;
  return true;
}
bool ValidMotionGrid(const MotionGrid& g) {
  return g.width>0 && g.height>0 && g.width<=100000 && g.height<=100000 &&
      g.width<=100000000/g.height && g.cells.size()==g.width*g.height &&
      Positive(g.resolution_m) && std::isfinite(g.origin_x_m) && std::isfinite(g.origin_y_m) &&
      std::abs(g.origin_x_m)<=1e6 && std::abs(g.origin_y_m)<=1e6 &&
      g.width*g.resolution_m<=1e6 && g.height*g.resolution_m<=1e6;
}
namespace {
MotionSweepResult CheckSweep(const MotionGrid& g,const MotionGeometryPolicy& p,
    const std::vector<MotionPose>& poses,
    const std::vector<std::vector<std::size_t>>* rows,
    const std::vector<std::size_t>* prefix) {
  MotionSweepResult out;
  if(!ValidMotionGrid(g) || !ValidMotionPolygon(p.footprint) || poses.empty() ||
     !Nonnegative(p.hard_padding_m) || !Nonnegative(p.integration_error_m) ||
     !Positive(p.translation_step_m) || !Positive(p.rotation_step_rad) ||
     p.max_samples==0 || p.max_samples>1000000)return out;
  for(const auto& pose:poses)if(!ValidMotionPose(pose))return out;
  double radius=0;
  for(const auto& v:p.footprint)radius=std::max(radius,std::hypot(v[0],v[1]));
  out.interpolation_padding_m=poses.size()>1 ?
      (p.translation_step_m+radius*p.rotation_step_rad)/2+p.integration_error_m : 0;
  const double padding=p.hard_padding_m+out.interpolation_padding_m;
  if(!std::isfinite(padding))return out;
  auto check=[&](const MotionPose& pose) {
    out.first_pose=pose;
    if(out.samples>=p.max_samples) {out.status=MotionCollision::SampleLimit;return false;}
    ++out.samples;
    MotionPolygon body;body.reserve(p.footprint.size());
    const double c=std::cos(pose.yaw_rad),s=std::sin(pose.yaw_rad);
    double xmin=INFINITY,xmax=-INFINITY,ymin=INFINITY,ymax=-INFINITY;
    for(const auto& v:p.footprint) {
      const double x=pose.x_m+c*v[0]-s*v[1],y=pose.y_m+s*v[0]+c*v[1];
      body.push_back({{x,y}});
      xmin=std::min(xmin,x-padding);xmax=std::max(xmax,x+padding);
      ymin=std::min(ymin,y-padding);ymax=std::max(ymax,y+padding);
    }
    if(xmin<=g.origin_x_m || ymin<=g.origin_y_m ||
       xmax>=g.origin_x_m+g.width*g.resolution_m || ymax>=g.origin_y_m+g.height*g.resolution_m) {
      out.status=MotionCollision::MapBoundary;return false;
    }
    // Include both cells when a polygon touches a cell boundary.
    const auto minx=static_cast<std::size_t>(std::max(0.,std::floor((xmin-g.origin_x_m)/g.resolution_m-1e-9)));
    const auto miny=static_cast<std::size_t>(std::max(0.,std::floor((ymin-g.origin_y_m)/g.resolution_m-1e-9)));
    const auto maxx=std::min(g.width-1,static_cast<std::size_t>(std::floor((xmax-g.origin_x_m)/g.resolution_m+1e-9)));
    const auto maxy=std::min(g.height-1,static_cast<std::size_t>(std::floor((ymax-g.origin_y_m)/g.resolution_m+1e-9)));
    if(prefix) {
      // Empty bounding boxes need no per-row lookup or polygon/cell SAT.
      const auto stride=g.width+1;
      const auto& sums=*prefix;
      const auto total=sums[(maxy+1)*stride+maxx+1]-sums[miny*stride+maxx+1]
                       -sums[(maxy+1)*stride+minx]+sums[miny*stride+minx];
      if(total==0)return true;
    }
    auto cell_overlaps=[&](std::size_t x,std::size_t y) {
      const auto cost=g.cells[y*g.width+x];if(cost<254)return false;
      const double l=g.origin_x_m+x*g.resolution_m,b=g.origin_y_m+y*g.resolution_m;
      const MotionPolygon cell{{{l,b}},{{l+g.resolution_m,b}},
          {{l+g.resolution_m,b+g.resolution_m}},{{l,b+g.resolution_m}}};
      if(Overlap(body,cell,padding)) {
        out.cell_x=x;out.cell_y=y;
        out.status=cost==255?MotionCollision::Unknown:MotionCollision::Occupied;return true;
      }
      return false;
    };
    for(std::size_t y=miny;y<=maxy;++y) {
      if(rows) {
        const auto& row=(*rows)[y];
        for(auto it=std::lower_bound(row.begin(),row.end(),minx);it!=row.end() && *it<=maxx;++it)
          if(cell_overlaps(*it,y))return false;
      } else {
        for(std::size_t x=minx;x<=maxx;++x)if(cell_overlaps(x,y))return false;
      }
    }
    return true;
  };
  if(poses.size()==1) {if(!check(poses.front()))return out;}
  for(std::size_t i=1;i<poses.size();++i) {
    const auto& a=poses[i-1];const auto& b=poses[i];
    const double yaw=MotionAngleDifference(b.yaw_rad,a.yaw_rad);
    const double steps=std::max(1.,std::max(std::ceil(std::hypot(b.x_m-a.x_m,b.y_m-a.y_m)/p.translation_step_m),
                                          std::ceil(std::abs(yaw)/p.rotation_step_rad)));
    if(!std::isfinite(steps) || steps>p.max_samples-out.samples) {
      out.status=MotionCollision::SampleLimit;return out;
    }
    const auto count=static_cast<std::size_t>(steps);
    for(std::size_t j=(i==1?0:1);j<=count;++j) {
      const double t=static_cast<double>(j)/count;
      const double angle=a.yaw_rad+t*yaw;
      if(!check({a.x_m+t*(b.x_m-a.x_m),a.y_m+t*(b.y_m-a.y_m),std::atan2(std::sin(angle),std::cos(angle))}))return out;
    }
  }
  out.status=MotionCollision::Clear;return out;
}
}  // namespace
MotionSweepResult CheckMotionSweep(const MotionGrid& grid,const MotionGeometryPolicy& policy,
                                  const std::vector<MotionPose>& poses) {
  return CheckSweep(grid,policy,poses,nullptr,nullptr);
}
MotionSweepChecker::MotionSweepChecker(const MotionGrid& grid) {
  if(!ValidMotionGrid(grid))return;
  grid_=grid;occupied_rows_.resize(grid.height);
  const auto stride=grid.width+1;
  occupied_prefix_.assign(stride*(grid.height+1),0);
  for(std::size_t y=0;y<grid.height;++y) {
    std::size_t row_count=0;
    for(std::size_t x=0;x<grid.width;++x) {
      if(grid.cells[y*grid.width+x]>=254) {occupied_rows_[y].push_back(x);++row_count;}
      occupied_prefix_[(y+1)*stride+x+1]=occupied_prefix_[y*stride+x+1]+row_count;
    }
  }
}
MotionSweepResult MotionSweepChecker::Check(const MotionGeometryPolicy& policy,
                                           const std::vector<MotionPose>& poses) const {
  return CheckSweep(grid_,policy,poses,&occupied_rows_,&occupied_prefix_);
}
}  // namespace agx_workbench_navigation
