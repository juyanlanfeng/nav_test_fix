// Conservative triangle/AABB surface rasterizer for binary STL.
// Output: sorted int32 XYZ voxel keys, little endian on the supported ROS host.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <vector>

using Vec = std::array<double, 3>;
using Key = std::array<int32_t, 3>;
Vec sub(Vec a, Vec b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
Vec cross(Vec a, Vec b) {
  return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
}
double dot(Vec a, Vec b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
struct Hash {
  size_t operator()(const Key& k) const {
    size_t h = 0;
    for (auto v:k) h ^= std::hash<int32_t>{}(v) + 0x9e3779b9 + (h<<6) + (h>>2);
    return h;
  }
};

// Separating-axis theorem: box axes, triangle normal, and edge x box axes.
bool intersects(const std::array<Vec,3>& t, Key key, double pitch,
                double min_z, double max_z) {
  Vec lo, hi, c, half;
  for (int a=0;a<3;++a) {lo[a]=key[a]*pitch; hi[a]=lo[a]+pitch;}
  lo[2]=std::max(lo[2],min_z); hi[2]=std::min(hi[2],max_z);
  if (lo[2]>hi[2]) return false;
  for (int a=0;a<3;++a) {c[a]=.5*(lo[a]+hi[a]); half[a]=.5*(hi[a]-lo[a]);}
  std::array<Vec,3> v = {sub(t[0],c),sub(t[1],c),sub(t[2],c)};
  auto separated = [&](Vec axis) {
    double p0=dot(axis,v[0]),p1=dot(axis,v[1]),p2=dot(axis,v[2]);
    double r=std::abs(axis[0])*half[0]+std::abs(axis[1])*half[1]+std::abs(axis[2])*half[2];
    double eps=1.e-10*(std::abs(axis[0])+std::abs(axis[1])+std::abs(axis[2]));
    return std::min({p0,p1,p2})>r+eps || std::max({p0,p1,p2})<-r-eps;
  };
  const std::array<Vec,3> axes={Vec{1,0,0},Vec{0,1,0},Vec{0,0,1}};
  for (auto a:axes) if (separated(a)) return false;
  Vec e0=sub(v[1],v[0]),e1=sub(v[2],v[1]),e2=sub(v[0],v[2]);
  if (separated(cross(e0,e1))) return false;
  for(auto e:{e0,e1,e2}) for(auto a:axes) if(separated(cross(e,a))) return false;
  return true;
}

int main(int argc,char** argv) {
  try {
    if(argc!=6) throw std::runtime_error("usage: surface_voxelizer input.stl keys.bin pitch min_z max_z");
    double pitch=std::stod(argv[3]),min_z=std::stod(argv[4]),max_z=std::stod(argv[5]);
    if(!(pitch>0 && std::isfinite(pitch) && std::isfinite(min_z) &&
         std::isfinite(max_z) && min_z<=max_z)) throw std::runtime_error("invalid grid/band");
    std::ifstream in(argv[1],std::ios::binary);
    char header[80];uint32_t faces=0;in.read(header,80);in.read(reinterpret_cast<char*>(&faces),4);
    if(!in) throw std::runtime_error("cannot read STL header");
    std::unordered_set<Key,Hash> keys; keys.reserve(1000000);
    uint64_t tests=0;
    for(uint32_t f=0;f<faces;++f) {
      float record[12];uint16_t attr;
      in.read(reinterpret_cast<char*>(record),48);in.read(reinterpret_cast<char*>(&attr),2);
      if(!in) throw std::runtime_error("truncated binary STL");
      std::array<Vec,3> t;
      for(int i=0;i<3;++i) for(int a=0;a<3;++a) {
        t[i][a]=record[3+3*i+a];
        if(!std::isfinite(t[i][a]) || std::abs(t[i][a]/pitch)>1.e8)
          throw std::runtime_error("invalid/out-of-range STL coordinate");
      }
      Vec lo,hi;
      for(int a=0;a<3;++a){lo[a]=std::min({t[0][a],t[1][a],t[2][a]});hi[a]=std::max({t[0][a],t[1][a],t[2][a]});}
      if(hi[2]<min_z || lo[2]>max_z)continue;
      lo[2]=std::max(lo[2],min_z);hi[2]=std::min(hi[2],max_z);
      Vec normal=cross(sub(t[1],t[0]),sub(t[2],t[0]));
      int dominant=0;
      for(int a=1;a<3;++a)if(std::abs(normal[a])>std::abs(normal[dominant]))dominant=a;
      if(std::abs(normal[dominant])<1.e-18)continue;
      int u=(dominant+1)%3,v=(dominant+2)%3;
      Key lower,upper;
      for(int a=0;a<3;++a){lower[a]=static_cast<int32_t>(std::floor((lo[a]-1.e-10)/pitch));upper[a]=static_cast<int32_t>(std::floor((hi[a]+1.e-10)/pitch));}
      lower[2]=std::max(lower[2],static_cast<int32_t>(std::floor(min_z/pitch)));
      upper[2]=std::min(upper[2],static_cast<int32_t>(std::floor(max_z/pitch)));
      for(int32_t i=lower[u];i<=upper[u];++i)for(int32_t j=lower[v];j<=upper[v];++j){
        // Only visit the few depth cells intersected by the triangle's plane
        // above this projected grid square, not its entire 3-D bounding box.
        double dmin=1.e100,dmax=-1.e100;
        for(int du=0;du<2;++du)for(int dv=0;dv<2;++dv){
          double d=t[0][dominant]-(normal[u]*((i+du)*pitch-t[0][u])+normal[v]*((j+dv)*pitch-t[0][v]))/normal[dominant];
          dmin=std::min(dmin,d);dmax=std::max(dmax,d);
        }
        int32_t begin=std::max(lower[dominant],static_cast<int32_t>(std::floor((dmin-1.e-10)/pitch)));
        int32_t end=std::min(upper[dominant],static_cast<int32_t>(std::floor((dmax+1.e-10)/pitch)));
        for(int32_t d=begin;d<=end;++d){
          Key k;k[u]=i;k[v]=j;k[dominant]=d;
          if(keys.count(k))continue;
          ++tests;
          if(intersects(t,k,pitch,min_z,max_z))keys.insert(k);
        }
      }
    }
    std::vector<Key> sorted(keys.begin(),keys.end());std::sort(sorted.begin(),sorted.end());
    std::ofstream out(argv[2],std::ios::binary);
    for(auto k:sorted)out.write(reinterpret_cast<const char*>(k.data()),12);
    if(!out)throw std::runtime_error("cannot write voxel keys");
    std::cout<<"Triangle/AABB rasterization: faces="<<faces<<" tested="<<tests<<" occupied="<<keys.size()<<"\n";
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
