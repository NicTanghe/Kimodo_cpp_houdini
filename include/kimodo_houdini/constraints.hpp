#pragma once
#include <kimodo_houdini/fk_skeleton.hpp>
#include <kimodo/kimodo_capi.h>
#include <algorithm>
#include <charconv>
#include <map>
#include <set>
#include <string>

namespace kimodo_houdini {
// Absolute Houdini frame numbers; ranges are inclusive, optionally start-end:step.
inline std::vector<int> parse_capture_frames(std::string text, int first, int count) {
    for (char &c:text) if (c==',') c=' ';
    std::istringstream input(text);
    std::set<int> frames;
    auto number=[](std::string_view s) {
        int n=0; auto [end,ec]=std::from_chars(s.data(),s.data()+s.size(),n);
        if(ec!=std::errc{} || end!=s.data()+s.size()) throw std::runtime_error("Invalid capture frame: "+std::string(s));
        return n;
    };
    for(std::string token; input>>token;) {
        const auto dash=token.find('-',1), colon=token.find(':');
        int a=number(std::string_view(token).substr(0,dash));
        int b=a, step=1;
        if(dash!=std::string::npos) {
            b=number(std::string_view(token).substr(dash+1,colon==std::string::npos?colon:colon-dash-1));
            if(colon!=std::string::npos) step=number(std::string_view(token).substr(colon+1));
        } else if(colon!=std::string::npos) throw std::runtime_error("Frame step requires a range");
        if(step<1 || a>b || a<first || static_cast<long long>(b)>=static_cast<long long>(first)+count)
            throw std::runtime_error("Capture frames must be inside the generated clip and use increasing ranges");
        for(long long f=a;f<=b;f+=step) frames.insert(static_cast<int>(f));
    }
    return {frames.begin(),frames.end()};
}
struct PoseCapture {
    int frame=0; // zero-based clip index
    std::vector<Vec3> positions;
    std::vector<std::array<float,9>> transforms; // Houdini row-vector world rotations
    std::vector<bool> selected;
    bool rotations=false;
};
struct Waypoint { int frame=0; float x=0,z=0; bool has_heading=false; float heading=0; };
struct Conditioning {
    unsigned frames=0,joints=0;
    std::vector<float> values,mask;
    bool any=false;
    void write(int frame, size_t feature, float value) {
        if(frame<0 || static_cast<unsigned>(frame)>=frames || !std::isfinite(value))
            throw std::runtime_error("Constraint frame is outside the clip, or value is not finite");
        const size_t i=static_cast<size_t>(frame)*(9+12*joints)+feature;
        if(mask.at(i) && std::abs(values[i]-value)>1.e-4f)
            throw std::runtime_error("Conflicting pose/waypoint constraints at frame index "+std::to_string(frame));
        values[i]=value;mask[i]=1;any=true;
    }
    kimodo_conditioning view() const {
        return {sizeof(kimodo_conditioning),frames,joints,values.data(),mask.data(),0.f};
    }
};
inline Conditioning make_conditioning(const SkeletonDef &s,unsigned frames,
    const std::vector<PoseCapture> &poses, const std::vector<Waypoint> &waypoints, bool dense_path=false) {
    if(frames<2 || frames>10000) throw std::runtime_error("Frame count must be between 2 and 10000");
    Conditioning result;result.frames=frames;result.joints=static_cast<unsigned>(s.joint_count());
    result.values.resize(static_cast<size_t>(frames)*(9+12*result.joints));result.mask.resize(result.values.size());
    std::map<int,Waypoint> path;
    for(const auto &w:waypoints) {
        if(!path.emplace(w.frame,w).second) throw std::runtime_error("Duplicate waypoint frame");
        result.write(w.frame,0,w.x);result.write(w.frame,2,w.z);
        if(w.has_heading) {result.write(w.frame,3,std::cos(w.heading));result.write(w.frame,4,std::sin(w.heading));}
    }
    if(dense_path && !path.empty()) {
        if(path.begin()->first!=0 || path.rbegin()->first!=static_cast<int>(frames)-1)
            throw std::runtime_error("Dense path requires waypoints at the first and last clip frames");
        for(auto a=path.begin(),b=std::next(a);b!=path.end();++a,++b)
            for(int f=a->first+1;f<b->first;++f) {
                const float t=float(f-a->first)/float(b->first-a->first);
                result.write(f,0,a->second.x*(1-t)+b->second.x*t);
                result.write(f,2,a->second.z*(1-t)+b->second.z*t);
                if(a->second.has_heading && b->second.has_heading) {
                    const float angle=a->second.heading+t*std::remainder(b->second.heading-a->second.heading,2.f*3.14159265358979323846f);
                    result.write(f,3,std::cos(angle));result.write(f,4,std::sin(angle));
                }
            }
    }
    for(const auto &p:poses) {
        const size_t J=s.joint_count();
        if(p.positions.size()!=J || p.selected.size()!=J || (p.rotations && p.transforms.size()!=J))
            throw std::runtime_error("Pose does not match selected skeleton");
        const Vec3 root=p.positions[0];
        result.write(p.frame,0,root.x);result.write(p.frame,1,root.y);result.write(p.frame,2,root.z);
        const size_t right=J==30?26:J==22?2:8, left=J==30?22:1;
        const auto r=p.positions[right],l=p.positions[left];
        if(std::hypot(r.z-l.z,r.x-l.x)<1.e-6f) throw std::runtime_error("Cannot derive heading from coincident hips");
        const float angle=std::atan2(r.z-l.z,-(r.x-l.x));
        result.write(p.frame,3,std::cos(angle));result.write(p.frame,4,std::sin(angle));
        for(size_t j=0;j<J;++j) if(p.selected[j]) {
            result.write(p.frame,5+j*3,p.positions[j].x-root.x);
            result.write(p.frame,6+j*3,p.positions[j].y);
            result.write(p.frame,7+j*3,p.positions[j].z-root.z);
            if(p.rotations) for(size_t d=0;d<6;++d) result.write(p.frame,5+3*J+j*6+d,p.transforms[j][d]);
        }
    }
    return result;
}
} // namespace kimodo_houdini
