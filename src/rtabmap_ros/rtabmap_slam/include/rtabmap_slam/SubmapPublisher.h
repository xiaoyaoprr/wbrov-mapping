#ifndef WBROV_SUBMAP_PUBLISHER_H
#define WBROV_SUBMAP_PUBLISHER_H
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap_msgs/msg/map_graph.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>
#include <iomanip>
#include <sstream>
#include <tuple>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace rtabmap_slam {
// Visualization snapshots only. This class never registers a map, adds a link,
// supplies odometry, publishes TF or touches the primary MapsManager cache.
// Call refresh only on Core's serialized processing lane, before any reset.
class SubmapPublisher {
    struct Entry {
        int mapId=0,componentId=0;unsigned long long epoch=0;
        bool registered=false,active=false,truncated=false;
        std::string frame,key,cloudFrame;
        std::map<int,rtabmap::Transform> poses,renderedPoses;
        std::map<int,long long> stamps;
        std::multimap<int,rtabmap::Link> links;
        std::map<std::tuple<int,int,int>,pcl::PointXYZRGB> voxels;
        rclcpp::Publisher<rtabmap_msgs::msg::MapGraph>::SharedPtr graph;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud;
    };
public:
    explicit SubmapPublisher(rclcpp::Node & node):node_(node) {
        registry_=node_.create_publisher<std_msgs::msg::String>("/wbrov/submaps/registry",rclcpp::QoS(1).reliable().transient_local());
    }
    void heartbeat() {
        std_msgs::msg::String msg;{std::lock_guard<std::mutex> lock(snapshotMutex_);msg.data=lastSnapshot_;}
        if(!msg.data.empty())registry_->publish(msg);
    }
    void refresh(rtabmap::Rtabmap & core,const std::string & session,const std::string & mainFrame,
                 const rclcpp::Time & stamp,bool localHealthy) {
        if(session.size()!=64 || session.find_first_not_of("0123456789abcdef")!=std::string::npos ||
           mainFrame.empty() || mainFrame.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_/")!=std::string::npos)return;
        if(session_!=session) {clear(stamp);session_=session;revision_=0;}
        mainFrame_=mainFrame;
        std::set<std::string> refreshed;
        for(auto & row:entries_)row.second.active=false;
        if(core.isCurrentMapFrameRegistered()) {
            // Only Core's already-committed connected graph supplies main poses.
            std::map<std::pair<int,unsigned long long>,std::map<int,rtabmap::Transform>> groups;
            for(const auto & row:core.getLocalOptimizedPoses()) {
                rtabmap::AcceptedImageReceipt receipt;
                if(row.first>0 && core.getAcceptedImageReceipt(row.first,receipt))
                    groups[{receipt.mapId,receipt.resetEpoch}][row.first]=row.second.clone();
            }
            for(const auto & group:groups) {
                Entry & entry=findOrCreate(group.first.first,group.first.second,group.second.begin()->first);
                // RViz fixed to the old local frame cannot transform a clear
                // message already relabeled as map. Clear in the ORIGINAL
                // frame first, using a valid zero-point cloud with XYZ fields.
                if(entry.frame!=mainFrame_)clearCloud(entry,stamp);
                entry.poses=group.second;entry.registered=true;entry.frame=mainFrame_;
                entry.active=entry.epoch==core.getResetEpoch();entry.links.clear();
                for(const auto & link:core.getLocalConstraints())
                    if(entry.poses.count(link.second.from()) && entry.poses.count(link.second.to()))entry.links.insert(link);
                fillStamps(core,entry);publishGraph(entry,stamp);clearCloud(entry,stamp);
                refreshed.insert(entry.key);
            }
        } else {
            std::vector<rtabmap::Rtabmap::LocalSubmapSnapshot> snapshots;std::string reason;
            if(!core.getUnregisteredLocalSubmaps(snapshots,reason)) {
                RCLCPP_WARN(node_.get_logger(),"Local submap snapshot unavailable: %s",reason.c_str());
            }
            for(const auto & snapshot:snapshots) {
                if(snapshot.poses.empty() || snapshot.registeredToMain)continue;
                Entry & entry=findOrCreate(snapshot.mapId,snapshot.resetEpoch,snapshot.componentId);
                entry.poses=snapshot.poses;entry.links=snapshot.links;entry.registered=false;
                entry.active=snapshot.resetEpoch==core.getResetEpoch();
                fillStamps(core,entry);publishGraph(entry,stamp);publishCloud(core,entry,stamp);
                refreshed.insert(entry.key);
            }
        }
        retireCoveredHistory(core,refreshed,stamp);
        for(auto & row:entries_)if(!row.second.active) {row.second.voxels.clear();row.second.renderedPoses.clear();}
        ++revision_;std_msgs::msg::String msg;msg.data=serialize(localHealthy,stamp.nanoseconds());
        {std::lock_guard<std::mutex> lock(snapshotMutex_);lastSnapshot_=msg.data;}registry_->publish(msg);
    }
private:
    Entry & findOrCreate(int mapId,unsigned long long epoch,int componentId) {
        for(auto & row:entries_)if(row.second.mapId==mapId && row.second.epoch==epoch &&
                (row.second.componentId==componentId || row.second.poses.count(componentId)))return row.second;
        std::string key="m"+std::to_string(mapId)+"_e"+std::to_string(epoch)+"_c"+std::to_string(componentId);
        auto & e=entries_[key];e.mapId=mapId;e.epoch=epoch;e.componentId=componentId;e.key=key;
        e.frame="wbrov_submap_"+session_.substr(0,16)+"_"+key;
        e.graph=node_.create_publisher<rtabmap_msgs::msg::MapGraph>("/wbrov/submaps/"+key+"/graph",rclcpp::QoS(1).reliable().transient_local());
        e.cloud=node_.create_publisher<sensor_msgs::msg::PointCloud2>("/wbrov/submaps/"+key+"/cloud",rclcpp::QoS(1).reliable().transient_local());
        return e;
    }
    static void fillStamps(rtabmap::Rtabmap & core,Entry & e) {
        e.stamps.clear();
        for(const auto & pose:e.poses) {rtabmap::AcceptedImageReceipt receipt;
            if(core.getAcceptedImageReceipt(pose.first,receipt) && receipt.mapId==e.mapId && receipt.resetEpoch==e.epoch)
                e.stamps[pose.first]=receipt.input.headerStampNs;
        }
    }
    void publishGraph(Entry & e,const rclcpp::Time & stamp) {
        rtabmap_msgs::msg::MapGraph msg;msg.header.stamp=stamp;msg.header.frame_id=e.frame;
        rtabmap_conversions::mapGraphToROS(e.poses,e.links,rtabmap::Transform::getIdentity(),msg);e.graph->publish(msg);
    }
    static void clearCloud(Entry & e,const rclcpp::Time & stamp) {
        pcl::PointCloud<pcl::PointXYZRGB> empty;
        sensor_msgs::msg::PointCloud2 msg;pcl::toROSMsg(empty,msg);
        // Keep the clear latched in the frame of the last actual cloud. A
        // subsequent map-frame clear could otherwise overwrite it at depth=1
        // before a local-frame RViz consumer receives the old-frame clear.
        msg.header.stamp=stamp;msg.header.frame_id=e.cloudFrame.empty()?e.frame:e.cloudFrame;e.cloud->publish(msg);
        e.voxels.clear();e.renderedPoses.clear();e.truncated=false;
    }
    void retireCoveredHistory(rtabmap::Rtabmap & core,const std::set<std::string> & refreshed,
                              const rclcpp::Time & stamp) {
        // Latest Core snapshots own their actual node IDs. This does not merge
        // poses or infer a transform between independently retained submaps.
        std::map<std::pair<int,unsigned long long>,std::set<int>> covered;
        for(const auto & key:refreshed) {
            const auto & entry=entries_.at(key);
            auto & ids=covered[{entry.mapId,entry.epoch}];
            for(const auto & pose:entry.poses)ids.insert(pose.first);
        }
        for(auto & row:entries_) {
            auto & old=row.second;
            if(refreshed.count(row.first) || old.poses.empty())continue;
            const auto owned=covered.find({old.mapId,old.epoch});
            if(owned==covered.end())continue; // Preserve truly separate epochs/maps.
            bool changed=false;
            for(auto pose=old.poses.begin();pose!=old.poses.end();) {
                if(owned->second.count(pose->first)) {
                    old.stamps.erase(pose->first);pose=old.poses.erase(pose);changed=true;
                } else ++pose;
            }
            if(!changed)continue; // Preserve nonoverlapping historical components.
            for(auto link=old.links.begin();link!=old.links.end();) {
                if(!old.poses.count(link->second.from()) || !old.poses.count(link->second.to()))link=old.links.erase(link);
                else ++link;
            }
            // Old origin/frame remains unchanged for any surviving nodes.
            // Rebuild the cloud from their true stored local poses, not from
            // a guessed main transform or a union with the newer component.
            clearCloud(old,stamp);publishGraph(old,stamp);
            if(!old.poses.empty() && !old.registered)publishCloud(core,old,stamp);
            // Empty entries retain their publishers as zero-point tombstones.
            // They are omitted from the registry below, but a late RViz
            // subscriber can still receive the valid latched clear message.
        }
    }
    void clear(const rclcpp::Time & stamp) {for(auto & row:entries_)clearCloud(row.second,stamp);entries_.clear();}
    void publishCloud(rtabmap::Rtabmap & core,Entry & e,const rclcpp::Time & stamp) {
        if(!core.getMemory())return;
        bool changed=false;
        for(const auto & old:e.renderedPoses) {
            auto now=e.poses.find(old.first);
            if(now==e.poses.end() || now->second.getDistance(old.second)>1e-6 || now->second.getAngle(old.second)>1e-6) {changed=true;break;}
        }
        if(changed) {e.voxels.clear();e.renderedPoses.clear();e.truncated=false;}
        for(const auto & row:e.poses) {
            if(e.renderedPoses.count(row.first))continue;
            auto data=core.getMemory()->getNodeData(row.first,true,false,false,false);data.uncompressData();
            if(data.cameraModels().size()!=1 || data.depthRaw().empty() || data.imageRaw().empty())continue;
            const auto & camera=data.cameraModels().front();const auto & depth=data.depthRaw();const auto & rgb=data.imageRaw();
            if(!camera.isValidForProjection() || camera.localTransform().isNull() || depth.size()!=rgb.size() ||
               depth.cols!=camera.imageWidth() || depth.rows!=camera.imageHeight() ||
               (depth.type()!=CV_16UC1 && depth.type()!=CV_32FC1) || (rgb.type()!=CV_8UC1 && rgb.type()!=CV_8UC3))continue;
            const auto transform=row.second*camera.localTransform();
            for(int y=0;y<depth.rows;y+=4)for(int x=0;x<depth.cols;x+=4) {
                const double z=depth.type()==CV_16UC1?double(depth.at<unsigned short>(y,x))*.001:double(depth.at<float>(y,x));
                if(!std::isfinite(z) || z<.2 || z>.8)continue;
                Eigen::Vector3f q=transform.toEigen3f()*Eigen::Vector3f(float((x-camera.cx())*z/camera.fx()),float((y-camera.cy())*z/camera.fy()),float(z));
                if(!q.allFinite() || q.cwiseAbs().maxCoeff()>10000)continue;
                const auto key=std::make_tuple(int(std::floor(q.x()/.03f)),int(std::floor(q.y()/.03f)),int(std::floor(q.z()/.03f)));
                if(e.voxels.size()>=200000 && !e.voxels.count(key)) {e.truncated=true;continue;}
                pcl::PointXYZRGB point;point.x=q.x();point.y=q.y();point.z=q.z();
                if(rgb.channels()==3) {auto c=rgb.at<cv::Vec3b>(y,x);point.b=c[0];point.g=c[1];point.r=c[2];}
                else point.r=point.g=point.b=rgb.at<unsigned char>(y,x);
                e.voxels[key]=point;
            }
            e.renderedPoses[row.first]=row.second.clone();
        }
        pcl::PointCloud<pcl::PointXYZRGB> cloud;cloud.reserve(e.voxels.size());for(const auto & row:e.voxels)cloud.push_back(row.second);
        sensor_msgs::msg::PointCloud2 msg;pcl::toROSMsg(cloud,msg);msg.header.stamp=stamp;msg.header.frame_id=e.frame;e.cloudFrame=e.frame;e.cloud->publish(msg);
    }
    std::string serialize(bool healthy,long long stampNs) const {
        std::ostringstream out;out<<std::setprecision(17)<<"{\"schema\":\"wbrov.submaps.v1\",\"session_short_hash\":\""<<session_.substr(0,16)
          <<"\",\"session_token_sha256\":\""<<session_<<"\",\"revision\":"<<revision_<<",\"source_stamp_ns\":"<<stampNs
          <<",\"main_frame_id\":\""<<mainFrame_<<"\",\"coarse_gap_translation_available\":false,\"submaps\":[";
        bool first=true;for(const auto & row:entries_) {const auto & e=row.second;if(e.poses.empty())continue;if(!first)out<<",";first=false;
            out<<"{\"key\":\""<<e.key<<"\",\"map_id\":"<<e.mapId<<",\"reset_epoch\":"<<e.epoch<<",\"component_id\":"<<e.componentId
               <<",\"frame_id\":\""<<e.frame<<"\",\"registered\":"<<(e.registered?"true":"false")<<",\"active\":"<<(e.active?"true":"false")
               <<",\"status\":\""<<(e.registered?"registered_main":e.active&&healthy?"local_tracking":"local_paused")
               <<"\",\"graph_topic\":\"/wbrov/submaps/"<<e.key<<"/graph\",\"cloud_topic\":\"/wbrov/submaps/"<<e.key
               <<"/cloud\",\"display_topic\":\"/wbrov/submaps/"<<e.key<<"/display\",\"cloud_truncated\":"<<(e.truncated?"true":"false")<<",\"nodes\":[";
            bool firstNode=true;for(const auto & pose:e.poses) {auto t=e.stamps.find(pose.first);if(t==e.stamps.end())continue;
                if(!firstNode) { out<<","; }
                firstNode=false;const auto q=pose.second.getQuaterniond();
                out<<"{\"id\":"<<pose.first<<",\"source_stamp_ns\":"<<t->second<<",\"translation\":["<<pose.second.x()<<","<<pose.second.y()<<","<<pose.second.z()
                   <<"],\"quaternion_xyzw\":["<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w()<<"]}";
            }out<<"]}";
        }out<<"]}";return out.str();
    }
    rclcpp::Node & node_;std::string session_,mainFrame_;unsigned long long revision_=0;
    std::map<std::string,Entry> entries_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr registry_;
    std::mutex snapshotMutex_;std::string lastSnapshot_;
};
}
#endif
