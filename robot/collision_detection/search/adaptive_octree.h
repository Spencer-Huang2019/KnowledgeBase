#ifndef COLLISION_DETECTION_SEARCH_ADAPTIVE_OCTREE_H
#define COLLISION_DETECTION_SEARCH_ADAPTIVE_OCTREE_H
#include <pcl/point_types.h>
#include <set>
#include <vector>
#include <unordered_map>

namespace knowledge_base {
namespace collison_detection {
namespace search {
using Point = pcl::PointXYZ;

CLASS_PTR_FORWARD(OctreeNode);
class OctreeNode
{
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;
    bool occupied = false;
    pcl::PointCloud<Point> cloud;
    OctreeNodePtr children[8];

    OctreeNode(float x1, float x2, float y1, float y2, float z1, float z2, Point point):
        min_x(x1), max_x(x2), min_y(y1), max_y(y2), min_z(z1), max_z(z2)
    {
        cloud.points.push_back(point);
    }

    bool is_leaf()
    {
        return children[0] == nullptr;
    }

    float voxel_size()
    {
        return max_x - min_x;
    }

    int num_points()
    {
        return cloud.size();
    }

    void insert(Point p)
    {
        cloud.points.push_back(p);
    }

    void clear()
    {
        cloud.points.clear();
    }
};


CLASS_PTR_FORWARD(AdaptiveOctree);
class AdaptiveOctree
{
public:
    AdaptiveOctree(float x1, float x2, float y1, float y2, float z1, float z2)
    {
        root_ptr_ = new OctreeNode(x1, x2, y1, y2, z1, z2);
    }

    ~AdaptiveOctree(){}

    void construct_octree(pcl::PointCloud<Point>::Ptr cloud_ptr);

    void get_neighbors(OctreeNodePtr node);

    std::unordered_map<int, OctreeNodePtr> get_octree_map() { return octree_map_; }


private:

    void insert_point(OctreeNodePtr node, Point& point);

    int get_octant(OctreeNodePtr node, Point point);

    void split_node(OctreeNodePtr node);

private:

    OctreeNodePtr root_ptr_;
    std::unordered_map<int, OctreeNodePtr> octree_map_;

    float resolution_ = 0.15;
    int global_index_ = 0;
};

} 
}
}

#endif