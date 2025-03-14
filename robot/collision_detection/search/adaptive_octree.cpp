#include "adaptive_octree.h"

namespace knowledge_base {
namespace collison_detection {
namespace search {

void AdaptiveOctree::construct_octree(pcl::PointCloud<Point>::Ptr cloud_ptr)
{
    for (auto& point : cloud_ptr->points)
    {
        insert_point(root_ptr_, point);
    }
}

void AdaptiveOctree::get_neighbors(OctreeNodePtr node)
{
    
}

void AdaptiveOctree::insert_point(OctreeNodePtr node, Point& point);
{
    if (node->is_leaf())
    {
        float voxel_size = node->voxel_size();
        node->insert(point);
        int num_points = node->num_points();
        if (voxel_size > resolution && node->occupied)
        {
            split_node(node);
        }
        else
        {
            node->occupied = true;
        }
    }
    else
    {
        int index = get_octant(node, point);
        insert_point(node->children[index], point);
    }
}

int AdaptiveOctree::get_octant(OctreeNodePtr node, Point point)
{
    int index = 0;
    float mid_x = (node->min_x + node->max_x) / 2;
    float mid_y = (node->min_y + node->max_y) / 2;
    float mid_z = (node->min_z + node->max_z) / 2;

    if (point.x >= mid_x) index |= 1;
    if (point.y >= mid_y) index |= 2;
    if (point.z >= mid_z) index |= 4;

    return index;
}

void AdaptiveOctree::split_node(OctreeNodePtr node)
{
    float mid_x = (node->min_x + node->max_x) / 2;
    float mid_y = (node->min_y + node->max_y) / 2;
    float mid_z = (node->min_z + node->max_z) / 2;

    std::vector<std::vector<float>> tmp(3, std::vector<float>());
    tmp[0] = {min_x, mid_x, max_x};
    tmp[1] = {min_y, mid_y, max_y};
    tmp[2] = {min_z, mid_z, max_z};

    children[0] = new OctreeNode(tmp[0][0], tmp[0][1], tmp[1][0], tmp[1][1], tmp[2][0], tmp[2][1]);
    children[1] = new OctreeNode(tmp[0][1], tmp[0][2], tmp[1][0], tmp[1][1], tmp[2][0], tmp[2][1]);
    children[2] = new OctreeNode(tmp[0][0], tmp[0][1], tmp[1][1], tmp[1][2], tmp[2][0], tmp[2][1]);
    children[3] = new OctreeNode(tmp[0][1], tmp[0][2], tmp[1][1], tmp[1][2], tmp[2][0], tmp[2][1]);
    children[4] = new OctreeNode(tmp[0][0], tmp[0][1], tmp[1][0], tmp[1][1], tmp[2][1], tmp[2][2]);
    children[5] = new OctreeNode(tmp[0][1], tmp[0][2], tmp[1][0], tmp[1][1], tmp[2][1], tmp[2][2]);
    children[6] = new OctreeNode(tmp[0][0], tmp[0][1], tmp[1][1], tmp[1][2], tmp[2][1], tmp[2][2]);
    children[7] = new OctreeNode(tmp[0][1], tmp[0][2], tmp[1][1], tmp[1][2], tmp[2][1], tmp[2][2]);


    for (auto& point : node->cloud.points)
    {
        int index = get_octant(node, point);
        children[index]->insert(point);
        children[index]->occupied = true;
    }

    node->clear();
}

}
}
}