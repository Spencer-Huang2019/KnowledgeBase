#ifndef KNOWLEDGE_BASE_COMMON_STRUCTS_H
#define KNOWLEDGE_BASE_COMMON_STRUCTS_H

#include <vector>
#include <set>
#include <climits>

namespace knowledge_base {
namespace common {

struct Point
{
    float x;
    float y;
    float z;
};

struct Node
{
    float x;
    float y;
    float z;

    float cost = 0.0;
    Node* parent_node = nullptr;
    std::set<Node*> children_node;

    Node(): 
        x(0.0), y(0.0), z(0.0){}

    Node(float var_x, float var_y, float var_z): 
        x(var_x), y(var_y), z(var_z){}

    Node(float var_x, float var_y, float var_z, float var_cost): 
        x(var_x), y(var_y), z(var_z), cost(var_cost){}

};

struct Path
{
    std::vector<Point> value;
    float cost = 0.0;
};

} // namespace common
} // namespace knowledge_base

#endif