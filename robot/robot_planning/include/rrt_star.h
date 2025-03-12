#ifndef ROBOT_PLANNING_RRTSTART_H
#define ROBOT_PLANNING_RRTSTART_H

#include "structs.h"

namespace knowledge_base {
namespace robot_planning {
namespace sampling_based {

using namespace common;

class RRT_star
{

public:
    RRT_star(float* start, float* end)
    {
        start_node_ = new Node(start[0], start[1], start[2]);
        end_node_ = new Node(end[0], end[1], end[2]);
    }

    ~RRT_star()
    {
        delete start_node_;
        delete end_node_;
    }

    /*
    
    */
    bool plan(Path& path);

private:
    /*
    
    */
    float* sampling_state();

    void find_nearest_node(float* pos, Node* root, Node* nearest_node, float last_cost);

    float* get_new_state(float* pos, Node* nearest_node);

    bool collide(float* pos);

    bool collide(float* pos1, float* pos2);

    bool collide(Node* node1, Node* node2);

    void find_neighbors(float* pos, Node* root,  std::vector<Node*>& neighbors_node);

    Node* find_parent(Node* new_node, std::vector<Node*>& neighbors_node);

    void rewiring(Node* new_node, std::vector<Node*> neighbors_node);

    bool is_finished(Node* new_node);

    Path get_path();

    float distance(float* pos, Node* node);

    float cost(Node* node1, Node* node2);

private:
    Node* start_node_;
    Node* end_node_;;

    float x_range_[2] = {-50, 50};
    float y_range_[2] = {-50, 50};
    float z_range_[2] = {-50, 50};

    float step_ = 0.1;
    int max_iter = 100;
    float radius_ = 0.3;
    
};

} // namespace sampling_base
} // namespace robot_planning
} // namespace knowledge_base

#endif