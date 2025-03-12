/*
实现：
1. 确定一个节点的数据结构，要求存储空间位置信息、代价信息、父节点指针信息
    补充：每个节点需要存储其子节点信息，不然 rewiring 的时候就连不上了。。。
2. 随机采样函数的实现，需要给定一个空间范围，在这个范围内进行随机采样（基于采样的算法一般都需要这个步骤，可以实现单点采样，和多点采样供不同算法使用）
3. 计算一个点的近邻点，使用 KDTree 搜索？其实直接树的遍历就行了吧。
4. 碰撞检测，要实现两方面的碰撞检测（不过这个可能还得分不同的碰撞检测方法，在碰撞检测那部分的学习里具体实现吧，这里就先不实现了）
5. 实现选择父节点的函数（应该做碰撞检测）
6. 实现 rewiring 的函数（应该做碰撞检测）

先用函数名把主流程串起来，确定每个函数的输入输出内容，也就是先写好了 plan 函数，然后再逐个实现其他的函数
*/

#include <algorithm>
#include <numeric>
#include <random>
#include "rrt_star.h"

namespace knowledge_base {
namespace robot_planning {
namespace sampling_based {

    bool RRT_star::plan(Path& path)
    {
        Node* cur_node = start_node_;
        for (int i = 0; i < max_iter; ++i)
        {
            // sampling
            float* random_s = sampling_state();

            // find nearest node in tree
            Node* nearest_node = start_node_;
            float last_cost = INT_MAX;
            find_nearest_node(random_s, start_node_, nearest_node, last_cost);

            // get new and do collision check
            float* new_s = get_new_state(random_s, nearest_node);
            if (!collide(new_s) && !collide(random_s, new_s))
            {
                // find neighbors in the tree
                std::vector<Node*> neighbors_node;
                Node* root = start_node_;
                find_neighbors(new_s, root, neighbors_node);

                Node* new_node = new Node(new_s[0], new_s[1], new_s[2]);

                // find optimal parent node
                Node* parent_node = find_parent(new_node, neighbors_node);
                parent_node->children_node.insert(new_node);
                new_node->parent_node = parent_node;
                new_node->cost = cost(new_node, parent_node);

                // rewiring
                rewiring(new_node, neighbors_node);

                if (is_finished(new_node))
                {   
                    // get path
                    Path path = get_path();

                    return true;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                continue;
            }
        }

        return false;
    }

    float* RRT_star::sampling_state()
    {
        std::random_device rd;
        std::mt19937 gen(rd());

        std::uniform_real_distribution<> dist_x(x_range_[0], x_range_[1]);
        std::uniform_real_distribution<> dist_y(y_range_[0], y_range_[1]);
        std::uniform_real_distribution<> dist_z(z_range_[0], z_range_[1]);

        float random_pos[3] = {dist_x(gen), dist_y(gen), dist_z(gen)};

        return random_pos;
    }

    void RRT_star::find_nearest_node(float* pos, Node* root, Node* nearest_node, float last_cost)
    {
        if (root != nullptr)
        {
            float dist = distance(pos, root);
            if (dist < last_cost)
            {
                nearest_node = root;
                last_cost = dist;
            }

            for (auto& child: root->children_node)
            {
                find_nearest_node(pos, child, nearest_node, last_cost);
            }
        }
    }

    float* RRT_star::get_new_state(float* pos, Node* nearest_node)
    {
        float vec[3] = {nearest_node->x - pos[0], nearest_node->y - pos[1], nearest_node->z - pos[2]};
        float factor = step_ / std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
        float new_pos[3] = {factor * vec[0], factor * vec[1], factor * vec[2]};

        return new_pos;
    }

    bool RRT_star::collide(float* pos)
    {
        return false;
    }

    bool RRT_star::collide(float* pos1, float* pos2)
    {
        return false;
    }

    bool RRT_star::collide(Node* node1, Node* node2)
    {
        float pos1[3] = {node1->x, node1->y, node1->z};
        float pos2[3] = {node2->x, node2->y, node2->z};

        return collide(pos1, pos2);
    }

    void RRT_star::find_neighbors(float* pos, Node* root, std::vector<Node*>& neighbors_node)
    {
        if (root != nullptr)
        {
            if (distance(pos, root) < radius_)
            {
                neighbors_node.push_back(root);
            }
            for (auto& child: root->children_node)
            {
                find_neighbors(pos, child, neighbors_node);
            }
        }
    }

    Node* RRT_star::find_parent(Node* new_node, std::vector<Node*>& neighbors_node)
    {
		// 这个 lambda 函数，内部调用了外部的 new_node 变量，而且还调用了外边的函数
		// 这种两种情况都需要通过捕捉器捕捉外部的变量和函数。捕捉一个可以使用 [new_node]\[this]
		// 捕获多个可以直接使用 [&] 捕捉所有的外部变量，包括 this 指针
        auto condition = [new_node, this](Node* node1, Node* node2) // capture no need to declear data type
        {
            float dist1 = cost(new_node, node1) + node1->cost;
            float dist2 = cost(new_node, node2) + node2->cost;

            return dist1 < dist2;
        }; // 分号别漏了。。。
        
        // #include <algorithm>
        std::sort(neighbors_node.begin(), neighbors_node.end(), condition);

        for (auto& neighbor: neighbors_node)
        {
            if (!collide(new_node, neighbor))
            {
                return neighbor;
            }
        }
    }
	
	// 画完图之后发现，这一步的计算好像是有点不对了。。。
    void RRT_star::rewiring(Node* new_node, std::vector<Node*> neighbors_node)
    {
		/* ##### 原始版本 #####
		* 把 new_node 的 parent 都给修改了，通过画图可以看出，这么做，上一步的 find_parent 就完全没有意义了
		* 实际上，这一步要做的仅仅是修改 neighbor 的父节点，new_node 父节点是固定的
        for (auto& neighbor: neighbors_node)
        {
            Node* parent = neighbor->parent_node;
            float c_nn = cost(new_node, neighbor);
            float cost_newp = cost(new_node, parent); 
            float cost_neip = cost(neighbor, parent);
            if (cost_newp < cost_neip && !collide(new_node, parent))
            {
                parent->children_node.erase(neighbor);
                parent->children_node.insert(new_node);

                new_node->parent_node = parent;
                new_node->cost = parent->cost + cost_newp;
                new_node->children_node.insert(neighbor);

                neighbor->parent_node = new_node;
                neighbor->cost = new_node->cost + c_nn;
            }
        }
		*/
		
		float new_node_cost = new_node->cost;
		for (auto& neighbor: neighbors_node)
        {
            float neighbor_cost = neighbor->cost; 
			float c_nn = cost(new_node, neighbor);
            float new_neig_cost = new_node_cost + c_nn;
            if (new_neig_cost < neighbor_cost && !collide(new_node, neighbor))
            {
				Node* parent = neighbor->parent_node;
            	
                parent->children_node.erase(neighbor);

                new_node->children_node.insert(neighbor);

                neighbor->parent_node = new_node;
                neighbor->cost = new_neig_cost;
            }
        }
    }

    bool RRT_star::is_finished(Node* new_node)
    {
        float dist = cost(new_node, end_node_);
        if (dist < step_)
        {
            new_node->children_node.insert(end_node_);
            end_node_->parent_node = new_node;
            end_node_->cost = dist;
            return true;
        }

        return false;
    }

    Path RRT_star::get_path()
    {
        Node* node = end_node_;
        Path p;
        // std::vector<Point> pts = &p.value; 这么写是错误的，容器的引用赋值和其他基本数据类型的应用赋值不同
		std::vector<Point>& pts = p.value;
        p.cost = end_node_->cost;
        while (node != nullptr)
        {
            Point p;
            p.x = node->x;
            p.y = node->y;
            p.z = node->z;

            pts.push_back(p);

            node = node->parent_node;
        }

        return p;
    }

    float RRT_star::distance(float* pos, Node* node)
    {
        float square_x = (pos[0] - node->x) * (pos[0] - node->x);
        float square_y = (pos[1] - node->y) * (pos[1] - node->y);
        float square_z = (pos[2] - node->z) * (pos[2] - node->z);

        return std::sqrt(square_x + square_y + square_z);
    }

    float RRT_star::cost(Node* node1, Node* node2)
    {
        float square_x = (node1->x - node2->x) * (node1->x - node2->x);
        float square_y = (node1->y - node2->y) * (node1->y - node2->y);
        float square_z = (node1->z - node2->z) * (node1->z - node2->z);

        return std::sqrt(square_x + square_y + square_z);
    }

} // namespace sampling_base
} // namespace robot_planning
} // namespace knowledge_base