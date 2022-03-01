#ifndef WP_CORE_H
#define WP_CORE_H

// C++ includes
#include <string>
#include <memory>
#include <fstream>
#include <sstream>
// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nmea_msgs/Sentence.h>
#include "geo_pos_conv.h"
using namespace std;

namespace way_point
{

std::vector<std::string> split(const std::string &string);


//对于四叉树节点的定义
class Node {
public:
    geometry_msgs::Point val;
    bool isLeaf;
	Node* R[4];
    Node* parent;

    Node() {}
//节点的构造函数
    Node(geometry_msgs::Point _val, bool _isLeaf, Node* _topLeft, Node* _topRight, Node* _bottomLeft, Node* _bottomRight, Node* _parent) {
        val = _val;
        isLeaf = _isLeaf;
        R[1] = _topLeft;
        R[0] = _topRight;
        R[2] = _bottomLeft;
        R[3] = _bottomRight;
        parent = _parent;
    }
};

class Gnss_Zmap {
private:

    void clear()
    {
        clear(root);
    }

    void clear(Node* &p)
    {
        if (p == nullptr) return;
        if (p->R[0]) clear(p->R[0]);
        if (p->R[1]) clear(p->R[1]);
        if (p->R[2]) clear(p->R[2]);
        if (p->R[3]) clear(p->R[3]);
        free(p);
        p = nullptr;
    }


    Node* root;
    std::vector<geometry_msgs::Point> point_list;

    geometry_msgs::Point string2point(std::string point_line);

    geometry_msgs::Point search(std::vector<geometry_msgs::Point> point_list, geometry_msgs::Point point);

    
    int Compare(const Node* node, const geometry_msgs::Point& pos)
    {
        if (pos.x == node->val.x && pos.y == node->val.y) return 0;   //相同
        if (pos.x >= node->val.x && pos.y>node->val.y)  return 1;     //右上
        if (pos.x<node->val.x  && pos.y >= node->val.y) return 2;     //左上
        if (pos.x <= node->val.x && pos.y<node->val.y)  return 3;     //左下
        if (pos.x>node->val.x  && pos.y <= node->val.y) return 4;     //右下
        return -1;
    }

public:

    Gnss_Zmap();

    ~Gnss_Zmap();
    
    void run();
    
    Node* get_tree_from_file(std::string filename);

    std::vector<geometry_msgs::Point> get_vector_from_file(std::string filename);
    
    geometry_msgs::Point gnss_remapping(geometry_msgs::Point pose);
    

};

}  
#endif  // WP_CORE_H

