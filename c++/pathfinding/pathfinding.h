#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <algorithm>
#include <cassert>
#include <iostream>
#include <list>
#include <map>
#include <unordered_map>
#include <utility>
#include <set>
#include <deque>
#include <string>
#include <vector>
#include <tuple>

#include "utils.h"

// namespace std {
//     template<typename X, typename Y>
//     struct hash<std::pair<X, Y>> {
//         std::size_t operator()(const std::pair<X, Y> &pair) const {
//             return std::hash<X>()(pair.first) != std::hash<Y>()(pair.second) ? std::hash<X>()(pair.first) ^ std::hash<Y>()(pair.second): std::hash<X>()(pair.first);
//         }
//     };
// }

class Node
{
private:
    double la_; // latitude
    double lo_; // longtitude
    Node *parent;  // parent node pointer
    double cost_;  // cost from current node to start node
    double weight_;  // weight for avoid landmark selection
    double size_;  // size for avoid landmark selection

public:
    std::vector<Node *> children{};
    std::unordered_map<int, double> dists2l;// distance from all landmarks to node
    Node(double la, double lo);
    ~Node();

    double get_la() const;
    double get_lo() const;
    void print_coords() const;
    double get_cost()  const;
    void set_cost(double cost);
    double get_weight()  const;
    void set_weight(double weight);
    double get_size()  const;
    void set_size(double weight);
    void set_parent(Node *ptr);
    Node *get_parent() const;
    void set_dists2l(const std::vector<std::vector<double>> &landmarks);
    void reset();

    bool operator==(const Node &rhs) const;
    bool operator!=(const Node &rhs) const;
};


/**
 * @brief search tree
 * 
 */

class Search_tree{
private: 
    Node* root_ptr;
public:
    Search_tree();
    ~Search_tree();
    
    Node* get_root_ptr() const;
    void set_root_ptr(Node* ptr);
    void reset_root_ptr();
    /**
     * @brief 
     *  add a child node to the parent node
     */
    void add_edge(Node *parent_ptr, Node *child_ptr);

    /**
     * @brief 
     *  remove a child node to the parent node
     */
    void remove_edge(Node *parent_ptr, Node *child_ptr);
    
    /**
     * @brief 
     *  pretty print a search tree
     */
    void print_tree(Node* root, const std::string& prefix, bool isLeft=false);
};


/**
 * \brief Graph algorithms
 */

class Graph
{
private:
    std::set<Node *> node_ptrs;
    
    /**
     *  adjacency_list maps every vertex ptr to the list of its neighbour ptrs in the
     * order in which they are added.
     */
    std::map<Node *, std::list<std::pair<Node *, double>>> adjacency_list;

    /// stack to store the nodes which are yet to be traversed
    std::deque<Node *> open_table;

    /// vector to store the nodes which are visited
    std::list<Node *> close_table;

    /// shortest path tree
    Search_tree t;

    /// exteneded paths during pathfinding search
    std::vector<double> ext_paths;

    /// the final path of the pathfinding result
    std::vector<double> final_path;

    /// landmarks for ALT a* (precomputed with random avoid method)
    // TODO 8 to 16 precomputed
    std::vector<std::vector<double>> landmarks = {
        {30.6357604, 114.2102454},
        {30.6206240, 114.4190201},
        {30.6201832, 114.2132492},
        {30.6551874, 114.2988177},
        {30.5827114, 114.4114266},
        {30.5211646, 114.2465313},
        {30.5005932, 114.2935017},
        {30.5751089, 114.2354420},
        {30.6550336, 114.2822843},
        {30.5017398, 114.4292775},
        {30.5617746, 114.4432224},
        {30.6385844, 114.3949592},
        {30.6433098, 114.2586681},
        {30.5226665, 114.4503951},
        {30.6474337, 114.3370946},
        {30.6018754, 114.4224124},
        {30.4942223, 114.3210146},
        {30.6049409, 114.4286292},
        {30.5996193, 114.2154722},
        {30.6291274, 114.4206168},
        {30.4979479, 114.3620929},
        {30.5701618, 114.4089203},
        {30.4986034, 114.3283489},
        {30.6567048, 114.3121701},
        {30.4910525, 114.3276655},
        {30.6371074, 114.3733771},
        {30.5027901, 114.3822579},
        {30.5656868, 114.2133197},
        {30.4929671, 114.3842194},
        {30.5059596, 114.4076078},
        {30.5529102, 114.2353832},
        {30.5824735, 114.4119657}
    };
    std::vector<int> landmark_ids;

    /**
     * @brief
     * reset nodes in the graph, open table, close table and search tree
     */
    void reset_graph();

    /**
     *  this function performs the uniform cost search traverse(Dijkstra) on graph and find the shortest path tree.
     */
    void uniform_cost_traverse(Node* root);

    /**
     * @brief precompute the distances between landmarks and nodes
     * 
     */
    void precompute_landmarks_distances();

    /**
     * @brief randomly select landmarks from nodes
     * 
     * @param cardinality  cardinality of the landmark set
     */
    void random_avoid_selection(unsigned int cardinality);

    /**
     * @brief farthest selection method for landmark selection
     * 
     * @param cardinality cardinality of the landmark set
     * @param subset_cardinality cardinality of the landmark subset
     */
    void farthest_selection(unsigned int cardinality, unsigned int subset_cardinality);

    /**
     * @brief avoid method to select landmarks
     * 
     * @param cardinality cardinality of the landmark set
     */
    void avoid_selection(unsigned int cardinality);

    /**
     * @brief calculate the weight of each node using triangular inequality lower bounds
     * 
     * @param node_ptr pointer of the node
     * @param root_ptr pointer of the root
     */
    double calc_weight(const Node* node_ptr, const Node* root_ptr);

    /**
     * @brief calculate the size of each node
     * @param node_ptr pointer of the node
     */
    double calc_size(const Node* node_ptr);

    bool landmark_flag{false};
    double add_weight(const Node* node_ptr);

    Node* find_leaf(Node* node_ptr);

    static double cost_with_euclidean_bound(const Node* node_ptr, const Node* dst_ptr){
        return node_ptr->get_cost() + euclidean_distance(node_ptr->get_la(), node_ptr->get_lo(),
                        dst_ptr->get_la(), dst_ptr->get_lo()) * 1000;
    };

    /**
     * @brief calculate the maximum triangular inequality from current node to dst node
     */
    double cost_with_max_triangular_inequality(Node* node_ptr, Node* dst_ptr, int landmark_num, bool improved);


    static std::vector<double> flatten(std::vector<std::vector<double>> const &vec)
    {
        std::vector<double> flattened;
        for (auto const &v: vec) {
            flattened.insert(flattened.end(), v.begin(), v.end());
        }
        return flattened;
    }


public:
    Graph();
    ~Graph();
    /**
     *  add_edge(u,v,bidir) is used to add an edge between node u and
     * node v by default , bidir is made true , i.e graph is
     * bidirectional . It means if edge(u,v) is added then u-->v  and
     * v-->u both edges exist.
     *
     *  to make the graph unidirectional pass the third parameter of
     * add_edge as false which will
     * the cost argument is the cost of the edge
     */
    void add_edge(Node &u, Node &v, double cost = 1, bool bidir = true);

    // /**
    //  * @brief Set the nodes coords object
    //  * 
    //  */
    // void set_nodes_coords();

    /**
     * @brief Preprocess the graph: set landmarks and precompute distances
     * 
     */
    void graph_preprocessing(unsigned int cardinality);

    /**
     *  this function performs the breadth first search pathfinding on graph and return a
     *  bool value true if the path is found, else return false.
     */
    bool breadth_first_search(Node &src, Node &dst);

    /**
     *  this function performs the depth first search pathfinding on graph and return a
     *  bool value true if the path is found, else return false.
     */
    bool depth_first_search(Node &src, Node &dst);

    /**
     *  this function performs the uniform cost search pathfinding(Dijkstra) on graph and return a
     *  bool value true if the path is found, else return false.
     */
    bool uniform_cost_search(Node &src, Node &dst);

    /**
     *  this function performs the a* search pathfinding with Euclidean bounds on graph and return a
     *  bool value true if the path is found, else return false.
     */
    bool astar_search(Node &src, Node &dst);

    /**
     *  this function performs the ALT(A* landmark triangular inequality) search pathfinding on graph and return a
     *  bool value true if the path is found, else return false.
     *  https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf
     *  https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GW05.pdf
     */
    bool ALT_search(Node &src, Node &dst, int landmarks_num, bool improved);

    std::vector<double> get_landmarks(int n, bool improved);

    /**
     * @brief Get the ext paths object
     * 
     * @return std::vector<std::vector<std::vector<double>>> 
     */
    std::vector<double> get_ext_paths();

    /**
     * @brief
     * the path is found from dst to src using the parent pointer in nodes
     */
    std::vector<double> get_path(Node &src, Node &dst);

    void display_path();
};
/* Class definition ends */

#endif