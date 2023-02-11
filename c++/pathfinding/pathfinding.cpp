#include "pathfinding.h"

#include <random>
#include <fstream>

#include <emscripten.h>
#include <emscripten/bind.h>

Node::Node(double la, double lo) : la_{la}, lo_{lo}, parent{nullptr}, cost_{0}, weight_{0}, size_{0}{
    // std::cout << "node (" << la << ", " << lo << ") constructor" << std::endl;
}

Node::~Node(){
    // std::cout << "node (" << la << ", " << lo << ") destructor" << std::endl;
}

double Node::get_la() const{return la_;}
double Node::get_lo() const{return lo_;}
void Node::print_coords() const { 
    std::cout << "(" << la_ << ", " << lo_ << ")";
}

double Node::get_cost()  const {return cost_;}

void Node::set_cost(double cost) {
    this->cost_ = cost;
}

double Node::get_weight()  const{
    return weight_;
}
void Node::set_weight(double weight){
    weight_ = weight;
}

double Node::get_size()  const{
    return size_;
}
void Node::set_size(double size){
    size_ = size;
}

void Node::set_parent(Node *ptr) { parent = ptr; }

Node *Node::get_parent() const { return parent; }

void Node::set_dists2l(const std::vector<std::vector<double>> &landmarks){
    for(size_t i = 0; i < landmarks.size(); i++){
        dists2l[i] = euclidean_distance(la_, lo_, landmarks[i][0], landmarks[i][1]);
    }
};

void Node::reset(){
    parent = nullptr;
    children.clear();
    cost_ = 0;
    weight_ = 0;
    size_ = 0;
}

bool Node::operator==(const Node &rhs) const
{
    return la_==(rhs.la_) && lo_==(rhs.lo_);
}

bool Node::operator!=(const Node &rhs) const
{
    return !(la_==(rhs.la_)) || !(lo_==(rhs.lo_));
}


Search_tree::Search_tree(): root_ptr{nullptr}{
    // std::cout << "search tree constructor" << std::endl;
};

Search_tree::~Search_tree(){
    // std::cout << "search tree destructor" << std::endl;
};

Node* Search_tree::get_root_ptr() const{return root_ptr;}

void Search_tree::set_root_ptr(Node* ptr){root_ptr = ptr;}

void Search_tree::reset_root_ptr(){root_ptr = nullptr;}

void Search_tree::add_edge(Node *parent_ptr, Node *child_ptr){
    parent_ptr->children.push_back(child_ptr);
} 

void Search_tree::remove_edge(Node *parent_ptr, Node *child_ptr){
    parent_ptr->children.erase(std::remove(parent_ptr->children.begin(), parent_ptr->children.end(), child_ptr),parent_ptr->children.end());
}

void Search_tree::print_tree(Node* root, const std::string& prefix, bool isLeft){
    /// base case of recursion
    if(!root) return;
    std::cout << prefix;
    std::cout << "└──";
    root->print_coords();
    std::cout << std::endl;

    auto it = root->children.begin();
    for(; it!= (--root->children.end());it++){
        print_tree(*it, prefix + ((isLeft ? "│   " : "    ")), true);            
    }
    print_tree(*it, prefix + ((isLeft ? "│   " : "    ")), false);        
}

Graph::Graph(){
    // std::cout << "graph constructor" << std::endl;

}

Graph::~Graph(){
    // std::cout << "graph destructor" << std::endl;
}

void Graph::add_edge(Node &u, Node &v, double cost, bool bidir)
{

    node_ptrs.insert(&u);
    node_ptrs.insert(&v);
    
    adjacency_list[&u].push_back(std::make_pair(&v, cost)); // u-->v edge added
    if (bidir == true) {
        // if graph is bidirectional
        adjacency_list[&v].push_back(std::make_pair(&u, cost)); // v-->u edge added 
    }
}

void Graph::graph_preprocessing(unsigned int cardinality){
    // avoid_selection(cardinality);  // preselected the landmarks and stored in this.landmarks
    for(size_t i = 0; i < landmarks.size(); i++)
        landmark_ids.push_back(i);
    precompute_landmarks_distances();
}

void Graph::farthest_selection(unsigned int cardinality, unsigned int subset_cardinality){
    std::vector<Node*> node_ptrs_vec(node_ptrs.begin(), node_ptrs.end());
    auto rng = std::default_random_engine {};
    std::shuffle(node_ptrs_vec.begin(), node_ptrs_vec.end(), rng);
    std::vector<double> start = {node_ptrs_vec[0]->get_la(), node_ptrs_vec[0]->get_lo()};
    landmarks.push_back(start);
    while(landmarks.size() <= cardinality){
        std::sort(node_ptrs_vec.begin(), node_ptrs_vec.end(), [this](Node * n1, Node * n2){
            return euclidean_distance_sum(n1->get_la(), n1->get_lo(), this->landmarks) > 
            euclidean_distance_sum(n2->get_la(), n2->get_lo(), this->landmarks);
        });
        // if(find_if(landmarks.begin(), landmarks.end(), [&node_ptrs_vec](std::vector<double> n){return (n[0] == node_ptrs_vec[0]->get_la()) && (n[1] == node_ptrs_vec[0]->get_lo());})!=landmarks.end())
        //     break;
        landmarks.push_back({node_ptrs_vec[0]->get_la(), node_ptrs_vec[0]->get_lo()});
    }
    landmarks.erase(landmarks.begin());
    std::shuffle(node_ptrs_vec.begin(), node_ptrs_vec.end(), rng);
    landmarks = std::vector<std::vector<double>>(landmarks.begin(), landmarks.begin() + (landmarks.size() < subset_cardinality? landmarks.size(): subset_cardinality));
}

// implementation of avoid landmark selection in https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GW05.pdf
void Graph::avoid_selection(unsigned int cardinality){
    std::vector<Node*> node_ptrs_vec(node_ptrs.begin(), node_ptrs.end());
    auto rng = std::default_random_engine {};
    std::shuffle(node_ptrs_vec.begin(), node_ptrs_vec.end(), rng);
    
    while(landmarks.size() < cardinality){
        std::shuffle(node_ptrs_vec.begin(), node_ptrs_vec.end(), rng);
        Node* root_ptr = node_ptrs_vec[0];

        // create shortest path tree
        uniform_cost_traverse(root_ptr);

        // calculate weight for each nodes
        for(Node*& node_ptr: node_ptrs_vec){
            node_ptr->set_weight(calc_weight(node_ptr, root_ptr));
        }

        // calculate size for each nodes
        for(Node*& node_ptr: node_ptrs_vec){
            node_ptr->set_size(calc_size(node_ptr));
        }

        // get node w of maximum size
        std::sort(node_ptrs_vec.begin(), node_ptrs_vec.end(), [this](Node * n1, Node * n2){ return n1->get_size() > n2->get_size(); });

        Node* w = node_ptrs_vec[0];

        // follow the node w to find a new landmark
        Node* leaf = find_leaf(w);
        if(find_if(landmarks.begin(), landmarks.end(), [&leaf](std::vector<double> n){return (n[0] == leaf->get_la()) && (n[1] == leaf->get_lo());})==landmarks.end())
            landmarks.push_back({leaf->get_la(), leaf->get_lo()});
    }

}

void Graph::random_avoid_selection(unsigned int cardinality){
    avoid_selection(cardinality);
    std::vector<Node*> node_ptrs_vec(node_ptrs.begin(), node_ptrs.end());
    auto rng = std::default_random_engine {};
    std::shuffle(node_ptrs_vec.begin(), node_ptrs_vec.end(), rng);
    int i = 0;
    while(landmarks.size() < 2*cardinality){
        std::vector<double> lm = {node_ptrs_vec[i]->get_la(), node_ptrs_vec[i]->get_lo()};
        if(find_if(landmarks.begin(), landmarks.end(), [&lm](std::vector<double> n){return (n[0] == lm[0]) && (n[1] == lm[1]);})==landmarks.end()){
            landmarks.push_back(lm);
        }
        i++;
    }
}

double Graph::calc_weight(const Node* node_ptr, const Node* root_ptr){
    std::list<double> triangular_inequalities;

    for(std::vector<double>& landmark: landmarks){
        triangular_inequalities.push_back(abs(euclidean_distance(node_ptr->get_la(), node_ptr->get_lo(), landmark[0], landmark[1]) - euclidean_distance(root_ptr->get_la(), root_ptr->get_lo(), landmark[0], landmark[1])));
    }
    double weight = euclidean_distance(node_ptr->get_la(), node_ptr->get_lo(), root_ptr->get_la(), root_ptr->get_lo()) - *(std::max_element(triangular_inequalities.begin(), triangular_inequalities.end()));
    return weight;
}

double Graph::calc_size(const Node* node_ptr){
    double size = add_weight(node_ptr);
    landmark_flag = false;
    return size;
}

double Graph::add_weight(const Node* node_ptr){
    if(!node_ptr) return 0;
    // if(find_if(landmarks.begin(), landmarks.end(), [&node_ptr](std::vector<double> n){return (n[0] == node_ptr->get_la()) && (n[1] == node_ptr->get_lo());})!=landmarks.end()){
    //     landmark_flag=true;
    //     return 0;
    // }
    double temp{0};
    temp += node_ptr->get_weight();
    for(const Node* child: node_ptr->children){
        temp += add_weight(child);
    }
    return temp;
    // return landmark_flag ? 0: temp;
};

Node* Graph::find_leaf(Node* node_ptr){
    while(node_ptr->children.size()){
        std::vector<Node*> temp = node_ptr->children;
        std::sort(temp.begin(), temp.end(), [](Node * n1, Node * n2){ return n1->get_size() > n2->get_size(); });
        node_ptr = *(temp.begin());
    }
    return node_ptr;
}

void Graph::precompute_landmarks_distances(){

    for(Node* node_ptr: node_ptrs){
        node_ptr->set_dists2l(landmarks);
    }
}

bool Graph::breadth_first_search(Node &src, Node &dst)
{   
    reset_graph();

    if (&src == &dst)
    {
        return true;
    }

    /// push the source vertex to queue to begin traversing
    open_table.push_back(&src);

    bool in_queue; /// node in the queue flag
    bool visited;  /// node visited flag

    while (!open_table.empty())
    {   
        /// traverse the graph till no connected vertex are left
        /// extract a node from queue for further traversal
        Node *node_ptr = open_table.front();
        /// remove the node from the queue
        open_table.pop_front();
        /// set the node as visited
        close_table.push_back(node_ptr);
        for (std::pair<Node*, double> neighbour : adjacency_list[node_ptr])
        {
            /// check every vertex connected to the node which are still
            /// unvisited
            in_queue = find(open_table.begin(), open_table.end(), neighbour.first) != open_table.end();
            visited = find(close_table.begin(), close_table.end(), neighbour.first) != close_table.end();
            if (!(in_queue || visited))
            {
                neighbour.first->set_parent(node_ptr);
                if (neighbour.first == &dst)
                {   
                    return true;
                }
                /// if the neighbour is unvisited , push it into the queue
                open_table.push_back(neighbour.first);
            }
        }
    }
    return false;
}

bool Graph::depth_first_search(Node &src, Node &dst)
{
    reset_graph();

    if (&src == &dst)
    {
        return true;
    }

    /// push the source vertex to stack to begin traversing
    open_table.push_back(&src);

    bool in_stack; /// node in the queue flag
    bool visited;  /// node visited flag

    while (!open_table.empty())
    {
        /// traverse the graph till no connected vertex are left
        /// extract a node from stack for further traversal
        Node *node_ptr = open_table.back();
        /// remove the node from the stack
        open_table.pop_back();
        /// set the node as visited
        close_table.push_back(node_ptr);
        for (std::pair<Node*, double> neighbour : adjacency_list[node_ptr])
        {
            /// check every vertex connected to the node which are still
            /// unvisited
            in_stack = find(open_table.begin(), open_table.end(), neighbour.first) != open_table.end();
            visited = find(close_table.begin(), close_table.end(), neighbour.first) != close_table.end();
            if (!(in_stack || visited))
            {
                neighbour.first->set_parent(node_ptr);
                if (neighbour.first == &dst)
                {
                    return true;
                }
                /// if the neighbour is unvisited , push it into the queue
                open_table.push_back(neighbour.first);
            }
        }
    }
    return false;
}

bool Graph::uniform_cost_search(Node &src, Node &dst)
{
    reset_graph();

    if (&src == &dst)
    {   
        return true;
    }

    /// push the source vertex to queue to begin traversing
    open_table.push_back(&src);

    bool in_queue; /// node in the queue flag
    bool visited;  /// node visited flag

    while (!open_table.empty())
    {
        /// sort the open table
        std::sort(open_table.begin(), open_table.end(), [](const Node* n1, const Node* n2){return n1->get_cost() < n2->get_cost();});
        /// traverse the graph till no connected vertex are left
        /// extract a node from stack for further traversal
        Node *node_ptr = open_table.front();
        /// remove the node from the stack
        open_table.pop_front();

        if (node_ptr == &dst)
            {   
                return true;
            }

        /// set the node as visited
        close_table.push_back(node_ptr);
        std::list<std::pair<Node *, double>> neighbours = adjacency_list[node_ptr];
        for (std::pair<Node*, double> neighbour : neighbours)
        {   
            std::vector<double> path = {node_ptr->get_la(), node_ptr->get_lo(), neighbour.first->get_la(), neighbour.first->get_lo()};
            ext_paths.insert(ext_paths.end(), path.begin(), path.end());
            /// check every vertex connected to the node which are still
            /// unvisited
            in_queue = find(open_table.begin(), open_table.end(), neighbour.first) != open_table.end();
            visited = find(close_table.begin(), close_table.end(), neighbour.first) != close_table.end();

            /// cost from current node to neighbour
            double node2curr_cost = (*find_if(neighbours.begin(), neighbours.end(), [&neighbour](std::pair<Node*, double> n){return n.first == neighbour.first;})).second;

            /// cost from neighbour to source
            double new_cost = node_ptr->get_cost() + node2curr_cost;
            if (!(in_queue || visited))
            {
                neighbour.first->set_parent(node_ptr);

                /// update cost of neighbour node
                neighbour.first->set_cost(new_cost);
                
                /// if the neighbour is unvisited , push it into the queue
                open_table.push_back(neighbour.first);
            }
            else if(in_queue){
                if(neighbour.first->get_cost() > new_cost){
                    /// update cost of neighbour node
                    neighbour.first->set_cost(new_cost);
                    neighbour.first->set_parent(node_ptr);
                }

            }
        }
    }
    return false;
}

void Graph::uniform_cost_traverse(Node* root_ptr)
{
    reset_graph();
    /// set the root ptr of the search tree
    t.set_root_ptr(root_ptr);

    /// push the source vertex to queue to begin traversing
    open_table.push_back(root_ptr);

    bool in_queue; /// node in the queue flag
    bool visited;  /// node visited flag

    while (!open_table.empty())
    {
        /// sort the open table
        std::sort(open_table.begin(), open_table.end(), [](const Node* n1, const Node* n2){return n1->get_cost() < n2->get_cost();});
        /// traverse the graph till no connected vertex are left
        /// extract a node from stack for further traversal
        Node *node_ptr = open_table.front();
        /// remove the node from the stack
        open_table.pop_front();

        /// set the node as visited
        close_table.push_back(node_ptr);
        std::list<std::pair<Node *, double>> neighbours = adjacency_list[node_ptr];
        for (std::pair<Node*, double> neighbour : neighbours)
        {   
            /// check every vertex connected to the node which are still
            /// unvisited
            in_queue = find(open_table.begin(), open_table.end(), neighbour.first) != open_table.end();
            visited = find(close_table.begin(), close_table.end(), neighbour.first) != close_table.end();

            /// cost from node to neighbour
            double node2curr_cost = (*find_if(neighbours.begin(), neighbours.end(), [&neighbour](std::pair<Node*, double> n){return n.first == neighbour.first;})).second;

            /// cost from neighbour to source
            double new_cost = node_ptr->get_cost() + node2curr_cost;
            if (!(in_queue || visited))
            {
                neighbour.first->set_parent(node_ptr);

                /// update cost of neighbour node
                neighbour.first->set_cost(new_cost);
                
                /// if the neighbour is unvisited , push it into the queue
                open_table.push_back(neighbour.first);
                t.add_edge(node_ptr, neighbour.first);
            }
            else if(in_queue){
                if(neighbour.first->get_cost() > new_cost){
                    /// update cost of neighbour node
                    Node* old_parent = neighbour.first->get_parent();
                    neighbour.first->set_cost(new_cost);
                    neighbour.first->set_parent(node_ptr);
                    t.remove_edge(old_parent, neighbour.first);
                    t.add_edge(node_ptr, neighbour.first);
                }

            }
        }
    }
}

bool Graph::astar_search(Node &src, Node &dst)
{   
    reset_graph();

    if (&src == &dst)
    {   
        return true;
    }

    Node* dst_ptr = &dst;

    /// push the source vertex to queue to begin traversing
    open_table.push_back(&src);

    bool in_queue; /// node in the queue flag
    bool visited;  /// node visited flag

    while (!open_table.empty())
    {
        /// sort the open table
        std::sort(open_table.begin(), open_table.end(), [dst_ptr](const Node* n1, const Node* n2){return cost_with_euclidean_bound(n1, dst_ptr) < cost_with_euclidean_bound(n2,dst_ptr);});
        /// traverse the graph till no connected vertex are left
        /// extract a node from stack for further traversal
        Node *node_ptr = open_table.front();
        /// remove the node from the stack
        open_table.pop_front();

        if (node_ptr == &dst)
            {   
                return true;
            }

        /// set the node as visited
        close_table.push_back(node_ptr);
        std::list<std::pair<Node *, double>> neighbours = adjacency_list[node_ptr];
        for (std::pair<Node*, double> neighbour : neighbours)
        {   
            std::vector<double> path = {node_ptr->get_la(), node_ptr->get_lo(), neighbour.first->get_la(), neighbour.first->get_lo()};
            for(const double& i: path){
                ext_paths.push_back(i);
            }
            /// check every vertex connected to the node which are still
            /// unvisited
            in_queue = find(open_table.begin(), open_table.end(), neighbour.first) != open_table.end();
            visited = find(close_table.begin(), close_table.end(), neighbour.first) != close_table.end();

            /// cost from current node to neighbour
            double node2curr_cost = (*find_if(neighbours.begin(), neighbours.end(), [&neighbour](std::pair<Node*, double> n){return n.first == neighbour.first;})).second;
            /// cost from neighbour to source
            double new_cost = node_ptr->get_cost() + node2curr_cost;
            if (!(in_queue || visited))
            {
                neighbour.first->set_parent(node_ptr);

                /// update cost of neighbour node
                neighbour.first->set_cost(new_cost);
                
                /// if the neighbour is unvisited , push it into the queue
                open_table.push_back(neighbour.first);
            }
            else if(in_queue){
                if(neighbour.first->get_cost() > new_cost){
                    /// update cost of neighbour node
                    neighbour.first->set_cost(new_cost);
                    neighbour.first->set_parent(node_ptr);
                }
            }
        }
    }
    return false;
}

// implementation of ALT search in https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf
bool Graph::ALT_search(Node &src, Node &dst, int landmark_num, bool improved)
{   
    reset_graph();

    if (&src == &dst)
    {   
        return true;
    }

    Node* src_ptr = &src;
    Node* dst_ptr = &dst;

    if(improved)
    // std::sort(landmark_ids.begin(), landmark_ids.end(), [&src_ptr, &dst_ptr](int l1, int l2){return (abs(src_ptr->dists2l[l1] - dst_ptr->dists2l[l1])) > (abs(src_ptr->dists2l[l2] - dst_ptr->dists2l[l2]));});

        // std::sort(landmark_ids.begin(), landmark_ids.end(), [&src_ptr, &dst_ptr](int l1, int l2){return std::max(src_ptr->dists2l[l1] - 2*dst_ptr->dists2l[l1], dst_ptr->dists2l[l1] - 2*src_ptr->dists2l[l1]) > std::max(src_ptr->dists2l[l2] - 2*dst_ptr->dists2l[l2], dst_ptr->dists2l[l2] - 2*src_ptr->dists2l[l2]);});

        std::sort(landmark_ids.begin(), landmark_ids.end(), [&src_ptr, &dst_ptr](int l1, int l2){return (src_ptr->dists2l[l1] - 2*dst_ptr->dists2l[l1])> (src_ptr->dists2l[l2] - 2*dst_ptr->dists2l[l2]);});

    /// push the source vertex to queue to begin traversing
    open_table.push_back(src_ptr);

    bool in_queue; /// node in the queue flag
    bool visited;  /// node visited flag

    while (!open_table.empty())
    {
        /// sort the open table
        std::sort(open_table.begin(), open_table.end(), [this, &dst_ptr, &landmark_num, &improved](Node* n1, Node* n2){return this->cost_with_max_triangular_inequality(n1, dst_ptr, landmark_num, improved) < this->cost_with_max_triangular_inequality(n2,dst_ptr, landmark_num, improved);});
        /// traverse the graph till no connected vertex are left
        /// extract a node from stack for further traversal
        Node *node_ptr = open_table.front();
        /// remove the node from the stack
        open_table.pop_front();

        if (node_ptr == &dst)
            {   
                return true;
            }

        /// set the node as visited
        close_table.push_back(node_ptr);
        std::list<std::pair<Node *, double>> neighbours = adjacency_list[node_ptr];
        for (std::pair<Node*, double> neighbour : neighbours)
        {   
            std::vector<double> path = {node_ptr->get_la(), node_ptr->get_lo(), neighbour.first->get_la(), neighbour.first->get_lo()};
            for(const double& i: path){
                ext_paths.push_back(i);
            }
            /// check every vertex connected to the node which are still
            /// unvisited
            in_queue = find(open_table.begin(), open_table.end(), neighbour.first) != open_table.end();
            visited = find(close_table.begin(), close_table.end(), neighbour.first) != close_table.end();

            /// cost from current node to neighbour
            double node2curr_cost = (*find_if(neighbours.begin(), neighbours.end(), [&neighbour](std::pair<Node*, double> n){return n.first == neighbour.first;})).second;
            /// cost from neighbour to source
            double new_cost = node_ptr->get_cost() + node2curr_cost;
            if (!(in_queue || visited))
            {
                neighbour.first->set_parent(node_ptr);

                /// update cost of neighbour node
                neighbour.first->set_cost(new_cost);
                
                /// if the neighbour is unvisited , push it into the queue
                open_table.push_back(neighbour.first);
            }
            else if(in_queue){
                if(neighbour.first->get_cost() > new_cost){
                    /// update cost of neighbour node
                    neighbour.first->set_cost(new_cost);
                    neighbour.first->set_parent(node_ptr);
                }
            }
        }
    }
    return false;
}

double Graph::cost_with_max_triangular_inequality(Node* node_ptr, Node* dst_ptr, int landmark_num, bool improved){
    if(improved){
        double max = abs(node_ptr->dists2l[landmark_ids[0]] - dst_ptr->dists2l[landmark_ids[0]]);
        double temp{0};
        for(size_t i = 1; i < landmark_num; i++){
            temp = abs(node_ptr->dists2l[landmark_ids[i]] - dst_ptr->dists2l[landmark_ids[i]]);
            if(temp > max) max = temp;
        }
        return node_ptr->get_cost() + max * 1000;
    }
    else{
        double max = abs(node_ptr->dists2l[0] - dst_ptr->dists2l[0]);
        double temp{0};
        for(size_t i = 1; i < landmark_num; i++){
            temp = abs(node_ptr->dists2l[i] - dst_ptr->dists2l[i]);
            if(temp > max) max = temp;
        }
        return node_ptr->get_cost() + max * 1000;
    }
    // double v1 = abs(node_ptr->dists2l[landmark_ids[0]] - dst_ptr->dists2l[landmark_ids[0]]);

    // return node_ptr->get_cost() + v1 * 1000;
}

std::vector<double> Graph::get_landmarks(int n, bool improved){
    std::vector<double> lms{};
    if(improved)
        for(size_t i = 0; i < n; i++){
            lms.push_back(landmarks[landmark_ids[i]][0]);
            lms.push_back(landmarks[landmark_ids[i]][1]);
        }   
    else
        for(size_t i = 0; i < n; i++){
            lms.push_back(landmarks[i][0]);
            lms.push_back(landmarks[i][1]);
        }  
    return lms;
}

std::vector<double> Graph::get_ext_paths(){
    return ext_paths;
}

std::vector<double> Graph::get_path(Node &src, Node &dst)
{   
    Node temp = dst;
    final_path.push_back(temp.get_lo());
    final_path.push_back(temp.get_la());
    while (temp != src)
    {
        temp = *(temp.get_parent());
        final_path.push_back(temp.get_lo());
        final_path.push_back(temp.get_la());
    }
    // Reverse the vector
    std::reverse(final_path.begin(), final_path.end());
    return final_path;
}

void Graph::display_path()
{
    for (size_t i = 0; i < final_path.size(); i+= 2)
    {
        std::cout << "(" << final_path[i] << ", " << final_path[i + 1] << ")->";
    }
    std::cout << "finish" << std::endl;
}

void Graph::reset_graph()
{   
    /// reset parent, children and cost of all nodes in the graph
    for (auto ptr : node_ptrs)
        ptr->reset();

    /// reset open table, close table, extended paths, final path and search tree
    open_table.clear();
    close_table.clear();
    ext_paths.clear();
    final_path.clear();  
    t.reset_root_ptr();
}

EMSCRIPTEN_BINDINGS(script){
    emscripten::class_<Node>("Node")
    .constructor<double, double>()
    .property("la", &Node::get_la)
    .property("lo", &Node::get_lo);

    emscripten::class_<Graph>("Graph")
    .constructor<>()
    .function("addEdge", &Graph::add_edge)
    .function("graphPreprocessing", &Graph::graph_preprocessing)
    .function("UCSearch", &Graph::uniform_cost_search)
    .function("AStarSearch", &Graph::astar_search)
    .function("ALTSearch", &Graph::ALT_search)
    .function("getLandmarks", &Graph::get_landmarks)
    .function("getExtPaths", &Graph::get_ext_paths)
    .function("getPath", &Graph::get_path);
    emscripten::register_vector<double>("vector1D");
}
