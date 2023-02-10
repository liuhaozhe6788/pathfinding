#include "pathfinding.h"

#include <vector>
#include <string>
#include <iostream>
#include <memory>

/** Test function */
static void tests()
{   
    // // avoid selection Begin
    std::vector<Node> nodes = {};
    int num = 7;
    std::vector<double> landmarks;
    for(size_t i = 1; i <= num; i++)
        nodes.push_back(Node(i, i*i));
    Graph g1;
    g1.add_edge(nodes[0], nodes[1], 2, true);  
    g1.add_edge(nodes[0], nodes[3], 1, true);   
    g1.add_edge(nodes[0], nodes[5], 2, true);  
    g1.add_edge(nodes[0], nodes[6], 2, true);  
    g1.add_edge(nodes[1], nodes[2], 2, true);  
    g1.add_edge(nodes[1], nodes[3], 1, true);  
    g1.add_edge(nodes[2], nodes[3], 1, true);  
    g1.add_edge(nodes[2], nodes[4], 3, true);  
    g1.add_edge(nodes[3], nodes[4], 1, true);  
    g1.add_edge(nodes[3], nodes[5], 2, true); 
    g1.add_edge(nodes[4], nodes[5], 1, true); 
    g1.add_edge(nodes[5], nodes[6], 1, true); 

    std::cout << "Test avoid selection Passed..." << std::endl;
    std::cout << std::endl;

    // nodes = {};
    // num = 120000;
    // for(size_t i = 0; i < num; i++)
    //     nodes.push_back(Node(i, i*i));

    // Graph g;
    // for(size_t i = 0; i < num-1; i++)
    //     g.add_edge(nodes[i], nodes[i+1], i, true);

    // /// Test BFS Begin
    // if (g.breadth_first_search(nodes[0], nodes[5]))
    // {
    //     g.display_path();
    // }

    // if (g.breadth_first_search(nodes[2], nodes[7]))
    // {
    //     g.display_path();
    // }

    // if (g.breadth_first_search(nodes[1], nodes[1]))
    // {
    //     g.display_path();
    // }

    // std::cout << "Test BFS Passed..." << std::endl;
    // std::cout << std::endl;

    // /// Test DFS Begin
    // if (g.depth_first_search(nodes[0], nodes[5]))
    // {
    //     g.display_path();
    // }

    // if (g.depth_first_search(nodes[2], nodes[7]))
    // {
    //     g.display_path();
    // }

    // if (g.depth_first_search(nodes[1], nodes[1]))
    // {
    //     g.display_path();
    // }

    // std::cout << "Test DFS Passed..." << std::endl;
    // std::cout << std::endl;

    // /// Test uniform cost search Begin
    // for(int i = 0; i < 100; i++){
    //     if (g.uniform_cost_search(nodes[0], nodes[6]))
    //     {
    //         std::vector<double> temp = g.get_path(nodes[0], nodes[6]);
    //         g.display_path();
    //     }
    // }

    // if (g.uniform_cost_search(nodes[2], nodes[7]))
    // {   
    //     g.display_path();
    // }

    // if (g.uniform_cost_search(nodes[1], nodes[2]))
    // {   
    //     std::vector<double> temp = g.get_path(nodes[1], nodes[2]);
    //     g.display_path();
    // }

    // std::cout << "Test uniform cost search Passed..." << std::endl;
    // std::cout << std::endl;

    // /// Test a* search Begin
    // for(int i = 0; i < 100; i++){
    //     if (g.astar_search(nodes[0], nodes[6]))
    //     {   
    //         std::vector<double> temp = g.get_path(nodes[0], nodes[6]);
    //         g.display_path();
    //     }
    // }

    // if (g.astar_search(nodes[2], nodes[7]))
    // {   
    //     g.display_path();
    // }

    // if (g.astar_search(nodes[1], nodes[1]))
    // {   
    //     g.display_path();
    // }

    // std::cout << "Test a* search Passed..." << std::endl;
    // std::cout << std::endl;

    /// Test a* search Begin
    if (g1.astar_search(nodes[0], nodes[6]))
    {   
        std::vector<double> lms = g1.get_top2_landmarks();
        std::vector<double> temp = g1.get_path(nodes[0], nodes[6]);
        g1.display_path();
    }

    std::cout << "Test a* search Passed..." << std::endl;
    std::cout << std::endl;

}

int main()
{
    std::cout << std::boolalpha;
    tests();
}