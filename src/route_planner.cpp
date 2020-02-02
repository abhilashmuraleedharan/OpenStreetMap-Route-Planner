#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Set the start_node and end_node attributes
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // Return the distance of the given node from the end node
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Populate current_node->neighbors vector with all the neighbors.
    current_node->FindNeighbors();

    /*
     * For each neighbor of the current node,
     *   a) Set the parent of neighbor to the current
     *   b) Set g value
     *   c) Set h value
     *   d) Add neighbor to open list
     *   e) Mark neighbor as valid
     */
    for (RouteModel::Node *nptr : current_node->neighbors) {
        nptr->parent = current_node;
        nptr->h_value = CalculateHValue(nptr);
        nptr->g_value = current_node->g_value + nptr->distance(*current_node);
        open_list.push_back(nptr);
        nptr->visited = true;
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list according to the sum of the h value and g value
    std::sort(open_list.begin(), open_list.end(), RoutePlanner::Compare);
    // Get the lowest (g+h) value node
    RouteModel::Node *least_sum_node = open_list.back();
    open_list.pop_back(); 
    return least_sum_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found{};

    /*
     * Iteratively follow the chain of parents of nodes until the starting node is found.
     * For each node in the chain, add the distance from the node to its parent to the distance variable.
     * The returned vector should be in the correct order: the start node should be the first element
     * of the vector, the end node should be the last element.
     */
    while(current_node != start_node) {
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    // Add the start_node to the path_found vector
    path_found.push_back(*current_node);
    // Reverse the order of elements in the vector
    std::reverse(path_found.begin(), path_found.end());
    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale(); 
    return path_found;
}

void RoutePlanner::AStarSearch() {
    // Add all of the neighbors of the starting node to the open_list
    AddNeighbors(start_node);

    while(!open_list.empty()) {
        RouteModel::Node *current_node = nullptr;
        // Sort the open_list and return the next node.
        current_node = NextNode();
        if (current_node == end_node) {
            // Goal node reached.
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        // Add all of the neighbors of the current node to the open_list
        AddNeighbors(current_node);
    }
    std::cout << "No path found\n";
}

/*
 * Custom compare function used to sort RouteModel::Nodes in descending order
 * of their f values where f = g+h
 */
bool RoutePlanner::Compare(RouteModel::Node *node1, RouteModel::Node *node2) {
    float f1 = node1->g_value + node1->h_value;
    float f2 = node2->g_value + node2->h_value;
    return f1 > f2;
}