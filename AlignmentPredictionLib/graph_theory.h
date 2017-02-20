#ifndef GRAPH_THEORY_H
#define GRAPH_THEORY_H

#include "alignmentpredictionlib_global.h"

#include <Eigen/Dense>

//!
//! \brief calculate_minimum_spanning_tree
//!     A function that calculates the minimum spanning tree of a graph. Running time is O(n^2).
//!     See: https://en.wikipedia.org/wiki/Minimum_spanning_tree
//! \param total_distance
//!     The total traversal distance of the minimum spanning tree.
//! \param minimum_spanning_tree
//!     The minimum spanning tree. The format for this is minimum_spanning_tree[i] = j, where i is the ith node in a graph
//!     and j is the jth node in the same graph. Thus (i, j) is an edge in the minimum spanning tree.
//! \param distance_matrix
//!     A distance matrix so that distance_matrix(i, j) indicates some distance between node i and node j.
//! \return
//!
bool ALIGNMENTPREDICTIONLIBSHARED_EXPORT calculate_minimum_spanning_tree(double &total_distance, Eigen::VectorXi &minimum_spanning_tree, const Eigen::MatrixXd &distance_matrix);

//!
//! \brief calculate_degree
//!     A function that calculates the degree of a node (or vertex) in a graph. This number correspondes to the number of edges a node has.
//!     Running time is O(n).
//!     See: http://mathworld.wolfram.com/VertexDegree.html
//! \param node_degree
//!     A vector of node degrees where node_degree[i] indicates the degree of node i
//! \param minimum_spanning_tree
//!     The minimum spanning tree. The format for this is minimum_spanning_tree[i] = j, where i is the ith node in a graph
//!     and j is the jth node in the same graph. Thus (i, j) is an edge in the minimum spanning tree.
//! \return
//!
bool ALIGNMENTPREDICTIONLIBSHARED_EXPORT calculate_degree(Eigen::VectorXi &node_degree, const Eigen::VectorXi &minimum_spanning_tree);

//!
//! \brief calculate_graph_balance
//!     A function that calculates the balance of a graph. This function is used to calculate which node should be the center node of a graph.
//!     Running time is O(n^2).
//! \param balance_matrix
//!     A matrix where balance_matrix(i, i) indicates the balance at node i. All other values are unused and are the result of intermediate calculations.
//! \param node_degree
//!     A vector of node degrees where node_degree[i] indicates the degree of node i
//! \param total_distance
//!     The total traversal distance of the minimum spanning tree.
//! \param minimum_spanning_tree
//!     The minimum spanning tree. The format for this is minimum_spanning_tree[i] = j, where i is the ith node in a graph
//!     and j is the jth node in the same graph. Thus (i, j) is an edge in the minimum spanning tree.
//! \param distance_matrix
//!     A distance matrix so that distance_matrix(i, j) indicates some distance between node i and node j.
//! \return
//!
bool ALIGNMENTPREDICTIONLIBSHARED_EXPORT calculate_graph_balance(Eigen::MatrixXd &balance_matrix, const Eigen::VectorXi &node_degree, const double &total_distance, const Eigen::VectorXi &minimum_spanning_tree, const Eigen::MatrixXd &distance_matrix);

//!
//! \brief calculate_graph_center
//!     A function that calculates the node that is the center of a graph. Running time is O(n).
//!     See: https://en.wikipedia.org/wiki/Graph_center
//! \param center
//!     The node which is the center of the graph
//! \param balance_matrix
//!     A matrix where balance_matrix(i, i) indicates the balance at node i. All other values are unused and are the result of intermediate calculations.
//! \param minimum_spanning_tree
//!     The minimum spanning tree. The format for this is minimum_spanning_tree[i] = j, where i is the ith node in a graph
//!     and j is the jth node in the same graph. Thus (i, j) is an edge in the minimum spanning tree.
//! \return
//!
bool ALIGNMENTPREDICTIONLIBSHARED_EXPORT calculate_graph_center(int &center, const Eigen::MatrixXd &balance_matrix, const Eigen::VectorXi &minimum_spanning_tree);

//!
//! \brief calculate_optimal_path
//!     A function that calculates the optimal path of a graph. This function does not compute the shortest path from a node i to the center node but rather
//!     the overall path the minimizes the total traversal distance.
//! \param path
//!     A vector of indices that indicate that given node i, i should travel to path[i] which is a node j.
//! \param center
//!     The node which is the center of the graph
//! \param minimum_spanning_tree
//!     The minimum spanning tree. The format for this is minimum_spanning_tree[i] = j, where i is the ith node in a graph
//!     and j is the jth node in the same graph. Thus (i, j) is an edge in the minimum spanning tree.
//! \param node_degree
//!     A vector of node degrees where node_degree[i] indicates the degree of node i
//! \return
//!
bool ALIGNMENTPREDICTIONLIBSHARED_EXPORT calculate_optimal_path(Eigen::VectorXi &path, const int &center, const Eigen::VectorXi &minimum_spanning_tree, const Eigen::VectorXi &node_degree);

#endif // GRAPH_THEORY_H
