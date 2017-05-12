#include "graph_theory.h"

#include <vector>
#include <stack>
#include <numeric>

bool calculate_minimum_spanning_tree(double &total_distance, Eigen::VectorXi &minimum_spanning_tree, const Eigen::MatrixXd &distance_matrix)
{
    const int N = distance_matrix.rows();

    std::vector<int> visited_nodes;
    visited_nodes.reserve(N);
    visited_nodes.push_back(0);

    std::vector<int> unvisited_nodes;
    unvisited_nodes.resize(N - 1);
    std::iota(unvisited_nodes.begin(), unvisited_nodes.end(), 1);

    total_distance = 0;

    while (!unvisited_nodes.empty())
    {
        auto i = visited_nodes.cbegin();
        auto j = unvisited_nodes.cbegin();

        for (auto v = visited_nodes.cbegin(); v != visited_nodes.cend(); ++v)
        {
            for (auto u = unvisited_nodes.cbegin(); u != unvisited_nodes.cend(); ++u)
            {
                if (distance_matrix(*i, *j) > distance_matrix(*v, *u))
                {
                    i = v;
                    j = u;
                }
            }
        }

        if (distance_matrix(*i, *j) == std::numeric_limits<double>::infinity())
        {
            return false;
        }
        else
        {
            minimum_spanning_tree[*j] = static_cast<int>(*i);
            total_distance += distance_matrix(*i, *j);
            visited_nodes.push_back(*j);
            unvisited_nodes.erase(j);
        }
    }

    return true;
}

bool calculate_degree(Eigen::VectorXi &node_degree, const Eigen::VectorXi &minimum_spanning_tree)
{
    const int N = minimum_spanning_tree.rows();
    node_degree.setZero(N);

    for (int i = 1; i < N; ++i)
    {
        node_degree[i] ++;
        node_degree[minimum_spanning_tree[i]] ++;
    }

    return true;
}

bool calculate_graph_balance(Eigen::MatrixXd &balance_matrix, const Eigen::VectorXi &node_degree, const double &total_distance, const Eigen::VectorXi &minimum_spanning_tree, const Eigen::MatrixXd &distance_matrix)
{
    const int N = node_degree.rows();
    balance_matrix.setZero(N, N);

    Eigen::VectorXi local_node_degree = node_degree;

    Eigen::VectorXi temp_node_degree(N);

    bool updated;
    do {
        updated = false;
        temp_node_degree = local_node_degree;

        for (int i = 1; i < N; ++i)
        {
            if (local_node_degree[i] > 0 && local_node_degree[minimum_spanning_tree[i]] == 1)
            {
                const int j = minimum_spanning_tree[i]; // 0
                const int k = i; // 1

                temp_node_degree[j] --;
                temp_node_degree[k] --;

                balance_matrix(k, j) = distance_matrix(k, j) + balance_matrix(j, j);
                balance_matrix(j, k) = total_distance - balance_matrix(j, j);
                balance_matrix(k, k) += balance_matrix(k, j);

                updated = true;
            }
            else if (local_node_degree[minimum_spanning_tree[i]] > 0 && local_node_degree[i] == 1)
            {
                const int j = i;
                const int k = minimum_spanning_tree[i];

                temp_node_degree[j] --;
                temp_node_degree[k] --;

                balance_matrix(k, j) = distance_matrix(k, j) + balance_matrix(j, j);
                balance_matrix(j, k) = total_distance - balance_matrix(j, j);
                balance_matrix(k, k) += balance_matrix(k, j);

                updated = true;
            }
        }
        local_node_degree = temp_node_degree;
    } while(updated);

    return true;
}

bool calculate_graph_center(int &center, const Eigen::MatrixXd &balance_matrix, const Eigen::VectorXi &minimum_spanning_tree)
{
    const int N = minimum_spanning_tree.rows();
    center = 0;

    double best_node_balance = 0;

    for (int i = 1; i < N; ++i)
    {
        int j = static_cast<int>(minimum_spanning_tree[i]);
        double node_balance = std::min(balance_matrix(i, j), balance_matrix(j, i)) / std::max(balance_matrix(i, j), balance_matrix(j, i));

        if (node_balance > best_node_balance)
        {
            center = i;
            best_node_balance = node_balance;
        }
    }

    return true;
}

bool calculate_optimal_path(Eigen::VectorXi &path, const int &center, const Eigen::VectorXi &minimum_spanning_tree, const Eigen::VectorXi &node_degree)
{
    const int N = minimum_spanning_tree.rows();

    Eigen::VectorXi local_node_degree = node_degree;
    Eigen::VectorXi temp_node_degree(N);

    bool updated;
    do
    {
        updated = false;
        temp_node_degree = local_node_degree;

        for (int i = 1; i < N; ++i)
        {
            if (local_node_degree[i] > 0 && local_node_degree[minimum_spanning_tree[i]] == 1 && minimum_spanning_tree[i] != center)
            {
                const int j = static_cast<int>(minimum_spanning_tree[i]);
                const int k = i;

                temp_node_degree[j] --;
                temp_node_degree[k] --;

                path[j] = static_cast<int>(k);

                updated = true;
            }
            else if (local_node_degree[minimum_spanning_tree[i]] > 0 && local_node_degree[i] == 1 && i != center)
            {
                const int j = i;
                const int k = static_cast<int>(minimum_spanning_tree[i]);

                temp_node_degree[j] --;
                temp_node_degree[k] --;

                path[j] = static_cast<int>(k);

                updated = true;
            }
        }

        local_node_degree = temp_node_degree;
    } while (updated);

    path[center] = static_cast<int>(center);

    return true;
}


bool calculate_node_distances(Eigen::VectorXi &node_distances, const Eigen::VectorXi &path, const Eigen::VectorXi &node_degree, const int &center)
{
    const auto n = path.rows();

    node_distances.resize(n);
    node_distances[center] = 0;

    std::stack<int> distance_stack;
    distance_stack.push(0);

    for (auto i = 0; i < n; ++i)
    {
        if (node_degree[i] == 1)
        {
            for (auto j = i; j != center; j = path[j])
            {
                distance_stack.push(distance_stack.top() + 1);
            }
            for (auto j = i; j != center; j = path[j])
            {
                node_distances[j] = distance_stack.top();
                distance_stack.pop();
            }
        }
    }

    return true;
}

bool sort_nodes_by_distance(Eigen::VectorXi &sorted_nodes, const Eigen::VectorXi &node_distances)
{
    const auto n = node_distances.rows();
    sorted_nodes.resize(n);

    std::iota(sorted_nodes.data(), sorted_nodes.data() + n, 0);
    std::sort(sorted_nodes.data(), sorted_nodes.data() + n, [&](int i, int j){
        return node_distances[i] < node_distances[j];
    });

    return true;
}
