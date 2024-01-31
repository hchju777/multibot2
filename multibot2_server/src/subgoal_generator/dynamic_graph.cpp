#include "multibot2_server/subgoal_generator/dynamic_graph.h"

namespace multibot2_server::SubgoalGenerator
{
    void DynamicGraph::addVertices(const std::map<std::string, Robot> &_robots)
    {
        reset();

        if (_robots.empty())
            return;

        for (const auto &robotPair : _robots)
        {
            const Robot &robot = robotPair.second;

            Vertex vertex(robot.name(), robot.pose(), robot.goal());

            for (const auto &neighborPair : robot.neighbors())
            {
                const Robot &neighbor = neighborPair.second;

                if (vertices_.contains(neighbor.name()))
                    continue;

                Vertex neighborVertex(neighbor.name(), neighbor.pose(), neighbor.goal());

                if (vertex.isGoal() and neighborVertex.isGoal())
                    continue;

                auto hasHigherPriority = [&](const Vertex &_low, const Vertex _high) -> bool
                {
                    if (_high.isGoal())
                        return false;

                    if (_low.goalDistance() < _high.goalDistance())
                        return false;

                    return true;
                };

                if (vertex.isGoal() or hasHigherPriority(vertex, neighborVertex))
                    adj_priority_list_[neighborVertex.name()].push_back(vertex.name());
                else if (neighborVertex.isGoal() or hasHigherPriority(neighborVertex, vertex))
                    adj_priority_list_[vertex.name()].push_back(neighborVertex.name());
                else
                    continue;

                adj_list_[vertex.name()].push_back(neighborVertex);
                adj_list_[neighborVertex.name()].push_back(vertex);
            }

            vertices_.emplace(vertex.name(), vertex);
        }
    }

    std::stack<std::string> DynamicGraph::topologicalSort()
    {
        std::stack<std::string> stack;

        std::map<std::string, bool> visited;
        for (const auto &vertexPair : vertices_)
        {
            const std::string &vertexName = vertexPair.first;
            visited.emplace(vertexName, false);
        }

        for (const auto &vertexPair : vertices_)
        {
            const std::string &vertexName = vertexPair.first;

            if (visited[vertexName] == false)
                topologicalSortUtil(vertexName, visited, stack);
        }

        return stack;
    }

    std::list<Vertices> DynamicGraph::generateGroupList()
    {
        std::list<Vertices> groupList;

        std::map<std::string, bool> visited;
        for (const auto &vertexPair : vertices_)
        {
            const std::string &vertexName = vertexPair.first;
            visited.emplace(std::make_pair(vertexName, false));
        }

        std::stack<std::string> priority_graph = topologicalSort();

        while (not(priority_graph.empty()))
        {
            auto vertexName = priority_graph.top();
            priority_graph.pop();

            if (visited[vertexName] == true)
                continue;

            Vertices group;
            generateGroupListUtil(vertexName, visited, group);

            groupList.emplace_back(group);
        }

        return groupList;
    }

    void DynamicGraph::print()
    {
        if (vertices_.empty())
            return;

        std::cout << "[DynamicGraph] vertices:" << std::endl;
        for (const auto &vertexPair : vertices_)
            std::cout << "\t" << vertexPair.second << std::endl;

        std::cout << "[DynamicGraph] adj_list:" << std::endl;
        for (const auto &adj_vertexPair : adj_list_)
        {
            std::cout << "\t[" << adj_vertexPair.first << "]" << std::endl;
            for (const auto &neighbor : adj_vertexPair.second)
                std::cout << "\t\t" << neighbor << std::endl;
        }

        std::cout << "[DynamicGraph] adj_priority_list:" << std::endl;
        for (const auto &adj_priority_vertexPair : adj_priority_list_)
        {
            std::cout << "\t[" << adj_priority_vertexPair.first << "]" << std::endl;
            for (const auto &lower_name : adj_priority_vertexPair.second)
                std::cout << "\t\t[" << lower_name << "]" << std::endl;
        }

        std::cout << std::endl;
    }

    void DynamicGraph::reset()
    {
        std::map<std::string, std::list<Vertex>> empty_graph;
        std::map<std::string, Vertex> empty_vertices;
        std::map<std::string, std::list<std::string>> empty_adj_priority;

        vertices_.swap(empty_vertices);
        adj_list_.swap(empty_graph);
        adj_priority_list_.swap(empty_adj_priority);

        assert(vertices_.empty());
        assert(adj_list_.empty());
        assert(adj_priority_list_.empty());
    }

    void DynamicGraph::topologicalSortUtil(
        std::string _name, std::map<std::string, bool> &_visited,
        std::stack<std::string> &_stack)
    {
        _visited[_name] = true;

        for (const auto &low : adj_priority_list_[_name])
        {
            if (_visited[low] == false)
                topologicalSortUtil(low, _visited, _stack);
        }

        _stack.push(_name);
    }

    void DynamicGraph::generateGroupListUtil(
        std::string _name, std::map<std::string, bool> &_visited,
        Vertices &_group)
    {
        _visited[_name] = true;

        for (const auto &neighbor : adj_list_[_name])
        {
            if (_visited[neighbor.name()] == false)
                generateGroupListUtil(neighbor.name(), _visited, _group);
        }

        _group.emplace(_name, vertices_[_name]);
    }
} // namespace multibot2_server::SubgoalGenerator