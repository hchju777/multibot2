#pragma once

#include <memory>
#include <list>
#include <map>
#include <stack>
#include <iostream>

#include "multibot2_server/subgoal_generator/vertex.h"
#include "multibot2_server/robot.h"

namespace multibot2_server::SubgoalGenerator
{
    class DynamicGraph
    {
    public:
        typedef std::unique_ptr<DynamicGraph> UniquePtr;
        typedef std::shared_ptr<DynamicGraph> SharedPtr;

    public:
        typedef std::map<std::string, Vertex> Vertices;

    public:
        DynamicGraph() { reset(); }

        DynamicGraph(const DynamicGraph &_dynamic_graph)
        {
            vertices_ = _dynamic_graph.vertices_;
            adj_list_ = _dynamic_graph.adj_list_;
            adj_priority_list_ = _dynamic_graph.adj_priority_list_;
        }

        ~DynamicGraph() {}

    public:
        Vertices &vertices() { return vertices_; }
        const Vertices &vertices() const { return vertices_; }

        std::map<std::string, std::list<Vertex>> &adj_list() { return adj_list_; }
        const std::map<std::string, std::list<Vertex>> &adj_list() const { return adj_list_; }

        std::map<std::string, std::list<std::string>> &adj_priority_list() { return adj_priority_list_; }
        const std::map<std::string, std::list<std::string>> &adj_priority_list() const { return adj_priority_list_; }

    public:
        void addVertices(const std::map<std::string, Robot> &_robots);

        std::stack<std::string> topologicalSort();

        std::list<Vertices> generateGroupList();

        void print();

    public:
        void reset();

    protected:
        void topologicalSortUtil(
            std::string _name, std::map<std::string, bool> &_visited,
            std::stack<std::string> &_stack);

        void generateGroupListUtil(
            std::string _name, std::map<std::string, bool> &_visited,
            Vertices &_group);

    public:
        DynamicGraph &operator=(const DynamicGraph &_rhs)
        {
            if (&_rhs != this)
            {
                vertices_ = _rhs.vertices_;
                adj_list_ = _rhs.adj_list_;
                adj_priority_list_ = _rhs.adj_priority_list_;
            }

            return *this;
        }

    protected:
        Vertices vertices_;
        std::map<std::string, std::list<Vertex>> adj_list_;
        std::map<std::string, std::list<std::string>> adj_priority_list_;

    }; // class DynamicGraph
} // namespace multibot2_server::SubgoalGenerator