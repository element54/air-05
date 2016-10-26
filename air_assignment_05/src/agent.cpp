/*
* Daniel Vazquez
* Aritificial Intelligence for Robotics
* SS 2016
* Assignment 5
*
* agent.cpp
* */

#include "agent.hpp"

#include <iostream>
#include <string>
#include <unistd.h>
#include <chrono>
#include <thread>

#include <algorithm>

#define map_rows 25
#define map_cols 141

using namespace std;

Agent::Agent(vector<vector<string> > selected_map, const pair<int, int> initial_pos, int number_of_goals):
empty_map(selected_map),
map(selected_map),
initial_pos(initial_pos),
number_of_goals(number_of_goals),
max_number_of_stored_nodes(0),
number_of_visited_nodes(0),
total_of_stored_nodes(0),
deepest_level(0)
{

    max_limit = map_rows * map_cols;
    print_map(empty_map);
}

Agent::~Agent()
{
}

void Agent::run()
{
    cout << "Running IDFS " << endl;
    cout << "Number of goals " << number_of_goals << endl;
    sleep(1);
    iterative_deepening_search();
}

void Agent::print_map(vector<vector<string> >& a_map)
{
    system("clear");

    for(int row = 0; row < map_rows; row++)
    {
        for(int col = 0; col < map_cols; col++)
        {
            cout << a_map[row][col];
        }
        cout << endl;
    }

    this_thread::sleep_for(chrono::milliseconds(10));
}


bool Agent::recursive_dls(pair<int, int> current_node,
                          int goal, int current_level,
                          int limit, vector<pair<int, int> > current_path,
                          vector<pair<int, int> > &explored_nodes,
                          pair<int, int> &new_node) {
    //TODO

    //Notes:
    //Backtrack from here once you have found a goal.
    //If you have found a goal, do not forget to get a fresh copy of the map.
    //Stop searching if you have found a goal or reached the depth limit.
    //Only return true if a goal has been found.

    if(current_level >= limit) {
        return false;
    }
    if(std::find(current_path.begin(), current_path.end(), current_node) != current_path.end()) {
        return false;
    }
    if(std::find(explored_nodes.begin(), explored_nodes.end(), current_node) != explored_nodes.end()) {
        return false;
    }
    int row = current_node.first;
    int col = current_node.second;
    if(!is_free(row, col)) {
        return false;
    }

    explored_nodes.push_back(current_node);
    current_path.push_back(current_node);

    string data = map[row][col];
    if(data.at(0)  - '0' == goal) {
        new_node = current_node;
        backtrack_path(current_path);
        this_thread::sleep_for(chrono::milliseconds(2000));
        return true;
    }
    //backtrack_path(current_path);


    bool res = recursive_dls(make_pair(row, col + 1), goal, current_level + 1, limit, current_path, explored_nodes, new_node);
    if(!res) {
        res = recursive_dls(make_pair(row + 1, col), goal, current_level + 1, limit, current_path, explored_nodes, new_node);
    }
    if(!res) {
        res = recursive_dls(make_pair(row, col - 1), goal, current_level + 1, limit, current_path, explored_nodes, new_node);
    }
    if(!res) {
        res = recursive_dls(make_pair(row - 1, col), goal, current_level + 1, limit, current_path, explored_nodes, new_node);
    }
    return res;
}

bool Agent::depth_limited_seach(pair<int, int> current_node, int goal, int limit, pair<int, int> &new_node) {
    vector<pair<int, int>> current_path;
    vector<pair<int, int>> explored_nodes;
    return recursive_dls(current_node, goal, 0, limit, current_path, explored_nodes, new_node);
}

void Agent::iterative_deepening_search() {
    pair<int, int> current_node = initial_pos;
    pair<int, int> new_node = initial_pos;
    for(uint i = 1;i <= number_of_goals;i++) {
    for(int limit = 1; limit < 1000; limit++) {
        //cout << "Limit: " << limit << endl;
        bool res = depth_limited_seach(new_node, i, limit, new_node);
        if(res) {
            break;
        }
    }
    }
    print_final_results();
}

void Agent::print_final_results() {
    cout << "Deepest level reached: " << deepest_level  << endl;
    cout << "Total of stored nodes: " << total_of_stored_nodes << endl;
    cout << "Total of visited nodes: " << number_of_visited_nodes << endl;
}

void Agent::backtrack_path(vector<pair<int, int> > current_path) {
    //use the original map to backtrace
    vector<vector<string> > local_map = empty_map;

    //Backtrace. Use the current path vector to set the path on the map.
    for (auto &node : current_path) {
        int row = node.first;
        int col = node.second;
        if(!is_goal(row, col) && local_map[row][col] != "s") {
            local_map[row][col] = "-";
        }
    }

    //print backtraced path
    print_map(local_map);
    //this_thread::sleep_for(chrono::milliseconds(2000));
}

bool Agent::is_free(int row, int col) {
    return is_free(map[row][col]);
}

bool Agent::is_free(string str) {
    return (str == " " || str == "s" || is_goal(str));
}

bool Agent::is_goal(int row, int col) {
    return is_goal(map[row][col]);
}
bool Agent::is_goal(string str) {
    return isdigit(str.at(0));
}
