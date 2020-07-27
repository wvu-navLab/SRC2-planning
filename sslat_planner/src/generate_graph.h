#include <boost/config.hpp>

#include <math.h>
#include <algorithm>
#include <vector>
#include <utility>
#include <iostream>
#include <queue>
#include <boost/graph/visitors.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_utility.hpp>

extern std::vector<float> current_pos;
extern float R;
extern int n_levels;
extern int n_trunks;
extern int n_branches;

struct Vertex
{
    std::vector<float> state = {0,0};
    int parent_idx = -1;
    float cost_from_parent = 0.0;
    float cost_from_start = 0.0;
    std::vector<int> children_set;
    std::vector<int> traj_from_parent;
    std::vector<int> traj_from_start;
};

typedef boost::adjacency_list<
    boost::vecS, 
    boost::vecS, 
    boost::directedS,
    Vertex, boost::no_property> StarGraph;

void generate_graph(StarGraph & G, int level);
void tree_lattice(StarGraph & G, int level, std::size_t head_descriptor, int idx);
float vector_field_cost(StarGraph * G, std::size_t v1_descriptor, std::size_t v2_descriptor);
void bfs_calculate_costs(StarGraph & G);
void print_graph(StarGraph & G, bool printE, bool printV);
void triangulate_graph(StarGraph & G);
void plan(StarGraph & G);
inline float phi(float x1, float x2);
