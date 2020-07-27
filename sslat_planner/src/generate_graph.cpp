#include "generate_graph.h"
#include "include/delaunator-header-only.hpp"

std::vector<float> current_pos = {50,50};
float R = 0.5;
int n_levels = 4;
int n_trunks = 16;
int n_branches = 3;

template<typename T>
std::vector<float> linspace(T start_in, T end_in, int num_in)
{
    std::vector<float> linspaced;

    float start = static_cast<float>(start_in);
    float end = static_cast<float>(end_in);
    float num = static_cast<float>(num_in);

    if (num == 0) 
    { 
        return linspaced; 
    }
    if (num == 1) 
    {
        linspaced.push_back(start);
        return linspaced;
    }

    float delta = (end - start) / (num - 1);

    for(int i=0; i < num-1; ++i)
    {
        linspaced.push_back(start + delta * i);
    }
    
    linspaced.push_back(end); // I want to ensure that start and end
                                // are exactly the same as the input
    return linspaced;
}

template <typename T>
std::vector<std::vector<T> > GetUniqueRowsTol(std::vector<std::vector<T> > input, double tol)
{
    // std::sort(input.begin(), input.end());
    // input.erase(std::unique(input.begin(), input.end()), input.end());
    std::vector<std::vector<T> > temp;
    temp.push_back(input.front());

    for (size_t i = 0; i < input.size(); ++i)
    {
        double min_dist = 100;

        for (size_t j = 0; j < temp.size(); ++j)
        {
            double dx = input[i][0] -temp[j][0];
            double dy = input[i][1] -temp[j][1];

            double dist =  dx*dx + dy*dy;
            // std::cout << "dx " << dx << ", dy " << dy <<std::endl;
            if (dist < min_dist)
            {
                // std::cout << "dist " << std::hypot(dx, dy) <<std::endl;
                min_dist = dist;
            }
        }

        if (min_dist > tol)
        {
            temp.push_back({input[i][0], input[i][1]});
        }
    }
    return temp;
}

void generate_graph(StarGraph & G, int level)
{
    Vertex v_start;

    v_start.state[0]=current_pos[0];
    v_start.state[1]=current_pos[1];
    std::size_t v_start_descriptor = add_vertex(v_start, G);

    int lower_level = level-1;

    for (int i = 0; i < n_trunks; i++)
    {
        tree_lattice(G, lower_level, v_start_descriptor, i);
    }
}

void tree_lattice(StarGraph & G, int level, std::size_t head_descriptor, int idx)
{
    float r = R * pow(2,(n_levels-level));

    float theta, theta_head;

    if(head_descriptor != 1)
    {
        theta_head = atan2(G[head_descriptor].state[1]-current_pos[1],G[head_descriptor].state[0]-current_pos[0]);
        // Bernie Lattice
        theta = theta_head + 2*M_PI/(n_trunks*pow((n_branches-1),(n_levels-1-level)))*(idx-1);
        // Bethe Lattice
        // theta = theta_head + (2*pi/(nTrunks*nBranches^(nLevels-1-level))) * (nBranches/2 - length(S.container(head.idx).children_set) - 0.5);
    }    
    else
    {
        theta = 2*M_PI/n_trunks * (idx-1);
    }

    Vertex v_new;
    v_new.state[0] = current_pos[0] + r*cos(theta);
    v_new.state[1] = current_pos[1] + r*sin(theta);
    std::size_t v_new_descriptor = add_vertex(v_new, G);


    // if (head_descriptor != v_new_descriptor)
    // {
    // Create edge from head to v_new
    auto e = add_edge(head_descriptor, v_new_descriptor, G).first;
    // Add v_new to the list of neighbors of each element of head
    G[head_descriptor].children_set.push_back(v_new_descriptor);
    // }

    if (level == 0)
    {
        return;
    }

    int lower_level = level - 1;
    for (int i = 0; i<n_branches; i++)
    {
        tree_lattice(G, lower_level, v_new_descriptor, i);
    }
}

void bfs_calculate_costs(StarGraph & G)
{
    auto itr = vertices(G).first;
    
    // start bfs at u
    bool *visited = new bool[G.vertex_set().size()]; 

    for(int i = 0; i < G.vertex_set().size(); i++)
    {
        visited[i] = false;
    }

    std::queue<std::size_t> active;

    std::size_t start_descriptor = *itr; 
    
    visited[start_descriptor] = true;
    active.push(start_descriptor);

    while(!active.empty()) 
    { 
        // Dequeue a vertex from queue and print it 
        std::size_t head_descriptor = active.front();
        // std::cout << "Head descriptor: " << head_descriptor << std::endl;

        active.pop();

        // Get all adjacent vertices of the dequeued 
        // vertex s. If a adjacent has not been visited,  
        // then mark it visited and enqueue it 
        for (std::size_t i = 0; i < G[head_descriptor].children_set.size(); ++i)
        { 
            std::size_t child_descriptor = G[head_descriptor].children_set[i];
            if (!visited[child_descriptor]) 
            { 
                visited[child_descriptor] = true; 
                active.push(child_descriptor);

                double cost_from_parent = vector_field_cost(&G, head_descriptor, child_descriptor);
                // std::cout << G[head_descriptor].cost_from_start  << std::endl;

                G[child_descriptor].parent_idx = head_descriptor;
                G[child_descriptor].cost_from_parent = cost_from_parent; 
                G[child_descriptor].cost_from_start = G[head_descriptor].cost_from_start + cost_from_parent;
                G[child_descriptor].traj_from_start.push_back(child_descriptor);
            }
        } 
    } 
    // for (int i = 0; i<G.vertex_set().size(); i++)
    // {
    //     std::cout << visited[i] << std::endl;
    // }
}

void print_graph(StarGraph & G, bool printE, bool printV)
{
    if (printV)
    {
        auto vpair = vertices(G);
        std::cout << "num vertices = " << G.vertex_set().size() << std::endl;
        for(auto iter=vpair.first; iter!=vpair.second; iter++) 
        {
            std::cout << "vertex " << *iter << ", state = ("  << G[*iter].state[0] << "," << G[*iter].state[1] << ")."<< std::endl;
            std::cout << "parent " << G[*iter].parent_idx << ", cost = "  << G[*iter].cost_from_start << "."<< std::endl;
        }
    }
    
    if (printE)
    {
        auto epair = edges(G);
        for(auto iter=epair.first; iter!=epair.second; iter++) 
        {
            std::cout << "edge "<< (*iter) << ": " << source(*iter, G) << " - " << target(*iter, G) << std::endl;
        }
    }
}

void plan(StarGraph & G)
{

}

void triangulate_graph(StarGraph & G)
{
    std::vector<std::vector<double>> points;
    std::vector<double> coords;

    auto vpair = vertices(G);

    for(auto iter=vpair.first; iter!=vpair.second; iter++) 
    {
        points.push_back({G[*iter].state[0], G[*iter].state[1]});
    }

    points = GetUniqueRowsTol(points, 0.1);
    std::cout << "points unique = " << points.size() << std::endl;

    for (size_t i = 0; i < points.size(); ++i)
    {
        coords.push_back(points[i][0]);
        coords.push_back(points[i][1]);
    }

    delaunator::Delaunator d(coords);

    std::cout << "num triangles = " << d.triangles.size()/3 <<std::endl;
}

inline float phi(float x1, float x2) 
{ 
    return (pow((x1/50),4) - 1.2*pow((x1/50),2) * pow((x2/50),2) +  pow((x2/50),4) -1); 
} 

std::pair<float, float> square_vf(float x, float y)
{
    float phi_value = phi(x,y);
    
    float dx = 0.1;
    float dy = 0.1; 
    float dpdx = (phi(x+dx,y)-phi(x-dx,y))/(2*dx);
    float dpdy = (phi(x,y+dy)-phi(x,y-dy))/(2*dy);    
    
    float grad_norm = hypot(dpdx, dpdy);
    float g = 1/grad_norm;
    float h = 1/grad_norm;
    float G = -2/M_PI*atan(10*y);
    float H = 1;

    float u = g*G*dpdx + h*H*(-dpdy);
    float v = g*G*dpdy + h*H*dpdx;
    std::pair<float, float> f (u,v);

    return f;
}


float vector_field_cost(StarGraph * G, std::size_t v1_descriptor, std::size_t v2_descriptor)
{
    float c = 0;
    int N = 10;
    std::vector<float> x = linspace((*G)[v1_descriptor].state[0], (*G)[v2_descriptor].state[0], N+1);
    std::vector<float> y = linspace((*G)[v1_descriptor].state[1], (*G)[v2_descriptor].state[1], N+1);

    for(int i=1; i<x.size(); i++)
    {
        float xm = (x[i] + x[i-1])/2;
        float ym = (y[i] + y[i-1])/2;
        std::pair<float, float> f = square_vf(xm, ym);
        float u = f.first;
        float v = f.second;
        float g = hypot(u, v);
        c  = c - (u*(x[i]-x[i-1]) + v*(y[i]-y[i-1]))/g;
    }
    return c/2;
}



int main(int, char *[])
{
    StarGraph G;

    generate_graph(G, n_levels);
    bfs_calculate_costs(G);
    plan(G);
    triangulate_graph(G);
    print_graph(G, 0, 1);

    return 0;
}