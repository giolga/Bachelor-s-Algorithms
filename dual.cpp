#include <iostream>
#include <map>
#include <utility>
#include <set>
#include <limits>
#include <string>
using namespace std;

const int INFINITY_METRIC = numeric_limits<int>::max();

// Routing table for a destination
struct RouteEntry {
    string destination_id; //R1, R2 ...
    string successor_id; // R1
    int feasible_distance;
    int reported_distance;

    bool is_active;
    //Neighbors we are waiting for replies from, if active...
    set<string> outstanding_query_replies;

    RouteEntry(string dest_id):
        destination_id(move(dest_id)),
        feasible_distance(INFINITY_METRIC),
        reported_distance(INFINITY_METRIC),
        is_active(false) {}
};

class Router {
public:
    string id; //R1
    map<string, Router*> neighbors_ptr;
    map<string, int> link_costs;
    map<string, map<string, int>> topology_table; // TD
    map<string, RouteEntry> routing_table; // Dest, Route entry for the router table
    static map<string, Router*> network_routers; // All routers in the network

    Router(string router_id) : id(move(router_id)) {
        network_routers[id] = this;
    }

    void add_link(const string& neighbor_id, int cost) {
        if(network_routers.count(neighbor_id)) {
            link_costs[neighbor_id] = cost;
            neighbors_ptr[neighbor_id] = network_routers[neighbor_id];

            cout << "[" << id << "] Link added to " << neighbor_id << " with cost " << cost << endl;
        }
        else {
            cout << "[" << "] Error: Cannot add link to unknown router " << neighbor_id << endl;
        }
    }

    void receive_update(const string& from_neighbor_id, const string& dest_id, int advertised_distance) {
        if(!link_costs.count(from_neighbor_id)) return; // Not a direct neighbor

        cout << "[" << id << "] RX UPDATE from " << from_neighbor_id << " for dest=" << dest_id << " with AD=" << (advertised_distance == INFINITY_METRIC ? "INF" : to_string(advertised_distance)) << endl;

        topology_table[dest_id][from_neighbor_id] = advertised_distance;
        //function compute paths for the destination
        compute_paths_for_destination(dest_id);
    }

    void compute_paths_for_destination(const string& dest_id) {

    }
};

int main() {

    return 0;
}