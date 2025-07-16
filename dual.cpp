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


    void process_link_change(const string& neighbor_id, const int new_cost) {
        cout << "[" << id << "] LINK EVENT: Link to " << neighbor_id << " cost changed to " << (new_cost == INFINITY_METRIC ? "INF (DOWN)" : to_string(new_cost)) << endl;

        link_costs[neighbor_id] = new_cost;

        // Delete neighbor's advertisments from TT
        if(new_cost == INFINITY_METRIC){
            for(auto& dest_entry: topology_table) {
                dest_entry.second.erase(neighbor_id);
            }
        }

        for(auto const& [dest_id, entry] : topology_table) {
            compute_paths_for_destination(dest_id);
        }

        // If router itself is a destination, it might need to advertise its new status (e.g. cost 0 to itself)
        if (routing_table.find(id) == routing_table.end() || routing_table[id].reported_distance != 0) {
            topology_table[id][id] = 0; // Special case for self
            RouteEntry& self_route = routing_table[id];
            self_route.destination_id = id;
            self_route.successor_id = id; // Self
            self_route.reported_distance = 0;
            self_route.feasible_distance = 0;
            self_route.is_active = false;
            advertise_route_to_neighbors(id, 0);
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

    void advertise_route_to_neighbors(const string& dest_id, int reported_distance) {

    }
};

int main() {
    Router r1("R1");
    Router r2("R2");
    Router r3("R3");
    Router r4("R4");

    r1.add_link("R2", 1);
    r2.add_link("R1", 1);
    r2.add_link("R3", 1);
    r3.add_link("R2", 1);
    r3.add_link("R4", 1);

    r1.process_link_change("R1",0);
    r2.process_link_change("R2",0);
    r3.process_link_change("R3",0);
    r4.process_link_change("R4",0);

    return 0;
}