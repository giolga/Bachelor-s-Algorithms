#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <limits>
#include <algorithm>
#include <set>
using namespace std;


const int INFINITY_METRIC = numeric_limits<int>::max();

// Forward declaration
class Router;

// Information about a path to a destination as advertised by a neighbor
struct AdvertisedPathInfo {
    string advertising_neighbor_id;
    int advertised_distance; // The distance the neighbor reported to the destination

    AdvertisedPathInfo(string nid, int ad) : advertising_neighbor_id(move(nid)), advertised_distance(ad) {}
};

// Entry in the routing table for a destination
struct RouteEntry {
    string destination_id;
    string successor_id;
    int feasible_distance;  // Lowest RD ever recorded for this destination when a successor was chosen
    int reported_distance;  // The total distance to the destination via the current successor (Our_Link_Cost + Successor_AD)

    bool is_active;
    set<string> outstanding_query_replies; // Neighbors we are waiting for replies from if active

    RouteEntry(string dest_id = "") :
        destination_id(move(dest_id)),
        feasible_distance(INFINITY_METRIC),
        reported_distance(INFINITY_METRIC),
        is_active(false) {}
};

class Router {
public:
    string id;
    map<string, Router*> neighbors_ptr; // For direct calls in simulation
    map<string, int> link_costs;      // Cost to direct neighbors <neighbor_id, cost>

    // Topology Table: <destination_id, <neighbor_id, advertised_distance_from_that_neighbor>>
    map<string, map<string, int>> topology_table;

    // Routing Table: <destination_id, RouteEntry>
    map<string, RouteEntry> routing_table;

    // Network simulation: All routers in the network
    static map<string, Router*> network_routers;

    Router(string router_id) : id(move(router_id)) {
        network_routers[id] = this;
    }

    void add_link(const string& neighbor_id, int cost) {
        if (network_routers.count(neighbor_id)) {
            link_costs[neighbor_id] = cost;
            neighbors_ptr[neighbor_id] = network_routers[neighbor_id];
            cout << "[" << id << "] Link added to " << neighbor_id << " with cost " << cost << endl;
        } else {
            cout << "[" << id << "] Error: Cannot add link to unknown router " << neighbor_id << endl;
        }
    }

    // Called when an update is received from a neighbor
    void receive_update(const string& from_neighbor_id, const string& dest_id, int advertised_distance) {
        if (!link_costs.count(from_neighbor_id)) return; // Not a direct neighbor

        cout << "[" << id << "] RX UPDATE from " << from_neighbor_id
                  << " for dest=" << dest_id << " with AD="
                  << (advertised_distance == INFINITY_METRIC ? "INF" : to_string(advertised_distance)) << endl;

        topology_table[dest_id][from_neighbor_id] = advertised_distance;
        compute_paths_for_destination(dest_id);
    }

    // Called when a link cost changes or a neighbor goes down
    void process_link_change(const string& neighbor_id, int new_cost) {
        cout << "[" << id << "] LINK EVENT: Link to " << neighbor_id << " cost changed to "
                  << (new_cost == INFINITY_METRIC ? "INF (DOWN)" : to_string(new_cost)) << endl;

        link_costs[neighbor_id] = new_cost;

        if (new_cost == INFINITY_METRIC) {
            // Remove neighbor's advertisements from topology table
            for (auto& dest_entry : topology_table) {
                dest_entry.second.erase(neighbor_id);
            }
        }
        // Re-evaluate paths for all destinations, as costs might have changed
        // or a successor might have been lost.
        // A more optimized approach would only re-evaluate affected routes.
        for (auto const& [dest_id, entry] : routing_table) {
            compute_paths_for_destination(dest_id);
        }
         // If this router itself is a destination, it might need to advertise its new status (e.g. cost 0 to itself)
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


    void compute_paths_for_destination(const string& dest_id) {
        cout << "[" << id << "] Computing paths for dest=" << dest_id << endl;

        RouteEntry& route = routing_table[dest_id]; // Get or create entry
        route.destination_id = dest_id;

        string old_successor_id = route.successor_id;
        int old_reported_distance = route.reported_distance;

        string new_successor_id = "";
        int min_total_distance = INFINITY_METRIC; // This will be the new Reported Distance (RD)

        // Find the best path based on current topology_table info
        if (topology_table.count(dest_id)) {
            for (auto const& [neighbor_id, advertised_dist_from_neighbor] : topology_table.at(dest_id)) {
                if (link_costs.count(neighbor_id) && link_costs.at(neighbor_id) != INFINITY_METRIC &&
                    advertised_dist_from_neighbor != INFINITY_METRIC) {

                    // Do not consider paths through self to avoid loops from bad advertisements
                    if (neighbor_id == id && dest_id != id) continue;


                    int total_dist_via_neighbor = link_costs.at(neighbor_id) + advertised_dist_from_neighbor;
                    if (total_dist_via_neighbor < min_total_distance) {
                        min_total_distance = total_dist_via_neighbor;
                        new_successor_id = neighbor_id;
                    }
                }
            }
        }
        
        // Check for self-destination, always cost 0
        if (dest_id == id) {
             new_successor_id = id;
             min_total_distance = 0;
        }


        bool found_feasible_successor = false;
        if (!new_successor_id.empty()) {
            // Feasibility Condition (FC): AD of potential successor < current FD
            // The AD for the *new_successor_id* is topology_table[dest_id][new_successor_id]
            int ad_of_new_successor = (new_successor_id == id && dest_id == id) ? 0 : topology_table[dest_id][new_successor_id];

            if (ad_of_new_successor < route.feasible_distance) {
                found_feasible_successor = true;
            } else if (new_successor_id == route.successor_id && min_total_distance < route.reported_distance) {
                // If it's the same successor but the path got shorter, it's fine.
                found_feasible_successor = true;
            } else if (route.successor_id.empty() && min_total_distance != INFINITY_METRIC) {
                 // If no prior successor, any valid path can become successor and set the initial FD
                found_feasible_successor = true;
            }
        }


        if (found_feasible_successor) {
            // PASSIVE state logic
            route.is_active = false;
            route.outstanding_query_replies.clear();
            route.successor_id = new_successor_id;
            route.reported_distance = min_total_distance;

            // Feasible Distance (FD) is the *lowest recorded reported_distance*
            // It only updates if the new reported_distance is lower, or it's the first time.
            if (route.feasible_distance == INFINITY_METRIC || min_total_distance < route.feasible_distance) {
                 route.feasible_distance = min_total_distance;
            }
             // If successor itself, FD is 0
            if (new_successor_id == id && dest_id == id) route.feasible_distance = 0;


            cout << "[" << id << "] PASSIVE for dest=" << dest_id << ". Successor=" << route.successor_id
                      << ", RD=" << (route.reported_distance == INFINITY_METRIC ? "INF" : to_string(route.reported_distance))
                      << ", FD=" << (route.feasible_distance == INFINITY_METRIC ? "INF" : to_string(route.feasible_distance)) << endl;

            // If RD changed, advertise to neighbors
            if (route.reported_distance != old_reported_distance || route.successor_id != old_successor_id) {
                advertise_route_to_neighbors(dest_id, route.reported_distance);
            }

        } else { // No feasible successor found
            if (!route.is_active) { // Transition to ACTIVE
                cout << "[" << id << "] No Feasible Successor for dest=" << dest_id << ". Transitioning to ACTIVE." << endl;
                route.is_active = true;
                route.successor_id = ""; // Lose current successor
                // The old FD is maintained during the query process
                // route.reported_distance = INFINITY_METRIC; // Path is lost

                // Send queries to all neighbors (except ex-successor if it was the cause, more complex logic needed here)
                route.outstanding_query_replies.clear();
                bool sent_any_query = false;
                for (auto const& [neighbor_id_to_query, cost_to_neighbor] : link_costs) {
                    if (cost_to_neighbor != INFINITY_METRIC && neighbors_ptr.count(neighbor_id_to_query)) {
                         // Don't query neighbor if it was the one that caused us to lose the route (e.g. if it was our successor)
                         // And don't query if this neighbor is already known to be part of a path that doesn't meet original FD for complex loop scenarios.
                         // Simplified: query all available neighbors.
                        cout << "[" << id << "]   QUERYING " << neighbor_id_to_query << " for dest=" << dest_id << " (My FD=" << route.feasible_distance << ")" << endl;
                        neighbors_ptr[neighbor_id_to_query]->receive_query(id, dest_id, route.feasible_distance);
                        route.outstanding_query_replies.insert(neighbor_id_to_query);
                        sent_any_query = true;
                    }
                }
                 if (!sent_any_query) { // No one to query
                    cout << "[" << id << "] ACTIVE for dest=" << dest_id << ", but no neighbors to query. Marking unreachable." << endl;
                    route.is_active = false; // Back to passive, but unreachable
                    route.reported_distance = INFINITY_METRIC;
                    // FD might remain or be set to INF. DUAL has rules for this. For simplicity, if no path, RD is INF.
                    // If FD was based on a path now gone, it might eventually reset higher or to INF.
                    if (route.reported_distance != old_reported_distance || route.successor_id != old_successor_id) {
                         advertise_route_to_neighbors(dest_id, INFINITY_METRIC);
                    }
                 }

            } else { // Already ACTIVE
                 cout << "[" << id << "] Still ACTIVE for dest=" << dest_id << ". Waiting for replies." << endl;
            }
        }
    }

    void receive_query(const string& querying_neighbor_id, const string& dest_id, int originator_fd) {
        cout << "[" << id << "] RX QUERY from " << querying_neighbor_id << " for dest=" << dest_id
                  << " (Originator_FD=" << (originator_fd == INFINITY_METRIC ? "INF" : to_string(originator_fd)) << ")" << endl;

        // If this router is the destination itself
        if (dest_id == id) {
            cout << "[" << id << "]   This is me (" << dest_id << "). Replying with RD=0." << endl;
            if (neighbors_ptr.count(querying_neighbor_id)) {
                neighbors_ptr[querying_neighbor_id]->receive_reply(id, dest_id, 0);
            }
            return;
        }
        
        // If this router is active for this dest, it cannot reply authoritatively until it resolves its own state.
        // In a full DUAL, it might need to propagate the query if its own search constraints are affected.
        // Simplified: if active, it just holds off or re-evaluates if the query affects its own path.
        if (routing_table.count(dest_id) && routing_table[dest_id].is_active) {
            cout << "[" << id << "]   I am also ACTIVE for dest=" << dest_id << ". Cannot reply to " << querying_neighbor_id << " yet." << endl;
            // If querying_neighbor_id was my successor for dest_id, I need to recompute as that path is now invalid for reply.
             if (routing_table[dest_id].successor_id == querying_neighbor_id) {
                cout << "[" << id << "]   My successor " << querying_neighbor_id << " is querying. Path via it is now suspect for replying." << endl;
                // This might trigger re-computation or specific handling of diffusing computation.
                // For simplicity, we just note this. A re-computation might be needed.
                // compute_paths_for_destination(dest_id); // This could cascade if not careful
             }
            return;
        }

        // If passive, compute and send reply.
        // Re-run path computation to ensure up-to-date info for reply,
        // considering the query (e.g., if the querying neighbor was my successor).
        // This is a simplification. A full DUAL router doesn't just recompute on every query.
        // It checks its current state.
        compute_paths_for_destination(dest_id); // Ensure my own state is optimal before replying.

        int my_reported_distance_for_reply = INFINITY_METRIC;
        if (routing_table.count(dest_id) && !routing_table[dest_id].is_active && !routing_table[dest_id].successor_id.empty()) {
            my_reported_distance_for_reply = routing_table[dest_id].reported_distance;
        }

        cout << "[" << id << "]   REPLYING to " << querying_neighbor_id << " for dest=" << dest_id
                  << " with my RD=" << (my_reported_distance_for_reply == INFINITY_METRIC ? "INF" : to_string(my_reported_distance_for_reply)) << endl;

        if (neighbors_ptr.count(querying_neighbor_id)) {
            neighbors_ptr[querying_neighbor_id]->receive_reply(id, dest_id, my_reported_distance_for_reply);
        }
    }

    void receive_reply(const string& replying_neighbor_id, const string& dest_id, int replied_advertised_distance) {
        cout << "[" << id << "] RX REPLY from " << replying_neighbor_id << " for dest=" << dest_id
                  << " with AD=" << (replied_advertised_distance == INFINITY_METRIC ? "INF" : to_string(replied_advertised_distance)) << endl;

        if (!routing_table.count(dest_id) || !routing_table[dest_id].is_active) {
            cout << "[" << id << "]   Got REPLY, but not ACTIVE for dest=" << dest_id << ". Ignoring." << endl;
            return;
        }

        // Update topology table with this new info from the neighbor (this is an AD)
        topology_table[dest_id][replying_neighbor_id] = replied_advertised_distance;

        RouteEntry& route = routing_table[dest_id];
        route.outstanding_query_replies.erase(replying_neighbor_id);

        if (route.outstanding_query_replies.empty()) {
            cout << "[" << id << "]   All replies received for dest=" << dest_id << ". Recomputing." << endl;
            compute_paths_for_destination(dest_id); // This will try to find a successor and go passive
        } else {
            cout << "[" << id << "]   Still waiting for " << route.outstanding_query_replies.size() << " replies for dest=" << dest_id << "." << endl;
        }
    }

    void advertise_route_to_neighbors(const string& dest_id, int reported_distance) {
        cout << "[" << id << "] ADVERTISING route to dest=" << dest_id << " with RD="
                  << (reported_distance == INFINITY_METRIC ? "INF" : to_string(reported_distance))
                  << " to neighbors." << endl;
        for (auto const& [neighbor_id_to_adv, cost] : link_costs) {
            if (cost != INFINITY_METRIC && neighbors_ptr.count(neighbor_id_to_adv)) {
                 // Split Horizon: Do not advertise a route back to the neighbor from which it was learned (if that neighbor is the successor)
                 // Poison Reverse: Advertise it with INF metric if neighbor is successor.
                 // Simplified: send update. For more complex split horizon, check if neighbor_id_to_adv is current successor for dest_id.
                if (routing_table.count(dest_id) && routing_table[dest_id].successor_id == neighbor_id_to_adv && neighbor_id_to_adv != id) {
                    // Poison reverse (optional, EIGRP does split horizon by default, not poison reverse in this way for updates)
                    // cout << "[" << id << "]   (Poison Reverse) Not sending update for " << dest_id << " to my successor " << neighbor_id_to_adv << endl;
                    // Or send INF: neighbors_ptr[neighbor_id_to_adv]->receive_update(id, dest_id, INFINITY_METRIC);
                    // EIGRP's split horizon means you don't advertise it if it was learned from that neighbor.
                    // Here, we advertise our *own calculated RD*, not just pass through.
                    // So, if neighbor_id_to_adv is my current successor for dest_id, I should not advertise this path back to it
                    // unless it's a specific EIGRP rule (e.g. for summary routes or specific query/reply contexts).
                    // For simplicity, let's just advertise unless it's sending to self about self.
                    if (neighbor_id_to_adv == id && dest_id == id) continue;

                     // A common split horizon rule: if the neighbor I'm about to update IS my successor for this route, don't update it.
                    if (routing_table.count(dest_id) && routing_table.at(dest_id).successor_id == neighbor_id_to_adv) {
                        cout << "[" << id << "]   (Split Horizon) Not sending update for " << dest_id << " to my successor " << neighbor_id_to_adv << endl;
                        continue;
                    }
                }
                neighbors_ptr[neighbor_id_to_adv]->receive_update(id, dest_id, reported_distance);
            }
        }
    }


    void print_routing_table_summary() {
        cout << "\n--- Routing Table Summary for " << id << " ---" << endl;
        for (auto const& [dest_id, entry] : routing_table) {
             if (entry.reported_distance == INFINITY_METRIC && entry.successor_id.empty() && !entry.is_active) continue; // Skip truly unreachables unless active

            cout << "Dest: " << dest_id
                      << " | Succ: " << (entry.successor_id.empty() ? "None" : entry.successor_id)
                      << " | RD: " << (entry.reported_distance == INFINITY_METRIC ? "Inf" : to_string(entry.reported_distance))
                      << " | FD: " << (entry.feasible_distance == INFINITY_METRIC ? "Inf" : to_string(entry.feasible_distance))
                      << " | State: " << (entry.is_active ? "ACTIVE (wait:" + to_string(entry.outstanding_query_replies.size()) + ")" : "Passive")
                      << endl;
        }
        cout << "------------------------------------" << endl;
    }
};

map<string, Router*> Router::network_routers;


int main() {
    // Create routers
    Router r1("R1");
    Router r2("R2");
    Router r3("R3");
    Router r4("R4"); // A destination network connected to R3

    // Setup initial links for R1, R2, R3
    // R1 --1-- R2 --1-- R3 --1-- R4(Dest)
    r1.add_link("R2", 1);
    r2.add_link("R1", 1);
    r2.add_link("R3", 1);
    r3.add_link("R2", 1);
    r3.add_link("R4", 1); // R4 is a "destination" node/network

    // Routers advertise themselves (cost 0 to self)
    // And R3 advertises its direct connection to R4
    r1.process_link_change("R1",0); // Effectively advertises itself
    r2.process_link_change("R2",0);
    r3.process_link_change("R3",0);
    r4.process_link_change("R4",0); // R4 is a node, advertises itself with cost 0
    
    // R3 is directly connected to "destination R4"
    // R3 learns about R4 (simulated as R4 advertising itself)
    // R3 should then advertise its path to R4 to its neighbors (R2)
    // Let's simulate R4 (as a simple node) being advertised by R3
    cout << "\n--- Initial Route Propagation for R4 ---" << endl;
    r3.receive_update("R4" /*from R4 itself, conceptual*/, "R4", 0); // R3 learns R4 is 0 hops from R4. Cost from R3 to R4 is link_cost (1)
                                                                // so R3's RD to R4 becomes 1.
    r3.compute_paths_for_destination("R4");

    r1.print_routing_table_summary();
    r2.print_routing_table_summary();
    r3.print_routing_table_summary();
    r4.print_routing_table_summary();


    cout << "\n--- Scenario: Link R2-R3 goes down ---" << endl;
    r2.process_link_change("R3", INFINITY_METRIC);
    r3.process_link_change("R2", INFINITY_METRIC); // Symmetric link failure


    r1.print_routing_table_summary();
    r2.print_routing_table_summary();
    r3.print_routing_table_summary();
    r4.print_routing_table_summary();

    // Example: R1 wants to reach R4. After R2-R3 fails, R2 goes active.
    // If R1 was using R2 for R4, R1 might also go active if R2 tells it R4 is unreachable or RD increases beyond FD.

    // What if R3 gets a new path to R4 later, e.g. R3-R5-R4?
    // Router r5("R5");
    // r3.add_link("R5", 2);
    // r5.add_link("R3", 2);
    // r5.add_link("R4", 1);
    // r5.receive_update("R4", "R4", 0); // R5 learns R4
    // r5.compute_paths_for_destination("R4"); // R5 calculates its path to R4, advertises to R3
    // r3.compute_paths_for_destination("R4"); // R3 re-evaluates
    // This would trigger new updates.

    return 0;
}