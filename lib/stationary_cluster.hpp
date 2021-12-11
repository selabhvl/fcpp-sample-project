// Copyright © 2021 Giorgio Audrito & Volker Stolz. All Rights Reserved.

/**
 * @file spreading_collection.hpp
 * @brief Simple composition of spreading and collection functions.
 *
 * This header file is designed to work under multiple execution paradigms.
 * Contributions by Quentin James.
 */

#ifndef FCPP_STATIONARY_CLUSTER_H_
#define FCPP_STATIONARY_CLUSTER_H_


#include "lib/fcpp.hpp"
#include <iostream>

using namespace std;



/**
 * @brief Namespace containing all the objects in the FCPP library.
 */
namespace fcpp {

// bool operator<(vec<2>, vec<2>) {
//     return true;
// }

// template <size_t n, size_t cx, size_t cy> DECLARE_OPTIONS(cluster, spawn_schedule<sequence::multiple_n<n, 0>>, init<...>);


//! @brief Minimum number whose square is at least n.
constexpr size_t discrete_sqrt(size_t n) {
    size_t lo = 0, hi = n, mid = 0;
    while (lo < hi) {
        mid = (lo + hi)/2;
        if (mid*mid < n) lo = mid+1;
        else hi = mid;
    }
    return lo;
}
//! @brief The final simulation time.
constexpr size_t end_time = 300;
//! @brief Number of devices.
constexpr size_t devices = 10;
//! @brief Communication radius.
constexpr size_t comm = 50;
//! @brief Dimensionality of the space.
constexpr size_t dim = 3;
//! @brief Side of the deployment area.
constexpr size_t side = discrete_sqrt(devices * 3000);
//! @brief Height of the deployment area.
constexpr size_t height = 100;
//! @brief Color hue scale.
constexpr float hue_scale = 360.0f/(side+height);
//! @brief Canvas size.
constexpr int canvas_width = 100;
constexpr int canvas_height = 100;



//! @brief Namespace containing the libraries of coordination routines.
namespace coordination {


//! @brief Tags used in the node storage.
namespace tags {
    //! @brief The device movement speed.
    struct speed {};
    //! @brief True distance of the current node from the source.
    struct true_distance {};
    //! @brief Computed distance of the current node from the source.
    struct calc_distance {};
    //! @brief Diameter of the network (in the source).
    struct source_diameter {};
    //! @brief Diameter of the network (in every node).
    struct diameter {};
    //! @brief Color representing the distance of the current node.
    struct distance_c {};
    //! @brief Color representing the diameter of the network (in the source).
    struct source_diameter_c {};
    //! @brief Color representing the diameter of the network (in every node).
    struct diameter_c {};
    //! @brief Size of the current node.
    struct node_size {};
    //! @brief Shape of the current node.
    struct node_shape {};
    //! @brief Number of neighbour nodes (Q).
    struct neighbours {};
    //! @brief Node type (cluster or non-custer).
    struct cluster {};
    //! @brief Node alarm
    struct alarm {};
}

//! @brief Function to change node colour
FUN void change_colour(ARGS, int h, int s, int v) {
    node.storage(tags::diameter_c{})        = color::hsva(h, s, v);
    node.storage(tags::distance_c{})        = color::hsva(h, s, v);
    node.storage(tags::source_diameter_c{}) = color::hsva(h, s, v);
}      

//! @brief Function selecting a source based on the current time
FUN bool select_source(ARGS, int step) { CODE
    // store relevant values in the node storage
    node.storage(tags::node_size{})         = 10;
    node.storage(tags::node_shape{})        = node.uid == 0 ? shape::star : shape::sphere;
    return true;
}
//! @brief Export types used by the select_source function (none).
FUN_EXPORT select_source_t = common::export_list<>;


// define a path
std::array<vec<3>, 2> path = {{
    {canvas_width, canvas_height, 0}
}};
real_t r2 = 1.2;    // speed
real_t period = 1; 

// TODO 
// turn colours back ON (red & green for cluster members)
// snowflake starts out green -> turn blue upon alarm firing
// snowflake becomes red within some dist within cluster
// everyone broadcasts their pos and if they are in cluster or not
// if snowflake has neighbors in cluster, ID the closest one and check if dist < some d



//! @brief Main function.
MAIN() {

    if (node.uid != 0) {
        rectangle_walk(CALL, make_vec(35, 35, 0), make_vec(65, 65, 0), node.storage(tags::speed{}), 1);
    }

    // make node 0 follow the L-shaped path
    // if (node.uid == 0) {
    //     follow_path(CALL, path, r2, period);
    // }

    // field<bool> nbr_cluster = nbr(CALL, node.storage(tags::cluster{}));
    // field<real_t> cluster_distances = mux(nbr_cluster, node.nbr_dist(), INF);
    // tuple<real_t vec<2>> min_val_pos = min_hood(CALL, tuple(cluster_distances, node.nbr_vec()));
    // node.storage(alarm{}) = (not node.storage(tags::cluster{})) and get<0>(min_val_pos < 1);

    // if (node.storage(tags::alarm{})) {
    // }


    // selects a different source every 50 simulated seconds
    bool is_source = select_source(CALL, 50);
    // calculate distances from the source

}

//! @brief Export types used by the main function.
FUN_EXPORT main_t = common::export_list<rectangle_walk_t<3>, select_source_t, abf_distance_t, mp_collection_t<double, double>, broadcast_t<double, double>, follow_path_t>;


} // namespace coordination


//! @brief Namespace for component options.
namespace option {


//! @brief Import tags to be used for component options.
using namespace component::tags;
//! @brief Import tags used by aggregate functions.
using namespace coordination::tags;


//! @brief The randomised sequence of rounds for every node (about one every second, with 10% variance).
using round_s = sequence::periodic<
    distribution::interval_n<times_t, 0, 1>,       // uniform time in the [0,1] interval for start
    distribution::weibull_n<times_t, 10, 1, 10>,   // weibull-distributed time for interval (10/10=1 mean, 1/10=0.1 deviation)
    distribution::constant_n<times_t, end_time+2>  // the constant end_time+2 number for end
>;
//! @brief The sequence of network snapshots (one every simulated second).
using log_s = sequence::periodic_n<1, 0, 1, end_time>;
//! @brief The sequence of node generation events (multiple devices all generated at time 0).
using spawn_s1 = sequence::multiple_n<devices, 0>;
using spawn_s2 = sequence::multiple_n<devices, 0>;
//! @brief The distribution of initial node positions (random in a given rectangle).
using rectangle_d1 = distribution::rect_n<1, 0, 0, 0, side, side, height>;
//! @brief The distribution of initial node positions (random in a given rectangle).
using rectangle_d2 = distribution::rect_n<1, 0, 0, 0, 10, 10, 10>;
//! @brief The distribution of node speeds (all equal to a fixed value).
using speed_d = distribution::constant_i<double, speed>;
//! @brief The contents of the node storage as tags and associated types.
using store_t = tuple_store<
    speed,              double,
    true_distance,      double,
    calc_distance,      double,
    source_diameter,    double,
    diameter,           double,
    distance_c,         color,
    source_diameter_c,  color,
    diameter_c,         color,
    node_shape,         shape,
    node_size,          double,
    neighbours,         int,
    cluster,            bool,
    alarm,              bool
>;
//! @brief The tags and corresponding aggregators to be logged.
using aggregator_t = aggregators<
    true_distance,      aggregator::max<double>,
    diameter,           aggregator::combine<
                            aggregator::min<double>,
                            aggregator::mean<double>,
                            aggregator::max<double>
                        >
>;
//! @brief The aggregator to be used on logging rows for plotting.
using row_aggregator_t = common::type_sequence<aggregator::mean<double>>;
//! @brief The logged values to be shown in plots as lines (true_distance, diameter).
using points_t = plot::values<aggregator_t, row_aggregator_t, true_distance, diameter>;
//! @brief A plot of the logged values by time for speed = 50 (intermediate speed).
using time_plot_t = plot::split<plot::time, plot::filter<speed, filter::equal<comm/4>, points_t>>;
//! @brief A plot of the logged values by speed for times >= 50 (after the first source switch).
using speed_plot_t = plot::split<speed, plot::filter<plot::time, filter::above<50>, points_t>>;
//! @brief Combining the two plots into a single row.
using plot_t = plot::join<time_plot_t, speed_plot_t>;

template <size_t sx, size_t sy, size_t vx, size_t vy> 
DECLARE_OPTIONS(snowflake, 
                spawn_schedule<sequence::multiple_n<1, 0>>, 
                init<   
                    x, distribution::rect_n<1, sx, sy, 0, sx, sy, 0>, 
                    // speed,  distribution::constant_i<double, speed>>,
                    v, distribution::rect_n<1, vx, vy, 0, vx, vy, 0>
                >
                );

template <size_t n, size_t cx1, size_t cy1, size_t cx2, size_t cy2> 
DECLARE_OPTIONS(cluster, 
                spawn_schedule<sequence::multiple_n<n, 0>>, 
                init<   
                    x, distribution::rect_n<1, cx1, cy1, 0, cx2, cy2, 0>, 
                    speed,  distribution::constant_i<double, speed>>
                );

//! @brief The general simulation options.
DECLARE_OPTIONS(list,
    parallel<false>,     // no multithreading on node rounds
    synchronised<false>, // optimise for asynchronous networks
    program<coordination::main>,   // program to be run (refers to MAIN above)
    exports<coordination::main_t>, // export type list (types used in messages)
    round_schedule<round_s>, // the sequence generator for round events on nodes
    log_schedule<log_s>,     // the sequence generator for log events on the network
    store_t,       // the contents of the node storage
    aggregator_t,  // the tags and corresponding aggregators to be logged
	snowflake<0, 0, 20, 20>,
    cluster<devices, 35, 35, 65, 65>,
    extra_info<speed, double>, // use the globally provided speed for plotting
    plot_type<plot_t>,         // the plot description to be used
    dimension<dim>, // dimensionality of the space
    connector<connect::fixed<comm, 1, dim>>, // connection allowed within a fixed comm range
    shape_tag<node_shape>, // the shape of a node is read from this tag in the store
    size_tag<node_size>,   // the size of a node is read from this tag in the store
    color_tag<distance_c, source_diameter_c, diameter_c>, // colors of a node are read from these
    area<0, 0, canvas_width, canvas_height>
);


} // namespace option


} // namespace fcpp


#endif // FCPP_STATIONARY_CLUSTER_H_
