#include "path_planner.h"
#include "helper.hpp"


void PathPlanner::Initialize()
{
    return;
}


void PathPlanner::GeneratePath(const InputState& input_state,
        std::vector<double>& next_x_vals, std::vector<double>& next_y_vals)
{
    const double dist_inc = 0.5;
    for(int i = 0; i < 50; i++)
    {
        double next_s = input_state.car_s + (i + 1) * dist_inc;
        double next_d = 6;
        vector<double> xy = getXY(next_s, next_d,
                input_state.map_waypoints_s,
                input_state.map_waypoints_x,
                input_state.map_waypoints_y);
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }
}
