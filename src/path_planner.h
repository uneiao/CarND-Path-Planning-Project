#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

struct InputState {
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;

    // Previous path data given to the Planner
    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;
    // Previous path's end s and d values
    double end_path_s;
    double end_path_d;
    std::vector<std::vector<double>> sensor_fusion;

    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;
};

class PathPlanner {

public:
    PathPlanner() {};
    ~PathPlanner() {};

    void Initialize();
    void GeneratePath(const InputState& input_state,
            std::vector<double>& next_x_vals, std::vector<double>& next_y_vals);

private:
    int LaneNumOf(double d);
    int LaneRelationBetween(double current_d, double target_d);
    bool IsChangingLane(const InputState& input_state);
    bool SelectLane(const InputState& input_state);

private:
    int m_changing_state;
    int m_heading_lane;
    double m_ref_v;
    double m_safe_gap;
};

#endif
