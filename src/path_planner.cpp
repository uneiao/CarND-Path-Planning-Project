#include "path_planner.h"
#include "helper.hpp"
#include "spline.h"


void PathPlanner::Initialize()
{
    m_ref_v = 0;
    m_changing_state = 0;

    m_heading_lane = 1;

    m_safe_gap = 30.0;
}

bool PathPlanner::IsChangingLane(const InputState& input_state)
{
    if (fabs(input_state.car_d - (2 + m_heading_lane * 4)) < 0.05) {
        m_changing_state = 0;
    }
    return m_changing_state ? true : false;
}

int PathPlanner::LaneNumOf(double d)
{
    return static_cast<int>(d / 4.0);
}

int PathPlanner::LaneRelationBetween(double current_d, double target_d)
{
    return LaneNumOf(target_d) - LaneNumOf(current_d);
}

bool PathPlanner::SelectLane(const InputState& input_state)
{
    size_t prev_size = input_state.previous_path_x.size() / 10;
    bool too_close = false;
    bool can_change = 0;
    double speed_limit[3] = {50, 50, 50};
    double min_s[3] = {m_safe_gap, m_safe_gap, m_safe_gap};
    //std::cout << "------" << std::endl;

    double self_car_s = input_state.car_s + 0.02 * prev_size * input_state.car_speed;

    for (int i = 0; i < input_state.sensor_fusion.size(); ++i) {
        const std::vector<double>& target = input_state.sensor_fusion[i];
        int lane_relation = LaneRelationBetween(input_state.car_d, target[6]);

        if (abs(lane_relation) == 2) {
            continue;
        }

        double vx = target[3];
        double vy = target[4];
        double check_speed = sqrt(vx * vx + vy * vy);
        double check_car_s = target[5];

        check_car_s += 0.02 * prev_size * check_speed;

        double sdiff = check_car_s - self_car_s;

        if (target[6] < 2 + 4 * m_heading_lane + 2 && target[6] > 2 + 4 * m_heading_lane - 2) {
            if (check_car_s > self_car_s
                    && check_speed < input_state.car_speed
                    && sdiff < m_safe_gap) {
                too_close = true;
            }
        }

        if (lane_relation == 0) {
            if (sdiff > m_safe_gap)
                continue;

            if (sdiff > 0 && sdiff < min_s[lane_relation + 1]) {
                min_s[lane_relation + 1] = sdiff;
                speed_limit[lane_relation + 1] = check_speed;
            }
            continue;
        }

        if (abs(lane_relation) == 1) {
            if (min_s[lane_relation + 1] == 0)
                continue;
            if (fabs(sdiff) > m_safe_gap)
                continue;
            if (target[5] - input_state.car_s > 0
                    && target[5] - input_state.car_s < m_safe_gap) {
                min_s[lane_relation + 1] = 0;
                speed_limit[lane_relation + 1] = 0;
                continue;
            }
            if (fabs(sdiff) < m_safe_gap / 2) {
                min_s[lane_relation + 1] = 0;
                speed_limit[lane_relation + 1] = 0;
                continue;
            }
            if (sdiff > 0 && sdiff < min_s[lane_relation + 1]) {
                min_s[lane_relation + 1] = sdiff;
                speed_limit[lane_relation + 1] = check_speed;
            }
            continue;
        }

    }
    int dlane = 0;
    double max_speed = speed_limit[0 + 1];
    /*
    std::cout << "mins" << " 0:" << min_s[0]
         << " 1:" << min_s[1]
         << " 2:" << min_s[2] << std::endl;
    std::cout << "speed" << " 0:" << speed_limit[0]
         << " 1:" << speed_limit[1]
         << " 2:" << speed_limit[2] << std::endl;
    */
    for (int i = -1; i < 2; i++) {
        if (m_heading_lane + i < 0 || m_heading_lane + i > 2) {
            continue;
        }
        if (min_s[i + 1] == 0) {
            continue;
        }
        if (speed_limit[i + 1] > max_speed) {
            max_speed = speed_limit[i + 1];
            dlane = i;
        }
    }
    if (dlane != 0 && input_state.car_speed > 30) {
        m_heading_lane += dlane;
        m_changing_state = true;
    }

    return too_close;
}

void PathPlanner::GeneratePath(const InputState& input_state,
        std::vector<double>& next_x_vals, std::vector<double>& next_y_vals)
{
    double car_x = input_state.car_x;
    double car_y = input_state.car_y;
    double car_yaw = input_state.car_yaw;
    double car_s = input_state.car_s;
    double car_d = input_state.car_d;
    double car_speed = input_state.car_speed;
    const std::vector<double>& previous_path_x = input_state.previous_path_x;
    const std::vector<double>& previous_path_y = input_state.previous_path_y;
    double end_path_s = input_state.end_path_s;
    double end_path_d = input_state.end_path_d;
    const std::vector<std::vector<double> >& sensor_fusion = input_state.sensor_fusion;

    const std::vector<double>& map_waypoints_x = input_state.map_waypoints_x;
    const std::vector<double>& map_waypoints_y = input_state.map_waypoints_y;
    const std::vector<double>& map_waypoints_s = input_state.map_waypoints_s;
    const std::vector<double>& map_waypoints_dx = input_state.map_waypoints_dx;
    const std::vector<double>& map_waypoints_dy = input_state.map_waypoints_dy;

    size_t prev_size = previous_path_x.size();

    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);


    if (prev_size > 0)
    {
        car_s = end_path_s;
    }

    bool too_close = false;
    if (IsChangingLane(input_state)) {
        for (int i = 0; i < sensor_fusion.size(); ++i) {
            double d = sensor_fusion[i][6];
            if (d < 2 + 4 * m_heading_lane + 2 && d > 2 + 4 * m_heading_lane - 2) {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s = sensor_fusion[i][5];

                check_car_s += 0.02 * prev_size * check_speed;

                if (check_car_s > car_s && check_car_s - car_s < m_safe_gap) {
                    too_close = true;
                }
            }
        }
    } else {
        too_close = SelectLane(input_state);
    }

    if (too_close) {
        m_ref_v -= 0.224;
    } else if (m_ref_v < 49.5) {
        m_ref_v += 0.224;
    }

    if (prev_size < 2) {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    } else {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    std::vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * m_heading_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    std::vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * m_heading_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    std::vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * m_heading_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    tk::spline s;
    s.set_points(ptsx, ptsy);

    next_x_vals.clear();
    next_y_vals.clear();

    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
        double N = target_dist / (0.02 * m_ref_v / 2.24);
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}

