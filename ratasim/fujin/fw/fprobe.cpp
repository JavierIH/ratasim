// The simulator's look at what Fujin's firmware believes (its navigation's public getters): for diagnostics only.
#include "services/navigation.hpp"

extern "C" void fsim_nav(float *out) {
    auto nav = services::Navigation::instance();
    const auto pos = nav->get_robot_position_mm();
    const auto cell = nav->get_robot_cell_position();
    out[0] = nav->get_robot_travelled_dist_mm();
    out[1] = pos.x;
    out[2] = pos.y;
    out[3] = cell.x;
    out[4] = cell.y;
    out[5] = (float)static_cast<int>(nav->get_robot_direction());
}
