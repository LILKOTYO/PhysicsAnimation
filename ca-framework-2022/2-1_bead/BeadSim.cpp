#include "BeadSim.h"


bool BeadSim::advance() {


    auto v = p_bead->getLinearVelocity();
    auto p = p_bead->getPosition();

    // TODO update position and velcity of p_bead
    // constraint C(x) = 0.5*(p.dot(p) - m_radius^2) = 0;


    m_time += m_dt;
    m_step++;

    // log
    if ((m_step % m_log_frequency) == 0) {
        m_trajectories.back().push_back(p_bead->getPosition());
    }
    return false;

}