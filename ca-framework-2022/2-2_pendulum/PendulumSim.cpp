#include "PendulumSim.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;

bool PendulumSim::advance() {
    auto pos1 = p1->getPosition();
    auto pos2 = p2->getPosition();
    auto vel1 = p1->getLinearVelocity();
    auto vel2 = p2->getLinearVelocity();

    // TODO update positions and velocities of particle p1, p2
    // c1 = 0.5 * (x1.dot(x1) - l1^2) = 0
    // c2 = 0.5 * ((xi-x2).dot(xi-x2) - l2^2) = 0



    // advance m_time
    m_time += m_dt;
    m_step++;

    // log
    if ((m_step % m_log_frequency) == 0) {
        m_trajectories[0].push_back(p1->getPosition());
        m_trajectories[1].push_back(p2->getPosition());
    }

    return false;
}