#include "CannonBallSim.h"


bool CannonBallSim::advance() {
    // perform time integration with different integrators

	// use p_ball, m_dt, m_gravity
	Eigen::Vector3d v = p_ball->getLinearVelocity();
	Eigen::Vector3d p = p_ball->getPosition();

    // TODO
    switch (m_method) {
        case 0:
            // analytical solution
			// p(t) = v_0*t + 0.5*a*t^2
            break;

        case 1:
            // explicit euler
			// p' = p + dt*v
			// v' = v + dt*a
            break;

        case 2:
            // symplectic euler
			// v' = v + dt*a
			// p' = p + dt*v'
            break;

        default:
            std::cerr << m_method << " is not a valid integrator method."
                        << std::endl;
    }

    // advance time
    m_time += m_dt;
    m_step++;

    // log
    if ((m_step % m_log_frequency) == 0) {
        m_trajectories.back().push_back(p_ball->getPosition());
    }

    return false;
}