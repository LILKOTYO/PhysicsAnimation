#include "SpinSim.h"

bool SpinSim::advance() {
	Eigen::Vector3d w = p_body->getAngularVelocity();
	
	// TODO
	// update orientation
	switch (m_method) {
	case 0: {
		// matrix-based angular velocity
		break;
	}

	case 1: {
		// quaternion-based angular velocity
		break;
	}
	default:
            std::cerr << m_method << " is not a valid rotation method."
                        << std::endl;
	}

	// advance time
	m_time += m_dt;
	m_step++;

	return false;
}