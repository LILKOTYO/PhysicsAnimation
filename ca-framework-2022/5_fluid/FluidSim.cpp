#include "FluidSim.h"

void FluidSim::solvePoisson() {
	double dx2 = m_dx * m_dx;
	double residual = m_acc + 1; // initial residual
	double rho = 1;

	Array2d& p = p_pressure->x();

	for (int it = 0; residual > m_acc && it < m_iter; ++it) {
		// Note that the boundaries are handles by the framework, so you iterations should be similar to:
		for (int y = 1; y < m_res_y - 1; ++y) {
			for (int x = 1; x < m_res_x - 1; ++x) {
				double b = -p_divergence->x()(x, y) / m_dt * rho; // right-hand
				// TODO: update the pressure values
				p(x, y) = p(x, y);
			}
		}

		// Compute the new residual, i.e. the sum of the squares of the individual residuals (squared L2-norm)
		residual = 0;
		for (int y = 1; y < m_res_y - 1; ++y) {
			for (int x = 1; x < m_res_x - 1; ++x) {
				double b = -p_divergence->x()(x, y) / m_dt * rho; // right-hand
				// TODO: compute the cell residual
				double cellResidual = 0;

				residual += cellResidual * cellResidual;
			}
		}

		// Get the L2-norm of the residual
		residual = sqrt(residual);

		// We assume the accuracy is meant for the average L2-norm per grid cell
		residual /= (m_res_x - 2) * (m_res_y - 2);

		// For your debugging, and ours, please add these prints after every iteration
		if (it == m_iter - 1)
			printf("Pressure solver: it=%d , res=%f \n", it, residual);
		if (residual < m_acc)
			printf("Pressure solver: it=%d , res=%f, converged \n", it, residual);
	}
}

void FluidSim::correctVelocity() {
	Array2d& p = p_pressure->x();
	Array2d& u = p_velocity->x();	// x velocity
	Array2d& v = p_velocity->y();	// y velocity

	// Note: velocity u_{i+1/2}
	for (int y = 1; y < m_res_y - 1; ++y)
		for (int x = 1; x < m_res_x; ++x)
			// TODO: update u
			u(x, y) = u(x, y);

	// Same for velocity v_{i+1/2}.
	for (int y = 1; y < m_res_y; ++y)
		for (int x = 1; x < m_res_x - 1; ++x)
			// TODO: update v
			v(x, y) = v(x, y);
}

void FluidSim::advectValues() {
	// Densities live on the grid centers, the velocities on the MAC grid
	// Separate their computation to avoid confusion

	Array2d& d = p_density->x();
	Array2d& u = p_velocity->x();
	Array2d& v = p_velocity->y();

	Array2d& d_ = p_density_tmp->x();
	Array2d& u_ = p_velocity_tmp->x();
	Array2d& v_ = p_velocity_tmp->y();

	// Densities, grid centers
	for (int y = 1; y < m_res_y - 1; ++y) {
		for (int x = 1; x < m_res_x - 1; ++x) {
			// TODO: Compute the velocity
			double last_x_velocity = 0;
			double last_y_velocity = 0;

			// TODO: Find the last position of the particle (in grid coordinates)
			double last_x = 0;
			double last_y = 0;

			// Make sure the coordinates are inside the boundaries
			// Densities are known between 1 and res-2
			if (last_x < 1) last_x = 1;
			if (last_y < 1) last_y = 1;
			if (last_x > m_res_x - 2) last_x = m_res_x - 2;
			if (last_y > m_res_y - 2) last_y = m_res_y - 2;

			// Determine corners for bilinear interpolation
			int x_low = (int)last_x;
			int y_low = (int)last_y;
			int x_high = x_low + 1;
			int y_high = y_low + 1;

			// Compute the interpolation weights
			double x_weight = last_x - x_low;
			double y_weight = last_y - y_low;

			// TODO: Bilinear interpolation
			d_(x, y) = d(x, y);
		}
	}

	// Velocities (u), MAC grid
	for (int y = 1; y < m_res_y - 1; ++y) {
		for (int x = 1; x < m_res_x; ++x) {
			// TODO: Compute the velocity
			double last_x_velocity = 0;
			double last_y_velocity = 0;

			// TODO: Find the last position of the particle (in grid coordinates)
			double last_x = 0;
			double last_y = 0;

			// Make sure the coordinates are inside the boundaries
			// Being conservative, one can say that the velocities are known between 1.5 and res-2.5
			// (the MAC grid is inside the known densities, which are between 1 and res - 2)
			if (last_x < 1.5) last_x = 1.5;
			if (last_y < 1.5) last_y = 1.5;
			if (last_x > m_res_x - 1.5) last_x = m_res_x - 1.5;
			if (last_y > m_res_y - 2.5) last_y = m_res_y - 2.5;

			// Determine corners for bilinear interpolation
			int x_low = (int)last_x;
			int y_low = (int)last_y;
			int x_high = x_low + 1;
			int y_high = y_low + 1;

			// Compute the interpolation weights
			double x_weight = last_x - x_low;
			double y_weight = last_y - y_low;

			// TODO: Bilinear interpolation
			u_(x, y) = u(x, y);
		}
	}

	// Velocities (v), MAC grid
	for (int y = 1; y < m_res_y; ++y) {
		for (int x = 1; x < m_res_x - 1; ++x) {
			// TODO: Compute the velocity
			double last_x_velocity = 0;
			double last_y_velocity = 0;

			// TODO: Find the last position of the particle (in grid coordinates)
			double last_x = 0;
			double last_y = 0;

			// Make sure the coordinates are inside the boundaries
			// Being conservative, one can say that the velocities are known between 1.5 and res-2.5
			// (the MAC grid is inside the known densities, which are between 1 and res - 2)
			if (last_x < 1.5) last_x = 1.5;
			if (last_y < 1.5) last_y = 1.5;
			if (last_x > m_res_x - 2.5) last_x = m_res_x - 2.5;
			if (last_y > m_res_y - 1.5) last_y = m_res_y - 1.5;

			// Determine corners for bilinear interpolation
			double x_low = (int)last_x;
			double y_low = (int)last_y;
			double x_high = x_low + 1;
			double y_high = y_low + 1;

			// Compute the interpolation weights
			double x_weight = last_x - x_low;
			double y_weight = last_y - y_low;

			// TODO: Bilinear interpolation
			v_(x, y) = v(x, y);
		}
	}

	// Copy the values in temp to the original buffers
	for (int y = 1; y < m_res_y - 1; ++y)
		for (int x = 1; x < m_res_x - 1; ++x)
			d(x, y) = d_(x, y);
	for (int y = 1; y < m_res_y - 1; ++y)
		for (int x = 1; x < m_res_x; ++x)
			u(x, y) = u_(x, y);
	for (int y = 1; y < m_res_y; ++y)
		for (int x = 1; x < m_res_x - 1; ++x)
			v(x, y) = v_(x, y);
}

