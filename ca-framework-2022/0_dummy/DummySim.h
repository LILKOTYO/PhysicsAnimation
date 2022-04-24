#include "Simulation.h"

/*
 * Example simulation that changes the colors of a cube.
 */
class DummySim : public Simulation {
public:
	DummySim();

	virtual void init() override;
	virtual void resetMembers() override;
	virtual void updateRenderGeometry() override;
	virtual bool advance() override;
	virtual void renderRenderGeometry(igl::opengl::glfw::Viewer &viewer) override;

private:
	Eigen::MatrixXd m_V;  // vertex positions
	Eigen::MatrixXi m_F;  // face indices
	Eigen::MatrixXd m_C;  // colors per face

	Eigen::MatrixXd m_renderV;  // vertex positions for rendering
	Eigen::MatrixXi m_renderF;  // face indices for rendering
	Eigen::MatrixXd m_renderC;  // colors per face for rendering
};