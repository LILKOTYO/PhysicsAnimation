#include "Simulation.h"
#include "igl/PI.h"

using namespace std;


class BeadSim : public Simulation {
public:
    BeadSim() : Simulation() {
        init();
        m_trajectories.clear();
        m_trajectoryColors.clear();
    }

	virtual void init() override {
		std::string path = "sphere.off";
		m_objects.clear();
		m_objects.push_back(RigidObject(path));
		p_bead = &m_objects[0];

		m_dt = 5e-2;
        m_mass = 1.0;
        m_log_frequency = 30;
		m_radius = 10;
        m_gravity << 0, -9.81, 0;

		reset();
	}

    virtual void resetMembers() override {
        p_bead->reset();
        p_bead->setScale(0.1);
        // initial position, should be valid for constraints
        p_bead->setPosition(Eigen::Vector3d(m_radius*cos(igl::PI/3), m_radius*sin(igl::PI/3.0), 0));

        if (m_trajectories.size() == 0 || m_trajectories.back().size() > 1) {
            m_trajectories.push_back(vector<Eigen::Vector3d>());
            m_trajectoryColors.push_back(m_color);
        } else {
            m_trajectoryColors.back() = m_color;
        }
    }

    virtual void updateRenderGeometry() override {
        p_bead->getMesh(m_renderV, m_renderF);
    }

	virtual bool advance() override;

    virtual void renderRenderGeometry(igl::opengl::glfw::Viewer &viewer) override {
        viewer.data().set_mesh(m_renderV, m_renderF);
       
        for (size_t trajectory = 0; trajectory < m_trajectories.size(); trajectory++) {
            for (size_t point = 1; point < m_trajectories[trajectory].size(); point++) {
                viewer.data().add_edges(m_trajectories[trajectory][point-1].transpose(),
                        m_trajectories[trajectory][point].transpose(),
                        m_trajectoryColors[trajectory]);
                // viewer.data().add_points(
                //     m_trajectories[trajectory][point].transpose(),
                //     m_trajectoryColors[trajectory]);
            }
        }
    }

    void clearTrajectories() {
        m_trajectories.clear();
        m_trajectories.push_back(vector<Eigen::Vector3d>());
        m_trajectoryColors.clear();
        m_trajectoryColors.push_back(m_color);
    }
    void setRadius(double r) { m_radius = r; }
    void setLogFrequency(int f) { m_log_frequency = f; }
    void getTrajectories(int index, Eigen::MatrixX3d &mat) const {
        int num_points = m_trajectories[index].size();
        mat.resize(num_points, 3);
        for (int i = 0; i < num_points; i++) {
            mat.row(i) = m_trajectories[index][i];
        }
    }
    int getNumTrajectories() const { return m_trajectories.size(); }
private:
    RigidObject *p_bead;
    double m_radius;
    double m_mass;

    Eigen::Vector3d m_gravity;

    Eigen::MatrixXd m_renderV;  // vertex positions for rendering
    Eigen::MatrixXi m_renderF;  // face indices for rendering

    int m_log_frequency;  // how often should we log the COM in the GUI
    vector<vector<Eigen::Vector3d>> m_trajectories;
    Eigen::RowVector3d m_color;
    vector<Eigen::RowVector3d> m_trajectoryColors;
};