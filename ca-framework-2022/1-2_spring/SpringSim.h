#include "Simulation.h"

using namespace std;

struct Spring {
   public:
    float stiffness;
    float length;
    float damping;
    Eigen::Vector3d start;
    Eigen::Vector3d end;
};

/*
 * Simulation of a string with an attached mass.
 */
class SpringSim : public Simulation {
public:
    SpringSim() : Simulation() {
        init();
        m_trajectories.clear();
        m_trajectoryColors.clear();
    }

    virtual void init() override {
        std::string path = "cube.off";
        m_objects.clear();
        m_objects.push_back(RigidObject(path));
        p_cube = &m_objects.back();

        m_dt = 1e-2;
        m_mass = 1.0;
        m_log_frequency = 30;
        m_method = 0;
        m_gravity << 0, -9.81, 0;

        reset();
    }

    virtual void resetMembers() override {
        p_cube->reset();
        m_spring.end = m_spring.start - m_spring.length * Eigen::Vector3d(0, 1, 0);
        p_cube->setPosition(m_spring.end);
        p_cube->setMass(m_mass);

        if (m_trajectories.size() == 0 || m_trajectories.back().size() > 1) {
            m_trajectories.push_back(vector<Eigen::Vector3d>());
            m_trajectoryColors.push_back(m_color);
        } else {
            m_trajectoryColors.back() = m_color;
        }

        setMethod(m_method);
    }

    virtual void updateRenderGeometry() override {
        p_cube->getMesh(m_renderV, m_renderF);
    }

	virtual bool advance() override;

    virtual void renderRenderGeometry(
        igl::opengl::glfw::Viewer &viewer) override {
        viewer.data().set_mesh(m_renderV, m_renderF);

        for (size_t trajectory = 0; trajectory < m_trajectories.size();
             trajectory++) {
            for (size_t point = 0; point < m_trajectories[trajectory].size();
                 point++) {
                viewer.data().add_points(
                    m_trajectories[trajectory][point].transpose(),
                    m_trajectoryColors[trajectory]);
            }
        }

		// draw the spring
		for (int i = 0; i < 20; ++i) {
			double w = i / 20.0;
			Eigen::RowVector3d p = m_spring.start * w + m_spring.end * (1 - w);
			viewer.data().add_points(p, Eigen::RowVector3d(0, 0, 0));
		}
    }

    void clearTrajectories() {
        m_trajectories.clear();
        m_trajectories.push_back(vector<Eigen::Vector3d>());
        m_trajectoryColors.clear();
        m_trajectoryColors.push_back(m_color);
    }

#pragma region SettersAndGetters
    void setMass(double m) { m_mass = m; }

    void setSpring(const Spring &s) {
        m_spring = s;
        m_spring.end =
            m_spring.start - m_spring.length * Eigen::Vector3d(0, 1, 0);
        p_cube->setPosition(m_spring.end);
    }

    void setMethod(int m) {
        m_method = m;
        switch (m_method) {
            case 0:
                m_color = Eigen::RowVector3d(1.0, 0.0, 0.0);
                break;
            case 1:
                m_color = Eigen::RowVector3d(0.0, 1.0, 0.0);
                break;
            case 2:
                m_color = Eigen::RowVector3d(0.0, 0.0, 1.0);
                break;
            case 3:
                m_color = Eigen::RowVector3d(1.0, 1.0, 0.0);
                break;
            default:
                std::cerr << m_method << " is not a valid integrator method."
                          << std::endl;
        }
        if (m_step == 0) {
            m_trajectoryColors.back() = m_color;
        }
    }

    void setLogFrequency(int f) { m_log_frequency = f; }

    void getTrajectories(int index, Eigen::MatrixX3d &mat) const {
        int num_points = m_trajectories[index].size();
        mat.resize(num_points, 3);
        for (int i = 0; i < num_points; i++) {
            mat.row(i) = m_trajectories[index][i];
        }
    }

    int getNumTrajectories() const { return m_trajectories.size(); }
#pragma endregion SettersAndGetters

private:
    int m_method;  // id of integrator to be used (0: analytical, 1: explicit euler, 2: semi-implicit euler, 3: midpoint)
    double m_mass;

    Eigen::Vector3d m_gravity;

    Spring m_spring;
    RigidObject *p_cube;

    Eigen::MatrixXd m_renderV;  // vertex positions for rendering
    Eigen::MatrixXi m_renderF;  // face indices for rendering

    int m_log_frequency;  // how often should we log the COM in the GUI
    vector<vector<Eigen::Vector3d>> m_trajectories;
    Eigen::RowVector3d m_color;
    vector<Eigen::RowVector3d> m_trajectoryColors;
};