#include "Simulation.h"
#include "igl/PI.h"

using namespace std;

struct DoublePendulum {
public:
    double m1, m2;
    double l1, l2;
};

/*
 */
class PendulumSim : public Simulation {
public:
    PendulumSim() : Simulation() {
        init();
        m_trajectories.clear();
        m_trajectoryColors.clear();
    }

    virtual void init() override {
        std::string path = "sphere.off";
        m_objects.clear();
        m_objects.push_back(RigidObject(path));
        m_objects.push_back(RigidObject(path));
        p1 = &m_objects[0];
        p2 = &m_objects[1];

        m_dt = 1e-2;
        m_log_frequency = 30;
        m_gravity << 0, -9.81, 0;
        // visualization color for sphere and trajectories
        m_color1 << 1.0, 0.5, 0;
        m_color2 << 0.0, 0.5, 1.0;
        reset();
    }

    virtual void resetMembers() override {
        p1->reset(); p2->reset();
        p1->setScale(0.04); p2->setScale(0.04);
        p1->setPosition(Eigen::Vector3d(m_pendulum.l1*cos(igl::PI/4), m_pendulum.l1*sin(igl::PI/4.0), 0));
        p2->setPosition(Eigen::Vector3d((m_pendulum.l1+m_pendulum.l2)*cos(igl::PI/4), (m_pendulum.l1+m_pendulum.l2)*sin(igl::PI/4.0), 0));
        p1->setColors(m_color1);
        p2->setColors(m_color2);

        clearTrajectories();
    }

    virtual void updateRenderGeometry() override {
        for (size_t i = 0; i < m_objects.size(); i++) {
            RigidObject &o = m_objects[i];
            if (o.getID() < 0) {    // negative id means newly created object, reverse memory for it
                m_renderVs.emplace_back();
                m_renderFs.emplace_back();
            }

            m_objects[i].getMesh(m_renderVs[i], m_renderFs[i]);
        }
    }

	virtual bool advance() override;

    virtual void renderRenderGeometry(igl::opengl::glfw::Viewer &viewer) override {
        // render point
        for (size_t i = 0; i < m_objects.size(); i++) {
            RigidObject &o = m_objects[i];
            if (o.getID() < 0) {
                int new_id = 0;
                if (i > 0) {
                    new_id = viewer.append_mesh();
                    o.setID(new_id);
                } else {
                    o.setID(new_id);
                }

                size_t meshIndex = viewer.mesh_index(o.getID());
                viewer.data_list[meshIndex].set_face_based(true);
                viewer.data_list[meshIndex].point_size = 2.0f;
                viewer.data_list[meshIndex].clear();
            }
            size_t meshIndex = viewer.mesh_index(o.getID());

            viewer.data_list[meshIndex].set_mesh(m_renderVs[i], m_renderFs[i]);
            viewer.data_list[meshIndex].compute_normals();

            Eigen::MatrixXd color;
            o.getColors(color);
            viewer.data_list[meshIndex].set_colors(color);
        }

        for (size_t trajectory = 0; trajectory < m_trajectories.size(); trajectory++) {
            if(m_trajectories[trajectory].size() <= 1) 
                continue;
            Eigen::MatrixXd p1s, p2s, cs;
            p1s.resize(m_trajectories[trajectory].size()-1, 3);
            p2s.resize(m_trajectories[trajectory].size()-1, 3);
            cs.resize(m_trajectories[trajectory].size()-1, 3);
            for (size_t point = 1; point < m_trajectories[trajectory].size(); point++) {
                p1s.row(point-1) << m_trajectories[trajectory][point-1].transpose();
                p2s.row(point-1) << m_trajectories[trajectory][point].transpose();
                cs.row(point-1) << m_trajectoryColors[trajectory];
                // viewer.data().add_edges(m_trajectories[trajectory][point-1].transpose(),
                //         m_trajectories[trajectory][point].transpose(),
                //         m_trajectoryColors[trajectory]);
            }
            viewer.data(0).add_edges(p1s, p2s, cs);
        }

		// draw the pendulum
        viewer.data(0).add_edges(Eigen::RowVector3d::Zero(), p1->getPosition().transpose(), m_color);
        viewer.data(0).add_edges(p1->getPosition().transpose(), p2->getPosition().transpose(), m_color);
    }

    void clearTrajectories() {
        m_trajectories.clear();
        m_trajectories.push_back(vector<Eigen::Vector3d>());    // for p1
        m_trajectories.push_back(vector<Eigen::Vector3d>());    // for p2
        m_trajectoryColors.clear();
        m_trajectoryColors.push_back(m_color1);                  // p1 color
        m_trajectoryColors.push_back(m_color2);                  // p1 color
    }

    void setPendulum(DoublePendulum &s) { m_pendulum = s; }
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

    Eigen::Vector3d m_gravity;

    DoublePendulum m_pendulum;
    RigidObject *p1, *p2;

    std::vector<Eigen::MatrixXd> m_renderVs;  // vertex positions for rendering
    std::vector<Eigen::MatrixXi> m_renderFs;  // face indices for rendering

    int m_log_frequency;  // how often should we log the COM in the GUI
    vector<vector<Eigen::Vector3d>> m_trajectories;
    Eigen::RowVector3d m_color, m_color1, m_color2;
    vector<Eigen::RowVector3d> m_trajectoryColors;
};