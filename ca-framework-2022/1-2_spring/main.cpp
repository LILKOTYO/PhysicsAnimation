#include <igl/writeOFF.h>
#include <thread>
#include "Gui.h"
#include "Simulator.h"
#include "SpringSim.h"

/*
 * GUI for the spring simulation. This time we need additional paramters,
 * e.g. which integrator to use for the simulation and the force applied to the
 * cube, and we also add some more visualizations (trajectories).
 */
class SpringGui : public Gui {
   public:
    // simulation parameters
    float m_dt = 1e-2;
    float m_mass = 1.0;
    int m_log_frequency = 30;

    Spring m_spring;  // stores properties of a spring

    SpringSim *p_springSim = NULL;

    const vector<char const *> m_integrators = {"Analytic", "Explicit Euler", "Symplectic Euler", "Midpoint"};
    int m_selected_integrator = 0;

    SpringGui() {
        // initialize the spring to be used
        m_spring.length = 5.0;
        m_spring.stiffness = 5.0;
        m_spring.damping = 0.1;
        m_spring.start = m_spring.end = Eigen::Vector3d::Zero();

        p_springSim = new SpringSim();
        p_springSim->setSpring(m_spring);
        setSimulation(p_springSim);

        // show vertex velocity instead of normal
        callback_clicked_vertex =
            [&](int clickedVertexIndex, int clickedObjectIndex,
                Eigen::Vector3d &pos, Eigen::Vector3d &dir) {
                RigidObject &o = p_springSim->getObjects()[clickedObjectIndex];
                pos = o.getVertexPosition(clickedVertexIndex);
                dir = o.getVelocity(pos);
            };
        start();
    }

    virtual void updateSimulationParameters() override {
        // change all parameters of the simulation to the values that are set in
        // the GUI
        p_springSim->setTimestep(m_dt);
        p_springSim->setMethod(m_selected_integrator);
        p_springSim->setLogFrequency(m_log_frequency);
        p_springSim->setMass(m_mass);
        p_springSim->setSpring(m_spring);
    }

    virtual void clearSimulation() override {
        p_springSim->clearTrajectories();
    }

    /*
     * Writes each trajectory to an individual off-file.
     */
    void exportTrajectories() {
        Eigen::MatrixX3d mat;
        for (int i = 0; i < p_springSim->getNumTrajectories(); i++) {
            string filename = "trajectory" + to_string(i) + ".off";
            p_springSim->getTrajectories(i, mat);
            if (mat.rows() <= 1) {
                continue;
            }
            if (igl::writeOFF(filename, mat, Eigen::MatrixXi())) {
                cout << "Wrote trajectory to " << filename << endl;
            } else {
                cout << "Failed to write trajectory to " << filename << endl;
            }
        }
    }

    virtual bool childKeyCallback(igl::opengl::glfw::Viewer &viewer,
                                  unsigned int key, int modifiers) override {
        switch (key) {
            case 'e':
            case 'E':
                exportTrajectories();
                return true;
            // cicle through different integrators
            case '>':
                m_selected_integrator++;
                m_selected_integrator %= m_integrators.size();
                return true;
            case '<':
                m_selected_integrator--;
                m_selected_integrator =
                    (m_integrators.size() + m_selected_integrator) %
                    m_integrators.size();
                return true;
        }
        return false;
    }

    virtual void drawSimulationParameterMenu() override {
        if (ImGui::Button("Export Trajectories", ImVec2(-1, 0))) {
            exportTrajectories();
        }
        ImGui::InputFloat("Spring Stiffness", &m_spring.stiffness, 0, 0);
        ImGui::InputFloat("Spring Length", &m_spring.length, 0, 0);
        ImGui::InputFloat("Mass", &m_mass, 0, 0);
        ImGui::InputFloat("Damping", &m_spring.damping, 0, 0);
        ImGui::InputFloat("dt", &m_dt, 0, 0);
        ImGui::Combo("Integrator", &m_selected_integrator, m_integrators.data(),
                     m_integrators.size());
        ImGui::InputInt("Log Frequency", &m_log_frequency, 0, 0);
    }
};

int main(int argc, char *argv[]) {
    // create a new instance of the GUI for the spring simulation
    new SpringGui();

    return 0;
}