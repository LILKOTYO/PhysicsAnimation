#include <igl/writeOFF.h>
#include <thread>
#include "Gui.h"
#include "Simulator.h"
#include "PendulumSim.h"

/*
 */
class PendulumGui : public Gui {
   public:
    // simulation parameters
    float m_dt = 1e-2;
    int m_log_frequency = 30;

    DoublePendulum m_pendulum;  // stores properties of the pendulum
    PendulumSim *p_pendulumSim = NULL;

    // const vector<char const *> m_integrators = {"Analytic", "Explicit Euler", "Symplectic Euler", "Midpoint"};
    // int m_selected_integrator = 0;

    PendulumGui() {
        // initialize the pendulum to be used
        m_pendulum.l1 = 5.0;
        m_pendulum.l2 = 6.0;
        m_pendulum.m1 = 0.1;
        m_pendulum.m2 = 0.3;

        p_pendulumSim = new PendulumSim();
        p_pendulumSim->setPendulum(m_pendulum);
        setSimulation(p_pendulumSim);

        // show vertex velocity instead of normal
        callback_clicked_vertex =
            [&](int clickedVertexIndex, int clickedObjectIndex, Eigen::Vector3d &pos, Eigen::Vector3d &dir) {
                RigidObject &o = p_pendulumSim->getObjects()[clickedObjectIndex];
                pos = o.getVertexPosition(clickedVertexIndex);
                dir = o.getVelocity(pos);
            };
        start();
    }

    virtual void updateSimulationParameters() override {
        // change all parameters of the simulation to the values that are set in the GUI
        p_pendulumSim->setPendulum(m_pendulum);
        p_pendulumSim->setTimestep(m_dt);
        p_pendulumSim->setLogFrequency(m_log_frequency);
    }

    virtual void clearSimulation() override {
        p_pendulumSim->clearTrajectories();
    }

    /*
     * Writes each trajectory to an individual off-file.
     */
    void exportTrajectories() {
        Eigen::MatrixX3d mat;
        for (int i = 0; i < p_pendulumSim->getNumTrajectories(); i++) {
            string filename = "trajectory" + to_string(i) + ".off";
            p_pendulumSim->getTrajectories(i, mat);
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
        }
        return false;
    }

    virtual void drawSimulationParameterMenu() override {
        if (ImGui::Button("Export Trajectories", ImVec2(-1, 0))) {
            exportTrajectories();
        }
        ImGui::InputDouble("mass 1", &m_pendulum.m1, 0, 0);
        ImGui::InputDouble("mass 2", &m_pendulum.m2, 0, 0);
        ImGui::InputDouble("L1", &m_pendulum.l1, 0, 0);
        ImGui::InputDouble("L2", &m_pendulum.l2, 0, 0);
        ImGui::InputFloat("dt", &m_dt, 0, 0);
        // ImGui::Combo("Integrator", &m_selected_integrator, m_integrators.data(), m_integrators.size());
        ImGui::InputInt("Log Frequency", &m_log_frequency, 0, 0);
    }
};

int main(int argc, char *argv[]) {
    // create a new instance of the GUI for the pendulum simulation
    new PendulumGui();

    return 0;
}