#include <deque>
#include "Gui.h"
#include "Simulator.h"
#include "SpinSim.h"

class SpinGui : public Gui {
   public:
    // simulation parameters
    Eigen::Vector3f m_force;
    float m_mass = 1.0;
    float m_dt = 1e-3;
    int m_maxHistory = 200;
    std::vector<float> m_energy_history;

    const vector<char const *> m_methods = {"Matrix", "Quaternion"};
    int m_selected_method = 0;

    SpinSim *p_spinSim = NULL;

    SpinGui() {
        m_force << 10000, 0, 0;
        m_energy_history.clear();

        p_spinSim = new SpinSim();
        p_spinSim->setForce(m_force.cast<double>());
        p_spinSim->setMass(m_mass);
        setSimulation(p_spinSim);

        // show vertex velocity instead of normal
        callback_clicked_vertex =
            [&](int clickedVertexIndex, int clickedObjectIndex,
                Eigen::Vector3d &pos, Eigen::Vector3d &dir) {
                RigidObject &o = p_spinSim->getObjects()[clickedObjectIndex];
                pos = o.getVertexPosition(clickedVertexIndex);
                dir = o.getVelocity(pos);
            };
        start();
    }

    virtual void updateSimulationParameters() override {
        // change all parameters of the simulation to the values that are set in
        // the GUI
        p_spinSim->setForce(m_force.cast<double>());
        p_spinSim->setMass(m_mass);
        p_spinSim->setTimestep(m_dt);
        p_spinSim->setMethod(m_selected_method);
    }

    virtual void drawSimulationParameterMenu() override {
        ImGui::Combo("Method", &m_selected_method, m_methods.data(),
                     m_methods.size());
        ImGui::InputFloat3("Force", m_force.data(), 0);
        ImGui::InputFloat("Mass", &m_mass, 0, 0, 6);
        ImGui::InputFloat("dt", &m_dt, 0, 0);
    }

    virtual void drawSimulationStats() override {
        Eigen::Vector3d E = p_spinSim->getRotationalEnergy();
        m_energy_history.push_back(E.cast<float>().cwiseAbs().sum());
        if (m_energy_history.size() > m_maxHistory)
            m_energy_history.erase(m_energy_history.begin(),
                                   m_energy_history.begin() + 1);
        ImGui::Text("E_x: %.3f", E(0));
        ImGui::Text("E_y: %.3f", E(1));
        ImGui::Text("E_z: %.3f", E(2));
        ImGui::PlotLines("Total Energy", &m_energy_history[0],
                         m_energy_history.size(), 0, NULL, 0, 1000,
                         ImVec2(0, 200));
    }
};

int main(int argc, char *argv[]) {
    // create a new instance of the GUI for the spinning simulation
    new SpinGui();

    return 0;
}