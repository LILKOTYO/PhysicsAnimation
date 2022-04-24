#include <igl/writeOFF.h>
#include "CollisionSim.h"
#include "Gui.h"

class CollisionGui : public Gui {
   public:
    float m_angle = 1.047f;
    float m_force = 10.0f;
    float m_dt = 1e-1 * 3;
    float m_mass = 1.0;
    bool m_showContacts = false;
    bool m_useAABB = false;
    float m_eps = 0.6;

    int m_maxHistory = 200;
    std::vector<float> m_energy_history;

    int m_selectedBroadPhase = 0;
    const std::vector<char const *> m_broadphases = {"None", "AABB", "Own"};
    int m_selectedNarrowPhase = 0;
    const std::vector<char const *> m_narrowphases = {"Exhaustive", "Own"};

    CollisionSim *p_CollisionSim = NULL;

    CollisionGui() {
        p_CollisionSim = new CollisionSim();
        setSimulation(p_CollisionSim);

        // show vertex velocity instead of normal
        callback_clicked_vertex = [&](int clickedVertexIndex,
                                      int clickedObjectIndex,
                                      Eigen::Vector3d &pos,
                                      Eigen::Vector3d &dir) {
            RigidObject &o = p_CollisionSim->getObjects()[clickedObjectIndex];
            pos = o.getVertexPosition(clickedVertexIndex);
            dir = o.getVelocity(pos);
        };
        start();
    }

    virtual void updateSimulationParameters() override {
        p_CollisionSim->setForce(m_force);
        p_CollisionSim->setAngle(m_angle);
        p_CollisionSim->setTimestep(m_dt);
        p_CollisionSim->setMass(m_mass);
        p_CollisionSim->setEps(m_eps);
    }

    virtual void clearSimulation() override {
        p_CollisionSim->showContacts(false);
        p_CollisionSim->showContacts(m_showContacts);
    }

    virtual void drawSimulationParameterMenu() override {
        ImGui::SliderAngle("Angle", &m_angle, -180.0f, 180.0f);
        ImGui::InputFloat("Force", &m_force, 0, 0);
        ImGui::InputFloat("Mass", &m_mass, 0, 0);
        ImGui::InputFloat("dt", &m_dt, 0, 0);
        if (ImGui::Checkbox("Show contacts", &m_showContacts)) {
            p_CollisionSim->showContacts(m_showContacts);
        }
        if (ImGui::Combo("Broadphase", &m_selectedBroadPhase,
                         m_broadphases.data(), m_broadphases.size())) {
            p_CollisionSim->setBroadPhaseMethod(m_selectedBroadPhase);
        }
        if (ImGui::Combo("Narrowphase", &m_selectedNarrowPhase,
                         m_narrowphases.data(), m_narrowphases.size())) {
            p_CollisionSim->setNarrowPhaseMethod(m_selectedNarrowPhase);
        }
        ImGui::InputFloat("eps", &m_eps, 0, 0);
    }

    virtual void drawSimulationStats() override {
        Eigen::Vector3d E = p_CollisionSim->getKineticEnergy();
        m_energy_history.push_back(E.cast<float>().cwiseAbs().sum());
        if (m_energy_history.size() > m_maxHistory)
            m_energy_history.erase(m_energy_history.begin(),
                                   m_energy_history.begin() + 1);
        ImGui::Text("E: %.3f, %.3f, %.3f", E(0), E(1), E(2));
        ImGui::PlotLines("Total Energy", &m_energy_history[0],
                         m_energy_history.size(), 0, NULL, 0, 1000,
                         ImVec2(0, 200));
        Eigen::Vector3d p = p_CollisionSim->getLinearMomentum();
        ImGui::Text("M: %.3f, %.3f, %.3f", p(0), p(1), p(2));
        Eigen::Vector3d l = p_CollisionSim->getAngularMomentum();
        ImGui::Text("L: %.3f, %.3f, %.3f", l(0), l(1), l(2));
    }
};

int main(int argc, char *argv[]) {
    new CollisionGui();

    return 0;
}