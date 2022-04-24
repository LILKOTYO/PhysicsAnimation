#include "BeadSim.h"
#include "Gui.h"

/*
 * GUI for the bead simulation.
 */
class BeadGui : public Gui {
   public:
    // simulation parameters
    float m_dt = 1e-2;
    float m_radius = 10.0;
    int m_log_frequency = 30;

    BeadSim *p_BeadSim = NULL;

    BeadGui() {
        p_BeadSim = new BeadSim();
        setSimulation(p_BeadSim);

        start();
    }

    virtual void updateSimulationParameters() override {
        // change all parameters of the simulation to the values that are set in the GUI
        p_BeadSim->setTimestep(m_dt);
        p_BeadSim->setRadius(m_radius);
        p_BeadSim->setLogFrequency(m_log_frequency);
    }


    virtual void drawSimulationParameterMenu() override {
        ImGui::InputFloat("Radius", &m_radius, 0, 0);
        ImGui::InputFloat("dt", &m_dt, 0, 0);
    }

    virtual void clearSimulation() override {
        p_BeadSim->clearTrajectories();
    }
};

int main(int argc, char *argv[]) {
    new BeadGui();

    return 0;
}