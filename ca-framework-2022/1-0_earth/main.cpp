#include "EarthSim.h"
#include "Gui.h"

/*
 * GUI for the earth simulation.
 */
class EarthGui : public Gui {
   public:
    // simulation parameters
    float m_dt = 1e-2;
    float m_radius = 10.0;

    EarthSim *p_EarthSim = NULL;

    EarthGui() {
        // create a new Earth simulation, set it in the GUI, and start the GUI
        p_EarthSim = new EarthSim();
        setSimulation(p_EarthSim);

        start();
    }

    virtual void updateSimulationParameters() override {
        // change all parameters of the simulation to the values that are set in the GUI
        p_EarthSim->setTimestep(m_dt);
        p_EarthSim->setRadius(m_radius);
    }


    virtual void drawSimulationParameterMenu() override {
        ImGui::InputFloat("Radius", &m_radius, 0, 0);
        ImGui::InputFloat("dt", &m_dt, 0, 0);
    }
};

int main(int argc, char *argv[]) {
    // create a new instance of the GUI for the Earth simulation
    new EarthGui();

    return 0;
}