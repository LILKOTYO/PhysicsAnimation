#include <igl/writeOFF.h>
#include <thread>
#include "Gui.h"
#include "Simulator.h"
#include "ClothSim.h"

/*
 */
class ClothGui : public Gui {
   public:
    // simulation parameters
    float m_dt = 1e-2;
    int n = 10, m = 10;
    double mass = 1;
    double domainX = 5;
    int m_log_frequency = 30;

    Spring m_spring;  // stores properties of the spring
    ClothSim *p_clothSim = NULL;
    const vector<char const *> m_integrators = {"Explicit Euler", "Implicit Euler", "Combination"};
    int m_selected_integrator = 0;


    ClothGui() {
        m_spring.damping = 10;
        m_spring.k_struct = 3000;
        m_spring.k_shear = 1000;
        m_spring.k_bend = 500;

        // initialize the pendulum to be used
        p_clothSim = new ClothSim();
        p_clothSim->setSpring(m_spring);
        setSimulation(p_clothSim);

        start();
    }

    virtual void updateSimulationParameters() override {
        // change all parameters of the simulation to the values that are set in the GUI
        double dx = domainX / (n-1);
        m_spring.L_struct = dx;
        m_spring.L_shear = dx * sqrt(2);
        m_spring.L_bend = 2 * dx;
        
        p_clothSim->setSpring(m_spring);
        p_clothSim->setTimestep(m_dt);
        p_clothSim->setLogFrequency(m_log_frequency);
        p_clothSim->setMethod(m_selected_integrator);
        p_clothSim->m_mass = mass;
        p_clothSim->n = n;
        p_clothSim->m = m;
        p_clothSim->domainX = domainX;
        p_clothSim->dx = dx;
    }

    virtual void clearSimulation() override {
        // 
    }


    virtual void drawSimulationParameterMenu() override {
        ImGui::InputInt("n", &n, 0, 0);
        ImGui::InputInt("m", &m, 0, 0);
        ImGui::InputDouble("mass", &mass, 0, 0);
        ImGui::InputDouble("Domain X", &domainX, 0, 0);
        ImGui::InputDouble("struct stiffness", &m_spring.k_struct, 0, 0);
        ImGui::InputDouble("shear stiffness", &m_spring.k_shear, 0, 0);
        ImGui::InputDouble("bend stiffness", &m_spring.k_bend, 0, 0);
        ImGui::InputDouble("damping", &m_spring.damping, 0, 0);
        ImGui::InputFloat("dt", &m_dt, 0, 0);
        ImGui::Combo("Integrator", &m_selected_integrator, m_integrators.data(), m_integrators.size());
        ImGui::InputInt("Log Frequency", &m_log_frequency, 0, 0);
    }
};

int main(int argc, char *argv[]) {
    // create a new instance of the GUI for the pendulum simulation
    new ClothGui();

    return 0;
}