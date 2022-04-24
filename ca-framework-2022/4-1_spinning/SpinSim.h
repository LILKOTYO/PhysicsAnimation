#include "Simulation.h"

using namespace std;

/*
 */
class SpinSim : public Simulation {
   public:
    SpinSim() : Simulation() { init(); }

    virtual void init() override {
        std::string file = "cube.off";

        m_objects.clear();
        m_objects.push_back(RigidObject(file));
        p_body = &m_objects.back();

        m_dt = 1e-3;

        reset();
    }

    virtual void resetMembers() override {
        p_body->reset();
        p_body->setMass(m_mass);

        p_body->applyForce(m_force, Eigen::Vector3d(1, 1, 1));
        p_body->applyForce(-m_force, Eigen::Vector3d(-1, -1, -1));

        // compute linear and angular momentums
        p_body->setLinearMomentum(p_body->getLinearMomentum() +
                                  m_dt * p_body->getForce());
        p_body->setAngularMomentum(p_body->getAngularMomentum() +
                                   m_dt * p_body->getTorque());
        p_body->resetForce();
        p_body->resetTorque();
    }

    virtual void updateRenderGeometry() override {
        p_body->getMesh(m_renderV, m_renderF);
    }

	virtual bool advance() override;

    virtual void renderRenderGeometry(
        igl::opengl::glfw::Viewer &viewer) override {
        viewer.data().set_mesh(m_renderV, m_renderF);
        viewer.data().compute_normals();
    }

#pragma region SettersAndGetters
    void setForce(const Eigen::Vector3d &f) { m_force = f; }

    void setMass(double m) { m_mass = m; }

    void setMethod(int m) { m_method = m; }

    Eigen::Vector3d getRotationalEnergy() {
        return 0.5 * p_body->getInertia().diagonal().cwiseProduct(
                         p_body->getAngularVelocity());
    }
#pragma endregion SettersAndGetters

   private:
    double m_mass;
    Eigen::Vector3d m_force;
    int m_method = 0;

    RigidObject *p_body;

    Eigen::MatrixXd m_renderV;  // vertex positions for rendering
    Eigen::MatrixXi m_renderF;  // face indices for rendering
};