#include "CollisionDetection.h"
#include "Simulation.h"

#include <deque>

using namespace std;

/*
 */
class CollisionSim : public Simulation {
   public:
    CollisionSim() : Simulation(), m_collisionDetection(m_objects) { init(); }

    const int NUM_CUBES = 5;  // complexity of scene

    virtual void init() override {
        std::string path = "cube.off";
        for (int i = 0; i < NUM_CUBES; i++) {
            m_objects.push_back(RigidObject(path));
        }
		path = "cube_shallow.off";
		for (int i = 0; i < 5; ++i) {
			m_objects.push_back(RigidObject(path));
		}
		
        m_collisionDetection.setObjects(m_objects);

        m_dt = 1e-3 * 3;
        m_gravity << 0, -9.81, 0;
        m_mass = 1.0;
        m_showContacts = false;
        m_broadPhaseMethod = 0;
        m_narrowPhaseMethod = 0;
        m_eps = 1.0;

        reset();
    }

    virtual void resetMembers() override {
        for (auto &o : m_objects) {
            o.reset();
        }

		double x1 = 5;
		double y1 = 4;
		m_objects[0].setPosition(Eigen::Vector3d(0, y1, 0));
		m_objects[0].setRotation(
			Eigen::Quaterniond(0, -0.3444844, -0.3444844, -0.8733046)); 
		
		Eigen::RowVector3d c0(204.0 / 255.0, 0, 0);
		Eigen::RowVector3d c1(0, 128.0 / 255.0, 204.0 / 255.0);
		for (int i = 1; i < NUM_CUBES; ++i) {
			m_objects[i].setPosition(Eigen::Vector3d(x1, y1*(i+1), 0));
			double a = double(i+1) / NUM_CUBES;
			m_objects[i].setColors(c0*a + c1*(1 - a));
		}

        for (size_t i = 0; i < NUM_CUBES; i++) {
            m_objects[i].setMass(m_mass);
        }

		for (int i = 0; i < 5; ++i) {
			m_objects[i + NUM_CUBES].setScale(10);
			m_objects[i + NUM_CUBES].setType(ObjType::STATIC);
			m_objects[i + NUM_CUBES].setColors(Eigen::RowVector3d(0.5, 0.5, 0.5));
			m_objects[i + NUM_CUBES].setMass(std::numeric_limits<double>::max());
		}

		m_objects[0 + NUM_CUBES].setPosition(
			Eigen::Vector3d(0, -0.1, 0));

		m_objects[1 + NUM_CUBES].setPosition(
			Eigen::Vector3d(-10.1, 10, 0));
		m_objects[1 + NUM_CUBES].setRotation(
			Eigen::Quaterniond(0.7071, 0, 0, 0.7071));

		m_objects[2 + NUM_CUBES].setPosition(
			Eigen::Vector3d(10.1, 10, 0));
		m_objects[2 + NUM_CUBES].setRotation(
			Eigen::Quaterniond(0.7071, 0, 0, 0.7071));

		m_objects[3 + NUM_CUBES].setPosition(
			Eigen::Vector3d(0, 10, -10.1));
		m_objects[3 + NUM_CUBES].setRotation(
			Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5));

		m_objects[4 + NUM_CUBES].setPosition(
			Eigen::Vector3d(0, 10, 10.1));
		m_objects[4 + NUM_CUBES].setRotation(
			Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5));


        updateVars();
    }

    virtual void updateRenderGeometry() override {
        for (size_t i = 0; i < m_objects.size(); i++) {
            RigidObject &o = m_objects[i];
            if (o.getID() < 0) {
                m_renderVs.emplace_back();
                m_renderFs.emplace_back();
            }

            m_objects[i].getMesh(m_renderVs[i], m_renderFs[i]);
        }
    }

    virtual bool advance() override {
        // compute the collision detection
        m_collisionDetection.computeCollisionDetection(m_broadPhaseMethod, m_narrowPhaseMethod, m_eps);

        // apply forces (only gravity in this case)
        for (auto &o : m_objects) {
            o.applyForceToCOM(m_gravity);
        }

        for (auto &o : m_objects) {
            // integrate velocities
            o.setLinearMomentum(o.getLinearMomentum() + m_dt * o.getForce());
            o.setAngularMomentum(o.getAngularMomentum() + m_dt * o.getTorque());
            o.resetForce();
            o.resetTorque();

            // angular velocity (matrix)
            Eigen::Vector3d w = o.getAngularVelocity();
            Eigen::Quaterniond wq;
            wq.w() = 0;
            wq.vec() = w;

            // integrate position and rotation
            o.setPosition(o.getPosition() + m_dt * o.getLinearVelocity());
            Eigen::Quaterniond q = o.getRotation();
            Eigen::Quaterniond dq = wq * q;
            Eigen::Quaterniond new_q;
            new_q.w() = q.w() + 0.5 * m_dt * dq.w();
            new_q.vec() = q.vec() + 0.5 * m_dt * dq.vec();
            o.setRotation(new_q.normalized());
        }
        // advance time
        m_time += m_dt;
        m_step++;

        return false;
    }

    virtual void renderRenderGeometry(
        igl::opengl::glfw::Viewer &viewer) override {
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
				if (i >= NUM_CUBES) {
					viewer.data_list[meshIndex].show_lines = true;
					viewer.data_list[meshIndex].show_faces = false;
				}
				else {
					viewer.data_list[meshIndex].show_lines = false;
				}
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

        if (m_showContacts) {
            // number of timesteps to keep showing collision
            int delay = 10;

            // clear old points
            viewer.data_list[1].points = Eigen::MatrixXd(0, 6);
            viewer.data_list[1].point_size = 10.0f;

            // remove expired points
            while (m_contactMemory.size() > 0 &&
                   m_contactMemory.front().second + delay < m_step) {
                m_contactMemory.pop_front();
            }

            // get new points and add them to memory
            auto contacts = m_collisionDetection.getContacts();
            for (auto &contact : contacts) {
                m_contactMemory.push_back(std::make_pair(contact, m_step));
            }

            // show points
            for (auto &contact_int_p : m_contactMemory) {
                viewer.data_list[1].add_points(
                    contact_int_p.first.p.transpose(),
                    (contact_int_p.first.type == ContactType::EDGEEDGE)
                        ? Eigen::RowVector3d(0, 1, 0)
                        : Eigen::RowVector3d(0, 0, 1));
            }
        }
    }

#pragma region SettersAndGetters
    /*
     * Compute magnitude and direction of momentum and apply it to o
     */
    void updateVars() {
        Eigen::Vector3d momentum;
        momentum << std::sin(m_angle), std::cos(m_angle), 0;
        momentum *= m_force;
        m_objects[0].setLinearMomentum(momentum);
    }

    void setAngle(double a) {
        m_angle = a;
        updateVars();
    }

    void setForce(double f) {
        m_force = f;
        updateVars();
    }

    void setMass(double m) { m_mass = m; }

    void showContacts(bool s) {
        if (!s) {
            m_contactMemory.clear();
        }
        m_showContacts = s;
    }

    void setBroadPhaseMethod(int m) { m_broadPhaseMethod = m; }
    void setNarrowPhaseMethod(int m) { m_narrowPhaseMethod = m; }

    void setEps(double eps) { m_eps = eps; }

    Eigen::Vector3d getKineticEnergy() {
        Eigen::Vector3d res;
        res.setZero();
        for (auto o : m_objects) {
            if (o.getType() == ObjType::STATIC) continue;
            Eigen::Vector3d rotE = 0.5 * o.getInertia().diagonal().cwiseProduct(
                                             o.getAngularVelocity());
            Eigen::Vector3d kinE =
                0.5 * o.getMass() * o.getLinearVelocity().array().square();
            res += rotE + kinE;
        }
        return res;
    }

    Eigen::Vector3d getLinearMomentum() {
        Eigen::Vector3d res;
        res.setZero();
        for (auto o : m_objects) {
            if (o.getType() == ObjType::STATIC) continue;
            res += o.getLinearMomentum();
        }
        return res;
    }

    Eigen::Vector3d getAngularMomentum() {
        Eigen::Vector3d res;
        res.setZero();
        for (auto o : m_objects) {
            if (o.getType() == ObjType::STATIC) continue;
            res += o.getAngularMomentum();
        }
        return res;
    }

#pragma endregion SettersAndGetters

   private:
    [[maybe_unused]] int m_method;  // id of integrator to be used (0: analytical, 1: explicit
                   // euler, 2: semi-implicit euler)
    double m_angle;
    double m_force;
    double m_mass;

    Eigen::Vector3d m_gravity;

    CollisionDetection m_collisionDetection;
    int m_broadPhaseMethod;
    int m_narrowPhaseMethod;
    double m_eps;

    std::vector<Eigen::MatrixXd> m_renderVs;  // vertex positions for rendering
    std::vector<Eigen::MatrixXi> m_renderFs;  // face indices for rendering

    bool m_showContacts;
    std::deque<std::pair<Contact, int>> m_contactMemory;
};