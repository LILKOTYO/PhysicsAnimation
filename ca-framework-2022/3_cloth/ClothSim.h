#include "Simulation.h"
#include "igl/list_to_matrix.h"

using namespace std;

struct Spring {
public:
    double damping;
    double k_struct, k_bend, k_shear;
    double L_struct, L_bend, L_shear;
};

struct Particle {
public:
    Particle(const Eigen::Vector3d& p = Eigen::Vector3d::Zero(), const Eigen::Vector3d& v=Eigen::Vector3d::Zero())
        : position(p), velocity(v) {}
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

/*
 */
class ClothSim : public Simulation {
public:
    ClothSim() : Simulation(), n(10), m(10), domainX(5) {
        init();
        // m_trajectories.clear();
        // m_trajectoryColors.clear();
    }

    virtual void init() override {
        std::string path = "sphere.off";
        m_objects.clear();
        m_objects.push_back(RigidObject(path));
        m_objects.push_back(RigidObject(path));
        p1 = &m_objects[0];
        p2 = &m_objects[1];

        m_dt = 1e-2;
        m_gravity << 0, -9.81, 0;
        n = 10, m = 10;
        domainX = 5;
        dx = domainX / (n-1);

        cgSolver.setTolerance(1e-7);
	    cgSolver.setMaxIterations(1000);
        reset();
    }

    virtual void resetMembers() override {
        double domainY = dx * (m-1);
        // initialize fixed point visualizations
        p1->reset(); p2->reset();
        p1->setScale(0.01); p2->setScale(0.01);
        p1->setPosition(Eigen::Vector3d(-domainX/2.0, domainY, 0));
        p2->setPosition(Eigen::Vector3d(domainX/2.0, domainY, 0));

        // initialize particles
        particles.resize(n * m);
        for(int i = 0; i < n; ++i) {
            for(int j = 0; j < m; ++j) {
                particles[particleCoordinateToIdx(i, j)] = 
                    Particle(Eigen::Vector3d(dx * i - domainX/2, domainY, dx * j), Eigen::Vector3d::Zero());
            }
        }

        // initialize fixed particle
        fixedParticleIdx.resize(2);
        fixedParticleIdx[0] = particleCoordinateToIdx(0, 0);
        fixedParticleIdx[1] = particleCoordinateToIdx(n-1, 0);
        isFixedParticle.resize(n*m, false);
        isFixedParticle[particleCoordinateToIdx(0, 0)] = true;
        isFixedParticle[particleCoordinateToIdx(n-1, 0)] = true;

        // initialize cloth mesh triangles
        std::vector<std::vector<int>> faces;
        faces.reserve((n-1)*(m-1)*2);
        for(int i = 0; i < n-1; ++i) {
            for(int j = 0; j < m-1; ++j) {
                faces.push_back({particleCoordinateToIdx(i, j), particleCoordinateToIdx(i, j+1), particleCoordinateToIdx(i+1, j+1)});
                faces.push_back({particleCoordinateToIdx(i, j), particleCoordinateToIdx(i+1, j+1), particleCoordinateToIdx(i+1, j)});
            }
        }
        igl::list_to_matrix(faces, m_cloth_renderF);
    }

    virtual void updateRenderGeometry() override {
        // create cloth geometry
        for (size_t i = 0; i < m_objects.size(); i++) {
            RigidObject &o = m_objects[i];
            if (o.getID() < 0) {    // negative id means newly created object, reverse memory for it
                m_renderVs.emplace_back();
                m_renderFs.emplace_back();
            }

            m_objects[i].getMesh(m_renderVs[i], m_renderFs[i]);
        }

        m_cloth_renderV.resize(n*m, 3);
        for(int i = 0; i < n*m; ++i) {
            m_cloth_renderV.row(i) << particles[i].position.transpose();
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

        if(cloth_render_id < 0) {
            cloth_render_id = viewer.append_mesh();
            viewer.data_list[cloth_render_id].set_face_based(true);
            viewer.data_list[cloth_render_id].point_size = 2.0f;
            viewer.data_list[cloth_render_id].clear();
        }
        viewer.data_list[cloth_render_id].set_mesh(m_cloth_renderV, m_cloth_renderF);
        viewer.data_list[cloth_render_id].compute_normals();
    }

    void setMethod(int m) { m_method = m; }
    void setSpring(Spring &s) { m_spring = s; }
    void setLogFrequency(int f) { m_log_frequency = f; }
    int n, m;
    double m_mass;
    double domainX, dx;

private:

    int particleCoordinateToIdx(int i, int j) { return i*m + j; }
    int particleCoordinateToIdx(const Eigen::Vector2i& coor) { return particleCoordinateToIdx(coor.x(), coor.y()); }
    Eigen::Vector2i particleIdxToCoordinate(int idx) { return Eigen::Vector2i(int(idx/m), idx%m); }
    bool isValidIdx(int idx) { return idx >= 0 && idx < particles.size(); }
    bool isValidCoor(const Eigen::Vector2i& coor) {
        return coor.x() >= 0 && coor.x() < n && coor.y() >= 0 && coor.y() < m;
    }

    int m_method;   // (0: euler, 1: implicit euler)
    Eigen::Vector3d m_gravity;
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower|Eigen::Upper, Eigen::DiagonalPreconditioner<double>> cgSolver;

    Spring m_spring;
    RigidObject *p1, *p2;

    std::vector<int> fixedParticleIdx;
    std::vector<bool> isFixedParticle;
    std::vector<Particle> particles;
    std::vector<Eigen::MatrixXd> m_renderVs;  // vertex positions for rendering
    std::vector<Eigen::MatrixXi> m_renderFs;  // face indices for rendering
    int cloth_render_id = -1;
    Eigen::MatrixXd m_cloth_renderV;  // vertex positions for rendering
    Eigen::MatrixXi m_cloth_renderF;  // face indices for rendering

    int m_log_frequency;  // how often should we log the COM in the GUI
    // vector<vector<Eigen::Vector3d>> m_trajectories;
    // Eigen::RowVector3d m_color, m_color1, m_color2;
    // vector<Eigen::RowVector3d> m_trajectoryColors;
};