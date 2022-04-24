#include "ClothSim.h"
#include "igl/Timer.h"

using Eigen::Vector2i;
typedef Eigen::Triplet<double> Triplet;
typedef Eigen::SparseMatrix<double> SparseMatrix;
typedef Eigen::Matrix<double, 3, 3> Matrix33;

// add matrix into triplet array with index [i*3, i*3+1, i*3+2]*[j*3, j*3+1, j*3+2]
static void fillInTriplet(int i, int j, std::vector<Triplet>& triplets, const Matrix33& matrix) {
    for(int a = 0; a < 3; ++a)
        for(int b = 0; b < 3; ++b)
            triplets.push_back(Triplet(i*3+a, j*3+b, matrix(a, b)));
}

// add v into vector b in index [i*3, i*3+1, i*3+2]
static void addInVector(int i, Eigen::VectorXd& b, const Eigen::Vector3d& v) {
    for(int a = 0; a < 3; ++a)
        b[i*3+a] += v[a];
}

// To test if a sparse matrix A is symmetric
static bool isSymmetrical(Eigen::SparseMatrix<double>& A) {
	bool ret = true;
	Eigen::SparseMatrix<double> res = Eigen::SparseMatrix<double>(A.transpose()) - A;
	for (unsigned int k = 0; k < res.outerSize(); k++) {
		for (Eigen::SparseMatrix<double>::InnerIterator it(res, k); it; ++it)
		{
			if (std::fabs(it.value()) > 1e-6) {
				std::cout<< "row:{} col:{} value: {}" << it.row() << it.col() << A.coeffRef(it.row(), it.col());
				return false;
			}
		}
	}
	return ret;
}

// useful functions:
// 1. particleCoordinateToIdx()
// 2. isValidCoor()
bool ClothSim::advance() {
    igl::Timer t;
    t.start();
    if(m_method == 0) { // euler

        std::vector<Eigen::Vector3d> f;
        f.resize(particles.size());

        // TODO compute force
        for(int idx = 0; idx < particles.size(); ++idx) {
            
        }

        // TODO constrain fixed particles
        for(int i = 0; i < fixedParticleIdx.size(); ++i) {

        }

        // TODO update velocity and position of particles
        for(int idx = 0; idx < particles.size(); ++idx) {
        }

    } else if (m_method == 1) { // implicit euler
        
        std::vector<Triplet> triplets;
        Eigen::VectorXd b(particles.size()*3); b.setZero();
        SparseMatrix A(particles.size()*3, particles.size()*3);

        // TODO compute triplets arrays and right-hand-size b
        // NOTE: remember to process fixed particles, use isFixedParticle to test if a particle is fixed
        for(int idx = 0; idx < particles.size(); ++idx) {

        }
        
        A.setFromTriplets(triplets.begin(), triplets.end());
        // NOTE: this is for debug purpose, make sure your A matrix is symmetric
        // if(!isSymmetrical(A)) {std::cout<<"wrong"<<std::endl; return true;}
        cgSolver.compute(A);
        Eigen::VectorXd x = cgSolver.solve(b);
        // std::cout<< "error:" << cgSolver.error() << ", iter:" << cgSolver.iterations() << std::endl;

        // TODO with x computed, update position and velocity of particles
        for(int i = 0; i < particles.size(); ++i) {

        }
    } else {    // combination of a half step of explicit Euler and a half step of implicit Euler
        // TODO
    }
    
    // advance m_time
    m_time += m_dt;
    m_step++;

    // log
    if ((m_step % m_log_frequency) == 0) { 
        std::cout<<t.getElapsedTimeInMicroSec()<<std::endl;
    }
    
    return false;
}