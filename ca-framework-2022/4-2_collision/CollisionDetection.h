#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <igl/ray_mesh_intersect.h>
#include <set>
#include <utility>
#include <vector>
#include "AABB.h"
#include "RigidObject.h"

enum class ContactType { VERTEXFACE, EDGEEDGE, NONE };

struct Contact {
    RigidObject* a;      // body containing vertex
    RigidObject* b;      // body containing face
    Eigen::Vector3d n;   // outwards pointing normal of face
    Eigen::Vector3d p;   // world-space vertex location
    Eigen::Vector3d ea;  // edge direction for A
    Eigen::Vector3d eb;  // edge direction for B

    ContactType type;  // type of contact
};

class CollisionDetection {
   public:
    CollisionDetection(std::vector<RigidObject>& world) : m_objects(world) {}

    // pass objects in scene to collision detection
    void setObjects(std::vector<RigidObject>& world) { m_objects = world; }

	void computeBroadPhase(int broadPhaseMethod);

    // test if ray from start towards end intersects object with vertices V and
    // faces F
    ContactType isColliding(const Eigen::Vector3d& start,
                            const Eigen::Vector3d& end,
                            const Eigen::MatrixXd& V,
                            const Eigen::MatrixXi& F) {
        std::vector<igl::Hit> hits;
        igl::ray_mesh_intersect(start, end - start, V, F, hits);

        // count number of intersections with object that happen between start
        // and end (so along the edge)
        int cntr = 0;
        for (auto hit : hits) {
            if (hit.t > 0 && hit.t <= 1) {
                cntr++;
            }
        }

        // if hits is odd then the ray enters through one face and
        // does not leave again through another, hence the starting point is
        // inside the object, if the number of intersections between start and
        // end of edge are even, then the edge entering through one and leaving
        // through another face, hence the edge is intersection the object
        ContactType ret = ContactType::NONE;
        if (hits.size() % 2 == 1) {
            ret = ContactType::VERTEXFACE;
        }
        if (cntr % 2 == 0 && cntr > 0) {
            ret = ContactType::EDGEEDGE;
        }
        return ret;
    }

    // compute normal for all faces and use it to find closesest face to
    // given vertex
    Contact findVertexFaceCollision(const Eigen::Vector3d& vertex,
                                    const Eigen::MatrixXd& V,
                                    const Eigen::MatrixXi& F) {
        double minDist = std::numeric_limits<double>::infinity();
        Eigen::Vector3d minNormal;
        for (int i = 0; i < F.rows(); i++) {
            Eigen::Vector3d a = V.row(F(i, 0));
            Eigen::Vector3d b = V.row(F(i, 1));
            Eigen::Vector3d c = V.row(F(i, 2));

            Eigen::Vector3d n = -(b - a).cross(c - a).normalized();

            Eigen::Vector3d v = vertex - a;

            double distance = v.dot(n);
            // if vertex inside
            if (distance >= 0) {
                if (distance < minDist) {
                    minDist = distance;
                    minNormal = -n;
                }
            }
        }
        Contact ret;
        ret.n = minNormal;
        ret.p = vertex + minDist * minNormal;
        return ret;
    }

    // loop over edges of faces and compute closest to given edge
    Contact findEdgeEdgeCollision(const Eigen::Vector3d& start,
                                  const Eigen::Vector3d& end,
                                  const Eigen::MatrixXd& V,
                                  const Eigen::MatrixXi& F) {
        double minDist = std::numeric_limits<double>::infinity();
        Contact ret;
        for (int i = 0; i < F.rows(); i++) {
            Eigen::Vector3d a = V.row(F(i, 0));
            Eigen::Vector3d b = V.row(F(i, 1));
            Eigen::Vector3d c = V.row(F(i, 2));

            Eigen::Vector3d n_face = -(b - a).cross(c - a).normalized();

            for (int j = 0; j < 3; j++) {
                Eigen::Vector3d s = V.row(F(i, j));
                Eigen::Vector3d e = V.row(F(i, (j + 1) % 3));

                Eigen::Vector3d ea = end - start;
                Eigen::Vector3d eb = e - s;
                Eigen::Vector3d n =
                    (ea).cross(eb);  // direction of shortest distance
                double distance = n.dot(start - s) / n.norm();

                Eigen::Vector3d plane_normal = n.cross(eb).normalized();
                double t =
                    (s - start).dot(plane_normal) / (ea.dot(plane_normal));
                if (n_face.dot(n) < 0 && distance < 0 && -distance < minDist &&
                    t >= 0 && t <= 1) {
                    ret.ea = ea;
                    ret.eb = eb;
                    ret.n = n;
                    ret.p = start + t * ea;
                    minDist = -distance;
                }
            }
        }
        return ret;
    }

	void computeNarrowPhase(int narrowPhaseMethod);

	void applyImpulse(double eps = 1.0);

    void clearDataStructures() {
        m_penetratingEdges.clear();
        m_penetratingVertices.clear();
        m_overlappingBodys.clear();
        m_contacts.clear();
    }

    void computeCollisionDetection(int broadPhaseMethod = 0,
                                   int narrowPhaseMethod = 0,
                                   double eps = 1.0) {
        clearDataStructures();

        computeBroadPhase(broadPhaseMethod);

        computeNarrowPhase(narrowPhaseMethod);

        applyImpulse(eps);
    }

    std::vector<Contact> getContacts() { return m_contacts; }

    std::vector<RigidObject>& m_objects;  // all objects in scene
    // result of broadphase, pairs of objects with possible collisions
    std::vector<std::pair<size_t, size_t>> m_overlappingBodys;

    // set of vertex indices that penetrate a face, used to avoid duplicates
    std::set<int> m_penetratingVertices;
    // set of pairs of vertex indices that represent a penetrating edge, used to
    // avoid duplicates
    std::set<std::pair<int, int>> m_penetratingEdges;

    // computed contact points
    std::vector<Contact> m_contacts;
};

#endif
