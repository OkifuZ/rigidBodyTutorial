#pragma once

#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/collision.h>

#include <Eigen/Dense>
#include <vector>

class Contact;
class RigidBody;
class RigidBodySystem;

//
// The main collision detection class,
// it will test the geometries of each pair of rigid bodies
// and populate an array with contacts.
//
class CollisionDetect
{
public:

    // Constructor.
    //
    CollisionDetect(RigidBodySystem* rigidBodySystem);

    // Tests for collisions between all pairs of bodies in the rigid body system
    // and generates contacts for any intersecting geometries.
    // The array of generated contacts can be retrieved by calling getContacts().
    //
    void detectCollisions();

    void detectCollisions_fcl();

    void clear();

    // Compute the Jacobians for contacts
    void computeContactJacobians();

    const std::vector<Contact*>& getContacts() const { return m_contacts; }
    std::vector<Contact*>& getContacts() { return m_contacts; }

    void print_contacts();

private:

    // Sphere-sphere collision test.
    // Assumes that both @a body0 and @a body1 have a sphere collision geometry.
    //
    void collisionDetectSphereSphere(RigidBody* body0, RigidBody* body1);

    // Sphere-box collision test.
    // Assumes that @a body0 has a sphere geometry and @a body1 has a box geometry.
    //
    void collisionDetectSphereBox(RigidBody* body0, RigidBody* body1);
    void collisionDetectSpherePlane(RigidBody* body0, RigidBody* body1);

    void collisionDetectBoxBox_fcl(RigidBody* body0, RigidBody* body1);

    void collisionDetectBoxPlane(RigidBody* body0, RigidBody* body1);

    Eigen::Vector3f ClosestPtPointBox(const Eigen::Vector3f& pt, const Eigen::Vector3f& bdim);
    Eigen::Vector3f ClosestPtPointBox(const Eigen::Vector3f& pt, const Eigen::Vector3f& bdim, Eigen::Vector3f& normal);

    hpp::fcl::CollisionRequest req_fcl;
    hpp::fcl::CollisionResult res_fcl;


private:

    RigidBodySystem* m_rigidBodySystem;     // Rigid body system where collision detection is performed.
    std::vector<Contact*> m_contacts;       // Contact array.
    const float thickness = 0.0f;
};
