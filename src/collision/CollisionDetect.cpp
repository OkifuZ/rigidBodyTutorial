#include "collision/CollisionDetect.h"

#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/collision.h>

#include <iostream>

CollisionDetect::CollisionDetect(RigidBodySystem* rigidBodySystem) : m_rigidBodySystem(rigidBodySystem)
{
    req_fcl = hpp::fcl::CollisionRequest(hpp::fcl::CONTACT | hpp::fcl::DISTANCE_LOWER_BOUND, 1);
    req_fcl.enable_cached_gjk_guess = false;
    req_fcl.distance_upper_bound = 1e-4;
    req_fcl.enable_contact = true;
    req_fcl.num_max_contacts = 100;
}

void CollisionDetect::detectCollisions()
{
    // First, clear any existing contacts.
    //
    clear();

    // Next, loop over all pairs of bodies and test for contacts.
    //
    auto bodies = m_rigidBodySystem->getBodies();
    for(unsigned int i = 0; i < bodies.size(); ++i)
    {
        for(unsigned int j = i+1; j < bodies.size(); ++j)
        {
            RigidBody* body0 = bodies[i];
            RigidBody* body1 = bodies[j];

            // Special case: skip tests for pairs of static bodies.
            //
            if (body0->fixed && body1->fixed) 
                continue;

            // Test for sphere-sphere collision.
            if( body0->geometry->getType() == kSphere &&
                body1->geometry->getType() == kSphere )
            {
                collisionDetectSphereSphere(body0, body1);
            }
            // Test for sphere-box collision
            else if( body0->geometry->getType() == kSphere &&
                     body1->geometry->getType() == kBox )
            {
                collisionDetectSphereBox(body0, body1);
            }
            // Test for box-sphere collision (order swap)
            else if( body1->geometry->getType() == kSphere &&
                     body0->geometry->getType() == kBox )
            {
                collisionDetectSphereBox(body1, body0);
            }
        }
    }

}

void CollisionDetect::detectCollisions_fcl() {
    clear();

    auto bodies = m_rigidBodySystem->getBodies();
    for (auto& body : bodies) {
        if (body->fixed) continue;
        body->update_fcl_transform();
    }

    for (unsigned int i = 0; i < bodies.size(); ++i)
    {
        for (unsigned int j = i + 1; j < bodies.size(); ++j)
        {
            RigidBody* body0 = bodies[i];
            RigidBody* body1 = bodies[j];

            // Special case: skip tests for pairs of static bodies.
            //
            if (body0->fixed && body1->fixed)
                continue;

            // Test for sphere-sphere collision.
            if (body0->geometry->getType() == kSphere &&
                body1->geometry->getType() == kSphere)
            {
                collisionDetectSphereSphere(body0, body1);
            }
            // Test for sphere-box collision
            else if (body0->geometry->getType() == kSphere &&
                body1->geometry->getType() == kBox)
            {
                collisionDetectSphereBox(body0, body1);
            }
            // Test for box-sphere collision (order swap)
            else if (body1->geometry->getType() == kSphere &&
                body0->geometry->getType() == kBox)
            {
                collisionDetectSphereBox(body1, body0);
            }
            else if (body0->geometry->getType() == kBox &&
                body1->geometry->getType() == kBox)
            {
                collisionDetectBoxBox_fcl(body1, body0);
            }
            else if (body0->geometry->getType() == kBox &&
                body1->geometry->getType() == kPlane)
            {
                collisionDetectBoxPlane(body0, body1);
            }
            else if (body0->geometry->getType() == kPlane &&
                body1->geometry->getType() == kBox)
            {
                collisionDetectBoxPlane(body1, body0);
            }
        }
    }
    //print_contacts();
}

void CollisionDetect::computeContactJacobians()
{
    // Build constraint Jacobians for all contacts
    //
    #pragma omp parallel default(shared)
    {
        // Update velocities	
#pragma omp for schedule(static) 
        for (int i = 0; i < m_contacts.size(); i++)
        {
            auto& c = m_contacts[i];
            c->computeContactFrame();
            c->computeJacobian();
        }
    }
}

void CollisionDetect::clear()
{
    // First, remove all contacts from rigid bodies.
    //
    auto bodies = m_rigidBodySystem->getBodies();
    for (auto b : bodies)
    {
        b->contacts.clear();
    }

    // Then, cleanup the local contact array.
    //
    for(auto c : m_contacts)
    {
        delete c;
    }
    m_contacts.clear();

}

void CollisionDetect::collisionDetectSphereSphere(RigidBody* body0, RigidBody* body1)
{
    Sphere* sphere0 = dynamic_cast<Sphere*>(body0->geometry.get());
    Sphere* sphere1 = dynamic_cast<Sphere*>(body1->geometry.get());

    // Implement sphere-sphere collision detection.
    // The function should check if a collision exists, and if it does
    // compute the contact normal, contact point, and penetration depth.
    //
    Eigen::Vector3f vec = body0->x - body1->x;

    const float rsum = (sphere0->radius + sphere1->radius);
    const float dist = vec.norm();
    if( dist < rsum )
    {
        const Eigen::Vector3f n = vec / dist;
        const Eigen::Vector3f p = 0.5f * ((body0->x - sphere0->radius*n) + (body1->x + sphere1->radius*n));
        const float phi = dist-rsum;

        m_contacts.push_back( new Contact(body0, body1, p, n, phi) );
    }
}

void CollisionDetect::collisionDetectSpherePlane(RigidBody* body0, RigidBody* body1)
{
    // todo
}

void CollisionDetect::collisionDetectSphereBox(RigidBody* body0, RigidBody* body1)
{
    // TODO Implement sphere-box collision detection.
    //      The function should check if a collision exists.
    // 
    //      If it does, compute the contact normal, contact point, and penetration depth and
    //      create a Contact and add it to m_contacts.
    //

    Sphere* sphere = dynamic_cast<Sphere*>(body0->geometry.get());
    Box* box = dynamic_cast<Box*>(body1->geometry.get());

    // transform sphere to local coord of box
    const auto& trans = body1->x;
    const auto& rot = body1->q;
    Eigen::Vector3f pos_loc = rot.inverse() * (body0->x - trans);

    Eigen::Vector3f n = Eigen::Vector3f::Zero();
    auto clostP = ClosestPtPointBox(pos_loc, box->dim, n);
    if (n == Eigen::Vector3f::Zero()) {
        // sphere center in the box, let it go
        return;
    }

    float penerationSq = (clostP - pos_loc).norm() - sphere->radius - thickness;
    if (penerationSq < 0) {
        n = rot * n;
        n.normalize();
        Eigen::Vector3f p = rot * clostP + trans;
        float penetration = std::sqrt(-penerationSq);
        m_contacts.push_back(new Contact(body0, body1, p, n, -penetration));
    }
    else {
        // no collision
    }
}

void CollisionDetect::collisionDetectBoxBox_fcl(RigidBody* body0, RigidBody* body1) {
    hpp::fcl::collide(body0->fcl_convex, *body0->fcl_trans, body1->fcl_convex, *body1->fcl_trans, req_fcl, res_fcl);
    //hpp::fcl::collide(body0->fcl_shape.get(), *body0->fcl_trans, body1->fcl_shape.get(), *body1->fcl_trans, req_fcl, res_fcl);
    if (res_fcl.isCollision()) {
        const auto& contacts = res_fcl .getContacts();
        for (const auto& ct : contacts) {
            m_contacts.push_back(new Contact(body1, body0, ct.pos.cast<float>(), ct.normal.cast<float>(), -ct.penetration_depth));
        }
    }
    res_fcl.clear();
}

Eigen::Vector3f CollisionDetect::ClosestPtPointBox(const Eigen::Vector3f& pt, const Eigen::Vector3f& bdim) {
    Eigen::Vector3f q;
    Eigen::Vector3f bdim_half = bdim * 0.5f;
    for (int i = 0; i < 3; i++) {
        float v = pt[i];
        if (v <= -bdim_half[i]) v = -bdim_half[i];
        if (v >= bdim_half[i]) v = bdim_half[i];
        q[i] = v;
    }
    return q;
}

Eigen::Vector3f CollisionDetect::ClosestPtPointBox(const Eigen::Vector3f& pt, const Eigen::Vector3f& bdim, Eigen::Vector3f& normal) {
    Eigen::Vector3f q;
    Eigen::Vector3f bdim_half = bdim * 0.5f;
    for (int i = 0; i < 3; i++) {
        float v = pt[i];
        if (v <= -bdim_half[i]) {
            v = -bdim_half[i];
            normal[i] = -1;
        }
        else if (v >= bdim_half[i]) {
            v = bdim_half[i];
            normal[i] = 1;
        }
        else {
            normal[i] = 0;
        }
        q[i] = v;
    }
    //if (normal != Eigen::Vector3f::Zero()) normal.normalize();
    return q;
}



static inline bool collisionDetectPointPlane(const Eigen::Vector3f& p, const Eigen::Vector3f& plane_p, const Eigen::Vector3f& plane_n, float& phi)
{
    const float dp = (p - plane_p).dot(plane_n);
    if (dp < 0.0f)
    {
        phi = std::min(0.0f, dp);
        return true;
    }
    return false;
}


void CollisionDetect::collisionDetectBoxPlane(RigidBody* body0, RigidBody* body1)
{
    Box* box = dynamic_cast<Box*>(body0->geometry.get());
    Plane* plane = dynamic_cast<Plane*>(body1->geometry.get());
    const Eigen::Vector3f pplane = body1->x;
    const Eigen::Vector3f nplane = body1->q * plane->normal;
    const Eigen::Vector3f plocal[8] = {
        0.5f * Eigen::Vector3f(-box->dim(0), -box->dim(1), -box->dim(2)),
        0.5f * Eigen::Vector3f(-box->dim(0), -box->dim(1),  box->dim(2)),
        0.5f * Eigen::Vector3f(-box->dim(0),  box->dim(1), -box->dim(2)),
        0.5f * Eigen::Vector3f(-box->dim(0),  box->dim(1),  box->dim(2)),
        0.5f * Eigen::Vector3f(box->dim(0), -box->dim(1), -box->dim(2)),
        0.5f * Eigen::Vector3f(box->dim(0), -box->dim(1),  box->dim(2)),
        0.5f * Eigen::Vector3f(box->dim(0),  box->dim(1), -box->dim(2)),
        0.5f * Eigen::Vector3f(box->dim(0),  box->dim(1),  box->dim(2))
    };

    for (unsigned int i = 0; i < 8; ++i)
    {
        const Eigen::Vector3f pbox = body0->q * plocal[i] + body0->x;
        float phi;
        if (collisionDetectPointPlane(pbox, pplane, nplane, phi))
        {
            m_contacts.push_back(new Contact(body0, body1, pbox, nplane, phi));
        }
    }
}


void CollisionDetect::print_contacts() {
    for (int i = 0; i < m_contacts.size(); i++) {
        const auto& ct = m_contacts[i];
        std::cout << i << " = p: " << ct->p.transpose() << ", n: " << ct->n.transpose() << ", phi: " << ct->phi << "\n";
    }
}