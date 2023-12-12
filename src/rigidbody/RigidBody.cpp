#include "rigidbody/RigidBody.h"
#include "contact/Contact.h"
#include "util/MeshAssets.h"
#include "polyscope/surface_mesh.h"

#include <hpp/fcl/shape/convex.h>

int RigidBody::counter = 0;


hpp::fcl::Convex<hpp::fcl::Quadrilateral>* buildBox(hpp::fcl::FCL_REAL l, hpp::fcl::FCL_REAL w, hpp::fcl::FCL_REAL d) {
    hpp::fcl::Vec3f* pts = new hpp::fcl::Vec3f[8];
    pts[0] = hpp::fcl::Vec3f(l, w, d);
    pts[1] = hpp::fcl::Vec3f(l, w, -d);
    pts[2] = hpp::fcl::Vec3f(l, -w, d);
    pts[3] = hpp::fcl::Vec3f(l, -w, -d);
    pts[4] = hpp::fcl::Vec3f(-l, w, d);
    pts[5] = hpp::fcl::Vec3f(-l, w, -d);
    pts[6] = hpp::fcl::Vec3f(-l, -w, d);
    pts[7] = hpp::fcl::Vec3f(-l, -w, -d);

    hpp::fcl::Quadrilateral* polygons = new hpp::fcl::Quadrilateral[6];
    polygons[0].set(0, 2, 3, 1);  // x+ side
    polygons[1].set(2, 6, 7, 3);  // y- side
    polygons[2].set(4, 5, 7, 6);  // x- side
    polygons[3].set(0, 1, 5, 4);  // y+ side
    polygons[4].set(1, 3, 7, 5);  // z- side
    polygons[5].set(0, 2, 6, 4);  // z+ side

    return new hpp::fcl::Convex<hpp::fcl::Quadrilateral>(true,
        pts,  // points
        8,    // num points
        polygons,
        6  // number of polygons
    );
}

RigidBody::RigidBody(float _mass, Geometry* _geometry, const std::string& _filename) :
    fixed(false),
    mass(_mass),
    x(0,0,0),
    //R(Eigen::Matrix3f::Identity()),
    xdot(0,0,0),
    omega(0,0,0),
    q(1,0,0,0),
    Ibody(Eigen::Matrix3f::Identity()),
    IbodyInv(Eigen::Matrix3f::Identity()),
    Iinv(Eigen::Matrix3f::Zero()),
    f(0,0,0),
    tau(0,0,0),
    fc(0,0,0),
    tauc(0,0,0),
    geometry(_geometry),
    contacts(),
    mesh(nullptr)
{
    if (geometry->getType() == kBox) {
        Box* box = dynamic_cast<Box*>(geometry.get());
        fcl_convex = buildBox(box->dim[0] / 2, box->dim[1] / 2, box->dim[2] / 2);
        fcl_shape = std::make_unique<hpp::fcl::Box>(box->dim[0], box->dim[1], box->dim[2]);
        fcl_trans = std::make_unique<hpp::fcl::Transform3f>(q.cast<double>(), x.cast<double>());
    }

    Ibody = geometry->computeInertia(mass);
    IbodyInv = Ibody.inverse();


    if( !_filename.empty() )
    {
        // Read the mesh using the asset registry, which caches previously loaded meshes.
        auto* cachedMesh = MeshAssetRegistry::loadObj(_filename);

        if (cachedMesh != nullptr)
        {
            // Register the mesh with Polyscope
            mesh = polyscope::registerSurfaceMesh(std::to_string(RigidBody::counter), cachedMesh->meshV, cachedMesh->meshF);
            mesh->setSmoothShade(true);
        }
    }
    contacts.clear();
    RigidBody::counter++;
}


void RigidBody::updateInertiaMatrix()
{
    if( !fixed )
    {
        I = q * Ibody * q.inverse();
        Iinv = q * IbodyInv * q.inverse();
    }
    else
    {
        Iinv.setZero();
    }
}

void RigidBody::addForceAtPos(const Eigen::Vector3f& pos, const Eigen::Vector3f& force)
{
    const Eigen::Vector3f r = pos - x;
    f += force;
    tau += r.cross(force);
}

void RigidBody::getVelocityAtPos(const Eigen::Vector3f& pos, Eigen::Vector3f& vel)
{
    const Eigen::Vector3f r = pos - x;
    vel = xdot + r.cross(omega);
}

void RigidBody::update_fcl_transform() {
    if (fcl_trans) {
        fcl_trans->setTransform(q.cast<double>(), x.cast<double>());
    }
}
