#pragma once

#include "RigidBody.h"
#include "RigidBodySystem.h"

#include <Eigen/Dense>

class Scenarios
{
public:
    // Box filled with balls.
    //
    static void createBoxOnPlane(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        RigidBody* body0 = new RigidBody(100000.0f, new Plane(Eigen::Vector3f(0.0f, 1.0f, 0.0f)), "resources/plane.obj");
        body0->fixed = true;

        RigidBody* body1 = new RigidBody(1.0f, new Box(Eigen::Vector3f(1.0f, 1.0f, 1.0f)), "resources/box.obj");
        body1->x = Eigen::Vector3f(0.0f, 0.79f, 0.0f);

        RigidBody* body2 = new RigidBody(10.0f, new Box(Eigen::Vector3f(1.0f, 1.0f, 1.0f)), "resources/box.obj");
        body2->x = Eigen::Vector3f(2.0f, 0.79f, 0.0f);
        body2->xdot = Eigen::Vector3f(0.0f, 0.0f, 10.0f);

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
        rigidBodySystem.addBody(body2);
    }

    static void createBoxBallStack(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        RigidBody* body0 = new RigidBody(10.0f, new Plane(Eigen::Vector3f(0.0f, 1.0f, 0.0f)), "resources/plane.obj");
        body0->fixed = true;

        RigidBody* body1 = new RigidBody(1.0f, new Box(Eigen::Vector3f(1.0f, 1.0f, 1.0f)), "resources/box.obj");
        body1->x = Eigen::Vector3f(0.0f, 0.5f, 0.0f);
        RigidBody* body2 = new RigidBody(1.0f, new Sphere(0.5f), "resources/sphere.obj");
        body2->x = Eigen::Vector3f(0.0f, 1.5f, 0.0f);
        RigidBody* body3 = new RigidBody(1.0f, new Box(Eigen::Vector3f(1.0f, 1.0f, 1.0f)), "resources/box.obj");
        body3->x = Eigen::Vector3f(0.0f, 2.5f, 0.0f);
        RigidBody* body4 = new RigidBody(1.0f, new Sphere(0.5f), "resources/sphere.obj");
        body4->x = Eigen::Vector3f(0.0f, 3.5f, 0.0f);

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
        rigidBodySystem.addBody(body2);
        rigidBodySystem.addBody(body3);
        rigidBodySystem.addBody(body4);
    }

    static void createMarbleBox(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        std::cout << "Loading marble box scenario" << std::endl;

        // Create two layers of "marbles", in a grid layout
        int tire = 6;
        for (int i = 0; i < 9; ++i)
        {
            for (int j = 0; j < 9; ++j)
            {
                for (int k = 0; k < tire; k++) {
                    RigidBody* body{};
                    /*if (k == 4) body = new RigidBody(100.0f, new Sphere(0.5f), "resources/sphere.obj");
                    else body = new RigidBody(1.0f, new Sphere(0.5f), "resources/sphere.obj");*/
                    body = new RigidBody(1.0f, new Sphere(0.5f), "resources/sphere.obj");
                    body->x.x() = -4.0f + (float)i * 1.0f;
                    body->x.z() = -4.0f + (float)j * 1.0f;
                    body->x.y() = 2.0f + k * 1.0f;
                    rigidBodySystem.addBody(body);
                    body->mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f });
                    body->mesh->setTransparency(0.8f);
                }
            }
        }

        RigidBody* body0 = new RigidBody(10000.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.0f)), "resources/box_side.obj");
        RigidBody* body1 = new RigidBody(10000.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.0f)), "resources/box_side.obj");
        RigidBody* body2 = new RigidBody(10000.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.0f)), "resources/box_side.obj");
        RigidBody* body3 = new RigidBody(10000.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.4f)), "resources/box_side.obj");
        RigidBody* body4 = new RigidBody(10000.0f, new Box(Eigen::Vector3f(10.0f, 0.4f, 10.0f)), "resources/box_bot.obj");
        body0->fixed = true;
        body1->fixed = true;
        body2->fixed = true;
        body3->fixed = true;
        body4->fixed = true;
        body0->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body1->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body2->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body3->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body4->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body0->x.x() = 4.75f;
        body1->x.x() = -4.75f;
        body2->x.z() = 4.75f;
        body2->q = Eigen::AngleAxisf(1.57, Eigen::Vector3f(0, 1, 0));
        body3->x.z() = -4.75f;
        body3->q = Eigen::AngleAxisf(1.57, Eigen::Vector3f(0, 1, 0));
        body4->x.y() = -2.0f;

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
        rigidBodySystem.addBody(body2);
        rigidBodySystem.addBody(body3);
        rigidBodySystem.addBody(body4);
    }

    static void createSlideBallBox(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        std::cout << "Loading slideBall box scenario" << std::endl;

        int tire = 8;
        for (int i = 0; i < 10; ++i)
        {
            for (int j = 0; j < 10; ++j)
            {
                for (int k = 0; k < tire; k++) {
                    RigidBody* body = new RigidBody(1.0f, new Sphere(0.5f), "resources/sphere.obj");
                    if (k % 2) {
                        body->x.x() = -4.5f + (float)i * 1.0f;
                        body->x.z() = -4.5f + (float)j * 1.0f;
                    }
                    else {
                        body->x.x() = -4.0f + (float)i * 1.0f;
                        body->x.z() = -4.0f + (float)j * 1.0f;
                    }
                    body->x.y() = 2.0f + k * 1.0f;
                    rigidBodySystem.addBody(body);
                    body->mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f });
                    body->mesh->setTransparency(0.8f);
                }
            }
        }

        RigidBody* body0 = new RigidBody(100000.0f, new Box(Eigen::Vector3f(0.4f, 6.0f, 12.0f)), "resources/box_side_12.obj");
        RigidBody* body1 = new RigidBody(100000.0f, new Box(Eigen::Vector3f(0.4f, 6.0f, 12.0f)), "resources/box_side_12.obj");
        RigidBody* body2 = new RigidBody(100000.0f, new Box(Eigen::Vector3f(0.4f, 6.0f, 12.0f)), "resources/box_side_12.obj");
        RigidBody* body3 = new RigidBody(100000.0f, new Box(Eigen::Vector3f(0.4f, 6.0f, 12.4f)), "resources/box_side_12.obj");
        RigidBody* body4 = new RigidBody(100000.0f, new Box(Eigen::Vector3f(12.0f, 0.4f, 12.0f)), "resources/box_bot_12.obj");
        body0->fixed = true;
        body1->fixed = true;
        body2->fixed = true;
        body3->fixed = true;
        body4->fixed = true;
        body0->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body1->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body2->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body3->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body4->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body0->x.x() = 6.0f;
        body1->x.x() = -6.0f;
        body2->x.z() = 6.0f;
        body2->q = Eigen::AngleAxisf(1.57, Eigen::Vector3f(0, 1, 0));
        body3->x.z() = -6.0f;
        body3->q = Eigen::AngleAxisf(1.57, Eigen::Vector3f(0, 1, 0));
        body4->x.y() = -2.0f;

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
        rigidBodySystem.addBody(body2);
        rigidBodySystem.addBody(body3);
        rigidBodySystem.addBody(body4);
    }

    // Simple sphere falling on a box.
    //
    static void createSphereOnBox(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        std::cout << "Loading sphere-on-box scenario." << std::endl;

        // Create a sphere.
        RigidBody* bodySphere = new RigidBody(1.0f, new Sphere(0.5f), "resources/sphere.obj");
        bodySphere->x.y() = 4.0f;
        //bodySphere->xdot = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
        bodySphere->omega = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
        bodySphere->mesh->setTransparency(0.7f);

        RigidBody* bodyBox = new RigidBody(1.0f, new Box(Eigen::Vector3f(10.0f, 0.4f, 10.0f)), "resources/box_bot.obj");
        bodyBox->fixed = true;

        rigidBodySystem.addBody(bodySphere);
        rigidBodySystem.addBody(bodyBox);

        bodySphere->mesh->setSurfaceColor({ 0.1f, 1.0f, 0.2f })->setEdgeWidth(1.0f);
        bodyBox->mesh->setSurfaceColor({ 0.2f, 0.2f, 0.2f })->setSmoothShade(false)->setTransparency(0.5f);
    }

    static void createBoxFall(RigidBodySystem& rigidBodySystem) {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        std::cout << "Loading box fall scenario." << std::endl;

        int tire = 1;
        for (int i = 0; i < 1; ++i)
        {
            for (int j = 0; j < 1; ++j)
            {
                for (int k = 0; k < tire; k++) {
                    RigidBody* body = new RigidBody(1.0f, new Box(Eigen::Vector3f(1.0f, 1.0f, 1.0f)), "resources/box.obj");
                    /*body->x.x() = -4.0f + (float)i * 1.2f;
                    body->x.z() = -4.0f + (float)j * 1.2f;*/
                    body->x.x() = (float)i * 1.2f + 0.0f;
                    body->x.z() = (float)j * 1.2f;
                    body->x.y() = 2.0f + k * 1.2f;
                    rigidBodySystem.addBody(body);
                    body->mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f });
                    body->mesh->setTransparency(0.8f);
                    //if (j == 0) body->fixed = true;
                }
            }
        }
        RigidBody* bodyBox = new RigidBody(10000.0f, new Box(Eigen::Vector3f(10.0f, 0.4f, 10.0f)), "resources/box_bot.obj");
        bodyBox->x.x() = 0.2f;
        bodyBox->fixed = true;
        rigidBodySystem.addBody(bodyBox);

        bodyBox->mesh->setSurfaceColor({ 0.2f, 0.2f, 0.2f })->setSmoothShade(false)->setTransparency(0.5f);
    }

};
