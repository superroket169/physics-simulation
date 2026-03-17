#ifndef COLLISION_HPP
#define COLLISION_HPP

#include <vector>
#include "raylib.h"
#include "raymath.h"
#include "obj.hpp"
#include "math.hpp"
#include "hash.hpp"

namespace inert {

    class PhysicsWorld {
    private:
        std::vector<PhysicsBody*> bodies;
        SpatialHash               spatialHash;

        bool  hasGroundCollision = false;
        float groundLevel        = 0.0f;

        PhysicsState groundState;

        void integrate(float dt) {
            for (auto body : bodies)
                body->updateBody(dt);
        }

        void applyGravity() {
            for (auto body : bodies)
                if (body->getBodyType() == BodyType::DYNAMIC)
                    body->addForce({ 0.0f, body->getMass() * settings.gravityY, 0.0f });
        }

        void resolveManifold(PhysicsBody* bodyA, PhysicsBody* bodyB, const CollisionManifold& m);
        void handleGroundCollisions();
        void handleBodyCollisions();

    public:
        PhysicsSettings settings;

        PhysicsWorld() {
            groundState.inverseMass    = 0.0f;
            groundState.inverseInertia = { 0.0f, 0.0f, 0.0f };
            groundState.velocity       = { 0.0f, 0.0f, 0.0f };
            groundState.rotatVel       = { 0.0f, 0.0f, 0.0f };
        }

        void addObject(PhysicsBody* body) { bodies.push_back(body); }
        void addGround(float y_level)     { hasGroundCollision = true; groundLevel = y_level; }

        void step(float dt) {
            applyGravity();
            integrate(dt);

            spatialHash.setCellSize(settings.spatialCellSize);
            spatialHash.update(bodies);

            for (int k = 0; k < settings.solverIterations; k++) {
                handleGroundCollisions();
                handleBodyCollisions();
            }
        }
    };

} // namespace inert

#endif // COLLISION_HPP
