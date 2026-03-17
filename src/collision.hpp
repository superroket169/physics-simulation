#ifndef COLLISION_HPP
#define COLLISION_HPP

#include <vector>
#include "raylib.h"
#include "raymath.h"
#include "obj.hpp"
#include "math.hpp"
#include "hash.hpp"
#include "dispatch.hpp"

namespace inert {

    class PhysicsWorld {
    private:
        std::vector<PhysicsBody*> bodies;
        SpatialHash               spatialHash;
        CollisionDispatch         dispatch;

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

            dispatch = buildDefaultDispatch();
        }

        void addObject(PhysicsBody* body) { bodies.push_back(body); }
        void addGround(float y_level) {
            auto* plane = new StaticPlaneBody({0.0f, -1.0f, 0.0f}, {0.0f, y_level, 0.0f});
            addObject(plane);
        }

        void registerCollision(ColliderType a, ColliderType b, CollisionFn fn) {
            dispatch.registerCollision(a, b, fn);
        }

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
