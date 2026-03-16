#include "collision.hpp"
#include <unordered_set>

namespace inert {
    
    void PhysicsWorld::resolveManifold(PhysicsBody* bodyA, PhysicsBody* bodyB, const CollisionManifold& m) {
        if (!m.isColliding) return;

        const PhysicsState stateA = bodyA->getState();
        const PhysicsState stateB = (bodyB != nullptr) ? bodyB->getState() : groundState;

        const float restitutionA = bodyA->getRestitution();
        const float restitutionB = (bodyB != nullptr) ? bodyB->getRestitution() : 1.0f;

        // --- Pozisyon correection ---
        PositionalCorrectionResult posRes =
            PureMath::calculatePositionalCorrection(stateA, stateB, m, settings);

        if (posRes.shouldCorrect) {
            bodyA->translate(posRes.translationA);
            if (bodyB != nullptr) bodyB->translate(posRes.translationB);
        }

        // --- Normal impulse ---
        NormalImpulseResult normRes =
            PureMath::calculateNormalImpulse(stateA, stateB, restitutionA, restitutionB, m);

        if (normRes.shouldApply) {
            bodyA->applyImpulseAtPoint(Vector3Scale(normRes.impulse, -1.0f), m.contactPoint);
            if (bodyB != nullptr) bodyB->applyImpulseAtPoint(normRes.impulse, m.contactPoint);
        }

        // --- Tangent impulse ---
        if (normRes.shouldApply) {
            const Vector3 tangentImpulse =
                PureMath::calculateTangentImpulse(stateA, stateB, m, normRes.magnitude, settings);

            bodyA->applyImpulseAtPoint(Vector3Scale(tangentImpulse, -1.0f), m.contactPoint);
            if (bodyB != nullptr) bodyB->applyImpulseAtPoint(tangentImpulse, m.contactPoint);
        }
    }


    void PhysicsWorld::handleGroundCollisions() {
        if (!hasGroundCollision) return;

        for (auto body : bodies) {
            for (const auto& collider : body->getColliders()) {

                if (collider.type == ColliderType::SPHERE) {
                    const float radius = collider.size.x;
                    const float lowestPoint = body->getPosition().y - radius;

                    if (lowestPoint < groundLevel) {
                        CollisionManifold m;
                        m.isColliding  = true;
                        m.normal       = { 0.0f, 1.0f, 0.0f };
                        m.depth        = groundLevel - lowestPoint;
                        m.contactPoint = { body->getPosition().x, groundLevel, body->getPosition().z };
                        resolveManifold(body, nullptr, m);
                    }
                }
        
                else if (collider.type == ColliderType::POINT_CLOUD) {
                    const Matrix rotMat = QuaternionToMatrix(body->getState().orientation);
    
                    for (const auto& localPt : collider.localPoints) {
                        const Vector3 worldPoint = Vector3Add(
                            body->getPosition(),
                            Vector3Transform(localPt, rotMat)
                        );
    
                        if (worldPoint.y < groundLevel) {
                            CollisionManifold m;
                            m.isColliding  = true;
                            m.normal       = { 0.0f, 1.0f, 0.0f };
                            m.depth        = groundLevel - worldPoint.y;
                            m.contactPoint = worldPoint;
                            resolveManifold(body, nullptr, m);
                        }
                    }
                }
            }
        }
    }

    void PhysicsWorld::handleBodyCollisions() {
        std::unordered_set<PhysicsBody*> processed;

        for (auto bodyA : bodies) {
            processed.insert(bodyA);
            for (auto bodyB : spatialHash.getNeighbors(bodyA)) {
                if (processed.count(bodyB)) continue;

                for (const auto& colA : bodyA->getColliders()) {
                    for (const auto& colB : bodyB->getColliders()) {

                        if (colA.type == ColliderType::SPHERE &&
                            colB.type == ColliderType::SPHERE)
                        {
                            const CollisionManifold m = PureMath::checkSphereSphere(
                                bodyA->getState(), colA.size.x,
                                bodyB->getState(), colB.size.x,
                                settings
                            );
                            resolveManifold(bodyA, bodyB, m);
                        }
                        // TODO: diğer collider kombinasyonları
                    }
                }
            }
        }
    }

        
}
