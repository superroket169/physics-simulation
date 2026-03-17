#include "collision.hpp"
#include <unordered_set>

namespace inert {

    void PhysicsWorld::resolveManifold(PhysicsBody* bodyA, PhysicsBody* bodyB, const CollisionManifold& m) {
        if (!m.isColliding) return;

        const PhysicsState stateA = bodyA->getState();
        const PhysicsState stateB = (bodyB != nullptr) ? bodyB->getState() : groundState;

        const float restitutionA = bodyA->getRestitution();
        const float restitutionB = (bodyB != nullptr) ? bodyB->getRestitution() : 1.0f;

        PositionalCorrectionResult posRes =
            PureMath::calculatePositionalCorrection(stateA, stateB, m, settings);

        if (posRes.shouldCorrect) {
            bodyA->translate(posRes.translationA);
            if (bodyB != nullptr) bodyB->translate(posRes.translationB);
        }

        const ContactData cd = PureMath::buildContactData(stateA, stateB, m);

        const ImpulseResult imp =
            PureMath::calculateImpulses(stateA, stateB, restitutionA, restitutionB, m, cd, settings);

        if (imp.shouldApply) {
            bodyA->applyImpulseAtPoint(Vector3Scale(imp.normal,  -1.0f), m.contactPoint);
            bodyA->applyImpulseAtPoint(Vector3Scale(imp.tangent, -1.0f), m.contactPoint);
            if (bodyB != nullptr) {
                bodyB->applyImpulseAtPoint(imp.normal,  m.contactPoint);
                bodyB->applyImpulseAtPoint(imp.tangent, m.contactPoint);
            }
        }
    }

    void PhysicsWorld::handleGroundCollisions() {
        if (!hasGroundCollision) return;

        for (auto body : bodies) {
            if (body->getBodyType() != BodyType::DYNAMIC) continue;

            for (const auto& collider : body->getColliders()) {
                if (collider.type == ColliderType::SPHERE) {
                    const float radius      = collider.size.x;
                    const float lowestPoint = body->getPosition().y - radius;

                    if (lowestPoint < groundLevel) {
                        CollisionManifold m;
                        m.isColliding  = true;
                        m.normal       = { 0.0f, -1.0f, 0.0f };
                        m.depth        = groundLevel - lowestPoint;
                        m.contactPoint = { body->getPosition().x, groundLevel, body->getPosition().z };
                        resolveManifold(body, nullptr, m);
                    }
                }
                else if (collider.type == ColliderType::POINT_CLOUD) {
                    const Matrix rotMat = QuaternionToMatrix(body->getState().orientation);
                    for (const auto& localPt : collider.localPoints) {
                        const Vector3 worldPt = Vector3Add(
                            body->getPosition(),
                            Vector3Transform(localPt, rotMat)
                        );
                        if (worldPt.y < groundLevel) {
                            CollisionManifold m;
                            m.isColliding  = true;
                            m.normal       = { 0.0f, -1.0f, 0.0f };
                            m.depth        = groundLevel - worldPt.y;
                            m.contactPoint = worldPt;
                            resolveManifold(body, nullptr, m);
                        }
                    }
                }
            }
        }
    }

    void PhysicsWorld::handleCollisions() {
        std::unordered_set<PhysicsBody*> processed;

        for (auto bodyA : bodies) {
            processed.insert(bodyA);
            for (auto bodyB : spatialHash.getNeighbors(bodyA)) {
                if (processed.count(bodyB)) continue;

                for (const auto& colA : bodyA->getColliders()) {
                    for (const auto& colB : bodyB->getColliders()) {
                        const CollisionManifold m = dispatch.dispatch(
                            colA, bodyA->getState(),
                            colB, bodyB->getState(),
                            settings
                        );
                        resolveManifold(bodyA, bodyB, m);
                    }
                }
            }
        }
    }

}
