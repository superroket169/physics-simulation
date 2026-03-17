#include "collision.hpp"
#include <unordered_set>

namespace inert {

    void PhysicsWorld::resolveManifold(PhysicsBody* bodyA, PhysicsBody* bodyB, const CollisionManifold& m) {
        if (!m.isColliding) return;

        const PhysicsState stateA = bodyA->getState();
        const PhysicsState stateB = bodyB->getState();

        const float restitutionA = bodyA->getRestitution();
        const float restitutionB = bodyB->getRestitution();

        PositionalCorrectionResult posRes =
            PureMath::calculatePositionalCorrection(stateA, stateB, m, settings);

        if (posRes.shouldCorrect) {
            bodyA->translate(posRes.translationA);
            bodyB->translate(posRes.translationB);
        }

        const ContactData cd = PureMath::buildContactData(stateA, stateB, m);

        const ImpulseResult imp =
            PureMath::calculateImpulses(stateA, stateB, restitutionA, restitutionB, m, cd, settings);

        if (imp.shouldApply) {
            bodyA->applyImpulseAtPoint(Vector3Scale(imp.normal,  -1.0f), m.contactPoint);
            bodyA->applyImpulseAtPoint(Vector3Scale(imp.tangent, -1.0f), m.contactPoint);
            bodyB->applyImpulseAtPoint(imp.normal,  m.contactPoint);
            bodyB->applyImpulseAtPoint(imp.tangent, m.contactPoint);
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
