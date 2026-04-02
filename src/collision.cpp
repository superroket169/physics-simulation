#include "collision.hpp"
#include <cstdio>
#include <unordered_set>

namespace inert {

    void PhysicsWorld::resolveManifold(PhysicsBody* bodyA, PhysicsBody* bodyB, CollisionManifold m) {
        if (!m.isColliding) return;

        const PhysicsState stateA = bodyA->getState();
        const PhysicsState stateB = (bodyB != nullptr) ? bodyB->getState() : groundState;

        const float restitutionA = bodyA->getRestitution();
        const float restitutionB = (bodyB != nullptr) ? bodyB->getRestitution() : 1.0f;

        // --- Positional correction (Baumgarte) ---
        {
            float totalInvMass = stateA.inverseMass + stateB.inverseMass;
            if (totalInvMass > 0.0f) {
                float correctionMag = std::max(m.depth - settings.baumgarteSlop, 0.0f)
                                      / totalInvMass * settings.baumgartePercent;
                if (correctionMag > 0.0001f) {
                    vec3f correction = m.normal * correctionMag;
                    bodyA->translate(correction *  stateA.inverseMass);
                    if (bodyB != nullptr)
                        bodyB->translate(correction * -stateB.inverseMass);
                }
            }
        }

        const PhysicsState stateA2 = bodyA->getState();
        const PhysicsState stateB2 = (bodyB != nullptr) ? bodyB->getState() : groundState;

        vec3f rA = m.contactPoint - stateA2.position;
        vec3f rB = m.contactPoint - stateB2.position;

        // rA = rA - m.normal * rA.getDotProduct(m.normal);
        // rB = rB - m.normal * rB.getDotProduct(m.normal);

        vec3f vA = stateA2.velocity + getCrossProduct(stateA2.rotatVel, rA);
        vec3f vB = stateB2.velocity + getCrossProduct(stateB2.rotatVel, rB);
        vec3f relVel = vA - vB;
        float velAlongNormal = relVel.getDotProduct(m.normal);

        if (velAlongNormal > 0.0f) return;

        // --- Normal impulse ---
        float angEffA = PureMath::calculateAngularEffect(stateA2, rA, m.normal);
        float angEffB = PureMath::calculateAngularEffect(stateB2, rB, m.normal);
        float totalInvMass = stateA2.inverseMass + stateB2.inverseMass;

        float e = std::min(restitutionA, restitutionB);
        if (std::abs(velAlongNormal) < settings.bounceThreshold)
            e = 0.0f;

        float j = -(1.0f + e) * velAlongNormal;
        j /= (totalInvMass + angEffA + angEffB);

        if (j < 0.0f) return; // güvenlik kontrolü

        vec3f normalImpulse = m.normal * j;

        bodyA->applyImpulseAtPoint( normalImpulse, m.contactPoint);
        if (bodyB != nullptr)
            bodyB->applyImpulseAtPoint(-normalImpulse, m.contactPoint);

        // --- Tangent (friction) impulse ---
        vec3f tangentVel = relVel - m.normal * velAlongNormal;
        float tangentSpeed = tangentVel.getLength();

        if (tangentSpeed >= settings.velocityEpsilon) {
            vec3f t = tangentVel * (1.0f / tangentSpeed);

            float angEffAt = PureMath::calculateAngularEffect(stateA2, rA, t);
            float angEffBt = PureMath::calculateAngularEffect(stateB2, rB, t);

            float jt = -tangentSpeed / (totalInvMass + angEffAt + angEffBt);

            // Coulomb: |jt| <= mu * j
            float jtMax = settings.baseFrictionMu * j;
            if (jt < -jtMax) jt = -jtMax;
            if (jt >  jtMax) jt =  jtMax;

            vec3f tangentImpulse = t * jt;

            bodyA->applyImpulseAtPoint( tangentImpulse, m.contactPoint);
            if (bodyB != nullptr)
                bodyB->applyImpulseAtPoint(-tangentImpulse, m.contactPoint);
        }
    }

    void PhysicsWorld::handleGroundCollisions() {
        if (!hasGroundCollision) return;

        for (auto body : bodies) {
            if (body->getBodyType() != BodyType::DYNAMIC) continue;

            for (const auto& collider : body->getColliders()) {
                if (collider.type == ColliderType::SPHERE) {
                    const float radius      = collider.size[0];
                    const float lowestPoint = body->getPosition()[1] - radius;

                    if (lowestPoint < groundLevel) {
                        CollisionManifold m;
                        m.isColliding  = true;
                        m.normal       = { 0.0f, 1.0f, 0.0f };
                        m.depth        = groundLevel - lowestPoint;
                        m.contactPoint = { body->getPosition()[0], groundLevel, body->getPosition()[2] };
                        resolveManifold(body, nullptr, m);
                    }
                }
                else if (collider.type == ColliderType::POINT_CLOUD) {
                    for (const auto& localPt : collider.localPoints) {
                        vec3f worldPt = body->getPosition() + rotate(localPt, body->getState().orientation);
                        if (worldPt[1] < groundLevel) {
                            CollisionManifold m;
                            m.isColliding  = true;
                            m.normal       = { 0.0f, 1.0f, 0.0f };
                            m.depth        = groundLevel - worldPt[1];
                            m.contactPoint = worldPt;
                            resolveManifold(body, nullptr, m);
                        }
                    }
                }
            }
        }
    }

    void PhysicsWorld::handleCollisions() {
        for (size_t i = 0; i < bodies.size(); i++) {
            for (size_t j = i + 1; j < bodies.size(); j++) {
                PhysicsBody* bodyA = bodies[i];
                PhysicsBody* bodyB = bodies[j];

                if (bodyA->getBodyType() == BodyType::STATIC &&
                    bodyB->getBodyType() == BodyType::STATIC) continue;

                for (const auto& colA : bodyA->getColliders()) {
                    for (const auto& colB : bodyB->getColliders()) {
                        PhysicsBody*    dynBody  = bodyA;
                        PhysicsBody*    statBody = bodyB;
                        const Collider* dynCol   = &colA;
                        const Collider* statCol  = &colB;

                        if (bodyA->getBodyType() == BodyType::STATIC) {
                            std::swap(dynBody,  statBody);
                            std::swap(dynCol,   statCol);
                        }

                        const CollisionManifold m = dispatch.dispatch(
                            *dynCol,  dynBody->getState(),
                            *statCol, statBody->getState(),
                            settings
                        );
                        resolveManifold(dynBody, statBody, m);
                    }
                }
            }
        }
    }

}
