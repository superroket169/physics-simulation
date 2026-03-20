#include "math.hpp"

namespace inert {
    namespace PureMath {

        float calculateAngularEffect(const PhysicsState& state, vec3f r, vec3f axis) {
            if (state.inverseMass == 0.0f) return 0.0f;
            if (r.getLengthSqr() < 1e-8f)  return 0.0f;

            vec3f rCrossAxis       = getCrossProduct(r, axis);
            quatf invOrientation   = state.orientation.getInverted();
            vec3f rCrossAxis_local = rotate(rCrossAxis, invOrientation);

            vec3f invI_cross_local = {
                rCrossAxis_local[0] * state.inverseInertia[0],
                rCrossAxis_local[1] * state.inverseInertia[1],
                rCrossAxis_local[2] * state.inverseInertia[2]
            };

            vec3f invI_cross      = rotate(invI_cross_local, state.orientation);
            vec3f cross_invI_cross_r = getCrossProduct(invI_cross, r);

            return cross_invI_cross_r.getDotProduct(axis);
        }

        CollisionManifold checkSphereSphere(const PhysicsState& stateA, float radiusA, const PhysicsState& stateB, float radiusB, const PhysicsSettings& settings) {
            CollisionManifold manifold;
            vec3f delta  = stateB.position - stateA.position;
            float distSq = delta.getLengthSqr();
            float sumRadii = radiusA + radiusB;

            if (distSq >= (sumRadii * sumRadii)) return manifold;

            float distance = sqrtf(distSq);
            manifold.isColliding = true;

            if (distance <= settings.distanceEpsilon) {
                manifold.normal       = { 0.0f, 1.0f, 0.0f };
                manifold.depth        = sumRadii;
                manifold.contactPoint = stateA.position;
            } else {
                manifold.normal       = delta * (1.0f / distance);
                manifold.depth        = sumRadii - distance;
                manifold.contactPoint = stateA.position + manifold.normal * (radiusA - manifold.depth * 0.5f);
            }
            return manifold;
        }

        PositionalCorrectionResult calculatePositionalCorrection(const PhysicsState& stateA, const PhysicsState& stateB, const CollisionManifold& m, const PhysicsSettings& settings) {
            PositionalCorrectionResult res = { vec3f{}, vec3f{}, false };

            float totalInvMass = stateA.inverseMass + stateB.inverseMass;
            if (totalInvMass <= 0.0f) return res;

            float correctionMag = std::max(m.depth - settings.baumgarteSlop, 0.0f) / totalInvMass * settings.baumgartePercent;

            if (correctionMag > 0.0001f) {
                vec3f correction   = m.normal * correctionMag;
                res.translationA   = correction * (-stateA.inverseMass);
                res.translationB   = correction * stateB.inverseMass;
                res.shouldCorrect  = true;
            }

            return res;
        }

        ContactData buildContactData(const PhysicsState& stateA, const PhysicsState& stateB, const CollisionManifold& m) {
            ContactData cd;
            cd.rA = m.contactPoint - stateA.position;
            cd.rB = m.contactPoint - stateB.position;

            vec3f vA      = stateA.velocity + getCrossProduct(stateA.rotatVel, cd.rA);
            vec3f vB      = stateB.velocity + getCrossProduct(stateB.rotatVel, cd.rB);
            cd.relVel         = vB - vA;
            cd.velAlongNormal = cd.relVel.getDotProduct(m.normal);
            cd.totalInvMass   = stateA.inverseMass + stateB.inverseMass;
            return cd;
        }

        ImpulseResult calculateImpulses(const PhysicsState& stateA, const PhysicsState& stateB, float restitutionA, float restitutionB, const CollisionManifold& m, const ContactData& cd, const PhysicsSettings& settings) {
            ImpulseResult res = { vec3f{}, vec3f{}, false };

            if (cd.velAlongNormal > 0) return res;

            // --- Normal impulse ---
            float angularEffectA = calculateAngularEffect(stateA, cd.rA, m.normal);
            float angularEffectB = calculateAngularEffect(stateB, cd.rB, m.normal);

            float e = std::min(restitutionA, restitutionB);
            float j = -(1.0f + e) * cd.velAlongNormal;
            j /= (cd.totalInvMass + angularEffectA + angularEffectB);

            res.normal     = m.normal * j;
            res.shouldApply = true;

            // --- Tangent impulse ---
            vec3f normalVelocity  = m.normal * cd.velAlongNormal;
            vec3f tangentVelocity = cd.relVel - normalVelocity;
            float tangentSpeed    = tangentVelocity.getLength();

            if (tangentSpeed >= settings.velocityEpsilon) {
                vec3f t = tangentVelocity * (1.0f / tangentSpeed);

                float angularEffectAt = calculateAngularEffect(stateA, cd.rA, t);
                float angularEffectBt = calculateAngularEffect(stateB, cd.rB, t);

                float jt = -tangentSpeed / (cd.totalInvMass + angularEffectAt + angularEffectBt);
                jt = std::max(-j * settings.baseFrictionMu, std::min(jt, j * settings.baseFrictionMu));

                res.tangent = t * jt;
            }

            return res;
        }

    }
}
