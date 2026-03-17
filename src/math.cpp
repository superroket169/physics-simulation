#include "math.hpp"

namespace inert {
    namespace PureMath {

        float calculateAngularEffect(const PhysicsState& state, Vector3 r, Vector3 axis) {
            if (state.inverseMass == 0.0f) return 0.0f;
            if (Vector3LengthSqr(r) < 1e-8f)  return 0.0f;

            Vector3 rCrossAxis = Vector3CrossProduct(r, axis);
            Quaternion invOrientation = QuaternionInvert(state.orientation);
            Vector3 rCrossAxis_local = Vector3RotateByQuaternion(rCrossAxis, invOrientation);
            
            Vector3 invI_cross_local = { 
                rCrossAxis_local.x * state.inverseInertia.x, 
                rCrossAxis_local.y * state.inverseInertia.y, 
                rCrossAxis_local.z * state.inverseInertia.z 
            };
            
            Vector3 invI_cross = Vector3RotateByQuaternion(invI_cross_local, state.orientation);
            Vector3 cross_invI_cross_r = Vector3CrossProduct(invI_cross, r);
            
            return Vector3DotProduct(cross_invI_cross_r, axis);
        }

        CollisionManifold checkSphereSphere(const PhysicsState& stateA, float radiusA, const PhysicsState& stateB, float radiusB, const PhysicsSettings& settings) {
            CollisionManifold manifold;
            Vector3 delta = Vector3Subtract(stateB.position, stateA.position);
            float distSq = Vector3LengthSqr(delta);
            float sumRadii = radiusA + radiusB;
            
            if (distSq >= (sumRadii * sumRadii)) return manifold; 

            float distance = sqrtf(distSq);
            manifold.isColliding = true;

            if (distance <= settings.distanceEpsilon) {
                manifold.normal = { 0.0f, 1.0f, 0.0f };
                manifold.depth = sumRadii;
                manifold.contactPoint = stateA.position;
            }
            else {
                manifold.normal = Vector3Scale(delta, 1.0f / distance);
                manifold.depth = sumRadii - distance;
                manifold.contactPoint = Vector3Add(stateA.position, Vector3Scale(manifold.normal, radiusA - (manifold.depth * 0.5f)));
            }
            return manifold;
        }

        PositionalCorrectionResult calculatePositionalCorrection(const PhysicsState& stateA, const PhysicsState& stateB, const CollisionManifold& m, const PhysicsSettings& settings) {
            PositionalCorrectionResult res = { {0, 0, 0}, {0, 0, 0}, false };
            
            float totalInvMass = stateA.inverseMass + stateB.inverseMass;
            if (totalInvMass <= 0.0f) return res;
            
            float correctionMag = std::max(m.depth - settings.baumgarteSlop, 0.0f) / totalInvMass * settings.baumgartePercent;
            
            if (correctionMag > 0.0001f) {
                Vector3 correction = Vector3Scale(m.normal, correctionMag);
                res.translationA = Vector3Scale(correction, -stateA.inverseMass);
                res.translationB = Vector3Scale(correction, stateB.inverseMass);
                res.shouldCorrect = true;
            }
            
            return res;
        }

        ContactData buildContactData(const PhysicsState& stateA, const PhysicsState& stateB, const CollisionManifold& m) {
            ContactData cd;
            cd.rA = Vector3Subtract(m.contactPoint, stateA.position);
            cd.rB = Vector3Subtract(m.contactPoint, stateB.position);

            Vector3 vA = Vector3Add(stateA.velocity, Vector3CrossProduct(stateA.rotatVel, cd.rA));
            Vector3 vB = Vector3Add(stateB.velocity, Vector3CrossProduct(stateB.rotatVel, cd.rB));
            cd.relVel         = Vector3Subtract(vB, vA);
            cd.velAlongNormal = Vector3DotProduct(cd.relVel, m.normal);
            cd.totalInvMass   = stateA.inverseMass + stateB.inverseMass;
            return cd;
        }

        ImpulseResult calculateImpulses(const PhysicsState& stateA, const PhysicsState& stateB, float restitutionA, float restitutionB, const CollisionManifold& m, const ContactData& cd, const PhysicsSettings& settings) {
            ImpulseResult res = { {0,0,0}, {0,0,0}, false };

            if (cd.velAlongNormal > 0) return res;

            // --- Normal impulse ---
            float angularEffectA = calculateAngularEffect(stateA, cd.rA, m.normal);
            float angularEffectB = calculateAngularEffect(stateB, cd.rB, m.normal);

            float e = std::min(restitutionA, restitutionB);
            float j = -(1.0f + e) * cd.velAlongNormal;
            j /= (cd.totalInvMass + angularEffectA + angularEffectB);

            res.normal    = Vector3Scale(m.normal, j);
            res.shouldApply = true;

            // --- Tangent impulse ---
            Vector3 normalVelocity  = Vector3Scale(m.normal, cd.velAlongNormal);
            Vector3 tangentVelocity = Vector3Subtract(cd.relVel, normalVelocity);
            float   tangentSpeed    = Vector3Length(tangentVelocity);

            if (tangentSpeed >= settings.velocityEpsilon) {
                Vector3 t = Vector3Scale(tangentVelocity, 1.0f / tangentSpeed);

                float angularEffectAt = calculateAngularEffect(stateA, cd.rA, t);
                float angularEffectBt = calculateAngularEffect(stateB, cd.rB, t);

                float jt = -tangentSpeed / (cd.totalInvMass + angularEffectAt + angularEffectBt);
                jt = Clamp(jt, -j * settings.baseFrictionMu, j * settings.baseFrictionMu);

                res.tangent = Vector3Scale(t, jt);
            }

            return res;
        }

    }
}
