#include "math.hpp"

namespace inert {
    namespace PureMath {

        float calculateAngularEffect(const PhysicsState& state, Vector3 r, Vector3 axis) {
            if (state.inverseMass == 0.0f) return 0.0f;
            
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

        NormalImpulseResult calculateNormalImpulse(const PhysicsState& stateA, const PhysicsState& stateB, float restitutionA, float restitutionB, const CollisionManifold& m) {
            NormalImpulseResult res = { {0, 0, 0}, 0.0f, false };
            
            Vector3 rA = Vector3Subtract(m.contactPoint, stateA.position);
            Vector3 rB = Vector3Subtract(m.contactPoint, stateB.position);
            
            Vector3 vA = Vector3Add(stateA.velocity, Vector3CrossProduct(stateA.rotatVel, rA));
            Vector3 vB = Vector3Add(stateB.velocity, Vector3CrossProduct(stateB.rotatVel, rB));
            Vector3 relVel = Vector3Subtract(vB, vA);

            float velAlongNormal = Vector3DotProduct(relVel, m.normal);
            if (velAlongNormal > 0) return res; 

            float angularEffectA = calculateAngularEffect(stateA, rA, m.normal);
            float angularEffectB = calculateAngularEffect(stateB, rB, m.normal);
            float totalInvMass = stateA.inverseMass + stateB.inverseMass;
            
            float e = std::min(restitutionA, restitutionB);
            float j = -(1.0f + e) * velAlongNormal;
            j /= (totalInvMass + angularEffectA + angularEffectB);

            res.impulse = Vector3Scale(m.normal, j);
            res.magnitude = j;
            res.shouldApply = true;
            
            return res;
        }

        Vector3 calculateTangentImpulse(const PhysicsState& stateA, const PhysicsState& stateB, const CollisionManifold& m, float normalImpulseMag, const PhysicsSettings& settings) {
            Vector3 rA = Vector3Subtract(m.contactPoint, stateA.position);
            Vector3 rB = Vector3Subtract(m.contactPoint, stateB.position);
            
            Vector3 vA = Vector3Add(stateA.velocity, Vector3CrossProduct(stateA.rotatVel, rA));
            Vector3 vB = Vector3Add(stateB.velocity, Vector3CrossProduct(stateB.rotatVel, rB));
            Vector3 relVel = Vector3Subtract(vB, vA);

            float velAlongNormal = Vector3DotProduct(relVel, m.normal);
            Vector3 normalVelocity = Vector3Scale(m.normal, velAlongNormal);
            Vector3 tangentVelocity = Vector3Subtract(relVel, normalVelocity);
            
            float tangentSpeed = Vector3Length(tangentVelocity);
            if (tangentSpeed < settings.velocityEpsilon) return {0, 0, 0};

            Vector3 t = Vector3Scale(tangentVelocity, 1.0f / tangentSpeed);
            
            float angularEffectA = calculateAngularEffect(stateA, rA, t);
            float angularEffectB = calculateAngularEffect(stateB, rB, t);
            float totalInvMass = stateA.inverseMass + stateB.inverseMass;
            
            float jt = -tangentSpeed / (totalInvMass + angularEffectA + angularEffectB);
            
            // NOTE : resting impulse implemente edilecek
            // float effectiveNormal = normalImpulseMag;
            // float massA = stateA.inverseMass > 0.0f ? (1.0f / stateA.inverseMass) : 0.0f;
            // float restingImpulse = massA * abs(settings.gravityY) * 0.016f; // m * g * dt (Yaklaşık)
            // çok özür ama sonradan tüm katsayıları değiştirilebilir yapacağım
            // if (effectiveNormal < restingImpulse) effectiveNormal = restingImpulse;

            float maxFriction = normalImpulseMag * settings.baseFrictionMu;
            jt = Clamp(jt, -maxFriction, maxFriction);
            
            return Vector3Scale(t, jt);
        }

    }
}
