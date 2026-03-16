#include "obj.hpp"

namespace inert {
    void PhysicsBody::integrateLinear(float deltaTime) {
        state.velocity.x += (state.forceAccum.x * state.inverseMass) * deltaTime;
        state.velocity.y += (state.forceAccum.y * state.inverseMass) * deltaTime;
        state.velocity.z += (state.forceAccum.z * state.inverseMass) * deltaTime;

        state.velocity.x *= powf(frictionMove, deltaTime);
        state.velocity.y *= powf(frictionMove, deltaTime);
        state.velocity.z *= powf(frictionMove, deltaTime);

        state.position.x += state.velocity.x * deltaTime;
        state.position.y += state.velocity.y * deltaTime;
        state.position.z += state.velocity.z * deltaTime;

        float speed = Vector3Length(state.velocity);

        linearActivity = true;
        if (speed < jitterCutoffLinear) {
            linearActivity = false;
            state.velocity = { 0.0f, 0.0f, 0.0f };
        }
    }

    
    void PhysicsBody::integrateAngular(float deltaTime) {
        state.rotatVel.x += (state.torqueAccum.x * state.inverseInertia.x) * deltaTime;
        state.rotatVel.y += (state.torqueAccum.y * state.inverseInertia.y) * deltaTime;
        state.rotatVel.z += (state.torqueAccum.z * state.inverseInertia.z) * deltaTime;

        state.rotatVel.x *= powf(frictionTurn, deltaTime);
        state.rotatVel.y *= powf(frictionTurn, deltaTime);
        state.rotatVel.z *= powf(frictionTurn, deltaTime);

        float rotSpeed = Vector3Length(state.rotatVel);
            
        if (rotSpeed > 0.0001f) { 
            Vector3 rotAxis = Vector3Scale(state.rotatVel, 1.0f / rotSpeed);
            Quaternion qDelta = QuaternionFromAxisAngle(rotAxis, rotSpeed * deltaTime);
            state.orientation = QuaternionNormalize(QuaternionMultiply(qDelta, state.orientation));
        }

        angularActivity = true;
        if (rotSpeed < jitterCutoffAngular) {
            angularActivity = false;
            state.rotatVel = { 0.0f, 0.0f, 0.0f };
        }
    }

    void PhysicsBody::applyAngularImpulse(Vector3 torqueImpulse) {
        if (Vector3LengthSqr(torqueImpulse) < 1e-10f) return;

        state.orientation = QuaternionNormalize(state.orientation);

        Quaternion invOrientation = QuaternionInvert(state.orientation);
        Vector3 localTorque = Vector3RotateByQuaternion(torqueImpulse, invOrientation);

        Vector3 localDelta = {
            localTorque.x * state.inverseInertia.x,
            localTorque.y * state.inverseInertia.y,
            localTorque.z * state.inverseInertia.z
        };

        state.rotatVel = Vector3Add(state.rotatVel,
            Vector3RotateByQuaternion(localDelta, state.orientation));
    }

    void PhysicsBody::applyImpulse(Vector3 contactVector, Vector3 impulse) {
        state.velocity = Vector3Add(state.velocity, Vector3Scale(impulse, state.inverseMass));
        applyAngularImpulse(Vector3CrossProduct(contactVector, impulse));

    }

    void PhysicsBody::applyImpulseAtPoint(Vector3 impulse, Vector3 contactPoint) {
        if (bodyType == BodyType::STATIC) return;

        Vector3 r = Vector3Subtract(contactPoint, state.position);

        state.velocity = Vector3Add(state.velocity, Vector3Scale(impulse, state.inverseMass));
        applyAngularImpulse(Vector3CrossProduct(r, impulse));

        linearActivity = true;
        angularActivity = true;
    }

    void PhysicsBody::debugDraw() {
        DrawSphere(state.position, 0.05f, RED);

        for (const auto& collider : colliders) {
            if (collider.type == ColliderType::POINT_CLOUD) {
                for (const auto& localPt : collider.localPoints) {
                    Vector3 worldPoint = Vector3Add(state.position, Vector3Transform(localPt, QuaternionToMatrix(state.orientation)));
                    DrawCube(worldPoint, 0.08f, 0.08f, 0.08f, BLUE);
                    DrawLine3D(state.position, worldPoint, Fade(GRAY, 0.5f));
                }
            }
        }
    }


}
