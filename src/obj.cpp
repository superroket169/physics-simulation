#include "obj.hpp"

namespace inert {

    void PhysicsBody::integrateLinear(float deltaTime) {
        state.velocity += state.forceAccum * state.inverseMass * deltaTime;

        state.velocity *= powf(frictionMove, deltaTime);

        state.position += state.velocity * deltaTime;

        linearActivity = true;
        if (state.velocity.getLength() < jitterCutoffLinear) {
            linearActivity = false;
            state.velocity = vec3f{};
        }
    }

    void PhysicsBody::integrateAngular(float deltaTime) {
        state.rotatVel[0] += (state.torqueAccum[0] * state.inverseInertia[0]) * deltaTime;
        state.rotatVel[1] += (state.torqueAccum[1] * state.inverseInertia[1]) * deltaTime;
        state.rotatVel[2] += (state.torqueAccum[2] * state.inverseInertia[2]) * deltaTime;

        state.rotatVel *= powf(frictionTurn, deltaTime);

        float rotSpeed = state.rotatVel.getLength();

        if (rotSpeed > 0.0001f) {
            vec3f rotAxis = state.rotatVel * (1.0f / rotSpeed);
            quatf qDelta  = quatf::fromAxisAngle(rotAxis, rotSpeed * deltaTime);
            state.orientation = (qDelta * state.orientation).getNormalized();
        }

        angularActivity = true;
        if (rotSpeed < jitterCutoffAngular) {
            angularActivity = false;
            state.rotatVel = vec3f{};
        }
    }

    void PhysicsBody::applyAngularImpulse(vec3f torqueImpulse) {
        if (torqueImpulse.getLengthSqr() < 1e-10f) return;

        state.orientation = state.orientation.getNormalized();

        quatf invOrientation = state.orientation.getInverted();
        vec3f localTorque    = rotate(torqueImpulse, invOrientation);

        vec3f localDelta;
        localDelta[0] = localTorque[0] * state.inverseInertia[0];
        localDelta[1] = localTorque[1] * state.inverseInertia[1];
        localDelta[2] = localTorque[2] * state.inverseInertia[2];

        state.rotatVel += rotate(localDelta, state.orientation);
    }

    void PhysicsBody::applyImpulse(vec3f contactVector, vec3f impulse) {
        state.velocity += impulse * state.inverseMass;
        applyAngularImpulse(getCrossProduct(contactVector, impulse));
    }

    void PhysicsBody::applyImpulseAtPoint(vec3f impulse, vec3f contactPoint) {
        if (bodyType == BodyType::STATIC) return;

        vec3f r = contactPoint - state.position;

        state.velocity += impulse * state.inverseMass;
        applyAngularImpulse(getCrossProduct(r, impulse));

        linearActivity  = true;
        angularActivity = true;
    }

    void PhysicsBody::debugDraw() {
        // raylib still used for drawing only
        DrawSphere(toRaylib(state.position), 0.05f, RED);

        for (const auto& collider : colliders) {
            if (collider.type == ColliderType::POINT_CLOUD) {
                for (const auto& localPt : collider.localPoints) {
                    vec3f worldPt = state.position + rotate(localPt, state.orientation);
                    DrawCube(toRaylib(worldPt), 0.08f, 0.08f, 0.08f, BLUE);
                    DrawLine3D(toRaylib(state.position), toRaylib(worldPt), Fade(GRAY, 0.5f));
                }
            }
        }
    }

} // namespace inert
