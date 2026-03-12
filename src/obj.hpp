#ifndef OBJ_H
#define OBJ_H

#include <cstdint>
#include "raylib.h"
#include "raymath.h"
#include <vector>


namespace inert {

    const float GRAVITY_FORCE = 9.81f;
    const float frictionTurn  =  0.98f;
    const float frictionMove  =  0.98f;

    const float jitterCutoffLinear = 0.001f;
    const float jitterCutoffAngular = 0.001f;
    
    enum class BodyType { STATIC, DYNAMIC, KINEMATIC };
    enum class FrictionState { NONE, STATIC, KINETIC };
    enum class ColliderType { POINT_CLOUD, BOX, SPHERE };

    struct Collider {
        ColliderType type;
        Vector3 size;
        std::vector<Vector3> localPoints;
    };
    
    struct PhysicsState {
        Vector3 position = { 0.0f, 0.0f, 0.0f };
        Vector3 velocity = { 0.0f, 0.0f, 0.0f };
        
        Quaternion orientation = { 0.0f, 0.0f, 0.0f, 1.0f }; 
        Vector3 rotatVel = { 0.0f, 0.0f, 0.0f }; 
        
        float mass = 1.0f;
        float inverseMass = 1.0f; // 1/m

        Vector3 inertia = { 0.5f, 0.5f, 0.8f }; 
        Vector3 inverseInertia = { 2.0f, 2.0f, 1.25f }; // 1/I
        
        float restitution = 0.4f;

        float muStatic = 0.90f;
        float muKinetic = 0.80f;

        // Accumulators
        Vector3 forceAccum = { 0.0f, 0.0f, 0.0f };
        Vector3 torqueAccum = { 0.0f, 0.0f, 0.0f };
    };

    class PhysicsBody {
    protected: 
        PhysicsState state;
        BodyType bodyType = BodyType::DYNAMIC;
        std::vector<Collider> colliders;
        bool hasGroundCollision = false;
        float groundLevel = 0.0f;

        bool angularActivity = true;
        bool linearActivity = true;

        // ==========================================
        //            PHYSICS INTEGRATERS
        // ==========================================
        void integrateLinear(float deltaTime) {
            state.velocity.x += (state.forceAccum.x * state.inverseMass) * deltaTime;
            state.velocity.y += (state.forceAccum.y * state.inverseMass) * deltaTime;
            state.velocity.z += (state.forceAccum.z * state.inverseMass) * deltaTime;

            state.velocity.x *= frictionMove;
            state.velocity.y *= frictionMove;
            state.velocity.z *= frictionMove;

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

        void integrateAngular(float deltaTime) {
            state.rotatVel.x += (state.torqueAccum.x * state.inverseInertia.x) * deltaTime;
            state.rotatVel.y += (state.torqueAccum.y * state.inverseInertia.y) * deltaTime;
            state.rotatVel.z += (state.torqueAccum.z * state.inverseInertia.z) * deltaTime;

            state.rotatVel.x *= frictionTurn;
            state.rotatVel.y *= frictionTurn;
            state.rotatVel.z *= frictionTurn;

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

    public:
        PhysicsBody() {}
        virtual ~PhysicsBody() {}

        // ============== GETTERS ==================
        
        Vector3 getPosition() const { return state.position; }
        Vector3 getVelocity() const { return state.velocity; }
        Vector3 getRotationVelocity() const { return state.rotatVel; }
        Quaternion getOrientation() const { return state.orientation; }
        
        float getMass() const { return state.mass; }
        Vector3 getInertia() const { return state.inertia; }
        Vector3 getForceAccum() const { return state.forceAccum; }
        Vector3 getTorqueAccum() const { return state.torqueAccum; }

        bool getAngularActivity() { return angularActivity; }
        bool getLinearActivity() { return linearActivity; }

        const std::vector<Collider>& getColliders() const { return colliders; }
        const PhysicsState& getState() const { return state; }
        float getRestitution() const { return state.restitution; }
        BodyType getBodyType() const { return bodyType; }

        Vector3 getVelocityAtPoint(Vector3 point) const {
            Vector3 r = Vector3Subtract(point, state.position);
            Vector3 angularVelEffect = Vector3CrossProduct(state.rotatVel, r);
            return Vector3Add(state.velocity, angularVelEffect);
        }
        
        // ==========================================
        //                  ADDERS
        // ==========================================
        
        void addForce(Vector3 force) {
            if (bodyType != BodyType::DYNAMIC) return;
            state.forceAccum = Vector3Add(state.forceAccum, force);
        }
        void addForceAtPoint(Vector3 force, Vector3 offsetFromCenter) {
            if (bodyType != BodyType::DYNAMIC) return;
            addForce(force);
            Vector3 torque = Vector3CrossProduct(offsetFromCenter, force);
            addTorque(torque);
        }
        void addTorque(Vector3 torque) {
            if (bodyType != BodyType::DYNAMIC) return;
            state.torqueAccum = Vector3Add(state.torqueAccum, torque);
        }
        void addVelocity(Vector3 deltaV) {
            if (bodyType != BodyType::DYNAMIC) return;
            state.velocity = Vector3Add(state.velocity, deltaV);
        }
        
        // ==========================================
        //                  SETTERS
        // ==========================================
        
        void setPosition(Vector3 Pos)           { state.position = Pos; }
        // void move(Vector3 deltaPos)             { state.position = Vector3Add(state.position, deltaPos); } // TODO
        void translate(Vector3 deltaPos)        { state.position = Vector3Add(state.position, deltaPos); }
        void setRestitution(float bounciness)   { state.restitution = bounciness; }
        void setLinearDamping(float friction)   { /* TODO: state.linearFriction = friction; */ }
        void setVelocity(Vector3 v)             { if (bodyType == BodyType::DYNAMIC) state.velocity = v; }
        void setAngularVelocity(Vector3 w)      { if (bodyType == BodyType::DYNAMIC) state.rotatVel = w; }
        void setMass(float m)                   { state.mass = m; state.inverseMass = (m > 0.0f) ? (1.0f / m) : 0.0f; }
        void setOrientation(Quaternion q)       { if (bodyType == BodyType::DYNAMIC) state.orientation = QuaternionNormalize(q); }
        void setInertia(Vector3 newInertia)     {
            state.inertia = newInertia;
            state.inverseInertia = {
                (newInertia.x > 0.0001f) ? 1.0f / newInertia.x : 0.0f,
                (newInertia.y > 0.0001f) ? 1.0f / newInertia.y : 0.0f,
                (newInertia.z > 0.0001f) ? 1.0f / newInertia.z : 0.0f
            };
        }

        // ==========================================
        //         HITBOX & COLLIDER SYSTEM
        // ==========================================

        // for box & sphere
        void addCollider(ColliderType mode, Vector3 dimensions) {
            Collider newCollider;
            newCollider.type = mode;
            newCollider.size = dimensions;
            colliders.push_back(newCollider);
        }

        // for Point Cloud
        void addCollider(ColliderType mode, const std::vector<Vector3>& points) {
            if (mode != ColliderType::POINT_CLOUD) return;
            Collider newCollider;
            newCollider.type = mode;
            newCollider.localPoints = points;
            colliders.push_back(newCollider);
        }

        // ==========================================
        //            PHYSICS UPDATES
        // ==========================================

        virtual void updateBody(float deltaTime) {
            if (bodyType == BodyType::STATIC) return; 

            if (bodyType == BodyType::DYNAMIC) {
                addForce({ 0.0f, -GRAVITY_FORCE * state.mass, 0.0f });

                integrateLinear(deltaTime);
                integrateAngular(deltaTime);
            }

            clearAccumulators();
        }

        void clearAccumulators() {
            state.forceAccum = { 0.0f, 0.0f, 0.0f };
            state.torqueAccum = { 0.0f, 0.0f, 0.0f };
        }

        // ==========================================
        //              DEBUG DRAW
        // ==========================================
        virtual void debugDraw() {
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

        // ==========================================
        //       QUATERNION ORIENTATION CALCS
        // ==========================================

        void applyImpulse(Vector3 contactVector, Vector3 impulse) {
            state.velocity = Vector3Add(state.velocity, Vector3Scale(impulse, state.inverseMass));

            Vector3 worldTorqueImpulse = Vector3CrossProduct(contactVector, impulse);
            
            Quaternion invOrientation = QuaternionInvert(state.orientation);
            Vector3 localTorque = Vector3RotateByQuaternion(worldTorqueImpulse, invOrientation);
            
            Vector3 localRotVelDelta = {
                localTorque.x * state.inverseInertia.x,
                localTorque.y * state.inverseInertia.y,
                localTorque.z * state.inverseInertia.z
            };
            
            Vector3 worldRotVelDelta = Vector3RotateByQuaternion(localRotVelDelta, state.orientation);
            
            state.rotatVel = Vector3Add(state.rotatVel, worldRotVelDelta);
        }

        void applyImpulseAtPoint(Vector3 impulse, Vector3 contactPoint) {
            if (bodyType == BodyType::STATIC) return;

            Vector3 r = Vector3Subtract(contactPoint, state.position);
            
            Vector3 velocityChange = Vector3Scale(impulse, state.inverseMass);
            state.velocity = Vector3Add(state.velocity, velocityChange);
            
            Vector3 angularImpulse = Vector3CrossProduct(r, impulse);
            
            Vector3 rotatVelChange = {
                angularImpulse.x * state.inverseInertia.x,
                angularImpulse.y * state.inverseInertia.y,
                angularImpulse.z * state.inverseInertia.z
            };
            
            state.rotatVel = Vector3Add(state.rotatVel, rotatVelChange);

            linearActivity = true;
            angularActivity = true;
        }
    };
}

#endif
