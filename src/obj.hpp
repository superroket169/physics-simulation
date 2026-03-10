#ifndef OBJ_H
#define OBJ_H

#include <cstdint>
#include "raylib.h"
#include "raymath.h"
#include <vector>


namespace inert {

    const float GRAVITY_FORCE = 9.81f;
    const float frictionTurn  = 0.98f;
    const float frictionMove  = 0.98f;

    const float jitterCutoffLinear = 0.1f;
    const float jitterCutoffAngular = 0.5f;
    
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

        // TODO: now just ground colisions
        void applyCollisions() {
            if (!hasGroundCollision) return;

            for (const auto& collider : colliders) {
                if (collider.type == ColliderType::POINT_CLOUD) {
                    for (const auto& localPt : collider.localPoints) {
                        Vector3 worldPoint = Vector3Add(state.position, Vector3Transform(localPt, QuaternionToMatrix(state.orientation)));
                        if (worldPoint.y < groundLevel) {
                            resolveCollision(worldPoint, { 0.0f, 1.0f, 0.0f });
                            state.position.y += (groundLevel - worldPoint.y); 
                        }
                    }
                }
            }
        }

    public:
        PhysicsBody() {}
        virtual ~PhysicsBody() {}

        // ============== GETTERS ==================
        
        Vector3 getPosition() const { return state.position; }
        Vector3 getVelocity() const { return state.velocity; }
        Quaternion getOrientation() const { return state.orientation; }
        
        bool getAngularActivity() { return angularActivity; }
        bool getLinearActivity() { return linearActivity; }
        
        // ==========================================
        //                  ADDERS
        // ==========================================
        
        void addForce(Vector3 force) {
            if (bodyType != BodyType::DYNAMIC) return;
            state.forceAccum = Vector3Add(state.forceAccum, force);
        }
        void addForceAtPoint(Vector3 force, Vector3 worldPoint) {
            if (bodyType != BodyType::DYNAMIC) return;
            addForce(force);
            
            Vector3 r = Vector3Subtract(worldPoint, state.position);
            Vector3 torque = Vector3CrossProduct(r, force);
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
        
        void setPosition(Vector3 Pos) { state.position = Pos; }
        void move(Vector3 deltaPos) { state.position = Vector3Add(state.position, deltaPos); }
        void setRestitution(float bounciness) { state.restitution = bounciness; }
        void setLinearDamping(float friction) { /* TODO: state.linearFriction = friction; */ }
        void setVelocity(Vector3 v) { 
            if (bodyType == BodyType::DYNAMIC) state.velocity = v; 
        }
        void setAngularVelocity(Vector3 w) { 
            if (bodyType == BodyType::DYNAMIC) state.rotatVel = w; 
        }
        void setMass(float m) { 
            state.mass = m; 
            state.inverseMass = (m > 0.0f) ? (1.0f / m) : 0.0f; 
        }
        void setInertia(Vector3 newInertia) {
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
        
        void addGround(float y_level) {
            hasGroundCollision = true;
            groundLevel = y_level;
        }

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

            applyCollisions();
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

            Vector3 torqueImpulse = Vector3CrossProduct(contactVector, impulse);
            
            state.rotatVel.x += torqueImpulse.x * state.inverseInertia.x;
            state.rotatVel.y += torqueImpulse.y * state.inverseInertia.y;
            state.rotatVel.z += torqueImpulse.z * state.inverseInertia.z;
        }

        // Main Collision
        void resolveCollision(Vector3 hitPoint, Vector3 normal) {
            Vector3 r = Vector3Subtract(hitPoint, state.position);
            Vector3 pointVelocity = Vector3Add(state.velocity, Vector3CrossProduct(state.rotatVel, r));

            float velAlongNormal = Vector3DotProduct(pointVelocity, normal);
            if (velAlongNormal > 0) return; 

            float normalImpulseMag = resolveNormalCollision(r, normal, pointVelocity, velAlongNormal);

            Vector3 newPointVelocity = Vector3Add(state.velocity, Vector3CrossProduct(state.rotatVel, r));
            
            resolveTangentCollision(r, normal, newPointVelocity, normalImpulseMag);
        }

        // Vertical
        float resolveNormalCollision(Vector3 r, Vector3 normal, Vector3 pointVelocity, float velAlongNormal) {

            Vector3 rCrossN = Vector3CrossProduct(r, normal);

            Quaternion qRot = state.orientation;

            Quaternion invOrientation = QuaternionInvert(qRot); 
            Vector3 rCrossN_local = Vector3RotateByQuaternion(rCrossN, invOrientation);

            Vector3 invI_crossRN_local = { 
                rCrossN_local.x / state.inertia.x, 
                rCrossN_local.y / state.inertia.y, 
                rCrossN_local.z / state.inertia.z 
            };

            Vector3 invI_crossRN = Vector3RotateByQuaternion(invI_crossRN_local, qRot);
            Vector3 cross_invI_crossRN_r = Vector3CrossProduct(invI_crossRN, r);
            
            float angularEffect = Vector3DotProduct(cross_invI_crossRN_r, normal);
            
            float j = -(1.0f + state.restitution) * velAlongNormal;
            j /= state.inverseMass + angularEffect;

            Vector3 impulse = Vector3Scale(normal, j);
            applyImpulse(r, impulse);

            return j;
        }

        // Coulomb friction state fn
        FrictionState determineFrictionState(float requiredTangentImpulse, float normalImpulseMag, float muStatic) {
            if (abs(requiredTangentImpulse) < normalImpulseMag * muStatic)
                return FrictionState::STATIC;
            return FrictionState::KINETIC;
        }

        // Tangent Collision
        void resolveTangentCollision(Vector3 r, Vector3 normal, Vector3 pointVelocity, float normalImpulseMag) {
            // TODO: 
        }
    };
}

#endif
