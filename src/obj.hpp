#ifndef OBJ_H
#define OBJ_H

#include <cstdint>
#include "raylib.h"
#include "raymath.h"
#include <vector>


namespace inert {

    const float GRAVITY_FORCE = 9.81f;
    const float frictionTurn  =  0.5f;
    const float frictionMove  =  0.5f;

    const float jitterCutoffLinear = 0.001f;
    const float jitterCutoffAngular = 0.001f;
    
    enum class BodyType { STATIC, DYNAMIC, KINEMATIC };
    // enum class FrictionState { NONE, STATIC, KINETIC };
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

        // float muStatic = 0.90f;
        // float muKinetic = 0.80f;

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
        void integrateLinear(float deltaTime);
        void integrateAngular(float deltaTime);
        void applyAngularImpulse(Vector3 torqueImpulse);

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
                integrateLinear(deltaTime);
                integrateAngular(deltaTime);
            }

            clearAccumulators();
        }

        void inline clearAccumulators() {
            state.forceAccum = { 0.0f, 0.0f, 0.0f };
            state.torqueAccum = { 0.0f, 0.0f, 0.0f };
        }

        // ==========================================
        //              DEBUG DRAW
        // ==========================================
        
        virtual void debugDraw();

        // ==========================================
        //       QUATERNION ORIENTATION CALCS
        // ==========================================

        void applyImpulse(Vector3 contactVector, Vector3 impulse);
        void applyImpulseAtPoint(Vector3 impulse, Vector3 contactPoint);
    };
}

#endif
