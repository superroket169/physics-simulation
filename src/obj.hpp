#ifndef OBJ_H
#define OBJ_H

#include <cstdint>
#include "raylib.h"
#include "raymath.h"
#include <vector>

namespace inert {

    inline constexpr float GRAVITY_FORCE       = 9.81f;
    inline constexpr float frictionTurn        = 0.5f;
    inline constexpr float frictionMove        = 0.5f;
    inline constexpr float jitterCutoffLinear  = 0.001f;
    inline constexpr float jitterCutoffAngular = 0.001f;

    enum class BodyType     { STATIC, DYNAMIC, KINEMATIC };
    enum class ColliderType { POINT_CLOUD, BOX, SPHERE };

    
    struct Collider {
        ColliderType type;

        // SPHERE      -> size.x = radius
        // POINT_CLOUD -> no use
        Vector3 size;

        // SPHERE      -> no use
        // BOX         -> no use
        // POINT_CLOUD -> point list
        std::vector<Vector3> localPoints;
    };


    struct PhysicsState {
        Vector3    position     = { 0.0f, 0.0f, 0.0f };
        Vector3    velocity     = { 0.0f, 0.0f, 0.0f };
        Quaternion orientation  = { 0.0f, 0.0f, 0.0f, 1.0f };
        Vector3    rotatVel     = { 0.0f, 0.0f, 0.0f };

        float   mass           = 1.0f;
        float   inverseMass    = 1.0f;              // 1/m
        Vector3 inertia        = { 0.5f, 0.5f, 0.8f };
        Vector3 inverseInertia = { 2.0f, 2.0f, 1.25f }; // 1/I

        float restitution = 0.4f;

        Vector3 forceAccum  = { 0.0f, 0.0f, 0.0f };
        Vector3 torqueAccum = { 0.0f, 0.0f, 0.0f };
    };

    class PhysicsBody {
    protected:
        PhysicsState          state;
        BodyType              bodyType = BodyType::DYNAMIC;
        std::vector<Collider> colliders;

        bool angularActivity = true;
        bool linearActivity  = true;

        // ==========================================
        //            PHYSICS INTEGRATORS
        // ==========================================
        void integrateLinear (float deltaTime);
        void integrateAngular(float deltaTime);
        void applyAngularImpulse(Vector3 torqueImpulse);

    public:
        PhysicsBody()          {}
        virtual ~PhysicsBody() {}

        // ==========================================
        //                  GETTERS
        // ==========================================

        Vector3    getPosition()         const { return state.position; }
        Vector3    getVelocity()         const { return state.velocity; }
        Vector3    getRotationVelocity() const { return state.rotatVel; }
        Quaternion getOrientation()      const { return state.orientation; }
        float      getMass()             const { return state.mass; }
        Vector3    getInertia()          const { return state.inertia; }
        Vector3    getForceAccum()       const { return state.forceAccum; }
        Vector3    getTorqueAccum()      const { return state.torqueAccum; }
        float      getRestitution()      const { return state.restitution; }
        BodyType   getBodyType()         const { return bodyType; }
        bool       getAngularActivity()  const { return angularActivity; }
        bool       getLinearActivity()   const { return linearActivity; }

        const std::vector<Collider>& getColliders() const { return colliders; }
        const PhysicsState&          getState()     const { return state; }

        Vector3 getVelocityAtPoint(Vector3 point) const {
            Vector3 r = Vector3Subtract(point, state.position);
            return Vector3Add(state.velocity, Vector3CrossProduct(state.rotatVel, r));
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
            addTorque(Vector3CrossProduct(offsetFromCenter, force));
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

        void setPosition(Vector3 pos)      { state.position = pos; }
        void translate(Vector3 deltaPos)   { state.position = Vector3Add(state.position, deltaPos); }
        void setVelocity(Vector3 v)        { if (bodyType == BodyType::DYNAMIC) state.velocity = v; }
        void setAngularVelocity(Vector3 w) { if (bodyType == BodyType::DYNAMIC) state.rotatVel = w; }
        void setRestitution(float r)       { state.restitution = r; }
        void setLinearDamping(float)       { /* TODO */ }

        void setOrientation(Quaternion q) {
            if (bodyType == BodyType::DYNAMIC)
                state.orientation = QuaternionNormalize(q);
        }

        void setMass(float m) {
            state.mass        = m;
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
        //                COLLIDER
        // ==========================================

        void addCollider(ColliderType mode, Vector3 dimensions) {
            if (mode == ColliderType::POINT_CLOUD) return;
            colliders.push_back({ mode, dimensions, {} });
        }

        void addCollider(ColliderType mode, const std::vector<Vector3>& points) {
            if (mode != ColliderType::POINT_CLOUD) return;
            colliders.push_back({ mode, {}, points });
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

        inline void clearAccumulators() {
            state.forceAccum  = { 0.0f, 0.0f, 0.0f };
            state.torqueAccum = { 0.0f, 0.0f, 0.0f };
        }

        // ==========================================
        //              DEBUG DRAW
        // ==========================================
        virtual void debugDraw();

        // ==========================================
        //                IMPULSE
        // ==========================================
        void applyImpulse(Vector3 contactVector, Vector3 impulse);
        void applyImpulseAtPoint(Vector3 impulse, Vector3 contactPoint);
    };



} // namespace inert

#endif // OBJ_H
