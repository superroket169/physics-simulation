#ifndef OBJ_H
#define OBJ_H

#include <cstdint>
#include "raylib.h"
#include <vector>
#include "vector.hpp"

namespace inert {

    inline constexpr float GRAVITY_FORCE       = 9.81f;
    inline constexpr float frictionTurn        = 0.5f;
    inline constexpr float frictionMove        = 0.5f;
    inline constexpr float jitterCutoffLinear  = 0.001f;
    inline constexpr float jitterCutoffAngular = 0.001f;

    // Only use these where raylib drawing functions require Vector3
    inline Vector3 toRaylib(const vec3f& v)  { return { v[0], v[1], v[2] }; }
    inline vec3f fromRaylib(const Vector3& v) { return { v.x, v.y, v.z }; }

    enum class BodyType     { STATIC, DYNAMIC, KINEMATIC };
    enum class ColliderType { POINT_CLOUD, BOX, SPHERE };

    struct Collider {
        ColliderType type;

        // SPHERE      -> size.x = radius
        // POINT_CLOUD -> no use
        vec3f size;

        // SPHERE      -> no use
        // BOX         -> no use
        // POINT_CLOUD -> point list
        std::vector<vec3f> localPoints;
    };

    struct PhysicsState {
        vec3f position;
        vec3f velocity;
        quatf orientation;
        vec3f rotatVel;

        float mass        = 1.0f;
        float inverseMass = 1.0f;              // 1/m
        vec3f inertia     = { 0.5f, 0.5f, 0.8f };
        vec3f inverseInertia = { 2.0f, 2.0f, 1.25f }; // 1/I

        float restitution = 0.4f;

        vec3f forceAccum;
        vec3f torqueAccum;
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
        void applyAngularImpulse(vec3f torqueImpulse);

    public:
        PhysicsBody()          {}
        virtual ~PhysicsBody() {}

        // ==========================================
        //                  GETTERS
        // ==========================================

        vec3f    getPosition()         const { return state.position; }
        vec3f    getVelocity()         const { return state.velocity; }
        vec3f    getRotationVelocity() const { return state.rotatVel; }
        quatf    getOrientation()      const { return state.orientation; }
        float    getMass()             const { return state.mass; }
        vec3f    getInertia()          const { return state.inertia; }
        vec3f    getForceAccum()       const { return state.forceAccum; }
        vec3f    getTorqueAccum()      const { return state.torqueAccum; }
        float    getRestitution()      const { return state.restitution; }
        BodyType getBodyType()         const { return bodyType; }
        bool     getAngularActivity()  const { return angularActivity; }
        bool     getLinearActivity()   const { return linearActivity; }

        const std::vector<Collider>& getColliders() const { return colliders; }
        const PhysicsState&          getState()     const { return state; }

        vec3f getVelocityAtPoint(vec3f point) const {
            vec3f r = point - state.position;
            return state.velocity + getCrossProduct(state.rotatVel, r);
        }

        // ==========================================
        //                  ADDERS
        // ==========================================

        void addForce(vec3f force) {
            if (bodyType != BodyType::DYNAMIC) return;
            state.forceAccum += force;
        }
        void addForceAtPoint(vec3f force, vec3f offsetFromCenter) {
            if (bodyType != BodyType::DYNAMIC) return;
            addForce(force);
            addTorque(getCrossProduct(offsetFromCenter, force));
        }
        void addTorque(vec3f torque) {
            if (bodyType != BodyType::DYNAMIC) return;
            state.torqueAccum += torque;
        }
        void addVelocity(vec3f deltaV) {
            if (bodyType != BodyType::DYNAMIC) return;
            state.velocity += deltaV;
        }

        // ==========================================
        //                  SETTERS
        // ==========================================

        void setPosition(vec3f pos)      { state.position = pos; }
        void translate(vec3f deltaPos)   { state.position += deltaPos; }
        void setVelocity(vec3f v)        { if (bodyType == BodyType::DYNAMIC) state.velocity = v; }
        void setAngularVelocity(vec3f w) { if (bodyType == BodyType::DYNAMIC) state.rotatVel = w; }
        void setRestitution(float r)     { state.restitution = r; }
        void setLinearDamping(float)     { /* TODO */ }

        void setOrientation(quatf q) {
            if (bodyType == BodyType::DYNAMIC)
                state.orientation = q.getNormalized();
        }

        void setMass(float m) {
            state.mass        = m;
            state.inverseMass = (m > 0.0f) ? (1.0f / m) : 0.0f;
        }

        void setInertia(vec3f newInertia) {
            state.inertia = newInertia;
            state.inverseInertia[0] = (newInertia[0] > 0.0001f) ? 1.0f / newInertia[0] : 0.0f;
            state.inverseInertia[1] = (newInertia[1] > 0.0001f) ? 1.0f / newInertia[1] : 0.0f;
            state.inverseInertia[2] = (newInertia[2] > 0.0001f) ? 1.0f / newInertia[2] : 0.0f;
        }

        // ==========================================
        //                COLLIDER
        // ==========================================

        void addCollider(ColliderType mode, vec3f dimensions) {
            if (mode == ColliderType::POINT_CLOUD) return;
            colliders.push_back({ mode, dimensions, {} });
        }

        void addCollider(ColliderType mode, const std::vector<vec3f>& points) {
            if (mode != ColliderType::POINT_CLOUD) return;
            colliders.push_back({ mode, vec3f{}, points });
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
            state.forceAccum  = vec3f{};
            state.torqueAccum = vec3f{};
        }

        // ==========================================
        //              DEBUG DRAW
        // ==========================================
        virtual void debugDraw();

        // ==========================================
        //                IMPULSE
        // ==========================================
        void applyImpulse(vec3f contactVector, vec3f impulse);
        void applyImpulseAtPoint(vec3f impulse, vec3f contactPoint);
    };

} // namespace inert

#endif // OBJ_H
