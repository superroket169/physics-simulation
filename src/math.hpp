#ifndef MATH_HPP
#define MATH_HPP

#include <algorithm>
#include "obj.hpp"

namespace inert {

    struct PhysicsSettings {
        float gravityY         = -9.81f;
        int   solverIterations = 5;
        float spatialCellSize  = 3.0f;
        float distanceEpsilon  = 0.0001f;
        float velocityEpsilon  = 0.001f;
        float bounceThreshold  = 0.2f;
        float baseFrictionMu   = 0.5f;
        float baumgartePercent = 0.8f;
        float baumgarteSlop    = 0.01f;
    };

    struct CollisionManifold {
        bool  isColliding  = false;
        vec3f normal;
        float depth        = 0.0f;
        vec3f contactPoint;
    };

    struct PositionalCorrectionResult {
        vec3f translationA;
        vec3f translationB;
        bool  shouldCorrect;
    };

    struct ContactData {
        vec3f rA;             // contactPoint - positionA
        vec3f rB;             // contactPoint - positionB
        vec3f relVel;         // vB - vA (with angular contribution)
        float velAlongNormal;
        float totalInvMass;
    };

    struct ImpulseResult {
        vec3f normal;
        vec3f tangent;
        bool  shouldApply;
    };

    namespace PureMath {

        float calculateAngularEffect
            (const PhysicsState& state, vec3f r, vec3f axis);

        ContactData buildContactData
            (const PhysicsState& stateA,
             const PhysicsState& stateB,
             const CollisionManifold& m);

        CollisionManifold checkSphereSphere
            (const PhysicsState& stateA,
             float radiusA,
             const PhysicsState& stateB,
             float radiusB,
             const PhysicsSettings& settings);

        PositionalCorrectionResult calculatePositionalCorrection
            (const PhysicsState& stateA,
             const PhysicsState& stateB,
             const CollisionManifold& m,
             const PhysicsSettings& settings);

        ImpulseResult calculateImpulses
            (const PhysicsState& stateA,
             const PhysicsState& stateB,
             float restitutionA,
             float restitutionB,
             const CollisionManifold& m,
             const ContactData& cd,
             const PhysicsSettings& settings);
    }
}

#endif
