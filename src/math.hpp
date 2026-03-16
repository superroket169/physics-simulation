#ifndef MATH_HPP
#define MATH_HPP

#include <algorithm>
#include "raylib.h"
#include "raymath.h"
#include "obj.hpp"

namespace inert {

    struct PhysicsSettings {
        float gravityY          = -9.81f;
        int solverIterations    = 5;
        float spatialCellSize   = 3.0f;
        float distanceEpsilon   = 0.0001f;
        float velocityEpsilon   = 0.001f;
        float bounceThreshold   = 0.2f;
        float baseFrictionMu    = 0.5f;
        float baumgartePercent  = 0.8f;
        float baumgarteSlop     = 0.01f;
    };

    struct CollisionManifold {
        bool isColliding = false;
        Vector3 normal = { 0.0f, 0.0f, 0.0f };
        float depth = 0.0f;
        Vector3 contactPoint = { 0.0f, 0.0f, 0.0f };
    };

    struct PositionalCorrectionResult {
        Vector3 translationA;
        Vector3 translationB;
        bool shouldCorrect;
    };

    struct NormalImpulseResult {
        Vector3 impulse; 
        float magnitude; 
        bool shouldApply;
    };

    namespace PureMath {

        float calculateAngularEffect
            (const PhysicsState& state, Vector3 r, Vector3 axis);
        
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
        
        NormalImpulseResult calculateNormalImpulse
            (const PhysicsState& stateA,
             const PhysicsState& stateB,
             float restitutionA,
             float restitutionB,
             const CollisionManifold& m);

        Vector3 calculateTangentImpulse
            (const PhysicsState& stateA,
             const PhysicsState& stateB,
             const CollisionManifold& m,
             float normalImpulseMag, const
             PhysicsSettings& settings);
    }
}

#endif
