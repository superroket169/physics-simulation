#ifndef TYPES_HPP
#define TYPES_HPP

#include "vector.hpp"

namespace inert {

    struct PhysicsSettings {
        float gravityY         = -9.81f;
        int   solverIterations = 5;
        float spatialCellSize  = 3.0f;
        float distanceEpsilon  = 0.0001f;
        float velocityEpsilon  = 0.001f;
        float bounceThreshold  = 0.2f;
        float baseFrictionMu   = 0.5f;
        float baumgartePercent = 0.3f;
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
        bool  shouldCorrect = false;
    };

    struct ContactData {
        vec3f rA;             // contactPoint - positionA
        vec3f rB;             // contactPoint - positionB
        vec3f relVel;         // vB - vA (with angular contribution)
        float velAlongNormal  = 0.0f;
        float totalInvMass    = 0.0f;
    };

    struct ImpulseResult {
        vec3f normal;
        vec3f tangent;
        bool  shouldApply = false;
    };

} // namespace inert

#endif // !TYPES_HPP
