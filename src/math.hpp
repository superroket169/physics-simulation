#ifndef MATH_HPP
#define MATH_HPP

#include "types.hpp"
#include "obj.hpp"

namespace inert {
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

    } // namespace PureMath
} // namespace inert

#endif // !MATH_HPP
