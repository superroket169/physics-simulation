#ifndef INERT_DISPATCH_HPP
#define INERT_DISPATCH_HPP

#include <functional>
#include <unordered_map>
#include "math.hpp"

namespace inert {

    using CollisionFn = std::function<
        CollisionManifold(
            const Collider&,       const PhysicsState&,
            const Collider&,       const PhysicsState&,
            const PhysicsSettings&
        )
    >;

    inline int makeKey(ColliderType a, ColliderType b) {
        int ia = static_cast<int>(a);
        int ib = static_cast<int>(b);
        if (ia > ib) std::swap(ia, ib);
        return ia * 100 + ib;
    }

    // ==========================================
    //              DISPATCH TABLE
    // ==========================================

    class CollisionDispatch {
    private:
        std::unordered_map<int, CollisionFn> table;

    public:
        void registerCollision(ColliderType a, ColliderType b, CollisionFn fn) {
            table[makeKey(a, b)] = fn;
        }

        CollisionManifold dispatch(
            const Collider&        colA, const PhysicsState& stateA,
            const Collider&        colB, const PhysicsState& stateB,
            const PhysicsSettings& settings) const
        {
            auto it = table.find(makeKey(colA.type, colB.type));
            if (it == table.end()) return CollisionManifold{};
            return it->second(colA, stateA, colB, stateB, settings);
        }

        bool hasHandler(ColliderType a, ColliderType b) const {
            return table.count(makeKey(a, b)) > 0;
        }
    };

    // ==========================================
    //              COLLISION FNS
    // ==========================================

    namespace CollisionFns {

        inline CollisionManifold sphereVsSphere(
            const Collider& colA, const PhysicsState& stateA,
            const Collider& colB, const PhysicsState& stateB,
            const PhysicsSettings& settings)
        {
            return PureMath::checkSphereSphere(
                stateA, colA.size.x,
                stateB, colB.size.x,
                settings
            );
        }

    } // namespace CollisionFns

    // ==========================================
    //         DEFAULT DISPATCH TABLE
    // ==========================================

    inline CollisionDispatch buildDefaultDispatch() {
        CollisionDispatch d;
        d.registerCollision(ColliderType::SPHERE, ColliderType::SPHERE, CollisionFns::sphereVsSphere);
        return d;
    }

} // namespace inert

#endif // INERT_DISPATCH_HPP
