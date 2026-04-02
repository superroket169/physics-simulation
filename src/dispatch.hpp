#ifndef INERT_DISPATCH_HPP
#define INERT_DISPATCH_HPP

#include <functional>
#include <unordered_map>
#include "math.hpp"
#include "triangle_mesh.hpp"

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
                stateA, colA.size[0],
                stateB, colB.size[0],
                settings
            );
        }

        inline CollisionManifold triangleVsTriangle(
            const Collider& colA, const PhysicsState& stateA,
            const Collider& colB, const PhysicsState& stateB,
            const PhysicsSettings&)
        {
            std::vector<Triangle> worldA, worldB;
            worldA.reserve(colA.triangles.size());
            worldB.reserve(colB.triangles.size());
            for (const Triangle& t : colA.triangles) {
                Triangle wt;
                wt.a = stateA.position + rotate(t.a, stateA.orientation);
                wt.b = stateA.position + rotate(t.b, stateA.orientation);
                wt.c = stateA.position + rotate(t.c, stateA.orientation);
                wt.normal = rotate(t.normal, stateA.orientation);
                worldA.push_back(wt);
            }
            for (const Triangle& t : colB.triangles) {
                Triangle wt;
                wt.a = stateB.position + rotate(t.a, stateB.orientation);
                wt.b = stateB.position + rotate(t.b, stateB.orientation);
                wt.c = stateB.position + rotate(t.c, stateB.orientation);
                wt.normal = rotate(t.normal, stateB.orientation);
                worldB.push_back(wt);
            }
            return checkMeshMesh(worldA, worldB);
        }

    } // namespace CollisionFns

    // ==========================================
    //         DEFAULT DISPATCH TABLE
    // ==========================================

    inline CollisionDispatch buildDefaultDispatch() {
        CollisionDispatch d;
        d.registerCollision(ColliderType::SPHERE,        ColliderType::SPHERE,        CollisionFns::sphereVsSphere);
        d.registerCollision(ColliderType::TRIANGLE_MESH, ColliderType::TRIANGLE_MESH, CollisionFns::triangleVsTriangle);
        return d;
    }

} // namespace inert

#endif // INERT_DISPATCH_HPP
