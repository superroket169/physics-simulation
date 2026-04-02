#ifndef TRIANGLE_MESH_HPP
#define TRIANGLE_MESH_HPP

#include <vector>
#include "vector.hpp"
#include "types.hpp"

namespace inert {

    struct Triangle {
        vec3f a, b, c;
        vec3f normal;

        // Builds triangle and computes normal from vertex order
        // (CCW = outward)
        static Triangle make(vec3f a, vec3f b, vec3f c) {
            Triangle t;
            t.a      = a;
            t.b      = b;
            t.c      = c;
            t.normal = getCrossProduct(b - a, c - a).getNormalized();
            return t;
        }

        // TODO: makeOutward(vec3f origin) — flips normal if pointing toward origin
        //       needed for closed meshes to ensure consistent outward normals

        // TODO: makeWithNormal(a, b, c, stlNormal) — STL parser will call this
        //       compare provided normal with computed one
        //       if deviation > threshold, log warning (red if large deviation)
        //       always use computed normal, provided is just for validation

        vec3f getCentroid() const {
            return (a + b + c) * (1.0f / 3.0f);
        }
    };

    // ==========================================
    //         SAT HELPERS
    // ==========================================

    inline bool intervalsOverlap(float minA, float maxA, float minB, float maxB) {
        return maxA >= minB && maxB >= minA;
    }

    inline float overlapAmount(float minA, float maxA, float minB, float maxB) {
        return (maxA < maxB ? maxA : maxB) - (minA > minB ? minA : minB);
    }

    inline void projectMesh(const std::vector<Triangle>& tris, const vec3f& axis,
                            float& outMin, float& outMax) {
        outMin =  1e30f;
        outMax = -1e30f;
        for (const auto& t : tris) {
            for (const vec3f* pt : { &t.a, &t.b, &t.c }) {
                float d = pt->getDotProduct(axis);
                if (d < outMin) outMin = d;
                if (d > outMax) outMax = d;
            }
        }
    }

    // ==========================================
    //         SAT — MESH VS MESH
    // ==========================================

    inline CollisionManifold checkMeshMesh(const std::vector<Triangle>& meshA,
                                           const std::vector<Triangle>& meshB) {
        CollisionManifold result;

        // A -> B
        // midpoint-of-projection can flip on edge-cross axes
        // centroid diff is stable.
        vec3f centroidA{}, centroidB{};
        int cntA = 0, cntB = 0;
        
        for (const auto& t : meshA) { centroidA += t.a; centroidA += t.b; centroidA += t.c; cntA += 3; }
        for (const auto& t : meshB) { centroidB += t.a; centroidB += t.b; centroidB += t.c; cntB += 3; }
        
        if (cntA > 0) centroidA = centroidA * (1.0f / cntA);
        if (cntB > 0) centroidB = centroidB * (1.0f / cntB);
        
        vec3f centerDiff = centroidB - centroidA; // points A -> B

        float minDepth = 1e30f;
        vec3f bestAxis;

        auto testAxis = [&](vec3f axis) -> bool {
            if (axis.getLengthSqr() < 1e-8f) return true;
            
            axis = axis.getNormalized();
            
            float minA, maxA, minB, maxB;

            projectMesh(meshA, axis, minA, maxA);
            projectMesh(meshB, axis, minB, maxB);
            
            if (!intervalsOverlap(minA, maxA, minB, maxB)) return false; // separating axis found
            
            float depth = overlapAmount(minA, maxA, minB, maxB);
            
            if (depth < minDepth) {
                minDepth = depth;
                bestAxis = (centerDiff.getDotProduct(axis) >= 0.0f) ? axis : -axis;
            }
            return true;
        };

        // Face normals from both meshes
        for (const auto& t : meshA)
            if (!testAxis(t.normal)) return result;
        for (const auto& t : meshB)
            if (!testAxis(t.normal)) return result;

        std::vector<vec3f> edgeDirsA, edgeDirsB;
        for (const auto& t : meshA) {
            edgeDirsA.push_back(t.b - t.a);
            edgeDirsA.push_back(t.c - t.b);
            edgeDirsA.push_back(t.a - t.c);
        }
        for (const auto& t : meshB) {
            edgeDirsB.push_back(t.b - t.a);
            edgeDirsB.push_back(t.c - t.b);
            edgeDirsB.push_back(t.a - t.c);
        }
        for (const auto& eA : edgeDirsA)
            for (const auto& eB : edgeDirsB)
                if (!testAxis(getCrossProduct(eA, eB))) return result;

        result.isColliding = true;
        result.depth       = minDepth;

        // bestAxis points A -> B
        // normal = -bestAxis (B -> A)
        result.normal = -bestAxis;

        {
            float threshold = 0.05f;
            float deepestVal = -1e30f;
            for (const auto& t : meshA)
                for (const vec3f* pt : { &t.a, &t.b, &t.c })
                    { float d = pt->getDotProduct(bestAxis); if (d > deepestVal) deepestVal = d; }
        
            vec3f avgPoint{};
            int count = 0;
            for (const auto& t : meshA) {
                for (const vec3f* pt : { &t.a, &t.b, &t.c }) {
                    if (pt->getDotProduct(bestAxis) >= deepestVal - threshold) {
                        avgPoint += *pt;
                        count++;
                    }
                }
            }
            result.contactPoint = (count > 0) ? avgPoint * (1.0f / count) : avgPoint;
        }

        return result;
    }

    // ==========================================
    //         MESH BUILDERS
    // ==========================================

    inline std::vector<Triangle> makeBox(float hx, float hy, float hz) {
        vec3f v[8] = {
            { -hx, -hy, -hz }, {  hx, -hy, -hz },
            {  hx,  hy, -hz }, { -hx,  hy, -hz },
            { -hx, -hy,  hz }, {  hx, -hy,  hz },
            {  hx,  hy,  hz }, { -hx,  hy,  hz }
        };

        return {
            // -Z face
            Triangle::make(v[0], v[2], v[1]), Triangle::make(v[0], v[3], v[2]),
            // +Z face
            Triangle::make(v[4], v[5], v[6]), Triangle::make(v[4], v[6], v[7]),
            // -X face
            Triangle::make(v[0], v[4], v[7]), Triangle::make(v[0], v[7], v[3]),
            // +X face
            Triangle::make(v[1], v[2], v[6]), Triangle::make(v[1], v[6], v[5]),
            // -Y face
            Triangle::make(v[0], v[1], v[5]), Triangle::make(v[0], v[5], v[4]),
            // +Y face
            Triangle::make(v[3], v[6], v[2]), Triangle::make(v[3], v[7], v[6])
        };
    }

    inline std::vector<Triangle> makeBox(float size) {
        return makeBox(size, size, size);
    }

    // TODO: makeSphere(radius, segments) — approximate sphere with triangles
    // TODO: makeCylinder(radius, height, segments)
    // TODO: makeCone(radius, height, segments)

} // namespace inert

#endif // !TRIANGLE_MESH_HPP
