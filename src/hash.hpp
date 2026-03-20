#ifndef HASH_HPP
#define HASH_HPP

#include <vector>
#include <unordered_map>
#include "obj.hpp"

namespace inert {
    
    class SpatialHash {
    private:
        static constexpr int P1 = 73856093;
        static constexpr int P2 = 19349663;
        static constexpr int P3 = 83492791;
        float cellSize = 5.0f;
        std::unordered_map<int, std::vector<PhysicsBody*>> grid;

        inline int getHash(const vec3f& pos) {
            int cx = (int)floor(pos[0] / cellSize);
            int cy = (int)floor(pos[1] / cellSize);
            int cz = (int)floor(pos[2] / cellSize);
            return (cx * P1) ^ (cy * P2) ^ (cz * P3);
        }

    public:
        SpatialHash(float size = 5.0f) : cellSize(size) {}
        
        void setCellSize(float size) { cellSize = size; }

        void inline update(const std::vector<PhysicsBody*>& bodies) {
            grid.clear(); 
            for (auto body : bodies)
                grid[getHash(body->getPosition())].push_back(body);
        }

        std::vector<PhysicsBody*> getNeighbors(PhysicsBody* body);
    };
}

#endif // !HASH_HPP
