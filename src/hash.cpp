#include "hash.hpp"

namespace inert {
    
    std::vector<PhysicsBody*> SpatialHash::getNeighbors(PhysicsBody* body) {
        std::vector<PhysicsBody*> neighbors;
        Vector3 pos = body->getPosition();

        int cx = (int)floor(pos.x / cellSize);
        int cy = (int)floor(pos.y / cellSize);
        int cz = (int)floor(pos.z / cellSize);

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    int h = ((cx + dx) * P1) ^ ((cy + dy) * P2) ^ ((cz + dz) * P3);
                    auto it = grid.find(h);
                    if (it != grid.end()) {
                        for (auto b : it->second) {
                            if (b != body) neighbors.push_back(b);
                        }
                    }
                }
            }
        }
        return neighbors;
    }

}

