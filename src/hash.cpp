#include "hash.hpp"

namespace inert {
    
    std::vector<PhysicsBody*> SpatialHash::getNeighbors(PhysicsBody* body) {
        std::vector<PhysicsBody*> neighbors;
        vec3f pos = body->getPosition();

        int cx = (int)floor(pos[0] / cellSize);
        int cy = (int)floor(pos[1] / cellSize);
        int cz = (int)floor(pos[2] / cellSize);

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
