#ifndef COLLISION_HPP
#define COLLISION_HPP

#include <vector>
#include <unordered_map>
#include "raylib.h"
#include "raymath.h"
#include "obj.hpp"
#include "math.hpp"

namespace inert {

    class SpatialHash {
    private:
        float cellSize;
        std::unordered_map<int, std::vector<PhysicsBody*>> grid;

        inline int getHash(Vector3 pos) {
            int cx = (int)floor(pos.x / cellSize);
            int cy = (int)floor(pos.y / cellSize);
            int cz = (int)floor(pos.z / cellSize);
            return (cx * 73856093) ^ (cy * 19349663) ^ (cz * 83492791);
        }

    public:
        SpatialHash(float size = 3.0f) : cellSize(size) {}
        
        void setCellSize(float size) { cellSize = size; }

        void update(const std::vector<PhysicsBody*>& bodies) {
            grid.clear(); 
            for (auto body : bodies) {
                grid[getHash(body->getPosition())].push_back(body);
            }
        }

        std::vector<PhysicsBody*> getNeighbors(PhysicsBody* body) {
            std::vector<PhysicsBody*> neighbors;
            Vector3 pos = body->getPosition();
            
            int cx = (int)floor(pos.x / cellSize);
            int cy = (int)floor(pos.y / cellSize);
            int cz = (int)floor(pos.z / cellSize);

            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dz = -1; dz <= 1; dz++) {
                        int h = ((cx + dx) * 73856093) ^ ((cy + dy) * 19349663) ^ ((cz + dz) * 83492791);
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
    };


    class PhysicsWorld {
    private:
        std::vector<PhysicsBody*> bodies;
        SpatialHash spatialHash;
        
        bool hasGroundCollision = false;
        float groundLevel = 0.0f;
        
        PhysicsState groundState;

        void integrateForces(float dt) {
            for (auto body : bodies) {
                if (body->getBodyType() == BodyType::DYNAMIC) {
                    body->addForce({ 0.0f, body->getMass() * settings.gravityY, 0.0f });
                }
                body->updateBody(dt); 
            }
        }

        void handleGroundCollisions() {
            if (!hasGroundCollision) return;

            for (auto body : bodies) {
                for (const auto& collider : body->getColliders()) {
                    if (collider.type == ColliderType::SPHERE) {
                        float radius = collider.size.x;
                        float lowestPoint = body->getPosition().y - radius;
                        
                        if (lowestPoint < groundLevel) {
                            CollisionManifold m;
                            m.isColliding = true;
                            m.normal = { 0.0f, -1.0f, 0.0f }; 
                            m.depth = groundLevel - lowestPoint;
                            m.contactPoint = { body->getPosition().x, groundLevel, body->getPosition().z };

                            PositionalCorrectionResult posRes = 
                                PureMath::calculatePositionalCorrection(body->getState(), groundState, m, settings);
                            if (posRes.shouldCorrect) body->translate(posRes.translationA);

                            NormalImpulseResult normRes = 
                                PureMath::calculateNormalImpulse(body->getState(), groundState, body->getRestitution(), 1.0f, m);
                            
                            if (normRes.shouldApply) {
                                body->applyImpulseAtPoint(Vector3Scale(normRes.impulse, -1.0f), m.contactPoint);
                            }

                            Vector3 tangentImpulse = 
                                PureMath::calculateTangentImpulse(body->getState(), groundState, m, normRes.magnitude, settings);
                            body->applyImpulseAtPoint(Vector3Scale(tangentImpulse, -1.0f), m.contactPoint);
                        }
                    }
                    else if (collider.type == ColliderType::POINT_CLOUD) {
                        for (const auto& localPt : collider.localPoints) {
                            Vector3 worldPoint = 
                                Vector3Add(body->getPosition(), Vector3Transform(localPt, QuaternionToMatrix(body->getState().orientation)));
                            
                            if (worldPoint.y < groundLevel) {
                                CollisionManifold m;
                                m.isColliding = true;
                                m.normal = { 0.0f, -1.0f, 0.0f };
                                m.depth = groundLevel - worldPoint.y;
                                m.contactPoint = worldPoint;

                                PositionalCorrectionResult posRes = 
                                    PureMath::calculatePositionalCorrection(body->getState(), groundState, m, settings);
                                if (posRes.shouldCorrect) body->translate(posRes.translationA);

                                NormalImpulseResult normRes = 
                                    PureMath::calculateNormalImpulse(body->getState(), groundState, body->getRestitution(), 1.0f, m);
                                if (normRes.shouldApply) {
                                    body->applyImpulseAtPoint(Vector3Scale(normRes.impulse, -1.0f), m.contactPoint);
                                }

                                Vector3 tangentImpulse = 
                                    PureMath::calculateTangentImpulse(body->getState(), groundState, m, normRes.magnitude, settings);
                                body->applyImpulseAtPoint(Vector3Scale(tangentImpulse, -1.0f), m.contactPoint);
                            }
                        }
                    }
                }
            }
        }

        void handleBodyCollisions() {
            for (auto bodyA : bodies) {
                auto neighbors = spatialHash.getNeighbors(bodyA);
                for (auto bodyB : neighbors) {
                    if (bodyA >= bodyB) continue; 

                    for (const auto& colA : bodyA->getColliders()) {
                        for (const auto& colB : bodyB->getColliders()) {
                            if (colA.type == ColliderType::SPHERE && colB.type == ColliderType::SPHERE) {
                                CollisionManifold m = PureMath::checkSphereSphere(bodyA->getState(), colA.size.x, bodyB->getState(), colB.size.x, settings);
                                
                                if (m.isColliding) {
                                    PositionalCorrectionResult posRes = PureMath::calculatePositionalCorrection(bodyA->getState(), bodyB->getState(), m, settings);
                                    if (posRes.shouldCorrect) {
                                        bodyA->translate(posRes.translationA);
                                        bodyB->translate(posRes.translationB);
                                    }

                                    NormalImpulseResult normRes = PureMath::calculateNormalImpulse(bodyA->getState(), bodyB->getState(), bodyA->getRestitution(), bodyB->getRestitution(), m);
                                    if (normRes.shouldApply) {
                                        bodyA->applyImpulseAtPoint(Vector3Scale(normRes.impulse, -1.0f), m.contactPoint);
                                        bodyB->applyImpulseAtPoint(normRes.impulse, m.contactPoint);
                                    }

                                    Vector3 tangentImpulse = PureMath::calculateTangentImpulse(bodyA->getState(), bodyB->getState(), m, normRes.magnitude, settings);
                                    bodyA->applyImpulseAtPoint(Vector3Scale(tangentImpulse, -1.0f), m.contactPoint);
                                    bodyB->applyImpulseAtPoint(tangentImpulse, m.contactPoint);
                                }
                            }
                        }
                    }
                }
            }
        }

    public:
        PhysicsSettings settings; 

        PhysicsWorld() {
            groundState.inverseMass = 0.0f;
            groundState.inverseInertia = { 0.0f, 0.0f, 0.0f };
            groundState.velocity = { 0.0f, 0.0f, 0.0f };
            groundState.rotatVel = { 0.0f, 0.0f, 0.0f };
        }

        void addObject(PhysicsBody* body) { bodies.push_back(body); }
        void addGround(float y_level) { hasGroundCollision = true; groundLevel = y_level; }

        void step(float dt) {
            integrateForces(dt);

            spatialHash.setCellSize(settings.spatialCellSize);
            spatialHash.update(bodies);

            for (int k = 0; k < settings.solverIterations; k++) {
                handleGroundCollisions();
                handleBodyCollisions();
            }
        }
    };
}

#endif
