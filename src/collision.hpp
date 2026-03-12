#ifndef COLLISION_HPP
#define COLLISION_HPP

#include <vector>
#include <algorithm>
#include <unordered_map>
#include "raylib.h"
#include "raymath.h"
#include "obj.hpp"
#include "math.hpp"

namespace inert {

    // --- PHYSICS WORLD ---
    class PhysicsWorld {
    private:
        std::vector<PhysicsBody*> bodies;
        bool hasGroundCollision = false;
        float groundLevel = 0.0f;
        
    public:
        PhysicsSettings settings;

        void addObject(PhysicsBody* body) { bodies.push_back(body); }
        void addGround(float y_level) { hasGroundCollision = true; groundLevel = y_level; }

        void step(float dt) {

            // gravity update
            for (auto body : bodies) {
                if (body->getBodyType() == BodyType::DYNAMIC)
                    body->addForce({ 0.0f, body->getMass() * settings.gravityY, 0.0f });
                body->updateBody(dt);
            }

            std::unordered_map<int, std::vector<PhysicsBody*>> grid;
            
            for (auto body : bodies) {
                int cx = (int)floor(body->getPosition().x / settings.spatialCellSize);
                int cy = (int)floor(body->getPosition().y / settings.spatialCellSize);
                int cz = (int)floor(body->getPosition().z / settings.spatialCellSize);
                int hash = (cx * 73856093) ^ (cy * 19349663) ^ (cz * 83492791);
                grid[hash].push_back(body);
            }

            // iterative solver
            for (int k = 0; k < settings.solverIterations; k++) {
                
                // ground collisions
                for (auto body : bodies) {
                    if (!hasGroundCollision) continue;
                    
                    for (const auto& collider : body->getColliders()) {
                        if (collider.type == ColliderType::SPHERE) {
                            float radius = collider.size.x;
                            float lowestPoint = body->getPosition().y - radius;
                            
                            if (lowestPoint < groundLevel) {
                                float penetration = groundLevel - lowestPoint;
                                body->translate({ 0.0f, penetration, 0.0f });
                                
                                Vector3 contactPoint = { body->getPosition().x, groundLevel, body->getPosition().z };
                                Vector3 normal = { 0.0f, 1.0f, 0.0f };
                                Vector3 r = Vector3Subtract(contactPoint, body->getPosition()); 

                                Vector3 velAtContact = body->getVelocityAtPoint(contactPoint);
                                float velAlongNormal = Vector3DotProduct(velAtContact, normal);
                                
                                if (velAlongNormal <= 0) {
                                    float angularEffect = PureMath::calculateAngularEffect(body->getState(), r, normal);
                                    float e = body->getRestitution();
                                    if (abs(velAlongNormal) < settings.bounceThreshold) e = 0.0f; 

                                    float j = -(1.0f + e) * velAlongNormal;
                                    j /= (body->getState().inverseMass + angularEffect);
                                    
                                    Vector3 impulse = Vector3Scale(normal, j);
                                    body->applyImpulseAtPoint(impulse, contactPoint);
                                    
                                    // Zemin Sürtünmesi
                                    Vector3 newVelAtContact = body->getVelocityAtPoint(contactPoint);
                                    float newVelAlongNormal = Vector3DotProduct(newVelAtContact, normal);
                                    Vector3 normalVelocity = Vector3Scale(normal, newVelAlongNormal);
                                    Vector3 tangentVelocity = Vector3Subtract(newVelAtContact, normalVelocity);
                                    
                                    float tangentSpeed = Vector3Length(tangentVelocity);
                                    if (tangentSpeed >= settings.velocityEpsilon) {
                                        Vector3 t = Vector3Scale(tangentVelocity, 1.0f / tangentSpeed);
                                        float angularEffectT = PureMath::calculateAngularEffect(body->getState(), r, t);
                                        float jt = -tangentSpeed / (body->getState().inverseMass + angularEffectT);
                                        float maxFriction = j * settings.baseFrictionMu;
                                        jt = Clamp(jt, -maxFriction, maxFriction); 
                                        
                                        body->applyImpulseAtPoint(Vector3Scale(t, jt), contactPoint);
                                    }
                                }
                            }
                        }
                    }
                }

                // object to object collisions
                for (auto bodyA : bodies) {
                    int cx = (int)floor(bodyA->getPosition().x / settings.spatialCellSize);
                    int cy = (int)floor(bodyA->getPosition().y / settings.spatialCellSize);
                    int cz = (int)floor(bodyA->getPosition().z / settings.spatialCellSize);

                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            for (int dz = -1; dz <= 1; dz++) {
                                
                                int neighborHash = ((cx + dx) * 73856093) ^ ((cy + dy) * 19349663) ^ ((cz + dz) * 83492791);
                                auto it = grid.find(neighborHash);
                                
                                if (it != grid.end()) {
                                    for (auto bodyB : it->second) {
                                        if (bodyA >= bodyB) continue; 
                                        
                                        for (const auto& colA : bodyA->getColliders()) {
                                            for (const auto& colB : bodyB->getColliders()) {
                                                if (colA.type == ColliderType::SPHERE && colB.type == ColliderType::SPHERE) {
                                                    
                                                    CollisionManifold m = PureMath::checkSphereSphere(bodyA->getState(),
                                                            colA.size.x, bodyB->getState(), colB.size.x, settings);
                                                    
                                                    if (m.isColliding) {
                                                        resolveCollision(bodyA, bodyB, m);
                                                        applyPositionalCorrection(bodyA, bodyB, m);
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // ------ IMPERATIVE SHELL ------

        void applyPositionalCorrection(PhysicsBody* a, PhysicsBody* b, const CollisionManifold& m) {
            PositionalCorrectionResult res = 
                PureMath::calculatePositionalCorrection(a->getState(), b->getState(), m, settings);
            
            if (res.shouldCorrect) {
                a->translate(res.translationA);
                b->translate(res.translationB);
            }
        }

        void resolveCollision(PhysicsBody* a, PhysicsBody* b, const CollisionManifold& m) {
            NormalImpulseResult normRes = 
                PureMath::calculateNormalImpulse(a->getState(), b->getState(), a->getRestitution(), b->getRestitution(), m);
            
            if (normRes.shouldApply) {
                a->applyImpulseAtPoint(Vector3Scale(normRes.impulse, -1.0f), m.contactPoint);
                b->applyImpulseAtPoint(normRes.impulse, m.contactPoint);
                
                Vector3 tangentImpulse = PureMath::calculateTangentImpulse(a->getState(), b->getState(), m, normRes.magnitude, settings);
                
                a->applyImpulseAtPoint(Vector3Scale(tangentImpulse, -1.0f), m.contactPoint);
                b->applyImpulseAtPoint(tangentImpulse, m.contactPoint);
            }
        }
    };
}
#endif
