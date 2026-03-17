#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <string>
#include "../src/obj.hpp"
#include "../src/collision.hpp"

// Topların renklerini ve çizim yarıçapını tutan yapı
struct SphereData {
    inert::PhysicsBody* body;
    Color color;
    float visualRadius; 
};

// Yeni sıvı partikülü oluşturma fonksiyonu
void spawnFluidParticle(inert::PhysicsWorld& world, std::vector<SphereData>& spheres, Vector3 position, float physRadius, float visRadius) {
    inert::PhysicsBody* newBody = new inert::PhysicsBody();
    
    // Toplar büyüdüğü için kütleleri de biraz artırıldı
    newBody->setMass(0.5f); 
    newBody->setRestitution(0.02f); // Çarpışma enerjisini emerek sıvı hissiyatı verir
    
    newBody->setPosition(position);
    newBody->addCollider(inert::ColliderType::SPHERE, { physRadius, 0.0f, 0.0f });
    
    world.addObject(newBody);
    
    // Mavi tonlarında su renkleri
    Color fluidColor = { 
        (unsigned char)GetRandomValue(0, 40), 
        (unsigned char)GetRandomValue(100, 180), 
        (unsigned char)GetRandomValue(200, 255), 
        255 
    };
    
    spheres.push_back({ newBody, fluidColor, visRadius });
}

int main() {
    InitWindow(1024, 768, "Inertia Physics - Yeni Mimari Testi (500 Obje)");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = { 0.0f, 25.0f, 40.0f };
    camera.target = { 0.0f, 5.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    inert::PhysicsWorld world;
    world.addGround(0.0f);

    std::vector<SphereData> spheres;
    
    // --- BÜYÜTÜLMÜŞ PARTİKÜLLER VE RADIUS TRICK ---
    // Not: collision.hpp içindeki 'cellSize' değeri 0.8f olduğu için, 
    // physRadius'un 0.4f civarında kalması Spatial Hashing'in sağlığı için idealdir.
    float physRadius = 0.40f; 
    float visRadius = 0.30f;  
    
    float containerSize = 15.0f; 
    float containerHeight = 25.0f;

    // --- 500 ADET SIVI PARTİKÜLÜ OLUŞTURMA ---
    int targetBalls = 500;
    int spawnedBalls = 0;
    float step = physRadius * 2.2f; 

    for (float y = 1.0f; y < 50.0f && spawnedBalls < targetBalls; y += step) {
        for (float x = -containerSize/2.0f + step; x < containerSize/2.0f - step && spawnedBalls < targetBalls; x += step) {
            for (float z = -containerSize/2.0f + step; z < containerSize/2.0f - step && spawnedBalls < targetBalls; z += step) {
                Vector3 jitterPos = {
                    x + GetRandomValue(-10, 10) * 0.001f,
                    y + GetRandomValue(-10, 10) * 0.001f,
                    z + GetRandomValue(-10, 10) * 0.001f
                };
                spawnFluidParticle(world, spheres, jitterPos, physRadius, visRadius);
                spawnedBalls++;
            }
        }
    }

    // --- BÜYÜK TEST TOPU (YÜZEN/BATAN OBJE) ---
    float bigBallRadius = 2.5f;
    inert::PhysicsBody* bigBall = new inert::PhysicsBody();
    bigBall->setMass(5.0f); // 500 partikül ile etkileşime girecek başlangıç kütlesi
    bigBall->setRestitution(0.2f);
    bigBall->setPosition({ 0.0f, 20.0f, 0.0f }); 
    bigBall->addCollider(inert::ColliderType::SPHERE, { bigBallRadius, 0.0f, 0.0f });
    world.addObject(bigBall);

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // --- KONTROLLER ---
        if (IsKeyPressed(KEY_UP)) {
            bigBall->setMass(bigBall->getMass() + 2.0f);
        }
        if (IsKeyPressed(KEY_DOWN)) {
            float newMass = bigBall->getMass() - 2.0f;
            if (newMass < 1.0f) newMass = 1.0f; 
            bigBall->setMass(newMass);
        }
        
        // Büyük topu tekrar havadan bırakmak ve hızını sıfırlamak için
        if (IsKeyPressed(KEY_R)) {
            bigBall->setPosition({ 0.0f, 25.0f, 0.0f });
            bigBall->setVelocity({ 0.0f, 0.0f, 0.0f });
        }

        // --- SIVI DİNAMİĞİ (TİTREŞİM VE YÜZEY GERİLİMİ) ---
        for (auto& s : spheres) {
            Vector3 pos = s.body->getPosition();
            
            // 1. Jitter (Kayganlaştırma Titreşimi)
            float jitterMag = 0.5f; // Toplar büyüdüğü için itki kuvveti de arttı
            Vector3 randomForce = {
                (GetRandomValue(-100, 100) / 100.0f) * jitterMag,
                (GetRandomValue(-100, 100) / 100.0f) * jitterMag,
                (GetRandomValue(-100, 100) / 100.0f) * jitterMag
            };

            // 2. Cohesion (Merkeze Çekme / Duvarlardan Uzaklaştırma)
            // Topların gaz gibi çeperlere yapışmasını engeller
            Vector3 cohesionForce = { -pos.x * 0.5f, 0.0f, -pos.z * 0.5f };
            
            s.body->addForce(Vector3Add(randomForce, cohesionForce));
        }

        // --- FİZİK MOTORU ADIMI (Yeni Mimarinin Test Edildiği Yer) ---
        world.step(dt);

        // --- KABIN DUVARLARI (Manuel Sınır Kontrolü) ---
        auto applyBoundary = [&](inert::PhysicsBody* b, float radius) {
            Vector3 pos = b->getPosition();
            Vector3 vel = b->getVelocity();
            bool hitWall = false;
            float halfBox = containerSize / 2.0f;

            // X ve Z ekseni sınırları
            if (pos.x - radius < -halfBox) { pos.x = -halfBox + radius; vel.x *= -0.3f; hitWall = true; }
            if (pos.x + radius > halfBox)  { pos.x = halfBox - radius;  vel.x *= -0.3f; hitWall = true; }
            if (pos.z - radius < -halfBox) { pos.z = -halfBox + radius; vel.z *= -0.3f; hitWall = true; }
            if (pos.z + radius > halfBox)  { pos.z = halfBox - radius;  vel.z *= -0.3f; hitWall = true; }
            
            // Tavan
            if (pos.y > containerHeight)   { pos.y = containerHeight - radius; vel.y *= -0.3f; hitWall = true; }

            if (hitWall) {
                b->setPosition(pos);
                b->setVelocity(vel);
            }
        };

        // Sınırları tüm objelere uygula
        for (auto& s : spheres) applyBoundary(s.body, physRadius);
        applyBoundary(bigBall, bigBallRadius);

        // --- ÇİZİM ---
        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);
                
                // Zemin
                DrawPlane({ 0.0f, 0.0f, 0.0f }, { 50.0f, 50.0f }, LIGHTGRAY);
                DrawGrid(50, 1.0f);
                
                // Cam Kap (Tel Kafes)
                Vector3 containerPos = { 0.0f, containerHeight / 2.0f, 0.0f };
                DrawCubeWires(containerPos, containerSize, containerHeight, containerSize, DARKGRAY);

                // Sıvı Partiküllerini Çiz
                for (int i = 0; i < spheres.size(); i++) {
                    DrawSphere(spheres[i].body->getPosition(), spheres[i].visualRadius, spheres[i].color);
                }

                // Büyük Topu Çiz
                DrawSphere(bigBall->getPosition(), bigBallRadius, MAROON);
                DrawSphereWires(bigBall->getPosition(), bigBallRadius, 16, 16, BLACK);

            EndMode3D();

            // --- ARAYÜZ (UI) ---
            DrawFPS(10, 10);
            DrawText("TEST 5: YENI MIMARI STABILITE TESTI", 10, 35, 20, DARKGRAY);
            DrawText("R TUSU    : Buyuk Topu Havadan Birak", 10, 65, 20, DARKBLUE);
            DrawText("YUKARI OK : Buyuk Topun Kutlesini Artir (Batar)", 10, 90, 20, BLACK);
            DrawText("ASAGI OK  : Buyuk Topun Kutlesini Azalt (Yuzer)", 10, 115, 20, BLACK);
            
            DrawText(TextFormat("Sivi Partikul Sayisi: %d", spheres.size()), 10, 155, 20, DARKGRAY);
            DrawText(TextFormat("Buyuk Topun Kutlesi: %.1f kg", bigBall->getMass()), 10, 180, 20, RED);

        EndDrawing();
    }

    // Bellek Temizliği
    for (auto& s : spheres) delete s.body;
    delete bigBall;

    CloseWindow();
    return 0;
}
