#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <string>
#include "obj.hpp"
#include "collision.hpp"

int main() {
    InitWindow(1024, 768, "Inertia Physics - Core Engine Functionality Test");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = { 0.0f, 15.0f, 40.0f };
    camera.target = { 0.0f, 5.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    inert::PhysicsWorld world;
    world.addGround(0.0f);

    std::vector<inert::PhysicsBody*> renderBodies;

    // 1. BÖLGE: SEKTİRME (RESTITUTION) TESTİ
    // Farklı sekme katsayılarına sahip 3 top yan yana düşecek
    float restitutions[3] = { 0.1f, 0.5f, 0.9f };
    for (int i = 0; i < 3; i++) {
        inert::PhysicsBody* b = new inert::PhysicsBody();
        b->setMass(1.0f);
        b->setRestitution(restitutions[i]);
        b->setPosition({ -10.0f + (i * 4.0f), 20.0f, 0.0f });
        b->addCollider(inert::ColliderType::SPHERE, { 1.5f, 0.0f, 0.0f });
        world.addObject(b);
        renderBodies.push_back(b);
    }

    // 2. BÖLGE: DİZİLİM (STACKING) VE ÇÖZÜCÜ (SOLVER) STABİLİTE TESTİ
    // Üst üste binen objeler, motorun iteratif çözüm yeteneğini sınar
    for (int i = 0; i < 5; i++) {
        inert::PhysicsBody* b = new inert::PhysicsBody();
        b->setMass(2.0f);
        b->setRestitution(0.0f); // Kule diziliminde stabilite için sekmeme
        b->setPosition({ 8.0f, 1.5f + (i * 3.01f), 0.0f }); // Tam üst üste (0.01f boşluk)
        b->addCollider(inert::ColliderType::SPHERE, { 1.5f, 0.0f, 0.0f });
        world.addObject(b);
        renderBodies.push_back(b);
    }

    // 3. BÖLGE: KULLANICI KONTROLLÜ KİNETİK OBJE
    inert::PhysicsBody* player = new inert::PhysicsBody();
    player->setMass(5.0f); // Yığını devirebilmesi için ağır
    player->setRestitution(0.4f);
    player->setPosition({ 0.0f, 2.0f, 20.0f });
    player->addCollider(inert::ColliderType::SPHERE, { 2.0f, 0.0f, 0.0f });
    world.addObject(player);
    renderBodies.push_back(player);

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // --- KONTROLLER ---
        Vector3 force = { 0.0f, 0.0f, 0.0f };
        float movePower = 150.0f;
        
        if (IsKeyDown(KEY_W)) force.z -= movePower;
        if (IsKeyDown(KEY_S)) force.z += movePower;
        if (IsKeyDown(KEY_A)) force.x -= movePower;
        if (IsKeyDown(KEY_D)) force.x += movePower;
        if (IsKeyPressed(KEY_SPACE)) player->applyImpulseAtPoint({0.0f, 40.0f, 0.0f}, player->getPosition());

        player->addForce(force);

        // --- FİZİK ADIMI ---
        world.step(dt);

        // --- ÇİZİM ---
        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);
                DrawPlane({ 0.0f, 0.0f, 0.0f }, { 50.0f, 50.0f }, LIGHTGRAY);
                DrawGrid(50, 1.0f);

                for (size_t i = 0; i < renderBodies.size(); i++) {
                    Vector3 pos = renderBodies[i]->getPosition();
                    float radius = renderBodies[i]->getColliders()[0].size.x;
                    
                    Color color = DARKBLUE;
                    if (i < 3) color = MAROON; // Sekme testindekiler kırmızı
                    else if (renderBodies[i] == player) color = DARKGREEN; // Oyuncu yeşil

                    DrawSphere(pos, radius, color);
                    DrawSphereWires(pos, radius, 16, 16, BLACK);
                }
            EndMode3D();

            DrawFPS(10, 10);
            DrawText("TEST MAIN: KATI CISIM MOTORU DOGRULAMASI", 10, 40, 20, DARKGRAY);
            DrawText("Sol Taraf : Enerji Korunumu (Sekme) Testi", 10, 70, 20, MAROON);
            DrawText("Sag Taraf : Kule Dizilimi (Solver) Testi", 10, 95, 20, DARKBLUE);
            DrawText("WASD      : Yesil Topu Hareket Ettir (Yigina Carp)", 10, 120, 20, DARKGREEN);
            DrawText("SPACE     : Yesil Topu Ziplat", 10, 145, 20, DARKGREEN);

        EndDrawing();
    }

    for (auto b : renderBodies) delete b;
    CloseWindow();
    return 0;
}
