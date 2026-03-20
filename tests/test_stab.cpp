#include "raylib.h"
#include <vector>
#include <string>
#include "../src/obj.hpp"
#include "../src/collision.hpp"

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
    float restitutions[3] = { 0.1f, 0.5f, 0.9f };
    for (int i = 0; i < 3; i++) {
        inert::PhysicsBody* b = new inert::PhysicsBody();
        b->setMass(1.0f);
        b->setRestitution(restitutions[i]);
        b->setPosition({ -10.0f + (i * 4.0f), 20.0f, 0.0f });
        b->addCollider(inert::ColliderType::SPHERE, inert::vec3f{ 1.5f, 0.0f, 0.0f });
        world.addObject(b);
        renderBodies.push_back(b);
    }

    // 2. BÖLGE: DİZİLİM (STACKING) VE ÇÖZÜCÜ (SOLVER) STABİLİTE TESTİ
    for (int i = 0; i < 5; i++) {
        inert::PhysicsBody* b = new inert::PhysicsBody();
        b->setMass(2.0f);
        b->setRestitution(0.0f);
        b->setPosition({ 8.0f, 1.5f + (i * 3.01f), 0.0f });
        b->addCollider(inert::ColliderType::SPHERE, inert::vec3f{ 1.5f, 0.0f, 0.0f });
        world.addObject(b);
        renderBodies.push_back(b);
    }

    // 3. BÖLGE: KULLANICI KONTROLLÜ OBJE
    inert::PhysicsBody* player = new inert::PhysicsBody();
    player->setMass(5.0f);
    player->setRestitution(0.4f);
    player->setPosition({ 0.0f, 2.0f, 20.0f });
    player->addCollider(inert::ColliderType::SPHERE, inert::vec3f{ 2.0f, 0.0f, 0.0f });
    world.addObject(player);
    renderBodies.push_back(player);

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        inert::vec3f force;
        float movePower = 150.0f;

        if (IsKeyDown(KEY_W)) force[2] -= movePower;
        if (IsKeyDown(KEY_S)) force[2] += movePower;
        if (IsKeyDown(KEY_A)) force[0] -= movePower;
        if (IsKeyDown(KEY_D)) force[0] += movePower;
        if (IsKeyPressed(KEY_SPACE)) player->applyImpulseAtPoint({ 0.0f, 40.0f, 0.0f }, player->getPosition());
        if (IsKeyPressed(KEY_E))     player->addTorque({ 0.0f, 1000.0f, 0.0f });

        player->addForce(force);

        world.step(dt);

        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);

                DrawPlane({ 0.0f, 0.0f, 0.0f }, { 50.0f, 50.0f }, LIGHTGRAY);
                DrawGrid(50, 1.0f);

                for (size_t i = 0; i < renderBodies.size(); i++) {
                    Vector3 pos    = toRaylib(renderBodies[i]->getPosition());
                    float   radius = renderBodies[i]->getColliders()[0].size[0];

                    Color color = DARKBLUE;
                    if (i < 3) color = MAROON;
                    else if (renderBodies[i] == player) color = DARKGREEN;

                    DrawSphere(pos, radius, color);
                    DrawSphereWires(pos, radius, 16, 16, BLACK);
                }

            EndMode3D();

            DrawFPS(10, 10);
            DrawText("TEST MAIN: KATI CISIM MOTORU DOGRULAMASI", 10, 40, 20, DARKGRAY);
            DrawText("Sol Taraf : Enerji Korunumu (Sekme) Testi", 10, 70, 20, MAROON);
            DrawText("Sag Taraf : Kule Dizilimi (Solver) Testi", 10, 95, 20, DARKBLUE);
            DrawText("WASD      : Yesil Topu Hareket Ettir", 10, 120, 20, DARKGREEN);
            DrawText("SPACE     : Yesil Topu Ziplat", 10, 145, 20, DARKGREEN);

        EndDrawing();
    }

    for (auto b : renderBodies) delete b;
    CloseWindow();
    return 0;
}
