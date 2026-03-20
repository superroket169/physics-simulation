#include "raylib.h"
#include <vector>
#include <cmath>
#include "../src/obj.hpp"
#include "../src/collision.hpp"

int main() {
    InitWindow(1024, 768, "Inertia Physics - Topac (Point Cloud) ve Tork Testi");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = { 0.0f, 15.0f, 25.0f };
    camera.target = { 0.0f, 2.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    inert::PhysicsWorld world;
    world.addGround(0.0f);

    inert::PhysicsBody* topac = new inert::PhysicsBody();
    topac->setMass(5.0f);
    topac->setRestitution(0.6f);
    topac->setPosition({ 0.0f, 10.0f, 0.0f });

    // Küre eylemsizliği: I = 2/5 * m * r^2 = 0.4 * 5 * 2^2 = 8.0
    topac->setInertia({ 8.0f, 8.0f, 8.0f });

    // 1. FİZİKSEL ÇARPIŞMA İÇİN KÜRE COLLIDER
    float radius = 2.0f;
    topac->addCollider(inert::ColliderType::SPHERE, inert::vec3f{ radius, 0.0f, 0.0f });

    // 2. GÖRSELLEŞTİRME İÇİN POINT CLOUD COLLIDER
    std::vector<inert::vec3f> pcPoints;

    // Merkez Şiş (Y Ekseni)
    pcPoints.push_back({ 0.0f, radius + 1.0f, 0.0f });
    pcPoints.push_back({ 0.0f, -radius, 0.0f });

    // Ekvator Çemberi
    int segments = 8;
    for (int i = 0; i < segments; i++) {
        float angle = (float)i / segments * PI * 2.0f;
        pcPoints.push_back({ cosf(angle) * radius, 0.0f, sinf(angle) * radius });
    }

    topac->addCollider(inert::ColliderType::POINT_CLOUD, pcPoints);
    world.addObject(topac);

    float tf = 300.0f;
    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        if (IsKeyDown(KEY_E))     topac->addTorque({ 0.0f,  tf, 0.0f });
        if (IsKeyDown(KEY_Q))     topac->addTorque({ 0.0f, -tf, 0.0f });
        if (IsKeyDown(KEY_UP))    topac->addTorque({  -tf, 0.0f, 0.0f });
        if (IsKeyDown(KEY_DOWN))  topac->addTorque({   tf, 0.0f, 0.0f });
        if (IsKeyDown(KEY_LEFT))  topac->addTorque({ 0.0f, 0.0f,  tf });
        if (IsKeyDown(KEY_RIGHT)) topac->addTorque({ 0.0f, 0.0f, -tf });

        inert::vec3f impulse;
        float movePower = 2.0f;
        if (IsKeyDown(KEY_W)) impulse[2] -= movePower;
        if (IsKeyDown(KEY_S)) impulse[2] += movePower;
        if (IsKeyDown(KEY_A)) impulse[0] -= movePower;
        if (IsKeyDown(KEY_D)) impulse[0] += movePower;

        if (impulse.getLengthSqr() > 0.0f)
            topac->applyImpulseAtPoint(impulse, topac->getPosition());

        if (IsKeyDown(KEY_SPACE))
            topac->applyImpulseAtPoint({ 0.0f, 5.0f, 0.0f }, topac->getPosition());

        if (IsKeyDown(KEY_T)) {
            topac->setPosition({ 0.0f, 10.0f, 0.0f });
            topac->setVelocity({ 0.0f, 0.0f, 0.0f });
            topac->setAngularVelocity({ 300.0f, 0.0f, 0.0f });
        }

        if (IsKeyPressed(KEY_R)) {
            topac->setPosition({ 0.0f, 10.0f, 0.0f });
            topac->setVelocity({ 0.0f, 0.0f, 0.0f });
            topac->setAngularVelocity({ 0.0f, 0.0f, 0.0f });
            topac->setOrientation({ 0.0f, 0.0f, 0.0f, 1.0f });
        }

        world.step(dt);

        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);

                DrawPlane({ 0.0f, 0.0f, 0.0f }, { 50.0f, 50.0f }, LIGHTGRAY);
                DrawGrid(50, 1.0f);

                DrawSphere(toRaylib(topac->getPosition()), radius, Fade(MAROON, 0.3f));
                DrawSphereWires(toRaylib(topac->getPosition()), radius, 16, 16, Fade(BLACK, 0.2f));

                topac->debugDraw();

            EndMode3D();

            DrawFPS(10, 10);
            DrawText("TEST: POINT CLOUD VE TORK", 10, 40, 20, DARKGRAY);
            DrawText("E / Q Tusu : Kendi etrafinda Tork Uygula (Spin at)", 10, 70, 20, DARKBLUE);
            DrawText("WASD       : Merkezi Itki Uygula (Yerde Yuvarlanir)", 10, 95, 20, DARKGREEN);
            DrawText("T Tusu     : Havada Ileri Takla Attir", 10, 120, 20, MAROON);
            DrawText("SPACE / R  : Zipla / Sifirla", 10, 145, 20, BLACK);

            inert::vec3f rotVel = topac->getRotationVelocity();
            DrawText(TextFormat("Acsal Hiz X: %.2f", rotVel[0]), 10, 185, 20, BLACK);
            DrawText(TextFormat("Acsal Hiz Y: %.2f", rotVel[1]), 10, 210, 20, BLACK);
            DrawText(TextFormat("Acsal Hiz Z: %.2f", rotVel[2]), 10, 235, 20, BLACK);

        EndDrawing();
    }

    delete topac;
    CloseWindow();
    return 0;
}
