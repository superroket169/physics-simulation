#include "raylib.h"
#include "raymath.h"
#include <vector>
#include "obj.hpp"

int main() {
    InitWindow(800, 600, "Inertia Physics - Test 1");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = { 5.0f, 3.0f, 5.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    inert::PhysicsBody testObj;
    testObj.setMass(1.0f);
    testObj.setInertia({ 0.5f, 0.5f, 0.8f });
    testObj.setRestitution(1.0f);
    testObj.addGround(0.0f);

    std::vector<Vector3> points = {
        { -0.5f, -0.5f,  0.5f },
        {  0.5f, -0.5f,  0.5f },
        { -0.5f, -0.5f, -0.5f },
        {  0.5f, -0.5f, -0.5f },
        { -0.5f, 0.5f,  0.5f },
        {  0.5f, 0.5f,  0.5f },
        { -0.5f, 0.5f, -0.5f },
        {  0.5f, 0.5f, -0.5f }
    };
    testObj.addCollider(inert::ColliderType::POINT_CLOUD, points);

    testObj.move({ 0.0f, 3.0f, 0.0f });
    testObj.setAngularVelocity({ 0.0f, 10.0f, 0.0f });
    testObj.setVelocity({ 0.f, 0.0f, 0.0f });

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        testObj.updateBody(dt);

        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);
                
                DrawGrid(10, 1.0f);
                testObj.debugDraw();

            EndMode3D();
            
            DrawFPS(10, 10);
            DrawText("inertia-engine test", 10, 30, 20, DARKGRAY);
            
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
