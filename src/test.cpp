#include "raylib.h"
#include "raymath.h"
#include <vector>
#include "obj.hpp"

// --- Gerçek Zamanlı Grafik Aracı ---
struct RealTimeGraph {
    std::vector<float> data;
    int maxPoints;
    Color color;
    const char* title;
    float maxExpectedValue;

    RealTimeGraph(int maxP, Color c, const char* t, float maxVal) 
        : maxPoints(maxP), color(c), title(t), maxExpectedValue(maxVal) {}

    void addValue(float val) {
        data.push_back(val);
        if (data.size() > maxPoints) {
            data.erase(data.begin());
        }
    }

    void draw(int x, int y, int width, int height) {
        DrawRectangle(x, y, width, height, Fade(LIGHTGRAY, 0.3f));
        DrawRectangleLines(x, y, width, height, DARKGRAY);
        DrawText(title, x + 5, y + 5, 10, DARKGRAY);

        if (data.empty()) return;

        float stepX = (float)width / maxPoints;
        for (size_t i = 1; i < data.size(); i++) {
            float x1 = x + (i - 1) * stepX;
            float y1 = y + height - (data[i - 1] / maxExpectedValue) * height;
            float x2 = x + i * stepX;
            float y2 = y + height - (data[i] / maxExpectedValue) * height;
            
            // Taşmaları (clamp) önle
            y1 = Clamp(y1, y, y + height);
            y2 = Clamp(y2, y, y + height);

            DrawLineEx({x1, y1}, {x2, y2}, 2.0f, color);
        }
        DrawText(TextFormat("%.2f", data.back()), x + width - 40, y + 5, 10, color);
    }
};

int main() {
    InitWindow(1024, 768, "Inertia Physics - Tekerlek ve Grafik Simülasyonu");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = { 0.0f, 3.0f, 8.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    inert::PhysicsBody wheel;
    float wheelMass = 2.0f;
    float wheelRadius = 1.0f;
    
    // Silindir (Tekerlek) için Eylemsizlik Momenti (I = 1/2 * m * r^2)
    float inertiaVal = 0.5f * wheelMass * (wheelRadius * wheelRadius);
    wheel.setMass(wheelMass);
    // Dönme ekseni Z ekseni olduğu için Z eylemsizliği önemlidir. 
    wheel.setInertia({ inertiaVal, inertiaVal, inertiaVal });
    wheel.setRestitution(0.1f);
    wheel.addGround(0.0f);

    // Dairesel bir nokta bulutu oluşturarak bir tekerlek profili çiziyoruz
    std::vector<Vector3> points;
    int segments = 16;
    for (int i = 0; i < segments; i++) {
        float angle = (i * 2.0f * PI) / segments;
        points.push_back({ cos(angle) * wheelRadius, sin(angle) * wheelRadius, 0.0f });
        points.push_back({ cos(angle) * wheelRadius, sin(angle) * wheelRadius, 0.5f });
        points.push_back({ cos(angle) * wheelRadius, sin(angle) * wheelRadius, -0.5f });
    }
    wheel.addCollider(inert::ColliderType::POINT_CLOUD, points);
    
    // Tekerleği havaya bırakıyoruz
    wheel.setPosition({ 0.0f, 3.0f, 0.0f });

    RealTimeGraph velocityGraph(200, BLUE, "Lineer Hiz (Z Ekseni)", 10.0f);
    RealTimeGraph angularGraph(100, RED, "Acisal Hiz (Rotasyon)", 15.0f);

    float upping = 3.0f;
    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

    // SADECE IsKeyDown KULLAN VE TEK BİR YERDE YAP
    if (IsKeyDown(KEY_SPACE)) { 
        Vector3 offsetFromCenter = { 0.0f, wheelRadius, 0.0f };
        wheel.addForceAtPoint({ upping, 0.0f, 0.0f }, offsetFromCenter);
    }

    if (IsKeyPressed(KEY_UP)) upping += 0.2;

    wheel.updateBody(dt);

        // Grafik verilerini güncelle
        velocityGraph.addValue(Vector3Length(wheel.getVelocity()));
        angularGraph.addValue(Vector3Length(wheel.getRotationVelocity()));

        // Kamerayı tekerleği takip edecek şekilde ayarla
        camera.target = wheel.getPosition();
        camera.position.z = wheel.getPosition().z + 8.0f;

        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);
            DrawGrid(50, 1.0f);
            wheel.debugDraw();
        EndMode3D();

        DrawFPS(10, 10);
        DrawText("SPACE: Tekerlegin tepesinden surekli kuvvet uygula", 10, 30, 20, DARKGRAY);
        
        // BURADAKİ İKİNCİ IsKeyPressed BLOĞUNU TAMAMEN SİL!
        
        velocityGraph.draw(10, 60, 200, 100);
        angularGraph.draw(10, 180, 200, 100);
    EndDrawing();    }

    CloseWindow();
    return 0;
}
