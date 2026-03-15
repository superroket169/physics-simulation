#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include "obj.hpp"
#include "collision.hpp"

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

    // ÇOK ÖNEMLİ: Eylemsizlik tensörünü atıyoruz ki obje dönebilsin.
    // Küre eylemsizliği: I = 2/5 * m * r^2 = 0.4 * 5 * 2^2 = 8.0
    topac->setInertia({ 8.0f, 8.0f, 8.0f });

    // 1. FİZİKSEL ÇARPIŞMA İÇİN KÜRE COLLIDER
    float radius = 2.0f;
    topac->addCollider(inert::ColliderType::SPHERE, { radius, 0.0f, 0.0f });

    // 2. GÖRSELLEŞTİRME İÇİN POINT CLOUD COLLIDER (obj.hpp'deki debugDraw için)
    std::vector<Vector3> pcPoints;
    
    // Merkez Şiş (Y Ekseni)
    pcPoints.push_back({ 0.0f, radius + 1.0f, 0.0f });  // Tepe noktası
    pcPoints.push_back({ 0.0f, -radius, 0.0f });        // Alt uç
    
    // Ekvator Çemberi (X ve Z Düzleminde sekizgen noktalar)
    int segments = 8;
    for (int i = 0; i < segments; i++) {
        float angle = (float)i / segments * PI * 2.0f;
        pcPoints.push_back({ cosf(angle) * radius, 0.0f, sinf(angle) * radius });
    }
    
    // Nokta bulutunu objeye ekle
    topac->addCollider(inert::ColliderType::POINT_CLOUD, pcPoints);

    world.addObject(topac);
    float tf = 300.0f;
    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (IsKeyDown(KEY_E)) {
            topac->addTorque({ 0.0f, tf, 0.0f }); 
        }

        if (IsKeyDown(KEY_Q)) {
            topac->addTorque({ 0.0f, -tf, 0.0f });
        
        }

        if (IsKeyDown(KEY_UP)) {
            topac->addTorque({ -tf, 0.0f, 0.0f }); 
        }

        if (IsKeyDown(KEY_DOWN)) {
            topac->addTorque({ tf, 0.0f, 0.0f });
        }


        if (IsKeyDown(KEY_LEFT)) {
            topac->addTorque({ 0.0f, 0.0f, tf }); 
        }

        if (IsKeyDown(KEY_RIGHT)) {
            topac->addTorque({ 0.0f, 0.0f, -tf });
        }

        Vector3 impulse = { 0.0f, 0.0f, 0.0f };
        float movePower = 2.0f;
        if (IsKeyDown(KEY_W)) impulse.z -= movePower;
        if (IsKeyDown(KEY_S)) impulse.z += movePower;
        if (IsKeyDown(KEY_A)) impulse.x -= movePower;
        if (IsKeyDown(KEY_D)) impulse.x += movePower;
        
        if (Vector3Length(impulse) > 0.0f) {
            // İtkiyi tam kütle merkezinden uyguluyoruz (saf öteleme)
            topac->applyImpulseAtPoint(impulse, topac->getPosition());
        }

        // SPACE: Zıplama
        if (IsKeyDown(KEY_SPACE)) {
            topac->applyImpulseAtPoint({ 0.0f, 5.0f, 0.0f }, topac->getPosition());
        }

        // T: İleri Takla atarak fırlatma (Yere çarpınca sürtünmeyle roket gibi gitmesi lazım)
        if (IsKeyDown(KEY_T)) {
            topac->setPosition({ 0.0f, 10.0f, 0.0f });
            topac->setVelocity({ 0.0f, 0.0f, 0.0f });
            // Direkt açısal hızı ayarlıyoruz
            topac->setAngularVelocity({ 300.0f, 0.0f, 0.0f }); 
        }

        // R: Sıfırlama
        if (IsKeyPressed(KEY_R)) {
            topac->setPosition({ 0.0f, 10.0f, 0.0f });
            topac->setVelocity({ 0.0f, 0.0f, 0.0f });
            topac->setAngularVelocity({ 0.0f, 0.0f, 0.0f });
            topac->setOrientation({ 0.0f, 0.0f, 0.0f, 1.0f });
        }

        // --- FİZİK ADIMI ---
        world.step(dt);

        // --- ÇİZİM ---
        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);
                
                // Zemin
                DrawPlane({ 0.0f, 0.0f, 0.0f }, { 50.0f, 50.0f }, LIGHTGRAY);
                DrawGrid(50, 1.0f);

                // Normal küreyi transparan çizelim ki içindeki Point Cloud net görünsün
                DrawSphere(topac->getPosition(), radius, Fade(MAROON, 0.3f));
                DrawSphereWires(topac->getPosition(), radius, 16, 16, Fade(BLACK, 0.2f));

                // SENİN YAZDIĞIN MUHTEŞEM DEBUG DRAW (Point Cloud'u çizer)
                topac->debugDraw();

            EndMode3D();

            // --- ARAYÜZ ---
            DrawFPS(10, 10);
            DrawText("TEST: POINT CLOUD VE TORK", 10, 40, 20, DARKGRAY);
            DrawText("E / Q Tusu : Kendi etrafinda Tork Uygula (Spin at)", 10, 70, 20, DARKBLUE);
            DrawText("WASD       : Merkezi Itki Uygula (Yerde Yuvarlanir)", 10, 95, 20, DARKGREEN);
            DrawText("T Tusu     : Havada Ileri Takla Attir (Yere degince firlar)", 10, 120, 20, MAROON);
            DrawText("SPACE / R  : Zipla / Sifirla", 10, 145, 20, BLACK);
            
            Vector3 rotVel = topac->getRotationVelocity();
            DrawText(TextFormat("Acsal Hiz X: %.2f", rotVel.x), 10, 185, 20, BLACK);
            DrawText(TextFormat("Acsal Hiz Y: %.2f", rotVel.y), 10, 210, 20, BLACK);
            DrawText(TextFormat("Acsal Hiz Z: %.2f", rotVel.z), 10, 235, 20, BLACK);

        EndDrawing();
    }

    delete topac;
    CloseWindow();
    return 0;
}
