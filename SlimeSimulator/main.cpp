#include "raylib.h"
#include <initializer_list> 
#include <cmath>
#include <vector>

struct Particle {
    Vector2 position;
    Vector2 prevPosition;
    float radius;
    bool fixed;
};


// Vector2Subtract �Լ� ���� �߰�  
Vector2 Vector2Subtract(Vector2 v1, Vector2 v2) {
    return { v1.x - v2.x, v1.y - v2.y };
}

// Vector2Add �Լ� ���� �߰�
Vector2 Vector2Add(Vector2 v1, Vector2 v2) {
	return { v1.x + v2.x, v1.y + v2.y };
}

//Vector2Length �Լ� ���� �߰�
float Vector2Length(Vector2 v) {
	return sqrtf(v.x * v.x + v.y * v.y);
}

// Vector2Scale �Լ� ���� �߰�
Vector2 Vector2Scale(Vector2 v, float scale) {
	return { v.x * scale, v.y * scale };
}

// Vector2Distance �Լ� ���� �߰�
float Vector2Distance(Vector2 v1, Vector2 v2) {
	return Vector2Length(Vector2Subtract(v1, v2));
}

void ApplySpring(Particle& a, Particle& b, float restLength, float stiffness) {
    if (a.fixed && b.fixed) return;

    Vector2 delta = Vector2Subtract(b.position, a.position);
    float distance = Vector2Length(delta);
    if (distance == 0.0f) return;

    float difference = (distance - restLength) / distance;
    Vector2 offset = Vector2Scale(delta, 0.5f * stiffness * difference);

    if (!a.fixed)
        a.position = Vector2Add(a.position, offset);
    if (!b.fixed)
        b.position = Vector2Subtract(b.position, offset);
}

// ���� ApplySpring �Լ� �� PBD �������� ����
void EnforceDistanceConstraint(Particle& a, Particle& b, float restLength) {
    Vector2 delta = Vector2Subtract(b.position, a.position);
    float dist = Vector2Length(delta);
    if (dist == 0.0f) return;

    float diff = (dist - restLength) / dist;
    Vector2 correction = Vector2Scale(delta, 0.5f * diff);

    if (!a.fixed)
        a.position = Vector2Add(a.position, correction);
    if (!b.fixed)
        b.position = Vector2Subtract(b.position, correction);
}

int main() {
    InitWindow(800, 600, "Slime Simulator (Verlet + PBD)");
    SetTargetFPS(100);

    const int cols = 50, rows = 30;
    const float spacing = 5.0f;
    const float startX = 300.0f, startY = 100.0f;
    const float gravity = 100.0f;
    const float stiffness = 0.3f;
    const float bounceFactor = -0.6f;
    
    // ���콺
    int grabbedIndex = -1;
    const float mouseSpringStiffness = 2000.0f;

    std::vector<Particle> particles;

    // ���� ����
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            Particle p;
            p.position = { startX + x * spacing, startY + y * spacing };
            p.prevPosition = Vector2Subtract(p.position, { 0, -50 * GetFrameTime() });
            p.radius = 6.0f;
            p.fixed = (y == 0); // �� ���� ����
            particles.push_back(p);
        }
    }

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        Vector2 mouse = GetMousePosition();

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            // ���� ����� �� ã��
            float minDist = 20.0f;
            for (int i = 0; i < particles.size(); ++i) {
                if (particles[i].fixed) continue;
                float d = Vector2Distance(particles[i].position, mouse);
                if (d < minDist) {
                    minDist = d;
                    grabbedIndex = i;
                }
            }
        }

        // ���콺 ���� �� ���� ������ ���
        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) && grabbedIndex != -1) {
            Particle& p = particles[grabbedIndex];
            
            Vector2 delta = Vector2Subtract(mouse, p.position);
            float dist = Vector2Length(delta);
            if (dist > 0.0f) {
                Vector2 direction = Vector2Scale(delta, 1.0f / dist);
                Vector2 offset = Vector2Scale(direction, mouseSpringStiffness * dt);
                p.position = Vector2Add(p.position, offset);
                
                // �̰� �߿���: prevPosition�� ���󰡰� ����
                p.prevPosition = Vector2Subtract(p.position, offset);
            }
        }

        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
            grabbedIndex = -1; // ���콺 ��ư�� ������ ����
        }

        // ������ ����
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                int i = y * cols + x;

                // ������
                if (x < cols - 1)
                    ApplySpring(particles[i], particles[i + 1], spacing, stiffness);

                // �Ʒ�
                if (y < rows - 1)
                    ApplySpring(particles[i], particles[i + cols], spacing, stiffness);

                // �밢�� (��)
                if (x < cols - 1 && y < rows - 1)
                    ApplySpring(particles[i], particles[i + cols + 1], spacing * 1.4142f, stiffness);

                // �밢�� (��)
                if (x > 0 && y < rows - 1)
                    ApplySpring(particles[i], particles[i + cols - 1], spacing * 1.4142f, stiffness);
            }
        }

        // Verlet ��ġ ������Ʈ + �ٴ� �浹
        for (auto& p : particles) {
            if (p.fixed) continue;
            Vector2 temp = p.position;
            Vector2 acceleration = { 0, gravity };
                
            // Verlet
            p.position = Vector2Add(
                Vector2Add(p.position, Vector2Subtract(p.position, p.prevPosition)),
                Vector2Scale(acceleration, dt * dt)
            );

            // �ٴ� �浹
            if (p.position.y + p.radius > 580) {
                p.position.y = 580 - p.radius;

                // �浹 ����: ���� ��ġ�� ������ ��� ��ġ�� ����
                p.prevPosition.y = p.position.y + (p.position.y - p.prevPosition.y) * bounceFactor;
            }

            p.prevPosition = temp;
        }

        // ������
        BeginDrawing();
        ClearBackground(RAYWHITE);

        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                int i = y * cols + x;
                DrawCircleV(particles[i].position, particles[i].radius, SKYBLUE);

                if (x < cols - 1)
                    DrawLineV(particles[i].position, particles[i + 1].position, LIGHTGRAY);
                if (y < rows - 1)
                    DrawLineV(particles[i].position, particles[i + cols].position, LIGHTGRAY);
            }
        }

        DrawText("slime grid simulation!", 10, 10, 20, DARKGRAY);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}