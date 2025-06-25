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
void EnforceDistanceConstraint(Particle& a, Particle& b, float restLength, float stiffness = 1.0f) {
    Vector2 delta = Vector2Subtract(b.position, a.position);
    float dist = Vector2Length(delta);
    if (dist == 0.0f) return;

    float diff = (dist - restLength) / dist;
    Vector2 correction = Vector2Scale(delta, 0.5f * diff * stiffness);

    if (!a.fixed)
        a.position = Vector2Add(a.position, correction);
    if (!b.fixed)
        b.position = Vector2Subtract(b.position, correction);
}

// �簢���� ������ �����ϱ� ���� PBD �Լ�
void EnforceAreaConstraint(
    Particle& p1, Particle& p2, Particle& p3, Particle& p4,
    float targetArea, float strength = 0.1f
) {
    // �� �ﰢ���� ���� �� = �簢�� ����
    float area = 0.5f * fabs(
        (p1.position.x * (p2.position.y - p4.position.y) +
            p2.position.x * (p4.position.y - p1.position.y) +
            p4.position.x * (p1.position.y - p2.position.y)) +

        (p2.position.x * (p3.position.y - p4.position.y) +
            p3.position.x * (p4.position.y - p2.position.y) +
            p4.position.x * (p2.position.y - p3.position.y))
    );

    float diff = area - targetArea;
    if (fabs(diff) < 1e-2) return;

    float correction = strength * diff / 4.0f; // 4���� �л� ����

    // ���� ���͸� ���� ��ġ�� ���� ������ ���� (�������� �ܼ�ȭ)
    if (!p1.fixed) p1.position.y -= correction;
    if (!p2.fixed) p2.position.y += correction;
    if (!p3.fixed) p3.position.y += correction;
    if (!p4.fixed) p4.position.y -= correction;
}

int main() {
    InitWindow(1000, 1000, "Slime Simulator (Verlet + PBD)");
    SetTargetFPS(100);


    //PBDIterations ��	���������� ������(��)
    //spacing ��	�� �����ϰ� ��ġ�� ����
    //mouseSpringStiffness ��	���콺�� �� �� �ε巴�� ������

    const int cols = 50, rows = 50;
    const float spacing = 10.0f;
    const float startX = 300.0f, startY = 100.0f;
    const float gravity = 300.0f;
    //const float stiffness = 0.2f;
    const float bounceFactor = -0.6f;
	const int PBDiterations = 2; // PBD �ݺ� Ƚ��
	const float strength = 0.03f; // ���� ���� ����
    
    // �����ڸ� ���� ���� ��ȭ
    const float edgeStiffness = 1.0f;  // �������� �� ���ϰ�
    
    // verlet ���� ó��
    const float damping = 0.98f;


    // ���콺
    int grabbedIndex = -1;
    const float mouseSpringStiffness = 5000.0f;

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
        for (int iter = 0; iter < PBDiterations; ++iter) {
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    int i = y * cols + x;
                
                    // ������
                    if (x < cols - 1)
                        EnforceDistanceConstraint(particles[i], particles[i + 1], spacing);

                    // �Ʒ�
                    if (y < rows - 1)
                        EnforceDistanceConstraint(particles[i], particles[i + cols], spacing);

                    // �밢�� (��)
                    if (x < cols - 1 && y < rows - 1)
                        EnforceDistanceConstraint(particles[i], particles[i + cols + 1], spacing * 1.4142f);

                    // �밢�� (��)
                    if (x > 0 && y < rows - 1)
                        EnforceDistanceConstraint(particles[i], particles[i + cols - 1], spacing * 1.4142f);
                }
			}
            // �����ڸ� ���� ���� ��ȭ
            for (int y = 0; y < rows; y++) {
                for (int x = 0; x < cols; x++) {
                    int i = y * cols + x;

                    // ���� �׵θ�
                    if (x == 0 && x < cols - 1)
                        EnforceDistanceConstraint(particles[i], particles[i + 1], spacing * 1.0f * 0.95f, edgeStiffness);
                    // ������ �׵θ�
                    if (x == cols - 1 && x > 0)
                        EnforceDistanceConstraint(particles[i], particles[i - 1], spacing * 1.0f * 0.95f, edgeStiffness);
                    // �� �׵θ�
                    if (y == 0 && y < rows - 1)
                        EnforceDistanceConstraint(particles[i], particles[i + cols], spacing * 1.0f * 0.95f, edgeStiffness);
                    // �Ʒ� �׵θ�
                    if (y == rows - 1 && y > 0)
                        EnforceDistanceConstraint(particles[i], particles[i - cols], spacing * 1.0f * 0.95f, edgeStiffness);
                }
            }
		}


        // Verlet ��ġ ������Ʈ + �ٴ� �浹
        for (auto& p : particles) {
            if (p.fixed) continue;
            Vector2 temp = p.position;
            Vector2 acceleration = { 0, gravity };
			
            // ���� ����
            Vector2 velocity = Vector2Subtract(p.position, p.prevPosition);
            velocity = Vector2Scale(velocity, damping);
            p.prevPosition = Vector2Subtract(p.position, velocity);

            // Verlet
            p.position = Vector2Add(
                Vector2Add(p.position, Vector2Subtract(p.position, p.prevPosition)),
                Vector2Scale(acceleration, dt * dt)
            );

            // �ٴ� �浹
            if (p.position.y + p.radius > 980) {
                p.position.y = 980 - p.radius;

                // �浹 ����: ���� ��ġ�� ������ ��� ��ġ�� ����
                p.prevPosition.y = p.position.y + (p.position.y - p.prevPosition.y) * bounceFactor;
            }

            p.prevPosition = temp;
        }

        for (int y = 0; y < rows - 1; y++) {
            for (int x = 0; x < cols - 1; x++) {
                int i1 = y * cols + x;
                int i2 = i1 + 1;
                int i3 = i1 + cols;
                int i4 = i3 + 1;

                float restArea = spacing * spacing;

                EnforceAreaConstraint(particles[i1], particles[i2], particles[i3], particles[i4], restArea, strength);
            }
        }


        // �޽� ������ ==> DrawTriangle
        BeginDrawing();
        ClearBackground(RAYWHITE);

        Color slimeColor = ColorAlpha(SKYBLUE, 0.8f);

        for (int y = 0; y < rows - 1; ++y) {
            for (int x = 0; x < cols - 1; ++x) {
                int i1 = y * cols + x;
                int i2 = i1 + 1;
                int i3 = i1 + cols;
                int i4 = i3 + 1;

                // �簢���� �� ���� �ﰢ������ ���� �׸���
                DrawTriangle(particles[i1].position, particles[i4].position, particles[i2].position, slimeColor);
                DrawTriangle(particles[i1].position, particles[i3].position, particles[i4].position, slimeColor);
            }
        }

        DrawText("slime grid simulation!", 10, 10, 20, DARKGRAY);
        //DrawFPS(10, 10);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}