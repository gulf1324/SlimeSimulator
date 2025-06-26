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

// Vector2Subtract 함수 정의 추가  
Vector2 Vector2Subtract(Vector2 v1, Vector2 v2) {
    return { v1.x - v2.x, v1.y - v2.y };
}

// Vector2Add 함수 정의 추가
Vector2 Vector2Add(Vector2 v1, Vector2 v2) {
	return { v1.x + v2.x, v1.y + v2.y };
}

//Vector2Length 함수 정의 추가
float Vector2Length(Vector2 v) {
	return sqrtf(v.x * v.x + v.y * v.y);
}

// Vector2Scale 함수 정의 추가
Vector2 Vector2Scale(Vector2 v, float scale) {
	return { v.x * scale, v.y * scale };
}

// Vector2Distance 함수 정의 추가
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

// 기존 ApplySpring 함수 → PBD 버전으로 변경
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

// 사각형의 면적을 유지하기 위한 PBD 함수
void EnforceAreaConstraint(
    Particle& p1, Particle& p2, Particle& p3, Particle& p4,
    float targetArea, float strength = 0.1f
) {
    // 두 삼각형의 면적 합 = 사각형 면적
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

    float correction = strength * diff / 4.0f; // 4점에 분산 적용

    // 보정 벡터를 점들 위치에 따라 간단히 가감 (실제보다 단순화)
    if (!p1.fixed) p1.position.y -= correction;
    if (!p2.fixed) p2.position.y += correction;
    if (!p3.fixed) p3.position.y += correction;
    if (!p4.fixed) p4.position.y -= correction;
}

int Orientation(Vector2 p, Vector2 q, Vector2 r) {
    float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (fabs(val) < 1e-6) return 0;
    return (val > 0) ? 1 : 2;
}

// 외곽선convex hull 알고리즘
std::vector<Vector2> ConvexHull(const std::vector<Vector2>& points) {
    int n = points.size();
    if (n < 3) return points;

    std::vector<Vector2> hull;
    int l = 0;

    for (int i = 1; i < n; i++)
        if (points[i].x < points[l].x)
            l = i;

    int p = l, q;
    do {
        hull.push_back(points[p]);
        q = (p + 1) % n;

        for (int i = 0; i < n; i++) {
            if (Orientation(points[p], points[i], points[q]) == 2)
                q = i;
        }
        p = q;
    } while (p != l);

    return hull;
}


int main() {
    const int screenWidth = 1000;
    const int screenHeight = 1000; // 화면 크기 설정
    InitWindow(screenWidth, screenHeight, "Slime Simulator (Verlet + PBD)");
    SetTargetFPS(100);


    //PBDIterations ↑	딱딱해지고 안정적(묵)
    //spacing ↓	더 촘촘하고 리치한 질감
    //mouseSpringStiffness ↓	마우스로 끌 때 부드럽고 느리게

    const int cols = 50, rows = 50;
    const float spacing = 10.0f;
    const float startX = 300.0f, startY = 100.0f;
    const float gravity = 300.0f;
    //const float stiffness = 0.2f;
    const float bounceFactor = -0.6f;
	const int PBDiterations = 2; // PBD 반복 횟수
	const float strength = 0.03f; // 면적 유지 강도
    
    // 가장자리 점들 연결 강화
    const float edgeStiffness = 1.0f;  // 기존보다 더 강하게
    
    // verlet 감속 처리
    const float damping = 0.98f;


    // 마우스
    int grabbedIndex = -1;
    const float mouseSpringStiffness = 5000.0f;

    std::vector<Particle> particles;

    // 격자 생성
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            Particle p;
            p.position = { startX + x * spacing, startY + y * spacing };
            p.prevPosition = Vector2Subtract(p.position, { 0, -50 * GetFrameTime() });
            p.radius = 6.0f;
            p.fixed = (y == 0); // 맨 윗줄 고정
            particles.push_back(p);
        }
    }

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        Vector2 mouse = GetMousePosition();

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            // 가장 가까운 점 찾기
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

        // 마우스 유지 시 점을 힘으로 당김
        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) && grabbedIndex != -1) {
            Particle& p = particles[grabbedIndex];
            
            Vector2 velocity = Vector2Subtract(mouse, p.position);
            float springForce = 4000.0f;  // 힘이 클수록 더 강하게 당겨짐
            Vector2 acceleration = Vector2Scale(velocity, springForce * dt * dt);
            p.position = Vector2Add(p.position, acceleration);

            // 반발력 적용 (Verlet 방식의 prevPosition 업데이트)
            Vector2 pullBack = Vector2Subtract(p.position, p.prevPosition);
            p.prevPosition = Vector2Subtract(p.position, Vector2Scale(pullBack, 0.9f));
        }

        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
            grabbedIndex = -1; // 마우스 버튼을 놓으면 해제
        }


        // 스프링 연결
        for (int iter = 0; iter < PBDiterations; ++iter) {
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    int i = y * cols + x;
                
                    // 오른쪽
                    if (x < cols - 1)
                        EnforceDistanceConstraint(particles[i], particles[i + 1], spacing);

                    // 아래
                    if (y < rows - 1)
                        EnforceDistanceConstraint(particles[i], particles[i + cols], spacing);

                    // 대각선 (↘)
                    if (x < cols - 1 && y < rows - 1)
                        EnforceDistanceConstraint(particles[i], particles[i + cols + 1], spacing * 1.4142f);

                    // 대각선 (↙)
                    if (x > 0 && y < rows - 1)
                        EnforceDistanceConstraint(particles[i], particles[i + cols - 1], spacing * 1.4142f);
                }
			}
            // 가장자리 점들 연결 강화
            for (int y = 0; y < rows; y++) {
                for (int x = 0; x < cols; x++) {
                    int i = y * cols + x;

                    // 왼쪽 테두리
                    if (x == 0 && x < cols - 1)
                        EnforceDistanceConstraint(particles[i], particles[i + 1], spacing * 1.0f * 0.95f, edgeStiffness);
                    // 오른쪽 테두리
                    if (x == cols - 1 && x > 0)
                        EnforceDistanceConstraint(particles[i], particles[i - 1], spacing * 1.0f * 0.95f, edgeStiffness);
                    // 위 테두리
                    if (y == 0 && y < rows - 1)
                        EnforceDistanceConstraint(particles[i], particles[i + cols], spacing * 1.0f * 0.95f, edgeStiffness);
                    // 아래 테두리
                    if (y == rows - 1 && y > 0)
                        EnforceDistanceConstraint(particles[i], particles[i - cols], spacing * 1.0f * 0.95f, edgeStiffness);
                }
            }
		}

        

        // Verlet 위치 업데이트 + 바닥 충돌
        for (auto& p : particles) {
            if (p.fixed) continue;

            Vector2 temp = p.position;
            Vector2 acceleration = { 0, gravity };
			
            // 감속 적용
            Vector2 velocity = Vector2Subtract(p.position, p.prevPosition);
            velocity = Vector2Scale(velocity, damping);
            p.prevPosition = Vector2Subtract(p.position, velocity);

            // Verlet
            p.position = Vector2Add(
                Vector2Add(p.position, Vector2Subtract(p.position, p.prevPosition)),
                Vector2Scale(acceleration, dt * dt)
            );

            // 바닥 충돌
            if (p.position.y + p.radius > 980) {
                p.position.y = 980 - p.radius;

                // 충돌 감쇠: 이전 위치를 밑으로 당긴 위치로 보정
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


        // 메쉬 렌더링 ==> DrawTriangle
        BeginDrawing();
        ClearBackground(RAYWHITE);
        //DrawRectangle(0, 0, screenWidth, screenHeight, ColorAlpha(RAYWHITE, 0.9f)); // 살짝 흐릿한 누적

        // 나중에 최적화 위해 삼각형 남겨둠
        Color slimeColor = ColorAlpha(SKYBLUE, 0.8f);

        for (int y = 0; y < rows - 1; ++y) {
            for (int x = 0; x < cols - 1; ++x) {
                int i1 = y * cols + x;
                int i2 = i1 + 1;
                int i3 = i1 + cols;
                int i4 = i3 + 1;

                // 사각형을 두 개의 삼각형으로 나눠 그리기
                DrawTriangle(particles[i1].position, particles[i4].position, particles[i2].position, slimeColor);
                DrawTriangle(particles[i1].position, particles[i3].position, particles[i4].position, slimeColor);
            }
        }

        // 외곽선
        std::vector<Vector2> slimePoints;
        for (auto& p : particles)
            if (!p.fixed)
                slimePoints.push_back(p.position);

       /* Color core = ColorAlpha(SKYBLUE, 0.6f);
        Color shell = ColorAlpha(BLUE, 0.2f);
        for (const auto& p : particles) {
            Color inner = ColorAlpha(SKYBLUE, 0.5f);
            Color outer = ColorAlpha(SKYBLUE, 0.1f);

            DrawCircleGradient(p.position.x, p.position.y, p.radius * 2.0f, core, shell);
        }*/

        DrawText("slime grid simulation!", 10, 10, 20, DARKGRAY);
        //DrawFPS(10, 10);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}