#include "raylibGraphics.hpp"
#include "iostream"
#include "rlgl.h"
#include <cmath>
#include <algorithm>

#define RAYGUI_IMPLEMENTATION
#include "../../raylib/examples/core/raygui.h"

RaylibGraphics::RaylibGraphics(SimThread* _simThread) 
    : simThread(_simThread)
{
    // Initialize camera
    camera.position = (Vector3){ 15.0f, 5.0f, 15.0f };
    camera.target = (Vector3){ 0.0f, 5.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;
}

// Muunna drone-koordinaatisto (Z ylös) → Raylib-koordinaatisto (Y ylös)
Vector3 toRaylib(const Vector3f& v) {
    return {v.x(), v.z(), v.y()}; // drone X → raylib X, drone Z → raylib Y (ylös), drone Y → raylib Z
}

std::vector<Vector3> vectorToRaylib(std::vector<Vector3f> vector){
    std::vector<Vector3> result;
    for (const auto& v : vector) {
        result.push_back(toRaylib(v));
    }
    return result;
}

// Helper function: draw arrow from origin to origin + direction
void drawArrow(Vector3 origin, Vector3 direction, Color color, float scale = 1.0f) {
    Vector3 end = Vector3Add(origin, Vector3Scale(direction, scale));
    DrawLine3D(origin, end, color);
    // DrawSphere(end, 0.3f * scale, color);  // Arrow head
}

void drawDrone(Eigen::Vector3f position, Eigen::Quaternionf orientation ){
    Vector3 center = toRaylib(position);

    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);

    // Kvaternio → rotaatiomatriisi
    Eigen::Matrix3f rotMat = orientation.toRotationMatrix();
    
    // Transponoi matriisi (käänteinen rotaatio)
    rotMat.transposeInPlace();
    
    // Muunna raylib-koordinaatistoon
    float m[16] = {
        rotMat(0,0), rotMat(0,2), rotMat(0,1), 0,
        rotMat(2,0), rotMat(2,2), rotMat(2,1), 0,
        rotMat(1,0), rotMat(1,2), rotMat(1,1), 0,
        0, 0, 0, 1
    };
    rlMultMatrixf(m);

    DrawCube({0, 0, 0}, 0.6f, 0.1f, 0.3f, DARKGRAY);
    DrawCubeWires({0, 0, 0}, 0.6f, 0.1f, 0.3f, BLACK);

    // Propellit - eri väreillä
    const float arm = 0.3f;
    Eigen::Vector3f offsets[4] = {
        {-arm, -arm, 0}, {arm, -arm, 0},
        {-arm, arm, 0}, {arm, arm, 0}
    };
    
    Color propellerColors[4] = {
        RED,    // BL (Back-Left)
        GREEN,  // BR (Back-Right)
        BLUE,   // FL (Front-Left)
        PINK   // FR (Front-Right)
    };

    for (int i = 0; i < 4; i++) {
        Vector3 rp = toRaylib(offsets[i]);
        DrawLine3D({0, 0, 0}, rp, propellerColors[i]);

        // Pyörivät terät - kulma kasvaa ajan myötä, eri propelleilla vaihe-ero
        float angle = (float)GetTime() * 720.0f + i * 90.0f;
        float rad = angle * DEG2RAD;
        float bl = 0.13f;
        float cx = cosf(rad) * bl, cz = sinf(rad) * bl;
        float cx2 = cosf(rad + (float)M_PI * 0.5f) * bl;
        float cz2 = sinf(rad + (float)M_PI * 0.5f) * bl;

        // Terät (2 viivaa, 4 kärkeä)
        DrawLine3D({rp.x + cx,  rp.y, rp.z + cz},  {rp.x - cx,  rp.y, rp.z - cz},  propellerColors[i]);
        DrawLine3D({rp.x + cx2, rp.y, rp.z + cz2}, {rp.x - cx2, rp.y, rp.z - cz2}, propellerColors[i]);

        // Sumennusympyrä propellin alle
        DrawCircle3D(rp, bl, {0, 1, 0}, 0, Fade(propellerColors[i], 0.35f));

        // Napa
        DrawSphere(rp, 0.025f, DARKGRAY);
    }

    rlPopMatrix();

    // Nuolet
    Eigen::Vector3f x_world = orientation * Eigen::Vector3f(1,0,0);
    Eigen::Vector3f y_world = orientation * Eigen::Vector3f(0,1,0);
    Eigen::Vector3f z_world = orientation * Eigen::Vector3f(0,0,1);

    drawArrow(center, toRaylib(x_world.normalized()), RED, 1.0f);
    drawArrow(center, toRaylib(y_world.normalized()), GREEN, 1.0f);
    drawArrow(center, toRaylib(z_world.normalized()), BLUE, 1.0f);
}

void drawDroneBody(Eigen::Vector3f position, Eigen::Quaternionf orientation, Color bodyColor, Color wireColor)
{
    Vector3 center = toRaylib(position);

    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);

    Eigen::Matrix3f rotMat = orientation.toRotationMatrix();
    rotMat.transposeInPlace();

    float m[16] = {
        rotMat(0,0), rotMat(0,2), rotMat(0,1), 0,
        rotMat(2,0), rotMat(2,2), rotMat(2,1), 0,
        rotMat(1,0), rotMat(1,2), rotMat(1,1), 0,
        0, 0, 0, 1
    };
    rlMultMatrixf(m);

    DrawCube({0, 0, 0}, 0.45f, 0.08f, 0.22f, bodyColor);
    DrawCubeWires({0, 0, 0}, 0.45f, 0.08f, 0.22f, wireColor);

    rlPopMatrix();
}

// Draw a multi-bar cost breakdown chart.
// Bars can be positive or negative; they grow up/down from a shared zero baseline.
static void drawCostBreakdown(int originX, int originY, int w, int h,
                               const CostBreakdown& cb)
{
    const int titleH = 18;
    const int labelH = 14;
    const int chartH = h - titleH - labelH - 6;
    const int chartY = originY + titleH;

    DrawRectangle(originX, originY, w, h, Fade(LIGHTGRAY, 0.80f));
    DrawRectangleLines(originX, originY, w, h, DARKGRAY);
    DrawText("Cost Breakdown (sum over horizon)", originX + 6, originY + 2, titleH - 4, DARKGRAY);

    const int   nBars = 6;
    const char* labels[6] = { "Lag", "Contour", "Progress", "LinVel", "AngVel", "Obstacle" };
    const double values[6] = {
        cb.lagCost, cb.contourCost, cb.progressCost,
        cb.linVelCost, cb.angVelCost, cb.obstacleCost
    };
    Color colors[6] = { BLUE, SKYBLUE, GREEN, YELLOW, ORANGE, RED };

    // Scale to the largest absolute value; ensure at least 1 to avoid div-by-zero
    double maxAbs = 1.0;
    for (int i = 0; i < nBars; ++i)
        maxAbs = std::max(maxAbs, std::abs(values[i]));

    const int pad  = 4;
    const int barW = (w - pad * (nBars + 1)) / nBars;

    // Zero baseline sits at the vertical midpoint of the chart area
    const int baseline = chartY + chartH / 2;
    const int halfH    = chartH / 2;

    // Baseline
    DrawLine(originX, baseline, originX + w, baseline, DARKGRAY);

    for (int i = 0; i < nBars; ++i) {
        const int bx  = originX + pad + i * (barW + pad);
        const double v = values[i];
        const int barPx = std::max(1, (int)(std::abs(v) / maxAbs * halfH));

        int by;
        Color col = colors[i];
        if (v >= 0.0) {
            by = baseline - barPx;
            DrawRectangle(bx, by, barW, barPx, col);
            DrawRectangleLines(bx, by, barW, barPx, Fade(BLACK, 0.3f));
        } else {
            by = baseline;
            DrawRectangle(bx, by, barW, barPx, Fade(col, 0.6f));
            DrawRectangleLines(bx, by, barW, barPx, Fade(BLACK, 0.3f));
        }

        // Value label
        const char* valStr = TextFormat("%.0f", v);
        int         valW   = MeasureText(valStr, 10);
        int         valY   = (v >= 0.0) ? (by - 12) : (by + barPx + 2);
        DrawText(valStr, bx + (barW - valW) / 2, valY, 10, DARKGRAY);

        // Category label at very bottom
        int lblW = MeasureText(labels[i], 9);
        DrawText(labels[i], bx + (barW - lblW) / 2,
                 originY + h - labelH, 9, DARKGRAY);
    }
}

void RaylibGraphics::cast()
{
    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode3D(camera);

    // Draw grid and coordinate axes at origin
    DrawGrid(30, 5.0f); // 20x20 ruudukko, ruudun koko 1.0
    DrawLine3D((Vector3){0,0,0}, (Vector3){25,0,0}, RED);   // X
    DrawLine3D((Vector3){0,0,0}, (Vector3){0,25,0}, GREEN); // Y
    DrawLine3D((Vector3){0,0,0}, (Vector3){0,0,25}, BLUE);  // Z

    SimSnapshot simSnapshot = simThread->getSimSnapshot();

   for (auto droneWholeSnapshot : simSnapshot.droneWholeSnapshots){
        
    // ===== hardMissile =====
        Vector3f dronePos = droneWholeSnapshot.position;
        Eigen::Quaternionf orientation = droneWholeSnapshot.orientation;

        drawDrone(dronePos, orientation);

        // Piirrä ennustettu reitti haaleana
        const auto& traj = droneWholeSnapshot.predictedTrajectory;
        for (size_t i = 0; i + 1 < traj.size(); ++i) {
            Vector3 a = toRaylib(traj[i].position);
            Vector3 b = toRaylib(traj[i + 1].position);
            DrawLine3D(a, b, Fade(SKYBLUE, 0.35f));
        }
        // Joka 10. piste: rungon mallinen haalea laatikko omalla orientaatiolla
        for (size_t i = 0; i < traj.size(); i += 10) {
            drawDroneBody(
                traj[i].position,
                traj[i].orientation,
                Fade(DARKBLUE, 0.20f),
                Fade(SKYBLUE, 0.30f));
        }
   }

    // Draw ball obstacle
    {
        Vector3f goal = simThread->getGoal();
        float ballR = simThread->getBallRadius();
        Vector3 goalRaylib = toRaylib(goal);
        DrawSphere(goalRaylib, ballR, Fade(RED, 0.5f));
        DrawSphereWires(goalRaylib, ballR, 8, 8, RED);
    }

    EndMode3D();

    // Draw UI controls
    Controller* controller = simThread->getController();
    if (controller) {
        Vector3f goal = simThread->getGoal();
        float goalX = goal.x();
        float goalY = goal.y();
        float goalZ = goal.z();

        // ---- 2D joystick for XY goal ----
        const int stickCx = 190;
        const int stickCy = 180;
        const int stickR  = 200;   // outer radius
        const float xyRange = 15.0f;

        DrawCircle(stickCx, stickCy, stickR, Fade(LIGHTGRAY, 0.8f));
        DrawCircleLines(stickCx, stickCy, stickR, DARKGRAY);
        // crosshair
        DrawLine(stickCx - stickR, stickCy, stickCx + stickR, stickCy, GRAY);
        DrawLine(stickCx, stickCy - stickR, stickCx, stickCy + stickR, GRAY);
        GuiLabel((Rectangle){stickCx - stickR, stickCy + stickR + 4, stickR*2, 16}, "XY Goal (drag)");

        // Thumb position mapped from goal
        int thumbX = stickCx + (int)(goalX / xyRange * stickR);
        int thumbY = stickCy - (int)(goalY / xyRange * stickR); // Y axel: upp = positiv
        thumbX = std::max(stickCx - stickR, std::min(stickCx + stickR, thumbX));
        thumbY = std::max(stickCy - stickR, std::min(stickCy + stickR, thumbY));
        DrawCircle(thumbX, thumbY, 14, Fade(DARKBLUE, 0.85f));
        DrawCircleLines(thumbX, thumbY, 14, BLUE);

        // Drag interaction
        Vector2 mouse = GetMousePosition();
        float dx = mouse.x - stickCx;
        float dy = mouse.y - stickCy;
        float dist = sqrtf(dx*dx + dy*dy);
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && dist <= stickR) {
            float nx = std::max(-1.0f, std::min(1.0f, dx / stickR));
            float ny = std::max(-1.0f, std::min(1.0f, -dy / stickR));
            goalX = nx * xyRange;
            goalY = ny * xyRange;
        }

        // ---- Z slider ----
        GuiGroupBox((Rectangle){ 10, 310, 160, 60 }, "Ball Z");
        GuiSlider((Rectangle){ 20, 330, 140, 20 }, "0", "20", &goalZ, -5.0f, 20.0f);

        // ---- Ball radius slider ----
        float ballR = simThread->getBallRadius();
        GuiGroupBox((Rectangle){ 10, 380, 160, 60 }, "Ball Radius");
        GuiSlider((Rectangle){ 20, 400, 140, 20 }, "0.5", "5", &ballR, 0.5f, 5.0f);
        simThread->setBallRadius(ballR);

        DrawText(TextFormat("X:%.1f  Y:%.1f  Z:%.1f  R:%.1f", goalX, goalY, goalZ, ballR), 10, 450, 16, DARKGRAY);

        simThread->setGoal(Vector3f{goalX, goalY, goalZ});
    }

    // Cost breakdown bar charts (one per drone, stacked vertically on the right)
    {
        const int chartW = 340;
        const int chartH = 180;
        const int marginR = 10;
        const int marginT = 10;
        int chartX = screenWidth - chartW - marginR;

        for (size_t i = 0; i < simSnapshot.droneWholeSnapshots.size(); ++i) {
            const auto& snap = simSnapshot.droneWholeSnapshots[i];
            int chartY = marginT + (int)i * (chartH + 10);
            drawCostBreakdown(chartX, chartY, chartW, chartH, snap.costBreakdown);
        }
    }

    EndDrawing();
}

void RaylibGraphics::updateCamera()
{
    // Toggle follow mode with T key
    if (IsKeyPressed(KEY_T)) {
        followDrone = !followDrone;
    }

    SimSnapshot simSnapshot = simThread->getSimSnapshot();

    if (followDrone && !simSnapshot.droneWholeSnapshots.empty()) {
        // Kolmannen persoonan kamera
        Vector3f dronePos = simSnapshot.droneWholeSnapshots.back().position;
        Eigen::Quaternionf orientation = simSnapshot.droneWholeSnapshots.back().orientation;
        
        // Kameran etäisyys ja korkeus dronesta
        float distance = 5.0f;
        float height = 2.0f;
        
        // Laske kameran sijainti dronen takana (dronen -X suunta)
        Eigen::Vector3f backDir = orientation * Eigen::Vector3f(-1, 0, 0);
        Eigen::Vector3f upDir = Eigen::Vector3f(0, 0, 1); // Z ylös drone-koordinaatistossa
        
        Eigen::Vector3f cameraPos = dronePos + backDir * distance + upDir * height;
        
        camera.position = toRaylib(cameraPos);
        camera.target = toRaylib(dronePos);
        camera.up = toRaylib(upDir);
    }
    else {
        // Vapaa kamera kun ei seurata
        Vector3 movement = {
            (IsKeyDown(KEY_W) - IsKeyDown(KEY_S)) * 0.25f,
            (IsKeyDown(KEY_D) - IsKeyDown(KEY_A)) * 0.25f,
            (IsKeyDown(KEY_Q) - IsKeyDown(KEY_E)) * 0.25f
        };
        
        Vector3 rotation = {0.0f, 0.0f, 0.0f};
        
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            rotation.x = GetMouseDelta().x * 0.5f;
            rotation.y = GetMouseDelta().y * 0.5f;
        }
        
        float zoom = GetMouseWheelMove() * 2.0f;

        UpdateCameraPro(&camera, movement, rotation, zoom);
    }
}

void RaylibGraphics::initRaylib()
{
    InitWindow(screenWidth, screenHeight, "Missile Simulation");
}

void RaylibGraphics::closeRaylib() {
    CloseWindow();
}