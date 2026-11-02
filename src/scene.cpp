#include "fluid.h"
#include <imgui.h>
#include <cmath>
#include <algorithm>

void Fluid::drawScene(int render_option) const
{
    ImDrawList* d = ImGui::GetBackgroundDrawList();
    auto p = [&](float x, float y) { return ImVec2(x, viewport_h - y); };
    auto rect = [&](float x0, float y0, float x1, float y1, ImU32 color, float round = 0) {
        d->AddRectFilled(p(x0, y1), p(x1, y0), color, round);
    };
    auto circle = [&](float x, float y, float r, ImU32 color) {
        d->AddCircleFilled(p(x, y), r, color, 20);
    };
    auto line = [&](float x0, float y0, float x1, float y1, ImU32 color, float width) {
        d->AddLine(p(x0, y0), p(x1, y1), color, width);
    };
    auto text = [&](float x, float y, float size, ImU32 color, const char* value) {
        d->AddText(ImGui::GetFont(), size, p(x, y), color, value);
    };
    const ImU32 ink = IM_COL32(26, 39, 43, 255);
    rect(67, 27, 1034, 833, IM_COL32(12, 21, 29, 255), 22);
    rect(70, 35, 1030, 830, IM_COL32(102, 76, 112, 255), 20);
    rect(79, 45, 1021, 821, IM_COL32(38, 65, 73, 255), 12);
    d->PushClipRect(p(80, 820), p(1020, 40), true);

    // Soil, permanent masonry, and cavities all use the same collision grid.
    for (int r = 4; r < std::min(cell_rows, 82); ++r)
        for (int c = 8; c < std::min(cell_cols, 102); ++c)
        {
            float x = c * cell_dim, y = r * cell_dim;
            if (fixed_wall[r][c])
            {
                int row = r / 2;
                int mortar = ((c + (row % 2) * 2) % 5 == 0);
                int tone = (row * 17 + c / 5 * 7) % 16;
                rect(x, y, x + cell_dim, y + cell_dim,
                     IM_COL32(52 + tone, 117 + tone, 122 + tone, 255));
                if (r % 2 == 0) line(x, y + 1, x + cell_dim, y + 1, IM_COL32(33, 78, 88, 255), 2);
                if (mortar) line(x + 1, y, x + 1, y + cell_dim, IM_COL32(33, 78, 88, 255), 2);
                if (!isTerrainSolid(r + 1, c)) line(x, y + cell_dim - 1, x + cell_dim, y + cell_dim - 1,
                    IM_COL32(128, 190, 169, 255), 3);
            }
            else if (terrain_solid[r][c])
            {
                unsigned hash = static_cast<unsigned>(r * 374761393u + c * 668265263u);
                int noise = (hash >> 17) % 12;
                rect(x, y, x + cell_dim, y + cell_dim, IM_COL32(178 + noise, 125 + noise, 66 + noise / 2, 255));
                if (hash % 7 == 0) circle(x + 3 + hash % 4, y + 4, 1.3f, IM_COL32(139, 94, 55, 130));
                if (hash % 23 == 0) line(x + 2, y + 7, x + 5, y + 8, IM_COL32(220, 166, 98, 180), 1.5f);
                const ImU32 edge = IM_COL32(225, 160, 216, 255);
                if (!isTerrainSolid(r + 1, c)) line(x, y + cell_dim - 1, x + cell_dim, y + cell_dim - 1, edge, 3);
                if (!isTerrainSolid(r - 1, c)) line(x, y + 1, x + cell_dim, y + 1, edge, 3);
                if (!isTerrainSolid(r, c - 1)) line(x + 1, y, x + 1, y + cell_dim, edge, 3);
                if (!isTerrainSolid(r, c + 1)) line(x + cell_dim - 1, y, x + cell_dim - 1, y + cell_dim, edge, 3);
            }
            else if (render_option != 2 && cell_types[r][c] == FLUID)
                rect(x, y, x + cell_dim, y + cell_dim, IM_COL32(40, 146, 196, 210));
        }

    // Brick-lined bathroom, kept clear by a protected-air mask.
    rect(360, 70, 750, 290, IM_COL32(55, 42, 60, 255));
    for (int row = 0; row < 8; ++row)
        for (int col = 0; col < 8; ++col)
        {
            float x = 361 + col * 56.0f - (row % 2) * 28;
            float y = 72 + row * 28.0f;
            rect(std::max(x, 361.0f), y, std::min(x + 53, 749.0f), y + 25,
                 IM_COL32(82 + (col * 3 + row * 7) % 18, 52, 66, 255), 3);
            line(std::max(x + 3, 363.0f), y + 23, std::min(x + 49, 747.0f), y + 23,
                 IM_COL32(137, 79, 90, 160), 1);
        }
    rect(383, 78, 726, 93, IM_COL32(29, 33, 45, 255), 6);

    // A connected pipe: intake -> underground run -> bathroom shower nozzle.
    const ImVec2 route[] = {p(900, 300), p(960, 300), p(960, 120), p(730, 120), p(730, 245), p(704, 245)};
    d->AddPolyline(route, 6, IM_COL32(26, 43, 48, 255), false, 43);
    d->AddPolyline(route, 6, IM_COL32(87, 142, 146, 255), false, 34);
    d->AddPolyline(route, 6, IM_COL32(148, 191, 180, 255), false, 23);
    d->AddPolyline(route, 6, IM_COL32(94, 148, 151, 255), false, 13);
    for (float y : {170.0f, 235.0f}) {
        rect(937, y - 5, 983, y + 5, IM_COL32(46, 88, 97, 255), 3);
        line(942, y + 2, 977, y + 2, IM_COL32(172, 205, 187, 255), 2);
    }
    rect(880, 269, 908, 331, ink, 6);
    rect(885, 272, 903, 328, IM_COL32(177, 207, 169, 255), 4);
    rect(887, 277, 897, 323, IM_COL32(25, 67, 78, 255), 4);
    rect(695, 217, 712, 274, ink, 5);
    rect(698, 221, 708, 270, IM_COL32(181, 212, 184, 255), 3);
    if (!pipe_queue.empty()) {
        for (int i = 0; i < 10; ++i) {
            float t = std::fmod(scene_time * 0.6f + i * 0.1f, 1.0f);
            circle(959, 145 + t * 136, 3, IM_COL32(113, 224, 251, 230));
        }
    }
    text(828, 371, 15, IM_COL32(233, 240, 209, 255), "PIPE INLET");
    line(871, 350, 890, 332, IM_COL32(220, 231, 197, 255), 2);

    // Procedural artwork generated by OpenAI Codex for this project, inspired
    // by the user-provided reference. No external crocodile image is loaded.
    // See README.md for the visual asset provenance and reference notes.
    // Crocodile: tail, body, raised eyes, long snout and teeth.
    const ImU32 green = IM_COL32(106, 184, 91, 255);
    const ImU32 light = IM_COL32(177, 217, 111, 255);
    ImVec2 tail[] = {p(480, 117), p(407, 136), p(444, 100), p(491, 95)};
    d->AddConvexPolyFilled(tail, 4, IM_COL32(69, 139, 81, 255));
    circle(492, 140, 47, ink);
    circle(492, 142, 42, green);
    circle(501, 134, 29, light);
    rect(487, 160, 552, 198, ink, 17);
    rect(490, 164, 552, 194, green, 14);
    rect(514, 165, 605, 191, ink, 12);
    rect(517, 168, 603, 189, green, 10);
    circle(514, 200, 17, ink); circle(540, 201, 17, ink);
    circle(514, 201, 14, green); circle(540, 202, 14, green);
    circle(516, 204, 10, IM_COL32(247, 244, 215, 255));
    circle(542, 205, 10, IM_COL32(247, 244, 215, 255));
    circle(519, 204, 4, ink); circle(545, 205, 4, ink);
    circle(591, 182, 2.5f, ink);

    // The croc starts sad and visibly cheers up after the 70% clear threshold.
    if (isLevelCleared()) {
        // Smile: a shallow U-shaped mouth in screen space.
        line(535, 178, 550, 170, ink, 2.5f);
        line(550, 170, 574, 170, ink, 2.5f);
        line(574, 170, 590, 178, ink, 2.5f);
    } else {
        // Frown plus a small tear under the right eye.
        line(535, 170, 550, 179, ink, 2.5f);
        line(550, 179, 574, 179, ink, 2.5f);
        line(574, 179, 590, 170, ink, 2.5f);
        circle(548, 191, 3.0f, IM_COL32(112, 211, 240, 235));
        d->AddTriangleFilled(p(548, 187), p(544, 192), p(552, 192), IM_COL32(112, 211, 240, 235));
    }
    for (int i = 0; i < 4; ++i) {
        float x = 546 + i * 11.0f;
        d->AddTriangleFilled(p(x, 173), p(x + 7, 173), p(x + 3, 167), IM_COL32(251, 243, 211, 255));
    }
    line(465, 147, 442, 160, ink, 15); line(465, 149, 442, 162, green, 10);
    line(524, 142, 550, 153, ink, 15); line(524, 144, 550, 155, green, 10);
    for (int i = 0; i < 4; ++i)
        d->AddTriangleFilled(p(458, 152.0f + i * 8), p(448, 156.0f + i * 8), p(463, 160.0f + i * 8), light);
    text(390, 278, 16, IM_COL32(236, 215, 187, 255), isLevelCleared() ? "THAT'S THE SPOT!" : "A LITTLE WATER, PLEASE...");

    // Particle water is drawn after the character and before the tub's front lip.
    if (render_option != 0)
        for (int i = 0; i < num_particles; ++i)
        {
            const auto& q = particle_pos[i];
            // Match the original particle size and speed-to-blue color ramp.
            float speed = std::min(particle_vel[i].norm(), 50.0f) / 50.0f;
            circle(q.x(), q.y(), particle_radius,
                   IM_COL32(int((0.10f + 0.55f * speed) * 255),
                            int((0.40f + 0.35f * speed) * 255), 255, 255));
        }

    rect(411, 84, 642, 132, ink, 19);
    rect(417, 89, 636, 132, IM_COL32(197, 221, 199, 255), 16);
    rect(425, 117, 628, 137, IM_COL32(235, 237, 209, 255), 8);
    // The visible water level follows the real collected fraction, not the
    // 70% clear threshold. This keeps 70%, 85% and 95% visually distinct.
    float fill = getCollectedRatio();
    rect(435, 120, 620, 124 + fill * 9, IM_COL32(88, 191, 219, 255), 4);
    line(430, 82, 426, 73, ink, 9); line(622, 82, 627, 73, ink, 9);
    if (fill > 0) for (int i = 0; i < 12; ++i) {
        float x = 440 + i * 14.0f;
        float y = 132 + 4 * std::sin(scene_time * 2 + i);
        circle(x, y, 5.0f + i % 3, IM_COL32(210, 243, 231, 225));
    }
    d->PopClipRect();
    text(106, 853, 15, IM_COL32(166, 194, 187, 255), "01  /  THE UNDERGROUND BATH");
    text(690, 853, 15, IM_COL32(166, 194, 187, 255), "DIG  >  PIPE  >  SHOWER");
}
