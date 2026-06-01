#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <unordered_map>
#include <vector>

#include "signal_generator.hpp"
#include "signal_processor.hpp"
#include "target_tracker.hpp"

static constexpr int   WATERFALL_ROWS = 150;
static constexpr float PI             = 3.14159265359f;
static constexpr float TWO_PI         = 6.28318530718f;
static constexpr float KM_PER_BIN    = 0.040f;  // 40 m/bin → max range ≈ 41 km
static constexpr float MS_PER_VEL    = 10.0f;   // m/s per bin/CPI velocity unit

struct RadarBlip {
    float angle;
    float range;      // normalised 0–1
    float intensity;  // 1.0 = fresh, decays to 0
    float velocity;   // |velocity| in bins/CPI, used for colour
};

// Per-track display state updated every frame by dead-reckoning.
struct TrackDisplay {
    float range;    // extrapolated range in bins
    float azimuth;  // last confirmed bearing in radians
};

static ImVec2 polar_to_screen(ImVec2 center, float angle, float r) {
    return ImVec2(center.x + r * std::sin(angle),
                  center.y - r * std::cos(angle));
}

// Green (slow) → yellow → red (fast)
static ImU32 velocity_color(float vel, int alpha) {
    float t = std::min(vel / MAX_VELOCITY, 1.0f);
    int r = int(255 * std::min(t * 2.0f, 1.0f));
    int g = int(255 * std::min((1.0f - t) * 2.0f, 1.0f));
    return IM_COL32(r, g, 0, alpha);
}

int main() {
    if (!glfwInit()) return -1;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(1400, 800, "RadarSim", nullptr, nullptr);
    if (!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    SignalGenerator generator(42, /*num_targets=*/4);
    SignalProcessor cfar(/*guard=*/2, /*reference=*/16, /*threshold=*/5.0f);
    // max_delta=12: generous gate because range prediction drifts slightly between sweeps.
    // miss_threshold=10: beam-gated, so this means ~10 beam-passes with no detection.
    TargetTracker tracker(/*max_delta=*/12.0f, /*miss_threshold=*/10);

    std::vector<float> waterfall_buf(WATERFALL_ROWS * RANGE_BINS, 0.0f);
    std::vector<float> waterfall_disp(WATERFALL_ROWS * RANGE_BINS, 0.0f);
    int waterfall_head = 0;

    std::vector<float> det_x, det_y;
    det_x.reserve(64);
    det_y.reserve(64);

    std::vector<RadarBlip> blips;
    blips.reserve(512);

    std::unordered_map<uint32_t, TrackDisplay> track_disp;

    float sweep_angle  = 0.0f;
    float rotation_rpm = 6.0f;

    float fps       = 60.0f;
    int   frame_cnt = 0;
    auto  t_last    = std::chrono::high_resolution_clock::now();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        auto  now = std::chrono::high_resolution_clock::now();
        float dt  = std::min(std::chrono::duration<float>(now - t_last).count(), 0.05f);
        t_last    = now;
        fps       = 0.9f * fps + 0.1f * (1.0f / dt);
        ++frame_cnt;

        float angle_step = TWO_PI * (rotation_rpm / 60.0f) * dt;
        sweep_angle += angle_step;
        if (sweep_angle >= TWO_PI) sweep_angle -= TWO_PI;

        // Adaptive beamwidth: must cover the full arc swept this frame or targets are
        // missed entirely at high RPM (beam advances faster than the fixed 5° width).
        float beamwidth = std::max(0.087f, angle_step * 1.5f);

        CPI cpi = generator.generate_cpi(sweep_angle, beamwidth);
        const auto& detections = cfar.process_cpi(cpi);
        tracker.update(detections, sweep_angle, beamwidth, frame_cnt);

        // Dead-reckon each track's display position every frame.
        // missed_frames == 0 means the track was just matched to a detection this frame.
        {
            const auto& tracks = tracker.tracks();
            for (const auto& t : tracks) {
                auto it = track_disp.find(t.id);
                if (it == track_disp.end()) {
                    track_disp[t.id] = {t.range_bin, t.azimuth};
                } else if (t.missed_frames == 0) {
                    // Fresh detection — snap display to measured position.
                    it->second.range   = t.range_bin;
                    it->second.azimuth = t.azimuth;
                } else {
                    // Coasting between sweeps — extrapolate by one frame of velocity.
                    it->second.range += t.velocity_est;
                }
            }
            for (auto it = track_disp.begin(); it != track_disp.end(); ) {
                bool alive = false;
                for (const auto& t : tracks)
                    if (t.id == it->first) { alive = true; break; }
                it = alive ? std::next(it) : track_disp.erase(it);
            }
        }

        for (const auto& d : detections)
            blips.push_back({sweep_angle, float(d.range_bin) / RANGE_BINS, 1.0f, d.velocity});

        constexpr float BLIP_LIFETIME = 1.5f;
        for (auto& b : blips) b.intensity -= dt / BLIP_LIFETIME;
        blips.erase(std::remove_if(blips.begin(), blips.end(),
            [](const RadarBlip& b){ return b.intensity <= 0.0f; }), blips.end());

        {
            float* row = &waterfall_buf[waterfall_head * RANGE_BINS];
            for (int i = 0; i < RANGE_BINS; ++i)
                row[i] = std::abs(cpi[0][i]);
            waterfall_head = (waterfall_head + 1) % WATERFALL_ROWS;
            for (int r = 0; r < WATERFALL_ROWS; ++r) {
                int src = (waterfall_head + r) % WATERFALL_ROWS;
                std::memcpy(&waterfall_disp[r * RANGE_BINS],
                            &waterfall_buf[src * RANGE_BINS],
                            RANGE_BINS * sizeof(float));
            }
        }

        det_x.clear(); det_y.clear();
        for (const auto& d : detections) {
            det_x.push_back(float(d.range_bin) * KM_PER_BIN);
            det_y.push_back(d.velocity * MS_PER_VEL);
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int fb_w, fb_h;
        glfwGetFramebufferSize(window, &fb_w, &fb_h);

        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(float(fb_w), float(fb_h)));
        ImGui::Begin("##root", nullptr,
            ImGuiWindowFlags_NoTitleBar  | ImGuiWindowFlags_NoResize    |
            ImGuiWindowFlags_NoMove      | ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoScrollWithMouse);

        bool blink = (frame_cnt / 30) % 2 == 0;
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 1.0f, 0.4f, 1.0f));
        ImGui::Text("PULSE-DOPPLER SEARCH RADAR");
        ImGui::PopStyleColor();
        ImGui::SameLine(0, 20);
        if (blink) ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "[  ACTIVE  ]");
        else        ImGui::TextColored(ImVec4(0.0f, 0.4f, 0.0f, 1.0f), "[  ACTIVE  ]");
        ImGui::SameLine(0, 20);
        ImGui::TextDisabled("PRF: MEDIUM  |  MODE: SEARCH  |  POL: LINEAR");
        ImGui::SameLine(0, 20);
        ImGui::TextDisabled("FPS: %.0f", fps);
        ImGui::Separator();

        float left_w  = float(fb_w) * 0.55f;
        float right_w = float(fb_w) - left_w - 8.0f;
        float top_h   = float(fb_h) * 0.45f;

        ImGui::BeginChild("##left", ImVec2(left_w, 0.0f));

        ImGui::BeginChild("##rdmap", ImVec2(-1.0f, top_h));
        if (ImPlot::BeginPlot("Range-Doppler Map", ImVec2(-1, -1))) {
            ImPlot::SetupAxes("Range (km)", "Velocity (m/s)");
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, RANGE_BINS * KM_PER_BIN, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1,
                -MAX_VELOCITY * MS_PER_VEL, MAX_VELOCITY * MS_PER_VEL, ImGuiCond_Always);

            ImPlot::SetNextLineStyle(ImVec4(0.4f, 0.4f, 0.4f, 0.6f));
            float zero_x[] = {0.0f, float(RANGE_BINS) * KM_PER_BIN};
            float zero_y[] = {0.0f, 0.0f};
            ImPlot::PlotLine("##zero", zero_x, zero_y, 2);

            ImPlot::PushColormap(ImPlotColormap_Hot);
            ImPlot::PlotHeatmap("##rdh", cfar.rd_display(), NUM_PULSES, RANGE_BINS,
                0.0, 0.0, nullptr,
                ImPlotPoint(0, -MAX_VELOCITY * MS_PER_VEL),
                ImPlotPoint(RANGE_BINS * KM_PER_BIN, MAX_VELOCITY * MS_PER_VEL));
            ImPlot::PopColormap();

            if (!det_x.empty()) {
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Cross, 8,
                    ImVec4(0.0f, 1.0f, 1.0f, 1.0f), 2.0f);
                ImPlot::PlotScatter("Detections", det_x.data(), det_y.data(), int(det_x.size()));
            }
            ImPlot::EndPlot();
        }
        ImGui::EndChild();

        ImGui::BeginChild("##wfall", ImVec2(-1.0f, -1.0f));
        if (ImPlot::BeginPlot("Range Profile History", ImVec2(-1, -1),
                ImPlotFlags_NoLegend | ImPlotFlags_NoMenus)) {
            ImPlot::SetupAxes("Range (km)", nullptr,
                ImPlotAxisFlags_None,
                ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoLabel);
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, RANGE_BINS * KM_PER_BIN, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, WATERFALL_ROWS, ImGuiCond_Always);
            ImPlot::PushColormap(ImPlotColormap_Hot);
            ImPlot::PlotHeatmap("##hm", waterfall_disp.data(), WATERFALL_ROWS, RANGE_BINS,
                0.0, 12.0, nullptr,
                ImPlotPoint(0, 0), ImPlotPoint(RANGE_BINS * KM_PER_BIN, WATERFALL_ROWS));
            ImPlot::PopColormap();
            ImPlot::EndPlot();
        }
        ImGui::EndChild();
        ImGui::EndChild();  // left

        ImGui::SameLine();
        ImGui::BeginChild("##right", ImVec2(0.0f, 0.0f));

        float avail_h  = float(fb_h) - 36.0f;
        float ppi_size = std::min(right_w - 10.0f, avail_h * 0.62f);
        float radius   = ppi_size * 0.5f - 24.0f;
        constexpr int NUM_RINGS = 4;

        ImVec2 canvas_tl = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("##ppi", ImVec2(ppi_size, ppi_size));
        ImVec2 center = ImVec2(canvas_tl.x + ppi_size * 0.5f,
                               canvas_tl.y + ppi_size * 0.5f);

        ImDrawList* dl = ImGui::GetWindowDrawList();

        dl->AddCircleFilled(center, radius, IM_COL32(2, 14, 2, 255));
        for (int i = 1; i <= NUM_RINGS; ++i) {
            float rr = radius * float(i) / float(NUM_RINGS);
            dl->AddCircle(center, rr, IM_COL32(0, 70, 0, 200), 128, 1.0f);
            char km_buf[16];
            snprintf(km_buf, sizeof(km_buf), "%.0fkm",
                float(i) / float(NUM_RINGS) * RANGE_BINS * KM_PER_BIN);
            dl->AddText(ImVec2(center.x + 4.0f, center.y - rr - 1.0f),
                IM_COL32(0, 140, 0, 200), km_buf);
        }

        for (int i = 0; i < 12; ++i) {
            float a = float(i) * TWO_PI / 12.0f;
            dl->AddLine(center, polar_to_screen(center, a, radius),
                IM_COL32(0, 55, 0, 180), 1.0f);
            char deg_buf[8];
            snprintf(deg_buf, sizeof(deg_buf), "%d\xc2\xb0", i * 30);
            ImVec2 lp = polar_to_screen(center, a, radius + 14.0f);
            dl->AddText(ImVec2(lp.x - 10.0f, lp.y - 6.0f),
                IM_COL32(0, 180, 0, 200), deg_buf);
        }
        dl->AddCircle(center, radius, IM_COL32(0, 160, 0, 220), 128, 2.0f);

        constexpr int   STEPS     = 60;
        constexpr float WEDGE_ARC = PI / 3.0f;
        for (int i = STEPS; i >= 1; --i) {
            float t     = float(i) / float(STEPS);
            float a     = sweep_angle - t * WEDGE_ARC;
            float alpha = (1.0f - t) * (1.0f - t) * 0.7f;
            dl->AddLine(center, polar_to_screen(center, a, radius),
                IM_COL32(0, 210, 60, int(alpha * 255)), 1.5f);
        }
        dl->AddLine(center, polar_to_screen(center, sweep_angle, radius),
            IM_COL32(100, 255, 100, 255), 2.5f);

        for (const auto& b : blips) {
            float  r     = b.range * radius;
            ImVec2 pt    = polar_to_screen(center, b.angle, r);
            int    alpha = int(b.intensity * 160);
            dl->AddCircleFilled(pt, 4.0f, velocity_color(b.velocity, alpha));
        }

        for (const auto& t : tracker.tracks()) {
            auto it = track_disp.find(t.id);
            if (it == track_disp.end()) continue;

            float disp_r  = it->second.range;
            float disp_az = it->second.azimuth;
            if (disp_r < 0.0f || disp_r >= RANGE_BINS) continue;  // extrapolated out of range

            float  r_px = disp_r / RANGE_BINS * radius;
            ImVec2 pt   = polar_to_screen(center, disp_az, r_px);

            float hs  = 7.0f;
            ImU32 col = IM_COL32(0, 220, 180, 240);
            dl->AddQuad(
                ImVec2(pt.x,      pt.y - hs),
                ImVec2(pt.x + hs, pt.y     ),
                ImVec2(pt.x,      pt.y + hs),
                ImVec2(pt.x - hs, pt.y     ),
                col, 2.0f);

            // 30-frame look-ahead along the radial direction
            float  r_pred  = std::clamp(disp_r + t.velocity_est * 30.0f,
                                        0.0f, float(RANGE_BINS - 1));
            ImVec2 pt_pred = polar_to_screen(center, disp_az, r_pred / RANGE_BINS * radius);
            dl->AddLine(pt, pt_pred, IM_COL32(0, 220, 180, 140), 1.5f);

            char id_buf[16];
            snprintf(id_buf, sizeof(id_buf), "#%u", t.id);
            dl->AddText(ImVec2(pt.x + 10.0f, pt.y - 6.0f), col, id_buf);
        }

        const struct { float a; const char* lbl; } cards[] = {
            {0.0f, "N"}, {PI*0.5f, "E"}, {PI, "S"}, {PI*1.5f, "W"}
        };
        for (const auto& c : cards) {
            ImVec2 pt = polar_to_screen(center, c.a, radius + 14.0f);
            dl->AddText(ImVec2(pt.x - 4, pt.y - 6), IM_COL32(0, 230, 0, 255), c.lbl);
        }

        ImGui::SeparatorText("Radar Control");
        ImGui::SetNextItemWidth(-1);
        ImGui::SliderFloat("##rpm", &rotation_rpm, 1.0f, 90.0f, "Rotation: %.0f RPM");
        ImGui::Text("Azimuth     %6.1f deg", sweep_angle * (180.0f / PI));
        ImGui::Text("Beamwidth   %6.2f deg", beamwidth * (180.0f / PI));

        ImGui::SeparatorText("Sensor Status");
        ImGui::Text("Detections  %zu", det_x.size());
        ImGui::Text("Blips       %zu", blips.size());
        ImGui::Text("Tracks      %zu", tracker.tracks().size());

        ImGui::SeparatorText("Track List");
        if (ImGui::BeginTable("##tracks", 5,
                ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                ImGuiTableFlags_SizingFixedFit)) {
            ImGui::TableSetupColumn("ID",    ImGuiTableColumnFlags_WidthFixed, 32.0f);
            ImGui::TableSetupColumn("Range", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableSetupColumn("Az",    ImGuiTableColumnFlags_WidthFixed, 44.0f);
            ImGui::TableSetupColumn("V r/s", ImGuiTableColumnFlags_WidthFixed, 54.0f);
            ImGui::TableSetupColumn("Age",   ImGuiTableColumnFlags_WidthFixed, 32.0f);
            ImGui::TableHeadersRow();

            for (const auto& t : tracker.tracks()) {
                float v_ms = t.velocity_est * MS_PER_VEL;  // signed m/s
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::Text("%u",    t.id);
                ImGui::TableSetColumnIndex(1); ImGui::Text("%.1fkm", t.range_bin * KM_PER_BIN);
                ImGui::TableSetColumnIndex(2); ImGui::Text("%.0f\xc2\xb0",
                    std::fmod(t.azimuth * (180.0f / PI) + 360.0f, 360.0f));
                ImGui::TableSetColumnIndex(3);
                if (t.age > 1) {
                    // Red = receding, cyan = approaching
                    ImVec4 vc = v_ms > 0 ? ImVec4(1,0.4f,0.4f,1) : ImVec4(0.4f,1,1,1);
                    ImGui::TextColored(vc, "%+.0fm/s", v_ms);
                } else {
                    ImGui::TextDisabled("---");
                }
                ImGui::TableSetColumnIndex(4); ImGui::Text("%d", t.age);
            }
            ImGui::EndTable();
        }

        ImGui::SeparatorText("Blip Velocity");
        ImDrawList* wdl   = ImGui::GetWindowDrawList();
        ImVec2      leg_p = ImGui::GetCursorScreenPos();
        float       leg_w = right_w - 20.0f;
        for (int px = 0; px < int(leg_w); ++px) {
            float t = float(px) / leg_w;
            int   r = int(255 * std::min(t * 2.0f, 1.0f));
            int   g = int(255 * std::min((1.0f - t) * 2.0f, 1.0f));
            wdl->AddRectFilled(ImVec2(leg_p.x + px, leg_p.y),
                               ImVec2(leg_p.x + px + 1, leg_p.y + 10),
                               IM_COL32(r, g, 0, 200));
        }
        ImGui::Dummy(ImVec2(leg_w, 10.0f));
        ImGui::Text("0");
        ImGui::SameLine(leg_w - 50.0f);
        ImGui::Text("%.0f m/s", MAX_VELOCITY * MS_PER_VEL);

        ImGui::EndChild();  // right
        ImGui::End();

        ImGui::Render();
        glViewport(0, 0, fb_w, fb_h);
        glClearColor(0.05f, 0.05f, 0.06f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);

        // Manual FPS cap — vsync alone is unreliable on Windows.
        auto   fe  = std::chrono::high_resolution_clock::now();
        double fms = std::chrono::duration<double, std::milli>(fe - t_last).count();
        constexpr double TGT = 1000.0 / 60.0;
        if (fms < TGT)
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(TGT - fms));
    }

    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
