// F1 Hotlap - a lightweight top-down time-trial racer.
// One source file, one dependency (SDL2). Drive, stay inside track limits,
// and beat your best lap. Best lap + ghost car are saved next to the binary.

#include <SDL.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr float DT = 1.0f / 120.0f;  // fixed physics step
constexpr int SCREEN_W = 1280;
constexpr int SCREEN_H = 720;

// World units are pixels; roughly 10 px = 1 m.
constexpr float HALF_W = 70.0f;     // half the asphalt width
constexpr float KERB_W = 12.0f;     // kerbs still count as track
constexpr float RUNOFF_W = 110.0f;  // grass between kerb and tyre wall
constexpr float LIMIT_TOL = 10.0f;  // how far the car centre may pass the kerb
constexpr float WALL_D = HALF_W + KERB_W + RUNOFF_W;

// Car tuning.
constexpr float ENGINE = 640.0f;
constexpr float TOP_SPEED = 830.0f;  // ~300 km/h
constexpr float GRASS_TOP = 260.0f;
constexpr float REVERSE_SPEED = 150.0f;
constexpr float BRAKE = 1350.0f;
constexpr float MAX_LAT = 1500.0f;  // lateral grip limit
constexpr float WHEELBASE = 32.0f;

struct Vec2 {
    float x = 0, y = 0;
    Vec2 operator+(Vec2 o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(Vec2 o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
};

float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
float cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
float length(Vec2 v) { return std::sqrt(dot(v, v)); }
Vec2 normalized(Vec2 v) {
    float l = length(v);
    return l > 1e-6f ? v * (1.0f / l) : Vec2{1, 0};
}
float signOf(float v) { return v < 0 ? -1.0f : 1.0f; }

// ---------------------------------------------------------------------------
// Track: a closed Catmull-Rom spline sampled into a dense centre line.

struct TrackHit {
    int seg = 0;
    float dist = 0;
    Vec2 closest;
};

struct Track {
    std::vector<Vec2> pts, normals;
    std::vector<float> curvature;
    float length = 0;

    void build(const std::vector<Vec2>& ctrl) {
        const int C = int(ctrl.size());
        for (int i = 0; i < C; ++i) {
            Vec2 p0 = ctrl[(i - 1 + C) % C], p1 = ctrl[i];
            Vec2 p2 = ctrl[(i + 1) % C], p3 = ctrl[(i + 2) % C];
            int steps = std::max(4, int(::length(p2 - p1) / 12.0f));
            for (int s = 0; s < steps; ++s) {
                float t = float(s) / steps, t2 = t * t, t3 = t2 * t;
                Vec2 p = (p1 * 2.0f + (p2 - p0) * t +
                          (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2 +
                          (p1 * 3.0f - p0 - p2 * 3.0f + p3) * t3) *
                         0.5f;
                pts.push_back(p);
            }
        }
        const int N = int(pts.size());
        normals.resize(N);
        curvature.assign(N, 0.0f);
        std::vector<float> raw(N);
        for (int i = 0; i < N; ++i) {
            Vec2 prev = pts[(i - 1 + N) % N], next = pts[(i + 1) % N];
            Vec2 t = normalized(next - prev);
            normals[i] = {-t.y, t.x};
            length += ::length(next - pts[i]);
            Vec2 a = pts[i] - prev, b = next - pts[i];
            float turn = std::fabs(std::atan2(cross(a, b), dot(a, b)));
            raw[i] = turn / (0.5f * (::length(a) + ::length(b)));
        }
        for (int i = 0; i < N; ++i) {
            for (int k = -4; k <= 4; ++k) curvature[i] += raw[((i + k) % N + N) % N];
            curvature[i] /= 9.0f;
        }
    }

    int size() const { return int(pts.size()); }
    Vec2 tangent(int i) const { return {normals[i].y, -normals[i].x}; }

    // Closest point on the centre line. window < 0 searches the whole track,
    // otherwise only +-window segments around hint (keeps it cheap and stable).
    TrackHit nearest(Vec2 p, int hint, int window) const {
        const int N = size();
        TrackHit best;
        best.dist = 1e30f;
        int count = window < 0 ? N : 2 * window + 1;
        int start = window < 0 ? 0 : hint - window;
        for (int k = 0; k < count; ++k) {
            int i = ((start + k) % N + N) % N, j = (i + 1) % N;
            Vec2 ab = pts[j] - pts[i];
            float t = std::clamp(dot(p - pts[i], ab) / std::max(dot(ab, ab), 1e-6f), 0.0f, 1.0f);
            Vec2 c = pts[i] + ab * t;
            float d = ::length(p - c);
            if (d < best.dist) best = {i, d, c};
        }
        return best;
    }
};

std::vector<Vec2> circuitLayout() {
    const float raw[][2] = {
        {300, 0},     {1200, 0},    {1800, 0},    {2300, 100},  {2600, 400},  {2600, 800},
        {2350, 1050}, {2000, 1000}, {1700, 800},  {1400, 850},  {1250, 1100}, {1400, 1400},
        {1900, 1500}, {2500, 1500}, {2900, 1700}, {2900, 2100}, {2500, 2300}, {1500, 2300},
        {800, 2300},  {400, 2100},  {500, 1800},  {300, 1500},  {-200, 1400}, {-500, 1100},
        {-500, 500},  {-350, 150},
    };
    std::vector<Vec2> out;
    for (auto& p : raw) out.push_back({p[0] * 1.6f, p[1] * 1.6f});
    return out;
}

// ---------------------------------------------------------------------------
// Game state and physics (no rendering here, so it can run headless).

struct Input {
    float throttle = 0, brake = 0, steer = 0;
    bool handbrake = false;
};

struct Car {
    Vec2 pos, vel;
    float heading = 0, steer = 0, slip = 0;
};

struct GhostSample {
    float x, y, h;
};

struct LapEntry {
    int number;
    float time;
    bool valid, best;
};

constexpr uint32_t SAVE_MAGIC = 0x4C484631;  // "1FHL"
constexpr uint32_t SAVE_VERSION = 1;
constexpr int GRID_OFFSET = 40;  // samples behind the line for the run-up
constexpr int CROSS_BAND = 60;   // samples either side of a line that count as "at" it
constexpr size_t MAX_SKIDS = 6000;

const SDL_Color WHITE{240, 240, 240, 255};
const SDL_Color RED{235, 70, 60, 255};
const SDL_Color GREEN{90, 220, 110, 255};
const SDL_Color GOLD{255, 200, 40, 255};
const SDL_Color PURPLE{200, 110, 255, 255};

std::string fmtTime(float s) {
    if (s < 0) return "-:--.---";
    int ms = int(std::lround(s * 1000.0f));
    char buf[32];
    std::snprintf(buf, sizeof buf, "%d:%02d.%03d", ms / 60000, (ms / 1000) % 60, ms % 1000);
    return buf;
}

std::string fmtDelta(float d) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%c%.3f", d < 0 ? '-' : '+', std::fabs(d));
    return buf;
}

struct Game {
    Track track;
    Car car;
    int idx = 0;
    int cpIdx[2] = {};
    bool lapStarted = false, lapValid = true, offTrack = false, onGrass = false;
    int nextCp = 3, lapSteps = 0, lapNumber = 0, splitsDone = 0;
    float splits[3] = {}, lastSplits[3] = {};
    float bestTime = -1, bestSectors[3] = {}, lastLap = -1;
    std::vector<GhostSample> bestGhost, curGhost;
    std::vector<LapEntry> history;
    std::vector<Vec2> skids;
    size_t skidHead = 0;
    long stepCount = 0;
    std::string message;
    float messageTimer = 0;
    SDL_Color messageColor = WHITE;
    std::string savePath;  // empty = don't persist
    Input lastInput;
    float steerSensitivity = 1.0f;  // scales the maximum steering angle
    float steerSpeed = 1.0f;        // scales how fast the wheel turns

    void init() {
        track.build(circuitLayout());
        cpIdx[0] = track.size() / 3;
        cpIdx[1] = 2 * track.size() / 3;
        load();
        resetToGrid();
    }

    void flash(const std::string& text, SDL_Color c, float secs = 2.5f) {
        message = text;
        messageColor = c;
        messageTimer = secs;
    }

    void placeAt(int i) {
        car = Car{};
        car.pos = track.pts[i];
        Vec2 t = track.tangent(i);
        car.heading = std::atan2(t.y, t.x);
        idx = i;
    }

    void resetToGrid() {
        placeAt(track.size() - GRID_OFFSET);
        lapStarted = false;
        lapValid = true;
        nextCp = 3;
        lapSteps = 0;
        splitsDone = 0;
        curGhost.clear();
        skids.clear();
        skidHead = 0;
        flash("CROSS THE LINE TO START YOUR LAP", WHITE, 4.0f);
    }

    float sectorTime(const float* sp, int k) const { return sp[k] - (k ? sp[k - 1] : 0.0f); }

    void startLap() {
        lapStarted = true;
        lapValid = !offTrack;
        lapSteps = 0;
        nextCp = 1;
        splitsDone = 0;
        curGhost.clear();
        curGhost.reserve(12000);
        ++lapNumber;
    }

    void finishLap() {
        float t = lapSteps * DT;
        splits[2] = t;
        std::copy(splits, splits + 3, lastSplits);
        lastLap = t;
        LapEntry e{lapNumber, t, lapValid, false};
        if (lapValid && (bestTime < 0 || t < bestTime)) {
            float prev = bestTime;
            bestTime = t;
            for (int k = 0; k < 3; ++k) bestSectors[k] = sectorTime(splits, k);
            bestGhost = curGhost;
            e.best = true;
            save();
            flash(prev < 0 ? "FIRST LAP RECORD SET " + fmtTime(t)
                           : "NEW BEST LAP! " + fmtDelta(t - prev),
                  PURPLE, 3.5f);
        } else if (!lapValid) {
            flash("LAP " + fmtTime(t) + " NOT COUNTED", RED);
        } else {
            flash("LAP " + fmtTime(t) + "  " + fmtDelta(t - bestTime), WHITE);
        }
        history.push_back(e);
        startLap();
    }

    void addSkid(Vec2 p) {
        if (skids.size() < MAX_SKIDS) skids.push_back(p);
        else skids[skidHead] = p;
        skidHead = (skidHead + 1) % MAX_SKIDS;
    }

    void step(const Input& in) {
        lastInput = in;
        ++stepCount;
        if (messageTimer > 0) messageTimer -= DT;

        Vec2 fwd{std::cos(car.heading), std::sin(car.heading)}, right{-fwd.y, fwd.x};
        float vf = dot(car.vel, fwd), vr = dot(car.vel, right);

        TrackHit here = track.nearest(car.pos, idx, 40);
        onGrass = here.dist > HALF_W + KERB_W;
        float grip = onGrass ? 0.5f : 1.0f;
        float top = onGrass ? GRASS_TOP : TOP_SPEED;

        // The steering wheel turns at a finite rate so keyboard input feels smooth.
        float steerRate = (std::fabs(in.steer) < std::fabs(car.steer) ? 7.0f : 4.5f) * steerSpeed;
        car.steer += std::clamp(in.steer - car.steer, -steerRate * DT, steerRate * DT);

        // Longitudinal: engine, brakes, reverse, drag.
        if (in.throttle > 0 && vf < top)
            vf += in.throttle * ENGINE * (1.0f - std::max(vf, 0.0f) / top) * DT;
        if (in.brake > 0) {
            if (vf > 5) vf = std::max(0.0f, vf - in.brake * BRAKE * (onGrass ? 0.6f : 1.0f) * DT);
            else vf = std::max(-REVERSE_SPEED, vf - in.brake * 300.0f * DT);
        }
        if (in.handbrake) vf -= signOf(vf) * std::min(std::fabs(vf), 350.0f * DT);
        vf -= vf * (0.10f + (onGrass ? 1.4f : 0.0f)) * DT;
        if (in.throttle == 0 && in.brake == 0) vf -= signOf(vf) * std::min(std::fabs(vf), 40.0f * DT);

        // Yaw from a simple bicycle model, capped by available grip.
        float steerAngle = car.steer * 0.62f * steerSensitivity / (1.0f + std::fabs(vf) / 320.0f);
        float yaw = vf / WHEELBASE * std::tan(steerAngle);
        float latLimit = MAX_LAT * grip * (in.handbrake ? 0.5f : 1.0f);
        float yawCap = latLimit * (in.handbrake ? 2.4f : 1.3f) / std::max(std::fabs(vf), 60.0f);
        yaw = std::clamp(yaw, -yawCap, yawCap);
        car.heading += yaw * DT;

        // Tyres kill sideways velocity, but only up to the grip limit -> slides.
        float cap = latLimit * DT;
        vr += std::clamp(-vr * (1.0f - std::exp(-28.0f * DT)), -cap, cap);
        vf -= signOf(vf) * std::min(std::fabs(vf), std::fabs(vr) * 0.6f * DT);
        car.slip = vr;

        // Velocity is rebuilt in the pre-turn frame, so turning creates slip next step.
        car.vel = fwd * vf + right * vr;
        car.pos = car.pos + car.vel * DT;

        if ((std::fabs(vr) > 110.0f || (in.handbrake && std::fabs(vf) > 120.0f)) && stepCount % 2 == 0) {
            Vec2 nf{std::cos(car.heading), std::sin(car.heading)}, nr{-nf.y, nf.x};
            addSkid(car.pos - nf * 14.0f + nr * 11.0f);
            addSkid(car.pos - nf * 14.0f - nr * 11.0f);
        }

        // Tyre wall.
        TrackHit after = track.nearest(car.pos, here.seg, 40);
        if (after.dist > WALL_D) {
            Vec2 n = normalized(car.pos - after.closest);
            car.pos = after.closest + n * WALL_D;
            float vn = dot(car.vel, n);
            if (vn > 0) car.vel = car.vel - n * (vn * 1.3f);
            car.vel = car.vel * 0.55f;
        }

        const int N = track.size();
        int prevIdx = idx;
        idx = after.seg;
        offTrack = after.dist > HALF_W + KERB_W + LIMIT_TOL;
        bool crossedFwd = prevIdx >= N - CROSS_BAND && idx < CROSS_BAND;
        bool crossedBack = prevIdx < CROSS_BAND && idx >= N - CROSS_BAND;

        if (lapStarted) {
            ++lapSteps;
            curGhost.push_back({car.pos.x, car.pos.y, car.heading});
            if (offTrack && lapValid) {
                lapValid = false;
                flash("TRACK LIMITS - LAP INVALID", RED);
            }
            for (int k = 0; k < 2; ++k) {
                if (nextCp == k + 1 && idx >= cpIdx[k] && idx < cpIdx[k] + CROSS_BAND) {
                    splits[k] = lapSteps * DT;
                    splitsDone = k + 1;
                    nextCp = k + 2;
                }
            }
        }
        if (crossedFwd) {
            if (!lapStarted) startLap();
            else if (nextCp == 3) finishLap();
        }
        if (crossedBack && lapStarted) {
            lapValid = false;
            nextCp = 3;
            flash("WRONG WAY", RED);
        }
    }

    uint32_t trackSignature() const { return uint32_t(track.size()) * 100003u + uint32_t(track.length); }

    void save() const {
        if (savePath.empty()) return;
        FILE* f = std::fopen(savePath.c_str(), "wb");
        if (!f) return;
        uint32_t header[4] = {SAVE_MAGIC, SAVE_VERSION, trackSignature(), uint32_t(bestGhost.size())};
        std::fwrite(header, sizeof header, 1, f);
        std::fwrite(&bestTime, sizeof bestTime, 1, f);
        std::fwrite(bestSectors, sizeof bestSectors, 1, f);
        std::fwrite(bestGhost.data(), sizeof(GhostSample), bestGhost.size(), f);
        std::fclose(f);
    }

    void load() {
        if (savePath.empty()) return;
        FILE* f = std::fopen(savePath.c_str(), "rb");
        if (!f) return;
        uint32_t header[4];
        bool ok = std::fread(header, sizeof header, 1, f) == 1 && header[0] == SAVE_MAGIC &&
                  header[1] == SAVE_VERSION && header[2] == trackSignature() && header[3] < 1000000;
        if (ok) {
            bestGhost.resize(header[3]);
            ok = std::fread(&bestTime, sizeof bestTime, 1, f) == 1 &&
                 std::fread(bestSectors, sizeof bestSectors, 1, f) == 1 &&
                 std::fread(bestGhost.data(), sizeof(GhostSample), header[3], f) == header[3];
        }
        std::fclose(f);
        if (!ok) {
            bestTime = -1;
            bestGhost.clear();
        }
    }

    void resetBest() {
        bestTime = -1;
        std::fill(bestSectors, bestSectors + 3, 0.0f);
        bestGhost.clear();
        for (auto& e : history) e.best = false;
        if (!savePath.empty()) std::remove(savePath.c_str());
    }
};

// Simple pure-pursuit driver used by --selftest to check the game is lappable.
Input autopilot(const Game& g) {
    const Track& t = g.track;
    const int N = t.size();
    float speed = length(g.car.vel);
    Vec2 target = t.pts[(g.idx + 6 + int(speed * 0.02f)) % N];
    Vec2 d = target - g.car.pos;
    float diff = std::remainder(std::atan2(d.y, d.x) - g.car.heading, 2.0f * float(M_PI));

    float limit = 1e9f, dist = 0;
    for (int k = 0; k < 160; ++k) {
        int j = (g.idx + k) % N;
        dist += length(t.pts[(j + 1) % N] - t.pts[j]);
        float vmax = t.curvature[j] > 1e-5f ? std::sqrt(MAX_LAT * 0.7f / t.curvature[j]) : 1e9f;
        limit = std::min(limit, std::sqrt(vmax * vmax + 2.0f * BRAKE * 0.6f * dist));
    }
    Vec2 fwd{std::cos(g.car.heading), std::sin(g.car.heading)};
    float vf = dot(g.car.vel, fwd);
    Input in;
    in.steer = std::clamp(diff * 3.0f, -1.0f, 1.0f);
    in.throttle = vf < limit ? 1.0f : 0.0f;
    in.brake = vf > limit + 15.0f ? 1.0f : 0.0f;
    return in;
}

int runSelfTest() {
    Game g;
    g.init();
    const Track& t = g.track;
    const int N = t.size();
    std::printf("track: %d samples, %.0f px long\n", N, t.length);

    // Sections of track that are far apart along the lap must not overlap.
    float cum = 0;
    std::vector<float> arc(N);
    for (int i = 0; i < N; ++i) {
        arc[i] = cum;
        cum += length(t.pts[(i + 1) % N] - t.pts[i]);
    }
    float minGap = 1e9f;
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j) {
            float along = std::min(arc[j] - arc[i], cum - (arc[j] - arc[i]));
            if (along > 1200.0f) minGap = std::min(minGap, length(t.pts[i] - t.pts[j]));
        }
    std::printf("closest separate sections: %.0f px (need > %.0f)\n", minGap, 2 * WALL_D);
    bool ok = minGap > 2 * WALL_D;

    for (long s = 0; s < 120L * 600 && g.history.size() < 3; ++s) g.step(autopilot(g));
    for (auto& e : g.history)
        std::printf("lap %d: %s %s\n", e.number, fmtTime(e.time).c_str(), e.valid ? "valid" : "INVALID");
    bool anyValid = false;
    for (auto& e : g.history) anyValid |= e.valid;
    ok = ok && g.history.size() == 3 && anyValid && g.bestGhost.size() > 0;
    std::printf("best: %s, ghost samples: %zu\n%s\n", fmtTime(g.bestTime).c_str(), g.bestGhost.size(),
                ok ? "SELFTEST PASS" : "SELFTEST FAIL");
    return ok ? 0 : 1;
}

// ---------------------------------------------------------------------------
// Rendering.

// 3x5 pixel font, rows top to bottom.
const char* glyph(char c) {
    switch (c) {
        case '0': return "111101101101111";
        case '1': return "010110010010111";
        case '2': return "111001111100111";
        case '3': return "111001111001111";
        case '4': return "101101111001001";
        case '5': return "111100111001111";
        case '6': return "111100111101111";
        case '7': return "111001001001001";
        case '8': return "111101111101111";
        case '9': return "111101111001111";
        case 'A': return "010101111101101";
        case 'B': return "110101110101110";
        case 'C': return "011100100100011";
        case 'D': return "110101101101110";
        case 'E': return "111100110100111";
        case 'F': return "111100110100100";
        case 'G': return "011100101101011";
        case 'H': return "101101111101101";
        case 'I': return "111010010010111";
        case 'J': return "001001001101010";
        case 'K': return "101101110101101";
        case 'L': return "100100100100111";
        case 'M': return "101111111101101";
        case 'N': return "110101101101101";
        case 'O': return "010101101101010";
        case 'P': return "110101110100100";
        case 'Q': return "010101101110011";
        case 'R': return "110101110101101";
        case 'S': return "011100010001110";
        case 'T': return "111010010010010";
        case 'U': return "101101101101111";
        case 'V': return "101101101101010";
        case 'W': return "101101111111101";
        case 'X': return "101101010101101";
        case 'Y': return "101101010010010";
        case 'Z': return "111001010100111";
        case ':': return "000010000010000";
        case '.': return "000000000000010";
        case '-': return "000000111000000";
        case '+': return "000010111010000";
        case '/': return "001001010100100";
        case '!': return "010010010000010";
        case '%': return "101001010100101";
        case '<': return "001010100010001";
        case '>': return "100010001010100";
        default: return "000000000000000";
    }
}

float textWidth(const std::string& s, float scale) { return s.empty() ? 0 : (s.size() * 4 - 1) * scale; }

// align: 0 = left, 1 = centre, 2 = right
void drawText(SDL_Renderer* r, float x, float y, float scale, const std::string& s, SDL_Color c, int align = 0) {
    if (align == 1) x -= textWidth(s, scale) / 2;
    if (align == 2) x -= textWidth(s, scale);
    std::vector<SDL_FRect> rects;
    for (char ch : s) {
        const char* g = glyph(char(std::toupper(static_cast<unsigned char>(ch))));
        for (int i = 0; i < 15; ++i)
            if (g[i] == '1') rects.push_back({x + (i % 3) * scale, y + (i / 3) * scale, scale, scale});
        x += 4 * scale;
    }
    SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a);
    SDL_RenderFillRectsF(r, rects.data(), int(rects.size()));
}

void fillRect(SDL_Renderer* r, float x, float y, float w, float h, SDL_Color c) {
    SDL_FRect rect{x, y, w, h};
    SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a);
    SDL_RenderFillRectF(r, &rect);
}

struct View {
    Vec2 cam;
    float zoom = 1;
    Vec2 toScreen(Vec2 p) const {
        return {(p.x - cam.x) * zoom + SCREEN_W * 0.5f, (p.y - cam.y) * zoom + SCREEN_H * 0.5f};
    }
};

void pushQuad(std::vector<SDL_Vertex>& out, Vec2 a, Vec2 b, Vec2 c, Vec2 d, SDL_Color col) {
    for (Vec2 p : {a, b, c, a, c, d}) out.push_back({{p.x, p.y}, col, {0, 0}});
}

// Rotated box in the car's local frame (x forward, y right).
void pushBox(std::vector<SDL_Vertex>& out, const View& v, Vec2 pos, float heading, float lx, float ly,
             float hl, float hw, SDL_Color col) {
    Vec2 f{std::cos(heading), std::sin(heading)}, r{-f.y, f.x};
    auto at = [&](float x, float y) { return v.toScreen(pos + f * (lx + x) + r * (ly + y)); };
    pushQuad(out, at(-hl, -hw), at(hl, -hw), at(hl, hw), at(-hl, hw), col);
}

void pushCar(std::vector<SDL_Vertex>& out, const View& v, Vec2 pos, float heading, float steer, SDL_Color body,
             Uint8 alpha) {
    SDL_Color tyre{25, 25, 28, alpha}, dark{30, 30, 40, alpha}, helmet{255, 210, 40, alpha};
    body.a = alpha;
    pushBox(out, v, pos, heading, -14, -11, 5.5f, 3.8f, tyre);
    pushBox(out, v, pos, heading, -14, 11, 5.5f, 3.8f, tyre);
    float fh = heading + steer * 0.45f;
    Vec2 f{std::cos(heading), std::sin(heading)}, r{-f.y, f.x};
    pushBox(out, v, pos + f * 14.0f - r * 10.5f, fh, 0, 0, 4.5f, 3.2f, tyre);
    pushBox(out, v, pos + f * 14.0f + r * 10.5f, fh, 0, 0, 4.5f, 3.2f, tyre);
    pushBox(out, v, pos, heading, 1, 0, 21, 3.5f, body);        // chassis + nose
    pushBox(out, v, pos, heading, -5, 0, 9, 7.5f, body);        // sidepods
    pushBox(out, v, pos, heading, 22, 0, 2, 11.5f, body);       // front wing
    pushBox(out, v, pos, heading, -21, 0, 2.5f, 9, dark);       // rear wing
    pushBox(out, v, pos, heading, 1, 0, 4.5f, 2.8f, dark);      // cockpit
    pushBox(out, v, pos, heading, 0, 0, 2.2f, 2.2f, helmet);    // driver
}

struct WorldQuad {
    Vec2 v[4];
    SDL_Color c;
};

std::vector<WorldQuad> buildTrackQuads(const Game& g) {
    const Track& t = g.track;
    const int N = t.size();
    std::vector<WorldQuad> q;
    // A band between two offsets from the centre line. Folded quads (inside of
    // tight corners at large offsets) are skipped so they don't draw garbage.
    auto band = [&](float o1, float o2, auto colorAt, bool skipFolded) {
        for (int i = 0; i < N; ++i) {
            int j = (i + 1) % N;
            Vec2 a = t.pts[i] + t.normals[i] * o1, b = t.pts[i] + t.normals[i] * o2;
            Vec2 c = t.pts[j] + t.normals[j] * o2, d = t.pts[j] + t.normals[j] * o1;
            Vec2 tan = t.tangent(i);
            if (skipFolded && (dot(c - b, tan) <= 0 || dot(d - a, tan) <= 0)) continue;
            q.push_back({{a, b, c, d}, colorAt(i)});
        }
    };
    auto solid = [](SDL_Color c) { return [c](int) { return c; }; };
    auto barrier = [](int i) { return (i / 4) % 4 == 0 ? SDL_Color{200, 50, 45, 255} : SDL_Color{45, 45, 50, 255}; };
    auto kerb = [&](int i) {
        if (t.curvature[i] < 0.0012f) return SDL_Color{230, 230, 230, 255};
        return (i / 2) % 2 ? SDL_Color{215, 40, 40, 255} : SDL_Color{240, 240, 240, 255};
    };

    band(-WALL_D, WALL_D, solid({78, 150, 64, 255}), false);
    band(WALL_D, WALL_D + 10, barrier, true);
    band(-WALL_D - 10, -WALL_D, barrier, true);
    band(HALF_W, HALF_W + KERB_W, kerb, false);
    band(-HALF_W - KERB_W, -HALF_W, kerb, false);
    band(-HALF_W, HALF_W, solid({72, 72, 78, 255}), false);

    auto across = [&](int i, float along0, float along1, float o1, float o2, SDL_Color c) {
        Vec2 tan = t.tangent(i), n = t.normals[i], p = t.pts[i];
        q.push_back({{p + tan * along0 + n * o1, p + tan * along1 + n * o1, p + tan * along1 + n * o2,
                      p + tan * along0 + n * o2},
                     c});
    };
    for (int k = 0; k < 2; ++k) across(g.cpIdx[k], -2, 2, -HALF_W, HALF_W, {250, 210, 60, 140});
    const int cols = 10;
    const float cw = 2 * HALF_W / cols;
    for (int row = 0; row < 2; ++row)
        for (int col = 0; col < cols; ++col) {
            SDL_Color c = (row + col) % 2 ? SDL_Color{20, 20, 20, 255} : SDL_Color{245, 245, 245, 255};
            across(0, -8.0f + row * 8, row * 8.0f, -HALF_W + col * cw, -HALF_W + (col + 1) * cw, c);
        }
    return q;
}

struct Minimap {
    float x, y, scale;
    Vec2 origin;
    std::vector<SDL_FPoint> line;

    void build(const Track& t, float bx, float by, float size) {
        Vec2 lo{1e9f, 1e9f}, hi{-1e9f, -1e9f};
        for (Vec2 p : t.pts) {
            lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)};
            hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
        }
        scale = size / std::max(hi.x - lo.x, hi.y - lo.y);
        origin = lo;
        x = bx + (size - (hi.x - lo.x) * scale) / 2;
        y = by + (size - (hi.y - lo.y) * scale) / 2;
        for (Vec2 p : t.pts) line.push_back(map(p));
        line.push_back(line.front());
    }
    SDL_FPoint map(Vec2 p) const { return {x + (p.x - origin.x) * scale, y + (p.y - origin.y) * scale}; }
};

// ---------------------------------------------------------------------------
// Settings (saved to settings.cfg next to the binary) and the in-game menu.

struct Settings {
    int steerSensitivity = 100, steerSpeed = 100, padDeadzone = 12, cameraZoom = 100, lookAhead = 100;
    int speedZoom = 1, mph = 0, ghost = 1, minimap = 1, skidMarks = 1, fullscreen = 0, vsync = 1;
};

enum class Kind { Percent, Toggle, Units, ResetBest, Resume, Quit };

struct MenuEntry {
    const char* label;
    const char* key;  // name in settings.cfg
    int Settings::*field;
    int lo, hi, step;
    Kind kind;
};

const MenuEntry MENU[] = {
    {"STEERING SENSITIVITY", "steer_sensitivity", &Settings::steerSensitivity, 50, 150, 5, Kind::Percent},
    {"STEERING SPEED", "steer_speed", &Settings::steerSpeed, 50, 200, 10, Kind::Percent},
    {"GAMEPAD DEADZONE", "pad_deadzone", &Settings::padDeadzone, 0, 30, 2, Kind::Percent},
    {"CAMERA ZOOM", "camera_zoom", &Settings::cameraZoom, 60, 150, 5, Kind::Percent},
    {"CAMERA LOOK-AHEAD", "look_ahead", &Settings::lookAhead, 0, 150, 10, Kind::Percent},
    {"ZOOM OUT WITH SPEED", "speed_zoom", &Settings::speedZoom, 0, 1, 1, Kind::Toggle},
    {"SPEED UNITS", "mph", &Settings::mph, 0, 1, 1, Kind::Units},
    {"GHOST CAR", "ghost", &Settings::ghost, 0, 1, 1, Kind::Toggle},
    {"MINIMAP", "minimap", &Settings::minimap, 0, 1, 1, Kind::Toggle},
    {"SKID MARKS", "skid_marks", &Settings::skidMarks, 0, 1, 1, Kind::Toggle},
    {"FULLSCREEN", "fullscreen", &Settings::fullscreen, 0, 1, 1, Kind::Toggle},
    {"VSYNC", "vsync", &Settings::vsync, 0, 1, 1, Kind::Toggle},
    {"RESET BEST LAP", nullptr, nullptr, 0, 0, 0, Kind::ResetBest},
    {"RESUME", nullptr, nullptr, 0, 0, 0, Kind::Resume},
    {"QUIT GAME", nullptr, nullptr, 0, 0, 0, Kind::Quit},
};
constexpr int MENU_COUNT = int(sizeof MENU / sizeof MENU[0]);

void saveSettings(const Settings& s, const std::string& path) {
    if (path.empty()) return;
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    for (const MenuEntry& m : MENU)
        if (m.key) std::fprintf(f, "%s=%d\n", m.key, s.*m.field);
    std::fclose(f);
}

void loadSettings(Settings& s, const std::string& path) {
    FILE* f = path.empty() ? nullptr : std::fopen(path.c_str(), "r");
    if (!f) return;
    char key[64];
    int value;
    while (std::fscanf(f, " %63[^=]=%d", key, &value) == 2)
        for (const MenuEntry& m : MENU)
            if (m.key && std::strcmp(m.key, key) == 0) s.*m.field = std::clamp(value, m.lo, m.hi);
    std::fclose(f);
}

struct Menu {
    bool open = false;
    int sel = 0;
    bool confirmReset = false;
};

// Menu geometry, shared by drawing and mouse hit-testing.
constexpr float MENU_X = 270, MENU_W = 740, MENU_Y = 50, MENU_ROW = 32;
constexpr float MENU_VALUE_RIGHT = MENU_X + MENU_W - 30;
float menuRowY(int i) { return MENU_Y + 80 + i * MENU_ROW; }

std::string menuValue(const MenuEntry& m, const Settings& s, const Menu& menu) {
    switch (m.kind) {
        case Kind::Percent: return "< " + std::to_string(s.*m.field) + "% >";
        case Kind::Toggle: return s.*m.field ? "ON" : "OFF";
        case Kind::Units: return s.*m.field ? "MPH" : "KM/H";
        case Kind::ResetBest: return menu.confirmReset ? "CLICK OR ENTER AGAIN" : "";
        default: return "";
    }
}

int menuRowAt(float x, float y) {
    if (x < MENU_X + 10 || x > MENU_X + MENU_W - 10) return -1;
    float top = menuRowY(0) - 7;
    if (y < top) return -1;
    int i = int((y - top) / MENU_ROW);
    return i < MENU_COUNT ? i : -1;
}

// For a percentage row: -1 if x is on the "<" arrow, +1 on ">", 0 elsewhere.
int menuArrowAt(const MenuEntry& m, const Settings& s, const Menu& menu, float x) {
    if (m.kind != Kind::Percent) return 0;
    float left = MENU_VALUE_RIGHT - textWidth(menuValue(m, s, menu), 3);
    if (std::fabs(x - (left + 4)) < 22) return -1;
    if (x > MENU_VALUE_RIGHT - 30) return 1;
    return 0;
}

void drawMenu(SDL_Renderer* r, const Menu& menu, const Settings& s) {
    fillRect(r, 0, 0, SCREEN_W, SCREEN_H, {0, 0, 0, 110});
    fillRect(r, MENU_X, MENU_Y, MENU_W, 80 + MENU_COUNT * MENU_ROW + 50, {10, 10, 16, 235});
    drawText(r, SCREEN_W / 2, MENU_Y + 22, 5, "SETTINGS", GOLD, 1);
    for (int i = 0; i < MENU_COUNT; ++i) {
        const MenuEntry& m = MENU[i];
        float y = menuRowY(i);
        bool sel = i == menu.sel;
        if (sel) fillRect(r, MENU_X + 10, y - 7, MENU_W - 20, MENU_ROW - 2, {255, 255, 255, 28});
        drawText(r, MENU_X + 30, y, 3, m.label, sel ? GOLD : WHITE);
        SDL_Color c = m.kind == Kind::ResetBest ? RED
                      : m.kind == Kind::Toggle && !(s.*m.field) ? SDL_Color{150, 150, 150, 255}
                                                                : WHITE;
        drawText(r, MENU_VALUE_RIGHT, y, 3, menuValue(m, s, menu), c, 2);
    }
    drawText(r, SCREEN_W / 2, menuRowY(MENU_COUNT) - 18, 2,
             "CLICK < > OR SCROLL TO CHANGE   CLICK TO TOGGLE   KEYS: ARROWS + ENTER   ESC CLOSE",
             {170, 170, 170, 255}, 1);
}

void render(SDL_Renderer* r, const Game& g, const View& view, const std::vector<WorldQuad>& quads,
            const Minimap& mini, const Settings& set, const Menu& menu, bool paused,
            std::vector<SDL_Vertex>& verts) {
    const bool showGhost = set.ghost;
    SDL_SetRenderDrawColor(r, 40, 96, 48, 255);
    SDL_RenderClear(r);

    // Track (culled to what's roughly on screen).
    verts.clear();
    float rx = SCREEN_W * 0.5f / view.zoom + 450, ry = SCREEN_H * 0.5f / view.zoom + 450;
    for (const WorldQuad& q : quads) {
        if (std::fabs(q.v[0].x - view.cam.x) > rx || std::fabs(q.v[0].y - view.cam.y) > ry) continue;
        pushQuad(verts, view.toScreen(q.v[0]), view.toScreen(q.v[1]), view.toScreen(q.v[2]),
                 view.toScreen(q.v[3]), q.c);
    }
    SDL_RenderGeometry(r, nullptr, verts.data(), int(verts.size()), nullptr, 0);

    // Skid marks.
    if (set.skidMarks) {
        std::vector<SDL_FRect> marks;
        float ms = 3.5f * view.zoom;
        for (Vec2 p : g.skids) {
            Vec2 s = view.toScreen(p);
            if (s.x < -10 || s.y < -10 || s.x > SCREEN_W + 10 || s.y > SCREEN_H + 10) continue;
            marks.push_back({s.x - ms / 2, s.y - ms / 2, ms, ms});
        }
        SDL_SetRenderDrawColor(r, 20, 20, 20, 70);
        SDL_RenderFillRectsF(r, marks.data(), int(marks.size()));
    }

    // Ghost of the best lap, then the player.
    verts.clear();
    bool ghostVisible = showGhost && g.lapStarted && g.lapSteps < int(g.bestGhost.size());
    if (ghostVisible) {
        const GhostSample& s = g.bestGhost[g.lapSteps];
        pushCar(verts, view, {s.x, s.y}, s.h, 0, {150, 200, 255, 255}, 90);
    }
    pushCar(verts, view, g.car.pos, g.car.heading, g.car.steer, {220, 30, 40, 255}, 255);
    SDL_RenderGeometry(r, nullptr, verts.data(), int(verts.size()), nullptr, 0);

    // Minimap.
    if (set.minimap) {
        fillRect(r, SCREEN_W - 270, 10, 260, 260, {0, 0, 0, 130});
        SDL_SetRenderDrawColor(r, 200, 200, 200, 255);
        std::vector<SDL_FPoint> shifted = mini.line;
        for (int d = 0; d < 2; ++d) {
            SDL_RenderDrawLinesF(r, shifted.data(), int(shifted.size()));
            for (auto& p : shifted) p.x += 1;
        }
        SDL_FPoint sl = mini.map(g.track.pts[0]);
        fillRect(r, sl.x - 2, sl.y - 5, 4, 10, WHITE);
        if (ghostVisible) {
            const GhostSample& s = g.bestGhost[g.lapSteps];
            SDL_FPoint p = mini.map({s.x, s.y});
            fillRect(r, p.x - 3, p.y - 3, 6, 6, {150, 200, 255, 255});
        }
        SDL_FPoint cp = mini.map(g.car.pos);
        fillRect(r, cp.x - 4, cp.y - 4, 8, 8, RED);
    }

    // Timing panel.
    const float S = 3;
    fillRect(r, 10, 10, 360, 218, {0, 0, 0, 140});
    float y = 22;
    drawText(r, 24, y, S, g.lapStarted ? "LAP " + std::to_string(g.lapNumber) : "OUT LAP", GOLD);
    if (g.lapStarted && !g.lapValid) drawText(r, 356, y, S, "INVALID", RED, 2);
    y += 30;
    drawText(r, 24, y, S, "TIME", WHITE);
    drawText(r, 356, y, S, g.lapStarted ? fmtTime(g.lapSteps * DT) : "-:--.---", g.lapValid ? WHITE : RED, 2);
    y += 26;
    drawText(r, 24, y, S, "LAST", WHITE);
    drawText(r, 356, y, S, fmtTime(g.lastLap), WHITE, 2);
    y += 26;
    drawText(r, 24, y, S, "BEST", PURPLE);
    drawText(r, 356, y, S, fmtTime(g.bestTime), PURPLE, 2);
    y += 36;

    // Sector times for the current lap (or the one just finished, for a few seconds).
    bool showLast = g.splitsDone == 0 && g.lastLap > 0 && g.lapSteps < 360;
    const float* sp = showLast ? g.lastSplits : g.splits;
    int done = showLast ? 3 : g.splitsDone;
    for (int k = 0; k < 3; ++k, y += 26) {
        drawText(r, 24, y, S, "S" + std::to_string(k + 1), WHITE);
        if (k >= done) {
            drawText(r, 200, y, S, "--.---", {150, 150, 150, 255}, 2);
            continue;
        }
        float st = g.sectorTime(sp, k);
        char buf[16];
        std::snprintf(buf, sizeof buf, "%.3f", st);
        drawText(r, 200, y, S, buf, WHITE, 2);
        if (g.bestTime > 0) {
            float d = st - g.bestSectors[k];
            drawText(r, 356, y, S, fmtDelta(d), d <= 0 ? GREEN : RED, 2);
        }
    }

    // Recent laps.
    int shown = std::min<int>(5, int(g.history.size()));
    if (shown > 0) {
        float hy = SCREEN_H - 70 - shown * 22;
        fillRect(r, SCREEN_W - 230, hy - 12, 220, shown * 22 + 20, {0, 0, 0, 140});
        for (int k = int(g.history.size()) - shown; k < int(g.history.size()); ++k, hy += 22) {
            const LapEntry& e = g.history[k];
            SDL_Color c = e.best ? PURPLE : e.valid ? WHITE : RED;
            drawText(r, SCREEN_W - 220, hy, 2.5f, "L" + std::to_string(e.number), c);
            drawText(r, SCREEN_W - 20, hy, 2.5f, e.valid ? fmtTime(e.time) : "X " + fmtTime(e.time), c, 2);
        }
    }

    // Speedo with throttle / brake bars.
    Vec2 fwd{std::cos(g.car.heading), std::sin(g.car.heading)};
    float vf = dot(g.car.vel, fwd);
    int kmh = int(std::fabs(vf) * 0.36f + 0.5f);
    int shownSpeed = set.mph ? int(std::fabs(vf) * 0.36f * 0.621371f + 0.5f) : kmh;
    std::string gear = vf < -5 ? "R" : kmh < 3 ? "N" : std::to_string(std::min(8, 1 + kmh / 40));
    fillRect(r, SCREEN_W / 2 - 130, SCREEN_H - 100, 260, 90, {0, 0, 0, 140});
    drawText(r, SCREEN_W / 2 + 40, SCREEN_H - 88, 8, std::to_string(shownSpeed), WHITE, 2);
    drawText(r, SCREEN_W / 2 + 52, SCREEN_H - 60, 2.5f, set.mph ? "MPH" : "KM/H", {180, 180, 180, 255});
    drawText(r, SCREEN_W / 2 + 52, SCREEN_H - 88, 4, gear, GOLD);
    float tb = g.lastInput.throttle * 70, bb = g.lastInput.brake * 70;
    fillRect(r, SCREEN_W / 2 - 120, SCREEN_H - 20 - tb, 10, tb, GREEN);
    fillRect(r, SCREEN_W / 2 - 104, SCREEN_H - 20 - bb, 10, bb, RED);

    if (g.onGrass) drawText(r, SCREEN_W / 2, SCREEN_H - 130, 3, "OFF TRACK", RED, 1);

    if (g.messageTimer > 0 && !menu.open) {
        float w = textWidth(g.message, 4);
        fillRect(r, SCREEN_W / 2 - w / 2 - 16, 70, w + 32, 44, {0, 0, 0, 160});
        drawText(r, SCREEN_W / 2, 80, 4, g.message, g.messageColor, 1);
    }
    if (paused && !menu.open) {
        fillRect(r, 0, 0, SCREEN_W, SCREEN_H, {0, 0, 0, 120});
        drawText(r, SCREEN_W / 2, SCREEN_H / 2 - 30, 8, "PAUSED", WHITE, 1);
    }

    drawText(r, 14, SCREEN_H - 22, 2,
             "WASD/ARROWS DRIVE  SPACE HANDBRAKE  R RESTART LAP  G GHOST  P PAUSE  ESC SETTINGS",
             {220, 220, 220, 200});
    if (menu.open) drawMenu(r, menu, set);
}

Input readInput(SDL_GameController* pad, const Settings& set) {
    const Uint8* k = SDL_GetKeyboardState(nullptr);
    Input in;
    in.throttle = (k[SDL_SCANCODE_W] || k[SDL_SCANCODE_UP]) ? 1.0f : 0.0f;
    in.brake = (k[SDL_SCANCODE_S] || k[SDL_SCANCODE_DOWN]) ? 1.0f : 0.0f;
    in.steer = float((k[SDL_SCANCODE_D] || k[SDL_SCANCODE_RIGHT]) - (k[SDL_SCANCODE_A] || k[SDL_SCANCODE_LEFT]));
    in.handbrake = k[SDL_SCANCODE_SPACE];
    if (pad) {
        auto axis = [&](SDL_GameControllerAxis a) { return SDL_GameControllerGetAxis(pad, a) / 32767.0f; };
        float lx = axis(SDL_CONTROLLER_AXIS_LEFTX);
        float dz = set.padDeadzone / 100.0f;
        if (std::fabs(lx) > dz) in.steer = std::clamp(signOf(lx) * (std::fabs(lx) - dz) / (1.0f - dz), -1.0f, 1.0f);
        in.throttle = std::max(in.throttle, axis(SDL_CONTROLLER_AXIS_TRIGGERRIGHT));
        in.brake = std::max(in.brake, axis(SDL_CONTROLLER_AXIS_TRIGGERLEFT));
        in.handbrake = in.handbrake || SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_A);
    }
    return in;
}

int runGame() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Window* window = SDL_CreateWindow("F1 Hotlap", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_W,
                                          SCREEN_H, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* r = window ? SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC)
                             : nullptr;
    if (!r) {
        std::fprintf(stderr, "Could not create window: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    SDL_RenderSetLogicalSize(r, SCREEN_W, SCREEN_H);
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);

    Game game;
    Settings set;
    std::string settingsPath;
    if (char* base = SDL_GetBasePath()) {
        game.savePath = std::string(base) + "best_lap.dat";
        settingsPath = std::string(base) + "settings.cfg";
        SDL_free(base);
    }
    loadSettings(set, settingsPath);
    game.init();
    if (game.bestTime > 0) game.flash("BEST LAP TO BEAT " + fmtTime(game.bestTime), PURPLE, 4.0f);

    // Push settings into the game, window and renderer.
    auto applySettings = [&]() {
        game.steerSensitivity = set.steerSensitivity / 100.0f;
        game.steerSpeed = set.steerSpeed / 100.0f;
        bool fs = SDL_GetWindowFlags(window) & SDL_WINDOW_FULLSCREEN_DESKTOP;
        if (fs != bool(set.fullscreen))
            SDL_SetWindowFullscreen(window, set.fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
        SDL_RenderSetVSync(r, set.vsync);
    };
    auto changed = [&]() {
        applySettings();
        saveSettings(set, settingsPath);
    };
    applySettings();

    std::vector<WorldQuad> quads = buildTrackQuads(game);
    Minimap mini;
    mini.build(game.track, SCREEN_W - 260, 20, 240);
    std::vector<SDL_Vertex> verts;
    View view;
    view.cam = game.car.pos;

    SDL_GameController* pad = nullptr;
    Menu menu;
    bool running = true, paused = false;

    auto menuMove = [&](int dir) {
        menu.sel = (menu.sel + dir + MENU_COUNT) % MENU_COUNT;
        menu.confirmReset = false;
    };
    auto menuAdjust = [&](int dir) {
        const MenuEntry& m = MENU[menu.sel];
        if (!m.field) return;
        int& v = set.*m.field;
        int nv = m.kind == Kind::Percent ? std::clamp(v + dir * m.step, m.lo, m.hi) : 1 - v;
        if (nv != v) {
            v = nv;
            changed();
        }
    };
    auto menuActivate = [&]() {
        const MenuEntry& m = MENU[menu.sel];
        switch (m.kind) {
            case Kind::Toggle:
            case Kind::Units: menuAdjust(1); break;
            case Kind::ResetBest:
                if (!menu.confirmReset) {
                    menu.confirmReset = true;
                } else {
                    menu.confirmReset = false;
                    game.resetBest();
                    game.flash("BEST LAP RESET", WHITE);
                }
                break;
            case Kind::Resume: menu.open = false; break;
            case Kind::Quit: running = false; break;
            default: break;
        }
    };
    auto restartLap = [&]() {
        game.resetToGrid();
        view.cam = game.car.pos;
    };
    auto toggleMenu = [&]() {
        menu.open = !menu.open;
        menu.confirmReset = false;
    };

    Uint64 prev = SDL_GetPerformanceCounter();
    double acc = 0;
    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            if (e.type == SDL_KEYDOWN && menu.open) {
                switch (e.key.keysym.scancode) {
                    case SDL_SCANCODE_ESCAPE: toggleMenu(); break;
                    case SDL_SCANCODE_UP: case SDL_SCANCODE_W: menuMove(-1); break;
                    case SDL_SCANCODE_DOWN: case SDL_SCANCODE_S: menuMove(1); break;
                    case SDL_SCANCODE_LEFT: case SDL_SCANCODE_A: menuAdjust(-1); break;
                    case SDL_SCANCODE_RIGHT: case SDL_SCANCODE_D: menuAdjust(1); break;
                    case SDL_SCANCODE_RETURN: case SDL_SCANCODE_SPACE: if (!e.key.repeat) menuActivate(); break;
                    default: break;
                }
            } else if (e.type == SDL_KEYDOWN && !e.key.repeat) {
                switch (e.key.keysym.scancode) {
                    case SDL_SCANCODE_ESCAPE: toggleMenu(); break;
                    case SDL_SCANCODE_R:
                    case SDL_SCANCODE_RETURN: restartLap(); break;
                    case SDL_SCANCODE_G: set.ghost = !set.ghost; changed(); break;
                    case SDL_SCANCODE_P: paused = !paused; break;
                    case SDL_SCANCODE_F11: set.fullscreen = !set.fullscreen; changed(); break;
                    default: break;
                }
            }
            if (menu.open && e.type == SDL_MOUSEMOTION) {
                int i = menuRowAt(float(e.motion.x), float(e.motion.y));
                if (i >= 0 && i != menu.sel) menuMove(i - menu.sel);
            }
            if (menu.open && e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                float mx = float(e.button.x), my = float(e.button.y);
                int i = menuRowAt(mx, my);
                if (i >= 0) {
                    if (i != menu.sel) menuMove(i - menu.sel);
                    if (MENU[i].kind == Kind::Percent) {
                        if (int dir = menuArrowAt(MENU[i], set, menu, mx)) menuAdjust(dir);
                    } else {
                        menuActivate();
                    }
                }
            }
            if (menu.open && e.type == SDL_MOUSEWHEEL && MENU[menu.sel].kind == Kind::Percent) {
                int dy = e.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -e.wheel.y : e.wheel.y;
                if (dy != 0) menuAdjust(dy > 0 ? 1 : -1);
            }
            if (e.type == SDL_CONTROLLERDEVICEADDED && !pad) pad = SDL_GameControllerOpen(e.cdevice.which);
            if (e.type == SDL_CONTROLLERDEVICEREMOVED && pad &&
                e.cdevice.which == SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(pad))) {
                SDL_GameControllerClose(pad);
                pad = nullptr;
            }
            if (e.type == SDL_CONTROLLERBUTTONDOWN) {
                Uint8 b = e.cbutton.button;
                if (b == SDL_CONTROLLER_BUTTON_BACK) toggleMenu();
                else if (menu.open) {
                    if (b == SDL_CONTROLLER_BUTTON_DPAD_UP) menuMove(-1);
                    if (b == SDL_CONTROLLER_BUTTON_DPAD_DOWN) menuMove(1);
                    if (b == SDL_CONTROLLER_BUTTON_DPAD_LEFT) menuAdjust(-1);
                    if (b == SDL_CONTROLLER_BUTTON_DPAD_RIGHT) menuAdjust(1);
                    if (b == SDL_CONTROLLER_BUTTON_A) menuActivate();
                    if (b == SDL_CONTROLLER_BUTTON_B) toggleMenu();
                } else {
                    if (b == SDL_CONTROLLER_BUTTON_START || b == SDL_CONTROLLER_BUTTON_B) restartLap();
                }
            }
        }

        Uint64 now = SDL_GetPerformanceCounter();
        acc += std::min(0.25, double(now - prev) / SDL_GetPerformanceFrequency());
        prev = now;
        if (paused || menu.open) acc = 0;
        Input in = readInput(pad, set);
        while (acc >= DT) {
            game.step(in);
            acc -= DT;
        }

        // Camera looks ahead and (optionally) zooms out with speed.
        float speed = length(game.car.vel);
        Vec2 target = game.car.pos + game.car.vel * (0.35f * set.lookAhead / 100.0f);
        view.cam = view.cam + (target - view.cam) * 0.12f;
        float speedFactor = set.speedZoom ? 1.0f - 0.25f * std::min(speed / TOP_SPEED, 1.0f) : 1.0f;
        float zoomTarget = set.cameraZoom / 100.0f * speedFactor;
        view.zoom += (zoomTarget - view.zoom) * 0.05f;

        render(r, game, view, quads, mini, set, menu, paused, verts);
        SDL_RenderPresent(r);
    }

    if (pad) SDL_GameControllerClose(pad);
    SDL_DestroyRenderer(r);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--selftest") == 0) return runSelfTest();
    return runGame();
}
