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
constexpr float DRS_TOP = 1.06f, DRS_DRAG = 0.6f;            // open rear wing: more top speed, less drag
constexpr float DRS_MIN_RUN = 1400.0f, DRS_LEAD_IN = 150.0f;  // zone length needed, and where it starts
constexpr size_t DRS_ZONES = 2;                              // per circuit, the longest straights
constexpr int DRS_GRID_CLEAR = 90;                           // samples before the line kept DRS-free

// Gearbox. It's cosmetic: it drives the HUD, rev lights and engine note, not the physics.
constexpr int GEARS = 8;
constexpr float IDLE_RPM = 4000, MAX_RPM = 12000, SHIFT_RPM = 11600, DOWNSHIFT_RPM = 9000;
constexpr float GEAR_TOP[GEARS] = {250, 340, 420, 500, 580, 660, 750, 845};  // speed at MAX_RPM

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
    std::vector<float> arc;  // distance from the start line to each sample
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
        arc.resize(N);
        std::vector<float> raw(N);
        for (int i = 0; i < N; ++i) {
            Vec2 prev = pts[(i - 1 + N) % N], next = pts[(i + 1) % N];
            Vec2 t = normalized(next - prev);
            normals[i] = {-t.y, t.x};
            arc[i] = length;
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

    // Racing line: a minimum-curvature path inside the track edges, with the
    // fastest speed the car can carry at each sample. zone: 0 = accelerating
    // or flat out, 1 = at the grip limit through a corner, 2 = braking.
    std::vector<Vec2> line;
    std::vector<float> lineSpeed;
    std::vector<float> lineGrip;  // cornering limit alone, uncapped on the straights
    std::vector<uint8_t> lineZone;
    std::vector<uint8_t> drsZone;  // 1 where the rear wing may be opened

    void buildRacingLine(float maxLat, float engine, float brake, float topSpeed) {
        const int N = size();
        const float lim = HALF_W - 10;
        auto at = [N](int i) { return ((i % N) + N) % N; };
        line.assign(pts.begin(), pts.end());
        // Minimum curvature swings the line out-in-out through the corners. Each
        // point slides across the asphalt to reduce the squared turning per unit
        // length (the integral of curvature squared) around it, using neighbours
        // k samples away, coarse to fine.
        auto bend = [&](int j, int k) {
            Vec2 a = line[at(j - k)], b = line[at(j)], c = line[at(j + k)];
            Vec2 u = b - a, v = c - b;
            float lu = ::length(u), lv = ::length(v);
            if (lu < 1e-3f || lv < 1e-3f) return 0.0f;
            float turn = ::length(v * (1 / lv) - u * (1 / lu));  // ~ turning angle
            return turn * turn / (lu + lv);
        };
        std::vector<float> off(N, 0.0f);
        for (int k : {32, 16, 8, 4, 2, 1}) {
            float step = std::min(8.0f, 1.0f + 0.5f * k);
            for (int iter = 0; iter < 40; ++iter, step *= 0.92f)
                for (int i = 0; i < N; ++i) {
                    float bestOff = off[i];
                    float best = bend(i - k, k) + bend(i, k) + bend(i + k, k);
                    for (float d : {-step, step}) {
                        float o = std::clamp(off[i] + d, -lim, lim);
                        line[i] = pts[i] + normals[i] * o;
                        float cost = bend(i - k, k) + bend(i, k) + bend(i + k, k);
                        if (cost < best) {
                            best = cost;
                            bestOff = o;
                        }
                    }
                    off[i] = bestOff;
                    line[i] = pts[i] + normals[i] * bestOff;
                }
        }

        // Speed: the grip limit from the line's curvature, then forward
        // (acceleration) and backward (braking) passes, twice round the loop.
        std::vector<float> ds(N), vmax(N);
        for (int i = 0; i < N; ++i) {
            ds[i] = ::length(line[at(i + 1)] - line[i]);
            float k = 0;
            for (int j = -3; j <= 3; ++j) {
                int m = at(i + j);
                Vec2 u = line[m] - line[at(m - 1)], v = line[at(m + 1)] - line[m];
                k += std::fabs(std::atan2(cross(u, v), dot(u, v))) / std::max(0.5f * (::length(u) + ::length(v)), 1e-3f);
            }
            k /= 7;
            vmax[i] = k > 1e-6f ? std::min(topSpeed, std::sqrt(maxLat / k)) : topSpeed;
        }
        lineGrip.resize(N);
        for (int i = 0; i < N; ++i) lineGrip[i] = vmax[i] < topSpeed ? vmax[i] : 1e9f;
        lineSpeed = vmax;
        for (int pass = 0; pass < 2 * N; ++pass) {
            int i = pass % N, j = (i + 1) % N;
            float v = lineSpeed[i], acc = engine * (1.0f - v / topSpeed);
            lineSpeed[j] = std::min(lineSpeed[j], std::sqrt(v * v + 2 * std::max(acc, 0.0f) * ds[i]));
        }
        for (int pass = 2 * N - 1; pass >= 0; --pass) {
            int i = pass % N, j = (i + 1) % N;
            lineSpeed[i] = std::min(lineSpeed[i], std::sqrt(lineSpeed[j] * lineSpeed[j] + 2 * brake * ds[i]));
        }
        lineZone.assign(N, 0);
        for (int i = 0; i < N; ++i) {
            float v = lineSpeed[i], vn = lineSpeed[(i + 1) % N];
            if (vn < v - 0.5f) lineZone[i] = 2;
            else if (v < topSpeed - 1 && v >= vmax[i] - 1) lineZone[i] = 1;
        }

        // DRS zones: the longest nearly straight stretches the line takes flat
        // out, at most DRS_ZONES of them. Each starts a little way in.
        drsZone.assign(N, 0);
        auto straight = [&](int i) { return lineZone[i] == 0 && curvature[i] < 0.0008f; };
        int first = 0;
        while (first < N && straight(first)) ++first;  // start scanning outside a run
        if (first == N) return;
        struct Run {
            int start, end;
            float len;
        };
        std::vector<Run> runs;
        for (int k = 1; k <= N; ++k) {
            int i = at(first + k);
            if (!straight(i) || straight(at(i - 1))) continue;
            Run run{i, i, 0};
            while (straight(run.end)) {
                run.len += ds[run.end];
                run.end = at(run.end + 1);
            }
            if (run.len >= DRS_MIN_RUN) runs.push_back(run);
        }
        std::sort(runs.begin(), runs.end(), [](const Run& a, const Run& b) { return a.len > b.len; });
        if (runs.size() > DRS_ZONES) runs.resize(DRS_ZONES);
        for (const Run& run : runs) {
            float skip = 0;
            for (int j = run.start; j != run.end; j = at(j + 1)) {
                if (skip >= DRS_LEAD_IN) drsZone[j] = 1;
                skip += ds[j];
            }
        }
        // Not on the grid or over the start line: a zone there begins just after it.
        for (int k = -DRS_GRID_CLEAR; k < 10; ++k) drsZone[at(k)] = 0;
    }

    // Distance along the lap at a hit, unwrapped so it stays continuous with
    // prev (just past the line reads as length + a bit, not ~0).
    float distanceAt(const TrackHit& h, float prev) const {
        float d = arc[h.seg] + ::length(h.closest - pts[h.seg]);
        while (d - prev > length * 0.5f) d -= length;
        while (prev - d > length * 0.5f) d += length;
        return d;
    }
};

// A circuit is either a loop of control points, or pieces (straights and
// constant-radius corners) laid end to end, which suits tight street layouts.
// Either way the first point is the start line, so it should sit on a
// straight with a straight run-up behind it.
struct Piece {
    float turn, radius, len;  // turn in degrees, + is right; turn == 0 is a straight of len
};
Piece straight(float len) { return {0, 0, len}; }
Piece corner(float deg, float radius) { return {deg, radius, 0}; }

struct TrackDef {
    const char* name;
    const char* saveFile;  // best lap file, next to the executable
    std::vector<Vec2> ctrl;
    std::vector<Piece> pieces;
    float aiLap;  // the selftest autopilot's lap; medal times are set from it
};

const TrackDef TRACKS[] = {
    {"PARKLAND", "best_lap.dat",
     {{300, 0},     {1200, 0},    {1800, 0},    {2300, 100},  {2600, 400},  {2600, 800},  {2350, 1050},
      {2000, 1000}, {1700, 800},  {1400, 850},  {1250, 1100}, {1400, 1400}, {1900, 1500}, {2500, 1500},
      {2900, 1700}, {2900, 2100}, {2500, 2300}, {1500, 2300}, {800, 2300},  {400, 2100},  {500, 1800},
      {300, 1500},  {-200, 1400}, {-500, 1100}, {-500, 500},  {-350, 150}},
     {},
     28.442f},
    // Fast: long straights and high-speed sweepers.
    {"COASTLINE", "best_lap_coastline.dat",
     {{0, 0},       {1000, 0},    {2000, 0},    {2850, 0},    {3080, 60},   {3180, 250},  {3250, 600},
      {3300, 1000}, {2900, 1200}, {2500, 1100}, {2100, 1250}, {1900, 1600}, {2100, 1950}, {2700, 2050},
      {3000, 2250}, {3080, 2480}, {2980, 2680}, {2750, 2780}, {2400, 2800}, {1600, 2800}, {700, 2800},
      {200, 2600},  {0, 2200},    {200, 1800},  {500, 1500},  {400, 1100},  {-100, 900},  {-500, 700},
      {-700, 300},  {-500, 0}},
     {},
     31.083f},
    // Tight street circuit: right angles and a chicane.
    {"HARBOUR", "best_lap_harbour.dat",
     {},
     {straight(1000), corner(90, 85),  straight(400),   corner(-90, 85), straight(500),   corner(90, 85),
      straight(500),  corner(90, 85),  straight(300),   corner(-40, 160), corner(80, 160), corner(-40, 160),
      straight(300),  corner(90, 85),  straight(500),   corner(-90, 85), straight(938.6f), corner(90, 85),
      straight(400),  corner(90, 85),  straight(450)},
     16.325f},
};
constexpr int TRACK_COUNT = int(sizeof TRACKS / sizeof TRACKS[0]);

// Lays pieces end to end from the origin, heading east, as evenly spaced
// control points (Catmull-Rom overshoots when the spacing is uneven).
std::vector<Vec2> layPieces(const std::vector<Piece>& pieces) {
    const float STEP = 60;
    Vec2 p;
    float h = 0;
    std::vector<Vec2> out{p};
    for (const Piece& pc : pieces) {
        if (pc.turn == 0) {
            int n = std::max(1, int(std::lround(pc.len / STEP)));
            Vec2 d{std::cos(h), std::sin(h)};
            for (int k = 0; k < n; ++k) out.push_back(p = p + d * (pc.len / n));
            continue;
        }
        float th = pc.turn * float(M_PI) / 180, side = h + signOf(th) * float(M_PI) / 2;
        Vec2 c = p + Vec2{std::cos(side), std::sin(side)} * pc.radius;
        float a0 = std::atan2(p.y - c.y, p.x - c.x);
        int n = std::max(1, int(std::lround(pc.radius * std::fabs(th) / STEP)));
        for (int k = 1; k <= n; ++k) {
            float a = a0 + th * k / n;
            out.push_back(p = c + Vec2{std::cos(a), std::sin(a)} * pc.radius);
        }
        h += th;
    }
    out.pop_back();  // the loop ends back on the start point
    return out;
}

std::vector<Vec2> circuitLayout(int track = 0) {
    const TrackDef& def = TRACKS[track];
    std::vector<Vec2> out;
    for (Vec2 p : def.pieces.empty() ? def.ctrl : layPieces(def.pieces)) out.push_back(p * 1.6f);
    return out;
}

// ---------------------------------------------------------------------------
// Game state and physics (no rendering here, so it can run headless).

struct Input {
    float throttle = 0, brake = 0, steer = 0;
    bool handbrake = false, drs = false;
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

enum Chime { CHIME_SLOWER, CHIME_FASTER, CHIME_PURPLE, CHIME_BEST_LAP };

constexpr uint32_t SAVE_MAGIC = 0x4C484631;  // "1FHL"
constexpr uint32_t SAVE_VERSION = 2;         // v2 adds the best-ever sectors; v1 still loads
constexpr int GRID_OFFSET = 40;  // samples behind the line for the run-up
constexpr float GRID_SLOT = HALF_W * 0.45f;  // sideways offset of the staggered grid slots
constexpr int CROSS_BAND = 60;   // samples either side of a line that count as "at" it
constexpr size_t MAX_SKIDS = 6000;

const SDL_Color WHITE{240, 240, 240, 255};
const SDL_Color RED{235, 70, 60, 255};
const SDL_Color GREEN{90, 220, 110, 255};
const SDL_Color GOLD{255, 200, 40, 255};
const SDL_Color PURPLE{200, 110, 255, 255};

// Medals for each circuit, as a share of the autopilot's lap time.
enum Medal { MEDAL_NONE, MEDAL_BRONZE, MEDAL_SILVER, MEDAL_GOLD, MEDAL_PLATINUM };
const char* MEDAL_NAMES[] = {"NONE", "BRONZE", "SILVER", "GOLD", "PLATINUM"};
const SDL_Color MEDAL_COLORS[] = {
    {150, 150, 150, 255}, {215, 135, 60, 255}, {200, 205, 215, 255}, {255, 200, 40, 255}, {120, 230, 255, 255}};
constexpr float MEDAL_FACTOR[] = {0, 1.12f, 1.06f, 1.02f, 0.985f};

float medalTime(int track, int medal) { return TRACKS[track].aiLap * MEDAL_FACTOR[medal]; }
int medalFor(int track, float lap) {
    int m = MEDAL_NONE;
    for (int k = MEDAL_BRONZE; k <= MEDAL_PLATINUM; ++k)
        if (lap > 0 && lap <= medalTime(track, k) + 1e-4f) m = k;
    return m;
}

std::string fmtTime(float s) {
    if (s < 0) return "-:--.---";
    int ms = int(std::lround(s * 1000.0f));
    char buf[32];
    std::snprintf(buf, sizeof buf, "%d:%02d.%03d", ms / 60000, (ms / 1000) % 60, ms % 1000);
    return buf;
}

std::string fmtDelta(float d) {
    // Round first so a tiny negative gap reads +0.000, not -0.000.
    long ms = std::lround(d * 1000.0f);
    char buf[32];
    std::snprintf(buf, sizeof buf, "%c%ld.%03ld", ms < 0 ? '-' : '+', std::labs(ms) / 1000, std::labs(ms) % 1000);
    return buf;
}

struct Game {
    Track track;
    Car car;
    int idx = 0;
    int cpIdx[2] = {};
    bool lapStarted = false, lapValid = true, offTrack = false, onGrass = false, onKerb = false;
    int gear = 0;  // -1 reverse, 0 neutral, 1..GEARS
    float rpm = IDLE_RPM;
    bool drsOpen = false;
    int wallHits = 0;
    float lastImpact = 0;  // speed into the wall on the latest hit
    int chimes = 0;        // timing beeps so far; the latest one is chimeKind
    int chimeKind = 0;
    float wrongWayTime = 0;  // how long the car has been heading backwards round the lap
    bool wrongWay = false;
    int nextCp = 3, lapSteps = 0, lapNumber = 0, splitsDone = 0;
    float splits[3] = {}, lastSplits[3] = {};
    float bestTime = -1, bestSectors[3] = {}, lastLap = -1;
    float bestEverSectors[3] = {};  // fastest each sector has been driven on a valid lap
    float lapDist = 0;              // distance along the track, continuous through the lap
    std::vector<GhostSample> bestGhost, curGhost;
    std::vector<GhostSample> aiGhost;  // the autopilot's lap, for the AI ghost (not saved)
    std::vector<float> bestDist;  // best lap's distance at each step, as a running max
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

    int trackId = 0;

    void init(int trackIndex = 0) {
        trackId = trackIndex;
        track.build(circuitLayout(trackIndex));
        // A little under the real limits, so the line is drivable rather than perfect.
        track.buildRacingLine(MAX_LAT * 0.85f, ENGINE, BRAKE * 0.8f, TOP_SPEED);
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

    void updateGearbox(float vf, float throttle) {
        float v = std::fabs(vf);
        if (vf < -5) {
            gear = -1;
        } else if (v < 3 && throttle == 0) {
            gear = 0;
        } else {
            gear = std::max(gear, 1);
            while (gear < GEARS && v > GEAR_TOP[gear - 1] * SHIFT_RPM / MAX_RPM) ++gear;
            while (gear > 1 && v < GEAR_TOP[gear - 2] * DOWNSHIFT_RPM / MAX_RPM) --gear;
        }
        float target = gear > 0    ? MAX_RPM * v / GEAR_TOP[gear - 1]
                       : gear < 0 ? IDLE_RPM + 6000.0f * v / REVERSE_SPEED
                                  : IDLE_RPM;
        // Clutch slip off the line, and free revs in neutral.
        float free = IDLE_RPM + throttle * (gear == 0 ? 7000.0f : gear == 1 ? 3500.0f : 0.0f);
        target = std::clamp(std::max(target, free), IDLE_RPM, MAX_RPM);
        rpm += (target - rpm) * (1.0f - std::exp(-25.0f * DT));
    }

    void placeAt(int i) {
        car = Car{};
        wrongWayTime = 0;
        wrongWay = false;
        drsOpen = false;
        gear = 0;
        rpm = IDLE_RPM;
        car.pos = track.pts[i];
        Vec2 t = track.tangent(i);
        car.heading = std::atan2(t.y, t.x);
        idx = i;
        lapDist = track.arc[i];
    }

    void resetToGrid() {
        placeAt(track.size() - GRID_OFFSET);
        car.pos = car.pos + track.normals[idx] * GRID_SLOT;
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

    // Sum of the best-ever sectors, or -1 until every sector has one.
    float idealLap() const {
        float sum = 0;
        for (float s : bestEverSectors) {
            if (s <= 0) return -1;
            sum += s;
        }
        return sum;
    }

    // Recomputes the best lap's distance profile from its ghost.
    void rebuildBestDist() {
        bestDist.clear();
        bestDist.reserve(bestGhost.size());
        int hint = 0;
        float d = 0, run = 0;
        for (size_t k = 0; k < bestGhost.size(); ++k) {
            TrackHit h = track.nearest({bestGhost[k].x, bestGhost[k].y}, hint, k ? 40 : -1);
            hint = h.seg;
            d = track.distanceAt(h, d);
            run = k ? std::max(run, d) : d;
            bestDist.push_back(run);
        }
    }

    // Time the best lap took to reach distance d, or -1 if it never did.
    float bestTimeAt(float d) const {
        auto it = std::lower_bound(bestDist.begin(), bestDist.end(), d);
        if (it == bestDist.end()) return -1;
        size_t k = size_t(it - bestDist.begin());
        float t = (k + 1) * DT;  // sample k is recorded at the end of step k + 1
        if (k > 0 && bestDist[k] > bestDist[k - 1])
            t -= DT * (bestDist[k] - d) / (bestDist[k] - bestDist[k - 1]);
        return t;
    }

    // Live gap to the best lap at the car's current position.
    bool liveDelta(float& out) const {
        if (!lapStarted || bestTime < 0) return false;
        float t = bestTimeAt(lapDist);
        if (t < 0) return false;
        out = lapSteps * DT - t;
        return true;
    }

    void startLap() {
        lapStarted = true;
        lapValid = !offTrack;
        lapDist = std::fmod(lapDist + track.length, track.length);
        lapSteps = 0;
        nextCp = 1;
        splitsDone = 0;
        curGhost.clear();
        curGhost.reserve(12000);
        ++lapNumber;
    }

    // Timing beeps, like a dash display: CHIME_SLOWER, then faster than the best
    // lap's sector, a best-ever (purple) sector, and a new best lap.
    void chime(int kind) {
        ++chimes;
        chimeKind = kind;
    }
    int sectorChime(int k) const {
        float st = sectorTime(splits, k);
        if (lapValid && (bestEverSectors[k] <= 0 || st < bestEverSectors[k])) return CHIME_PURPLE;
        if (lapValid && bestTime > 0 && st < bestSectors[k]) return CHIME_FASTER;
        return CHIME_SLOWER;
    }

    void finishLap() {
        float t = lapSteps * DT;
        splits[2] = t;
        bool newBest = lapValid && (bestTime < 0 || t < bestTime);
        chime(newBest ? CHIME_BEST_LAP : sectorChime(2));
        std::copy(splits, splits + 3, lastSplits);
        lastLap = t;
        LapEntry e{lapNumber, t, lapValid, false};
        bool newSectorPb = false;
        if (lapValid) {
            for (int k = 0; k < 3; ++k) {
                float st = sectorTime(splits, k);
                if (bestEverSectors[k] <= 0 || st < bestEverSectors[k]) {
                    bestEverSectors[k] = st;
                    newSectorPb = true;
                }
            }
        }
        if (lapValid && (bestTime < 0 || t < bestTime)) {
            float prev = bestTime;
            bestTime = t;
            for (int k = 0; k < 3; ++k) bestSectors[k] = sectorTime(splits, k);
            bestGhost = curGhost;
            rebuildBestDist();
            e.best = true;
            save();
            int medal = medalFor(trackId, t);
            if (medal > medalFor(trackId, prev))
                flash(std::string(MEDAL_NAMES[medal]) + " MEDAL! " + fmtTime(t), MEDAL_COLORS[medal], 4.0f);
            else
                flash(prev < 0 ? "FIRST LAP RECORD SET " + fmtTime(t) : "NEW BEST LAP! " + fmtDelta(t - prev),
                      PURPLE, 3.5f);
        } else if (!lapValid) {
            flash("LAP " + fmtTime(t) + " NOT COUNTED", RED);
        } else {
            flash("LAP " + fmtTime(t) + "  " + fmtDelta(t - bestTime), WHITE);
            if (newSectorPb) save();
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
        onKerb = here.dist > HALF_W && !onGrass;
        float grip = onGrass ? 0.5f : 1.0f;
        // DRS: the wing opens while the button is held in a zone, and shuts on
        // the brakes or at the end of the zone.
        bool drsZone = track.drsZone[here.seg];
        if (!drsZone || in.brake > 0 || onGrass) drsOpen = false;
        else if (in.drs) drsOpen = true;
        float top = onGrass ? GRASS_TOP : TOP_SPEED * (drsOpen ? DRS_TOP : 1.0f);

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
        vf -= vf * (0.10f * (drsOpen ? DRS_DRAG : 1.0f) + (onGrass ? 1.4f : 0.0f)) * DT;
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
        updateGearbox(vf, in.throttle);

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
            if (vn > 40) {
                ++wallHits;
                lastImpact = vn;
            }
        }

        const int N = track.size();
        int prevIdx = idx;
        idx = after.seg;
        lapDist = track.distanceAt(after, lapDist);
        offTrack = after.dist > HALF_W + KERB_W + LIMIT_TOL;
        wrongWayTime = dot(car.vel, track.tangent(idx)) < -40 ? wrongWayTime + DT : 0;
        wrongWay = wrongWayTime > 0.5f;
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
                    chime(sectorChime(k));
                }
            }
        }
        if (crossedFwd) {
            if (!lapStarted) {
                startLap();
                messageTimer = 0;  // the "cross the line" prompt has done its job
            } else if (nextCp == 3) {
                finishLap();
            }
        }
        // Reversing over the line abandons the lap; crossing it forwards again starts a fresh one.
        if (crossedBack && lapStarted) {
            lapStarted = false;
            lapValid = true;
            splitsDone = 0;
            curGhost.clear();
            flash("LAP ABANDONED", RED);
        }
    }

    // Replay: puts the car at sample k of a recorded lap and works out what the
    // HUD, sound and effects need (speed, slip, steering, pedals, surface, gear).
    void puppetTo(const std::vector<GhostSample>& ghost, int k) {
        const int last = int(ghost.size()) - 1;
        k = std::clamp(k, 0, last);
        const GhostSample &s = ghost[k], &n = ghost[std::min(k + 1, last)];
        Vec2 pos{s.x, s.y}, vel = (Vec2{n.x, n.y} - pos) * (1.0f / DT);
        bool jumped = length(pos - car.pos) > 60;  // seeking, or looping back to the start
        float accel = (length(vel) - length(car.vel)) / DT;
        Vec2 fwd{std::cos(s.h), std::sin(s.h)}, right{-fwd.y, fwd.x};
        float vf = dot(vel, fwd);
        float yawRate = std::remainder(n.h - s.h, 2.0f * float(M_PI)) / DT;
        float wheel = std::atan(yawRate * WHEELBASE / std::max(std::fabs(vf), 60.0f));
        car.pos = pos;
        car.vel = vel;
        car.heading = s.h;
        car.slip = dot(vel, right);
        car.steer = std::clamp(wheel * (1.0f + std::fabs(vf) / 320.0f) / 0.62f, -1.0f, 1.0f);
        lastInput = Input{};
        if (!jumped) {
            lastInput.throttle = accel > 30 ? 1.0f : 0.0f;
            lastInput.brake = accel < -400 ? 1.0f : 0.0f;
        }
        TrackHit h = track.nearest(pos, idx, jumped ? -1 : 40);
        idx = h.seg;
        drsOpen = track.drsZone[idx] && vf > TOP_SPEED * 0.9f && lastInput.brake == 0;
        onGrass = h.dist > HALF_W + KERB_W;
        onKerb = h.dist > HALF_W && !onGrass;
        lapStarted = true;
        lapSteps = k + 1;
        ++stepCount;
        updateGearbox(vf, lastInput.throttle);
    }

    uint32_t trackSignature() const { return uint32_t(track.size()) * 100003u + uint32_t(track.length); }

    void writeSave(FILE* f) const {
        uint32_t header[4] = {SAVE_MAGIC, SAVE_VERSION, trackSignature(), uint32_t(bestGhost.size())};
        std::fwrite(header, sizeof header, 1, f);
        std::fwrite(&bestTime, sizeof bestTime, 1, f);
        std::fwrite(bestSectors, sizeof bestSectors, 1, f);
        std::fwrite(bestEverSectors, sizeof bestEverSectors, 1, f);
        std::fwrite(bestGhost.data(), sizeof(GhostSample), bestGhost.size(), f);
    }

    // Reads into locals first so a bad file leaves the current record alone.
    bool readSave(FILE* f) {
        uint32_t header[4];
        if (std::fread(header, sizeof header, 1, f) != 1 || header[0] != SAVE_MAGIC || header[1] < 1 ||
            header[1] > SAVE_VERSION || header[2] != trackSignature() || header[3] >= 1000000)
            return false;
        float time, sectors[3], ever[3];
        std::vector<GhostSample> ghost(header[3]);
        bool ok = std::fread(&time, sizeof time, 1, f) == 1 && std::fread(sectors, sizeof sectors, 1, f) == 1;
        if (header[1] >= 2) ok = ok && std::fread(ever, sizeof ever, 1, f) == 1;
        else std::copy(sectors, sectors + 3, ever);
        ok = ok && std::fread(ghost.data(), sizeof(GhostSample), header[3], f) == header[3];
        if (!ok) return false;
        bestTime = time;
        std::copy(sectors, sectors + 3, bestSectors);
        std::copy(ever, ever + 3, bestEverSectors);
        bestGhost = std::move(ghost);
        rebuildBestDist();
        return true;
    }

    void save() const {
        if (savePath.empty()) return;
        FILE* f = std::fopen(savePath.c_str(), "wb");
        if (!f) return;
        writeSave(f);
        std::fclose(f);
    }

    void load() {
        if (savePath.empty()) return;
        FILE* f = std::fopen(savePath.c_str(), "rb");
        if (!f) return;
        readSave(f);
        std::fclose(f);
    }

    void resetBest() {
        bestTime = -1;
        std::fill(bestSectors, bestSectors + 3, 0.0f);
        std::fill(bestEverSectors, bestEverSectors + 3, 0.0f);
        bestGhost.clear();
        bestDist.clear();
        for (auto& e : history) e.best = false;
        if (!savePath.empty()) std::remove(savePath.c_str());
    }
};

// ---------------------------------------------------------------------------
// Sound: everything is synthesised, so there are still no asset files.

struct SoundParams {
    float rpm = IDLE_RPM, throttle = 0, slip = 0, speed = 0, volume = 0, impact = 0;
    bool kerb = false, grass = false;
    int wallHits = 0;
    int chimes = 0, chimeKind = 0;
};

SoundParams soundParams(const Game& g, float volume) {
    SoundParams p;
    p.rpm = g.rpm;
    p.throttle = g.lastInput.throttle;
    p.slip = std::fabs(g.car.slip);
    p.speed = length(g.car.vel);
    p.volume = volume;
    p.kerb = g.onKerb;
    p.grass = g.onGrass;
    p.wallHits = g.wallHits;
    p.impact = g.lastImpact;
    p.chimes = g.chimes;
    p.chimeKind = g.chimeKind;
    return p;
}

struct Synth {
    SoundParams p;  // written by the game thread under SDL_LockAudioDevice
    float rate = 48000;
    float phase = 0, subPhase = 0, rpm = IDLE_RPM, throttle = 0, gain = 0;
    float engineLp = 0, grassLp = 0, kerbLp = 0, kerbPhase = 0;
    float squeal = 0, squealPhase = 0, vibPhase = 0;
    float pop = 0, thump = 0, thumpTime = 0;
    int seenHits = 0;
    int seenChimes = -1;  // -1 until the first buffer, so old beeps aren't replayed
    float chimeNotes[4] = {}, chimeTime = 0;
    int chimeLen = 0;
    uint32_t seed = 0x9E3779B9u;

    float noise() {
        seed = seed * 1664525u + 1013904223u;
        return float(seed >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }

    void fill(float* out, int n) {
        const float TAU = 2.0f * float(M_PI), dt = 1.0f / rate;
        auto coef = [&](float hz) { return 1.0f - std::exp(-hz * dt); };
        const float kRpm = coef(30), kThrottle = coef(15), kGain = coef(8), kSqueal = coef(12);
        const float kPop = std::exp(-45.0f * dt), kThump = std::exp(-14.0f * dt);
        const float kGrass = coef(TAU * 250), kKerb = coef(TAU * 180);
        if (p.wallHits > seenHits) {
            thump = std::clamp(p.impact / 400.0f, 0.3f, 1.0f);
            thumpTime = 0;
        }
        seenHits = p.wallHits;  // also resyncs after a track change resets the count
        if (seenChimes >= 0 && p.chimes > seenChimes) {
            static const float NOTES[][4] = {{660}, {880, 880}, {880, 1175, 1568}, {784, 988, 1175, 1568}};
            static const int COUNT[] = {1, 2, 3, 4};
            int kind = std::clamp(p.chimeKind, 0, 3);
            std::copy(NOTES[kind], NOTES[kind] + 4, chimeNotes);
            chimeLen = COUNT[kind];
            chimeTime = 0;
        }
        seenChimes = p.chimes;
        float squealTarget = p.speed > 60 ? std::clamp((p.slip - 120.0f) / 350.0f, 0.0f, 1.0f) : 0.0f;
        float speedAmt = std::min(p.speed / 300.0f, 1.0f);

        for (int i = 0; i < n; ++i) {
            rpm += (p.rpm - rpm) * kRpm;
            throttle += (p.throttle - throttle) * kThrottle;
            gain += (p.volume - gain) * kGain;
            squeal += (squealTarget - squeal) * kSqueal;

            // Engine: a V6 fires three times per revolution. A few harmonics plus a
            // half-order rumble, roughened with combustion noise and low-passed.
            float f = rpm / 60.0f * 3.0f;
            phase += f * dt;
            phase -= std::floor(phase);
            subPhase += 0.5f * f * dt;
            subPhase -= std::floor(subPhase);
            float x = std::sin(TAU * phase) + 0.6f * std::sin(2 * TAU * phase + 0.5f) +
                      0.35f * std::sin(3 * TAU * phase + 1.1f) + 0.45f * std::sin(TAU * subPhase);
            x += noise() * (0.25f + 0.5f * throttle) * (1.0f - phase);
            float cutoff = 600.0f + 2600.0f * throttle + rpm * 0.12f;
            engineLp += (x - engineLp) * std::min(1.0f, TAU * cutoff * dt);
            float mix = engineLp * (0.18f + 0.32f * throttle);

            // Crackles on the overrun.
            if (throttle < 0.2f && rpm > 7000 && noise() > 0.9993f) pop = 0.5f + 0.5f * std::fabs(noise());
            mix += pop * noise() * 0.5f;
            pop *= kPop;

            // Tyre squeal: two detuned partials with a little vibrato.
            if (squeal > 0.001f) {
                vibPhase += 7.0f * dt;
                squealPhase += (950.0f + 50.0f * std::sin(TAU * vibPhase)) * dt;
                squealPhase -= std::floor(squealPhase);
                mix += squeal * 0.16f * (std::sin(TAU * squealPhase) + 0.5f * std::sin(TAU * squealPhase * 1.51f));
            }

            // Kerb rumble: a buzz at the rate the stripes pass under the tyres.
            float kerbIn = 0;
            if (p.kerb && p.speed > 40) {
                kerbPhase += p.speed / 24.0f * dt;
                kerbPhase -= std::floor(kerbPhase);
                kerbIn = kerbPhase < 0.5f ? 1.0f : -1.0f;
            }
            kerbLp += (kerbIn - kerbLp) * kKerb;
            mix += kerbLp * 0.45f;

            // Grass: low, rough noise.
            grassLp += ((p.grass ? noise() * speedAmt : 0.0f) - grassLp) * kGrass;
            mix += grassLp * 1.2f;

            // Wall impact: a short low thud.
            if (thump > 0.001f) {
                mix += thump * (std::sin(TAU * 55.0f * thumpTime) + 0.4f * noise()) * 0.8f;
                thump *= kThump;
                thumpTime += dt;
            }

            // Timing beeps sit outside the soft clip so they stay clean.
            float beep = 0;
            if (chimeLen > 0) {
                const float NOTE = 0.11f;
                int note = int(chimeTime / NOTE);
                float tn = chimeTime - note * NOTE;
                if (note < chimeLen) beep = std::min(1.0f, tn * 300) * std::exp(-tn * 16) * std::sin(TAU * chimeNotes[note] * tn);
                else chimeLen = 0;
                chimeTime += dt;
            }
            out[i] = std::tanh(mix * gain * 1.2f) * 0.75f + beep * gain * 0.22f;
        }
    }
};

void audioCallback(void* userdata, Uint8* stream, int len) {
    static_cast<Synth*>(userdata)->fill(reinterpret_cast<float*>(stream), len / int(sizeof(float)));
}

// Simple pure-pursuit driver used by --selftest to check the game is lappable.
// It follows the racing line at the line's own speeds.
Input autopilot(const Game& g, bool useDrs = true) {
    const Track& t = g.track;
    const int N = t.size();
    float speed = length(g.car.vel);
    Vec2 target = t.line[(g.idx + 6 + int(speed * 0.02f)) % N];
    Vec2 d = target - g.car.pos;
    float diff = std::remainder(std::atan2(d.y, d.x) - g.car.heading, 2.0f * float(M_PI));

    // The fastest speed from which the car can still brake for every corner ahead.
    float limit = 1e9f, dist = 0;
    for (int k = 0; k < 160; ++k) {
        int j = (g.idx + k) % N;
        limit = std::min(limit, std::sqrt(t.lineGrip[j] * t.lineGrip[j] + 2.0f * BRAKE * 0.8f * dist));
        dist += length(t.line[(j + 1) % N] - t.line[j]);
    }
    Vec2 fwd{std::cos(g.car.heading), std::sin(g.car.heading)};
    float vf = dot(g.car.vel, fwd);
    Input in;
    in.steer = std::clamp(diff * 3.0f, -1.0f, 1.0f);
    in.throttle = std::clamp((limit - vf) / 20.0f, 0.0f, 1.0f);
    in.brake = std::clamp((vf - limit - 5.0f) / 20.0f, 0.0f, 1.0f);
    in.drs = useDrs;
    return in;
}

// The autopilot's lap on a copy of the game (its faster of two, so a flying
// lap), for the AI ghost. Nothing is saved.
std::vector<GhostSample> aiLapGhost(const Game& from) {
    Game g = from;
    g.savePath.clear();
    g.resetBest();
    g.history.clear();
    g.resetToGrid();
    for (long s = 0; s < 120L * 300 && g.history.size() < 2; ++s) g.step(autopilot(g));
    return g.bestGhost;
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
    float angle = 0;  // screen rotation in radians (0 = north up)
    float cosA = 1, sinA = 0;
    void setAngle(float a) {
        angle = a;
        cosA = std::cos(a);
        sinA = std::sin(a);
    }
    Vec2 toScreen(Vec2 p) const {
        Vec2 d = p - cam;
        return {(d.x * cosA - d.y * sinA) * zoom + SCREEN_W * 0.5f, (d.x * sinA + d.y * cosA) * zoom + SCREEN_H * 0.5f};
    }
    Vec2 toWorld(Vec2 s) const {
        Vec2 u{(s.x - SCREEN_W * 0.5f) / zoom, (s.y - SCREEN_H * 0.5f) / zoom};
        return cam + Vec2{u.x * cosA + u.y * sinA, -u.x * sinA + u.y * cosA};
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
             Uint8 alpha, bool wingOpen = false) {
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
    if (wingOpen) {  // DRS: the flap lifts, leaving a slot in the rear wing
        pushBox(out, v, pos, heading, -22.5f, 0, 1, 9, dark);
        pushBox(out, v, pos, heading, -19.5f, 0, 1, 9, dark);
    } else {
        pushBox(out, v, pos, heading, -21, 0, 2.5f, 9, dark);   // rear wing
    }
    pushBox(out, v, pos, heading, 1, 0, 4.5f, 2.8f, dark);      // cockpit
    pushBox(out, v, pos, heading, 0, 0, 2.2f, 2.2f, helmet);    // driver
}

// Soft drop shadow under the car, offset down-right on screen.
void pushCarShadow(std::vector<SDL_Vertex>& out, const View& v, Vec2 pos, float heading) {
    size_t first = out.size();
    const SDL_Color shade{0, 0, 0, 70};
    pushBox(out, v, pos, heading, 0, 0, 23, 12, shade);
    pushBox(out, v, pos, heading, -5, 0, 10, 8.5f, shade);
    for (size_t k = first; k < out.size(); ++k) {
        out[k].position.x += 4 * v.zoom;
        out[k].position.y += 6 * v.zoom;
    }
}

struct WorldQuad {
    Vec2 v[4];
    SDL_Color c;
};

// Small deterministic RNG so the scenery is the same every time.
struct Rng {
    uint32_t s;
    float next() {
        s = s * 1664525u + 1013904223u;
        return float(s >> 8) / 16777216.0f;
    }
    float range(float lo, float hi) { return lo + (hi - lo) * next(); }
};

// Is p inside the loop traced by the centre line? (even-odd ray cast)
bool insideLoop(const Track& t, Vec2 p) {
    bool in = false;
    for (int i = 0, n = t.size(), j = n - 1; i < n; j = i++) {
        Vec2 a = t.pts[i], b = t.pts[j];
        if ((a.y > p.y) != (b.y > p.y) && p.x < a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y)) in = !in;
    }
    return in;
}

// Octagon as four quads fanned from the centre.
void pushBlob(std::vector<WorldQuad>& q, Vec2 c, float r, SDL_Color col) {
    Vec2 rim[8];
    for (int k = 0; k < 8; ++k) {
        float a = k * float(M_PI) / 4 + 0.39f;
        rim[k] = c + Vec2{std::cos(a), std::sin(a)} * r;
    }
    for (int k = 0; k < 8; k += 2) q.push_back({{c, rim[k], rim[k + 1], rim[(k + 2) % 8]}, col});
}

// Tyre walls as a chain of stacks along the true boundary (WALL_D from the
// centre line). On the inside of a corner tighter than WALL_D the offset line
// loops back on itself; those points sit closer to the track than the wall,
// so they're dropped and the wall closes into a clean sharp corner.
void addBarriers(const Track& t, std::vector<WorldQuad>& q) {
    const int N = t.size();
    const float off = WALL_D + 5, SPACING = 10, HALF = 4.5f;
    for (float side : {-1.0f, 1.0f}) {
        std::vector<Vec2> line;
        for (int i = 0; i < N; ++i) {
            Vec2 p = t.pts[i] + t.normals[i] * (side * off);
            if (t.nearest(p, i, 60).dist > off - 1) line.push_back(p);
        }
        float carry = 0;
        int stack = 0;
        for (size_t k = 0; k < line.size(); ++k) {
            Vec2 a = line[k], b = line[(k + 1) % line.size()];
            float len = length(b - a);
            Vec2 dir = normalized(b - a), nrm{-dir.y, dir.x};
            for (; carry < len; carry += SPACING, ++stack) {
                Vec2 c = a + dir * carry;
                SDL_Color col = (stack / 4) % 4 == 0 ? SDL_Color{200, 50, 45, 255} : SDL_Color{45, 45, 50, 255};
                q.push_back({{c - dir * HALF - nrm * HALF, c + dir * HALF - nrm * HALF, c + dir * HALF + nrm * HALF,
                              c - dir * HALF + nrm * HALF},
                             col});
            }
            carry -= len;
        }
    }
}

// A grandstand along the outside of the main straight, and trees in the
// spare ground beyond the tyre walls.
void addScenery(const Track& t, std::vector<WorldQuad>& q) {
    const int N = t.size();
    Rng rng{20261024u};

    int standA = 0, standB = 0;
    while (standA < N && t.arc[standA] < 300) ++standA;
    standB = standA;
    while (standB < N && t.arc[standB] < 1700) ++standB;
    float side = insideLoop(t, t.pts[standA] + t.normals[standA] * (WALL_D + 90)) ? -1.0f : 1.0f;
    const float s0 = WALL_D + 40, rowW = 22;
    const int rows = 5;
    auto standPt = [&](int i, float off) { return t.pts[i] + t.normals[i] * (side * off); };
    for (int i = standA; i < standB; ++i) {
        int j = i + 1;
        q.push_back({{standPt(i, s0 - 8), standPt(i, s0 + rows * rowW + 14), standPt(j, s0 + rows * rowW + 14),
                      standPt(j, s0 - 8)},
                     {95, 98, 110, 255}});
        for (int row = 0; row < rows; ++row) {
            float o = s0 + row * rowW + 4;
            q.push_back({{standPt(i, o), standPt(i, o + rowW - 6), standPt(j, o + rowW - 6), standPt(j, o)},
                         {60, 62, 72, 255}});
            // Spectators: a couple of coloured dots per seat block.
            for (int k = 0; k < 2; ++k) {
                if (rng.next() < 0.25f) continue;
                Vec2 c = standPt(i, o + rowW * 0.35f) + t.tangent(i) * (k * 6.0f - 3.0f);
                SDL_Color shirt{Uint8(rng.range(60, 255)), Uint8(rng.range(60, 255)), Uint8(rng.range(60, 255)), 255};
                q.push_back({{c + Vec2{-2.5f, -2.5f}, c + Vec2{2.5f, -2.5f}, c + Vec2{2.5f, 2.5f}, c + Vec2{-2.5f, 2.5f}},
                             shirt});
            }
        }
    }

    Vec2 lo{1e9f, 1e9f}, hi{-1e9f, -1e9f};
    for (Vec2 p : t.pts) {
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
    }
    struct Tree {
        Vec2 p;
        float r;
    };
    std::vector<Tree> trees;
    for (int tries = 0; tries < 5000 && trees.size() < 420; ++tries) {
        Vec2 p{rng.range(lo.x - 900, hi.x + 900), rng.range(lo.y - 900, hi.y + 900)};
        float r = rng.range(16, 38);
        TrackHit h = t.nearest(p, 0, -1);
        if (h.dist < WALL_D + 30 + r || h.dist > WALL_D + 900) continue;
        if (h.seg >= standA - 10 && h.seg <= standB + 10 && h.dist < s0 + rows * rowW + 40 + r) continue;
        bool clear = true;
        for (const Tree& o : trees) clear = clear && length(o.p - p) > (o.r + r) * 0.8f;
        if (clear) trees.push_back({p, r});
    }
    for (const Tree& tr : trees) pushBlob(q, tr.p + Vec2{6, 9}, tr.r, {0, 0, 0, 60});
    for (const Tree& tr : trees) {
        float shade = rng.range(0.8f, 1.15f);
        SDL_Color leaf{Uint8(34 * shade), Uint8(92 * shade), Uint8(40 * shade), 255};
        SDL_Color light{Uint8(52 * shade), Uint8(122 * shade), Uint8(52 * shade), 255};
        pushBlob(q, tr.p, tr.r, leaf);
        pushBlob(q, tr.p - Vec2{tr.r * 0.22f, tr.r * 0.22f}, tr.r * 0.55f, light);
    }
}

std::vector<WorldQuad> buildTrackQuads(const Game& g) {
    const Track& t = g.track;
    const int N = t.size();
    std::vector<WorldQuad> q;
    // A band between two offsets from the centre line.
    auto band = [&](float o1, float o2, auto colorAt) {
        for (int i = 0; i < N; ++i) {
            int j = (i + 1) % N;
            Vec2 a = t.pts[i] + t.normals[i] * o1, b = t.pts[i] + t.normals[i] * o2;
            Vec2 c = t.pts[j] + t.normals[j] * o2, d = t.pts[j] + t.normals[j] * o1;
            q.push_back({{a, b, c, d}, colorAt(i)});
        }
    };
    auto kerb = [&](int i) {
        if (t.curvature[i] < 0.0012f) return SDL_Color{230, 230, 230, 255};
        return (i / 2) % 2 ? SDL_Color{215, 40, 40, 255} : SDL_Color{240, 240, 240, 255};
    };
    // Mowing stripes on the runoff and faint bands in the asphalt give a sense of speed.
    auto runoff = [](int i) { return (i / 8) % 2 ? SDL_Color{78, 150, 64, 255} : SDL_Color{72, 141, 59, 255}; };
    auto asphalt = [](int i) { return (i / 6) % 2 ? SDL_Color{72, 72, 78, 255} : SDL_Color{68, 68, 74, 255}; };

    band(-WALL_D, WALL_D, runoff);
    addBarriers(t, q);
    band(HALF_W, HALF_W + KERB_W, kerb);
    band(-HALF_W - KERB_W, -HALF_W, kerb);
    band(-HALF_W, HALF_W, asphalt);

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

    // Staggered grid slots behind the line; the player starts in the second one.
    // Slots that would land in a corner are left off.
    const SDL_Color paint{235, 235, 235, 200};
    for (int k = 1; k <= 4; ++k) {
        int i = N - k * GRID_OFFSET / 2;
        if (t.curvature[i] > 0.0008f) continue;
        float o = (k % 2 ? -GRID_SLOT : GRID_SLOT);
        across(i, 27, 30, o - 22, o + 22, paint);
        across(i, -26, 30, o - 22, o - 19, paint);
        across(i, -26, 30, o + 19, o + 22, paint);
    }

    // DRS activation: a dashed line across the track with DRS painted after it,
    // upright for the driver.
    for (int i = 0; i < N; ++i) {
        if (!t.drsZone[i] || t.drsZone[(i - 1 + N) % N]) continue;
        const SDL_Color white{240, 240, 240, 190};
        for (float o = -HALF_W; o < HALF_W; o += 18) across(i, -2, 2, o, std::min(o + 10, HALF_W), white);
        const float S = 7, W = 11 * S;
        Vec2 u = t.normals[i], f = t.tangent(i);
        Vec2 topLeft = t.pts[i] + f * (24 + 5 * S) - u * (W / 2);
        const char* text = "DRS";
        for (int c = 0; c < 3; ++c) {
            const char* g = glyph(text[c]);
            for (int k = 0; k < 15; ++k) {
                if (g[k] != '1') continue;
                Vec2 a = topLeft + u * ((c * 4 + k % 3) * S) - f * ((k / 3) * S);
                q.push_back({{a, a + u * S, a + u * S - f * S, a - f * S}, white});
            }
        }
    }

    addScenery(t, q);
    return q;
}

// Tyre smoke, grass dust and camera shake. Purely visual, so it lives outside Game.
struct Particle {
    Vec2 pos, vel;
    float age, life, size;
    SDL_Color c;
};

struct Effects {
    std::vector<Particle> parts;
    float shake = 0, smokeAcc = 0, dustAcc = 0;
    int seenHits = -1;  // -1 until the first update, so old hits don't shake a fresh view
    Rng rng{7u};

    void update(const Game& g, float dt) {
        if (seenHits < 0) seenHits = g.wallHits;
        for (Particle& p : parts) {
            p.age += dt;
            p.pos = p.pos + p.vel * dt;
            p.vel = p.vel * std::exp(-2.5f * dt);
        }
        parts.erase(std::remove_if(parts.begin(), parts.end(), [](const Particle& p) { return p.age >= p.life; }),
                    parts.end());
        shake *= std::exp(-9.0f * dt);
        if (g.wallHits > seenHits) shake = std::min(g.lastImpact / 40.0f, 14.0f);
        seenHits = g.wallHits;
        if (dt <= 0) return;

        float speed = length(g.car.vel), slip = std::fabs(g.car.slip);
        float smoke = g.onGrass ? 0.0f
                                : std::max(std::clamp((slip - 160.0f) / 300.0f, 0.0f, 1.0f),
                                           g.lastInput.handbrake && speed > 120 ? 0.6f : 0.0f);
        float dust = g.onGrass ? std::min(speed / 400.0f, 1.0f) : 0.0f;
        smokeAcc += smoke * 70.0f * dt;
        dustAcc += dust * 60.0f * dt;
        Vec2 f{std::cos(g.car.heading), std::sin(g.car.heading)}, r{-f.y, f.x};
        auto spawn = [&](float& acc, SDL_Color c, float size) {
            for (; acc >= 1; acc -= 1) {
                if (parts.size() >= 400) continue;
                Vec2 wheel = g.car.pos - f * 14.0f + r * (rng.next() < 0.5f ? 11.0f : -11.0f);
                Vec2 jitter{rng.range(-30, 30), rng.range(-30, 30)};
                parts.push_back({wheel, g.car.vel * 0.15f + jitter, 0, rng.range(0.6f, 1.2f), size, c});
            }
        };
        spawn(smokeAcc, {215, 215, 220, 255}, 7);
        spawn(dustAcc, {140, 115, 70, 255}, 6);
    }
};

struct Minimap {
    float x, y, scale;
    Vec2 origin;
    std::vector<SDL_FPoint> line;
    std::vector<std::vector<SDL_FPoint>> drs;  // one polyline per DRS zone

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
        const int N = t.size();
        for (int i = 0; i < int(t.drsZone.size()); ++i) {
            if (!t.drsZone[i] || t.drsZone[(i - 1 + N) % N]) continue;
            drs.emplace_back();
            for (int j = i; t.drsZone[j % N] && int(drs.back().size()) <= N; ++j) drs.back().push_back(map(t.pts[j % N]));
        }
    }
    SDL_FPoint map(Vec2 p) const { return {x + (p.x - origin.x) * scale, y + (p.y - origin.y) * scale}; }
};

// ---------------------------------------------------------------------------
// Settings (saved to settings.cfg next to the binary) and the in-game menu.

struct Settings {
    int steerSensitivity = 100, steerSpeed = 100, padDeadzone = 12, cameraZoom = 100, lookAhead = 100;
    int cameraMode = 0;
    int speedZoom = 1, mph = 0, ghost = 1, minimap = 1, skidMarks = 1, fullscreen = 0, vsync = 1;
    int volume = 70;
    int track = 0;
    int racingLine = 0;
    int vibration = 1;
};

enum class Kind { Percent, Choice, Toggle, Units, Replay, ResetBest, Resume, Quit };

enum CameraMode { CAM_FOLLOW, CAM_CHASE, CAM_LOCKED, CAM_HELICOPTER, CAM_COUNT };
const char* CAMERA_NAMES[CAM_COUNT] = {"FOLLOW", "CHASE", "LOCKED", "HELICOPTER"};

const char* cameraName(int i) { return CAMERA_NAMES[i]; }
enum GhostBits { GHOST_BEST = 1, GHOST_AI = 2 };  // Settings::ghost is a mask of these
const char* GHOST_NAMES[] = {"OFF", "BEST LAP", "AI", "BEST + AI"};
const char* ghostName(int i) { return GHOST_NAMES[i]; }
const char* trackName(int i) { return TRACKS[i].name; }

struct MenuEntry {
    const char* label;
    const char* key;  // name in settings.cfg
    int Settings::*field;
    int lo, hi, step;
    Kind kind;
    const char* (*choiceName)(int) = nullptr;  // for Kind::Choice
};

const MenuEntry MENU[] = {
    {"TRACK", "track", &Settings::track, 0, TRACK_COUNT - 1, 1, Kind::Choice, trackName},
    {"STEERING SENSITIVITY", "steer_sensitivity", &Settings::steerSensitivity, 50, 150, 5, Kind::Percent},
    {"STEERING SPEED", "steer_speed", &Settings::steerSpeed, 50, 200, 10, Kind::Percent},
    {"GAMEPAD DEADZONE", "pad_deadzone", &Settings::padDeadzone, 0, 30, 2, Kind::Percent},
    {"GAMEPAD VIBRATION", "vibration", &Settings::vibration, 0, 1, 1, Kind::Toggle},
    {"CAMERA MODE", "camera_mode", &Settings::cameraMode, 0, CAM_COUNT - 1, 1, Kind::Choice, cameraName},
    {"CAMERA ZOOM", "camera_zoom", &Settings::cameraZoom, 60, 150, 5, Kind::Percent},
    {"CAMERA LOOK-AHEAD", "look_ahead", &Settings::lookAhead, 0, 150, 10, Kind::Percent},
    {"ZOOM OUT WITH SPEED", "speed_zoom", &Settings::speedZoom, 0, 1, 1, Kind::Toggle},
    {"SPEED UNITS", "mph", &Settings::mph, 0, 1, 1, Kind::Units},
    {"SOUND VOLUME", "volume", &Settings::volume, 0, 100, 10, Kind::Percent},
    {"RACING LINE", "racing_line", &Settings::racingLine, 0, 1, 1, Kind::Toggle},
    {"GHOST CAR", "ghost", &Settings::ghost, 0, 3, 1, Kind::Choice, ghostName},
    {"MINIMAP", "minimap", &Settings::minimap, 0, 1, 1, Kind::Toggle},
    {"SKID MARKS", "skid_marks", &Settings::skidMarks, 0, 1, 1, Kind::Toggle},
    {"FULLSCREEN", "fullscreen", &Settings::fullscreen, 0, 1, 1, Kind::Toggle},
    {"VSYNC", "vsync", &Settings::vsync, 0, 1, 1, Kind::Toggle},
    {"WATCH BEST LAP", nullptr, nullptr, 0, 0, 0, Kind::Replay},
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
    float bestTime = -1;  // the current circuit's record, for the replay row
};

// Menu geometry, shared by drawing and mouse hit-testing.
constexpr float MENU_X = 270, MENU_W = 740, MENU_Y = 14, MENU_ROW = 28;
constexpr float MENU_VALUE_RIGHT = MENU_X + MENU_W - 30;
float menuRowY(int i) { return MENU_Y + 64 + i * MENU_ROW; }

std::string menuValue(const MenuEntry& m, const Settings& s, const Menu& menu) {
    switch (m.kind) {
        case Kind::Percent: return "< " + std::to_string(s.*m.field) + "% >";
        case Kind::Choice: return std::string("< ") + m.choiceName(s.*m.field) + " >";
        case Kind::Toggle: return s.*m.field ? "ON" : "OFF";
        case Kind::Units: return s.*m.field ? "MPH" : "KM/H";
        case Kind::Replay: return menu.bestTime > 0 ? fmtTime(menu.bestTime) : "NO LAP YET";
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

bool hasArrows(const MenuEntry& m) { return m.kind == Kind::Percent || m.kind == Kind::Choice; }

// For a row with arrows: -1 if x is on the "<" arrow, +1 on ">", 0 elsewhere.
int menuArrowAt(const MenuEntry& m, const Settings& s, const Menu& menu, float x) {
    if (!hasArrows(m)) return 0;
    float left = MENU_VALUE_RIGHT - textWidth(menuValue(m, s, menu), 3);
    if (std::fabs(x - (left + 4)) < 22) return -1;
    if (x > MENU_VALUE_RIGHT - 30) return 1;
    return 0;
}

void drawMenu(SDL_Renderer* r, const Menu& menu, const Settings& s) {
    fillRect(r, 0, 0, SCREEN_W, SCREEN_H, {0, 0, 0, 110});
    fillRect(r, MENU_X, MENU_Y, MENU_W, 64 + MENU_COUNT * MENU_ROW + 44, {10, 10, 16, 235});
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
    drawText(r, SCREEN_W / 2, menuRowY(MENU_COUNT) + 6, 2,
             "CLICK < > OR SCROLL TO CHANGE   CLICK TO TOGGLE   KEYS: ARROWS + ENTER   ESC CLOSE",
             {170, 170, 170, 255}, 1);
}

// Where a recorded lap's car is at this point of the current lap, if it's still going.
const GhostSample* ghostSample(const Game& g, const std::vector<GhostSample>& ghost) {
    return g.lapStarted && g.lapSteps < int(ghost.size()) ? &ghost[g.lapSteps] : nullptr;
}
const SDL_Color BEST_GHOST{150, 200, 255, 255}, AI_GHOST{255, 200, 60, 255};

// The world: ground, track, scenery, skid marks, racing line, smoke and cars.
void drawWorld(SDL_Renderer* r, const Game& g, const View& view, const std::vector<WorldQuad>& quads,
               const Settings& set, int ghosts, const Effects& fx, std::vector<SDL_Vertex>& verts) {
    SDL_SetRenderDrawColor(r, 40, 96, 48, 255);
    SDL_RenderClear(r);

    // Wide mowing stripes on the ground beyond the walls, so there's always
    // something moving past even far from the track.
    verts.clear();
    {
        Vec2 lo{1e9f, 1e9f}, hi{-1e9f, -1e9f};
        for (Vec2 s : {Vec2{0, 0}, Vec2{SCREEN_W, 0}, Vec2{0, SCREEN_H}, Vec2{SCREEN_W, SCREEN_H}}) {
            Vec2 w = view.toWorld(s);
            lo = {std::min(lo.x, w.x), std::min(lo.y, w.y)};
            hi = {std::max(hi.x, w.x), std::max(hi.y, w.y)};
        }
        const float STRIPE = 260;
        for (int k = int(std::floor(lo.x / STRIPE)); k * STRIPE < hi.x; ++k) {
            if (k % 2 == 0) continue;
            float x0 = k * STRIPE, x1 = x0 + STRIPE;
            pushQuad(verts, view.toScreen({x0, lo.y}), view.toScreen({x1, lo.y}), view.toScreen({x1, hi.y}),
                     view.toScreen({x0, hi.y}), {44, 104, 52, 255});
        }
    }

    // Track and scenery (culled to what's roughly on screen).
    float cull = std::hypot(SCREEN_W * 0.5f, SCREEN_H * 0.5f) / view.zoom + 450;
    for (const WorldQuad& q : quads) {
        if (length(q.v[0] - view.cam) > cull) continue;
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

    // Racing line ahead of the car: green to accelerate, yellow at the
    // cornering limit, red to brake. It fades out with distance.
    verts.clear();
    if (set.racingLine) {
        const Track& t = g.track;
        const int N = t.size(), AHEAD = 170;
        const float w = std::max(3.0f, 5.0f * view.zoom);
        for (int k = -10; k < AHEAD; ++k) {
            int i = ((g.idx + k) % N + N) % N, j = (i + 1) % N;
            Vec2 a = view.toScreen(t.line[i]), b = view.toScreen(t.line[j]);
            Vec2 n = normalized(b - a);
            n = Vec2{-n.y, n.x} * w;
            SDL_Color c = t.lineZone[i] == 2 ? SDL_Color{235, 60, 50, 255}
                          : t.lineZone[i] == 1 ? SDL_Color{255, 200, 40, 255}
                                               : SDL_Color{60, 220, 90, 255};
            c.a = Uint8(210 * std::clamp(1.0f - float(k) / AHEAD, 0.0f, 1.0f));
            pushQuad(verts, a - n, b - n, b + n, a + n, c);
        }
    }

    // Smoke and dust: puffs that grow and fade.
    for (const Particle& p : fx.parts) {
        float t = p.age / p.life, s = p.size * (1.0f + 2.5f * t) * view.zoom;
        Vec2 c = view.toScreen(p.pos);
        SDL_Color col = p.c;
        col.a = Uint8(110 * (1.0f - t));
        pushQuad(verts, c + Vec2{-s, -s}, c + Vec2{s, -s}, c + Vec2{s, s}, c + Vec2{-s, s}, col);
    }

    // Ghosts (the AI's lap in gold, your best in blue), then the player.
    if (ghosts & GHOST_AI)
        if (const GhostSample* s = ghostSample(g, g.aiGhost)) pushCar(verts, view, {s->x, s->y}, s->h, 0, AI_GHOST, 90);
    if (ghosts & GHOST_BEST)
        if (const GhostSample* s = ghostSample(g, g.bestGhost)) pushCar(verts, view, {s->x, s->y}, s->h, 0, BEST_GHOST, 90);
    pushCarShadow(verts, view, g.car.pos, g.car.heading);
    pushCar(verts, view, g.car.pos, g.car.heading, g.car.steer, {220, 30, 40, 255}, 255, g.drsOpen);
    SDL_RenderGeometry(r, nullptr, verts.data(), int(verts.size()), nullptr, 0);
}

// Everything over the world: minimap, timing, dash, banners and hints.
void drawHud(SDL_Renderer* r, const Game& g, const Minimap& mini, const Settings& set, const Menu& menu,
             bool paused, bool replay, bool replayPaused) {
    int ghosts = replay ? 0 : set.ghost;

    // Minimap.
    if (set.minimap) {
        fillRect(r, SCREEN_W - 270, 10, 260, 286, {0, 0, 0, 130});
        drawText(r, SCREEN_W - 140, 274, 2.5f, TRACKS[g.trackId].name, {170, 170, 170, 255}, 1);
        SDL_SetRenderDrawColor(r, 200, 200, 200, 255);
        std::vector<SDL_FPoint> shifted = mini.line;
        for (int d = 0; d < 2; ++d) {
            SDL_RenderDrawLinesF(r, shifted.data(), int(shifted.size()));
            for (auto& p : shifted) p.x += 1;
        }
        SDL_SetRenderDrawColor(r, GREEN.r, GREEN.g, GREEN.b, 255);
        for (std::vector<SDL_FPoint> zone : mini.drs)
            for (int d = 0; d < 3; ++d) {
                SDL_RenderDrawLinesF(r, zone.data(), int(zone.size()));
                for (auto& p : zone) p.y += 1;
            }
        SDL_FPoint sl = mini.map(g.track.pts[0]);
        fillRect(r, sl.x - 2, sl.y - 5, 4, 10, WHITE);
        for (int bit : {GHOST_AI, GHOST_BEST}) {
            const GhostSample* s = ghosts & bit ? ghostSample(g, bit == GHOST_AI ? g.aiGhost : g.bestGhost) : nullptr;
            if (!s) continue;
            SDL_FPoint p = mini.map({s->x, s->y});
            fillRect(r, p.x - 3, p.y - 3, 6, 6, bit == GHOST_AI ? AI_GHOST : BEST_GHOST);
        }
        SDL_FPoint cp = mini.map(g.car.pos);
        fillRect(r, cp.x - 4, cp.y - 4, 8, 8, RED);
    }

    // Timing panel.
    const float S = 3;
    fillRect(r, 10, 10, 360, 270, {0, 0, 0, 140});
    float y = 22;
    if (replay) drawText(r, 24, y, S, "REPLAY: BEST LAP", GOLD);
    else drawText(r, 24, y, S, g.lapStarted ? "LAP " + std::to_string(g.lapNumber) : "OUT LAP", GOLD);
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
    y += 26;
    drawText(r, 24, y, S, "IDEAL", {170, 170, 170, 255});
    drawText(r, 356, y, S, fmtTime(g.idealLap()), {170, 170, 170, 255}, 2);
    y += 26;
    // The medal your best lap has earned, and the time for the next one.
    int medal = medalFor(g.trackId, g.bestTime);
    drawText(r, 24, y, S, medal ? MEDAL_NAMES[medal] : "NO MEDAL", MEDAL_COLORS[medal]);
    if (medal < MEDAL_PLATINUM)
        drawText(r, 356, y, S, std::string(MEDAL_NAMES[medal + 1]) + " " + fmtTime(medalTime(g.trackId, medal + 1)),
                 MEDAL_COLORS[medal + 1], 2);
    y += 36;

    // Sector times for the current lap (or the one just finished, for a few seconds).
    // Purple marks a sector that's the fastest you've ever driven it.
    bool showLast = g.splitsDone == 0 && g.lastLap > 0 && g.lapSteps < 360;
    const float* sp = showLast ? g.lastSplits : g.splits;
    int done = showLast ? 3 : g.splitsDone;
    bool sectorsValid = showLast ? !g.history.empty() && g.history.back().valid : g.lapValid;
    for (int k = 0; k < 3; ++k, y += 26) {
        drawText(r, 24, y, S, "S" + std::to_string(k + 1), WHITE);
        if (k >= done) {
            drawText(r, 200, y, S, "--.---", {150, 150, 150, 255}, 2);
            continue;
        }
        float st = g.sectorTime(sp, k);
        float ever = g.bestEverSectors[k];
        bool purple = sectorsValid && (ever <= 0 || st <= ever + 0.0005f);
        char buf[16];
        std::snprintf(buf, sizeof buf, "%.3f", st);
        drawText(r, 200, y, S, buf, purple ? PURPLE : WHITE, 2);
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

    // Speedo: rev lights, speed, gear, and throttle / brake bars.
    Vec2 fwd{std::cos(g.car.heading), std::sin(g.car.heading)};
    float vf = dot(g.car.vel, fwd);
    float kmh = std::fabs(vf) * 0.36f;
    int shownSpeed = int((set.mph ? kmh * 0.621371f : kmh) + 0.5f);
    std::string gear = g.gear < 0 ? "R" : g.gear == 0 ? "N" : std::to_string(g.gear);
    const float cx = SCREEN_W / 2, top = SCREEN_H - 124;
    fillRect(r, cx - 130, top, 260, 114, {0, 0, 0, 140});

    // 15 lights fill green, red, then blue up to the shift point; all blue flashing means shift.
    const int LIGHTS = 15;
    int lit = int(std::clamp((g.rpm - 8800.0f) / (SHIFT_RPM - 8800.0f), 0.0f, 1.0f) * LIGHTS);
    bool shiftNow = lit >= LIGHTS && g.gear > 0 && g.gear < GEARS;
    bool flashOn = (g.stepCount / 8) % 2 == 0;
    for (int k = 0; k < LIGHTS; ++k) {
        SDL_Color c = k < 5 ? SDL_Color{60, 220, 90, 255} : k < 10 ? SDL_Color{235, 50, 45, 255}
                                                                   : SDL_Color{80, 120, 255, 255};
        if (shiftNow) c = flashOn ? SDL_Color{80, 120, 255, 255} : SDL_Color{40, 40, 50, 255};
        else if (k >= lit) c = {40, 40, 50, 255};
        fillRect(r, cx - 119 + k * 16, top + 8, 12, 8, c);
    }

    drawText(r, cx + 40, top + 30, 8, std::to_string(shownSpeed), WHITE, 2);
    drawText(r, cx + 40, top + 78, 2.5f, set.mph ? "MPH" : "KM/H", {180, 180, 180, 255}, 2);
    // DRS light: dim outside a zone, outlined when it can open, green when open.
    bool drsReady = g.track.drsZone[g.idx] && !g.onGrass;
    SDL_Color drsCol = g.drsOpen ? GREEN : drsReady ? WHITE : SDL_Color{70, 70, 80, 255};
    if (g.drsOpen) fillRect(r, cx + 56, top + 84, 52, 20, GREEN);
    else {
        fillRect(r, cx + 56, top + 84, 52, 2, drsCol);
        fillRect(r, cx + 56, top + 102, 52, 2, drsCol);
        fillRect(r, cx + 56, top + 84, 2, 20, drsCol);
        fillRect(r, cx + 106, top + 84, 2, 20, drsCol);
    }
    drawText(r, cx + 82, top + 88, 2.5f, "DRS", g.drsOpen ? SDL_Color{10, 30, 10, 255} : drsCol, 1);
    fillRect(r, cx + 56, top + 26, 52, 52, {255, 255, 255, 20});
    drawText(r, cx + 82, top + 32, 8, gear, GOLD, 1);
    float tb = g.lastInput.throttle * 74, bb = g.lastInput.brake * 74;
    fillRect(r, cx - 120, SCREEN_H - 20 - tb, 10, tb, GREEN);
    fillRect(r, cx - 104, SCREEN_H - 20 - bb, 10, bb, RED);

    // Live gap to the best lap at this point on track.
    float delta;
    if (replay) {
        const char* state = replayPaused ? "REPLAY PAUSED" : "REPLAY";
        float w = textWidth("SPACE PAUSE  LEFT/RIGHT SEEK  ESC EXIT", 2);
        fillRect(r, SCREEN_W / 2 - w / 2 - 16, 10, w + 32, 54, {0, 0, 0, 150});
        drawText(r, SCREEN_W / 2, 18, 4, state, (g.stepCount / 40) % 2 || replayPaused ? GOLD : WHITE, 1);
        drawText(r, SCREEN_W / 2, 44, 2, "SPACE PAUSE  LEFT/RIGHT SEEK  ESC EXIT", {170, 170, 170, 255}, 1);
    } else if (g.liveDelta(delta)) {
        std::string text = fmtDelta(delta);
        float w = std::max(textWidth(text, 5), textWidth("DELTA", 2));
        fillRect(r, SCREEN_W / 2 - w / 2 - 16, 10, w + 32, 54, {0, 0, 0, 150});
        drawText(r, SCREEN_W / 2, 16, 2, "DELTA", {170, 170, 170, 255}, 1);
        drawText(r, SCREEN_W / 2, 32, 5, text, delta <= 0 ? GREEN : RED, 1);
    }

    if (g.onGrass) drawText(r, SCREEN_W / 2, SCREEN_H - 150, 3, "OFF TRACK", RED, 1);
    if (g.wrongWay && (g.stepCount / 30) % 2 == 0) {
        fillRect(r, SCREEN_W / 2 - 190, SCREEN_H / 2 - 150, 380, 64, {120, 0, 0, 180});
        drawText(r, SCREEN_W / 2, SCREEN_H / 2 - 138, 8, "WRONG WAY", WHITE, 1);
    }

    if (g.messageTimer > 0 && !menu.open) {
        float w = textWidth(g.message, 4);
        fillRect(r, SCREEN_W / 2 - w / 2 - 16, 70, w + 32, 44, {0, 0, 0, 160});
        drawText(r, SCREEN_W / 2, 80, 4, g.message, g.messageColor, 1);
    }
    if (paused && !menu.open) {
        fillRect(r, 0, 0, SCREEN_W, SCREEN_H, {0, 0, 0, 120});
        drawText(r, SCREEN_W / 2, SCREEN_H / 2 - 30, 8, "PAUSED", WHITE, 1);
        drawText(r, SCREEN_W / 2, SCREEN_H / 2 + 20, 3, "PRESS P TO RESUME", {200, 200, 200, 255}, 1);
    }

    // Two short lines in the corner so they stay clear of the speedo (the
    // replay banner has its own).
    if (!replay) {
        fillRect(r, 6, SCREEN_H - 48, 450, 42, {0, 0, 0, 110});
        drawText(r, 14, SCREEN_H - 40, 2, "WASD/ARROWS DRIVE  SPACE HANDBRAKE  R RESTART  P PAUSE", {220, 220, 220, 200});
        drawText(r, 14, SCREEN_H - 22, 2, "C CAMERA  G GHOST  L LINE  V REPLAY  ESC MENU", {220, 220, 220, 200});
    }
}

// Title screen: pick a circuit while a demo car laps it behind the cards.
struct TitleCard {
    Minimap outline;
    float best = -1;
};

// The cards sit low so the demo car, in the middle of the screen, stays in view.
constexpr float CARD_W = 300, CARD_H = 218, CARD_GAP = 30, CARD_Y = 406, CARD_MAP = 130;
float cardX(int i) {
    return (SCREEN_W - (TRACK_COUNT * CARD_W + (TRACK_COUNT - 1) * CARD_GAP)) / 2 + i * (CARD_W + CARD_GAP);
}
int cardAt(float x, float y) {
    for (int i = 0; i < TRACK_COUNT; ++i)
        if (x >= cardX(i) && x < cardX(i) + CARD_W && y >= CARD_Y && y < CARD_Y + CARD_H) return i;
    return -1;
}

void drawTitle(SDL_Renderer* r, const std::vector<TitleCard>& cards, int sel, long tick) {
    fillRect(r, 0, 0, SCREEN_W, SCREEN_H, {0, 0, 0, 70});
    fillRect(r, 0, 36, SCREEN_W, 172, {0, 0, 0, 190});
    drawText(r, SCREEN_W / 2, 56, 16, "F1 HOTLAP", WHITE, 1);
    fillRect(r, SCREEN_W / 2 - 300, 150, 600, 6, RED);
    drawText(r, SCREEN_W / 2, 172, 3, "TOP-DOWN TIME TRIAL", GOLD, 1);

    for (int i = 0; i < TRACK_COUNT; ++i) {
        float x = cardX(i), y = CARD_Y;
        bool on = i == sel;
        fillRect(r, x, y, CARD_W, CARD_H, on ? SDL_Color{22, 22, 30, 235} : SDL_Color{0, 0, 0, 170});
        if (on) {
            fillRect(r, x, y, CARD_W, 4, GOLD);
            fillRect(r, x, y + CARD_H - 4, CARD_W, 4, GOLD);
            fillRect(r, x, y, 4, CARD_H, GOLD);
            fillRect(r, x + CARD_W - 4, y, 4, CARD_H, GOLD);
        }
        std::vector<SDL_FPoint> line = cards[i].outline.line;
        Uint8 shade = on ? 240 : 150;
        SDL_SetRenderDrawColor(r, shade, shade, shade, 255);
        for (int d = 0; d < 2; ++d) {
            SDL_RenderDrawLinesF(r, line.data(), int(line.size()));
            for (auto& p : line) p.x += 1;
        }
        SDL_FPoint start = cards[i].outline.line.front();
        fillRect(r, start.x - 3, start.y - 3, 7, 7, RED);
        drawText(r, x + CARD_W / 2, y + 148, 3, TRACKS[i].name, on ? GOLD : WHITE, 1);
        if (cards[i].best > 0) drawText(r, x + CARD_W / 2, y + 174, 2.5f, "BEST " + fmtTime(cards[i].best), PURPLE, 1);
        else drawText(r, x + CARD_W / 2, y + 174, 2.5f, "NO TIME YET", {150, 150, 150, 255}, 1);
        int medal = medalFor(i, cards[i].best);
        std::string medalText = medal == MEDAL_PLATINUM ? "PLATINUM MEDAL"
                                : medal ? std::string(MEDAL_NAMES[medal]) + "  NEXT " + fmtTime(medalTime(i, medal + 1))
                                        : "BRONZE AT " + fmtTime(medalTime(i, MEDAL_BRONZE));
        drawText(r, x + CARD_W / 2, y + 196, 2.5f, medalText, MEDAL_COLORS[medal], 1);
    }

    fillRect(r, 0, 632, SCREEN_W, 76, {0, 0, 0, 120});
    if ((tick / 60) % 2 == 0) drawText(r, SCREEN_W / 2, 644, 4, "PRESS ENTER TO DRIVE", WHITE, 1);
    drawText(r, SCREEN_W / 2, 684, 2, "LEFT/RIGHT OR CLICK TO CHOOSE A CIRCUIT   ESC SETTINGS", {190, 190, 190, 255}, 1);
}

void render(SDL_Renderer* r, const Game& g, const View& view, const std::vector<WorldQuad>& quads,
            const Minimap& mini, const Settings& set, const Menu& menu, bool paused, bool replay,
            bool replayPaused, const Effects& fx, std::vector<SDL_Vertex>& verts) {
    drawWorld(r, g, view, quads, set, replay ? 0 : set.ghost, fx, verts);  // in a replay the car is the ghost
    drawHud(r, g, mini, set, menu, paused, replay, replayPaused);
    if (menu.open) drawMenu(r, menu, set);
}

// Gamepad rumble from what the tyres are doing. Only sent when it changes, or
// to keep it going, since some controllers are slow to take updates.
struct Rumble {
    float low = 0, high = 0;
    Uint32 sentAt = 0;

    void update(SDL_GameController* pad, const Game& g, const Effects& fx, bool active) {
        if (!pad) return;
        float speed = std::min(length(g.car.vel) / 400.0f, 1.0f), l = 0, h = 0;
        if (active) {
            if (g.onKerb) {
                l = 0.35f * speed;
                h = 0.3f * speed;
            }
            if (g.onGrass) l = std::max(l, 0.5f * speed);
            h = std::max(h, std::clamp((std::fabs(g.car.slip) - 150.0f) / 400.0f, 0.0f, 0.5f));
            l = std::max(l, std::min(fx.shake / 10.0f, 1.0f));  // wall hits
        }
        Uint32 now = SDL_GetTicks();
        bool changed = std::fabs(l - low) > 0.03f || std::fabs(h - high) > 0.03f;
        bool refresh = (l > 0 || h > 0) && now - sentAt > 80;
        if (!changed && !refresh) return;
        low = l;
        high = h;
        sentAt = now;
        SDL_GameControllerRumble(pad, Uint16(l * 65535), Uint16(h * 65535), 150);
    }
};

Input readInput(SDL_GameController* pad, const Settings& set) {
    const Uint8* k = SDL_GetKeyboardState(nullptr);
    Input in;
    in.throttle = (k[SDL_SCANCODE_W] || k[SDL_SCANCODE_UP]) ? 1.0f : 0.0f;
    in.brake = (k[SDL_SCANCODE_S] || k[SDL_SCANCODE_DOWN]) ? 1.0f : 0.0f;
    in.steer = float((k[SDL_SCANCODE_D] || k[SDL_SCANCODE_RIGHT]) - (k[SDL_SCANCODE_A] || k[SDL_SCANCODE_LEFT]));
    in.handbrake = k[SDL_SCANCODE_SPACE];
    in.drs = k[SDL_SCANCODE_E];
    if (pad) {
        auto axis = [&](SDL_GameControllerAxis a) { return SDL_GameControllerGetAxis(pad, a) / 32767.0f; };
        float lx = axis(SDL_CONTROLLER_AXIS_LEFTX);
        float dz = set.padDeadzone / 100.0f;
        if (std::fabs(lx) > dz) in.steer = std::clamp(signOf(lx) * (std::fabs(lx) - dz) / (1.0f - dz), -1.0f, 1.0f);
        in.throttle = std::max(in.throttle, axis(SDL_CONTROLLER_AXIS_TRIGGERRIGHT));
        in.brake = std::max(in.brake, axis(SDL_CONTROLLER_AXIS_TRIGGERLEFT));
        in.handbrake = in.handbrake || SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_A);
        in.drs = in.drs || SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
    }
    return in;
}

float lerpAngle(float a, float b, float t) { return a + std::remainder(b - a, 2.0f * float(M_PI)) * t; }

// Moves the camera for the chosen mode. Smoothing is time-based so it feels
// the same at any frame rate; snap jumps straight to the target.
void updateCamera(View& view, const Game& g, const Settings& set, float frameDt, bool snap) {
    float speed = length(g.car.vel);
    float userZoom = set.cameraZoom / 100.0f, look = set.lookAhead / 100.0f;
    float speedFactor = set.speedZoom ? 1.0f - 0.25f * std::min(speed / TOP_SPEED, 1.0f) : 1.0f;
    float posRate = 8.0f, angleTarget = 0, zoomTarget = userZoom * speedFactor;
    Vec2 target = g.car.pos + g.car.vel * (0.35f * look);

    switch (set.cameraMode) {
        case CAM_CHASE: {
            // Rotates with the car so it always points up; car sits low on screen.
            angleTarget = -g.car.heading - float(M_PI) / 2;
            float a = -(snap ? angleTarget : view.angle) - float(M_PI) / 2;  // world direction of screen-up
            target = g.car.pos + Vec2{std::cos(a), std::sin(a)} * (SCREEN_H * 0.22f * look / view.zoom);
            posRate = 20.0f;
            break;
        }
        case CAM_LOCKED:
            target = g.car.pos;
            zoomTarget = userZoom;
            break;
        case CAM_HELICOPTER:
            zoomTarget = 0.42f * userZoom;
            posRate = 3.0f;
            break;
        default: break;
    }

    auto blend = [&](float rate) { return snap ? 1.0f : 1.0f - std::exp(-rate * frameDt); };
    view.zoom += (zoomTarget - view.zoom) * blend(3.0f);
    view.setAngle(lerpAngle(view.angle, angleTarget, blend(6.0f)));
    view.cam = set.cameraMode == CAM_LOCKED ? target : view.cam + (target - view.cam) * blend(posRate);
}

// ---------------------------------------------------------------------------
// Self-test (--selftest): runs headless, no window or sound device needed.

// Checks one circuit: geometry, three autopilot laps, the delta, gearbox and saves.
bool testTrack(int id) {
    Game g;
    g.init(id);
    const Track& t = g.track;
    const int N = t.size();
    std::printf("== %s: %d samples, %.0f px long\n", TRACKS[id].name, N, t.length);

    // Sections of track that are far apart along the lap must not overlap.
    float minGap = 1e9f;
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j) {
            float along = std::min(t.arc[j] - t.arc[i], t.length - (t.arc[j] - t.arc[i]));
            if (along > 1200.0f) minGap = std::min(minGap, length(t.pts[i] - t.pts[j]));
        }
    std::printf("closest separate sections: %.0f px (need > %.0f)\n", minGap, 2 * WALL_D);
    bool ok = minGap > 2 * WALL_D;

    // Corners tighter than this fold the kerbs and asphalt over themselves.
    float maxCurv = *std::max_element(t.curvature.begin(), t.curvature.end());
    const float minRadius = HALF_W + KERB_W + 25;
    std::printf("tightest corner: %.0f px radius (need > %.0f)\n", 1 / maxCurv, minRadius);
    ok = ok && 1 / maxCurv > minRadius;

    // The racing line should open the corners up well beyond the centre line,
    // and stay on the asphalt. Curvature is averaged over 9 samples, as for the centre line.
    std::vector<float> raw(N);
    float lineCurv = 0, lineOff = 0;
    for (int i = 0; i < N; ++i) {
        Vec2 u = t.line[i] - t.line[(i - 1 + N) % N], v = t.line[(i + 1) % N] - t.line[i];
        raw[i] = std::fabs(std::atan2(cross(u, v), dot(u, v))) / (0.5f * (length(u) + length(v)));
        lineOff = std::max(lineOff, std::fabs(dot(t.line[i] - t.pts[i], t.normals[i])));
    }
    for (int i = 0; i < N; ++i) {
        float k = 0;
        for (int j = -4; j <= 4; ++j) k += raw[((i + j) % N + N) % N];
        lineCurv = std::max(lineCurv, k / 9);
    }
    std::printf("racing line: tightest %.0f px radius (need > 1.3x centre), widest %.0f px off centre (need < %.0f)\n",
                1 / lineCurv, lineOff, HALF_W);
    ok = ok && 1 / lineCurv > 1.3f / maxCurv && lineOff < HALF_W;

    // Laps 2 and 3 are driven identically, so the live delta on lap 3 should hover near zero.
    float maxDelta = 0;
    int deltaSamples = 0;
    int topGear = 0, shifts = 0;
    float rpmLo = 1e9f, rpmHi = 0;
    for (long s = 0; s < 120L * 600 && g.history.size() < 3; ++s) {
        int gearBefore = g.gear;
        g.step(autopilot(g));
        topGear = std::max(topGear, g.gear);
        shifts += g.gear != gearBefore;
        rpmLo = std::min(rpmLo, g.rpm);
        rpmHi = std::max(rpmHi, g.rpm);
        float d;
        if (g.history.size() == 2 && g.liveDelta(d)) {
            maxDelta = std::max(maxDelta, std::fabs(d));
            ++deltaSamples;
        }
    }
    for (auto& e : g.history)
        std::printf("lap %d: %s %s\n", e.number, fmtTime(e.time).c_str(), e.valid ? "valid" : "INVALID");
    bool allValid = g.history.size() == 3;
    for (auto& e : g.history) allValid = allValid && e.valid;
    ok = ok && allValid && g.bestGhost.size() > 0;
    std::printf("timing beeps: %d over %zu laps (need 3 a lap)\n", g.chimes, g.history.size());
    ok = ok && g.chimes == 3 * int(g.history.size());
    std::printf("best: %s, ghost samples: %zu\n", fmtTime(g.bestTime).c_str(), g.bestGhost.size());

    std::printf("lap 3 live delta: max %.3f s over %d samples (need < 0.05)\n", maxDelta, deltaSamples);
    ok = ok && deltaSamples > int(g.bestGhost.size() * 9 / 10) && maxDelta < 0.05f;
    float ideal = g.idealLap();
    std::printf("ideal lap: %s (need <= best)\n", fmtTime(ideal).c_str());
    ok = ok && ideal > 0 && ideal <= g.bestTime + 1e-4f;

    std::printf("gearbox: top gear %d, %d shifts, rpm %.0f-%.0f\n", topGear, shifts, rpmLo, rpmHi);
    ok = ok && topGear >= 5 && shifts >= topGear && rpmLo >= IDLE_RPM - 1 && rpmHi <= MAX_RPM + 1;

    // Every circuit has DRS on at least one straight, and using it saves time.
    int zones = 0;
    float zoneLen = 0;
    for (int i = 0; i < N; ++i) {
        zones += t.drsZone[i] && !t.drsZone[(i - 1 + N) % N];
        if (t.drsZone[i]) zoneLen += length(t.pts[(i + 1) % N] - t.pts[i]);
    }
    Game noDrs;
    noDrs.init(id);
    for (long s = 0; s < 120L * 600 && noDrs.history.size() < 2; ++s) noDrs.step(autopilot(noDrs, false));
    std::printf("drs: %d zone(s), %.0f px; best lap %s with it, %s without\n", zones, zoneLen,
                fmtTime(g.bestTime).c_str(), fmtTime(noDrs.bestTime).c_str());
    ok = ok && zones >= 1 && noDrs.bestTime > g.bestTime;

    // Medal times hang off the autopilot's lap. If the handling changes, update aiLap.
    float drift = std::fabs(g.bestTime - TRACKS[id].aiLap) / TRACKS[id].aiLap;
    std::printf("medals: autopilot %s vs aiLap %s (%.2f%% off, need < 1%%), earned %s\n", fmtTime(g.bestTime).c_str(),
                fmtTime(TRACKS[id].aiLap).c_str(), drift * 100, MEDAL_NAMES[medalFor(id, g.bestTime)]);
    ok = ok && drift < 0.01f && medalFor(id, g.bestTime) == MEDAL_GOLD;

    // The AI ghost is that same pace, recorded from a fresh session.
    Game fresh;
    fresh.init(id);
    float aiGhostLap = aiLapGhost(fresh).size() * DT;
    std::printf("ai ghost: %s (need within 1%% of aiLap)\n", fmtTime(aiGhostLap).c_str());
    ok = ok && std::fabs(aiGhostLap - TRACKS[id].aiLap) / TRACKS[id].aiLap < 0.01f && fresh.history.empty();

    // Save round trip, in the current format and the old v1 one (no best-ever sectors).
    for (uint32_t version : {SAVE_VERSION, 1u}) {
        FILE* f = std::tmpfile();
        if (!f) return false;
        if (version == SAVE_VERSION) {
            g.writeSave(f);
        } else {
            uint32_t header[4] = {SAVE_MAGIC, 1, g.trackSignature(), uint32_t(g.bestGhost.size())};
            std::fwrite(header, sizeof header, 1, f);
            std::fwrite(&g.bestTime, sizeof g.bestTime, 1, f);
            std::fwrite(g.bestSectors, sizeof g.bestSectors, 1, f);
            std::fwrite(g.bestGhost.data(), sizeof(GhostSample), g.bestGhost.size(), f);
        }
        std::rewind(f);
        Game h;
        h.init(id);
        bool loaded = h.readSave(f);
        std::fclose(f);
        const float* wantEver = version == SAVE_VERSION ? g.bestEverSectors : g.bestSectors;
        bool same = loaded && h.bestTime == g.bestTime && h.bestGhost.size() == g.bestGhost.size() &&
                    h.bestDist.size() == g.bestDist.size() && std::equal(wantEver, wantEver + 3, h.bestEverSectors);
        std::printf("save v%u round trip: %s\n", version, same ? "ok" : "MISMATCH");
        ok = ok && same;
    }

    // A record from one circuit must not load on another.
    if (TRACK_COUNT > 1) {
        FILE* f = std::tmpfile();
        if (!f) return false;
        g.writeSave(f);
        std::rewind(f);
        Game other;
        other.init((id + 1) % TRACK_COUNT);
        bool rejected = !other.readSave(f) && other.bestTime < 0;
        std::fclose(f);
        std::printf("save rejected on %s: %s\n", TRACKS[other.trackId].name, rejected ? "ok" : "LOADED");
        ok = ok && rejected;
    }
    return ok;
}

int runSelfTest() {
    bool ok = true;
    for (int id = 0; id < TRACK_COUNT; ++id) ok = testTrack(id) && ok;

    // Replaying a best lap drives the puppet all the way round and over the line.
    {
        Game g;
        g.init(0);
        for (long s = 0; s < 120L * 300 && g.history.empty(); ++s) g.step(autopilot(g));
        Game rp = g;
        int topGear = 0, grass = 0;
        float maxSteer = 0;
        for (int k = 0; k < int(rp.bestGhost.size()); ++k) {
            rp.puppetTo(rp.bestGhost, k);
            topGear = std::max(topGear, rp.gear);
            grass += rp.onGrass;
            maxSteer = std::max(maxSteer, std::fabs(rp.car.steer));
        }
        bool fine = !g.bestGhost.empty() && topGear >= 6 && grass == 0 && rp.idx < CROSS_BAND && maxSteer <= 1;
        std::printf("== replay: top gear %d, %d steps on grass, ends at sample %d: %s\n", topGear, grass, rp.idx,
                    fine ? "ok" : "WRONG");
        ok = ok && fine;
    }

    // Fresh effects (after a replay or track change) must not shake for old wall hits.
    {
        Game g;
        g.init(0);
        g.wallHits = 5;
        g.lastImpact = 500;
        Effects fx;
        fx.update(g, DT);
        bool calm = fx.shake == 0;
        ++g.wallHits;
        fx.update(g, DT);
        bool shook = fx.shake > 0;
        std::printf("== camera shake: %s on old hits, %s on a new one\n", calm ? "none" : "SHOOK", shook ? "shakes" : "NONE");
        ok = ok && calm && shook;
    }

    // Turning round just after the line: warn, then abandon the lap on crossing
    // back, without logging a junk lap.
    {
        Game g;
        g.init(0);
        g.placeAt(30);
        g.startLap();
        g.car.heading += float(M_PI);
        Input in;
        in.throttle = 1;
        bool warned = false;
        for (int s = 0; s < 240 && g.lapStarted; ++s) {
            g.step(in);
            warned = warned || g.wrongWay;
        }
        bool abandoned = !g.lapStarted && g.history.empty();
        std::printf("== reversing over the line: warned %s, abandoned %s\n", warned ? "yes" : "NO",
                    abandoned ? "yes" : "NO");
        ok = ok && warned && abandoned;
    }

    // The synth must stay finite and inside [-1, 1], and get louder on throttle.
    auto loudness = [](SoundParams p) {
        Synth s;
        s.p = p;
        std::vector<float> buf(48000);
        s.fill(buf.data(), int(buf.size()));
        double sum = 0;
        for (float x : buf) {
            if (!std::isfinite(x) || std::fabs(x) > 1.0f) return -1.0;
            sum += double(x) * x;
        }
        return std::sqrt(sum / buf.size());
    };
    SoundParams idle, flat;
    idle.volume = flat.volume = 1;
    flat.rpm = 11500;
    flat.throttle = 1;
    flat.speed = 800;
    flat.slip = 500;
    flat.kerb = true;
    flat.wallHits = 1;
    flat.impact = 900;
    double rmsIdle = loudness(idle), rmsFlat = loudness(flat);
    std::printf("== synth rms: idle %.3f, flat out %.3f\n", rmsIdle, rmsFlat);
    ok = ok && rmsIdle > 0.01 && rmsFlat > rmsIdle;

    // Each timing beep is heard over the idle once the chime count goes up.
    for (int kind = CHIME_SLOWER; kind <= CHIME_BEST_LAP; ++kind) {
        Synth s;
        s.p = idle;
        std::vector<float> buf(4800);
        auto rms = [&]() {
            s.fill(buf.data(), int(buf.size()));
            double sum = 0;
            for (float x : buf) sum += double(x) * x;
            return std::sqrt(sum / buf.size());
        };
        rms();  // settle the engine
        double before = rms();
        s.p.chimes = 1;
        s.p.chimeKind = kind;
        double during = rms();
        std::printf("== chime %d: rms %.3f over %.3f idle\n", kind, during, before);
        ok = ok && during > before * 1.15;
    }

    std::printf("%s\n", ok ? "SELFTEST PASS" : "SELFTEST FAIL");
    return ok ? 0 : 1;
}

int runGame() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER | SDL_INIT_AUDIO) != 0) {
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
    std::string basePath, settingsPath;
    if (char* base = SDL_GetBasePath()) {
        basePath = base;
        settingsPath = basePath + "settings.cfg";
        SDL_free(base);
    }
    loadSettings(set, settingsPath);

    std::vector<WorldQuad> quads;
    Minimap mini;
    std::vector<SDL_Vertex> verts;
    View view;
    Effects fx;
    bool snapCamera = true;

    Game demo;  // laps the circuit behind the title screen

    // Builds everything for the chosen circuit; the session's lap list starts afresh.
    auto loadTrack = [&]() {
        game = Game{};
        if (!basePath.empty()) game.savePath = basePath + TRACKS[set.track].saveFile;
        game.init(set.track);
        game.aiGhost = aiLapGhost(game);
        demo = game;
        demo.savePath.clear();
        demo.resetToGrid();
        if (game.bestTime > 0) game.flash("BEST LAP TO BEAT " + fmtTime(game.bestTime), PURPLE, 4.0f);
        quads = buildTrackQuads(game);
        mini = Minimap{};
        mini.build(game.track, SCREEN_W - 260, 20, 240);
        view.cam = game.car.pos;
        fx = Effects{};
        snapCamera = true;
    };

    // Push settings into the game, window and renderer.
    auto applySettings = [&]() {
        if (game.track.size() == 0 || game.trackId != set.track) loadTrack();
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

    // Sound is optional: without an audio device the game just runs silent.
    Synth synth;
    SDL_AudioSpec want{}, have{};
    want.freq = 48000;
    want.format = AUDIO_F32SYS;
    want.channels = 1;
    want.samples = 512;
    want.callback = audioCallback;
    want.userdata = &synth;
    SDL_AudioDeviceID audio = SDL_OpenAudioDevice(nullptr, 0, &want, &have, 0);
    if (audio) {
        synth.rate = float(have.freq);
        SDL_PauseAudioDevice(audio, 0);
    } else {
        std::fprintf(stderr, "No sound: %s\n", SDL_GetError());
    }

    SDL_GameController* pad = nullptr;
    Menu menu;
    Rumble rumble;
    bool running = true, paused = false;

    // Title screen, shown at launch. Each card shows a circuit and its record.
    bool title = true;
    double demoAcc = 0;
    std::vector<TitleCard> cards(TRACK_COUNT);
    for (int i = 0; i < TRACK_COUNT; ++i) {
        Game peek;  // just the geometry and the saved record; no need for a racing line
        peek.track.build(circuitLayout(i));
        if (!basePath.empty()) {
            peek.savePath = basePath + TRACKS[i].saveFile;
            peek.load();
        }
        cards[i].best = peek.bestTime;
        cards[i].outline.build(peek.track, cardX(i) + (CARD_W - CARD_MAP) / 2, CARD_Y + 10, CARD_MAP);
    }
    auto titleSelect = [&](int i) {
        i = (i + TRACK_COUNT) % TRACK_COUNT;
        if (set.track == i) return;
        set.track = i;
        changed();  // loads the circuit, demo car included
    };
    auto startDriving = [&]() {
        title = false;
        game.resetToGrid();
        if (game.bestTime > 0) game.flash("BEST LAP TO BEAT " + fmtTime(game.bestTime), PURPLE, 4.0f);
        snapCamera = true;
        fx = Effects{};
    };

    // Best-lap replay: a copy of the game whose car is driven from the ghost.
    Game replay;
    bool replaying = false, replayPaused = false;
    int replayStep = 0;
    double replayAcc = 0;
    auto startReplay = [&]() {
        if (game.bestGhost.empty()) {
            game.flash("NO BEST LAP TO WATCH YET", WHITE);
            return;
        }
        replay = game;
        replay.messageTimer = 0;
        replay.wrongWay = false;
        float t = 0;
        for (int k = 0; k < 3; ++k) replay.splits[k] = t += game.bestSectors[k];
        // When it loops, the lap just shown is the best lap again.
        std::copy(replay.splits, replay.splits + 3, replay.lastSplits);
        replay.lastLap = game.bestTime;
        replay.lapValid = true;
        replayStep = 0;
        replayAcc = 0;
        replay.puppetTo(replay.bestGhost, 0);
        replaying = true;
        replayPaused = paused = menu.open = title = false;
        snapCamera = true;
        fx = Effects{};
    };
    auto stopReplay = [&]() {
        replaying = false;
        snapCamera = true;
        fx = Effects{};
    };
    auto seekReplay = [&](float secs) {
        replayStep = std::clamp(replayStep + int(secs / DT), 0, int(replay.bestGhost.size()) - 1);
        replay.puppetTo(replay.bestGhost, replayStep);
    };

    auto menuMove = [&](int dir) {
        menu.sel = (menu.sel + dir + MENU_COUNT) % MENU_COUNT;
        menu.confirmReset = false;
    };
    auto menuAdjust = [&](int dir) {
        const MenuEntry& m = MENU[menu.sel];
        if (!m.field) return;
        int& v = set.*m.field;
        int nv = m.kind == Kind::Percent  ? std::clamp(v + dir * m.step, m.lo, m.hi)
                 : m.kind == Kind::Choice ? (v + dir - m.lo + (m.hi - m.lo + 1)) % (m.hi - m.lo + 1) + m.lo
                                          : 1 - v;
        if (nv != v) {
            v = nv;
            changed();
        }
    };
    auto menuActivate = [&]() {
        const MenuEntry& m = MENU[menu.sel];
        switch (m.kind) {
            case Kind::Choice:
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
            case Kind::Replay: startReplay(); break;
            case Kind::Resume: menu.open = false; break;
            case Kind::Quit: running = false; break;
            default: break;
        }
    };
    auto restartLap = [&]() {
        game.resetToGrid();
        snapCamera = true;
    };
    auto cycleCamera = [&]() {
        set.cameraMode = (set.cameraMode + 1) % CAM_COUNT;
        changed();
        game.flash(std::string("CAMERA: ") + CAMERA_NAMES[set.cameraMode], WHITE, 1.5f);
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
            // Don't keep the clock running while the player is in another window.
            if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_FOCUS_LOST && !menu.open && !title) {
                if (replaying) replayPaused = true;  // P resumes a replay, so pause that
                else paused = true;
            }
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
            } else if (e.type == SDL_KEYDOWN && title) {
                switch (e.key.keysym.scancode) {
                    case SDL_SCANCODE_ESCAPE: if (!e.key.repeat) toggleMenu(); break;
                    case SDL_SCANCODE_LEFT: case SDL_SCANCODE_A: titleSelect(set.track - 1); break;
                    case SDL_SCANCODE_RIGHT: case SDL_SCANCODE_D: titleSelect(set.track + 1); break;
                    case SDL_SCANCODE_RETURN: case SDL_SCANCODE_SPACE: if (!e.key.repeat) startDriving(); break;
                    case SDL_SCANCODE_F11: set.fullscreen = !set.fullscreen; changed(); break;
                    default: break;
                }
            } else if (e.type == SDL_KEYDOWN && replaying) {
                switch (e.key.keysym.scancode) {
                    case SDL_SCANCODE_ESCAPE: case SDL_SCANCODE_R: case SDL_SCANCODE_RETURN: case SDL_SCANCODE_V:
                        if (!e.key.repeat) stopReplay();
                        break;
                    case SDL_SCANCODE_SPACE: case SDL_SCANCODE_P: if (!e.key.repeat) replayPaused = !replayPaused; break;
                    case SDL_SCANCODE_LEFT: case SDL_SCANCODE_A: seekReplay(-3); break;
                    case SDL_SCANCODE_RIGHT: case SDL_SCANCODE_D: seekReplay(3); break;
                    case SDL_SCANCODE_C: if (!e.key.repeat) cycleCamera(); break;
                    case SDL_SCANCODE_L: if (!e.key.repeat) { set.racingLine = !set.racingLine; changed(); } break;
                    default: break;
                }
            } else if (e.type == SDL_KEYDOWN && !e.key.repeat) {
                switch (e.key.keysym.scancode) {
                    case SDL_SCANCODE_ESCAPE: toggleMenu(); break;
                    case SDL_SCANCODE_V: startReplay(); break;
                    case SDL_SCANCODE_R:
                    case SDL_SCANCODE_RETURN: restartLap(); break;
                    case SDL_SCANCODE_C: cycleCamera(); break;
                    case SDL_SCANCODE_G:
                        set.ghost = (set.ghost + 1) % 4;
                        changed();
                        game.flash(std::string("GHOST: ") + GHOST_NAMES[set.ghost], WHITE, 1.5f);
                        break;
                    case SDL_SCANCODE_L: set.racingLine = !set.racingLine; changed(); break;
                    case SDL_SCANCODE_P: paused = !paused; break;
                    case SDL_SCANCODE_F11: set.fullscreen = !set.fullscreen; changed(); break;
                    default: break;
                }
            }
            if (title && !menu.open && e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                int i = cardAt(float(e.button.x), float(e.button.y));
                if (i == set.track) startDriving();
                else if (i >= 0) titleSelect(i);
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
                    // Arrows step either way; elsewhere on the row, choices cycle forward
                    // and toggles flip, but percentages need an arrow.
                    if (int dir = menuArrowAt(MENU[i], set, menu, mx)) menuAdjust(dir);
                    else if (MENU[i].kind != Kind::Percent) menuActivate();
                }
            }
            if (menu.open && e.type == SDL_MOUSEWHEEL && hasArrows(MENU[menu.sel])) {
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
                if (b == SDL_CONTROLLER_BUTTON_BACK && !replaying) toggleMenu();
                else if (title && !menu.open) {
                    if (b == SDL_CONTROLLER_BUTTON_DPAD_LEFT) titleSelect(set.track - 1);
                    if (b == SDL_CONTROLLER_BUTTON_DPAD_RIGHT) titleSelect(set.track + 1);
                    if (b == SDL_CONTROLLER_BUTTON_A || b == SDL_CONTROLLER_BUTTON_START) startDriving();
                } else if (replaying) {
                    if (b == SDL_CONTROLLER_BUTTON_A) replayPaused = !replayPaused;
                    if (b == SDL_CONTROLLER_BUTTON_DPAD_LEFT) seekReplay(-3);
                    if (b == SDL_CONTROLLER_BUTTON_DPAD_RIGHT) seekReplay(3);
                    if (b == SDL_CONTROLLER_BUTTON_B || b == SDL_CONTROLLER_BUTTON_START || b == SDL_CONTROLLER_BUTTON_BACK)
                        stopReplay();
                } else if (menu.open) {
                    if (b == SDL_CONTROLLER_BUTTON_DPAD_UP) menuMove(-1);
                    if (b == SDL_CONTROLLER_BUTTON_DPAD_DOWN) menuMove(1);
                    if (b == SDL_CONTROLLER_BUTTON_DPAD_LEFT) menuAdjust(-1);
                    if (b == SDL_CONTROLLER_BUTTON_DPAD_RIGHT) menuAdjust(1);
                    if (b == SDL_CONTROLLER_BUTTON_A) menuActivate();
                    if (b == SDL_CONTROLLER_BUTTON_B) toggleMenu();
                } else {
                    if (b == SDL_CONTROLLER_BUTTON_START || b == SDL_CONTROLLER_BUTTON_B) restartLap();
                    if (b == SDL_CONTROLLER_BUTTON_Y) cycleCamera();
                }
            }
        }

        Uint64 now = SDL_GetPerformanceCounter();
        float frameDt = float(std::min(0.25, double(now - prev) / SDL_GetPerformanceFrequency()));
        acc += frameDt;
        prev = now;
        if (paused || menu.open || replaying || title) acc = 0;
        Input in = readInput(pad, set);
        while (acc >= DT) {
            game.step(in);
            acc -= DT;
        }
        // The replay loops at the end of the lap.
        if (replaying && !replayPaused && !paused && !menu.open) replayAcc += frameDt;
        for (; replayAcc >= DT; replayAcc -= DT) {
            replayStep = (replayStep + 1) % int(replay.bestGhost.size());
            replay.puppetTo(replay.bestGhost, replayStep);
        }
        if (replaying) {
            float t = replay.lapSteps * DT;
            replay.splitsDone = int(t >= replay.splits[0]) + int(t >= replay.splits[1]) + int(t >= replay.splits[2]);
        }
        if (title && !menu.open) demoAcc += frameDt;
        for (; demoAcc >= DT; demoAcc -= DT) demo.step(autopilot(demo));
        const Game& shownGame = replaying ? replay : title ? demo : game;
        menu.bestTime = game.bestTime;

        updateCamera(view, shownGame, set, frameDt, snapCamera);
        snapCamera = false;

        if (audio) {
            // Fade out while paused or in the menu.
            bool quiet = paused || menu.open || (replaying && replayPaused);
            SoundParams sp = soundParams(shownGame, quiet ? 0.0f : set.volume / 100.0f * (title ? 0.5f : 1.0f));
            if (title) sp.chimes = game.chimes;  // the demo car laps silently
            SDL_LockAudioDevice(audio);
            synth.p = sp;
            SDL_UnlockAudioDevice(audio);
        }

        fx.update(shownGame, paused || menu.open || (replaying && replayPaused) ? 0.0f : frameDt);
        View shown = view;
        if (fx.shake > 0.3f)
            shown.cam = view.cam + Vec2{fx.rng.range(-1, 1), fx.rng.range(-1, 1)} * (fx.shake / view.zoom);

        rumble.update(pad, shownGame, fx, set.vibration && !paused && !menu.open && !replaying && !title);
        // The mouse is only for the menu and title screen.
        bool wantCursor = menu.open || title;
        if (bool(SDL_ShowCursor(SDL_QUERY)) != wantCursor) SDL_ShowCursor(wantCursor ? SDL_ENABLE : SDL_DISABLE);

        if (title) {
            cards[set.track].best = game.bestTime;  // in case it was reset from the menu
            drawWorld(r, demo, shown, quads, set, 0, fx, verts);
            drawTitle(r, cards, set.track, demo.stepCount);
            if (menu.open) drawMenu(r, menu, set);
        } else {
            render(r, shownGame, shown, quads, mini, set, menu, paused, replaying, replayPaused, fx, verts);
        }
        SDL_RenderPresent(r);
    }

    if (audio) SDL_CloseAudioDevice(audio);
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
