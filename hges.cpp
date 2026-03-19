/**
 * hges.cpp — Hybrid Green Edge Server Scheduling
 * ================================================
 * Implements the three-algorithm framework from the CLOUD short paper:
 *   Algorithm 1: Inter-Window Planner  (offline, per billing window)
 *   Algorithm 2: Intra-Window Planner  (offline, per energy slot within window)
 *   Algorithm 3: Online Adaptation     (runtime, O(1) per slot)
 *
 * Key difference from exp3.cpp:
 *   - Window-level aggregate tiered pricing (Eq.9 in paper):
 *     C_grid(m) = c_k * G(m)  where G(m) = sum of grid power in window m
 *     The ENTIRE window's consumption is priced at one tier, not marginal per-slot.
 *   - Battery leakage factor h_l applied every energy slot.
 *   - Three heterogeneous time scales: task slot (15ms), energy slot (1 min),
 *     billing window (1 hr). In this simulation, each "slot" in the CSV
 *     represents one energy slot (~2 min), and a billing window = 30 slots = 1 hr.
 *
 * Baselines implemented for comparison:
 *   GG      : Grid-Greedy           (solar offline → immediate grid for rest)
 *   SDC     : Online Window Shaving  (grid-only, look-behind leveling)
 *   SP+SDC  : Solar-Power + SDC      (solar offline → SDC for rest)
 *   H-GES   : Proposed               (Algo 1 + 2 + 3)
 *
 * Build:  g++ -O2 -std=c++17 -o hges hges.cpp
 * Run:    ./hges
 */

#include <bits/stdc++.h>
#include <chrono>
using namespace std;

/* ═══════════════════════════════════════════════════════════════════════
 *  CONSTANTS & PARAMETERS  (aligned with CLOUD paper Section V)
 * ═══════════════════════════════════════════════════════════════════════ */

// --- Simulation grid ---
constexpr int    NUM_SLOTS           = 1440;       // energy slots over 24 h (1 per minute)
constexpr double SLOT_DURATION_HR    = 24.0 / NUM_SLOTS;  // hours per slot (1/60)

// --- Billing window (runtime, set via --billing) ---
static int    BILLING_WINDOW_SIZE = 60;          // slots per billing window (default: 60 = 1 hr)
static int    NUM_WINDOWS         = NUM_SLOTS / BILLING_WINDOW_SIZE;  // recomputed at startup

// --- Server power model (Eq 2-3 in paper) ---
constexpr double P_S    = 100.0;    // static/idle power (W)
constexpr double P_MAX  = 400.0;    // peak power at full utilization (W)
constexpr double U_MAX  = 20.0;     // max utilization units per slot
constexpr double EPS    = 1e-6;

// --- Tiered grid pricing per billing window (Eq 9) ---
// Thresholds in W-hr per hour of billing window; scale with window size.
static double TIER1_BASE_WHR = 120.0;   // W-hr/hr — T1 ceiling (set via --tier_th)
static double TIER2_BASE_WHR = 240.0;   // W-hr/hr — T2 ceiling (set via --tier_th)
static double TIER1_THRESHOLD = TIER1_BASE_WHR * BILLING_WINDOW_SIZE;
static double TIER2_THRESHOLD = TIER2_BASE_WHR * BILLING_WINDOW_SIZE;
static double TIER1_RATE      = 5.0;    // $/W-hr  (cheapest, set via --tier_price)
static double TIER2_RATE      = 7.0;    // $/W-hr
static double TIER3_RATE      = 10.0;   // $/W-hr  (most expensive)

// --- Battery defaults ---
static double BATTERY_CAPACITY = 10000.0;  // W·slot (β^max)
static double BATTERY_LEAKAGE  = 0.99;     // h_l: fraction retained per slot

// --- SDC baseline ---
constexpr int SDC_WINDOW = 4;  // look-behind window for SDC shaving

/* ═══════════════════════════════════════════════════════════════════════
 *  DATA STRUCTURES
 * ═══════════════════════════════════════════════════════════════════════ */

struct Task {
    int    id;
    int    arrival;     // earliest eligible slot
    int    deadline;    // latest eligible slot
    double util;        // utilization demand
};

struct WindowPlan {
    enum Mode { SOLAR_FIRST, GRID_FIRST };
    Mode   mode;
    double gridBudget;       // G^budget_m in W·slot
    double batteryTarget;    // b^target_m to carry to next window
};

struct SimResult {
    int    tasksOnSolar;
    int    tasksOnGrid;
    int    tasksDropped;
    double totalGridCost;    // total $ paid over all windows
    double totalGridEnergy;  // total W·slot drawn from grid
    vector<double> windowGridUsage;      // G(m) per window
    vector<double> windowGridCost;       // C(m) per window
    vector<double> slotGridPower;        // per-slot grid power
    vector<double> slotBatteryLevel;     // battery level per slot
};

/* ═══════════════════════════════════════════════════════════════════════
 *  UTILITY FUNCTIONS
 * ═══════════════════════════════════════════════════════════════════════ */

// Cubic power model: P = P_s + (P_max - P_s) * (U / U_max)^3
inline double util_to_power(double u) {
    return P_S + (P_MAX - P_S) * pow(max(0.0, u) / U_MAX, 3.0);
}

// Inverse: given power, find utilization
inline double power_to_util(double p) {
    if (p <= P_S + EPS) return 0.0;
    double frac = (p - P_S) / (P_MAX - P_S);
    if (frac <= 0.0) return 0.0;
    return U_MAX * cbrt(frac);
}

/**
 * Window-level tiered cost (Eq 9 in paper).
 * Given total grid usage G(m) for a billing window, returns cost.
 * Key: the ENTIRE consumption is billed at one tier rate.
 */
double window_tiered_cost(double Gm) {
    if (Gm <= EPS) return 0.0;
    if (Gm <= TIER1_THRESHOLD) {
        return TIER1_RATE * Gm * SLOT_DURATION_HR;  // convert W·slot to W-hr
    } else if (Gm <= TIER2_THRESHOLD) {
        return TIER2_RATE * Gm * SLOT_DURATION_HR;
    } else {
        return TIER3_RATE * Gm * SLOT_DURATION_HR;
    }
}

/**
 * What tier rate applies for a given G(m)?
 */
double tier_rate_for(double Gm) {
    if (Gm <= TIER1_THRESHOLD) return TIER1_RATE;
    if (Gm <= TIER2_THRESHOLD) return TIER2_RATE;
    return TIER3_RATE;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  FILE I/O + DATASET GENERATION
 *
 *  Directory layout:
 *    Dataset/Real-life/power_reallife.csv     (base solar)
 *    Dataset/Real-life/task_reallife.csv       (base tasks)
 *    Dataset/Deviation/solar_dev_5pct.csv      (generated)
 *    Dataset/Deviation/solar_dev_10pct.csv
 *    Dataset/Deviation/task_dev_5pct.csv
 *    Dataset/Deviation/task_dev_10pct.csv
 *    ...
 *
 *  If a deviation file does not exist, it is generated automatically
 *  from the base file using the requested deviation percentage.
 * ═══════════════════════════════════════════════════════════════════════ */

// --- Directory constants ---
const string DIR_REALLIFE  = "Dataset/Real-life/";
const string DIR_DEVIATION = "Dataset/Deviation/";

const string BASE_SOLAR = DIR_REALLIFE + "power_reallife.csv";
const string BASE_TASK  = DIR_REALLIFE + "task_reallife.csv";

// --- Helpers ---
bool fileExists(const string& path) {
    ifstream f(path);
    return f.good();
}

void ensureDir(const string& dir) {
    // Create directory tree (works on Linux)
    string cmd = "mkdir -p " + dir;
    system(cmd.c_str());
}

// --- Loaders ---

vector<double> loadSolarPower(const string& filename) {
    vector<double> solar(NUM_SLOTS, 0.0);
    ifstream f(filename);
    if (!f.is_open()) { cerr << "[ERR] Cannot open " << filename << "\n"; return solar; }
    string line;
    getline(f, line); // skip header
    int t = 0;
    while (getline(f, line) && t < NUM_SLOTS) {
        stringstream ss(line);
        string seg;
        getline(ss, seg, ','); // skip timestamp
        getline(ss, seg, ',');
        try { solar[t] = stod(seg); } catch (...) {}
        t++;
    }
    cout << "[IO] Loaded " << t << " solar slots from " << filename << "\n";
    return solar;
}

// Raw CSV loader for solar (returns int values + timestamps)
struct SolarRow { int timestamp; int power; };

vector<SolarRow> loadSolarRaw(const string& filename) {
    vector<SolarRow> rows;
    ifstream f(filename);
    if (!f.is_open()) { cerr << "[ERR] Cannot open " << filename << "\n"; return rows; }
    string line;
    getline(f, line); // skip header
    while (getline(f, line)) {
        stringstream ss(line);
        string s1, s2;
        getline(ss, s1, ',');
        getline(ss, s2, ',');
        try { rows.push_back({stoi(s1), stoi(s2)}); } catch (...) {}
    }
    return rows;
}

// Raw CSV loader for tasks (preserves original float fields)
struct TaskRow { double id, arrival, deadline, util; };

vector<TaskRow> loadTasksRaw(const string& filename) {
    vector<TaskRow> rows;
    ifstream f(filename);
    if (!f.is_open()) { cerr << "[ERR] Cannot open " << filename << "\n"; return rows; }
    string line;
    getline(f, line); // skip header
    while (getline(f, line)) {
        stringstream ss(line);
        string seg;
        vector<string> parts;
        while (getline(ss, seg, ',')) parts.push_back(seg);
        if (parts.size() < 4) continue;
        try {
            rows.push_back({stod(parts[0]), stod(parts[1]), stod(parts[2]),
                            stod(parts[3])});
            // Profit column (parts[4]) ignored if present
        } catch (...) {}
    }
    return rows;
}

vector<Task> loadTasks(const string& filename) {
    vector<Task> tasks;
    ifstream f(filename);
    if (!f.is_open()) { cerr << "[ERR] Cannot open " << filename << "\n"; return tasks; }
    string line;
    getline(f, line); // skip header
    int id = 0;
    while (getline(f, line)) {
        stringstream ss(line);
        string seg;
        vector<string> parts;
        while (getline(ss, seg, ',')) parts.push_back(seg);
        if (parts.size() < 4) continue;
        try {
            Task t;
            t.id       = id++;
            t.arrival  = max(0, (int)stod(parts[1]) - 1);
            t.deadline = min(NUM_SLOTS - 1, (int)stod(parts[2]) - 1);
            t.util     = stod(parts[3]);
            if (t.arrival <= t.deadline && t.deadline < NUM_SLOTS) {
                tasks.push_back(t);
            }
        } catch (...) {}
    }
    cout << "[IO] Loaded " << tasks.size() << " tasks from " << filename << "\n";
    return tasks;
}

/* ─────────────────────────────────────────────────────────────────────
 *  Solar Deviation Generator
 *  Port of the Python fast-up-slow-down variation:
 *    - Non-zero solar values get multiplicative noise in [1-dev%, 1+dev%]
 *    - Rises apply immediately, falls decay slowly (alpha_down = 0.08)
 *    - Zero values remain exactly zero
 *    - Output clamped to [0, 400], rounded to int
 * ───────────────────────────────────────────────────────────────────── */

void generateSolarDeviation(const string& baseFile, const string& outFile,
                             int deviationPct, unsigned seed = 42) {
    vector<SolarRow> rows = loadSolarRaw(baseFile);
    if (rows.empty()) { cerr << "[ERR] Empty base solar file\n"; return; }

    mt19937 rng(seed);
    double eps = deviationPct / 100.0;
    double alpha_down = 0.08;

    // Step 1: Generate target multipliers for non-zero entries
    int n = (int)rows.size();
    vector<double> targetMult(n, 1.0);
    uniform_real_distribution<double> dist(1.0 - eps, 1.0 + eps);
    for (int i = 0; i < n; i++) {
        if (rows[i].power > 0) targetMult[i] = dist(rng);
    }

    // Step 2: Apply fast-up slow-down shaping
    vector<double> shapedMult = targetMult;
    for (int t = 1; t < n; t++) {
        if (rows[t].power == 0) { shapedMult[t] = 1.0; continue; }
        double prev = shapedMult[t - 1];
        double curr = shapedMult[t];
        if (curr >= prev) {
            shapedMult[t] = curr;  // fast up
        } else {
            shapedMult[t] = prev * (1.0 - alpha_down) + curr * alpha_down;  // slow down
        }
    }

    // Step 3: Apply multiplier, clamp, round
    ofstream out(outFile);
    out << "timestamp,Solar Power\n";
    for (int i = 0; i < n; i++) {
        double val = rows[i].power;
        if (val > 0) val *= shapedMult[i];
        val = max(0.0, min(400.0, val));
        int ival = (int)round(val);
        out << rows[i].timestamp << "," << ival << "\n";
    }
    out.close();
    cout << "[GEN] Solar deviation " << deviationPct << "% → " << outFile << "\n";
}

/* ─────────────────────────────────────────────────────────────────────
 *  Task Deviation Generator
 *  Port of the Python robustness task-spike logic:
 *    1. Pick deviationPct% of time slots as "shock" slots
 *    2. At each shock slot, count existing task arrivals
 *    3. Add spikePct ± 5% extra tasks at those shock slots
 *    4. New tasks: arrival=t, deadline=min(maxSlot, t+10), util=1
 *    5. No removal — existing tasks are preserved intact
 *  This models unexpected AV demand surges at random times.
 * ───────────────────────────────────────────────────────────────────── */

// Global spike percentage (set from CLI, default 25 → range [20%,30%])
static int SPIKE_PCT = 25;

void generateTaskDeviation(const string& baseFile, const string& outFile,
                            int deviationPct, unsigned seed = 42) {
    vector<TaskRow> rows = loadTasksRaw(baseFile);
    if (rows.empty()) { cerr << "[ERR] Empty base task file\n"; return; }

    mt19937 rng(seed);

    // Spike range: SPIKE_PCT ± 5%
    double spikeLo = max(0.01, (SPIKE_PCT - 5) / 100.0);
    double spikeHi = (SPIKE_PCT + 5) / 100.0;
    constexpr int    D_I       = 10;     // deadline window for new tasks
    constexpr double UTIL_VAL  = 1.0;

    // Find time horizon from data (max arrival slot)
    double maxSlot = 0;
    for (const auto& r : rows) maxSlot = max(maxSlot, r.arrival);
    int T = (int)maxSlot;

    // Step 1: Pick deviationPct% of time slots as shock slots
    int k = max(1, (int)round(T * deviationPct / 100.0));
    k = min(k, T);

    vector<int> allSlots(T);
    iota(allSlots.begin(), allSlots.end(), 1);
    shuffle(allSlots.begin(), allSlots.end(), rng);
    set<int> shockSlots(allSlots.begin(), allSlots.begin() + k);

    // Step 2: Count existing arrivals per slot
    map<int, int> arrivalCounts;
    for (const auto& r : rows) {
        arrivalCounts[(int)r.arrival]++;
    }

    // Step 3: Generate spike tasks at shock slots
    int nextId = (int)rows.size();
    vector<TaskRow> newTasks;
    uniform_real_distribution<double> spikeDist(spikeLo, spikeHi);

    for (int t : shockSlots) {
        int base = arrivalCounts.count(t) ? arrivalCounts[t] : 0;
        if (base <= 0) continue;

        double spikeRatio = spikeDist(rng);
        int addN = (int)ceil(base * spikeRatio);
        if (addN <= 0) continue;

        for (int j = 0; j < addN; j++) {
            TaskRow nt;
            nt.id       = nextId++;
            nt.arrival  = t;
            nt.deadline = min((double)(T + D_I), (double)(t + D_I));
            nt.util     = UTIL_VAL;
            newTasks.push_back(nt);
        }
    }

    // Step 4: Merge, sort, write (no Profit column)
    vector<TaskRow> result = rows;
    result.insert(result.end(), newTasks.begin(), newTasks.end());

    sort(result.begin(), result.end(), [](const TaskRow& a, const TaskRow& b) {
        if (a.arrival != b.arrival) return a.arrival < b.arrival;
        return a.id < b.id;
    });

    ofstream out(outFile);
    out << fixed << setprecision(1);
    out << "id,arrival,deadline,Utilization\n";
    for (int i = 0; i < (int)result.size(); i++) {
        out << (double)i << ","
            << result[i].arrival << ","
            << result[i].deadline << ","
            << result[i].util << "\n";
    }
    out.close();
    cout << "[GEN] Task deviation " << deviationPct << "% (spike "
         << SPIKE_PCT << "%) → " << outFile
         << " (" << result.size() << " tasks, +" << newTasks.size()
         << " spikes at " << shockSlots.size() << " shock slots)\n";
}

/* ─────────────────────────────────────────────────────────────────────
 *  Path resolution + auto-generation
 * ───────────────────────────────────────────────────────────────────── */

string solarFilePath(int dev) {
    if (dev == 0) return BASE_SOLAR;
    return DIR_DEVIATION + "solar_dev_" + to_string(dev) + "pct.csv";
}

string taskFilePath(int dev) {
    if (dev == 0) return BASE_TASK;
    return DIR_DEVIATION + "task_dev_" + to_string(dev) + "pct_spike_"
           + to_string(SPIKE_PCT) + "pct.csv";
}

/**
 * Ensure base files exist in Dataset/Real-life/.
 * Copies from working directory if Dataset/ layout not yet created.
 */
void setupBaseFiles() {
    ensureDir(DIR_REALLIFE);
    ensureDir(DIR_DEVIATION);

    // Copy base files if not already in Dataset/Real-life/
    if (!fileExists(BASE_SOLAR)) {
        // Try common locations
        for (const string& src : {"power_reallife.csv",
                                   "../power_reallife.csv"}) {
            if (fileExists(src)) {
                string cmd = "cp " + src + " " + BASE_SOLAR;
                system(cmd.c_str());
                cout << "[SETUP] Copied " << src << " → " << BASE_SOLAR << "\n";
                break;
            }
        }
    }
    if (!fileExists(BASE_TASK)) {
        for (const string& src : {"task_reallife.csv",
                                   "../task_reallife.csv"}) {
            if (fileExists(src)) {
                string cmd = "cp " + src + " " + BASE_TASK;
                system(cmd.c_str());
                cout << "[SETUP] Copied " << src << " → " << BASE_TASK << "\n";
                break;
            }
        }
    }

    if (!fileExists(BASE_SOLAR))
        cerr << "[WARN] Base solar file not found: " << BASE_SOLAR << "\n";
    if (!fileExists(BASE_TASK))
        cerr << "[WARN] Base task file not found: " << BASE_TASK << "\n";
}

/**
 * For a given deviation %, ensure the deviated file exists.
 * If not, generate it from the base file.
 */
void ensureSolarDeviation(int dev) {
    if (dev == 0) return;  // base file, no generation needed
    string path = solarFilePath(dev);
    if (fileExists(path)) {
        cout << "[IO] Found existing: " << path << "\n";
        return;
    }
    generateSolarDeviation(BASE_SOLAR, path, dev, 42 + dev);
}

void ensureTaskDeviation(int dev) {
    if (dev == 0) return;
    string path = taskFilePath(dev);
    if (fileExists(path)) {
        cout << "[IO] Found existing: " << path << "\n";
        return;
    }
    generateTaskDeviation(BASE_TASK, path, dev, 42 + dev);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  COMMON: OFFLINE SOLAR SCHEDULING  (shared by GG, SP+SDC, H-GES)
 *
 *  Greedy solar pass: for each slot (latest first), schedule as many
 *  eligible tasks as solar power allows, using Earliest-Deadline-First.
 * ═══════════════════════════════════════════════════════════════════════ */

struct OfflineSolarPlan {
    // offlineSolar[t] = set of task IDs scheduled on solar at slot t
    vector<set<int>>  solarSchedule;
    vector<double>    solarUsedUtil;      // utilization used by solar tasks at each slot
    set<int>          scheduledTaskIDs;
};

OfflineSolarPlan offlineSolarSchedule(const vector<Task>& tasks,
                                       const vector<double>& predictedSolar) {
    OfflineSolarPlan plan;
    plan.solarSchedule.resize(NUM_SLOTS);
    plan.solarUsedUtil.assign(NUM_SLOTS, 0.0);

    // Build per-slot eligible task lists, sorted by earliest deadline first (EDF)
    for (int t = NUM_SLOTS - 1; t >= 0; t--) {
        double solarUtilLimit = power_to_util(predictedSolar[t]);

        // Collect eligible tasks for this slot, sorted by deadline (EDF)
        vector<int> eligible;
        for (const auto& task : tasks) {
            if (plan.scheduledTaskIDs.count(task.id)) continue;
            if (task.arrival <= t && task.deadline >= t) {
                eligible.push_back(task.id);
            }
        }
        sort(eligible.begin(), eligible.end(), [&](int a, int b) {
            return tasks[a].deadline < tasks[b].deadline;  // EDF
        });

        for (int tid : eligible) {
            if (plan.solarUsedUtil[t] + tasks[tid].util <= solarUtilLimit + EPS) {
                plan.solarSchedule[t].insert(tid);
                plan.solarUsedUtil[t] += tasks[tid].util;
                plan.scheduledTaskIDs.insert(tid);
            }
        }
    }
    return plan;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  OFFLINE SOLAR + BATTERY LEVELING  (Algorithm 2 from ICFEC paper)
 *
 *  Smooths solar utilization across billing windows by shifting workload
 *  from high-load windows to low-load windows via the battery.
 * ═══════════════════════════════════════════════════════════════════════ */

void solarBatteryLeveling(OfflineSolarPlan& plan,
                           const vector<Task>& tasks,
                           const vector<double>& predictedSolar,
                           double batteryCap) {
    if (batteryCap <= EPS) return;

    vector<double> battLevel(NUM_SLOTS + 1, 0.0);
    const int MAX_ITER = 2000;

    for (int iter = 0; iter < MAX_ITER; iter++) {
        // Compute average power across slots
        double totalPow = 0.0;
        for (int t = 0; t < NUM_SLOTS; t++) {
            totalPow += util_to_power(plan.solarUsedUtil[t]);
        }
        double avgPow = totalPow / NUM_SLOTS;

        // Find slot with max power consumption
        int tMax = -1;
        double maxPow = -1.0;
        for (int t = 0; t < NUM_SLOTS; t++) {
            double p = util_to_power(plan.solarUsedUtil[t]);
            if (p > maxPow) { maxPow = p; tMax = t; }
        }

        // Find nearest subsequent slot below average
        int tMin = -1;
        for (int t = tMax + 1; t < NUM_SLOTS; t++) {
            if (util_to_power(plan.solarUsedUtil[t]) < avgPow - EPS) {
                tMin = t;
                break;
            }
        }
        if (tMax == -1 || tMin == -1) break;

        // Compute how much battery capacity is available between tMax and tMin
        double minBattSpace = 1e18;
        for (int j = tMax + 1; j <= tMin; j++) {
            minBattSpace = min(minBattSpace, batteryCap - battLevel[j]);
        }
        if (minBattSpace <= EPS) break;

        double limitHigh = max(0.0, maxPow - avgPow);
        double limitLow  = max(0.0, avgPow - util_to_power(plan.solarUsedUtil[tMin]));
        double transferable = min({minBattSpace, limitHigh, limitLow});
        if (transferable <= EPS) break;

        // Remove tasks from tMax (lowest-util first)
        double savedPow = 0.0;
        vector<int> removedTasks;
        {
            // Sort tasks at tMax by util ascending
            vector<int> tasksAtMax(plan.solarSchedule[tMax].begin(),
                                    plan.solarSchedule[tMax].end());
            sort(tasksAtMax.begin(), tasksAtMax.end(), [&](int a, int b) {
                return tasks[a].util < tasks[b].util;  // remove lowest-util first
            });
            for (int tid : tasksAtMax) {
                if (savedPow >= transferable) break;
                double prevPow = util_to_power(plan.solarUsedUtil[tMax]);
                plan.solarUsedUtil[tMax] -= tasks[tid].util;
                plan.solarSchedule[tMax].erase(tid);
                plan.scheduledTaskIDs.erase(tid);
                double newPow = util_to_power(plan.solarUsedUtil[tMax]);
                savedPow += prevPow - newPow;
                removedTasks.push_back(tid);
            }
        }
        if (savedPow <= EPS) break;

        double powToMove = min(savedPow, transferable);

        // Update battery levels between tMax and tMin
        for (int j = tMax + 1; j <= tMin; j++) {
            battLevel[j] = min(batteryCap, battLevel[j] + powToMove);
        }

        // Try to place removed tasks at tMin
        double solarLimitAtMin = power_to_util(predictedSolar[tMin] + powToMove);
        for (int tid : removedTasks) {
            if (tasks[tid].arrival <= tMin && tasks[tid].deadline >= tMin &&
                plan.solarUsedUtil[tMin] + tasks[tid].util <= solarLimitAtMin + EPS) {
                plan.solarSchedule[tMin].insert(tid);
                plan.solarUsedUtil[tMin] += tasks[tid].util;
                plan.scheduledTaskIDs.insert(tid);
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  BASELINE 1: GRID-GREEDY (GG)
 *  Solar offline → remaining tasks executed immediately on grid at
 *  their arrival slot. No deferral, no smoothing.
 * ═══════════════════════════════════════════════════════════════════════ */

SimResult runGridGreedy(const vector<Task>& tasks,
                         const vector<double>& predictedSolar,
                         const vector<double>& actualSolar,
                         double batteryCap,
                         double leakage) {
    // Offline solar
    OfflineSolarPlan solar = offlineSolarSchedule(tasks, predictedSolar);
    solarBatteryLeveling(solar, tasks, predictedSolar, batteryCap);

    // Online: execute solar plan + greedy grid
    SimResult res;
    res.tasksOnSolar = 0; res.tasksOnGrid = 0; res.tasksDropped = 0;
    res.totalGridCost = 0.0; res.totalGridEnergy = 0.0;
    res.windowGridUsage.assign(NUM_WINDOWS, 0.0);
    res.windowGridCost.assign(NUM_WINDOWS, 0.0);
    res.slotGridPower.assign(NUM_SLOTS, 0.0);
    res.slotBatteryLevel.assign(NUM_SLOTS, 0.0);

    double battery = 0.0;
    vector<bool> executed(tasks.size(), false);

    // Build arrival index
    vector<vector<int>> arrivalsAt(NUM_SLOTS);
    for (const auto& t : tasks) {
        if (t.arrival >= 0 && t.arrival < NUM_SLOTS) arrivalsAt[t.arrival].push_back(t.id);
    }

    // Pending grid tasks (EDF ordering)
    auto cmpDeadline = [&](int a, int b) { return tasks[a].deadline > tasks[b].deadline; };
    priority_queue<int, vector<int>, decltype(cmpDeadline)> gridPending(cmpDeadline);

    for (int t = 0; t < NUM_SLOTS; t++) {
        int window = t / BILLING_WINDOW_SIZE;

        // Apply leakage
        battery *= leakage;

        // Execute solar-planned tasks
        double solarAvail = actualSolar[t] + battery;
        double solarUsedUtil = 0.0;
        for (int tid : solar.solarSchedule[t]) {
            double pReq = util_to_power(solarUsedUtil + tasks[tid].util) -
                          util_to_power(solarUsedUtil);
            if (pReq <= solarAvail + EPS) {
                solarUsedUtil += tasks[tid].util;
                solarAvail -= pReq;
                executed[tid] = true;
                res.tasksOnSolar++;
            } else {
                gridPending.push(tid);
            }
        }

        // Add newly arrived unscheduled tasks to grid queue
        for (int tid : arrivalsAt[t]) {
            if (!executed[tid] && !solar.scheduledTaskIDs.count(tid)) {
                gridPending.push(tid);
            }
        }

        // Scavenge leftover solar for grid candidates
        {
            vector<int> deferred;
            while (!gridPending.empty()) {
                int tid = gridPending.top(); gridPending.pop();
                if (executed[tid] || t > tasks[tid].deadline) continue;
                double pReq = util_to_power(solarUsedUtil + tasks[tid].util) -
                              util_to_power(solarUsedUtil);
                if (pReq <= solarAvail + EPS &&
                    solarUsedUtil + tasks[tid].util <= U_MAX + EPS) {
                    solarUsedUtil += tasks[tid].util;
                    solarAvail -= pReq;
                    executed[tid] = true;
                    res.tasksOnSolar++;
                } else {
                    deferred.push_back(tid);
                }
            }
            for (int tid : deferred) gridPending.push(tid);
        }

        // Greedy grid: execute all remaining eligible tasks NOW
        // Paper constraint: ALL tasks must complete by deadline
        double gridUsedUtil = 0.0;
        {
            vector<int> deferred;
            while (!gridPending.empty()) {
                int tid = gridPending.top(); gridPending.pop();
                if (executed[tid]) continue;
                if (t > tasks[tid].deadline) continue;
                // Force execute if at deadline, otherwise try to fit
                bool atDeadline = (tasks[tid].deadline == t);
                if (atDeadline || gridUsedUtil + tasks[tid].util <= U_MAX + EPS) {
                    gridUsedUtil += tasks[tid].util;
                    executed[tid] = true;
                    res.tasksOnGrid++;
                } else {
                    deferred.push_back(tid);
                }
            }
            for (int tid : deferred) gridPending.push(tid);
        }

        // Record grid power
        double gridPower = (gridUsedUtil > EPS) ?
                           util_to_power(gridUsedUtil) - P_S : 0.0;
        res.slotGridPower[t] = gridPower;
        res.windowGridUsage[window] += gridPower;

        // Battery update: store surplus solar
        double solarConsumed = util_to_power(solarUsedUtil) - P_S;
        double solarSurplus = max(0.0, actualSolar[t] - solarConsumed);
        battery = min(batteryCap, battery + solarSurplus);
        // Drain for solar deficit
        double solarDeficit = max(0.0, solarConsumed - actualSolar[t]);
        battery = max(0.0, battery - solarDeficit);
        res.slotBatteryLevel[t] = battery;
    }

    // Compute window costs
    for (int m = 0; m < NUM_WINDOWS; m++) {
        res.windowGridCost[m] = window_tiered_cost(res.windowGridUsage[m]);
        res.totalGridCost += res.windowGridCost[m];
        res.totalGridEnergy += res.windowGridUsage[m];
    }

    // Count dropped
    for (size_t i = 0; i < tasks.size(); i++) {
        if (!executed[i]) res.tasksDropped++;
    }

    return res;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  BASELINE 2: SDC  (Online Window Shaving, grid-only)
 *  Pure online: look-behind leveling with battery buffering.
 *  No solar used at all.
 * ═══════════════════════════════════════════════════════════════════════ */

SimResult runSDC(const vector<Task>& tasks,
                  const vector<double>& actualSolar,
                  double batteryCap,
                  double leakage) {
    SimResult res;
    res.tasksOnSolar = 0; res.tasksOnGrid = 0; res.tasksDropped = 0;
    res.totalGridCost = 0.0; res.totalGridEnergy = 0.0;
    res.windowGridUsage.assign(NUM_WINDOWS, 0.0);
    res.windowGridCost.assign(NUM_WINDOWS, 0.0);
    res.slotGridPower.assign(NUM_SLOTS, 0.0);
    res.slotBatteryLevel.assign(NUM_SLOTS, 0.0);

    double gridBattery = 0.0;
    vector<double> powerHistory(NUM_SLOTS, 0.0);
    vector<bool> executed(tasks.size(), false);

    auto cmpDeadline = [&](int a, int b) { return tasks[a].deadline > tasks[b].deadline; };
    priority_queue<int, vector<int>, decltype(cmpDeadline)> pending(cmpDeadline);

    vector<vector<int>> arrivalsAt(NUM_SLOTS);
    for (const auto& t : tasks) {
        if (t.arrival >= 0 && t.arrival < NUM_SLOTS)
            arrivalsAt[t.arrival].push_back(t.id);
    }

    for (int t = 0; t < NUM_SLOTS; t++) {
        int window = t / BILLING_WINDOW_SIZE;
        gridBattery *= leakage;

        // Ingest arrivals
        for (int tid : arrivalsAt[t]) {
            if (!executed[tid]) pending.push(tid);
        }

        // Compute look-behind target
        double histSum = 0; int cnt = 0;
        for (int i = max(0, t - SDC_WINDOW); i < t; i++) {
            histSum += powerHistory[i];
            cnt++;
        }
        double pTarget = (cnt > 0) ? histSum / cnt : TIER1_THRESHOLD / BILLING_WINDOW_SIZE;

        // Process tasks with SDC logic
        double gridUtil = 0.0;
        double battDischarge = 0.0;
        vector<int> deferred;

        while (!pending.empty()) {
            int tid = pending.top(); pending.pop();
            if (executed[tid]) continue;
            if (t > tasks[tid].deadline) continue;

            double pPrev = util_to_power(gridUtil);
            double pNext = util_to_power(gridUtil + tasks[tid].util);
            double powerReq = pNext - pPrev;

            // Battery support: level above pTarget
            double battSupport = 0.0;
            if (pNext > pTarget && gridBattery > EPS) {
                double needed = pNext - pTarget;
                battSupport = min({needed, gridBattery, powerReq});
            }

            bool isCritical = (tasks[tid].deadline <= t);
            // Force execute at deadline; otherwise check capacity
            if (isCritical || gridUtil + tasks[tid].util <= U_MAX + EPS) {
                gridUtil += tasks[tid].util;
                gridBattery -= battSupport;
                battDischarge += battSupport;
                executed[tid] = true;
                res.tasksOnGrid++;
            } else {
                deferred.push_back(tid);
            }
        }
        for (int tid : deferred) pending.push(tid);

        // Actual grid draw (minus battery support)
        double actualGridDraw = max(0.0,
            (gridUtil > EPS ? util_to_power(gridUtil) - P_S : 0.0) - battDischarge);

        // Valley filling: charge battery when below target
        if (actualGridDraw < pTarget) {
            double headroom = pTarget - actualGridDraw;
            double space = batteryCap - gridBattery;
            double charge = min(headroom, space);
            if (charge > EPS) {
                actualGridDraw += charge;
                gridBattery += charge * leakage;  // imperfect charging
            }
        }

        powerHistory[t] = actualGridDraw;
        res.slotGridPower[t] = actualGridDraw;
        res.windowGridUsage[window] += actualGridDraw;
        res.slotBatteryLevel[t] = gridBattery;
    }

    for (int m = 0; m < NUM_WINDOWS; m++) {
        res.windowGridCost[m] = window_tiered_cost(res.windowGridUsage[m]);
        res.totalGridCost += res.windowGridCost[m];
        res.totalGridEnergy += res.windowGridUsage[m];
    }
    for (size_t i = 0; i < tasks.size(); i++) {
        if (!executed[i]) res.tasksDropped++;
    }
    return res;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  BASELINE 3: SP+SDC  (Solar-Power + Online Window Shaving)
 *  Solar offline scheduling → remaining tasks handled with SDC online.
 * ═══════════════════════════════════════════════════════════════════════ */

SimResult runSPplusSDC(const vector<Task>& tasks,
                        const vector<double>& predictedSolar,
                        const vector<double>& actualSolar,
                        double batteryCap,
                        double leakage) {
    // Offline solar
    OfflineSolarPlan solar = offlineSolarSchedule(tasks, predictedSolar);
    solarBatteryLeveling(solar, tasks, predictedSolar, batteryCap * 0.5);

    SimResult res;
    res.tasksOnSolar = 0; res.tasksOnGrid = 0; res.tasksDropped = 0;
    res.totalGridCost = 0.0; res.totalGridEnergy = 0.0;
    res.windowGridUsage.assign(NUM_WINDOWS, 0.0);
    res.windowGridCost.assign(NUM_WINDOWS, 0.0);
    res.slotGridPower.assign(NUM_SLOTS, 0.0);
    res.slotBatteryLevel.assign(NUM_SLOTS, 0.0);

    double solarBattery = 0.0;
    double gridBattery  = 0.0;
    double solarBattCap = batteryCap * 0.5;
    double gridBattCap  = batteryCap * 0.5;
    vector<double> powerHistory(NUM_SLOTS, 0.0);
    vector<bool> executed(tasks.size(), false);

    auto cmpDeadline = [&](int a, int b) { return tasks[a].deadline > tasks[b].deadline; };
    priority_queue<int, vector<int>, decltype(cmpDeadline)> gridPending(cmpDeadline);

    vector<vector<int>> arrivalsAt(NUM_SLOTS);
    for (const auto& t : tasks) {
        if (t.arrival >= 0 && t.arrival < NUM_SLOTS)
            arrivalsAt[t.arrival].push_back(t.id);
    }

    for (int t = 0; t < NUM_SLOTS; t++) {
        int window = t / BILLING_WINDOW_SIZE;
        solarBattery *= leakage;
        gridBattery  *= leakage;

        // Phase A: Solar execution
        double solarAvail = actualSolar[t] + solarBattery;
        double solarUsedUtil = 0.0;
        for (int tid : solar.solarSchedule[t]) {
            double pReq = util_to_power(solarUsedUtil + tasks[tid].util) -
                          util_to_power(solarUsedUtil);
            if (pReq <= solarAvail + EPS) {
                solarUsedUtil += tasks[tid].util;
                solarAvail -= pReq;
                executed[tid] = true;
                res.tasksOnSolar++;
            } else {
                gridPending.push(tid);
            }
        }

        // New arrivals not in solar plan → grid
        for (int tid : arrivalsAt[t]) {
            if (!executed[tid] && !solar.scheduledTaskIDs.count(tid))
                gridPending.push(tid);
        }

        // Scavenge leftover solar
        {
            vector<int> deferred;
            while (!gridPending.empty()) {
                int tid = gridPending.top(); gridPending.pop();
                if (executed[tid] || t > tasks[tid].deadline) continue;
                double pReq = util_to_power(solarUsedUtil + tasks[tid].util) -
                              util_to_power(solarUsedUtil);
                if (pReq <= solarAvail + EPS &&
                    solarUsedUtil + tasks[tid].util <= U_MAX + EPS) {
                    solarUsedUtil += tasks[tid].util;
                    solarAvail -= pReq;
                    executed[tid] = true;
                    res.tasksOnSolar++;
                } else {
                    deferred.push_back(tid);
                }
            }
            for (int tid : deferred) gridPending.push(tid);
        }

        // Phase B: SDC grid execution
        double histSum = 0; int cnt = 0;
        for (int i = max(0, t - SDC_WINDOW); i < t; i++) {
            histSum += powerHistory[i]; cnt++;
        }
        double pTarget = (cnt > 0) ? histSum / cnt : TIER1_THRESHOLD / BILLING_WINDOW_SIZE;

        double gridUtil = 0.0;
        double battDischarge = 0.0;
        {
            vector<int> deferred;
            while (!gridPending.empty()) {
                int tid = gridPending.top(); gridPending.pop();
                if (executed[tid]) continue;
                if (t > tasks[tid].deadline) continue;

                double pPrev = util_to_power(gridUtil);
                double pNext = util_to_power(gridUtil + tasks[tid].util);
                double powerReq = pNext - pPrev;

                double battSupport = 0.0;
                if (pNext > pTarget && gridBattery > EPS) {
                    battSupport = min({pNext - pTarget, gridBattery, powerReq});
                }

                if (gridUtil + tasks[tid].util <= U_MAX + EPS ||
                    tasks[tid].deadline <= t) {  // force at deadline
                    gridUtil += tasks[tid].util;
                    gridBattery -= battSupport;
                    battDischarge += battSupport;
                    executed[tid] = true;
                    res.tasksOnGrid++;
                } else {
                    deferred.push_back(tid);
                }
            }
            for (int tid : deferred) gridPending.push(tid);
        }

        double actualGridDraw = max(0.0,
            (gridUtil > EPS ? util_to_power(gridUtil) - P_S : 0.0) - battDischarge);

        // Valley filling for grid battery
        if (actualGridDraw < pTarget) {
            double charge = min(pTarget - actualGridDraw, gridBattCap - gridBattery);
            if (charge > EPS) {
                actualGridDraw += charge;
                gridBattery += charge * leakage;
            }
        }

        powerHistory[t] = actualGridDraw;
        res.slotGridPower[t] = actualGridDraw;
        res.windowGridUsage[window] += actualGridDraw;

        // Solar battery update
        double solarConsumed = (solarUsedUtil > EPS) ?
                               util_to_power(solarUsedUtil) - P_S : 0.0;
        double surplus = max(0.0, actualSolar[t] - solarConsumed);
        solarBattery = min(solarBattCap, solarBattery + surplus);
        double deficit = max(0.0, solarConsumed - actualSolar[t]);
        solarBattery = max(0.0, solarBattery - deficit);
        res.slotBatteryLevel[t] = solarBattery + gridBattery;
    }

    for (int m = 0; m < NUM_WINDOWS; m++) {
        res.windowGridCost[m] = window_tiered_cost(res.windowGridUsage[m]);
        res.totalGridCost += res.windowGridCost[m];
        res.totalGridEnergy += res.windowGridUsage[m];
    }
    for (size_t i = 0; i < tasks.size(); i++) {
        if (!executed[i]) res.tasksDropped++;
    }
    return res;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  PROPOSED: H-GES  (Algorithms 1, 2, 3 from CLOUD paper)
 *
 *  Key insight: pre-charging battery from cheap-tier grid can displace
 *  expensive peak-hour grid usage indirectly by enabling more tasks to
 *  be served from solar+battery.  However, battery leakage can erode
 *  the benefit.  The planner evaluates multiple pre-charging intensities
 *  using predicted traces and selects the cost-minimizing strategy.
 *
 *  Algorithm 1 (Inter-Window Planner):
 *    Evaluates candidate pre-charging intensities (0%..100%) using
 *    predicted solar/task data and selects the one that minimizes
 *    total grid cost.
 *
 *  Algorithm 2 (Intra-Window Planner):
 *    Executes slot-by-slot within each window using the selected
 *    pre-charge budget, with solar-first → battery → grid ordering
 *    and EDF scheduling.
 *
 *  Algorithm 3 (Online Adaptation):
 *    At each 15 ms slot, corrects for deviations between predicted
 *    and actual solar generation in O(1).
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * Core simulation engine: runs a full 24-hour simulation with a given
 * pre-charging intensity.  Used both for planning (with predicted solar)
 * and execution (with actual solar).
 *
 * @param preChargeFrac  0.0 = no pre-charging (SolarFirst),
 *                       1.0 = max pre-charging up to θ_Th (GridFirst),
 *                       values in between = partial pre-charging
 */
SimResult hgesSimulate(
    const vector<Task>& tasks,
    const vector<double>& solar,         // predicted or actual solar
    double batteryCap,
    double leakage,
    const OfflineSolarPlan& solarPlan,
    double preChargeFrac,                // 0.0 to 1.0
    double thetaTh)
{
    SimResult res;
    res.tasksOnSolar = 0; res.tasksOnGrid = 0; res.tasksDropped = 0;
    res.totalGridCost = 0.0; res.totalGridEnergy = 0.0;
    res.windowGridUsage.assign(NUM_WINDOWS, 0.0);
    res.windowGridCost.assign(NUM_WINDOWS, 0.0);
    res.slotGridPower.assign(NUM_SLOTS, 0.0);
    res.slotBatteryLevel.assign(NUM_SLOTS, 0.0);

    double battery = 0.0;
    vector<bool> executed(tasks.size(), false);
    vector<double> windowGridAccum(NUM_WINDOWS, 0.0);

    vector<vector<int>> arrivalsAt(NUM_SLOTS);
    for (const auto& t : tasks) {
        if (t.arrival >= 0 && t.arrival < NUM_SLOTS)
            arrivalsAt[t.arrival].push_back(t.id);
    }

    auto cmpDeadline = [&](int a, int b) { return tasks[a].deadline > tasks[b].deadline; };
    priority_queue<int, vector<int>, decltype(cmpDeadline)> gridBacklog(cmpDeadline);

    for (int t = 0; t < NUM_SLOTS; t++) {
        int window = t / BILLING_WINDOW_SIZE;
        battery *= leakage;

        // ── Phase A: Solar + battery execution ──
        double solarAvail = solar[t];
        double solarUsedUtil = 0.0;

        for (int tid : solarPlan.solarSchedule[t]) {
            double pReq = util_to_power(solarUsedUtil + tasks[tid].util) -
                          util_to_power(solarUsedUtil);
            double fromSolar = min(pReq, solarAvail);
            double fromBat   = min(pReq - fromSolar, battery);
            if (fromSolar + fromBat >= pReq - EPS &&
                solarUsedUtil + tasks[tid].util <= U_MAX + EPS) {
                solarUsedUtil += tasks[tid].util;
                solarAvail -= fromSolar;
                battery -= fromBat;
                executed[tid] = true;
                res.tasksOnSolar++;
            } else {
                gridBacklog.push(tid);
            }
        }

        // New arrivals not in solar plan → grid queue
        for (int tid : arrivalsAt[t]) {
            if (!executed[tid] && !solarPlan.scheduledTaskIDs.count(tid))
                gridBacklog.push(tid);
        }

        // Scavenge remaining solar+battery for backlog tasks
        {
            vector<int> deferred;
            while (!gridBacklog.empty()) {
                int tid = gridBacklog.top(); gridBacklog.pop();
                if (executed[tid] || t > tasks[tid].deadline) continue;
                double pReq = util_to_power(solarUsedUtil + tasks[tid].util) -
                              util_to_power(solarUsedUtil);
                double fromSolar = min(pReq, solarAvail);
                double fromBat   = min(pReq - fromSolar, battery);
                if (fromSolar + fromBat >= pReq - EPS &&
                    solarUsedUtil + tasks[tid].util <= U_MAX + EPS) {
                    solarUsedUtil += tasks[tid].util;
                    solarAvail -= fromSolar;
                    battery -= fromBat;
                    executed[tid] = true;
                    res.tasksOnSolar++;
                } else {
                    deferred.push_back(tid);
                }
            }
            for (int tid : deferred) gridBacklog.push(tid);
        }

        // ── Phase B: Grid execution (battery-first, then grid) ──
        double gridPowerThisSlot = 0.0;
        double gridUtil = 0.0;
        {
            vector<int> deferred;
            while (!gridBacklog.empty()) {
                int tid = gridBacklog.top(); gridBacklog.pop();
                if (executed[tid]) continue;
                if (t > tasks[tid].deadline) continue;

                double pPrev = (gridUtil > EPS) ? util_to_power(gridUtil) - P_S : 0.0;
                double pNext = util_to_power(gridUtil + tasks[tid].util) - P_S;
                double marginalPow = pNext - pPrev;

                double battSupply = min(battery, marginalPow);
                double gridNeeded = marginalPow - battSupply;

                bool isUrgent = (tasks[tid].deadline <= t);
                if (isUrgent || gridUtil + tasks[tid].util <= U_MAX + EPS) {
                    gridUtil += tasks[tid].util;
                    battery -= battSupply;
                    gridPowerThisSlot += gridNeeded;
                    executed[tid] = true;
                    res.tasksOnGrid++;
                } else {
                    deferred.push_back(tid);
                }
            }
            for (int tid : deferred) gridBacklog.push(tid);
        }

        // ── Phase C: Pre-charging (controlled by preChargeFrac) ──
        // Pre-charge battery from cheap-tier grid up to a fraction of θ_Th.
        // This fills the battery so future slots can serve more tasks on
        // solar+battery instead of expensive grid.
        if (preChargeFrac > EPS) {
            double cheapTierRemaining = max(0.0, thetaTh - windowGridAccum[window]
                                            - gridPowerThisSlot);
            double maxCharge = cheapTierRemaining * preChargeFrac;
            double batRoom = batteryCap - battery;
            double charge = min(maxCharge, batRoom);
            if (charge > EPS) {
                gridPowerThisSlot += charge;
                battery += charge * leakage;
            }
        }

        // ── Record ──
        res.slotGridPower[t] = gridPowerThisSlot;
        windowGridAccum[window] += gridPowerThisSlot;
        res.windowGridUsage[window] = windowGridAccum[window];

        // ── Phase D: Battery update — store solar surplus ──
        double solarConsumed = (solarUsedUtil > EPS) ?
                               util_to_power(solarUsedUtil) - P_S : 0.0;
        double solarSurplus = max(0.0, solar[t] - solarConsumed);
        battery = min(batteryCap, battery + solarSurplus);
        res.slotBatteryLevel[t] = battery;
    }

    // Compute window costs
    for (int m = 0; m < NUM_WINDOWS; m++) {
        res.windowGridCost[m] = window_tiered_cost(res.windowGridUsage[m]);
        res.totalGridCost += res.windowGridCost[m];
        res.totalGridEnergy += res.windowGridUsage[m];
    }
    for (size_t i = 0; i < tasks.size(); i++) {
        if (!executed[i]) res.tasksDropped++;
    }
    return res;
}

/**
 * H-GES entry point: Algorithm 1 selects the optimal pre-charge intensity,
 * then Algorithms 2+3 execute the plan with online adaptation.
 */
SimResult runHGES(const vector<Task>& tasks,
                   const vector<double>& predictedSolar,
                   const vector<double>& actualSolar,
                   double batteryCap,
                   double leakage) {
    // ── Step 1: Offline Solar Scheduling (shared across all strategies) ──
    OfflineSolarPlan solarPlan = offlineSolarSchedule(tasks, predictedSolar);
    solarBatteryLeveling(solarPlan, tasks, predictedSolar, batteryCap);

    double thetaTh = TIER1_THRESHOLD;

    // ── Step 2: Algorithm 1 — Evaluate pre-charging intensity ──
    // Simulate candidate strategies using PREDICTED traces to find the
    // optimal pre-charge fraction.  This captures the indirect benefit
    // of pre-charging (fuller battery → more solar task coverage →
    // fewer tasks on expensive grid) and the cost of leakage.
    double bestFrac = 0.0;
    double bestCost = 1e18;

    // Coarse sweep: 0% to 100% in 5% steps (21 candidates)
    for (int pct = 0; pct <= 100; pct += 5) {
        double frac = pct / 100.0;
        SimResult candidate = hgesSimulate(
            tasks, predictedSolar, batteryCap, leakage,
            solarPlan, frac, thetaTh);
        if (candidate.totalGridCost < bestCost - EPS) {
            bestCost = candidate.totalGridCost;
            bestFrac = frac;
        }
    }

    // Fine-tune around the winner: ±5% in 1% steps
    double lo = max(0.0, bestFrac - 0.05);
    double hi = min(1.0, bestFrac + 0.05);
    for (double frac = lo; frac <= hi + EPS; frac += 0.01) {
        SimResult candidate = hgesSimulate(
            tasks, predictedSolar, batteryCap, leakage,
            solarPlan, frac, thetaTh);
        if (candidate.totalGridCost < bestCost - EPS) {
            bestCost = candidate.totalGridCost;
            bestFrac = frac;
        }
    }

    // ── Step 3: Algorithms 2+3 — Execute with ACTUAL solar ──
    // The selected fraction is used as the pre-charging policy;
    // Algorithm 3's O(1) online adaptation handles solar deviations.
    return hgesSimulate(
        tasks, actualSolar, batteryCap, leakage,
        solarPlan, bestFrac, thetaTh);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  ABLATION: SolarFirst-Only
 *  Force all windows to SolarFirst. No inter-window pre-charging.
 *  Grid is drawn only when solar+battery cannot meet demand at a slot.
 * ═══════════════════════════════════════════════════════════════════════ */

SimResult runHGES_SolarOnly(const vector<Task>& tasks,
                             const vector<double>& predictedSolar,
                             const vector<double>& actualSolar,
                             double batteryCap,
                             double leakage) {
    OfflineSolarPlan solarPlan = offlineSolarSchedule(tasks, predictedSolar);
    solarBatteryLeveling(solarPlan, tasks, predictedSolar, batteryCap);

    SimResult res;
    res.tasksOnSolar = 0; res.tasksOnGrid = 0; res.tasksDropped = 0;
    res.totalGridCost = 0.0; res.totalGridEnergy = 0.0;
    res.windowGridUsage.assign(NUM_WINDOWS, 0.0);
    res.windowGridCost.assign(NUM_WINDOWS, 0.0);
    res.slotGridPower.assign(NUM_SLOTS, 0.0);
    res.slotBatteryLevel.assign(NUM_SLOTS, 0.0);

    double battery = 0.0;
    vector<bool> executed(tasks.size(), false);

    vector<vector<int>> arrivalsAt(NUM_SLOTS);
    for (const auto& t : tasks) {
        if (t.arrival >= 0 && t.arrival < NUM_SLOTS)
            arrivalsAt[t.arrival].push_back(t.id);
    }

    auto cmpDeadline = [&](int a, int b) { return tasks[a].deadline > tasks[b].deadline; };
    priority_queue<int, vector<int>, decltype(cmpDeadline)> gridBacklog(cmpDeadline);

    for (int t = 0; t < NUM_SLOTS; t++) {
        int window = t / BILLING_WINDOW_SIZE;
        battery *= leakage;

        // Phase A: Solar + battery execution
        double solarAvail = actualSolar[t];
        double solarUsedUtil = 0.0;

        for (int tid : solarPlan.solarSchedule[t]) {
            double pReq = util_to_power(solarUsedUtil + tasks[tid].util) -
                          util_to_power(solarUsedUtil);
            double fromSolar = min(pReq, solarAvail);
            double fromBat   = min(pReq - fromSolar, battery);
            if (fromSolar + fromBat >= pReq - EPS &&
                solarUsedUtil + tasks[tid].util <= U_MAX + EPS) {
                solarUsedUtil += tasks[tid].util;
                solarAvail -= fromSolar;
                battery -= fromBat;
                executed[tid] = true;
                res.tasksOnSolar++;
            } else {
                gridBacklog.push(tid);
            }
        }

        for (int tid : arrivalsAt[t]) {
            if (!executed[tid] && !solarPlan.scheduledTaskIDs.count(tid))
                gridBacklog.push(tid);
        }

        // Scavenge solar+battery for backlog
        {
            vector<int> deferred;
            while (!gridBacklog.empty()) {
                int tid = gridBacklog.top(); gridBacklog.pop();
                if (executed[tid] || t > tasks[tid].deadline) continue;
                double pReq = util_to_power(solarUsedUtil + tasks[tid].util) -
                              util_to_power(solarUsedUtil);
                double fromSolar = min(pReq, solarAvail);
                double fromBat   = min(pReq - fromSolar, battery);
                if (fromSolar + fromBat >= pReq - EPS &&
                    solarUsedUtil + tasks[tid].util <= U_MAX + EPS) {
                    solarUsedUtil += tasks[tid].util;
                    solarAvail -= fromSolar;
                    battery -= fromBat;
                    executed[tid] = true;
                    res.tasksOnSolar++;
                } else {
                    deferred.push_back(tid);
                }
            }
            for (int tid : deferred) gridBacklog.push(tid);
        }

        // Phase B: Grid ONLY for tasks that solar+battery cannot serve
        // NO pre-charging, NO proactive grid allocation
        double gridPowerThisSlot = 0.0;
        double gridUtil = 0.0;
        {
            vector<int> deferred;
            while (!gridBacklog.empty()) {
                int tid = gridBacklog.top(); gridBacklog.pop();
                if (executed[tid]) continue;
                if (t > tasks[tid].deadline) continue;

                double pPrev = (gridUtil > EPS) ? util_to_power(gridUtil) - P_S : 0.0;
                double pNext = util_to_power(gridUtil + tasks[tid].util) - P_S;
                double marginalPow = pNext - pPrev;

                // Battery first
                double battSupply = min(battery, marginalPow);
                double gridNeeded = marginalPow - battSupply;

                bool isUrgent = (tasks[tid].deadline <= t);
                if (isUrgent || gridUtil + tasks[tid].util <= U_MAX + EPS) {
                    gridUtil += tasks[tid].util;
                    battery -= battSupply;
                    gridPowerThisSlot += gridNeeded;
                    executed[tid] = true;
                    res.tasksOnGrid++;
                } else {
                    deferred.push_back(tid);
                }
            }
            for (int tid : deferred) gridBacklog.push(tid);
        }

        // NO Phase C (no pre-charging at all)

        res.slotGridPower[t] = gridPowerThisSlot;
        res.windowGridUsage[window] += gridPowerThisSlot;

        // Battery update
        double solarConsumed = (solarUsedUtil > EPS) ?
                               util_to_power(solarUsedUtil) - P_S : 0.0;
        double solarSurplus = max(0.0, actualSolar[t] - solarConsumed);
        battery = min(batteryCap, battery + solarSurplus);
        res.slotBatteryLevel[t] = battery;
    }

    for (int m = 0; m < NUM_WINDOWS; m++) {
        res.windowGridCost[m] = window_tiered_cost(res.windowGridUsage[m]);
        res.totalGridCost += res.windowGridCost[m];
        res.totalGridEnergy += res.windowGridUsage[m];
    }
    for (size_t i = 0; i < tasks.size(); i++) {
        if (!executed[i]) res.tasksDropped++;
    }
    return res;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  ABLATION: GridFirst-Only
 *  Force all windows to GridFirst mode. Every billing window gets the
 *  full cheap-tier budget θ_Th allocated. Pre-charges battery from grid
 *  in every window that has remaining cheap-tier headroom.
 * ═══════════════════════════════════════════════════════════════════════ */

SimResult runHGES_GridOnly(const vector<Task>& tasks,
                            const vector<double>& predictedSolar,
                            const vector<double>& actualSolar,
                            double batteryCap,
                            double leakage) {
    OfflineSolarPlan solarPlan = offlineSolarSchedule(tasks, predictedSolar);
    solarBatteryLeveling(solarPlan, tasks, predictedSolar, batteryCap);

    double thetaTh = TIER1_THRESHOLD;

    SimResult res;
    res.tasksOnSolar = 0; res.tasksOnGrid = 0; res.tasksDropped = 0;
    res.totalGridCost = 0.0; res.totalGridEnergy = 0.0;
    res.windowGridUsage.assign(NUM_WINDOWS, 0.0);
    res.windowGridCost.assign(NUM_WINDOWS, 0.0);
    res.slotGridPower.assign(NUM_SLOTS, 0.0);
    res.slotBatteryLevel.assign(NUM_SLOTS, 0.0);

    double battery = 0.0;
    vector<bool> executed(tasks.size(), false);
    vector<double> windowGridAccum(NUM_WINDOWS, 0.0);

    vector<vector<int>> arrivalsAt(NUM_SLOTS);
    for (const auto& t : tasks) {
        if (t.arrival >= 0 && t.arrival < NUM_SLOTS)
            arrivalsAt[t.arrival].push_back(t.id);
    }

    auto cmpDeadline = [&](int a, int b) { return tasks[a].deadline > tasks[b].deadline; };
    priority_queue<int, vector<int>, decltype(cmpDeadline)> gridBacklog(cmpDeadline);

    for (int t = 0; t < NUM_SLOTS; t++) {
        int window = t / BILLING_WINDOW_SIZE;
        battery *= leakage;

        // Phase A: Solar execution (same as H-GES)
        double solarAvail = actualSolar[t];
        double solarUsedUtil = 0.0;

        for (int tid : solarPlan.solarSchedule[t]) {
            double pReq = util_to_power(solarUsedUtil + tasks[tid].util) -
                          util_to_power(solarUsedUtil);
            double fromSolar = min(pReq, solarAvail);
            double fromBat   = min(pReq - fromSolar, battery);
            if (fromSolar + fromBat >= pReq - EPS &&
                solarUsedUtil + tasks[tid].util <= U_MAX + EPS) {
                solarUsedUtil += tasks[tid].util;
                solarAvail -= fromSolar;
                battery -= fromBat;
                executed[tid] = true;
                res.tasksOnSolar++;
            } else {
                gridBacklog.push(tid);
            }
        }

        for (int tid : arrivalsAt[t]) {
            if (!executed[tid] && !solarPlan.scheduledTaskIDs.count(tid))
                gridBacklog.push(tid);
        }

        // Scavenge
        {
            vector<int> deferred;
            while (!gridBacklog.empty()) {
                int tid = gridBacklog.top(); gridBacklog.pop();
                if (executed[tid] || t > tasks[tid].deadline) continue;
                double pReq = util_to_power(solarUsedUtil + tasks[tid].util) -
                              util_to_power(solarUsedUtil);
                double fromSolar = min(pReq, solarAvail);
                double fromBat   = min(pReq - fromSolar, battery);
                if (fromSolar + fromBat >= pReq - EPS &&
                    solarUsedUtil + tasks[tid].util <= U_MAX + EPS) {
                    solarUsedUtil += tasks[tid].util;
                    solarAvail -= fromSolar;
                    battery -= fromBat;
                    executed[tid] = true;
                    res.tasksOnSolar++;
                } else {
                    deferred.push_back(tid);
                }
            }
            for (int tid : deferred) gridBacklog.push(tid);
        }

        // Phase B: Grid execution with battery support
        double gridPowerThisSlot = 0.0;
        double gridUtil = 0.0;
        {
            vector<int> deferred;
            while (!gridBacklog.empty()) {
                int tid = gridBacklog.top(); gridBacklog.pop();
                if (executed[tid]) continue;
                if (t > tasks[tid].deadline) continue;

                double pPrev = (gridUtil > EPS) ? util_to_power(gridUtil) - P_S : 0.0;
                double pNext = util_to_power(gridUtil + tasks[tid].util) - P_S;
                double marginalPow = pNext - pPrev;

                double battSupply = min(battery, marginalPow);
                double gridNeeded = marginalPow - battSupply;

                bool isUrgent = (tasks[tid].deadline <= t);
                if (isUrgent || gridUtil + tasks[tid].util <= U_MAX + EPS) {
                    gridUtil += tasks[tid].util;
                    battery -= battSupply;
                    gridPowerThisSlot += gridNeeded;
                    executed[tid] = true;
                    res.tasksOnGrid++;
                } else {
                    deferred.push_back(tid);
                }
            }
            for (int tid : deferred) gridBacklog.push(tid);
        }

        // Phase C: AGGRESSIVE pre-charging — always fill cheap tier headroom
        double cheapTierRemaining = max(0.0, thetaTh - windowGridAccum[window]
                                        - gridPowerThisSlot);
        if (cheapTierRemaining > EPS) {
            double batRoom = batteryCap - battery;
            double charge = min(cheapTierRemaining, batRoom);
            if (charge > EPS) {
                gridPowerThisSlot += charge;
                battery += charge * leakage;
            }
        }

        res.slotGridPower[t] = gridPowerThisSlot;
        windowGridAccum[window] += gridPowerThisSlot;
        res.windowGridUsage[window] = windowGridAccum[window];

        // Battery update
        double solarConsumed = (solarUsedUtil > EPS) ?
                               util_to_power(solarUsedUtil) - P_S : 0.0;
        double solarSurplus = max(0.0, actualSolar[t] - solarConsumed);
        battery = min(batteryCap, battery + solarSurplus);
        res.slotBatteryLevel[t] = battery;
    }

    for (int m = 0; m < NUM_WINDOWS; m++) {
        res.windowGridCost[m] = window_tiered_cost(res.windowGridUsage[m]);
        res.totalGridCost += res.windowGridCost[m];
        res.totalGridEnergy += res.windowGridUsage[m];
    }
    for (size_t i = 0; i < tasks.size(); i++) {
        if (!executed[i]) res.tasksDropped++;
    }
    return res;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  BASELINE: Naive Grid
 *  All tasks executed on grid at arrival. No solar, no battery.
 *  This is the worst-case reference: total grid cost without renewables.
 * ═══════════════════════════════════════════════════════════════════════ */

SimResult runNaiveGrid(const vector<Task>& tasks,
                        const vector<double>& /* solar unused */,
                        double /* batteryCap unused */,
                        double /* leakage unused */) {
    SimResult res;
    res.tasksOnSolar = 0; res.tasksOnGrid = 0; res.tasksDropped = 0;
    res.totalGridCost = 0.0; res.totalGridEnergy = 0.0;
    res.windowGridUsage.assign(NUM_WINDOWS, 0.0);
    res.windowGridCost.assign(NUM_WINDOWS, 0.0);
    res.slotGridPower.assign(NUM_SLOTS, 0.0);
    res.slotBatteryLevel.assign(NUM_SLOTS, 0.0);

    vector<bool> executed(tasks.size(), false);

    // Build arrival index
    vector<vector<int>> arrivalsAt(NUM_SLOTS);
    for (const auto& t : tasks) {
        if (t.arrival >= 0 && t.arrival < NUM_SLOTS)
            arrivalsAt[t.arrival].push_back(t.id);
    }

    auto cmpDeadline = [&](int a, int b) { return tasks[a].deadline > tasks[b].deadline; };
    priority_queue<int, vector<int>, decltype(cmpDeadline)> pending(cmpDeadline);

    for (int t = 0; t < NUM_SLOTS; t++) {
        int window = t / BILLING_WINDOW_SIZE;

        for (int tid : arrivalsAt[t]) {
            if (!executed[tid]) pending.push(tid);
        }

        double gridUtil = 0.0;
        vector<int> deferred;
        while (!pending.empty()) {
            int tid = pending.top(); pending.pop();
            if (executed[tid]) continue;
            if (t > tasks[tid].deadline) continue;
            bool isUrgent = (tasks[tid].deadline <= t);
            if (isUrgent || gridUtil + tasks[tid].util <= U_MAX + EPS) {
                gridUtil += tasks[tid].util;
                executed[tid] = true;
                res.tasksOnGrid++;
            } else {
                deferred.push_back(tid);
            }
        }
        for (int tid : deferred) pending.push(tid);

        double gridPower = (gridUtil > EPS) ? util_to_power(gridUtil) - P_S : 0.0;
        res.slotGridPower[t] = gridPower;
        res.windowGridUsage[window] += gridPower;
    }

    for (int m = 0; m < NUM_WINDOWS; m++) {
        res.windowGridCost[m] = window_tiered_cost(res.windowGridUsage[m]);
        res.totalGridCost += res.windowGridCost[m];
        res.totalGridEnergy += res.windowGridUsage[m];
    }
    for (size_t i = 0; i < tasks.size(); i++) {
        if (!executed[i]) res.tasksDropped++;
    }
    return res;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  BASELINE: Reactive Grid  (Grid-Only-When-Needed)
 *  Solar + battery first.  Grid drawn ONLY when solar and battery are
 *  both insufficient.  No pre-charging, no planning ahead.
 *  This is the simplest hybrid strategy.
 * ═══════════════════════════════════════════════════════════════════════ */

// NOTE: This is identical to SF-Only (runHGES_SolarOnly).
// We alias it for paper clarity.
SimResult runReactiveGrid(const vector<Task>& tasks,
                           const vector<double>& predictedSolar,
                           const vector<double>& actualSolar,
                           double batteryCap,
                           double leakage) {
    return runHGES_SolarOnly(tasks, predictedSolar, actualSolar, batteryCap, leakage);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  BASELINE: Lyapunov-Based Online Control
 *  Adapted from Guo et al. (IEEE IoT-J 2022) and Dabbagh et al.
 *  (IEEE TCC 2019) limited-horizon control.
 *
 *  Core idea: at each slot t, minimize a drift-plus-penalty function:
 *    V * gridCost(t) + Q(t) * (energyOut - energyIn)
 *
 *  where Q(t) = battery - θ is a virtual queue.
 *  - When Q(t) < 0 (battery low): penalty encourages charging.
 *  - When Q(t) > 0 (battery high): penalty encourages discharging.
 *  - V controls cost-vs-stability tradeoff.
 *
 *  No offline planning.  Purely online O(1) per slot.
 *  Solar-first execution, then Lyapunov-guided grid/battery decisions.
 * ═══════════════════════════════════════════════════════════════════════ */

SimResult runLyapunov(const vector<Task>& tasks,
                       const vector<double>& predictedSolar,
                       const vector<double>& actualSolar,
                       double batteryCap,
                       double leakage) {
    // Still use offline solar scheduling (same as all solar-aware methods)
    OfflineSolarPlan solarPlan = offlineSolarSchedule(tasks, predictedSolar);
    solarBatteryLeveling(solarPlan, tasks, predictedSolar, batteryCap);

    double thetaTh = TIER1_THRESHOLD;
    // Lyapunov parameters
    double theta = batteryCap * 0.5;   // perturbation: target battery level
    double V = thetaTh * SLOT_DURATION_HR * TIER2_RATE;  // cost-stability weight

    SimResult res;
    res.tasksOnSolar = 0; res.tasksOnGrid = 0; res.tasksDropped = 0;
    res.totalGridCost = 0.0; res.totalGridEnergy = 0.0;
    res.windowGridUsage.assign(NUM_WINDOWS, 0.0);
    res.windowGridCost.assign(NUM_WINDOWS, 0.0);
    res.slotGridPower.assign(NUM_SLOTS, 0.0);
    res.slotBatteryLevel.assign(NUM_SLOTS, 0.0);

    double battery = 0.0;
    vector<bool> executed(tasks.size(), false);
    vector<double> windowGridAccum(NUM_WINDOWS, 0.0);

    vector<vector<int>> arrivalsAt(NUM_SLOTS);
    for (const auto& t : tasks) {
        if (t.arrival >= 0 && t.arrival < NUM_SLOTS)
            arrivalsAt[t.arrival].push_back(t.id);
    }

    auto cmpDeadline = [&](int a, int b) { return tasks[a].deadline > tasks[b].deadline; };
    priority_queue<int, vector<int>, decltype(cmpDeadline)> gridBacklog(cmpDeadline);

    for (int t = 0; t < NUM_SLOTS; t++) {
        int window = t / BILLING_WINDOW_SIZE;
        battery *= leakage;

        // Virtual queue: Q(t) = battery - theta
        double Q = battery - theta;

        // Phase A: Solar + battery execution (same as other methods)
        double solarAvail = actualSolar[t];
        double solarUsedUtil = 0.0;

        for (int tid : solarPlan.solarSchedule[t]) {
            double pReq = util_to_power(solarUsedUtil + tasks[tid].util) -
                          util_to_power(solarUsedUtil);
            double fromSolar = min(pReq, solarAvail);
            double fromBat   = min(pReq - fromSolar, battery);
            if (fromSolar + fromBat >= pReq - EPS &&
                solarUsedUtil + tasks[tid].util <= U_MAX + EPS) {
                solarUsedUtil += tasks[tid].util;
                solarAvail -= fromSolar;
                battery -= fromBat;
                executed[tid] = true;
                res.tasksOnSolar++;
            } else {
                gridBacklog.push(tid);
            }
        }

        for (int tid : arrivalsAt[t]) {
            if (!executed[tid] && !solarPlan.scheduledTaskIDs.count(tid))
                gridBacklog.push(tid);
        }

        // Scavenge solar+battery
        {
            vector<int> deferred;
            while (!gridBacklog.empty()) {
                int tid = gridBacklog.top(); gridBacklog.pop();
                if (executed[tid] || t > tasks[tid].deadline) continue;
                double pReq = util_to_power(solarUsedUtil + tasks[tid].util) -
                              util_to_power(solarUsedUtil);
                double fromSolar = min(pReq, solarAvail);
                double fromBat   = min(pReq - fromSolar, battery);
                if (fromSolar + fromBat >= pReq - EPS &&
                    solarUsedUtil + tasks[tid].util <= U_MAX + EPS) {
                    solarUsedUtil += tasks[tid].util;
                    solarAvail -= fromSolar;
                    battery -= fromBat;
                    executed[tid] = true;
                    res.tasksOnSolar++;
                } else {
                    deferred.push_back(tid);
                }
            }
            for (int tid : deferred) gridBacklog.push(tid);
        }

        // Phase B: Grid execution with Lyapunov-guided battery discharge
        double gridPowerThisSlot = 0.0;
        double gridUtil = 0.0;
        {
            vector<int> deferred;
            while (!gridBacklog.empty()) {
                int tid = gridBacklog.top(); gridBacklog.pop();
                if (executed[tid]) continue;
                if (t > tasks[tid].deadline) continue;

                double pPrev = (gridUtil > EPS) ? util_to_power(gridUtil) - P_S : 0.0;
                double pNext = util_to_power(gridUtil + tasks[tid].util) - P_S;
                double marginalPow = pNext - pPrev;

                // Lyapunov decision: discharge battery when Q > 0 (battery above target)
                double battSupply = 0.0;
                if (Q > 0 && battery > EPS) {
                    battSupply = min(battery, marginalPow);
                }
                double gridNeeded = marginalPow - battSupply;

                bool isUrgent = (tasks[tid].deadline <= t);
                if (isUrgent || gridUtil + tasks[tid].util <= U_MAX + EPS) {
                    gridUtil += tasks[tid].util;
                    battery -= battSupply;
                    gridPowerThisSlot += gridNeeded;
                    executed[tid] = true;
                    res.tasksOnGrid++;
                } else {
                    deferred.push_back(tid);
                }
            }
            for (int tid : deferred) gridBacklog.push(tid);
        }

        // Phase C: Lyapunov-guided pre-charging
        // When Q < 0 (battery below target), drift-plus-penalty favours charging.
        // Charge amount proportional to how far below target, weighted against
        // the grid cost rate for this window.
        if (Q < 0) {
            double currentRate = tier_rate_for(windowGridAccum[window] + gridPowerThisSlot);
            // Benefit of charging: reduces future grid need → save at future rate
            // Cost of charging now: currentRate * charge * SLOT_DURATION_HR
            // Lyapunov says charge when: -Q > V * currentRate * SLOT_DURATION_HR
            double lyapunovThreshold = V * currentRate * SLOT_DURATION_HR;
            if (-Q > lyapunovThreshold) {
                // Charge proportional to deficit, capped at cheap tier headroom
                double deficit = -Q;
                double cheapRemain = max(0.0, thetaTh - windowGridAccum[window]
                                         - gridPowerThisSlot);
                double batRoom = batteryCap - battery;
                double charge = min({deficit, cheapRemain, batRoom});
                if (charge > EPS) {
                    gridPowerThisSlot += charge;
                    battery += charge * leakage;
                }
            }
        }

        res.slotGridPower[t] = gridPowerThisSlot;
        windowGridAccum[window] += gridPowerThisSlot;
        res.windowGridUsage[window] = windowGridAccum[window];

        // Battery update: store solar surplus
        double solarConsumed = (solarUsedUtil > EPS) ?
                               util_to_power(solarUsedUtil) - P_S : 0.0;
        double solarSurplus = max(0.0, actualSolar[t] - solarConsumed);
        battery = min(batteryCap, battery + solarSurplus);
        res.slotBatteryLevel[t] = battery;
    }

    for (int m = 0; m < NUM_WINDOWS; m++) {
        res.windowGridCost[m] = window_tiered_cost(res.windowGridUsage[m]);
        res.totalGridCost += res.windowGridCost[m];
        res.totalGridEnergy += res.windowGridUsage[m];
    }
    for (size_t i = 0; i < tasks.size(); i++) {
        if (!executed[i]) res.tasksDropped++;
    }
    return res;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  OUTPUT HELPERS
 * ═══════════════════════════════════════════════════════════════════════ */

void printResult(const string& name, const SimResult& r, int /*totalTasks*/) {
    printf("  %-14s | Cost: %10.2f | GridEnergy: %10.2f\n",
           name.c_str(), r.totalGridCost, r.totalGridEnergy);
}

void csvRow(ofstream& csv, const string& scenario, double batteryCap, double leakage,
            const string& algo, const SimResult& r, int N) {
    csv << scenario << ","
        << batteryCap << ","
        << leakage << ","
        << algo << ","
        << r.totalGridCost << ","
        << r.totalGridEnergy << ","
        << r.tasksOnSolar << ","
        << r.tasksOnGrid << ","
        << r.tasksDropped << ","
        << (100.0 * (1.0 - (double)r.tasksDropped / N)) << "\n";
}

/* ═══════════════════════════════════════════════════════════════════════
 *  CLI ARGUMENT PARSER
 *
 *  Usage:
 *    ./hges --battery 10000 20000 --solar_dev 10 5 --task_dev 10 20
 *           --leakage 0.99 --output results.csv
 *
 *  --battery     : space-separated list of battery capacities
 *  --solar_dev   : space-separated solar deviation percentages (0=no error)
 *  --task_dev    : space-separated task deviation percentages (paired with solar_dev)
 *  --leakage     : battery leakage factor (default 0.99)
 *  --output      : CSV output filename (default: results.csv)
 * ═══════════════════════════════════════════════════════════════════════ */

struct CLIArgs {
    vector<double> batteryCaps;
    vector<int>    solarDevs;
    vector<int>    taskDevs;
    int            spike;
    int            billing;
    double         leakage;
    // Tier pricing
    double         tierTh1, tierTh2;       // W-hr per hour thresholds
    double         tierP1, tierP2, tierP3; // $/W-hr rates
    string         outputFile;
};

// Collect numeric values after a flag until next flag or end
vector<string> collectValues(int argc, char* argv[], int& i) {
    vector<string> vals;
    i++;  // skip the flag itself
    while (i < argc && argv[i][0] != '-') {
        vals.push_back(argv[i]);
        i++;
    }
    return vals;
}

CLIArgs parseArgs(int argc, char* argv[]) {
    CLIArgs args;
    args.spike = 25;
    args.billing = 60;
    args.leakage = 0.99;
    args.tierTh1 = 120.0; args.tierTh2 = 240.0;     // W-hr per hour
    args.tierP1 = 5.0; args.tierP2 = 7.0; args.tierP3 = 10.0;  // $/W-hr
    args.outputFile = "results.csv";

    int i = 1;
    while (i < argc) {
        string flag = argv[i];
        if (flag == "--battery") {
            auto vals = collectValues(argc, argv, i);
            for (auto& v : vals) args.batteryCaps.push_back(stod(v));
        } else if (flag == "--solar_dev") {
            auto vals = collectValues(argc, argv, i);
            for (auto& v : vals) args.solarDevs.push_back(stoi(v));
        } else if (flag == "--task_dev") {
            auto vals = collectValues(argc, argv, i);
            for (auto& v : vals) args.taskDevs.push_back(stoi(v));
        } else if (flag == "--spike") {
            i++;
            if (i < argc) { args.spike = stoi(argv[i]); i++; }
        } else if (flag == "--billing") {
            i++;
            if (i < argc) { args.billing = stoi(argv[i]); i++; }
        } else if (flag == "--tier_th") {
            // Expects 2 values: θ1 θ2 in W-hr per hour
            i++;
            if (i < argc) { args.tierTh1 = stod(argv[i]); i++; }
            if (i < argc && argv[i][0] != '-') { args.tierTh2 = stod(argv[i]); i++; }
        } else if (flag == "--tier_price") {
            // Expects 3 values: c1 c2 c3 in $/W-hr
            i++;
            if (i < argc) { args.tierP1 = stod(argv[i]); i++; }
            if (i < argc && argv[i][0] != '-') { args.tierP2 = stod(argv[i]); i++; }
            if (i < argc && argv[i][0] != '-') { args.tierP3 = stod(argv[i]); i++; }
        } else if (flag == "--leakage") {
            i++;
            if (i < argc) { args.leakage = stod(argv[i]); i++; }
        } else if (flag == "--output") {
            i++;
            if (i < argc) { args.outputFile = argv[i]; i++; }
        } else {
            cerr << "[WARN] Unknown flag: " << flag << "\n";
            i++;
        }
    }

    // Defaults if nothing provided
    if (args.batteryCaps.empty())
        args.batteryCaps = {10000};
    if (args.solarDevs.empty())
        args.solarDevs = {0};
    if (args.taskDevs.empty())
        args.taskDevs = {0};

    // Pad shorter list to match longer
    while (args.taskDevs.size() < args.solarDevs.size())
        args.taskDevs.push_back(args.taskDevs.back());
    while (args.solarDevs.size() < args.taskDevs.size())
        args.solarDevs.push_back(args.solarDevs.back());

    return args;
}

// solarFilePath() and taskFilePath() are defined in the FILE I/O section above

void printUsage() {
    cout << "Usage:\n"
         << "  ./hges --battery 10000 --solar_dev 5 --task_dev 5 --spike 25\n"
         << "         --billing 60 --leakage 0.99\n"
         << "         --tier_th 120 240 --tier_price 5 7 10\n"
         << "         --output results.csv\n\n"
         << "Flags (8 experimental parameters):\n"
         << "  --battery     Battery capacities in W·slot (space-separated)\n"
         << "  --solar_dev   Solar deviation %% (paired with task_dev)\n"
         << "  --task_dev    Task deviation %% (paired with solar_dev)\n"
         << "  --spike       Spike intensity %% at shock slots (default: 25)\n"
         << "  --billing     Billing window size in slots (default: 60 = 1hr)\n"
         << "  --leakage     Battery leakage factor h_l (default: 0.99)\n"
         << "  --tier_th     Tier thresholds: θ1 θ2 in W-hr/hr (default: 120 240)\n"
         << "  --tier_price  Tier prices: c1 c2 c3 in $/W-hr (default: 5 7 10)\n\n"
         << "Examples:\n"
         << "  --tier_th 120 240 --tier_price 5 7 10    (default: 3-tier)\n"
         << "  --tier_th 100 200 --tier_price 4 8 12    (custom pricing)\n"
         << "  --tier_th 80 160  --tier_price 3 6 15    (tight tiers, steep penalty)\n\n"
         << "Other:\n"
         << "  --output      Output CSV filename (default: results.csv)\n"
         << "  -h, --help    Show this help\n\n"
         << "Algorithms: NaiveGrid, GG, SDC, SP+SDC, Lyapunov,\n"
         << "            SF-Only (ablation), GF-Only (ablation), H-GES (proposed)\n";
}

/* ═══════════════════════════════════════════════════════════════════════
 *  MAIN
 * ═══════════════════════════════════════════════════════════════════════ */

int main(int argc, char* argv[]) {
    if (argc > 1 && (string(argv[1]) == "-h" || string(argv[1]) == "--help")) {
        printUsage();
        return 0;
    }

    CLIArgs args = parseArgs(argc, argv);

    cout << "═══════════════════════════════════════════════════════════════\n";
    cout << "  H-GES: Hybrid Green Edge Server Scheduling Simulator\n";
    cout << "═══════════════════════════════════════════════════════════════\n\n";

    // ── Setup dataset directories and base files ──
    setupBaseFiles();

    // Set runtime globals from CLI
    SPIKE_PCT = args.spike;
    BILLING_WINDOW_SIZE = args.billing;
    NUM_WINDOWS = NUM_SLOTS / BILLING_WINDOW_SIZE;
    TIER1_BASE_WHR = args.tierTh1;
    TIER2_BASE_WHR = args.tierTh2;
    TIER1_THRESHOLD = TIER1_BASE_WHR * BILLING_WINDOW_SIZE;
    TIER2_THRESHOLD = TIER2_BASE_WHR * BILLING_WINDOW_SIZE;
    TIER1_RATE = args.tierP1;
    TIER2_RATE = args.tierP2;
    TIER3_RATE = args.tierP3;

    // ── Pre-generate any missing deviation files ──
    for (size_t i = 0; i < args.solarDevs.size(); i++) {
        ensureSolarDeviation(args.solarDevs[i]);
        ensureTaskDeviation(args.taskDevs[i]);
    }
    cout << "\n";

    cout << "System parameters:\n";
    cout << "  Slots=" << NUM_SLOTS << "  Windows=" << NUM_WINDOWS
         << "  Slots/Window=" << BILLING_WINDOW_SIZE
         << " (" << BILLING_WINDOW_SIZE * SLOT_DURATION_HR * 60 << " min)\n";
    cout << "  P_S=" << P_S << "  P_MAX=" << P_MAX << "  U_MAX=" << U_MAX << "\n";
    cout << "  Tier thresholds (W·slot): T1=" << TIER1_THRESHOLD
         << "  T2=" << TIER2_THRESHOLD << "\n";
    cout << "  Tier rates ($/W-hr): " << TIER1_RATE << "/" << TIER2_RATE
         << "/" << TIER3_RATE << "\n";
    cout << "  Leakage h_l=" << args.leakage << "\n";
    cout << "  Spike intensity=" << args.spike << "%\n\n";

    cout << "Run configuration:\n";
    cout << "  Batteries: ";
    for (auto b : args.batteryCaps) cout << b << " ";
    cout << "\n  Scenarios:  ";
    for (size_t i = 0; i < args.solarDevs.size(); i++) {
        cout << "(S" << args.solarDevs[i] << "%,T" << args.taskDevs[i] << "%) ";
    }
    cout << "\n  Output:     " << args.outputFile << "\n\n";

    // Open CSV
    ofstream csv(args.outputFile);
    csv << "Scenario,BatteryCap,Leakage,Algorithm,GridCost,GridEnergy,"
        << "TasksSolar,TasksGrid,TasksDropped,Completion\n";

    // Run all combinations
    for (size_t si = 0; si < args.solarDevs.size(); si++) {
        int sd = args.solarDevs[si];
        int td = args.taskDevs[si];
        string scenarioName = "S" + to_string(sd) + "_T" + to_string(td);
        string sFile = solarFilePath(sd);
        string tFile = taskFilePath(td);

        cout << "╔══════════════════════════════════════════════════════════════╗\n";
        printf("║  Scenario: %-49s║\n", scenarioName.c_str());
        printf("║    Solar: %-50s║\n", sFile.c_str());
        printf("║    Tasks: %-50s║\n", tFile.c_str());
        cout << "╚══════════════════════════════════════════════════════════════╝\n";

        vector<Task>   tasks     = loadTasks(tFile);
        vector<double> predicted = loadSolarPower(BASE_SOLAR);  // offline plan uses base
        vector<double> actual    = loadSolarPower(sFile);        // runtime sees deviated
        int N = tasks.size();

        if (N == 0) {
            cerr << "[ERROR] No tasks loaded from " << tFile << ", skipping.\n";
            continue;
        }

        for (double cap : args.batteryCaps) {
            BATTERY_CAPACITY = cap;
            double hl = args.leakage;

            printf("\n  Battery = %.0f W·slot   Leakage = %.2f\n", cap, hl);
            cout << "  " << string(72, '-') << "\n";

            auto t0 = chrono::high_resolution_clock::now();

            // ── Run all methods ──
            SimResult rNaive    = runNaiveGrid(tasks, actual, cap, hl);
            SimResult rGG       = runGridGreedy(tasks, predicted, actual, cap, hl);
            SimResult rSDC      = runSDC(tasks, actual, cap, hl);
            SimResult rSPSDC    = runSPplusSDC(tasks, predicted, actual, cap, hl);
            SimResult rLyap     = runLyapunov(tasks, predicted, actual, cap, hl);
            SimResult rSFOnly   = runHGES_SolarOnly(tasks, predicted, actual, cap, hl);
            SimResult rGFOnly   = runHGES_GridOnly(tasks, predicted, actual, cap, hl);
            SimResult rHGES     = runHGES(tasks, predicted, actual, cap, hl);

            auto t1 = chrono::high_resolution_clock::now();
            double elapsed = chrono::duration<double>(t1 - t0).count();

            // Print: baselines, then ablations, then proposed
            printResult("NaiveGrid",    rNaive,  N);
            printResult("GG",           rGG,     N);
            printResult("SDC",          rSDC,    N);
            printResult("SP+SDC",       rSPSDC,  N);
            printResult("Lyapunov",     rLyap,   N);
            printResult("SF-Only",      rSFOnly, N);
            printResult("GF-Only",      rGFOnly, N);
            printResult("H-GES",        rHGES,   N);

            // Improvement summary
            printf("  ── H-GES improvement over ──\n");
            auto pctImprove = [](double base, double proposed) {
                if (base < 1e-6) return 0.0;
                return 100.0 * (base - proposed) / base;
            };
            printf("    NaiveGrid:    %+.2f%%\n", pctImprove(rNaive.totalGridCost, rHGES.totalGridCost));
            printf("    GG:           %+.2f%%\n", pctImprove(rGG.totalGridCost,    rHGES.totalGridCost));
            printf("    SDC:          %+.2f%%\n", pctImprove(rSDC.totalGridCost,   rHGES.totalGridCost));
            printf("    SP+SDC:       %+.2f%%\n", pctImprove(rSPSDC.totalGridCost, rHGES.totalGridCost));
            printf("    Lyapunov:     %+.2f%%\n", pctImprove(rLyap.totalGridCost,  rHGES.totalGridCost));
            printf("    SF-Only:      %+.2f%%\n", pctImprove(rSFOnly.totalGridCost,rHGES.totalGridCost));
            printf("    GF-Only:      %+.2f%%\n", pctImprove(rGFOnly.totalGridCost,rHGES.totalGridCost));
            printf("  (elapsed: %.3f s)\n", elapsed);

            // CSV rows
            csvRow(csv, scenarioName, cap, hl, "NaiveGrid",    rNaive,  N);
            csvRow(csv, scenarioName, cap, hl, "GG",           rGG,     N);
            csvRow(csv, scenarioName, cap, hl, "SDC",          rSDC,    N);
            csvRow(csv, scenarioName, cap, hl, "SP+SDC",       rSPSDC,  N);
            csvRow(csv, scenarioName, cap, hl, "Lyapunov",     rLyap,   N);
            csvRow(csv, scenarioName, cap, hl, "SF-Only",      rSFOnly, N);
            csvRow(csv, scenarioName, cap, hl, "GF-Only",      rGFOnly, N);
            csvRow(csv, scenarioName, cap, hl, "H-GES",        rHGES,   N);
        }
    }

    csv.close();
    cout << "\n═══════════════════════════════════════════════════════════════\n";
    cout << "  Results written to: " << args.outputFile << "\n";
    cout << "═══════════════════════════════════════════════════════════════\n";
    return 0;
}
