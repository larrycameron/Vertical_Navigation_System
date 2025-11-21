#include <algorithm>
#include <utility>
#include <string>
#include <cstdint>
#include <random>
#include <cmath>
#include <iostream>
#include <vector>
#include <fstream>

// ===================== VNAV MODES & STATE =====================

enum class VNAVMode : std::uint8_t
{
    OFF,      // VNAV is not active
    ARMED,
    CLIMB,    // VNAV climb mode
    PATH,     // VNAV path / descent
    ALT_CAP,  // capturing selected altitude
    ALT_HOLD  // holding selected altitude
};

std::string to_string(VNAVMode mode)
{
    switch (mode)
    {
    case VNAVMode::OFF:      return "OFF";
    case VNAVMode::ARMED:    return "ARMED";
    case VNAVMode::CLIMB:    return "CLIMB";
    case VNAVMode::PATH:     return "PATH";
    case VNAVMode::ALT_CAP:  return "ALT_CAP";
    case VNAVMode::ALT_HOLD: return "ALT_HOLD";
    }
    return "UNKNOWN";
}

struct VNAVState
{
    VNAVMode mode{VNAVMode::OFF};

    double current_altitude_ft{};   // actual aircraft altitude
    double selected_altitude_ft{};  // altitude preselector
    double target_speed_kts{};      // reference speed

    bool is_armed()    const noexcept { return mode == VNAVMode::ARMED; }
    bool is_climb()    const noexcept { return mode == VNAVMode::CLIMB; }
    bool is_path()     const noexcept { return mode == VNAVMode::PATH; }
    bool is_alt_cap()  const noexcept { return mode == VNAVMode::ALT_CAP; }
    bool is_alt_hold() const noexcept { return mode == VNAVMode::ALT_HOLD; }
};

struct VNAVCommand
{
    VNAVMode commanded_mode{VNAVMode::OFF};
    double target_vertical_speed_fpm{}; // climb / descent rate
    double target_speed_kts{};          // requested speed
};

class VNAVController
{
public:
    VNAVController() = default;

    VNAVCommand update(double current_altitude_ft,
                       double selected_altitude_ft,
                       double target_speed_kts)
    {
        m_state.current_altitude_ft  = current_altitude_ft;
        m_state.selected_altitude_ft = selected_altitude_ft;
        m_state.target_speed_kts     = target_speed_kts;

        VNAVCommand cmd{};

        switch (m_state.mode)
        {
        case VNAVMode::OFF:
            cmd.commanded_mode            = VNAVMode::OFF;
            cmd.target_vertical_speed_fpm = 0.0;
            cmd.target_speed_kts          = target_speed_kts;
            break;

        case VNAVMode::ARMED:
            cmd.commanded_mode            = VNAVMode::ARMED;
            cmd.target_vertical_speed_fpm = 0.0;
            cmd.target_speed_kts          = target_speed_kts;
            break;

        case VNAVMode::CLIMB:
            cmd.commanded_mode = VNAVMode::CLIMB;
            if (current_altitude_ft < selected_altitude_ft - 100.0)
            {
                cmd.target_vertical_speed_fpm = +1500.0;
            }
            else
            {
                m_state.mode                = VNAVMode::ALT_CAP;
                cmd.commanded_mode          = VNAVMode::ALT_CAP;
                cmd.target_vertical_speed_fpm = +500.0;
            }
            cmd.target_speed_kts = target_speed_kts;
            break;

        case VNAVMode::PATH:
            cmd.commanded_mode            = VNAVMode::PATH;
            cmd.target_vertical_speed_fpm = -1500.0; // simple fixed descent
            cmd.target_speed_kts          = target_speed_kts;
            break;

        case VNAVMode::ALT_CAP:
            cmd.commanded_mode = VNAVMode::ALT_CAP;

            if (std::fabs(current_altitude_ft - selected_altitude_ft) < 20.0)
            {
                m_state.mode                = VNAVMode::ALT_HOLD;
                cmd.commanded_mode          = VNAVMode::ALT_HOLD;
                cmd.target_vertical_speed_fpm = 0.0;
            }
            else
            {
                double sign = (selected_altitude_ft > current_altitude_ft) ? +1.0 : -1.0;
                cmd.target_vertical_speed_fpm = 300.0 * sign;
            }
            cmd.target_speed_kts = target_speed_kts;
            break;

        case VNAVMode::ALT_HOLD:
            cmd.commanded_mode            = VNAVMode::ALT_HOLD;
            cmd.target_vertical_speed_fpm = 0.0;
            cmd.target_speed_kts          = target_speed_kts;
            break;
        }

        return cmd;
    }

    void arm()          { m_state.mode = VNAVMode::ARMED; }
    void disarm()       { m_state.mode = VNAVMode::OFF; }
    void start_climb()  { m_state.mode = VNAVMode::CLIMB; }
    void start_descent(){ m_state.mode = VNAVMode::PATH; }

    const VNAVState& state() const noexcept { return m_state; }

private:
    VNAVState m_state{};
};

// ===================== DIAGNOSTIC LOGGER =====================

class VNAVDiagnosticLogger
{
public:
    explicit VNAVDiagnosticLogger(const std::string& filename)
        : m_file(filename)
    {
        if (m_file)
        {
            m_file << "step,"
                   << "mode,"
                   << "current_altitude_ft,"
                   << "selected_altitude_ft,"
                   << "target_speed_kts,"
                   << "cmd_mode,"
                   << "cmd_vs_fpm,"
                   << "cmd_speed_kts,"
                   << "distance_to_next_nm\n";
        }
    }

    bool is_open() const noexcept { return static_cast<bool>(m_file); }

    void log(std::uint64_t step,
             const VNAVState& state,
             const VNAVCommand& cmd,
             double distance_to_next_nautical_mile)
    {
        if (!m_file) return;

        m_file << step << ","
               << to_string(state.mode) << ","
               << state.current_altitude_ft << ","
               << state.selected_altitude_ft << ","
               << state.target_speed_kts << ","
               << to_string(cmd.commanded_mode) << ","
               << cmd.target_vertical_speed_fpm << ","
               << cmd.target_speed_kts << ","
               << distance_to_next_nautical_mile
               << "\n";
    }

private:
    std::ofstream m_file;
};

// ===================== ALTITUDE / WAYPOINT CONSTRAINTS =====================

enum class AltitudeConstraintType : std::uint8_t
{
    AT,
    AT_OR_ABOVE,
    AT_OR_BELOW,
    BETWEEN
};

struct AltitudeConstraint
{
    AltitudeConstraintType type{AltitudeConstraintType::AT};
    double lower_altitude_ft{};
    double upper_altitude_ft{};
};

struct WaypointConstraint
{
    double altitude_ft{};
    bool   use_mach{};
    double speed_value{};
};

// ===================== RANDOM GENERATOR & DISTRIBUTIONS =====================

std::mt19937 random_number_generator{12345};

std::uniform_real_distribution<double> waypoint_alt_ft{200.0, 50000.0};
std::uniform_real_distribution<double> mach_constraint{0.0, 1.8};
std::uniform_real_distribution<double> speed_kts{250.0, 700.0};
std::uniform_real_distribution<double> initial_alt_ft{0.0, 50000.0};
std::uniform_real_distribution<double> initial_mach{0.0, 1.8};
std::bernoulli_distribution use_mach_dist{0.86};

WaypointConstraint make_random_waypoint()
{
    WaypointConstraint wp{};
    wp.altitude_ft = waypoint_alt_ft(random_number_generator);
    wp.use_mach    = use_mach_dist(random_number_generator);

    if (wp.use_mach)
        wp.speed_value = mach_constraint(random_number_generator);
    else
        wp.speed_value = speed_kts(random_number_generator);

    return wp;
}

// ===================== FLIGHT ENVELOPE =====================

struct FlightEnvelope
{
    static constexpr double mach_cruise_min{0.86};
    static constexpr double mach_cruise_max{0.90};
    static constexpr double mach_max{1.8};

    static constexpr double ground_speed_lowalt_max_mph{700.0};
    static constexpr double ground_speed_lowalt_max_kts{
        ground_speed_lowalt_max_mph / 1.15078
    };

    static constexpr double low_alt_limit_ft{10000.0};

    static double clamp_mach(double mach_cmd, double altitude_ft)
    {
        if (altitude_ft < 25000.0 && mach_cmd > 1.0)
            mach_cmd = 1.0;

        if (mach_cmd > mach_max)
            mach_cmd = mach_max;

        return mach_cmd;
    }

    static double clamp_ground_speed(double ground_speed_kts, double altitude_ft)
    {
        if (altitude_ft < low_alt_limit_ft &&
            ground_speed_kts > ground_speed_lowalt_max_kts)
        {
            return ground_speed_lowalt_max_kts;
        }

        return ground_speed_kts;
    }
};

// ===================== ROUTE & DISTANCE =====================

struct Airport
{
    std::string code;
    double latitude_deg;
    double longitude_deg;
};

struct FlightPlan
{
    std::vector<Airport> airports{
        {"BWI", 39.177540, -76.668526},
        {"MCO", 28.424618, -81.310753},
        {"DFW", 32.89748,  -97.040443},
        {"IKA", 35.41611,  51.15222},
        {"ATL", 33.640411, -84.419853},
        {"HKG", 22.308046, 113.918480},
        {"LAX", 33.942791, -118.410042},
        {"HND", 35.553333, 139.781111},
        {"PEK", 40.08,     116.585},
        {"ICN", 37.4625,   126.439167},
    };

    std::vector<std::size_t> route_indices{0, 1, 2, 3, 4, 5, 6, 8, 9};
};

const Airport* find_airport(const FlightPlan& fp, const std::string& code)
{
    for (const auto& ap : fp.airports)
    {
        if (ap.code == code)
            return &ap;
    }
    return nullptr;
}

constexpr double mean_earth_radius{3958.8};
constexpr double PI{3.141592653589793};

inline double degrees_to_radians(double degrees) noexcept
{
    return degrees * PI / 180.0;
}

double great_circle_distance_miles(const Airport& a, const Airport& b)
{
    const double lat1 = degrees_to_radians(a.latitude_deg);
    const double lon1 = degrees_to_radians(a.longitude_deg);
    const double lat2 = degrees_to_radians(b.latitude_deg);
    const double lon2 = degrees_to_radians(b.longitude_deg);

    const double delta_longitude = lon2 - lon1;

    double cos_sigma =
        std::sin(lat1) * std::sin(lat2) +
        std::cos(lat1) * std::cos(lat2) * std::cos(delta_longitude);

    cos_sigma = std::clamp(cos_sigma, -1.0, 1.0);

    const double sigma    = std::acos(cos_sigma);
    const double distance = mean_earth_radius * sigma;

    return distance;
}

double total_route_distance_miles(const FlightPlan& fp)
{
    double total = 0.0;

    for (std::size_t i = 1; i < fp.route_indices.size(); ++i)
    {
        const auto from_idx = fp.route_indices[i - 1];
        const auto to_idx   = fp.route_indices[i];

        const Airport& from = fp.airports[from_idx];
        const Airport& to   = fp.airports[to_idx];

        total += great_circle_distance_miles(from, to);
    }

    return total;
}

// ===================== MAIN SIMULATION =====================

int main()
{
    FlightPlan fp;
    VNAVController vnav;
    VNAVDiagnosticLogger diag{"vnav_log.csv"};

    std::cout << "log open? " << std::boolalpha << diag.is_open() << "\n";

    double current_altitude_ft   = 0.0;
    double selected_altitude_ft  = 10000.0;
    double target_speed_kts      = 280.0;

    vnav.start_climb(); // VNAV into CLIMB mode

    const Airport& from = fp.airports[0];
    const Airport& to   = fp.airports[1];

    double leg_distance_mi      = great_circle_distance_miles(from, to);
    double distance_remaining_mi = leg_distance_mi;

    std::uint64_t step = 0;

while (step < 5000)  // run long enough for climb + cruise + descent
{
    VNAVCommand cmd = vnav.update(current_altitude_ft,
                                  selected_altitude_ft,
                                  target_speed_kts);

    // ==== AUTO-DESCENT LOGIC (NEW) ====
    if (vnav.state().is_alt_hold() && distance_remaining_mi < 200.0)
    {
        selected_altitude_ft = 3000.0;  
        vnav.start_descent();
    }

    // Simple physics
    current_altitude_ft += cmd.target_vertical_speed_fpm / 60.0;
    distance_remaining_mi -= 2.0;

    if (diag.is_open())
    {
        diag.log(step, vnav.state(), cmd, distance_remaining_mi / 1.15078);
    }

    ++step;
}


    std::cout << "Simulation finished, check vnav_log.csv\n";
    return 0;
}

    
    




 

