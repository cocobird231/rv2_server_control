/**
 * fake_unitree_api_node.cpp  (test / validation composable node)
 *
 * Mimics the on-robot Unitree high-level sport API server.
 *
 * Subscribes : /api/sport/request  (unitree_api::msg::Request)
 * Publishes  : /api/sport/response (unitree_api::msg::Response)
 *
 * For every incoming request the node:
 *   1. Decodes the api_id and any JSON parameter.
 *   2. Logs a human-readable summary of the command.
 *   3. Publishes a success Response (status.code = 0).
 *   4. Accumulates per-API call counters; prints stats every 5 s.
 *
 * All API IDs are taken directly from ros2_b2_sport_client.h.
 */

#include <rclcpp/rclcpp.hpp>
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <map>
#include <string>
#include <sstream>


// ── API ID registry ──────────────────────────────────────────────────────────

static constexpr int64_t API_DAMP              = 1001;
static constexpr int64_t API_BALANCE_STAND     = 1002;
static constexpr int64_t API_STOPMOVE          = 1003;
static constexpr int64_t API_STANDUP           = 1004;
static constexpr int64_t API_STANDDOWN         = 1005;
static constexpr int64_t API_RECOVERY_STAND    = 1006;
static constexpr int64_t API_MOVE              = 1008;
static constexpr int64_t API_SWITCH_GAIT       = 1011;
static constexpr int64_t API_BODY_HEIGHT       = 1013;
static constexpr int64_t API_SPEED_LEVEL       = 1015;
static constexpr int64_t API_TRAJECTORY_FOLLOW = 1018;
static constexpr int64_t API_CONTINUOUS_GAIT   = 1019;
static constexpr int64_t API_MOVE_TO_POS       = 1036;
static constexpr int64_t API_SWITCH_MOVE_MODE  = 1038;
static constexpr int64_t API_HAND_STAND        = 1039;
static constexpr int64_t API_AUTO_RECOVERY_SET = 1040;
static constexpr int64_t API_FREE_WALK         = 1045;
static constexpr int64_t API_CLASSIC_WALK      = 1049;
static constexpr int64_t API_FAST_WALK         = 1050;
static constexpr int64_t API_EULER             = 1051;
static constexpr int64_t API_VISION_WALK       = 1101;

static const std::map<int64_t, std::string> API_NAMES = {
    {API_DAMP,              "Damp"},
    {API_BALANCE_STAND,     "BalanceStand"},
    {API_STOPMOVE,          "StopMove"},
    {API_STANDUP,           "StandUp"},
    {API_STANDDOWN,         "StandDown"},
    {API_RECOVERY_STAND,    "RecoveryStand"},
    {API_MOVE,              "Move"},
    {API_SWITCH_GAIT,       "SwitchGait"},
    {API_BODY_HEIGHT,       "BodyHeight"},
    {API_SPEED_LEVEL,       "SpeedLevel"},
    {API_TRAJECTORY_FOLLOW, "TrajectoryFollow"},
    {API_CONTINUOUS_GAIT,   "ContinuousGait"},
    {API_MOVE_TO_POS,       "MoveToPos"},
    {API_SWITCH_MOVE_MODE,  "SwitchMoveMode"},
    {API_HAND_STAND,        "HandStand"},
    {API_AUTO_RECOVERY_SET, "AutoRecoverySet"},
    {API_FREE_WALK,         "FreeWalk"},
    {API_CLASSIC_WALK,      "ClassicWalk"},
    {API_FAST_WALK,         "FastWalk"},
    {API_EULER,             "Euler"},
    {API_VISION_WALK,       "VisionWalk"},
};


// ── Parameter decoder ─────────────────────────────────────────────────────────

static std::string decodeParam(int64_t api_id, const std::string & param_str)
{
    if (param_str.empty()) return "";

    nlohmann::json js;
    try { js = nlohmann::json::parse(param_str); }
    catch (...) { return "(parse error: " + param_str + ")"; }

    std::ostringstream ss;
    switch (api_id)
    {
        case API_MOVE:
            ss << "vx=" << js.value("x", 0.0f)
               << " vy=" << js.value("y", 0.0f)
               << " vyaw=" << js.value("z", 0.0f);
            break;
        case API_SWITCH_GAIT:
            ss << "gait=" << js.value("data", -1);
            break;
        case API_BODY_HEIGHT:
            ss << "height=" << js.value("data", 0.0f);
            break;
        case API_SPEED_LEVEL:
            ss << "level=" << js.value("data", -1);
            break;
        case API_CONTINUOUS_GAIT:
        case API_SWITCH_MOVE_MODE:
        case API_HAND_STAND:
        case API_AUTO_RECOVERY_SET:
        case API_CLASSIC_WALK:
        case API_FAST_WALK:
        case API_VISION_WALK:
            ss << "flag=" << (js.value("data", false) ? "true" : "false");
            break;
        case API_MOVE_TO_POS:
            ss << "x=" << js.value("x", 0.0f)
               << " y=" << js.value("y", 0.0f)
               << " yaw=" << js.value("yaw", 0.0f);
            break;
        case API_EULER:
            ss << "roll=" << js.value("x", 0.0f)
               << " pitch=" << js.value("y", 0.0f)
               << " yaw=" << js.value("z", 0.0f);
            break;
        case API_TRAJECTORY_FOLLOW:
            if (js.contains("param") && js["param"].is_array())
                ss << "waypoints=" << js["param"].size();
            else
                ss << "waypoints=?";
            break;
        default:
            if (!js.empty()) ss << js.dump();
            break;
    }
    return ss.str();
}


// ── FakeUnitreeApiNode ────────────────────────────────────────────────────────

class FakeUnitreeApiNode : public rclcpp::Node
{
public:
    explicit FakeUnitreeApiNode(const rclcpp::NodeOptions & options)
        : Node("fake_unitree_api", options)
    {
        req_sub_ = this->create_subscription<unitree_api::msg::Request>(
            "/api/sport/request", rclcpp::QoS(10),
            [this](const unitree_api::msg::Request::SharedPtr msg) {
                onRequest(msg);
            });

        res_pub_ = this->create_publisher<unitree_api::msg::Response>(
            "/api/sport/response", rclcpp::QoS(10));

        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() { printStats(); });

        RCLCPP_INFO(this->get_logger(),
            "FakeUnitreeApiNode started — listening on /api/sport/request");
    }

private:
    void onRequest(const unitree_api::msg::Request::SharedPtr & msg)
    {
        const int64_t api_id = msg->header.identity.api_id;
        const int64_t req_id = msg->header.identity.id;

        auto name_it = API_NAMES.find(api_id);
        const std::string api_name =
            (name_it != API_NAMES.end()) ? name_it->second : "UNKNOWN";

        const std::string param_desc = decodeParam(api_id, msg->parameter);

        // MOVE is sent at the output rate (up to 20 Hz); log it at DEBUG to avoid
        // flooding the terminal.  All other commands are infrequent enough for INFO.
        if (api_id == API_MOVE) {
            RCLCPP_DEBUG(this->get_logger(),
                "[RX] api_id=%-5ld  %-18s  %s  (req_id=%ld)",
                api_id, api_name.c_str(), param_desc.c_str(), req_id);
        } else if (param_desc.empty()) {
            RCLCPP_INFO(this->get_logger(),
                "[RX] api_id=%-5ld  %-18s  (req_id=%ld)",
                api_id, api_name.c_str(), req_id);
        } else {
            RCLCPP_INFO(this->get_logger(),
                "[RX] api_id=%-5ld  %-18s  %s  (req_id=%ld)",
                api_id, api_name.c_str(), param_desc.c_str(), req_id);
        }

        call_counts_[api_id]++;

        unitree_api::msg::Response res;
        res.header.identity.id     = req_id;
        res.header.identity.api_id = api_id;
        res.header.status.code     = 0;
        res.data                   = "{}";
        res_pub_->publish(res);
    }

    void printStats()
    {
        if (call_counts_.empty()) {
            RCLCPP_INFO(this->get_logger(), "[STATS] No requests received yet.");
            return;
        }
        RCLCPP_INFO(this->get_logger(),
            "[STATS] ── Cumulative request counts ──────────────");
        for (const auto & [api_id, count] : call_counts_) {
            auto name_it = API_NAMES.find(api_id);
            const std::string name =
                (name_it != API_NAMES.end()) ? name_it->second : "UNKNOWN";
            RCLCPP_INFO(this->get_logger(),
                "        api_id=%-5ld  %-18s  calls=%lu",
                api_id, name.c_str(), static_cast<unsigned long>(count));
        }
        RCLCPP_INFO(this->get_logger(),
            "[STATS] ────────────────────────────────────────────");
    }

    rclcpp::Subscription<unitree_api::msg::Request>::SharedPtr req_sub_;
    rclcpp::Publisher<unitree_api::msg::Response>::SharedPtr   res_pub_;
    rclcpp::TimerBase::SharedPtr                               stats_timer_;
    std::map<int64_t, uint64_t>                                call_counts_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(FakeUnitreeApiNode)
