#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <algorithm>
#include <cmath>
#include <string>
#include <cstdlib>
#include <sstream>
#include <unordered_map>

class LLMControllerNode : public rclcpp::Node
{
public:
    LLMControllerNode() : Node("p3dx_llm_controller"), emergency_stop_active_(false)
    {
        // Declare and get parameters (defaults match velocity_limits.yaml)
        this->declare_parameter("max_linear_velocity", 0.25);
        this->declare_parameter("max_angular_velocity", 0.5);
        this->declare_parameter("max_command_duration", 15.0);
        this->declare_parameter("cmd_vel_publish_rate", 20.0);
        this->declare_parameter("llm_model", "gemini-1.5-flash");
        this->declare_parameter("gemini_api_key", "YOUR_API_KEY_HERE");
        this->declare_parameter("llm_timeout", 5.0);

        max_linear_vel_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_velocity").as_double();
        max_duration_ = this->get_parameter("max_command_duration").as_double();
        publish_rate_ = this->get_parameter("cmd_vel_publish_rate").as_double();
        llm_model_ = this->get_parameter("llm_model").as_string();
        api_key_ = this->get_parameter("gemini_api_key").as_string();

        // Create publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Create subscriber
        llm_command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "llm_command", 10,
            std::bind(&LLMControllerNode::llmCommandCallback, this, std::placeholders::_1));

        // Create services
        emergency_stop_srv_ = this->create_service<std_srvs::srv::Empty>(
            "emergency_stop",
            std::bind(&LLMControllerNode::emergencyStopCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Validate API key
        if (api_key_ == "YOUR_API_KEY_HERE" || api_key_.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "Gemini API key not configured! Please set gemini_api_key parameter.");
            RCLCPP_WARN(this->get_logger(),
                        "Get your free API key from: https://aistudio.google.com/apikey");
        }

        RCLCPP_INFO(this->get_logger(), "P3DX LLM Controller Node initialized");
        RCLCPP_INFO(this->get_logger(), "Max linear velocity: %.2f m/s", max_linear_vel_);
        RCLCPP_INFO(this->get_logger(), "Max angular velocity: %.2f rad/s", max_angular_vel_);
        RCLCPP_INFO(this->get_logger(), "Max command duration: %.2f s", max_duration_);
        RCLCPP_INFO(this->get_logger(), "Command caching enabled (starts empty each run)");
    }

private:
    void llmCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (emergency_stop_active_) {
            RCLCPP_WARN(this->get_logger(), "Emergency stop active! Ignoring command.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received LLM command: '%s'", msg->data.c_str());

        MotionCommand cmd;

        // Check cache first
        auto cache_it = command_cache_.find(msg->data);
        if (cache_it != command_cache_.end()) {
            // Cache hit! Use cached command
            cmd = cache_it->second;
            RCLCPP_INFO(this->get_logger(), "✓ Cache HIT! Instant execution (no LLM call)");
        } else {
            // Cache miss - call LLM
            RCLCPP_INFO(this->get_logger(), "Cache miss - calling Gemini LLM...");

            std::string parsed_result = callPythonLLM(msg->data);

            if (parsed_result.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse command with LLM");
                return;
            }

            // Parse the JSON result
            cmd = parseMotionCommand(parsed_result);

            if (!validateCommand(cmd)) {
                RCLCPP_ERROR(this->get_logger(), "Invalid command from LLM");
                return;
            }

            // Cap velocities to safe limits
            capVelocities(cmd);

            // Store in cache for next time
            command_cache_[msg->data] = cmd;
            RCLCPP_INFO(this->get_logger(), "Cached command '%s' (cache size: %zu)",
                       msg->data.c_str(), command_cache_.size());
        }

        // Execute the command
        executeCommand(cmd);
    }

    void emergencyStopCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
    {
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED!");
        emergency_stop_active_ = true;
        stopRobot();
    }

    std::string callPythonLLM(const std::string& command_text)
    {
        // Use source directory path (works during development)
        // In production, could use installed path from share directory
        std::string python_script = "/home/robot/vishwa/p3dx_ws/src/p3dx_llm_control/src/llm_interface.py";
        std::string venv_python = "/home/robot/vishwa/p3dx_ws/.venv/bin/python3";

        // Call venv python directly (no need to activate)
        std::stringstream cmd;
        cmd << venv_python << " " << python_script << " "
            << "\"" << api_key_ << "\" "
            << "\"" << command_text << "\"";

        // Execute Python script and capture output
        FILE* pipe = popen(cmd.str().c_str(), "r");
        if (!pipe) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute Python LLM interface");
            return "";
        }

        char buffer[1024];
        std::string result = "";
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }
        pclose(pipe);

        // Extract JSON from output (look for lines starting with "Parsed:")
        size_t parsed_pos = result.find("Parsed:");
        if (parsed_pos != std::string::npos) {
            result = result.substr(parsed_pos + 7); // Skip "Parsed:"
        }

        return result;
    }

    struct MotionCommand {
        std::string action;
        double linear_velocity;
        double angular_velocity;
        double duration;
    };

    MotionCommand parseMotionCommand(const std::string& json_str)
    {
        MotionCommand cmd;
        cmd.action = "stop";
        cmd.linear_velocity = 0.0;
        cmd.angular_velocity = 0.0;
        cmd.duration = 0.0;

        // Simple JSON parsing (looking for key-value pairs)
        // This is a basic implementation - for production, use a proper JSON library

        auto extractValue = [](const std::string& str, const std::string& key) -> std::string {
            size_t key_pos = str.find("\"" + key + "\"");
            if (key_pos == std::string::npos) return "";

            size_t colon_pos = str.find(":", key_pos);
            if (colon_pos == std::string::npos) return "";

            size_t value_start = str.find_first_not_of(" \t\n", colon_pos + 1);
            if (value_start == std::string::npos) return "";

            size_t value_end;
            if (str[value_start] == '"') {
                value_start++;
                value_end = str.find('"', value_start);
            } else {
                value_end = str.find_first_of(",}", value_start);
            }

            return str.substr(value_start, value_end - value_start);
        };

        cmd.action = extractValue(json_str, "action");
        cmd.linear_velocity = std::stod(extractValue(json_str, "linear_velocity"));
        cmd.angular_velocity = std::stod(extractValue(json_str, "angular_velocity"));
        cmd.duration = std::stod(extractValue(json_str, "duration"));

        return cmd;
    }

    bool validateCommand(const MotionCommand& cmd)
    {
        if (std::abs(cmd.linear_velocity) > max_linear_vel_ * 1.2) {
            RCLCPP_ERROR(this->get_logger(), "Linear velocity %.2f exceeds maximum",
                        cmd.linear_velocity);
            return false;
        }

        if (std::abs(cmd.angular_velocity) > max_angular_vel_ * 1.2) {
            RCLCPP_ERROR(this->get_logger(), "Angular velocity %.2f exceeds maximum",
                        cmd.angular_velocity);
            return false;
        }

        if (cmd.duration < 0 || cmd.duration > max_duration_ * 1.1) {
            RCLCPP_ERROR(this->get_logger(), "Duration %.2f is invalid", cmd.duration);
            return false;
        }

        return true;
    }

    void capVelocities(MotionCommand& cmd)
    {
        cmd.linear_velocity = std::clamp(cmd.linear_velocity, -max_linear_vel_, max_linear_vel_);
        cmd.angular_velocity = std::clamp(cmd.angular_velocity, -max_angular_vel_, max_angular_vel_);
        cmd.duration = std::min(cmd.duration, max_duration_);
    }

    void executeCommand(const MotionCommand& cmd)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Executing: action=%s, linear=%.2f m/s, angular=%.2f rad/s, duration=%.2f s",
                    cmd.action.c_str(), cmd.linear_velocity, cmd.angular_velocity, cmd.duration);

        // Cancel any existing timer
        if (cmd_timer_) {
            cmd_timer_->cancel();
        }

        // Store current command
        current_cmd_ = cmd;

        // Create twist message
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = cmd.linear_velocity;
        twist.angular.z = cmd.angular_velocity;

        // Create timer to publish at specified rate
        auto period = std::chrono::duration<double>(1.0 / publish_rate_);
        cmd_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this, twist]() {
                if (!emergency_stop_active_) {
                    cmd_vel_pub_->publish(twist);
                }
            });

        // Create timer to stop after duration
        auto duration_period = std::chrono::duration<double>(cmd.duration);
        stop_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(duration_period),
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Command duration completed, stopping");
                stopRobot();
                stop_timer_->cancel();
            });
    }

    void stopRobot()
    {
        if (cmd_timer_) {
            cmd_timer_->cancel();
        }

        auto stop_msg = geometry_msgs::msg::Twist();
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_msg);

        RCLCPP_INFO(this->get_logger(), "Robot stopped");
    }

    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr llm_command_sub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr emergency_stop_srv_;

    rclcpp::TimerBase::SharedPtr cmd_timer_;
    rclcpp::TimerBase::SharedPtr stop_timer_;

    double max_linear_vel_;
    double max_angular_vel_;
    double max_duration_;
    double publish_rate_;
    std::string llm_model_;
    std::string api_key_;

    bool emergency_stop_active_;
    MotionCommand current_cmd_;

    // Command cache: maps command text -> parsed motion command
    std::unordered_map<std::string, MotionCommand> command_cache_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LLMControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
