#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <stdexcept>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <execution>  // for parallel for_each
#include <memory>

//=====================================================
//                 CIRCULAR BUFFER
//   (Stripped-down for illustration; adapt yours.)
//=====================================================
template<typename T, unsigned int size>
class CircularBuffer
{
public:
    explicit CircularBuffer(T initial_value = T()) 
    {
        clear(initial_value);
    }

    void clear(T val = T()) {
        head_ = 0;
        full_ = false;
        std::fill(data_.begin(), data_.end(), val);
    }

    void add(const T& value) {
        data_[head_] = value;
        head_ = (head_ + 1) % size;
        if (head_ == 0) {
            full_ = true;
        }
    }

    // Return up to n most recent samples [0..n-1], where data[0] is newest
    std::vector<T> get_snapshot(unsigned int n = size) const
    {
        if (n > size) {
            throw std::runtime_error("Request exceeds buffer capacity.");
        }

        std::vector<T> snapshot;
        snapshot.reserve(n);

        // Start from the newest
        int current = static_cast<int>(head_) - 1;
        if (current < 0) current += size;

        for (unsigned int i = 0; i < n; i++)
        {
            snapshot.push_back(data_[current]);
            current--;
            if (current < 0) {
                current += size;
                if (!full_ && static_cast<unsigned int>(current) < head_) {
                    // We’ve run out of valid data
                    break;
                }
            }
        }
        return snapshot;
    }

    // A convenience method to clamp outliers. 
    // (Used internally or can be made public.)
    static inline T clampOutlier(const T& val, double threshold)
    {
        if (std::fabs(val) > threshold) {
            return (val > 0.0) ? threshold : -threshold;
        }
        return val;
    }

private:
    std::array<T, size> data_{};
    unsigned int head_ = 0;
    bool full_ = false;
};


//=====================================================
//   TIME UTILS todo move
//=====================================================
inline double timeToDouble(const rclcpp::Time& t)
{
    return t.seconds();
}
inline double durationToDouble(const rclcpp::Duration& d)
{
    return d.seconds();
}

//=====================================================
//   Derivative Utilities (Templated best practice)
//=====================================================
/**
 * @brief First derivative via simple difference: (f[0] - f[1]) / dt
 */
template<typename T>
inline T firstOrderDifferenceDerivative(const std::vector<T>& values, double dt)
{
    if (values.size() < 2 || dt < 1e-9) {
        return T();
    }
    return (values[0] - values[1]) / dt;
}

/**
 * @brief Second-order difference for first derivative:
 *        (f[0] - 2f[1] + f[2]) / dt^2
 *        Interpreted as a "better" finite difference of derivative if you prefer.
 */
template<typename T>
inline T secondOrderDifferenceDerivative(const std::vector<T>& values, double dt)
{
    if (values.size() < 3 || dt < 1e-9) {
        return T();
    }
    return (values[0] - 2.0*values[1] + values[2]) / (dt*dt);
}

/**
 * @brief Second derivative, e.g. central finite difference:
 *        f''(n) ~ (f[n+1] - 2f[n] + f[n-1]) / (dt^2)
 *        Here we do it with newest as f[0], so it might be:
 *        (f[0] - 2f[1] + f[2]) / dt^2
 */
template<typename T>
inline T secondDerivative(const std::vector<T>& values, double dt)
{
    if (values.size() < 3 || dt < 1e-9) {
        return T();
    }
    // Interpreted at the newest sample. 
    return (values[0] - 2.0*values[1] + values[2]) / (dt*dt);
}

//=====================================================
//   Helper: Convert 6-element std::vector<double>
//           into geometry_msgs::msg::Wrench
//=====================================================
inline geometry_msgs::msg::Wrench vector6ToWrench(const std::vector<double>& v)
{
    geometry_msgs::msg::Wrench w;
    if (v.size() < 6) {
        return w; // or throw
    }
    w.force.x  = v[0];
    w.force.y  = v[1];
    w.force.z  = v[2];
    w.torque.x = v[3];
    w.torque.y = v[4];
    w.torque.z = v[5];
    return w;
}

//=====================================================
//   Parameter struct (move to yaml todo)
//=====================================================
struct ParametersForceSampling
{
    int N_lagrange = 5;         // #points for Lagrange
    double outlier_threshold = 1000.0;

    // You can add more params for filter, etc.
};

//=====================================================
//   ForceSamplingNode class
//=====================================================
class ForceSamplingNode : public rclcpp::Node
{
public:
    explicit ForceSamplingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("force_node_example", options)
    {
        // Initialize parameters (could load from param server, etc.)
        params_ = std::make_shared<ParametersForceSampling>();

        // Reinit the 6D buffer
        wrench_buffer_.reinit(6, 0.0);

        // Setup subscribers
        sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
            "wrench_in",
            10,
            std::bind(&ForceSamplingNode::forceCallback, this, std::placeholders::_1)
        );

        // Setup publishers for each of the 6 vectors
        pub_lagrange_   = create_publisher<geometry_msgs::msg::Wrench>("wrench_lagrange", 10);
        pub_ema_        = create_publisher<geometry_msgs::msg::Wrench>("wrench_ema", 10);
        pub_deriv_lagr_ = create_publisher<geometry_msgs::msg::Wrench>("wrench_deriv_lagrange", 10);
        pub_deriv_out_  = create_publisher<geometry_msgs::msg::Wrench>("wrench_deriv_outlier", 10);
        pub_deriv_so_   = create_publisher<geometry_msgs::msg::Wrench>("wrench_deriv_secondorder", 10);
        pub_second_der_ = create_publisher<geometry_msgs::msg::Wrench>("wrench_second_derivative", 10);

        RCLCPP_INFO(get_logger(), "ForceSamplingNode started.");
    }

private:
    // Subscription
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_lagrange_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_ema_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_deriv_lagr_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_deriv_out_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_deriv_so_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_second_der_;

    // Buffers
    static constexpr unsigned int CB_SIZE = 200;
    CircularBufferVector6<CB_SIZE> wrench_buffer_;
    CircularBuffer<double, CB_SIZE> time_buffer_;

    // EMA per dimension (for 6D)
    std::array<Filter::EMAFilter<double>, 6> ema_filters_;

    // Also an EMA for derivative if desired
    std::array<Filter::EMAFilter<double>, 6> ema_derivative_filters_;

    // Some housekeeping
    rclcpp::Time prev_time_;
    rclcpp::Time init_time_ = this->now();
    size_t counter_ = 0;

    // Shared parameters
    std::shared_ptr<ParametersForceSampling> params_;

    void forceCallback(const geometry_msgs::msg::WrenchStamped>::SharedPtr msg)
    {
        rclcpp::Time current_time = msg->header.stamp;

        if (counter_ == 0) {
            prev_time_ = current_time;
        }
        double dt = durationToDouble(current_time - prev_time_);
        prev_time_ = current_time;

        // 1) Store data in buffers: wrench + time
        std::array<double,6> wrench_in {
            msg->wrench.force.x,
            msg->wrench.force.y,
            msg->wrench.force.z,
            msg->wrench.torque.x,
            msg->wrench.torque.y,
            msg->wrench.torque.z
        };
        wrench_buffer_.add(wrench_in);
        time_buffer_.add(timeToDouble(current_time) - timeToDouble(init_time_));

        counter_++;

        // 2) Check if we have enough samples for Lagrange
        if (counter_ <= static_cast<size_t>(params_->N_lagrange + 1)) {
            // Not enough data yet
            return;
        }

        // 3) Grab time snapshot
        auto time_snap = time_buffer_.get_snapshot(params_->N_lagrange + 1);

        // 4) If out of order => flush
        if (!isSorted(time_snap)) {
            RCLCPP_ERROR(get_logger(), "Time buffer out of order => Flushing!");
            wrench_buffer_.reinit(6, 0.0);
            time_buffer_.clear();
            counter_ = 0;
            return;
        }

        // Allocate result containers
        std::vector<double> lagrange_6d(6, 0.0);
        std::vector<double> ema_6d(6, 0.0);

        std::vector<double> deriv_lagr_6d(6, 0.0);
        std::vector<double> deriv_out_6d(6, 0.0);
        std::vector<double> deriv_so_6d(6, 0.0);
        std::vector<double> second_der_6d(6, 0.0);

        // We'll process each dimension in parallel
        std::array<size_t, 6> dims = {0,1,2,3,4,5};

        std::for_each(std::execution::par, dims.begin(), dims.end(),
            [&](size_t dim)
        {
            // 5) Retrieve data for this dimension
            auto channel_snap = wrench_buffer_.get_snapshot_for_dim(
                dim, params_->N_lagrange + 1
            );

            // 6.a) Lagrange filter at t_eval = time_snap[0]
            double t_eval = time_snap[0];
            double lag_val = Lagrange::interpolate(t_eval, time_snap, channel_snap);
            lagrange_6d[dim] = lag_val;

            // 6.b) EMA filter on newest sample
            double ema_val = ema_filters_[dim].apply(channel_snap[0]);
            ema_6d[dim] = ema_val;

            // 7.a) Lagrange derivative
            double lag_der = Lagrange::derivative(t_eval, time_snap, channel_snap);
            deriv_lagr_6d[dim] = lag_der;

            // 7.b) Simple difference + outlier clamp
            double sd = simpleDifference(channel_snap, dt);
            sd = CircularBuffer<double, CB_SIZE>::clampOutlier(sd, params_->outlier_threshold);
            // optional derivative filtering
            double sd_filt = ema_derivative_filters_[dim].apply(sd);
            deriv_out_6d[dim] = sd_filt;

            // 7.c) Second-order difference for first derivative
            double so = secondOrderDifference(channel_snap, dt);
            so = CircularBuffer<double, CB_SIZE>::clampOutlier(so, params_->outlier_threshold);
            deriv_so_6d[dim] = so;

            // 8) Second derivative
            double sderiv = secondDerivative(channel_snap, dt);
            sderiv = CircularBuffer<double, CB_SIZE>::clampOutlier(sderiv, params_->outlier_threshold);
            second_der_6d[dim] = sderiv;
        });

        // 9) Publish all
        pub_lagrange_->publish( vector6ToWrench(lagrange_6d) );
        pub_ema_->publish(      vector6ToWrench(ema_6d) );
        pub_deriv_lagr_->publish( vector6ToWrench(deriv_lagr_6d) );
        pub_deriv_out_->publish(  vector6ToWrench(deriv_out_6d) );
        pub_deriv_so_->publish(   vector6ToWrench(deriv_so_6d) );
        pub_second_der_->publish( vector6ToWrench(second_der_6d) );
    }
};

//=====================================================
//   MAIN
//=====================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForceSamplingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
