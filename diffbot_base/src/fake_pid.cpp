
#include <diffbot_base/fake_pid.h>

namespace diffbot_base
{
    FakePID::FakePID(double p, double i, double d, double i_max, double i_min, bool antiwindup, double out_max, double out_min)
    : control_toolbox::Pid()
    , dynamic_reconfig_initialized_(false)
    {
        f_ = 0.0;
        initPid(p, i, d, i_max, i_min, antiwindup);
        error_ = 0.0;

        out_min_ = out_min;
        out_max_ = out_max;
    }

    void FakePID::init(ros::NodeHandle& nh, double f, double p, double i, double d, double i_max, double i_min, bool antiwindup, double out_max, double out_min)
    {
        ROS_INFO("Initialize FakePID");
        f_ = f;
        
        error_ = 0.0;
        initPid(p, i, d, i_max, i_min, antiwindup);

        out_min_ = out_min;
        out_max_ = out_max;

        initDynamicReconfig(nh);

        Gains gains = getGains();
        ROS_INFO_STREAM("Initialized FakePID: F=" << f << ", P=" << gains.p_gain_ << ", I=" << gains.i_gain_ << ", D=" << gains.d_gain_ << ", out_min=" << out_min_ << ", out_max=" << out_max_);

    }

    double FakePID::operator()(const double &measured_value, const double &setpoint, const ros::Duration &dt)
    {
        // Compute error terms
        error_ = setpoint - measured_value;
        ROS_DEBUG_STREAM_THROTTLE(1, "Error: " << error_);

        // Reset the i_error in case the p_error and the setpoint is zero
        // Otherwise there will always be a constant i_error_ that won't vanish
        if (0.0 == setpoint && 0.0 == error_)
        {
            // reset() will reset
            // p_error_last_ = 0.0;
            // p_error_ = 0.0;
            // i_error_ = 0.0;
            // d_error_ = 0.0;
            // cmd_ = 0.0;
            reset();
        }

        // Use control_toolbox::Pid::computeCommand()
        double output = setpoint;
        ROS_DEBUG_STREAM_THROTTLE(1, "FakePID return setpoint as command: " << output);

        return output;
    }

    void FakePID::getParameters(double &f, double &p, double &i, double &d, double &i_max, double &i_min)
    {
        bool antiwindup;
        getParameters(f, p, i, d, i_max, i_min, antiwindup);
    }

    void FakePID::getParameters(double &f, double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        f = f_;
        // Call getGains from control_toolbox
        getGains(p, i, d, i_max, i_min, antiwindup);
    }

    void FakePID::setParameters(double f, double p, double i, double d, double i_max, double i_min, bool antiwindup)
    {
        f_ = f;
        setGains(p, i, d, i_max, i_min, antiwindup);

        Gains gains = getGains();
        ROS_INFO_STREAM("Update FakePID Gains: F=" << f << ", P=" << gains.p_gain_ << ", I=" << gains.i_gain_ << ", D=" << gains.d_gain_ << ", out_min=" << out_min_ << ", out_max=" << out_max_);
    }

    void FakePID::setOutputLimits(double output_max, double output_min)
    {
        out_max_ = output_max;
        out_min_ = output_min;
        ROS_INFO_STREAM("Update FakePID output limits: lower=" << out_min_ << ", upper=" << out_max_);
    }


    double FakePID::clamp(const double& value, const double& lower_limit, const double& upper_limit)
    {
        if (value > upper_limit)
        {
            ROS_DEBUG_STREAM_THROTTLE(1, "Clamp " << value << " to upper limit " << upper_limit);
            return upper_limit;
        }
        else if (value < lower_limit)
        {
            ROS_DEBUG_STREAM_THROTTLE(1, "Clamp " << value << " to lower limit " << upper_limit);
            return lower_limit;
        }

        return value;
    }

    void FakePID::initDynamicReconfig(ros::NodeHandle &node)
    {
        ROS_INFO_STREAM_NAMED("fake_pid","Initializing dynamic reconfigure in namespace "
            << node.getNamespace());

        // Start dynamic reconfigure server
        param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, node));
        dynamic_reconfig_initialized_ = true;

        // Set Dynamic Reconfigure's gains to Pid's values
        updateDynamicReconfig();

        // Set callback
        param_reconfig_callback_ = boost::bind(&FakePID::dynamicReconfigCallback, this, _1, _2);
        param_reconfig_server_->setCallback(param_reconfig_callback_);
        ROS_INFO_NAMED("fake_pid", "Initialized dynamic reconfigure");
    }

    void FakePID::updateDynamicReconfig()
    {
        // Make sure dynamic reconfigure is initialized
        if(!dynamic_reconfig_initialized_)
            return;

        // Get starting values
        diffbot_base::ParametersConfig config;
        config.f = f_;

        // Get starting values
        getGains(config.p, config.i, config.d, config.i_clamp_max, config.i_clamp_min, config.antiwindup);

        updateDynamicReconfig(config);
    }

    void FakePID::updateDynamicReconfig(diffbot_base::ParametersConfig config)
    {
        // Make sure dynamic reconfigure is initialized
        if(!dynamic_reconfig_initialized_)
            return;

        // Set starting values, using a shared mutex with dynamic reconfig
        param_reconfig_mutex_.lock();
        param_reconfig_server_->updateConfig(config);
        param_reconfig_mutex_.unlock();
    }

    void FakePID::dynamicReconfigCallback(diffbot_base::ParametersConfig &config, uint32_t /*level*/)
    {
        ROS_DEBUG_STREAM_NAMED("fake_pid","Dynamics reconfigure callback recieved.");

        // Set the gains
        setParameters(config.f, config.p, config.i, config.d, config.i_clamp_max, config.i_clamp_min, config.antiwindup);
    }

}