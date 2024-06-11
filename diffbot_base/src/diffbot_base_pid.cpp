
#include <diffbot_base/diffbot_base_pid.h>

namespace diffbot_base
{
    DiffbotBasePID::DiffbotBasePID(double p, double i, double d, double i_max, double i_min, bool antiwindup, double out_max, double out_min)
    : control_toolbox::Pid()
    , dynamic_reconfig_initialized_(false)
    {
        f_ = 0.0;
        initPid(p, i, d, i_max, i_min, antiwindup);
        error_ = 0.0;

        out_min_ = out_min;
        out_max_ = out_max;
    }

    void DiffbotBasePID::init(ros::NodeHandle& nh, double f, double p, double i, double d, double i_max, double i_min, bool antiwindup, double out_max, double out_min)
    {
        ROS_INFO("Initialize DiffbotBasePID");
        f_ = f;
        initPid(p, i, d, i_max, i_min, antiwindup);
        error_ = 0.0;

        out_min_ = out_min;
        out_max_ = out_max;

        initDynamicReconfig(nh);

        Gains gains = getGains();
        ROS_INFO_STREAM("Initialized DiffbotBasePID: F=" << f << ", P=" << gains.p_gain_ << ", I=" << gains.i_gain_ << ", D=" << gains.d_gain_ << ", out_min=" << out_min_ << ", out_max=" << out_max_);

    }

    double DiffbotBasePID::operator()(const double &measured_value, const double &setpoint, const ros::Duration &dt)
    {   
        // actually point can not be out of (min, max)
        double act_setpoint = clamp(setpoint, out_min_, out_max_);     
        // Compute error terms
        error_ = act_setpoint - measured_value;
        ROS_DEBUG_STREAM_THROTTLE(1, "Error: " << error_);

        // Reset the i_error in case the p_error and the setpoint is zero
        // Otherwise there will always be a constant i_error_ that won't vanish
        if (0.0 == setpoint && error_ == 0.00)
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
        double pid_output = computeCommand(error_, dt);
        ROS_DEBUG_STREAM_THROTTLE(1, "DiffbotBasePID computed command: " << pid_output);

        // Compute final output including feed forward term
        double output = f_ * setpoint + pid_output;
        double fin_output = clamp(output, out_min_, out_max_);

        ROS_DEBUG_STREAM("  pid_output:" << pid_output << ", output:" << output << ", fin_output:" << fin_output);
        return fin_output;
    }

    void DiffbotBasePID::getParameters(double &f, double &p, double &i, double &d, double &i_max, double &i_min)
    {
        bool antiwindup;
        getParameters(f, p, i, d, i_max, i_min, antiwindup);
    }

    void DiffbotBasePID::getParameters(double &f, double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        f = f_;
        // Call getGains from control_toolbox
        getGains(p, i, d, i_max, i_min, antiwindup);
    }

    void DiffbotBasePID::setParameters(double f, double p, double i, double d, double i_max, double i_min, bool antiwindup)
    {
        f_ = f;
        setGains(p, i, d, i_max, i_min, antiwindup);

        Gains gains = getGains();
        ROS_INFO_STREAM("Update DiffbotBasePID Gains: F=" << f << ", P=" << gains.p_gain_ << ", I=" << gains.i_gain_ << ", D=" << gains.d_gain_ << ", out_min=" << out_min_ << ", out_max=" << out_max_);
    }

    void DiffbotBasePID::setOutputLimits(double output_max, double output_min)
    {
        out_max_ = output_max;
        out_min_ = output_min;
        ROS_INFO_STREAM("Update DiffbotBasePID output limits: lower=" << out_min_ << ", upper=" << out_max_);
    }


    double DiffbotBasePID::clamp(const double& value, const double& lower_limit, const double& upper_limit)
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

    void DiffbotBasePID::initDynamicReconfig(ros::NodeHandle &node)
    {
        ROS_INFO_STREAM_NAMED("pid","Initializing dynamic reconfigure in namespace "
            << node.getNamespace());

        // Start dynamic reconfigure server
        param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, node));
        dynamic_reconfig_initialized_ = true;

        // Set Dynamic Reconfigure's gains to Pid's values
        updateDynamicReconfig();

        // Set callback
        param_reconfig_callback_ = boost::bind(&DiffbotBasePID::dynamicReconfigCallback, this, _1, _2);
        param_reconfig_server_->setCallback(param_reconfig_callback_);
        ROS_INFO_NAMED("pid", "Initialized dynamic reconfigure");
    }

    void DiffbotBasePID::updateDynamicReconfig()
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

    void DiffbotBasePID::updateDynamicReconfig(diffbot_base::ParametersConfig config)
    {
        // Make sure dynamic reconfigure is initialized
        if(!dynamic_reconfig_initialized_)
            return;

        // Set starting values, using a shared mutex with dynamic reconfig
        param_reconfig_mutex_.lock();
        param_reconfig_server_->updateConfig(config);
        param_reconfig_mutex_.unlock();
    }

    void DiffbotBasePID::dynamicReconfigCallback(diffbot_base::ParametersConfig &config, uint32_t /*level*/)
    {
        ROS_DEBUG_STREAM_NAMED("pid","Dynamics reconfigure callback recieved.");

        // Set the gains
        setParameters(config.f, config.p, config.i, config.d, config.i_clamp_max, config.i_clamp_min, config.antiwindup);
    }

}