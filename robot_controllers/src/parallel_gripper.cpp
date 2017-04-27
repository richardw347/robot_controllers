#include <pluginlib/class_list_macros.h>
#include <robot_controllers/parallel_gripper.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::ParallelGripperController, robot_controllers::Controller)

namespace robot_controllers {

    int ParallelGripperController::init(ros::NodeHandle &nh, ControllerManager *manager) {
        // We absolutely need access to the controller manager
        if (!manager) {
            initialized_ = false;
            return -1;
        }

        Controller::init(nh, manager);
        manager_ = manager;

        // Setup Joints */
        std::string l_name, r_name;
        nh.param<std::string>("l_gripper_joint", l_name, "l_gripper_finger_joint");
        nh.param<std::string>("r_gripper_joint", r_name, "r_gripper_finger_joint");

        left_ = manager_->getJointHandle(l_name);
        right_ = manager_->getJointHandle(r_name);

        // Checking to see if Joint Handles exists
        if (!left_) {
            ROS_ERROR_NAMED("ParallelGripperController",
                            "Unable to retreive joint (%s), Namespace: %s/l_gripper_joint",
                            l_name.c_str(), nh.getNamespace().c_str());
            return -1;
        }

        if (!right_) {
            ROS_ERROR_NAMED("ParallelGripperController",
                            "Unable to retreive joint (%s), Namespace: %s/r_gripper_joint",
                            r_name.c_str(), nh.getNamespace().c_str());
            return -1;
        }

        // Start action server
        server_.reset(new server_t(nh, "",
                                   boost::bind(&ParallelGripperController::executeCb, this, _1),
                                   false));
        server_->start();

        // Get Params
        nh.param<double>("max_position", min_position_, 0.01);
        nh.param<double>("max_position", max_position_, 0.1);
        nh.param<double>("max_effort", max_effort_, 10.0);

        // PID controller for centering the gripper
        if (centering_pid_.init(ros::NodeHandle(nh, "centering"))) {
            use_centering_controller_ = false;
        }

        // Set default to max
        goal_ = min_position_;
        effort_ = max_effort_;

        initialized_ = true;
        return 0;
    }

    bool ParallelGripperController::start() {
        if (!initialized_) {
            ROS_ERROR_NAMED("ParallelGripperController",
                            "Unable to start, not initialized.");
            return false;
        }

        if (!server_->isActive()) {
            ROS_ERROR_NAMED("ParallelGripperController",
                            "Unable to start, action server is not active.");
            return false;
        }

        return true;
    }

    bool ParallelGripperController::stop(bool force) {
        if (!initialized_)
            return true;

        if (server_->isActive()) {
            if (force) {
                // Shut down the action
                server_->setPreempted();
                return true;
            }

            // Do not abort unless forced
            return false;
        }

        // Just holding position, go ahead and stop us
        return true;
    }

    bool ParallelGripperController::reset() {
        // Nothing to do here
        return true;
    }

    void ParallelGripperController::update(const ros::Time &now, const ros::Duration &dt) {
        if (!initialized_)
            return;
        left_->setPosition(goal_, 0, effort_);
        right_->setPosition(goal_, 0, effort_);
    }

    void ParallelGripperController::executeCb(const control_msgs::GripperCommandGoalConstPtr &goal) {
        control_msgs::GripperCommandFeedback feedback;
        control_msgs::GripperCommandResult result;

        if (!initialized_) {
            server_->setAborted(result, "Controller is not initialized.");
            return;
        }

        if (manager_->requestStart(getName()) != 0) {
            server_->setAborted(result, "Cannot execute, unable to start controller.");
            ROS_ERROR_NAMED("ParallelGripperController",
                            "Cannot execute, unable to start controller.");
            return;
        }
        effort_ = goal->command.max_effort;
        goal_ = goal->command.position;


        // Track position/time for stall detection
        float last_position = left_->getPosition();
        ros::Time last_position_time = ros::Time::now();

        ros::Rate r(50);
        while (true) {
            // Abort detection
            if (server_->isPreemptRequested() || !ros::ok()) {
                ROS_DEBUG_NAMED("ParallelGripperController", "Command preempted.");
                server_->setPreempted();
                break;
            }
            ROS_INFO_STREAM("Gripper position " << left_->getPosition() << " goal position: " << goal_);
            // Publish feedback before possibly completing
            feedback.position = left_->getPosition();
            feedback.effort = left_->getEffort();
            feedback.reached_goal = false;
            feedback.stalled = false;
            server_->publishFeedback(feedback);

            // Goal detection
            if (fabs(feedback.position - goal->command.position) < 0.002) {
                result.position = feedback.position;
                result.effort = feedback.effort;
                result.reached_goal = true;
                result.stalled = false;
                ROS_DEBUG_NAMED("ParallelGripperController", "Command Succeeded.");
                server_->setSucceeded(result);
                return;
            }

            // Stall detection
            if (fabs(feedback.position - last_position) > 0.005) {
                last_position = feedback.position;
                last_position_time = ros::Time::now();
            } else {
                if (ros::Time::now() - last_position_time > ros::Duration(2.0)) {
                    result.position = feedback.position;
                    result.effort = feedback.effort;
                    result.reached_goal = false;
                    result.stalled = true;
                    ROS_DEBUG_NAMED("ParallelGripperController", "Gripper stalled, but succeeding.");
                    server_->setSucceeded(result);
                    return;
                }
            }

            r.sleep();
        }
    }

    std::vector<std::string> ParallelGripperController::getCommandedNames() {
        std::vector<std::string> names;
        names.push_back(left_->getName());
        names.push_back(right_->getName());
        return names;
    }

    std::vector<std::string> ParallelGripperController::getClaimedNames() {
        // Claimed == commanded.
        return getCommandedNames();
    }

}  // namespace robot_controllers
