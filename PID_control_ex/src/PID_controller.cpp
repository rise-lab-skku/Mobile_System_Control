#include <PID_control_ex/PID_controller.h>

namespace mobile_system_control
{
    struct PIDController::PointXY
    {
        float x;
        float y;
    };

    struct PIDController::Impl
    {
        // Subscriber
        ros::Subscriber sub_carla;

        // Publisher
        ros::Publisher pub_cmd;

        // csv data path
        std::string data_path;

        // track vector
        std::vector<PointXY> track;

        // Input state
        float x;
        float y;
        float theta;
        float vel;
        float steer;

        // PI Gain
        float Kp;
        float Ki;

        double err_sum;
        float time_step;
        float throttle;
    };

    PIDController::PIDController(ros::NodeHandle &nh) : impl_(new Impl)
    {
        // Init subscriber
        impl_->sub_carla = nh.subscribe("/mobile_system_control/ego_vehicle", 1, &PIDController::CarlaInputCallback, this);

        // Init publisher
        impl_->pub_cmd = nh.advertise<geometry_msgs::Vector3Stamped>("/mobile_system_control/control_msg", 3);

        // Get parameters
        nh.getParam("path", impl_->data_path);
        nh.getParam("Kp", impl_->Kp);
        nh.getParam("Ki", impl_->Ki);
        nh.getParam("accel", impl_->throttle);

        impl_->err_sum = 0;
        impl_->time_step = 0.025;

        // Read track data
        ReadPath();
    }

    PIDController::~PIDController() {}

    void PIDController::ReadPath()
    {
        std::ifstream istr;
        double x, y;
        char space;
        PointXY temp;

        istr.open(impl_->data_path, std::ios::in);

        if (!istr.is_open())
        {
            ROS_FATAL("Cannot open file: %s", impl_->data_path.c_str());
        }

        while (istr >> x && istr >> space && istr >> y)
        {
            temp.x = x;
            temp.y = y;
            impl_->track.push_back(temp);
        }

        istr.close();
    }

    void PIDController::CarlaInputCallback(const std_msgs::Float32MultiArray::Ptr &input)
    {
        impl_->x = input->data[0];
        impl_->y = input->data[1];
        impl_->theta = input->data[2];
        impl_->vel = input->data[3];
        impl_->steer = input->data[4];
        RunPID();
    }

    void PIDController::RunPID()
    {
        const int &idx_1 = (FindClosestIndex() + 1) % impl_->track.size();
        const int &idx_2 = (idx_1 + 1) % impl_->track.size();

        double ref_theta = atan2(impl_->track[idx_2].y - impl_->track[idx_1].y,
                                 impl_->track[idx_2].x - impl_->track[idx_1].x);

        double err = ref_theta - impl_->theta;

        err = (err < -M_PI) ? err + 2 * M_PI : err;
        err = (err > M_PI) ? err - 2 * M_PI : err;

        // P control
        const double &p_control = impl_->Kp * err;

        // I control
        impl_->err_sum += err * impl_->time_step;
        impl_->err_sum *= impl_->Ki * err;

        double control_cmd = p_control + impl_->err_sum;
        control_cmd = std::min(1.0, std::max(-1.0, control_cmd));

        geometry_msgs::Vector3Stamped cmd;
        cmd.header.frame_id = "PID_example";
        cmd.vector.x = impl_->throttle;
        cmd.vector.y = -control_cmd;
        cmd.vector.z = 0.0;
        impl_->pub_cmd.publish(cmd);
    }

    int PIDController::FindClosestIndex()
    {
        int closest_idx = 0;
        int cur_idx = 0;
        float dis = 10.0;
        for (auto &point : impl_->track)
        {
            float temp_dis = sqrt(pow(impl_->x - point.x, 2) + pow(impl_->y - point.y, 2));
            if (temp_dis < dis)
            {
                dis = temp_dis;
                closest_idx = cur_idx;
            }
            cur_idx++;
        }

        return closest_idx;
    }
}