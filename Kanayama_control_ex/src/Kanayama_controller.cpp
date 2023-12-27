#include <Kanayama_control_ex/Kanayama_controller.h>

namespace mobile_system_control
{
    struct KanayamaController::PointXY
    {
        float x;
        float y;
    };

    float Distance(const KanayamaController::PointXY &p0, const KanayamaController::PointXY &p1)
    {
        return sqrt(pow((p0.x - p1.x), 2) + pow((p0.y - p1.y), 2));
    }

    struct KanayamaController::Impl
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

        // Gains
        float K_x;
        float K_y;
        float K_t;

        // references
        float x_ref;
        float y_ref;
        float theta_ref;
        float v_ref = 20.0;

        float L = 1.212;

        double err_sum;
        float time_step;
        float throttle = 0.8;
    };

    KanayamaController::KanayamaController(ros::NodeHandle &nh) : impl_(new Impl)
    {
        // Init subscriber
        impl_->sub_carla = nh.subscribe("/mobile_system_control/ego_vehicle", 1, &KanayamaController::CarlaInputCallback, this);

        // Init publisher
        impl_->pub_cmd = nh.advertise<geometry_msgs::Vector3Stamped>("/mobile_system_control/control_msg", 3);

        // Get parameters
        nh.getParam("path", impl_->data_path);
        ROS_ASSERT(nh.getParam("K_x", impl_->K_x));
        ROS_ASSERT(nh.getParam("K_y", impl_->K_y));
        ROS_ASSERT(nh.getParam("K_t", impl_->K_t));

        impl_->time_step = 0.025;

        // Read track data
        ReadPath();
    }

    KanayamaController::~KanayamaController() {}

    void KanayamaController::ReadPath()
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

    void KanayamaController::CarlaInputCallback(const std_msgs::Float32MultiArray::Ptr &input)
    {
        impl_->x = input->data[0];
        impl_->y = input->data[1];
        impl_->theta = input->data[2];
        impl_->vel = input->data[3];
        impl_->steer = input->data[4];
        RunKanayama();
    }

    void KanayamaController::RunKanayama()
    {
        PointXY ref = impl_->track[(FindClosestIndex() + 5) % impl_->track.size()];
        PointXY ref_next = impl_->track[(FindClosestIndex() + 6) % impl_->track.size()];

        PointXY err_state;
        err_state.x = std::cos(impl_->theta) * (ref.x - impl_->x) + std::sin(impl_->theta) * (ref.y - impl_->y);
        err_state.y = -std::sin(impl_->theta) * (ref.x - impl_->x) + std::cos(impl_->theta) * (ref.y - impl_->y);

        float ref_theta = atan2(ref_next.y - ref.y, ref_next.x - ref.x);
        float err_theta = ref_theta - impl_->theta;

        if (err_theta > M_PI / 2)
        {
            err_theta - 2 * M_PI;
        }
        else if (err_theta < -M_PI / 2)
        {
            err_theta + 2 * M_PI;
        }

        // calculate control inputs
        double speed = impl_->v_ref * std::cos(err_theta) * 5 / 18 + impl_->K_x * err_state.x; // change km/h to m/s
        const double &w_ctrl = speed * (impl_->K_y * err_state.y + impl_->K_t * std::sin(err_theta));
        double delta = (atan((w_ctrl * impl_->L) / speed) * 180 / M_PI) / 20.0;

        speed = std::min(1.0, std::max(0.0, speed / 5.56));
        delta = std::min(1.0, std::max(-1.0, delta));

        geometry_msgs::Vector3Stamped cmd;
        cmd.header.frame_id = "Kanayama_example";
        cmd.vector.x = speed;
        cmd.vector.y = -delta;
        cmd.vector.z = 0.0;
        impl_->pub_cmd.publish(cmd);
    }

    int KanayamaController::FindClosestIndex()
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