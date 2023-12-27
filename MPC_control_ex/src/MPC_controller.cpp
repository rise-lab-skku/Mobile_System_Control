#include <MPC_control_ex/MPC_controller.h>

namespace mobile_system_control
{
    struct MPCControler::PointXY
    {
        float x;
        float y;
    };

    float Distance(const MPCControler::PointXY &p0, const MPCControler::PointXY &p1)
    {
        return sqrt(pow((p0.x - p1.x), 2) + pow((p0.y - p1.y), 2));
    }

    struct MPCControler::Impl
    {
        int ControlNum;
        int close_idx;

        // Subscriber
        ros::Subscriber sub_carla;

        // Publisher
        ros::Publisher pub_cmd;
        ros::Publisher pub_path;
        ros::Publisher pub_sol;

        tf::TransformBroadcaster br;
        tf::Transform map2car;

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

        // MPC wieghts
        float Q;
        float P;
        float R;
        Eigen::MatrixXf Q_bar;
        Eigen::MatrixXf R_bar;
        Eigen::MatrixXf T_bar;
        Eigen::MatrixXf S_bar;
        Eigen::SparseMatrix<float> H;
        Eigen::VectorXd F;
        Eigen::MatrixXf Y;

        Eigen::VectorXd upper_lim;
        Eigen::VectorXd lower_lim;
        Eigen::SparseMatrix<double> linearConstraintMatrix;

        Eigen::SparseMatrix<float> hessian;
        Eigen::VectorXd gradient;

        // references
        std::vector<double> x_ref, y_ref;
        std::vector<float> speed_ref;
        std::vector<float> steer_ref;
        std::vector<float> head_ref;
        std::vector<float> x_sol, y_sol;
        float v_ref = 20.0;

        std::vector<Eigen::Matrix3f> A_matrices;
        std::vector<Eigen::Matrix<float, 3, 2>> B_matrices;

        float L = 1.212;
        float dt;
        float x_threshold;
        float y_threshold;
        float head_threshold;

        double err_sum;
        float time_step;
        float throttle = 0.8;

        // gains
        float K_x, K_y, K_t;
        bool is_zero = false;

        // solution
        Eigen::VectorXd QPsol;
        float steer_sol;
        float speed_sol;
    };

    MPCControler::MPCControler(ros::NodeHandle &nh) : impl_(new Impl)
    {
        // Init subscriber
        impl_->sub_carla = nh.subscribe("/mobile_system_control/ego_vehicle", 1, &MPCControler::CarlaInputCallback, this);

        // Init publisher
        impl_->pub_cmd = nh.advertise<geometry_msgs::Vector3Stamped>("/mobile_system_control/control_msg", 3);
        impl_->pub_path = nh.advertise<visualization_msgs::MarkerArray>("reference_path", 3);
        impl_->pub_sol = nh.advertise<visualization_msgs::MarkerArray>("mpc_solution", 3);

        // Get parameters
        nh.getParam("path", impl_->data_path);

        nh.getParam("ControlNum", impl_->ControlNum);

        nh.getParam("Q", impl_->Q);
        nh.getParam("P", impl_->P);
        nh.getParam("R", impl_->R);
        nh.getParam("x_threshold", impl_->x_threshold);
        nh.getParam("y_threshold", impl_->y_threshold);
        nh.getParam("head_threshold", impl_->head_threshold);
        nh.getParam("time_interval", impl_->dt);

        nh.getParam("K_x", impl_->K_x);
        nh.getParam("K_y", impl_->K_y);
        nh.getParam("K_t", impl_->K_t);

        // Make wieght matrices
        impl_->Q_bar.resize(3 * impl_->ControlNum, 3 * impl_->ControlNum);
        impl_->R_bar.resize(2 * impl_->ControlNum, 2 * impl_->ControlNum);
        for (int idx = 0; idx < 3 * impl_->ControlNum; idx++)
        {
            impl_->Q_bar(idx, idx) = impl_->Q;
        }
        for (int idx = 0; idx < 2 * impl_->ControlNum; idx++)
        {
            impl_->R_bar(idx, idx) = impl_->R;
        }

        // Read track data
        ReadPath();
    }

    MPCControler::~MPCControler() {}

    void MPCControler::ReadPath()
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

    void MPCControler::CarlaInputCallback(const std_msgs::Float32MultiArray::Ptr &input)
    {
        impl_->x = input->data[0];
        impl_->y = input->data[1];
        impl_->theta = input->data[2];
        impl_->vel = input->data[3];
        impl_->steer = input->data[4];

        if (impl_->theta < -M_PI / 2 || impl_->theta > M_PI / 2)
        {
            impl_->is_zero = false;
        }
        else
        {
            impl_->is_zero = true;
        }

        if (!impl_->is_zero)
        {
            impl_->theta = (impl_->theta < 0) ? impl_->theta + 2 * M_PI : impl_->theta;
        }

        RunMPC();
    }

    void MPCControler::RunMPC()
    {
        const float &dt = impl_->dt;

        SetPath();
        SetReferences();
        BuildMatrices();
        SetHessianMatrix();
        SetGradient();
        SetConstraintMatrix();
        SetConstrainVectors();

        // Set solver
        OsqpEigen::Solver solver;
        solver.settings()->setWarmStart(true);
        solver.data()->setNumberOfVariables((impl_->ControlNum + 1) * 3 + impl_->ControlNum * 2);
        solver.data()->setNumberOfConstraints((impl_->ControlNum + 1) * 3 * 2 + impl_->ControlNum * 2);

        if (!solver.data()->setHessianMatrix(impl_->hessian))
            return;
        if (!solver.data()->setGradient(impl_->gradient))
            return;
        if (!solver.data()->setLinearConstraintsMatrix(impl_->linearConstraintMatrix))
            return;
        if (!solver.data()->setLowerBound(impl_->lower_lim))
            return;
        if (!solver.data()->setUpperBound(impl_->upper_lim))
            return;
        if (!solver.initSolver())
            return;
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return;

        Eigen::Vector2d ctr;
        Eigen::VectorXd QPSolution;

        int delay = 0;
        int filter = 3;
        float steer = 0;
        QPSolution = solver.getSolution();

        for (int idx = delay; idx < delay + filter; idx++)
        {
            steer += (QPSolution(3 * (impl_->ControlNum + 1) + 1 + 2 * idx) + impl_->steer_ref[idx]);
        }

        steer /= filter;

        impl_->steer_sol = steer;
        impl_->speed_sol = (QPSolution(3 * (impl_->ControlNum + 1) + 2 * delay) + impl_->speed_ref[delay]);
        impl_->QPsol = QPSolution;

        impl_->speed_sol = std::min(1.0, std::max(0.0, impl_->speed_sol / 5.56));
        impl_->steer_sol = std::min(1.0, std::max(-1.0, impl_->steer_sol / (20.0 * M_PI / 180)));

        geometry_msgs::Vector3Stamped cmd;
        cmd.header.frame_id = "MPC_example";
        cmd.vector.x = impl_->speed_sol;
        cmd.vector.y = -impl_->steer_sol;
        cmd.vector.z = 0.0;
        impl_->pub_cmd.publish(cmd);

        for (int i = 0; i < impl_->ControlNum; i++)
        {
            float steer = (QPSolution(3 * (impl_->ControlNum + 1) + 2 * i + 1) + impl_->steer_ref[i]) * 180 / M_PI;
            float speed = QPSolution(3 * (impl_->ControlNum + 1) + 2 * i) + impl_->speed_ref[i];
        }

        std::vector<double> x_sol;
        std::vector<double> y_sol;

        for (int i = 0; i < (impl_->ControlNum); i++)
        {
            x_sol.push_back(QPSolution(i * 3) + impl_->x_ref[i]);
            y_sol.push_back(QPSolution(i * 3 + 1) + impl_->y_ref[i]);
        }

        visualization_msgs::MarkerArray track_array;

        for (int i = 0; i < impl_->track.size(); i++)
        {
            visualization_msgs::Marker marker_sol;
            marker_sol.header.frame_id = "map";
            marker_sol.header.stamp = ros::Time();
            marker_sol.id = i;
            marker_sol.type = visualization_msgs::Marker::SPHERE;
            marker_sol.action = visualization_msgs::Marker::ADD;
            marker_sol.pose.position.x = impl_->track[i].x;
            marker_sol.pose.position.y = impl_->track[i].y;
            marker_sol.pose.position.z = 0;
            marker_sol.pose.orientation.x = 0.0;
            marker_sol.pose.orientation.y = 0.0;
            marker_sol.pose.orientation.z = 0.0;
            marker_sol.pose.orientation.w = 1.0;
            marker_sol.scale.x = 0.5;
            marker_sol.scale.y = 0.5;
            marker_sol.scale.z = 0.5;
            marker_sol.color.a = 1.0;
            marker_sol.color.r = 0.0;
            marker_sol.color.g = 1.0;
            marker_sol.color.b = 0.0;
            track_array.markers.push_back(marker_sol);
        }
        impl_->pub_path.publish(track_array);

        visualization_msgs::MarkerArray marker_array_sol;
        for (int i = 0; i < x_sol.size(); i++)
        {
            visualization_msgs::Marker marker_sol;
            marker_sol.header.frame_id = "map";
            marker_sol.header.stamp = ros::Time();
            marker_sol.id = i;
            marker_sol.type = visualization_msgs::Marker::SPHERE;
            marker_sol.action = visualization_msgs::Marker::ADD;
            marker_sol.pose.position.x = x_sol[i];
            marker_sol.pose.position.y = y_sol[i];
            marker_sol.pose.position.z = 0;
            marker_sol.pose.orientation.x = 0.0;
            marker_sol.pose.orientation.y = 0.0;
            marker_sol.pose.orientation.z = 0.0;
            marker_sol.pose.orientation.w = 1.0;
            marker_sol.scale.x = 0.5;
            marker_sol.scale.y = 0.5;
            marker_sol.scale.z = 0.5;
            marker_sol.color.a = 1.0;
            marker_sol.color.r = 0.0;
            marker_sol.color.g = 1.0;
            marker_sol.color.b = 1.0;
            marker_array_sol.markers.push_back(marker_sol);
        }
        impl_->pub_sol.publish(marker_array_sol);

        impl_->map2car.setOrigin(tf::Vector3(impl_->x, impl_->y, 0));
        tf::Quaternion q;
        q.setRPY(0, 0, impl_->theta);
        impl_->map2car.setRotation(q);
        impl_->br.sendTransform(tf::StampedTransform(impl_->map2car, ros::Time::now(), "map", "car"));

        impl_->x_ref.clear();
        impl_->y_ref.clear();
        impl_->steer_ref.clear();
        impl_->head_ref.clear();
        impl_->speed_ref.clear();

        return;
    }

    void MPCControler::SetPath()
    {
        int closest_point = (FindClosestIndex() + 1) % impl_->track.size();
        impl_->close_idx = closest_point;
        for (int idx = 0; idx < impl_->ControlNum; idx++)
        {
            impl_->x_ref.push_back(impl_->track[(closest_point + idx) % impl_->track.size()].x);
            impl_->y_ref.push_back(impl_->track[(closest_point + idx) % impl_->track.size()].y);
        }
    }

    void MPCControler::SetReferences()
    {
        const float &dt = impl_->dt;

        impl_->steer_ref.clear();
        impl_->head_ref.clear();
        impl_->speed_ref.clear();

        for (int idx = 0; idx < impl_->ControlNum; idx++)
        {
            float delta = 0.0;
            float speed = 0.0;
            float theta = 0.0;

            if (idx + 2 < impl_->ControlNum)
            {
                theta = atan2(impl_->y_ref[idx + 1] - impl_->y_ref[idx],
                              impl_->x_ref[idx + 1] - impl_->x_ref[idx]);
            }
            else
            {
                theta = atan2(impl_->y_ref[impl_->ControlNum - 2] - impl_->y_ref[impl_->ControlNum - 3],
                              impl_->x_ref[impl_->ControlNum - 2] - impl_->x_ref[impl_->ControlNum - 3]);
            }

            if (!impl_->is_zero)
            {
                theta = (theta < 0) ? theta + 2 * M_PI : theta;
            }
            impl_->head_ref.push_back(theta);
        }

        KanayamaSteer();
    }

    void MPCControler::BuildMatrices()
    {
        const float &dt = impl_->dt;

        // make A and B matrices
        impl_->A_matrices.clear();
        impl_->B_matrices.clear();
        Eigen::Matrix3f A;
        Eigen::MatrixXf B(3, 2);

        for (int idx = 0; idx < impl_->ControlNum; idx++)
        {
            // A matrices
            A << 1, 0, -impl_->vel * sin(impl_->head_ref[idx]) * dt,
                0, 1, impl_->vel * cos(impl_->head_ref[idx]) * dt,
                0, 0, 1;
            impl_->A_matrices.push_back(A);

            // B matrices
            B << cos(impl_->head_ref[idx]) * dt, 0,
                sin(impl_->head_ref[idx]) * dt, 0,
                (tan(impl_->steer_ref[idx]) / impl_->L) * dt,
                impl_->vel * (1 + pow(tan(impl_->steer_ref[idx]), 2) / impl_->L) * dt;
            impl_->B_matrices.push_back(B);
        }
    }

    void MPCControler::SetHessianMatrix()
    {
        impl_->hessian.resize((impl_->ControlNum + 1) * 3 + impl_->ControlNum * 2,
                              (impl_->ControlNum + 1) * 3 + impl_->ControlNum * 2);

        for (int idx = 0; idx < (impl_->ControlNum + 1) * 3 + impl_->ControlNum * 2; idx++)
        {
            if (idx < (impl_->ControlNum + 1) * 3)
            {
                impl_->hessian.insert(idx, idx) = impl_->Q;
            }
            else
            {
                impl_->hessian.insert(idx, idx) = impl_->R;
            }
        }
    }

    void MPCControler::SetGradient()
    {
        impl_->gradient.resize((impl_->ControlNum + 1) * 3 + impl_->ControlNum * 2);
        for (int idx = 0; idx < (impl_->ControlNum + 1) * 3 + impl_->ControlNum * 2; idx++)
        {
            impl_->gradient(idx) = 0;
        }
    }

    void MPCControler::KanayamaSteer()
    {
        int size = impl_->track.size();
        for (int idx = 0; idx < impl_->ControlNum; idx++)
        {
            PointXY n1, n1_, n2, n3;
            n1.x = impl_->x_ref[idx];
            n1.y = impl_->y_ref[idx];
            n1_.x = impl_->track[(impl_->close_idx + 1 + idx) % size].x;
            n1_.y = impl_->track[(impl_->close_idx + 1 + idx) % size].y;

            n2.x = impl_->track[(impl_->close_idx + 5 + idx) % size].x;
            n2.y = impl_->track[(impl_->close_idx + 5 + idx) % size].y;
            n3.x = impl_->track[(impl_->close_idx + 6 + idx) % size].x;
            n3.y = impl_->track[(impl_->close_idx + 6 + idx) % size].y;

            float theta = atan2(n1_.y - n1.y, n1_.x - n1.x);
            PointXY err_state;
            err_state.x = std::cos(theta) * (n2.x - n1.x) + std::sin(theta) * (n2.y - n1.y);
            err_state.y = -std::sin(theta) * (n2.x - n1.x) + std::cos(theta) * (n2.y - n1.y);

            float ref_theta = atan2(n3.y - n2.y, n3.x - n2.x);
            float err_theta = ref_theta - theta;

            if (err_theta > M_PI / 2)
            {
                err_theta - 2 * M_PI;
            }
            else if (err_theta < -M_PI / 2)
            {
                err_theta + 2 * M_PI;
            }
            // calculate control inputs
            double speed = impl_->v_ref * std::cos(err_theta) * 5 / 18 + impl_->K_x * err_state.x;
            speed = std::min(5.56, std::max(0.0, speed));
            const double &w_ctrl = speed * (impl_->K_y * err_state.y + impl_->K_t * std::sin(err_theta));
            double delta = atan((w_ctrl * impl_->L) / speed);
            delta = std::min(20.0 * M_PI / 180, std::max(-20.0 * M_PI / 180, delta));

            impl_->speed_ref.push_back(speed);
            impl_->steer_ref.push_back(delta);
        }
    }

    void MPCControler::SetConstraintMatrix()
    {
        impl_->linearConstraintMatrix.resize((impl_->ControlNum + 1) * 3 * 2 + impl_->ControlNum * 2,
                                             (impl_->ControlNum + 1) * 3 + impl_->ControlNum * 2);

        // populate linear constraint matrix
        for (int i = 0; i < 3 * (impl_->ControlNum + 1); i++)
        {
            impl_->linearConstraintMatrix.insert(i, i) = -1;
        }

        for (int i = 0; i < impl_->ControlNum; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                {
                    float value = impl_->A_matrices[i](j, k);
                    if (value != 0)
                    {
                        impl_->linearConstraintMatrix.insert(3 * (i + 1) + j, 3 * i + k) = value;
                    }
                }

        for (int i = 0; i < impl_->ControlNum; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 2; k++)
                {
                    float value = impl_->B_matrices[i](j, k);
                    if (value != 0)
                    {
                        impl_->linearConstraintMatrix.insert(3 * (i + 1) + j, 2 * i + k + 3 * (impl_->ControlNum + 1)) = value;
                    }
                }

        for (int i = 0; i < (impl_->ControlNum + 1) * 3 + impl_->ControlNum * 2; i++)
        {
            impl_->linearConstraintMatrix.insert(i + (impl_->ControlNum + 1) * 3, i) = 1;
        }
    }

    void MPCControler::SetConstrainVectors()
    {
        impl_->lower_lim.resize((impl_->ControlNum + 1) * 3 * 2 + impl_->ControlNum * 2);
        impl_->upper_lim.resize((impl_->ControlNum + 1) * 3 * 2 + impl_->ControlNum * 2);

        Eigen::VectorXd lower_eq;
        Eigen::VectorXd upper_eq;
        Eigen::VectorXd x_min, x_max;
        Eigen::VectorXd u_min, u_max;

        lower_eq.resize(3 * (impl_->ControlNum + 1));
        upper_eq.resize(3 * (impl_->ControlNum + 1));
        x_min.resize(3 * (impl_->ControlNum + 1));
        x_max.resize(3 * (impl_->ControlNum + 1));
        u_min.resize(impl_->ControlNum * 2);
        u_max.resize(impl_->ControlNum * 2);

        for (int idx = 0; idx < (impl_->ControlNum + 1) * 3; idx += 3)
        {
            if (idx == 0)
            {
                lower_eq(idx) = -impl_->x + impl_->x_ref[idx];
                lower_eq(idx + 1) = -impl_->y + impl_->y_ref[idx];
                lower_eq(idx + 2) = -impl_->theta + impl_->head_ref[idx];
                upper_eq(idx) = -impl_->x + impl_->x_ref[idx];
                upper_eq(idx + 1) = -impl_->y + impl_->y_ref[idx];
                upper_eq(idx + 2) = -impl_->theta + impl_->head_ref[idx];
            }
            else
            {
                lower_eq(idx) = 0;
                lower_eq(idx + 1) = 0;
                lower_eq(idx + 2) = 0;
                upper_eq(idx) = 0;
                upper_eq(idx + 1) = 0;
                upper_eq(idx + 2) = 0;
            }
            x_min(idx) = -impl_->x_threshold;
            x_min(idx + 1) = -impl_->y_threshold;
            x_min(idx + 2) = -impl_->head_threshold * M_PI / 180;
            x_max(idx) = impl_->x_threshold;
            x_max(idx + 1) = impl_->y_threshold;
            x_max(idx + 2) = impl_->head_threshold * M_PI / 180;
        }

        for (int idx = 0; idx < 2 * impl_->ControlNum; idx += 2)
        {
            u_min(idx) = (10.0 - 20.0) * 5.0 / 18.0;
            u_min(idx + 1) = -20 * M_PI / 180 - impl_->steer_ref[idx / 2];
            u_max(idx) = (20.0 - 20.0) * 5.0 / 18.0;
            u_max(idx + 1) = 20 * M_PI / 180 - impl_->steer_ref[idx / 2];
        }
        impl_->lower_lim << lower_eq, x_min, u_min;
        impl_->upper_lim << upper_eq, x_max, u_max;
    }

    int MPCControler::FindClosestIndex()
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