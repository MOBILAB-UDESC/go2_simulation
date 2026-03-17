#ifndef TRUNK_OPTIMIZATION_HPP
#define TRUNK_OPTIMIZATION_HPP

#include <robotlib/robot_base.hpp>

namespace Eigen
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

namespace dls
{
    class TrunkOptimization
    {
    public:
        TrunkOptimization(const std::shared_ptr<robotlib::RobotBase>&);
        ~TrunkOptimization();

        /// @brief 
        /// @param surf_normal_in 
        /// @param muEstimate_in 
        /// @param stance_legs_in 
        /// @param force_max_in 
        /// @param force_min_in 
        /// @param R_in 
        /// @param footPos_in 
        /// @param foot_jacobian
        /// @param com_in 
        /// @param h_joints 
        /// @param desWrench_in 
        void computeOptimization(const robotlib::LimbDataMap<Eigen::Vector3d>& surf_normal_in,//TODO use ROBOTstate to reduce state dimension
                                const robotlib::LimbDataMap<double>& muEstimate_in,
                                const robotlib::LimbDataMap<bool>& stance_legs_in,
                                const robotlib::LimbDataMap<double>& force_max_in,
                                const robotlib::LimbDataMap<double>& force_min_in,
                                const Eigen::Matrix3d& R_in,
                                const robotlib::LimbDataMap<Eigen::Vector3d>& footPos_in,
                                const robotlib::LimbDataMap<Eigen::Matrix3d>& foot_jacobian,
                                const Eigen::Vector3d& com_in,
                                const robotlib::JointState& h_joints,
                                const Eigen::Vector6d& desWrench_in, robotlib::JointState& tau_joints);
        /// @brief 
        /// @param foot_pos 
        /// @param stance_legs 
        /// @param foot_jacobian
        void setCostFunction(
            const Eigen::Matrix3d& R,
            const Eigen::Vector3d& com,
            const robotlib::LimbDataMap<Eigen::Vector3d>& surf_normal,
            const robotlib::LimbDataMap<Eigen::Vector3d>& foot_pos, 
            const robotlib::LimbDataMap<bool>& stance_legs,
            const robotlib::LimbDataMap<Eigen::Matrix3d>& foot_jacobian,
            const Eigen::Vector6d& desWrench
        );

        /// @brief 
        /// @param CI 
        /// @param ci0 
        /// @param stance_legs 
        void setInequalities(
            const robotlib::LimbDataMap<double>& muEstimate,
            const robotlib::LimbDataMap<double>& force_max,
            const robotlib::LimbDataMap<double>& force_min,
            const robotlib::LimbDataMap<bool>& stance_legs, 
            const robotlib::LimbDataMap<Eigen::Vector3d>& surf_normal
        );

        void getFeetForces(robotlib::LimbDataMap<Eigen::Vector3d> &feet_forces);

    private:

        const std::shared_ptr<robotlib::RobotBase> pRobot;

        int number_of_slacks;
        bool use_slacks;
        bool use_multiple_slacks;

        //optimization variables (we just optimize for leg variables)
        robotlib::LimbDataMap<Eigen::Vector3d> W_torques;
        robotlib::LimbDataMap<Eigen::Vector3d> W_legs;
        robotlib::LimbDataMap<Eigen::Vector3d> feet_forces;

        //user defined variables
        enum MinMethod{WRENCHDIRECTION = 0, NORMALS, TORQUES};

        MinMethod min_goal;
        Eigen::Vector3d W_forces;
        Eigen::Vector6d W_wrench;
        double desired_forces_weight;
        bool frictionConstrFlag;
        bool baseControlFlag;

        //internal variables
        int cleg_count;
        int contact_forces;	//number of stance legs
        int friction_constr;
        int num_ineq;

        static const int num_cc{6}; //number of friction constraints per leg

        //internal variables for Whole body Optimization
        Eigen::MatrixXd GQ;
        Eigen::MatrixXd W;
        Eigen::MatrixXd CI;
        Eigen::MatrixXd CE;
        Eigen::MatrixXd A; //A is used for computing wrencherror
        Eigen::VectorXd g0;
        Eigen::VectorXd ce0;
        Eigen::VectorXd ci0;
        Eigen::VectorXd x;
        Eigen::VectorXd slacks;
        Eigen::VectorXd solution;
        Eigen::VectorXd desired_forces;

        Eigen::Matrix<double, 6, 6> S;
        Eigen::Vector6d b;

        //for debug
        double quadCost;
        double taskCost;
        double slackCost;
        int contactConstrCount;
    };
}//@namespace dls

#endif
