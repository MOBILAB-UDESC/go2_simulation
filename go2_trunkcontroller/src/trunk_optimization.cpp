#include "controllers/trunk_controller/trunk_optimization.hpp"

#include "controllers/trunk_controller/eiquadprog.hpp"
#include "dls2/math/algebra.hpp"
#include "dls2/math/rotations.hpp"

namespace dls{
    
    TrunkOptimization::TrunkOptimization(const std::shared_ptr<robotlib::RobotBase>& pRobot_)
        : pRobot(pRobot_)
        , W_torques(pRobot_->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero()))
        , W_legs(pRobot_->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Ones()))
        , feet_forces(pRobot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero()))
        , contactConstrCount{3}
    {
        //initialize matrix (without slacks)
        GQ.setZero(12+24, 12+24);
        W.setZero(12, 12);
        CI.setZero(24, 12+24);
        g0.setZero(12+24);
        ci0.setZero(24+24);

        x.setZero(12+24);
        A.setZero(6, 12);

        //set minimization method for contact forces
        min_goal = NORMALS;

        //set weitghing vector for wrench
        /// 6D vectors with 'A' that stands for angular (AX=0, AY=1, AZ=2) and 'L' for linear (LX=3, LY=4, LZ=5)
        W_wrench(0) = 10;
        W_wrench(1) = 10;
        W_wrench(2) = 10;
        W_wrench(3) = 5;
        W_wrench(4) = 5;
        W_wrench(5) = 10;

        //set weigthing matrix for forces
        W_forces << 1 ,1 ,0.01;
        W_torques = Eigen::Vector3d(5, 1 , 0.2);
        desired_forces_weight = 0.0;
        //set default values for flags
        frictionConstrFlag = true;
        baseControlFlag = false;
        use_slacks = true;
        use_multiple_slacks = false;
        number_of_slacks = 0;

        //draw_.reset(new RvizPolygonsTools("world", "/trunk_cont", root_node));
    }

    TrunkOptimization::~TrunkOptimization()
    { };

    void TrunkOptimization::computeOptimization(const robotlib::LimbDataMap<Eigen::Vector3d>& surf_normal,//TODO use ROBOTstate to reduce state dimension
                                            const robotlib::LimbDataMap<double>& muEstimate,
                                            const robotlib::LimbDataMap<bool>& stance_legs,
                                            const robotlib::LimbDataMap<double>& force_max,
                                            const robotlib::LimbDataMap<double>& force_min,
                                            const Eigen::Matrix3d& R,
                                            const robotlib::LimbDataMap<Eigen::Vector3d>& foot_pos,
                                            const robotlib::LimbDataMap<Eigen::Matrix3d>& foot_jacobian,
                                            const Eigen::Vector3d& com,
                                            const robotlib::JointState& h_joints,
                                            const Eigen::Vector6d& desWrench,
                                            robotlib::JointState& joint_torques)
    {
        // auto joint_torques = this->pRobot->makeJointState();

        //find number of stance legs
        cleg_count = pRobot->computeNumStanceLegs(stance_legs);
        contact_forces= contactConstrCount*cleg_count;
        //ineq constraints
        friction_constr = num_cc*cleg_count;
        num_ineq = frictionConstrFlag*friction_constr;

        if (use_multiple_slacks){
            number_of_slacks = num_ineq;
        } else  {
            number_of_slacks = 1;}

        solution.resize(contact_forces);

        if (use_slacks){
            if (use_multiple_slacks){
                number_of_slacks = num_ineq;
            } else  {
                number_of_slacks = 1;
            }
            slacks.resize(number_of_slacks);
        }

        setCostFunction(R, com, surf_normal, foot_pos, stance_legs, foot_jacobian, desWrench);
        setInequalities(muEstimate, force_max, force_min, stance_legs, surf_normal);

        //no equality constraints
        CE.resize(0,0);ce0.resize(0);

        x.setZero(); //x incorporates grfs and slacks x = [f' s' ]' has size contact_forces + n_ineq
        this->feet_forces = Eigen::Vector3d::Zero();
        auto result = Eigen::solve_quadprog(GQ, g0, CE.transpose(), ce0, CI.transpose(), ci0, x);

        //check if a solution was found
        if(result == std::numeric_limits<double>::infinity())
        {
            std::cout << "couldn't find a feasible solution" << std::endl;
        }
        else
        {
            //map feet forces into a joint state vector because the number of contact forces is variable a for loop is needed
            Eigen::Vector3d tau_leg{Eigen::Vector3d::Zero()};

            int cleg_counter{0};
            int count{0};

            for(auto &leg: this->pRobot->getLegs())
            {
                tau_leg.setZero();

                if (stance_legs[leg])
                {
                    this->feet_forces[leg] = x.segment(cleg_counter*contactConstrCount, 3);

                    //compute joint torques
                    tau_leg = -(foot_jacobian[leg]).transpose()*R*this->feet_forces[leg];

                    count = 0;
                    //TODO2 improve
                    for(auto &joint : leg->getJoints())
                    {
                    tau_leg[count] += h_joints[joint];
                    joint_torques[joint] = tau_leg[count];

                    count++;
                    }
                    cleg_counter++;
                }
            }
        }

        solution = x.segment(0, contact_forces);

        if (use_slacks){
            slacks =  x.segment(contact_forces, number_of_slacks);
        }

        // return joint_torques;
    }

    void TrunkOptimization::setCostFunction(
        const Eigen::Matrix3d& R,
        const Eigen::Vector3d& com,
        const robotlib::LimbDataMap<Eigen::Vector3d>& surf_normal,
        const robotlib::LimbDataMap<Eigen::Vector3d>& foot_pos, 
        const robotlib::LimbDataMap<bool>& stance_legs,
        const robotlib::LimbDataMap<Eigen::Matrix3d>& foot_jacobian,
        const Eigen::Vector6d& desWrench)
    {
        //initialize local matrix
        b.setZero();
        A.resize(6, contact_forces); 
        A.setZero();
        desired_forces.resize(contact_forces);

        //resize input matrix
        if (use_slacks) {
            GQ.resize(contact_forces + number_of_slacks, contact_forces + number_of_slacks); 
            GQ.setZero();
            g0.resize(contact_forces + number_of_slacks); 
            g0.setZero();
        } 
        else {
            GQ.resize(contact_forces, contact_forces); 
            GQ.setZero();
            g0.resize(contact_forces); 
            g0.setZero();
        }

        W.resize(contact_forces, contact_forces);
        W.setIdentity();
        W *= 1e-4;

        /*
        for(auto leg: *this->pRobot->getLegs())
        {
            for(int dof = 0; dof <= 2; dof++)
                W[leg](dof) = W_forces[leg](dof);
        }
        */

        //W.setIdentity();
        //W *= 1e-4;
        //W(5) = 1e-3;
        //W(8) = 1e-3;
        
        //A matrix maps feet forces into CoM wrench
        int cleg_counter{0};

        for(auto &leg: this->pRobot->getLegs())
        {
            if (stance_legs[leg]){
                //feet forces are already in world frame so they should not be rotated to be mapped into wrenches
                A.block(3, cleg_counter*contactConstrCount, 3, 3) = Eigen::Matrix3d::Identity();
                //foot pos should be mapped in world coords
                A.block(0, cleg_counter*contactConstrCount, 3, 3)= dls::math::skew_sim(R.transpose()*(foot_pos[leg] - com));

                if (min_goal == NORMALS) {
                    Eigen::Matrix3d BaseChange{Eigen::Matrix3d::Zero()};
                    Eigen::Vector3d tangentDir1{Eigen::Vector3d::Zero()}, tangentDir2{Eigen::Vector3d::Zero()};
                    //compute tangent components
                    tangentDir1 = Eigen::Vector3d::UnitX().cross(surf_normal[leg]); 
                    tangentDir1.normalize(); //in y direction)
                    tangentDir2 = surf_normal[leg].cross(tangentDir1); 
                    tangentDir2.normalize();//in x direction)
                    //compute tanget components (old)
                    //				tangentDir1 = surf_normal[leg].cross(Vector3d::UnitY()); tangentDir1.normalize();
                    //				tangentDir2 = surf_normal[leg].cross(tangentDir1); tangentDir2.normalize();
                    //compute rotation matrix
                    BaseChange<<tangentDir1, tangentDir2,surf_normal[leg];
                    W.block(cleg_counter*contactConstrCount,cleg_counter*contactConstrCount, 3, 3) = W_legs[leg].asDiagonal()*BaseChange*0.01*W_forces.asDiagonal()*BaseChange.transpose();
                }
                if (min_goal == TORQUES){
                    W.block(cleg_counter*contactConstrCount,cleg_counter*contactConstrCount, 3, 3) = W_legs[leg].asDiagonal()*R.transpose()*(foot_jacobian[leg])*0.01*W_torques[leg].asDiagonal()*(foot_jacobian[leg]).transpose()*R;
                }
                cleg_counter++;
            }
        }

        b = desWrench;
        //set wrench weighting matrix
        S.setIdentity();
        S.diagonal() = W_wrench;

        //Finds x that minimizes f = (Ax-b)' S (Ax-b) + x' W x
        //f = (Ax-b)' S (Ax-b) + x' W x = x'A'SAx - 2x'A'Sb + b'Sb + x'Wx.
        if (desired_forces_weight != 0.0)
        {
            GQ.block(0,0, contact_forces, contact_forces) = A.transpose() * S * A + desired_forces_weight*Eigen::MatrixXd::Identity(contact_forces, contact_forces);
            g0.segment(0, contact_forces) = - b.transpose()* S * A - desired_forces.transpose()*desired_forces_weight;
        } else {
            GQ.block(0,0, contact_forces, contact_forces) = A.transpose() * S * A + W;
            g0.segment(0, contact_forces) = - b.transpose()* S * A;
        }
        //add slacks
        if (use_slacks)
        {
            double w_slack{1e8};
            if (use_multiple_slacks) {
                Eigen::MatrixXd I_slack(number_of_slacks,number_of_slacks); I_slack.setIdentity();
            TrunkOptimization:  GQ.block(contact_forces, contact_forces, number_of_slacks, number_of_slacks) = sqrt(w_slack)*I_slack;
            } else  {
                GQ(contact_forces, contact_forces) = sqrt(w_slack);
            }
        }
    }

    void  TrunkOptimization::setInequalities(
        const robotlib::LimbDataMap<double>& muEstimate,
        const robotlib::LimbDataMap<double>& force_max,
        const robotlib::LimbDataMap<double>& force_min,
        const robotlib::LimbDataMap<bool>& stance_legs, 
        const robotlib::LimbDataMap<Eigen::Vector3d>& surf_normal)
    {
        double number_of_columns{0};
        //init matrix
        if (use_slacks) {
            CI.resize(num_ineq,  contact_forces + number_of_slacks); CI.setZero();
            ci0.resize(num_ineq); ci0.setZero();
        } else {
            CI.resize(num_ineq,  contact_forces); CI.setZero();
            ci0.resize(num_ineq); ci0.setZero();
        }

        if (frictionConstrFlag){
            //set friction cone limits
            Eigen::Vector3d tangentDir1{Eigen::Vector3d::Zero()}, tangentDir2{Eigen::Vector3d::Zero()};
            int cleg_counter{0}; //contsrained leg count

            for(auto &leg: this->pRobot->getLegs())
            {
                if (stance_legs[leg]){
                    //0 constraint -- fi.n >= 0  no pulling forces, only pushing...
                    CI.block(num_cc*cleg_counter+0,  contactConstrCount*cleg_counter,1,3) = surf_normal[leg].transpose();
                    ci0(num_cc*cleg_counter + 0) = -force_min[leg];

                    //1 constraint -- fi.n <= Fzmax => -fi.n >= -Fzmax limit normal force
                    CI.block(num_cc*cleg_counter+1,  contactConstrCount*cleg_counter,1,3) = -surf_normal[leg].transpose();
                    ci0(num_cc*cleg_counter + 1) =  force_max[leg];

                    //add 2-6 cone constraints
                    //for each tangent direction t,
                    //we want: -mu*n . fi <= fi . t <= mu*n . fi,
                    //(n.fi == normal component of force, t.fi = tangential component along vector t)
                    //which is equivalent to the two constraints: mu*n . fi >= - fi . t and mu*n . fi >= fi . t, or equivalently
                    //mu*n . fi + fi . t >=0 and mu*n . fi - fi . t >= 0

                    //compute tanget components
                    tangentDir1 = Eigen::Vector3d::UnitX().cross(surf_normal[leg]); tangentDir1.normalize(); //in y direction)
                    tangentDir2 = surf_normal[leg].cross(tangentDir1); tangentDir2.normalize();//in x direction)
                    //compute 4 constraints
                    Eigen::Vector3d cone_constr1{Eigen::Vector3d::Zero()}, cone_constr2{Eigen::Vector3d::Zero()}, cone_constr3{Eigen::Vector3d::Zero()}, cone_constr4{Eigen::Vector3d::Zero()};
                    cone_constr1 = surf_normal[leg]*muEstimate[leg] + tangentDir1;
                    cone_constr2 = surf_normal[leg]*muEstimate[leg] - tangentDir1;
                    cone_constr3 = surf_normal[leg]*muEstimate[leg] + tangentDir2;
                    cone_constr4 = surf_normal[leg]*muEstimate[leg] - tangentDir2;

                    CI.block(num_cc*cleg_counter + 2, contactConstrCount*cleg_counter,1,3) = cone_constr1.transpose();
                    CI.block(num_cc*cleg_counter + 3, contactConstrCount*cleg_counter,1,3) = cone_constr2.transpose();
                    CI.block(num_cc*cleg_counter + 4, contactConstrCount*cleg_counter,1,3) = cone_constr3.transpose();
                    CI.block(num_cc*cleg_counter + 5, contactConstrCount*cleg_counter,1,3) = cone_constr4.transpose();
                    ci0.segment(num_cc*cleg_counter + 2, 4).setZero();

                    cleg_counter++;
                }
            }
            //add slacks
            if (use_slacks)
            {
                if (use_multiple_slacks)
                { //add an Identity matrix
                    Eigen::MatrixXd I_slack(number_of_slacks, number_of_slacks); 
                    I_slack.setIdentity();
                    CI.block(0, contact_forces, number_of_slacks, number_of_slacks) = -I_slack;
                } else { //add a column
                    CI.block(0, contact_forces, num_ineq, 1) = - Eigen::MatrixXd::Ones(num_ineq,1);
                }
            }
        }
    }

    void TrunkOptimization::getFeetForces(robotlib::LimbDataMap<Eigen::Vector3d> &feet_forces)
    {
        feet_forces = this->feet_forces;
    }
}