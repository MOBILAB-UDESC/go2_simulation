#include <Eigen/Dense>

class SineWaveTrajectory
{
public:
    SineWaveTrajectory(const double ampl_lin, const double ampl_ang, const double w, const double height, const double dt) : dt(dt/1000000.0)
    {
        for(int i=0; i<this->ampl.size();i++)
        {
            if(i<3) {this->ampl(i) *= ampl_lin;}
            else {this->ampl(i) *= ampl_ang*3.14/180.0;}//ampl_ang are in degree
            this->w(i) *= w;
        }
        this->des_pos_offset_axes(2) = height;
        this->des_pos_axes(2) = this->des_pos_offset_axes(2);
    }
    ~SineWaveTrajectory(){};

    Eigen::Matrix<int, 6, 1> enabled_axes = {0,0,0,0,0,0};
    Eigen::Matrix<double, 6, 1> des_angle_axes = {0.0,0.0,0.0,0.0,0.0,0.0};
    Eigen::Matrix<double, 6, 1> des_pos_axes = {0.0,0.0,0.0,0.0,0.0,0.0};
    Eigen::Matrix<double, 6, 1> des_vel_axes = {0.0,0.0,0.0,0.0,0.0,0.0};
    Eigen::Matrix<double, 6, 1> ampl = {1.0,1.0,1.0,1.0,1.0,1.0};
    Eigen::Matrix<double, 6, 1> w = {1.0,1.0,1.0,1.0,1.0,1.0};
    Eigen::Matrix<double, 6, 1> des_pos_offset_axes = {0.0,0.0,0.0,0.0,0.0,0.0};
    const double dt;

    void computeTrajectory()
    {
        for(int i=0; i<this->enabled_axes.size();i++)
        {
            if(this->enabled_axes(i)==1)
            {
                this->des_angle_axes(i) += this->w(i)*this->dt; 
                this->des_pos_axes(i) = des_pos_offset_axes(i) + this->ampl(i)*sin(this->des_angle_axes(i));
                this->des_vel_axes(i) = this->ampl(i)*this->w(i)*cos(this->des_angle_axes(i));
            }
        }
    }
};