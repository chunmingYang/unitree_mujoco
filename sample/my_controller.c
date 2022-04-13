double setpt[nact] = {0};

void init_controller()
{
    int i;
    double target[]={0,0,0,   //FR B
                     0,0,0,   //FL A
                     0,0,0,   //RR A
                     0,0,0};  //RL B
    
    for (i=0;i<nact;i++)
    {
        setpt[i]=target[i];
    }
}

void control_loop(const mjModel* m, mjData* d,
                  double q[nact], double u[nact], double base_x[3], double base_quat[4],
                  double base_angvel[3], double base_linacc[3], double base_linvel[3],
                  double pos_FR_shoulder[3], double pos_FL_shoulder[3], double pos_RR_shoulder[3], double pos_RL_shoulder[3],
                  double pos_FR_elbow[3], double pos_FL_elbow[3], double pos_RR_elbow[3], double pos_RL_elbow[3],
                  double pos_FR_foot[3], double pos_FL_foot[3], double pos_RR_foot[3], double pos_RL_foot[3], double robot_com[3], double tau[nact], int flag, int state, double t,
                  double theta_FR_thigh_ctrl, double theta_FR_calf_ctrl,
                  double theta_FL_thigh_ctrl, double theta_FL_calf_ctrl,
                  double theta_RR_thigh_ctrl, double theta_RR_calf_ctrl,
                  double theta_RL_thigh_ctrl, double theta_RL_calf_ctrl)
{
    int i;

    if (state == 1)
    {
        double target[]={0,theta_FR_thigh_ctrl,theta_FR_calf_ctrl,   //FR B
                         0,theta_FL_thigh_ctrl,theta_FL_calf_ctrl,   //FL A
                         0,theta_RR_thigh_ctrl,theta_RR_calf_ctrl,   //RR A
                         0,theta_RL_thigh_ctrl,theta_RL_calf_ctrl};  //RL B

        for (i=0;i<nact;i++)
        {
            setpt[i]=target[i];
        }

        for (i=0; i<nact; i++)
        {
            tau[i] = -250*(q[i]-setpt[i])-10*u[i];
        }
    }     
}