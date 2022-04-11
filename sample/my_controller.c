double setpt[nact] = {0};
double theta_thigh_ctrl;
double theta_calf_ctrl;

void init_controller()
{
    int i;
    double target[]={0,0.9,-2.2,   //FR B
                     0,0.9,-2.2,   //FL A
                     0,0.9,-2.2,   //RR A
                     0,0.9,-2.2};  //RL B
    
    for (i=0;i<nact;i++)
    {
        setpt[i]=target[i];
    }
}

void optimizer(const mjModel* m, mjData* d, mjData* dsim)
{
    // optimizer main loop
    if (d->time <= 5)
    {
        // trajectory generation
        double t0 = 0;
        double tf = 10;
        double y0 = 0.199341+0.05;
        double yf = 0.199341+0.4;
        double a0 = (yf*t0*t0*(3*tf-t0) + y0*tf*tf*(tf-3*t0))/((tf-t0)*(tf-t0)*(tf-t0));
        double a1 = 6*t0*tf*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));
        double a2 = 3*(t0+tf)*(yf-y0)/((tf-t0)*(tf-t0)*(tf-t0));
        double a3 = 2*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));
        double x = 0;
        double y = a0 + a1*d->time + a2*d->time*d->time + a3*d->time*d->time*d->time;

        // nlopt optimizer
        double Xin[2] = {0.3, -1.2};
        double Xref[2] = {0.183026, y};
        nlopt_ik(Xin, Xref);
        
        // input for torque PD controller
        theta_thigh_ctrl =  Xin[0]; 
        theta_calf_ctrl = Xin[1];
    }
}

void control_loop(const mjModel* m, mjData* d,
                  double q[nact], double u[nact], double base_x[3], double base_quat[4],
                  double base_angvel[3], double base_linacc[3], double base_linvel[3],
                  double pos_FR_shoulder[3], double pos_FL_shoulder[3], double pos_RR_shoulder[3], double pos_RL_shoulder[3],
                  double pos_FR_elbow[3], double pos_FL_elbow[3], double pos_RR_elbow[3], double pos_RL_elbow[3],
                  double pos_FR_foot[3], double pos_FL_foot[3], double pos_RR_foot[3], double pos_RL_foot[3], double robot_com[3], double tau[nact], int flag, int state, double t)
{
    int i;

    // torque pd controller
    if (state == 4)
    {
        double target[]={0,theta_thigh_ctrl,theta_calf_ctrl,   //FR B
                         0,0.5,-1.7,                           //FL A
                         0,0.5,-1.7,                           //RR A
                         0,0.9,-2.2};                          //RL B
        
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