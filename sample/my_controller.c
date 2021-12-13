double setpt[nact] = {0};

void init_controller()
{
    int i;
    double target[]={0,0,-0.9,   //FR
                     0,0,-0.9,   //FL
                     0,0,-0.9,   //RR
                     0,0,-0.9};  //RL
    
    for (i=0;i<nact;i++)
    {
        setpt[i]=target[i];
    }
}

void control_loop(double q[nact], double u[nact], double base_x[3], double base_quat[4],
               double base_angvel[3], double base_linacc[3], double base_linvel[3],
               double pos_FR_foot[3], double pos_FL_foot[3], double pos_RR_foot[3], double pos_RL_foot[3], double robot_com[3], double tau[nact],
               int flag)
{
    int i;

    if (flag == 1)
    {
        double target[]={0,0,-1.5,   //FR
                         0,0,-1.5,   //FL
                         0,0,-1.5,   //RR
                         0,0,-1.5};  //RL
        for (i=0;i<nact;i++)
        {
        setpt[i]=target[i];
        }
    }

    if (flag == -1)
    {
        double target[]={0,0,-1.2,   //FR
                         0,0,-1.2,   //FL
                         0,0,-1.2,   //RR
                         0,0,-1.2};  //RL
        for (i=0;i<nact;i++)
        {
        setpt[i]=target[i];
        }
    }

    for (i=0; i<nact; i++)
    {
        tau[i] = -200*(q[i]-setpt[i])-10*u[i];
    }
}

