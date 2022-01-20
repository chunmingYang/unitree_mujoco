double setpt[nact] = {0};

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

void control_loop(double q[nact], double u[nact], double base_x[3], double base_quat[4],
                  double base_angvel[3], double base_linacc[3], double base_linvel[3],
                  double pos_FR_shoulder[3], double pos_FL_shoulder[3], double pos_RR_shoulder[3], double pos_RL_shoulder[3],
                  double pos_FR_elbow[3], double pos_FL_elbow[3], double pos_RR_elbow[3], double pos_RL_elbow[3],
                  double pos_FR_foot[3], double pos_FL_foot[3], double pos_RR_foot[3], double pos_RL_foot[3], double robot_com[3], double tau[nact], int flag, int state, double t)
{
    int i;

    if (state == 4)
    {
        double target[]={0,0.9,-2.2,   //FR B
                         0,0.5,-1.7,   //FL A
                         0,0.5,-1.7,   //RR A
                         0,0.9,-2.2};  //RL B
        
        for (i=0;i<nact;i++)
        {
            setpt[i]=target[i];
        }

        for (i=0; i<nact; i++)
        {
            tau[i] = -150*(q[i]-setpt[i])-10*u[i];
        }
    }

    if (state == 1) // A toughdown then A stance
    {
        double target[]={0,0.5,-2.2,   //FR B
                         0,0.9,-1.4,   //FL A
                         0,0.9,-1.4,   //RR A
                         0,0.5,-2.2};  //RL B
        
        for (i=0;i<nact;i++)
        {
            setpt[i]=target[i];
        }

        for (i=0; i<nact; i++)
        {
            tau[i] = -150*(q[i]-setpt[i])-10*u[i];
        }
    }

    if (state == 2)
    {
        double target[]={0,0.5,-1.7,   //FR B
                         0,0.9,-2.2,   //FL A
                         0,0.9,-2.2,   //RR A
                         0,0.5,-1.7};  //RL B
        
        for (i=0;i<nact;i++)
        {
            setpt[i]=target[i];
        }

        for (i=0; i<nact; i++)
        {
            tau[i] = -150*(q[i]-setpt[i])-10*u[i];
        }
    }

    if (state == 3) // B toughdown then B stance
    {
        double target[]={0,0.9,-1.4,   //FR B
                         0,0.5,-2.2,   //FL A
                         0,0.5,-2.2,   //RR A
                         0,0.9,-1.4};  //RL B
        
        for (i=0;i<nact;i++)
        {
            setpt[i]=target[i];
        }

        for (i=0; i<nact; i++)
        {
            tau[i] = -150*(q[i]-setpt[i])-10*u[i];
        }
    }        
}