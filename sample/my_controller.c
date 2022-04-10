double setpt[nact] = {0};

// sdroot() initialization
#define NFUNC 2                 // number of functions
#define NVAR  2                 // number of variables -> root -> joints' angle in this case
#define NDES  0
#define N_JW NFUNC*NVAR
#define N_DW 2*(NFUNC+NVAR)*(NFUNC+NVAR)
#define N_RW 9*(NFUNC+NVAR)
#define N_IW 4*(NFUNC+NVAR)
#define REF_X 0.3
#define REF_Y 0
double *ref_x;
double *ref_y;
int constraint_eqn(double vars[NVAR],double param[0],double defect[NFUNC])
{
    double pos_foot_x = *ref_x;      // given foot position x
	double pos_foot_y = *ref_y;      // given foot position y
  	double theta1 = vars[0];         // find joint1 angle
  	double theta2 = vars[1];         // find joint2 angle
	double l1 = 0.2;
    double l2 = 0.2;
  	defect[0] =  l1*sin(theta1) + l2*sin(theta1+theta2) - pos_foot_x;    // from analytical solver explanation
	defect[1] =  l1*cos(theta1) + l2*cos(theta1+theta2) - pos_foot_y;    // from analytical solver explanation
}

// controller initialization
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

// main controller loop
void control_loop(const mjModel* m, mjData* d,
                  double q[nact], double u[nact], double base_x[3], double base_quat[4],
                  double base_angvel[3], double base_linacc[3], double base_linvel[3],
                  double pos_FR_shoulder[3], double pos_FL_shoulder[3], double pos_RR_shoulder[3], double pos_RL_shoulder[3],
                  double pos_FR_elbow[3], double pos_FL_elbow[3], double pos_RR_elbow[3], double pos_RL_elbow[3],
                  double pos_FR_foot[3], double pos_FL_foot[3], double pos_RR_foot[3], double pos_RL_foot[3], double robot_com[3], double tau[nact], int flag, int state, double t)
{
    int i;

    // ********************************************************* sdroot solvers ********************************************************* //
    double vars[NVAR]={0};       //number of optimization variables
  	double defect[NFUNC]={0};    //constraint = 0
  	double param[1] = {0};       //no parameters (is independent of NVAR or NFUNC)
	int lock[NVAR] = {0};        //no vars are locked
	double root_tol = 1e-10;     //tolerance for root
	int root_maxevals = 100;     //number of evaluations permitted
	double jw[N_JW];             //nfunc*nvar
	double dw[N_DW];             //2*(nfunc+nvar)^2
	double rw[N_RW];             //9*(nfunc+nvar)
	int iw[N_IW];                //4*(nfunc+nvar)
    int fcnt,rooterr;
    double theta_thigh_ctrl = 0;
    double theta_calf_ctrl = 0;
    
    // initial guess
    vars[0] = 0;
    vars[1] = 0;

    if (d->time <=10)
    {
        // trajectory generation (x=0.183 y=-0.13205 z=[0.25 0.45] z_hip=0.599156) --- using cubic trajectory
        double t0 = 0;
        double tf = 10;
        double y0 = 0.2;
        double yf = 0.4;
        double a0 = (yf*t0*t0*(3*tf-t0) + y0*tf*tf*(tf-3*t0))/((tf-t0)*(tf-t0)*(tf-t0));
        double a1 = 6*t0*tf*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));
        double a2 = 3*(t0+tf)*(yf-y0)/((tf-t0)*(tf-t0)*(tf-t0));
        double a3 = 2*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));
        double x = 0;
        double y = a0 + a1*d->time + a2*d->time*d->time + a3*d->time*d->time*d->time;
        ref_x=&x;
        ref_y=&y;
        
        // sdroot solver
        sdroot(constraint_eqn,vars,param,NFUNC,NVAR,NDES,lock,root_tol,root_tol,root_maxevals,
           jw,dw,rw,iw,defect,&fcnt,&rooterr);

        // input for pd controller
        theta_thigh_ctrl = -vars[0];
        theta_calf_ctrl = -vars[1];
    }
    else
    {
        theta_thigh_ctrl = 0;
        theta_calf_ctrl = 0;
    }
    // ********************************************************* sdroot solvers ********************************************************* //

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