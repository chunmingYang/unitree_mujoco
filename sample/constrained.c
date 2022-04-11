/* modifiy based on pranav's mujoco mini class (https://www.youtube.com/watch?v=qMRUxFexKUc) (https://pab47.github.io/mujoco.html) */
/* simple nlopt example reference to (https://www.youtube.com/watch?v=m75bHHkPgqo) (https://pab47.github.io/mujoco.html) */


#include <stdio.h>
#include <math.h>
#include "nlopt.h"
extern mjModel* m;
// extern mjData* d;
extern mjData* dsim;
double x_target, z_target;


/*
  optimization problem is given foot position then output joints' angle
    --- cost = 0
    --- euqality constraint: eqc_1 = x_ref - x_real
                             eqc_2 = z_ref - z_real
    --- decision variables (root): Xin = {q_1, q_2}
    --- decision variables bounds: q_1, q_2 = {-3.14, 3.14}
*/

// data structure type set up
// typedef struct {
//     double a[5];
// }mycost_data;
// typedef struct {
//     double ceq_1;
//     double ceq_2;
// }myequalityconstraints_data;
// typedef struct {
//     double cin_1;
// }myinequalityconstraints_data;

// simulator loop
void simulator(double Xin[2], double Xout[2])
{
  // update the state --- set the joints' angle
  int thigh_jnt_adr = m->jnt_qposadr[2]; // thigh_jnt_id=2 from MJMODEL.TXT
  int calf_jnt_adr = m->jnt_qposadr[3]; // calf_jnt_id=3 from MJMODEL.TXT
  dsim->qpos[thigh_jnt_adr] = Xin[0]; // FR thigh joint
  dsim->qpos[calf_jnt_adr] = Xin[1]; // FR calf joint
  // mj_forward(m, dsim); // model forward propagation
  mj_forward(m, dsim);
  int pos_FR_adr = m->sensor_adr[29];   // from MJMODEL find FR_foot_pos sensorID 29
  Xout[0] = dsim->sensordata[pos_FR_adr]; // FR foot x position
  Xout[1] = dsim->sensordata[pos_FR_adr+2]; // FR foot z position
  
  // checking
  // printf("simulator works fine\n");
  // printf("foot pos x, z, is %f %f\n", Xout[0], Xout[1]);
}

// cost function set up
double mycost(unsigned n, const double *x, double *grad, void *costdata)
{
  // mycost_data *data = (mycost_data *) costdata;
  // int i;
  // double a[5]={0};
  // for (i=0;i<n;i++)
  //   a[i] = data->a[i];
  double cost = 0;
  // for (i=0;i<n;i++)
  //   cost += a[i]*x[i]*x[i];
  return cost;
}

// equality constraint set up
void myequalityconstraints(unsigned m, double *result, unsigned n,
                             const double *x,  double *grad,
                             void *equalitydata)
{
  // myequalityconstraints_data *data = (myequalityconstraints_data *) equalitydata;
  // double c1 = data->ceq_1;
  // double c2 = data->ceq_2;
  // double x1 = x[0], x2 = x[1], x3 = x[2], x4 = x[3], x5 = x[4];
  // result[0] = x1+x2+x3-c1; //5;
  // result[1] = x3*x3+x4-c2; //2;
  
  double Xin[2];
  double Xout[2];
  Xin[0] = x[0];
  Xin[1] = x[1];
  simulator(Xin, Xout); // x is the optimization variables that is joints' angle, Xout is the foot position
  result[0] = x_target - Xout[0];  // foot x position
  result[1] = z_target - Xout[1];  // foot z position
}

// inequality constraint set up
//  void myinequalityconstraints(unsigned m, double *result, unsigned n,
//                                 const double *x,  double *grad,
//                                 void* inequalitydata)
// {
//   myinequalityconstraints_data *data = (myinequalityconstraints_data *) inequalitydata;
//   double c1 = data->cin_1;
//   double x1 = x[0], x2 = x[1], x3 = x[2], x4 = x[3], x5 = x[4];
//   result[0] = x4*x4+x5*x5-c1; //5;
// }

// nlopt optimization main loop
void nlopt_ik(double Xin[2], double Xref[2])
{
  int i;
  nlopt_opt opt;
  x_target = Xref[0];
  z_target = Xref[1];

  //establish sizes
  unsigned n = 2; //number of decision variables
  unsigned m_eq = 2; //number of equality constraints
  // unsigned m_in = 1; //number of inequality constraints

  //bounds for decision variables
  double lb[] = {-3.14, -3.14}; /* lower bounds */
  double ub[] = {3.14, 3.14}; /* upper bounds */

  //Set the algorithm and dimensionality
  //L,G = global/local
  //D,N = derivative / no derivative
  opt = nlopt_create(NLOPT_LN_COBYLA, n); /* algorithm and dimensionality */

  //Set the lower and upper bounds
  nlopt_set_lower_bounds(opt, lb);
  nlopt_set_upper_bounds(opt, ub);

  //Set up cost
  // mycost_data costdata;
  // for (i=0;i<n;i++)
  //   costdata.a[i]=1;
  nlopt_set_min_objective(opt, mycost, NULL);

  //set up equality constraint
  double tol_eq[]={1e-8,1e-8};
  // myequalityconstraints_data equalitydata;
  // equalitydata.ceq_1 = 5;
  // equalitydata.ceq_2 = 2;
  nlopt_add_equality_mconstraint(opt, m_eq, myequalityconstraints, NULL, tol_eq);

  // double tol_in[]={1e-8};
  // myinequalityconstraints_data inequalitydata;
  // inequalitydata.cin_1 = 5;
  // nlopt_add_inequality_mconstraint(opt, m_in, myinequalityconstraints,&inequalitydata, tol_in);

  nlopt_set_xtol_rel(opt, 1e-4);
  // double x[] = { 1, 1, 1, 2, 1 };  // initial guess
  double minf; /* `*`the` `minimum` `objective` `value,` `upon` `return`*` */
  if (nlopt_optimize(opt, Xin, &minf) < 0) {
      printf("nlopt failed!\n");
  }
  else {
    // printf("******************\n");
    // printf("found minimum at f(%g,%g) = %0.10g\n", Xin[0], Xin[1], minf);
  }

  nlopt_destroy(opt);

  // check the extern variables
  // printf("time is %f\n", dsim->time);
}

// some checking
void check(double Xin[2], double Xout[2])
{
  // update the state --- set the joints' angle
  int thigh_jnt_adr = m->jnt_qposadr[2]; // thigh_jnt_id=2 from MJMODEL.TXT
  int calf_jnt_adr = m->jnt_qposadr[3]; // calf_jnt_id=3 from MJMODEL.TXT
  dsim->qpos[thigh_jnt_adr] = Xin[0]; // FR thigh joint
  dsim->qpos[calf_jnt_adr] = Xin[1]; // FR calf joint
  printf("qpos_1, qpos_2: %f, %f\n", dsim->qpos[thigh_jnt_adr], dsim->qpos[calf_jnt_adr]);
  mj_forward(m, dsim);
  // mj_forward(m,dsim); // model forward propagation
  int pos_FR_adr = m->sensor_adr[29];   // from MJMODEL find FR_foot_pos sensorID 29
  Xout[0] = dsim->sensordata[pos_FR_adr]; // FR foot x position
  Xout[1] = dsim->sensordata[pos_FR_adr+2]; // FR foot z position
  printf("foot position in x, z is %f %f\n", Xout[0], Xout[1]);
}
