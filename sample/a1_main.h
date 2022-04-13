#define nact 12            // number of actuator
#define nbase 16           // body information like body position, orientation, angvel, linacc, linvel
#define nendeff 12         // 4 toes each one has 3 coordinates, 3*4

double trq_max = 33.5;
double trq_min = -33.5;

// There are 12 motors on the a1
double joint_min[] = 
{
  -0.802851,
  -1.0472,
  -2.69653,
  -0.802851,
  -1.0472,
  -2.69653,
  -0.802851,
  -1.0472,
  -2.69653,
  -0.802851,
  -1.0472,
  -2.69653
};

double joint_max[] = 
{
  0.802851,
  4.18879,
  -0.916298,
  0.802851,
  4.18879,
  -0.916298,
  0.802851,
  4.18879,
  -0.916298,
  0.802851,
  4.18879,
  -0.916298,
  0.802851,
  4.18879,
  -0.916298
};
