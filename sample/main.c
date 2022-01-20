#include <stdbool.h>      
#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "a1_main.h"
#include "my_estimator.c"
#include "my_controller.c"

// initialization
int q_id[nact]={0}, v_id[nact]={0}, act_id[nact]={0};
int base_stateid[nbase]={0};
int shoulder_stateid[nendeff]={0}, elbow_stateid[nendeff]={0}, endeff_stateid[nendeff]={0};
int com_id[3]={0};
mjModel* m = NULL;
mjData* d = NULL;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

// data collection setting
FILE *fid;
char path[] = "../sample/data.csv";
int loop_index = 0;
const int data_frequency = 10;

// mouse interaction initialization
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

// initialize the ids
void get_ID()
{
    const char* joint_name;
    const char* actuator_name;
    int i, jointid;

    // joints
    joint_name = "FR_hip_joint";
    actuator_name = "FR_hip";
    jointid = mj_name2id(m, mjOBJ_JOINT, joint_name);
    int FR_hip_joint_qpos_adr = m->jnt_qposadr[jointid];
    int FR_hip_joint_qvel_adr = m->jnt_dofadr[jointid];
    int FR_hip_joint_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name);

    joint_name = "FR_thigh_joint";
    actuator_name = "FR_thigh";
    jointid = mj_name2id(m, mjOBJ_JOINT, joint_name);
    int FR_thigh_joint_qpos_adr = m->jnt_qposadr[jointid];
    int FR_thigh_joint_qvel_adr = m->jnt_dofadr[jointid];
    int FR_thigh_joint_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name);

    joint_name = "FR_calf_joint";
    actuator_name = "FR_calf";
    jointid = mj_name2id(m, mjOBJ_JOINT, joint_name);
    int FR_calf_joint_qpos_adr = m->jnt_qposadr[jointid];
    int FR_calf_joint_qvel_adr = m->jnt_dofadr[jointid];
    int FR_calf_joint_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name);

    joint_name = "FL_hip_joint";
    actuator_name = "FL_hip";
    jointid = mj_name2id(m, mjOBJ_JOINT, joint_name);
    int FL_hip_joint_qpos_adr = m->jnt_qposadr[jointid];
    int FL_hip_joint_qvel_adr = m->jnt_dofadr[jointid];
    int FL_hip_joint_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name);

    joint_name = "FL_thigh_joint";
    actuator_name = "FL_thigh";
    jointid = mj_name2id(m, mjOBJ_JOINT, joint_name);
    int FL_thigh_joint_qpos_adr = m->jnt_qposadr[jointid];
    int FL_thigh_joint_qvel_adr = m->jnt_dofadr[jointid];
    int FL_thigh_joint_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name);
  
    joint_name = "FL_calf_joint";
    actuator_name = "FL_calf";
    jointid = mj_name2id(m, mjOBJ_JOINT, joint_name);
    int FL_calf_joint_qpos_adr = m->jnt_qposadr[jointid];
    int FL_calf_joint_qvel_adr = m->jnt_dofadr[jointid];
    int FL_calf_joint_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name);

    joint_name = "RR_hip_joint";
    actuator_name = "RR_hip";
    jointid = mj_name2id(m, mjOBJ_JOINT, joint_name);
    int RR_hip_joint_qpos_adr = m->jnt_qposadr[jointid];
    int RR_hip_joint_qvel_adr = m->jnt_dofadr[jointid];
    int RR_hip_joint_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name);
  
    joint_name = "RR_thigh_joint";
    actuator_name = "RR_thigh";
    jointid = mj_name2id(m, mjOBJ_JOINT, joint_name);
    int RR_thigh_joint_qpos_adr = m->jnt_qposadr[jointid];
    int RR_thigh_joint_qvel_adr = m->jnt_dofadr[jointid];
    int RR_thigh_joint_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name);
  
    joint_name = "RR_calf_joint";
    actuator_name = "RR_calf";
    jointid = mj_name2id(m, mjOBJ_JOINT, joint_name);
    int RR_calf_joint_qpos_adr = m->jnt_qposadr[jointid];
    int RR_calf_joint_qvel_adr = m->jnt_dofadr[jointid];
    int RR_calf_joint_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name);

    joint_name = "RL_hip_joint";
    actuator_name = "RL_hip";
    jointid = mj_name2id(m, mjOBJ_JOINT, joint_name);
    int RL_hip_joint_qpos_adr = m->jnt_qposadr[jointid];
    int RL_hip_joint_qvel_adr = m->jnt_dofadr[jointid];
    int RL_hip_joint_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name);

    joint_name = "RL_thigh_joint";
    actuator_name = "RL_thigh";
    jointid = mj_name2id(m, mjOBJ_JOINT, joint_name);
    int RL_thigh_joint_qpos_adr = m->jnt_qposadr[jointid];
    int RL_thigh_joint_qvel_adr = m->jnt_dofadr[jointid];
    int RL_thigh_joint_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name);

    joint_name = "RL_calf_joint";
    actuator_name = "RL_calf";
    jointid = mj_name2id(m, mjOBJ_JOINT, joint_name);
    int RL_calf_joint_qpos_adr = m->jnt_qposadr[jointid];
    int RL_calf_joint_qvel_adr = m->jnt_dofadr[jointid];
    int RL_calf_joint_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name);

    // base
    const char* base_pos_name = "Body_Pos";
    int base_sensorID = mj_name2id(m, mjOBJ_SENSOR, base_pos_name);
    int base_sensor_adr = m->sensor_adr[base_sensorID];

    const char* base_quat_name = "Body_Quat";
    int base_quat_sensorID = mj_name2id(m, mjOBJ_SENSOR, base_quat_name);
    int base_quat_sensor_adr = m->sensor_adr[base_quat_sensorID];

    int base_linvel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "Body_LinVel");
    int base_linvel_sensor_adr = m->sensor_adr[base_linvel_sensorID];

    int base_angvel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "Body_Gyro");
    int base_angvel_sensor_adr = m->sensor_adr[base_angvel_sensorID];

    int base_linacc_sensorID = mj_name2id(m, mjOBJ_SENSOR, "Body_Acc");
    int base_linacc_sensor_adr = m->sensor_adr[base_linacc_sensorID];
    
    // shoulder
    const char* pos_FR_shoulder = "FR_shoulder_pos";
    int pos_FR_shoulder_sensorID = mj_name2id(m, mjOBJ_SENSOR, pos_FR_shoulder);
    int pos_FR_shoulder_adr = m->sensor_adr[pos_FR_shoulder_sensorID];

    const char* pos_FL_shoulder = "FL_shoulder_pos";
    int pos_FL_shoulder_sensorID = mj_name2id(m, mjOBJ_SENSOR, pos_FL_shoulder);
    int pos_FL_shoulder_adr = m->sensor_adr[pos_FL_shoulder_sensorID];

    const char* pos_RR_shoulder = "RR_shoulder_pos";
    int pos_RR_shoulder_sensorID = mj_name2id(m, mjOBJ_SENSOR, pos_RR_shoulder);
    int pos_RR_shoulder_adr = m->sensor_adr[pos_RR_shoulder_sensorID];

    const char* pos_RL_shoulder = "RL_shoulder_pos";
    int pos_RL_shoulder_sensorID = mj_name2id(m, mjOBJ_SENSOR, pos_RL_shoulder);
    int pos_RL_shoulder_adr = m->sensor_adr[pos_RL_shoulder_sensorID];
    
    // elbow
    const char* pos_FR_elbow = "FR_elbow_pos";
    int pos_FR_elbow_sensorID = mj_name2id(m, mjOBJ_SENSOR, pos_FR_elbow);
    int pos_FR_elbow_adr = m->sensor_adr[pos_FR_elbow_sensorID];

    const char* pos_FL_elbow = "FL_elbow_pos";
    int pos_FL_elbow_sensorID = mj_name2id(m, mjOBJ_SENSOR, pos_FL_elbow);
    int pos_FL_elbow_adr = m->sensor_adr[pos_FL_elbow_sensorID];

    const char* pos_RR_elbow = "RR_elbow_pos";
    int pos_RR_elbow_sensorID = mj_name2id(m, mjOBJ_SENSOR, pos_RR_elbow);
    int pos_RR_elbow_adr = m->sensor_adr[pos_RR_elbow_sensorID];

    const char* pos_RL_elbow = "RL_elbow_pos";
    int pos_RL_elbow_sensorID = mj_name2id(m, mjOBJ_SENSOR, pos_RL_elbow);
    int pos_RL_elbow_adr = m->sensor_adr[pos_RL_elbow_sensorID];
    
    // foot
    const char* pos_FR_foot = "FR_foot_pos";
    int pos_FR_sensorID = mj_name2id(m, mjOBJ_SENSOR, pos_FR_foot);
    int pos_FR_adr = m->sensor_adr[pos_FR_sensorID];

    const char* pos_FL_foot = "FL_foot_pos";
    int pos_FL_sensorID = mj_name2id(m, mjOBJ_SENSOR, pos_FL_foot);
    int pos_FL_adr = m->sensor_adr[pos_FL_sensorID];

    const char* pos_RR_foot = "RR_foot_pos";
    int pos_RR_sensorID = mj_name2id(m, mjOBJ_SENSOR, pos_RR_foot);
    int pos_RR_adr = m->sensor_adr[pos_RR_sensorID];

    const char* pos_RL_foot = "RL_foot_pos";
    int pos_RL_sensorID = mj_name2id(m, mjOBJ_SENSOR, pos_RL_foot);
    int pos_RL_adr = m->sensor_adr[pos_RL_sensorID];

    // COM
    const char* robot_com = "robot_com";
    int com_sensorID = mj_name2id(m, mjOBJ_SENSOR, robot_com);
    int com_sensor_adr = m->sensor_adr[com_sensorID];

    // collections
    int temp_IDq[] = {FR_hip_joint_qpos_adr,FR_thigh_joint_qpos_adr,FR_calf_joint_qpos_adr,
                     FL_hip_joint_qpos_adr,FL_thigh_joint_qpos_adr,FL_calf_joint_qpos_adr,
                     RR_hip_joint_qpos_adr,RR_thigh_joint_qpos_adr,RR_calf_joint_qpos_adr,
                     RL_hip_joint_qpos_adr,RL_thigh_joint_qpos_adr,RL_calf_joint_qpos_adr};

    int temp_IDv[] = {FR_hip_joint_qvel_adr,FR_thigh_joint_qvel_adr,FR_calf_joint_qvel_adr,
                     FL_hip_joint_qvel_adr,FL_thigh_joint_qvel_adr,FL_calf_joint_qvel_adr,
                     RR_hip_joint_qvel_adr,RR_thigh_joint_qvel_adr,RR_calf_joint_qvel_adr,
                     RL_hip_joint_qvel_adr,RL_thigh_joint_qvel_adr,RL_calf_joint_qvel_adr};

    int temp_IDT[] = {FR_hip_joint_actuatorID,FR_thigh_joint_actuatorID,FR_calf_joint_actuatorID,
                     FL_hip_joint_actuatorID,FL_thigh_joint_actuatorID,FL_calf_joint_actuatorID,
                     RR_hip_joint_actuatorID,RR_thigh_joint_actuatorID,RR_calf_joint_actuatorID,
                     RL_hip_joint_actuatorID,RL_thigh_joint_actuatorID,RL_calf_joint_actuatorID};

    int temp_baseID[] = {base_sensor_adr,base_sensor_adr+1,base_sensor_adr+2,
                        base_quat_sensor_adr,base_quat_sensor_adr+1,base_quat_sensor_adr+2,base_quat_sensor_adr+3,
                        base_angvel_sensor_adr,base_angvel_sensor_adr+1,base_angvel_sensor_adr+2,
                        base_linacc_sensor_adr,base_linacc_sensor_adr+1,base_linacc_sensor_adr+2,
                        base_linvel_sensor_adr,base_linvel_sensor_adr+1,base_linvel_sensor_adr+2};

    int temp_shoulderID[] = {pos_FR_shoulder_adr,pos_FR_shoulder_adr+1,pos_FR_shoulder_adr+2,
                             pos_FL_shoulder_adr,pos_FL_shoulder_adr+1,pos_FL_shoulder_adr+2,
                             pos_RR_shoulder_adr,pos_RR_shoulder_adr+1,pos_RR_shoulder_adr+2,
                             pos_RL_shoulder_adr,pos_RL_shoulder_adr+1,pos_RL_shoulder_adr+2};

    int temp_elbowID[] = {pos_FR_elbow_adr,pos_FR_elbow_adr+1,pos_FR_elbow_adr+2,
                           pos_FL_elbow_adr,pos_FL_elbow_adr+1,pos_FL_elbow_adr+2,
                           pos_RR_elbow_adr,pos_RR_elbow_adr+1,pos_RR_elbow_adr+2,
                           pos_RL_elbow_adr,pos_RL_elbow_adr+1,pos_RL_elbow_adr+2};

    int temp_endeffectorID[] = {pos_FR_adr,pos_FR_adr+1,pos_FR_adr+2,
                               pos_FL_adr,pos_FL_adr+1,pos_FL_adr+2,
                               pos_RR_adr,pos_RR_adr+1,pos_RR_adr+2,
                               pos_RL_adr,pos_RL_adr+1,pos_RL_adr+2};

    int temp_comID[] = {com_sensor_adr,com_sensor_adr+1,com_sensor_adr+2};

    // applied ids
    for (i=0;i<nact;i++)
    {
        q_id[i] = temp_IDq[i];
        v_id[i] = temp_IDv[i];
        act_id[i] = temp_IDT[i];
    }

    for (i=0;i<nbase;i++)
    {
        base_stateid[i] = temp_baseID[i];
    }

    for (i=0;i<nendeff;i++)
    {
        shoulder_stateid[i] = temp_shoulderID[i];
        elbow_stateid[i] = temp_elbowID[i];
        endeff_stateid[i] = temp_endeffectorID[i];
    }

    for (i=0;i<3;i++)
    {
        com_id[i] = temp_comID[i];
    }
}

// get states from the simulator - kind of like loading data according to ids
void get_state(double q[nact],double u[nact],double base_x[3],double base_quat[4],
               double base_angvel[3],double base_linacc[3],double base_linvel[3],
               double pos_FR_shoulder[3],double pos_FL_shoulder[3],double pos_RR_shoulder[3],double pos_RL_shoulder[3],
               double pos_FR_elbow[3],double pos_FL_elbow[3],double pos_RR_elbow[3],double pos_RL_elbow[3],
               double pos_FR_foot[3],double pos_FL_foot[3],double pos_RR_foot[3],double pos_RL_foot[3],double robot_com[3])
{
    int i;

    // joints
    for (i=0;i<nact;i++)
    {
        q[i] = d->qpos[q_id[i]];
        u[i] = d->qvel[v_id[i]];
    }
    
    // base
    for (i=0;i<nbase;i++)
    {
        if (i<=2)
        {
            base_x[i] = d->sensordata[base_stateid[i]];
        }

        else if (i>2 && i <=6)
        {
            base_quat[i-3] = d->sensordata[base_stateid[i]];
        }

        else if (i>6 && i<=9)
        {
            base_angvel[i-7] = d->sensordata[base_stateid[i]];
        }

        else if (i>9 && i<=12)
        {
            base_linacc[i-10] = d->sensordata[base_stateid[i]];
        }

        else if (i>12 && i<=15)
        {
            base_linvel[i-13] = d->sensordata[base_stateid[i]]; 
        }
    }

    // foot
    for (i=0;i<nendeff;i++)
    {
        if (i>=0 && i<3)
        {
            pos_FR_shoulder[i]=d->sensordata[shoulder_stateid[i]];
            pos_FR_elbow[i]=d->sensordata[elbow_stateid[i]];
            pos_FR_foot[i]=d->sensordata[endeff_stateid[i]];
        }

        if (i>=3 && i<6)
        {
            pos_FL_shoulder[i-3]=d->sensordata[shoulder_stateid[i]];
            pos_FL_elbow[i-3]=d->sensordata[elbow_stateid[i]];
            pos_FL_foot[i-3]=d->sensordata[endeff_stateid[i]]; 
        }

        if (i>=6 && i<9)
        {
            pos_RR_shoulder[i-6]=d->sensordata[shoulder_stateid[i]];
            pos_RR_elbow[i-6]=d->sensordata[elbow_stateid[i]];
            pos_RR_foot[i-6]=d->sensordata[endeff_stateid[i]];
        }

        if (i>=9 && i<12)
        {
            pos_RL_shoulder[i-9]=d->sensordata[shoulder_stateid[i]];
            pos_RL_elbow[i-9]=d->sensordata[elbow_stateid[i]];
            pos_RL_foot[i-9]=d->sensordata[endeff_stateid[i]];
        }
    }

    // COM
    for (i=0;i<3;i++)
    {
        robot_com[i]=d->sensordata[com_id[i]];
    }
}

// mjData saving
void init_save_data()
{
    fprintf(fid,"t,");
    fprintf(fid,"x, y, z, q0, qx, qy, qz, vx, vy, vz, wx, wy, wz,");
    fprintf(fid,"q_FR_hip,q_FR_thigh,q_FR_calf,q_FL_hip,q_FL_thigh,q_FL_calf,q_RR_hip,q_RR_thigh,q_RR_calR,q_RL_hip,q_RL_thigh,q_RL_calR,");
    fprintf(fid,"v_FR_hip,v_FR_thigh,v_FR_calf,v_FL_hip,v_FL_thigh,v_FL_calf,v_RR_hip,v_RR_thigh,v_RR_calR,v_RL_hip,v_RL_thigh,v_RL_calR,");
    fprintf(fid,"tau_FR_hip,tau_FR_thigh,tau_FR_calf,tau_FL_hip,tau_FL_thigh,tau_FL_calf,tau_RR_hip,tau_RR_thigh,tau_RR_calR,tau_RL_hip,tau_RL_thigh,tau_RL_calR,");
    fprintf(fid,"com_x,com_y,com_z,");
    fprintf(fid,"pos_FR_shoulder_x,pos_FR_shoulder_y,pos_FR_shoulder_z,pos_FL_shoulder_x,pos_FL_shoulder_y,pos_FL_shoulder_z,pos_RR_shoulder_x,pos_RR_shoulder_y,pos_RR_shoulder_z,pos_RL_shoulder_x,pos_RL_shoulder_y,pos_RL_shoulder_z,");
    fprintf(fid,"pos_FR_elbow_x,pos_FR_elbow_y,pos_FR_elbow_z,pos_FL_elbow_x,pos_FL_elbow_y,pos_FL_elbow_z,pos_RR_elbow_x,pos_RR_elbow_y,pos_RR_elbow_z,pos_RL_elbow_x,pos_RL_elbow_y,pos_RL_elbow_z,");
    fprintf(fid,"pos_FR_foot_x,pos_FR_foot_y,pos_FR_foot_z,pos_FL_foot_x,pos_FL_foot_y,pos_FL_foot_z,pos_RR_foot_x,pos_RR_foot_y,pos_RR_foot_z,pos_RL_foot_x,pos_RL_foot_y,pos_RL_foot_z,");
    fprintf(fid,"\n");
}
void save_data(double t,double q[nact],double u[nact],double tau[nact],double base_x[3],double base_quat[4],
               double base_angvel[3],double base_linacc[3],double base_linvel[3],
               double pos_FR_shoulder[3],double pos_FL_shoulder[3],double pos_RR_shoulder[3],double pos_RL_shoulder[3],
               double pos_FR_elbow[3],double pos_FL_elbow[3],double pos_RR_elbow[3],double pos_RL_elbow[3],
               double pos_FR_foot[3],double pos_FL_foot[3],double pos_RR_foot[3],double pos_RL_foot[3],
               double robot_com[3])
{
    int i;
    fprintf(fid,"%f,",t);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",base_x[i]);
    for (i=0;i<4;i++)
        fprintf(fid,"%f,",base_quat[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",base_linvel[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",base_angvel[i]);
    for (i=0;i<nact;i++)
        fprintf(fid,"%f,",q[i]);
    for (i=0;i<nact;i++)
        fprintf(fid,"%f,",u[i]);
    for (i=0;i<nact;i++)
        fprintf(fid,"%f,",tau[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",robot_com[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",pos_FR_shoulder[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",pos_FL_shoulder[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",pos_RR_shoulder[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",pos_RL_shoulder[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",pos_FR_elbow[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",pos_FL_elbow[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",pos_RR_elbow[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",pos_RL_elbow[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",pos_FR_foot[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",pos_FL_foot[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",pos_RR_foot[i]);
    for (i=0;i<3;i++)
        fprintf(fid,"%f,",pos_RL_foot[i]);
    fprintf(fid,"\n");
}

// check the tau limits
void clamp_command(double tau[nact])
{
    int i;
    double cmd;
    for (i=0;i<nact;i++)
    {
        cmd = tau[i];

        if (cmd<trq_min)
        {
            cmd = trq_min;
        }

        else if (cmd>trq_max)
        {
            cmd = trq_max;
        }

        d->ctrl[act_id[i]] = cmd;
    }
}

// control loop callback
void loop(const mjModel* m, mjData* d)
{
    int i;
    double t = d->time;
    double tau[nact]={0};
    double q[nact]={0},u[nact]={0};
    double base_x[3]={0},base_quat[4]={0};
    double base_angvel[3]={0},base_linacc[3]={0},base_linvel[3]={0};
    double pos_FR_shoulder[3]={0},pos_FL_shoulder[3]={0},pos_RR_shoulder[3]={0},pos_RL_shoulder[3]={0};
    double pos_FR_elbow[3]={0},pos_FL_elbow[3]={0},pos_RR_elbow[3]={0},pos_RL_elbow[3]={0};
    double pos_FR_foot[3]={0},pos_FL_foot[3]={0},pos_RR_foot[3]={0},pos_RL_foot[3]={0};
    double robot_com[3]={0};

    /*
        trigger: B_liftoff
        state: state_4
        actions: A_hip_forward = 0.5
                 A_calf_retract= -1.7
                 B_hip_backward = 0.9
                 B_calf_short = -2.2

        trigger: A_toughdown
        state: state_1
        actions: A_hip_backward = 0.9
                 A_calf_extend = -1.4
                 B_hip_forward = 0.5
                 B_calf_short = -2.2

        trigger: A_liftoff
        state: state_2
        actions: A_hip_backward = 0.9
                 A_calf_short = -2.2
                 B_hip_forward = 0.5
                 B_calf_retract = -1.7
        
        trigger: B_toughdown
        state: state_3
        actions: A_hip_forward = 0.5
                 A_calf_short = -2.2
                 B_hip_backward = 0.9
                 B_calf_extend = -1.4
    */

    static int flag = 1;
    static int state = 4;

    get_state(q,u,base_x,base_quat,base_angvel,base_linacc,base_linvel,
              pos_FR_shoulder,pos_FL_shoulder,pos_RR_shoulder,pos_RL_shoulder,
              pos_FR_elbow,pos_FL_elbow,pos_RR_elbow,pos_RL_elbow,
              pos_FR_foot,pos_FL_foot,pos_RR_foot,pos_RL_foot,robot_com);

    if (state == 4 && pos_FL_foot[2] < 0.021 && pos_RR_foot[2] < 0.021)
    {
        flag = 2;
        state = 1;
    }

    if (state == 1 && pos_FL_foot[2] > 0.021 && pos_RR_foot[2] > 0.021)
    {
        flag = 3;
        state = 2;
    }

    if (state == 2 && pos_FR_foot[2] < 0.021 && pos_RL_foot[2] < 0.021)
    {
        flag = 4;
        state = 3;
    }

    if (state == 3 && pos_FR_foot[2] > 0.021 && pos_RL_foot[2] > 0.021)
    {
        flag = 1;
        state = 4;
    }

    control_loop(q,u,base_x,base_quat,base_angvel,base_linacc,base_linvel,
                 pos_FR_shoulder,pos_FL_shoulder,pos_RR_shoulder,pos_RL_shoulder,
                 pos_FR_elbow,pos_FL_elbow,pos_RR_elbow,pos_RL_elbow,
                 pos_FR_foot,pos_FL_foot,pos_RR_foot,pos_RL_foot,robot_com,tau,flag,state,t);
    
    clamp_command(tau);

    if (loop_index%data_frequency==0)
    {
        save_data(t,q,u,tau,base_x,base_quat,
                  base_angvel,base_linacc,base_linvel,
                  pos_FR_shoulder,pos_FL_shoulder,pos_RR_shoulder,pos_RL_shoulder,
                  pos_FR_elbow,pos_FL_elbow,pos_RR_elbow,pos_RL_elbow,
                  pos_FR_foot,pos_FL_foot,pos_RR_foot,pos_RL_foot,
                  robot_com);
    }
    
    loop_index = loop_index + 2;
}

//initialization release
void init_release()
{
    /*
    double quat[4] = {-0.9990, 0, 0.0436, 0};
    d->qpos[1] = 0.08;
    d->qpos[3] = quat[0];
    d->qpos[4] = quat[1];
    d->qpos[5] = quat[2];
    d->qpos[6] = quat[3];
    */
    m->opt.gravity[2] = 0;
}

// main function
int main(int argc, const char** argv)
{
    // check command-line arguments
    if( argc!=2 )
    {
        printf(" USAGE:  basic modelfile\n");
        return 0;
    }

    // active software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";
    if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
        m = mj_loadModel(argv[1], 0);
    else
        m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data 
    d = mj_makeData(m);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // install control callback and do the initialization
    get_ID();
    mjcb_control = loop;
    init_release();
    init_controller();
    fid = fopen(path,"w");
    init_save_data();

    // run main loop, target real-  simulation and 60 fps rendering
    while( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
            mj_step(m, d);

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model & data, and close data collection
    mj_deleteData(d);
    mj_deleteModel(m);
    fclose(fid);

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}