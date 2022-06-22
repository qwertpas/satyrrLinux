// Global stored vars
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <math.h>

float xW_des_acc_prev;
float dxW_des_acc_prev;
float xW_prev_odom;
float yPos_prev;
float xPos_prev;
float z_prev;
float theta_lWheel_p;
float theta_rWheel_p;
float t_prev;
float t_offset;
float dxW_des_prev;
float xW_des_prev;
float dYaw_des_prev;
float yaw_des_prev;
float xW;
float thetaWheelPrev;
float prev_raw_angle[4];
float wrapped_angle[4];
float angleOffset[4];
float yaw_wheel_prev;
float Lpf_dyaw_wheels;
float pitchEst;
float pitch_vel;
float IMU_pitch_prev;
float Lpf_pitch;
float timeCompleted;

int loop_count = 0;
int i_rec = 0;


// struct{
//     uint32_t ESTOP;
//     uint32_t startControl;
//     uint32_t startTraj;
//     uint32_t zeroAngle;
//     uint32_t HMI_enabled;
//     uint32_t rampGain;
//     uint32_t rampGainWheel;
//     uint32_t roamingMode;
// } flagsIn;

float sign(float num){
    if(num>=0) return 1;
    else return -1;
}



void update(float* from_robo, float* to_robo, float t_sec) {

    loop_count++;


    // ************************************************ SYSTEM PARAMETERS*******************************//
    float TAU_MIN = -17;
    float TAU_MAX = 17;
    float PI_ = 3.14159;
    float R_wheel = .06;
    int RLEG = 0;
    int LLEG = 1;
    float L_leg = .15;
    float L_torso = .4;
    float wh = .212; // distance between two wheels

    float add_grav = 1.8;
    float mR = 7.54 + add_grav; // .9 is for weight of each leg
    float g = 9.81;

    float P_MAX_V1 = 12.5663706144;
    float V_MAX_V1 = 30;

    float dt = 250 / 1000000; // Approximate dt and fix for filter

    float hmi_deltaXArr[2] = {0, 0};
    float hmi_deltaYawRaw = 0;

    float t_ms_exp = 0;

    float tau_lHip = 0;
    float tau_rHip = 0;
    float tau_wheel = 0;

    float motor1 = 0;
    float motor2 = 0;
    float motor8 = 0;
    float motor9 = 0;

    //replace ???
    t_sec = loop_count * dt;



    // ************************************************ INPUTS FROM LABVIEW *******//
    //memcpy(&from_robo, &flagsIn, sizeof(flagsIn));
    float ESTOP = from_robo[0];
    float startControl = from_robo[1];
    float startTraj = from_robo[2];
    float zeroAngle = from_robo[3];
    float HMI_enabled = from_robo[4];
    float rampGain = from_robo[5];
    float rampGainWheel = from_robo[6];
    float roamingMode = from_robo[7];
    
    float actual_angles[4];
    float actual_vel_angles[4];
    for(int i=0; i < 4; i++){
        actual_angles[i] = from_robo[i+8];
        actual_vel_angles[i] = from_robo[i+12];
    }

    float IMU_pitch = from_robo[16];
    float dIMU_pitch = from_robo[17];
    float IMU_yaw = from_robo[18];
    float dIMU_yaw = from_robo[19];
    float IMU_roll = from_robo[20];
    float dIMU_roll = from_robo[21];

    // ************************************************ ANGLE ZEROING************************************//
    float angleExp[4];
    angleExp[0] = 0.0f;
    angleExp[1] = -62.12f * PI_ / 180; // From CAD
    angleExp[2] = 62.12f * PI_ / 180;  // From CAD
    angleExp[3] = 0.0f;

    if (zeroAngle == 1)
    {
        int j = 0;
        for (j = 0; j < 4; j++)
        {
            angleOffset[j] = angleExp[j] - actual_angles[j];
        }
        angleOffset[0] = -wrapped_angle[RLEG];
        angleOffset[3] = -wrapped_angle[LLEG];
    }
    float random;

    // ************************************************JOINT ANGLE CONVERSEIONS*******************************//
    float delta_theta[2];
    delta_theta[RLEG] = actual_angles[0] - prev_raw_angle[RLEG];
    delta_theta[LLEG] = actual_angles[3] - prev_raw_angle[LLEG];

    // Check wrap around for right wheel
    if (delta_theta[RLEG] > P_MAX_V1)
    {
        delta_theta[RLEG] = (-P_MAX_V1 - prev_raw_angle[RLEG]) + (actual_angles[0] - P_MAX_V1);
    }
    else if (delta_theta[RLEG] < -P_MAX_V1)
    {
        delta_theta[RLEG] = (P_MAX_V1 - prev_raw_angle[RLEG]) + (actual_angles[0] + P_MAX_V1);
    }
    wrapped_angle[RLEG] += delta_theta[RLEG];
    prev_raw_angle[RLEG] = actual_angles[0];

    // Check wrap around for right wheel
    if (delta_theta[LLEG] > P_MAX_V1)
    {
        delta_theta[LLEG] = (-P_MAX_V1 - prev_raw_angle[LLEG]) + (actual_angles[3] - P_MAX_V1);
    }
    else if (delta_theta[LLEG] < -P_MAX_V1)
    {
        delta_theta[LLEG] = (P_MAX_V1 - prev_raw_angle[LLEG]) + (actual_angles[3] + P_MAX_V1);
    }
    wrapped_angle[LLEG] += delta_theta[LLEG];
    prev_raw_angle[LLEG] = actual_angles[3];

    // Apply offsets and angle flipping
    float theta_rWheel = (wrapped_angle[RLEG] + angleOffset[0]);
    float theta_rHip = -(actual_angles[1] + angleOffset[1]);
    float theta_lHip = (actual_angles[2] + angleOffset[2]);
    float theta_lWheel = -(wrapped_angle[LLEG] + angleOffset[3]);

    float dtheta_rWheel = actual_vel_angles[0];
    float dtheta_rHip = -actual_vel_angles[1];
    float dtheta_lHip = actual_vel_angles[2]; // Flipping angles so they match convention of right leg
    float dtheta_lWheel = -actual_vel_angles[3];

    // Computed vel from position (UNUSED)
    // float dtheta_rWheel = (theta_rWheel - theta_rWheel_p) / ((t_ms - t_ms_prev)/1000);
    // float dtheta_lWheel = (theta_lWheel - theta_lWheel_p) / ((t_ms - t_ms_prev)/1000);
    theta_rWheel_p = theta_rWheel;
    theta_lWheel_p = theta_lWheel;

    // IMU angles deg -> radians conversion
    IMU_pitch = IMU_pitch * (PI_ / 180);
    float dIMU_pitch_rad = dIMU_pitch * (PI_ / 180);

    IMU_roll = IMU_roll * (PI_ / 180);
    dIMU_roll = dIMU_roll * (PI_ / 180);

    IMU_yaw = IMU_yaw * (PI_ / 180);
    dIMU_yaw = dIMU_yaw * (PI_ / 180);

    // Full Model -> Reduced Model state space
    float thetaHip = (theta_rHip + theta_lHip) / 2;
    float dthetaHip = (dtheta_rHip + dtheta_lHip) / 2;
    float thetaWheel = (theta_rWheel + theta_lWheel) / 2;    // - thetaHip + IMU_pitch; //Pos or neg thetaHip
    float dthetaWheel = (dtheta_rWheel + dtheta_lWheel) / 2; // - dthetaHip + dIMU_pitch_rad;
    float delta_thetaWheel = thetaWheel - thetaWheelPrev;

    xW = xW + R_wheel * (delta_thetaWheel);
    float dxW = R_wheel * (dthetaWheel);
    float L_h = 2 * L_leg * cos(thetaHip);
    float dL_h = -2 * L_leg * sin(thetaHip) * dthetaHip;
    thetaWheelPrev = thetaWheel;

    //************************************* PITCH ESTIMATION ******************************************//
    // Interpolation
    // pitch_vel := computed slope of pitch data
    // track pitchEst, pitch_vel, i_rec, IMU_pitch_prev

    if (IMU_pitch == IMU_pitch_prev)
    {
        pitchEst = pitchEst + (pitch_vel * dt);
    }
    else
    {
        float t_imu_elap = (dt * (loop_count - i_rec));
        if (t_imu_elap == 0)
        {
            pitch_vel = 0;
        }
        else
        {
            pitch_vel = ((IMU_pitch - IMU_pitch_prev) / t_imu_elap);
        }

        pitchEst = IMU_pitch; // update the estimated
        IMU_pitch_prev = IMU_pitch;
        i_rec = loop_count;
    }

    // Low-pass filter the extrapolation
    if (abs(IMU_pitch * 180 / 3.141592) < 180)
    {
        float lpf_alpha = 0.7;
        Lpf_pitch = lpf_alpha * Lpf_pitch + (1 - lpf_alpha) * pitchEst;
    }

    // Set estimated & filtered pitch to pitch used below
    IMU_pitch = Lpf_pitch;

    // ************************************************GRAPHING ROBOT LINKAGE*******************************//

    // Calculate robot graph position
    float x_R[4];
    float z_R[4];

    float wheel_pos[3];
    float knee_pos[3];
    float hip_pos[3];
    float torso_pos[3];
    float CoM[2];
    float CoM_R[2];
    float CoM_L[2];

    wheel_pos[0] = 0;
    wheel_pos[1] = 0;
    wheel_pos[2] = 0;

    knee_pos[0] = L_leg * sin(thetaHip + IMU_pitch);
    knee_pos[1] = 0;
    knee_pos[2] = L_leg * cos(thetaHip + IMU_pitch);
    hip_pos[0] = L_leg * sin(thetaHip + IMU_pitch) - L_leg * sin(thetaHip - IMU_pitch);
    hip_pos[1] = 0;
    hip_pos[2] = L_leg * cos(thetaHip + IMU_pitch) + L_leg * cos(thetaHip - IMU_pitch);
    torso_pos[0] = L_leg * sin(thetaHip + IMU_pitch) - L_leg * sin(thetaHip - IMU_pitch) + L_torso * sin(IMU_pitch);
    ;
    torso_pos[1] = 0;
    torso_pos[2] = L_leg * cos(thetaHip + IMU_pitch) + L_leg * cos(thetaHip - IMU_pitch) + L_torso * cos(IMU_pitch);

    x_R[0] = wheel_pos[0];
    x_R[1] = knee_pos[0];
    x_R[2] = hip_pos[0];
    x_R[3] = torso_pos[0];

    z_R[0] = wheel_pos[2];
    z_R[1] = knee_pos[2];
    z_R[2] = hip_pos[2];
    z_R[3] = torso_pos[2];

    // CoM[0] = -.01884*cos(IMU_pitch) + .07329*sin(IMU_pitch) - L_leg*sin(thetaHip - IMU_pitch) + L_leg*sin(thetaHip + IMU_pitch);
    // CoM[1] = .07329*cos(IMU_pitch) + .01884*sin(IMU_pitch) + L_leg*cos(thetaHip+ IMU_pitch) + L_leg*cos(thetaHip - IMU_pitch);

    CoM_R[0] = -.01884 * cos(IMU_pitch) + .07329 * sin(IMU_pitch) - L_leg * sin(theta_rHip - IMU_pitch) + L_leg * sin(theta_rHip + IMU_pitch);
    CoM_R[1] = .07329 * cos(IMU_pitch) + .01884 * sin(IMU_pitch) + L_leg * cos(theta_rHip + IMU_pitch) + L_leg * cos(theta_rHip - IMU_pitch);

    CoM_L[0] = -.01884 * cos(IMU_pitch) + .07329 * sin(IMU_pitch) - L_leg * sin(theta_lHip - IMU_pitch) + L_leg * sin(theta_lHip + IMU_pitch);
    CoM_L[1] = .07329 * cos(IMU_pitch) + .01884 * sin(IMU_pitch) + L_leg * cos(theta_lHip + IMU_pitch) + L_leg * cos(theta_lHip - IMU_pitch);

    float alpha = -0.017;
    CoM[0] = (CoM_R[0] + CoM_L[0]) / 2 + alpha;
    CoM[1] = (CoM_R[1] + CoM_L[1]) / 2;

    float pitch_actual = atan2(CoM[0], CoM[1]); // I already incorporated imu pitch into my model

    //********************************************* YAW ESTIMATION ********************************************//
    // Compute robot yaw from wheel odometry
    float yaw_wheels = (R_wheel / wh) * (theta_rWheel - theta_lWheel);
    float dyaw_wheels = (yaw_wheels - yaw_wheel_prev) / dt;
    yaw_wheel_prev = yaw_wheels;

    // Low-pass filter dYaw
    float dyaw_alpha = 0.5; // lpf constant
    Lpf_dyaw_wheels = (dyaw_alpha * Lpf_dyaw_wheels) + ((1 - dyaw_alpha) * dyaw_wheels);

    // **************************************** TRAJECTORY GENERATION **********************************//
    float filterVec[5];
    float RC = .25;
    float RC_yaw = .025;
    float RC_stand = 1.5;

    float RC_coeff = dt / (RC + dt);
    float RC_coeff_yaw = dt / (RC_yaw + dt);
    float RC_coeff_stand = dt / (RC_stand + dt);
    float dYaw_des = 0; // filtered angular velocity
    float vel_des_traj = 0;
    float theta_des = 0;
    float ddxW_des = 0;
    float dxW_des = 0;      // filtered velocity
    float dxW_des_traj = 0; // filtered velocity
    float xW_des = 0;
    float yaw_des = 0;
    float pitch_des = 0;
    float dpitch_des = 0;
    float ddtheta_des = 0; // Joystick differentiated acceleration
    float dxW_des_acc = 0;
    float xW_des_acc = 0;

    // Analog Joystick Mapping Parameters
    float joyMax = 2.6;
    float joyMin = -2.6;
    float joyDead = .05; // Deadband for joystick
    // Velocity mapping max
    float velMax_J = 0.40;  // in m/s
    float velMax2_J = .675; // in m/s //try .85
    float yawMax_J = 1.15;  // in rad/s (was 1.05)
    float yawMax2_J = 1.35; // in rad/s (was 1.30)
    // Acceleration mapping theta_max
    float thetaMax_J = 1.75 * (PI_ / 180); // in rad
    float thetaMax2_J = thetaMax_J;

    // Switch-points
    float joyYawSwitch = 1.0;  // Analog voltage to switch piecewise linearity
    float joyVelSwitch = 1.25; // Analog voltage to switch piecewise linearity

    float m1J = yawMax_J / (joyMax - joyDead);                      // slope of the first piece-wise func.
    float yYawSwitch = m1J * (joyYawSwitch - joyDead);              // switchpoint value for yaw
    float m2J = (yawMax2_J - yYawSwitch) / (joyMax - joyYawSwitch); // slope of the second piece-wise func.

    float m3J = velMax_J / (joyMax - joyDead);                      // slope of the first piece-wise func.
    float yVelSwitch = m3J * (joyVelSwitch - joyDead);              // switchpoint value for yaw
    float m4J = (velMax2_J - yVelSwitch) / (joyMax - joyVelSwitch); // slope of the second piece-wise func.

    // HMI Mapping Parameters
    float copMax = .45;
    float copMin = -.45;
    float copDead = .009;

    float comMax = .125;
    float comMin = -.125;
    float comDead = .008;

    // HMI velocity mapping max
    float velMax_H = 1.35;     // in m/s 1.35
    float velMax2_H = 1.35;    // in m/s 1.525
    float yawMax_H = 1.45125;  // in rad/s
    float yawMax2_H = 1.45125; // in rad/s

    float copSwitch = .35;
    float comSwitch = .07;

    float m1 = yawMax_H / (copMax - copDead);
    float ySwitch = m1 * (copSwitch - copDead);
    float m2 = (yawMax2_H - ySwitch) / (copMax - copSwitch);

    float m3 = velMax_H / (comMax - comDead);
    float xSwitch = m3 * (comSwitch - comDead);
    float m4 = (velMax2_H - xSwitch) / (comMax - comSwitch);

    float hmi_deltaXRaw = hmi_deltaXArr[0];
    float hmi_ddXRaw = hmi_deltaXArr[1];
    // hmi_deltaYawRaw = hmi_deltaXArr[1]; // for cop mapping

    float hmi_deltaX = 0; // represents either des_dx or des_theta. It is ultimately the joystick displacement scaled
    float hmi_deltaYaw = 0;
    float z_des = .30 - .02; // required to place here for hip joint controller
    float z_filt = z_des;

    if (startTraj == 1)
    {
        float t_elap = t_sec - t_offset;

        // Joystick Mapping
        if (HMI_enabled == 0)
        {

            // // Piecewise linear mapping function for analog joystick -> Yaw_des
            float hmi_resX = hmi_deltaXRaw - joyMax;     // Shift [0, 5] -> [-2.6, 2.6]
            float hmi_resYaw = hmi_deltaYawRaw - joyMax; // Shift [0, 5] -> [-2.6, 2.6]

            if (abs(hmi_resYaw) < joyDead)
            {
                hmi_deltaYaw = 0;
            }
            else if (abs(hmi_resYaw) < joyYawSwitch)
            {
                hmi_deltaYaw = -sign(hmi_resYaw) * (m1J * (abs(hmi_resYaw) - joyDead));
            }
            else if (abs(hmi_resYaw) < joyMax)
            {
                hmi_deltaYaw = -sign(hmi_resYaw) * (m2J * (abs(hmi_resYaw) - joyYawSwitch) + yYawSwitch);
            }
            else
            {
                hmi_deltaYaw = -sign(hmi_resYaw) * yawMax2_J;
            }

            // Piecewise linear mapping function for analog joystick -> dxW_des (velocity mapping)
            //  if(abs(hmi_resX) < joyDead){
            //      hmi_deltaX = 0;
            //  }
            //  else if(abs(hmi_resX) < joyVelSwitch){
            //      hmi_deltaX = sign(hmi_resX)*(m3J*(abs(hmi_resX)-joyDead));
            //  }
            //  else if (abs(hmi_resX) < joyMax)
            //  {
            //      hmi_deltaX = sign(hmi_resX)*(m4J*(abs(hmi_resX) - joyVelSwitch) + yVelSwitch);
            //  }
            //  else{
            //      hmi_deltaX = sign(hmi_resX)*velMax2_J;
            //  }

            // Acceleration mapping analog joystick -> ddxW_des (acceleration mapping)
            if (abs(hmi_resX) < joyDead)
            {
                hmi_deltaX = 0;
            }
            else
            {
                // Perform the piece-wise mapping to desired theta
                hmi_deltaX = sign(hmi_resX) * (thetaMax_J / joyMax) * (abs(hmi_resX) - joyDead); // Linear mapping [-2.6, 2.6] --> [-.172, .172]

                // Find the ddtheta
                ddtheta_des = hmi_ddXRaw * (thetaMax_J / joyMax); // Low pass filtered ddtheta_des map

                // Find desired acceleration via cart-pole dynamics
                float L_pend = .35329;
                float I0 = .19732;
                ddxW_des = (-(mR * L_pend * L_pend) * ddtheta_des + mR * g * L_pend * hmi_deltaX) / (mR * L_pend);
                // ddxW_des = 0 + mR*g*L_pend*hmi_deltaX;
            }
            dxW_des_acc = dxW_des_acc_prev + ddxW_des * (t_elap - t_prev); // Note that the low pass filter below integrates this velocity for position
            xW_des_acc = xW_des_acc_prev + dxW_des_acc * (t_elap - t_prev) + (1 / 2) * ddxW_des * (t_elap - t_prev) * (t_elap - t_prev);
            dxW_des_acc_prev = dxW_des_acc;
            xW_des_acc_prev = xW_des_acc;
        }

        // HMI Mapping
        if (HMI_enabled == 1)
        {

            // hmi_deltaYawRaw = (.5/copMax)*hmi_deltaYawRaw ; //  map between [-.5, .5]
            // hmi_deltaXRaw = 2*(.5/comMax)*hmi_deltaXRaw; // map between [-1,1] : The  times 2 takes it from .5 to 1

            // Piecewise linear mapping function for HMI CoP  -> Yaw_des
            if (abs(hmi_deltaYawRaw) < copDead)
            {
                hmi_deltaYaw = 0;
            }
            else if (abs(hmi_deltaYawRaw) < copSwitch)
            {
                hmi_deltaYaw = sign(hmi_deltaYawRaw) * (m1 * (abs(hmi_deltaYawRaw) - copDead));
            }
            else if (abs(hmi_deltaYawRaw) < copMax)
            {
                hmi_deltaYaw = sign(hmi_deltaYawRaw) * (m2 * (abs(hmi_deltaYawRaw) - copSwitch) + ySwitch);
            }
            else
            {
                hmi_deltaYaw = sign(hmi_deltaYawRaw) * yawMax2_H;
            }

            // // Piecewise linear mapping function for HMI CoM  -> dxW_des
            if (abs(hmi_deltaXRaw) < comDead)
            {
                hmi_deltaX = 0;
            }
            else if (abs(hmi_deltaXRaw) < comSwitch)
            {
                hmi_deltaX = sign(hmi_deltaXRaw) * (m3 * (abs(hmi_deltaXRaw) - comDead));
            }
            else if (abs(hmi_deltaXRaw) < comMax)
            {
                hmi_deltaX = sign(hmi_deltaXRaw) * (m4 * (abs(hmi_deltaXRaw) - comSwitch) + xSwitch);
            }
            else
            {
                hmi_deltaX = sign(hmi_deltaXRaw) * velMax2_H;
            }

            // Acceleration mapping HMI -> ddxW_des (acceleration mapping)
            //  if(abs(hmi_deltaXRaw) < comDead){
            //      hmi_deltaX = 0;
            //  }
            //  else{
            //      // Perform the piece-wise mapping to desired theta
            //      hmi_deltaX = sign(hmi_deltaXRaw)*(thetaMax_J/comMax)*(abs(hmi_deltaXRaw) - comDead); // Linear mapping [-2.6, 2.6] --> [-.172, .172]

            //     //Find the ddtheta
            //     ddtheta_des = hmi_ddXRaw*(thetaMax_J/comMax); // Low pass filtered ddtheta_des map

            //     //Find desired acceleration via cart-pole dynamics
            //     float L_pend = .35329;
            //     float I0 = .19732;
            //     ddxW_des = ( -(mR*L_pend*L_pend)*ddtheta_des + mR*g*L_pend*hmi_deltaX)/(mR*L_pend);
            //     //ddxW_des = 0 + mR*g*L_pend*hmi_deltaX;
            // }
            //  dxW_des_acc = dxW_des_acc_prev + ddxW_des * (t_elap - t_prev);  // Note that the low pass filter below integrates this velocity for position
            //  xW_des_acc =  xW_des_acc_prev + dxW_des_acc*(t_elap - t_prev) + (1/2)*ddxW_des*(t_elap - t_prev)*(t_elap - t_prev);
            //  dxW_des_acc_prev = dxW_des_acc;
            //  xW_des_acc_prev = xW_des_acc;
        }

        // Mapping dependant selection
        vel_des_traj = hmi_deltaX; // velocity mapping
                                // vel_des_traj = dxW_des_acc; // acceleration mapping
        // theta_des  = hmi_deltaX; // acceleration mapping

        // Set remaining desired variables for LQR below
        // pitch_des =  theta_des; // acceleration mapping here can be dangerous
        // dpitch_des  =  dIMU_pitch_rad; // acceleration mapping here can be dangerous
        dpitch_des = 0;

        // Heuristic: limit turning radius while driving faster
        // hmi_deltaYaw = hmi_deltaYaw - sign(hmi_deltaYaw) * (.10 *abs(vel_des_traj));

        // Low pass filter velocity & integrate for position
        dxW_des = dxW_des_prev + RC_coeff * (vel_des_traj - dxW_des_prev);
        xW_des = xW_des_prev + dxW_des * (t_elap - t_prev);
        dxW_des_prev = dxW_des;
        xW_des_prev = xW_des;

        // Low pass filter yaw vel & integrate for yaw pos
        dYaw_des = dYaw_des_prev + RC_coeff_yaw * (hmi_deltaYaw - dYaw_des_prev);
        yaw_des = yaw_des_prev + dYaw_des * (t_elap - t_prev);
        dYaw_des_prev = dYaw_des;
        yaw_des_prev = yaw_des;

        t_prev = t_elap;
    }

    else
    {
        t_offset = t_sec;

        // Set desired states
        if (roamingMode == 0)
        {
            xW_des = 0;
            dxW_des = 0;
            yaw_des = 0;
            dYaw_des = 0;
        }
        else if (roamingMode == 1)
        {
            xW_des = xW;
            dxW_des = dxW;
            yaw_des = yaw_wheels;
            dYaw_des = Lpf_dyaw_wheels;
        }

        // Set previos to current values
        xW_des_prev = xW;
        dxW_des_prev = dxW;
        yaw_des_prev = yaw_wheels;
        dYaw_des_prev = dYaw_des;
        dxW_des_acc = dxW_des_acc_prev;
        xW_des_acc = xW_des_acc_prev;
    }

    // *************************** INVERTED PEND.  STABILIZATION CONTROLLERS ***************************//
    // Balancing Controller
    float K_xW = -70; // was -90 for vel mapping and -180 for acc
    float K_pitch = -640;
    float K_dxW = -70; // was -70 for vel mapping and -120 for acc
    float K_dpitch = -70;

    // float K_xW = K_lqr[0];
    // float K_pitch = K_lqr[1];
    // float K_dxW = K_lqr[2];
    // float K_dpitch = K_lqr[3];

    // Velocity mapping force gen
    float FxR = K_xW * (xW_des - xW) + K_dxW * (dxW_des - dxW) + K_pitch * (0 - pitch_actual) + K_dpitch * (0 - dIMU_pitch_rad);

    // Acceleration mapping force gen (w/o lpf)
    // float FxR = K_xW *(xW_des_acc - xW) + K_dxW*(dxW_des_acc - dxW) + K_pitch*(pitch_des - pitch_actual) + K_dpitch*(dpitch_des - dIMU_pitch_rad);

    tau_wheel = (FxR * R_wheel) / 2;

    // **************************************** YAW CONTROLLER  ************************************************//
    float Kp_yaw = 1;
    float Kd_yaw = .1;

    // Yaw PD Controller
    float tau_yaw = Kp_yaw * (yaw_des - yaw_wheels) + Kd_yaw * (dYaw_des - Lpf_dyaw_wheels);
    // float tau_yaw = Kp_yaw *(yaw_des - IMU_yaw) + Kd_yaw * (dYaw_des - dIMU_yaw);
    float tau_wheel_R = tau_wheel + tau_yaw;
    float tau_wheel_L = tau_wheel - tau_yaw;

    // Dead-band reduction at low torques and velocities
    float K_db = .1;
    tau_wheel_R = tau_wheel_R + K_db * sign(dIMU_pitch_rad);
    tau_wheel_L = tau_wheel_L + K_db * sign(dIMU_pitch_rad);

    // ************************************************ ODOM RECONSTRUCTION ****************************//
    float lowerBound = -.75;
    float upperBound = .75;
    float xPos[1];
    float yPos[1];
    xPos[0] = (xW - xW_prev_odom) * cos(yaw_wheels) + xPos_prev;
    yPos[0] = (xW - xW_prev_odom) * sin(yaw_wheels) + yPos_prev;
    xW_prev_odom = xW;
    xPos_prev = xPos[0];
    yPos_prev = yPos[0];

    float upperLine_x[2];
    float upperLine_y[2];
    float lowerLine_x[2];
    float lowerLine_y[2];

    float target_pos_x = 4;
    float target_pos_y = 0;

    // Target Box
    // float target_box_x[5];
    // float target_box_y[5];
    // float box_half_length = .125;
    // target_box_x[0] = target_pos_x - box_half_length;
    // target_box_x[1] = target_pos_x - box_half_length;
    // target_box_x[2] = target_pos_x + box_half_length;
    // target_box_x[3] = target_pos_x + box_half_length;
    // target_box_x[4] =  target_pos_x - box_half_length;

    // target_box_y[0] = target_pos_y + box_half_length;
    // target_box_y[1] = target_pos_y - box_half_length;
    // target_box_y[2] =  target_pos_y - box_half_length;
    // target_box_y[3] =  target_pos_y + box_half_length;
    // target_box_y[4] = target_pos_y + box_half_length;

    // Target Line
    float target_box_x[2];
    float target_box_y[2];
    float box_half_length = .5;
    target_box_x[0] = target_pos_x;
    target_box_x[1] = target_pos_x;

    target_box_y[0] = target_pos_y + box_half_length;
    target_box_y[1] = target_pos_y - box_half_length;

    upperLine_x[0] = 0;
    upperLine_y[0] = upperBound;
    upperLine_x[1] = target_pos_x + .5;
    upperLine_y[1] = upperBound;

    lowerLine_x[0] = 0;
    lowerLine_y[0] = lowerBound;
    lowerLine_x[1] = target_pos_x + .5;
    lowerLine_y[1] = lowerBound;

    if ((xPos[0] >= 4.0) && (timeCompleted == 0))
    {
        timeCompleted = t_ms_exp / 1000;
    }

    // ************************************************ DATA LOGGING ****************************//
    int count = 0;
    float omega = 1;
    int startLog = 0;
    float volMax = 5.0;
    float stateVec[30];

    // Time since experiment started
    stateVec[0] = t_ms_exp / 1000;

    // LQR Variables
    stateVec[1] = xW;
    stateVec[2] = dxW; // mapping to analog signal for logging elsewhere
    stateVec[3] = pitch_actual;
    stateVec[4] = dIMU_pitch;

    stateVec[5] = xW_des;
    stateVec[6] = dxW_des;
    stateVec[7] = pitch_des;
    stateVec[8] = dpitch_des;

    stateVec[9] = FxR * R_wheel; // Computed LQR torque

    // Yaw Controller Variables
    stateVec[10] = yaw_wheels;      // robot yaw position
    stateVec[11] = Lpf_dyaw_wheels; // robot yaw velocity

    stateVec[12] = yaw_des; //
    stateVec[13] = dYaw_des;

    // Wheel Torques
    stateVec[14] = tau_wheel_R;
    stateVec[15] = tau_wheel_L;

    // Raw values from HMI
    stateVec[16] = hmi_deltaXRaw;
    stateVec[17] = hmi_deltaYawRaw;

    // IMU Values
    stateVec[18] = IMU_pitch;
    stateVec[19] = IMU_yaw;
    stateVec[20] = dIMU_yaw;

    // Odometry Reconstruction
    stateVec[21] = xPos[0];
    stateVec[22] = yPos[0];

    // Mapping gains
    stateVec[23] = velMax_H;
    stateVec[24] = velMax2_H;
    stateVec[25] = comSwitch;
    stateVec[26] = yawMax_H;
    stateVec[27] = yawMax2_H;
    stateVec[28] = copSwitch;
    stateVec[29] = thetaMax_J;

    // float stateVec[12];
    // stateVec[0] = t_ms;
    // stateVec[1] = xW_des;
    // stateVec[2] = dxW_des; // mapping to analog signal for logging elsewhere
    // stateVec[3] = xW;
    // stateVec[4] = dxW;
    // stateVec[5] = pitch_actual;
    // stateVec[6] = dIMU_pitch;
    // stateVec[7] = theta_rWheel; // mapping to analog signal for logging elsewhere
    // stateVec[8] = theta_lWheel;
    // stateVec[9] = tau_wheel_R;
    // stateVec[10] = tau_wheel_L;
    // stateVec[11] = Lpf_pitch;

    // ************************************************ JOINT LEVEL CONTROLLER *******************************//
    float thetaDes = acos(z_filt / (2 * L_leg)); // about 26.12 deg

    float Kp_r = 100;
    float Kp_l = 100;
    float Kd_r = 1;
    float Kd_l = 1;

    // Feedforward gravity compensation
    float J_rleg = 2 * L_leg * sin(theta_rHip);
    float J_lleg = 2 * L_leg * sin(theta_lHip);
    float tau_rGrav = J_rleg * (mR * g / 2);
    float tau_lGrav = J_lleg * (mR * g / 2);
    float tau_rJoint = Kp_r * ((theta_rHip)-thetaDes) + Kd_r * ((dtheta_rHip)-0);
    float tau_lJoint = Kp_l * ((theta_lHip)-thetaDes) + Kd_l * ((dtheta_lHip)-0);

    tau_rHip = tau_rJoint + tau_rGrav;
    tau_lHip = tau_lJoint + tau_lGrav;

    // ************************************************ APPLY TORQUES *******************************//
    // Motor Order: 2 ->8 -> 9 -> 1
    // right wheel -> right knee -> left knee -> left wheel

    if (tau_wheel > 7)
    {
        tau_wheel = 7;
    }
    else if (tau_wheel < -7)
    {
        tau_wheel = -7;
    }

    if (tau_rHip > TAU_MAX)
    {
        tau_rHip = TAU_MAX;
    }
    else if (tau_rHip < TAU_MIN)
    {
        tau_rHip = TAU_MIN;
    }

    if (tau_lHip > TAU_MAX)
    {
        tau_lHip = TAU_MAX;
    }
    else if (tau_lHip < TAU_MIN)
    {
        tau_lHip = TAU_MIN;
    }

    if (startControl == 1)
    {
        motor2 = rampGainWheel * tau_wheel_R;
        motor8 = rampGain * tau_rHip;
        motor9 = -rampGain * tau_lHip;         // Flip signs to go back to left leg convention
        motor1 = -rampGainWheel * tau_wheel_L; // Flip signed to go back to left lef convention
    }

    if ((ESTOP == 1) || (startControl == 0))
    {
        motor2 = 0.0;
        motor8 = 0.0;
        motor9 = 0.0;
        motor1 = 0.0;
    }

    to_robo[0] = motor1;
    to_robo[1] = motor2;
    to_robo[2] = motor9;
    to_robo[3] = motor8;

    to_robo[4] = tau_wheel;
    to_robo[5] = tau_rHip;
    to_robo[6] = tau_lHip;
}