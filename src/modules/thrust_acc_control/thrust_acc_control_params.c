/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file thrust_acc_control_params.c
 * Parameters for thrust_acc control
 *
 * @author WarriorHanamy rongerch@mail2.sysu.edu.cn
 */

/**
 * thrust accleration error P gain
 *
 * [at_sp - hat(at)] * K * (P+I*1/s+D*s) + thrust_ff = Thrust_setpoint
 *
 * @min 0.001
 * @max 0.008
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(THR_P, 0.005f);


/**
 * thrust setpoint low pass filter cutoff frequency 
 *
 * thrust setpoint low pass filter cutoff frequency 
 *
 * @min 0.00
 * @max 100.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(THR_LPF_CUTOFF, 50.0f);

/**
 * thrust curve over-confident SLOPE
 *
 * [at_sp - hat(at)] * K * (P+I*1/s+D*s) + thrust_ff = Thrust_setpoint
 *
 * @min 20.0
 * @max 200.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(THR_CUR_LIN_K, 150.0f);

/**
 * bound of delta_thrust on vehicle_angular_velocity coming
 *
 * bound of delta_thrust on vehicle_angular_velocity coming
 *
 * @min 0.0
 * @max 0.01
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(THR_DELTA_BOUND, 0.005f);


/**
 * acc-thrust model blend
 *
 * u = (1-beta) * u + beta * model_ff
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(THR_BETA, 0.0f);



/**
 * thrust contrl timeout acc when mpc is lost
 *
 * thrust contrl timeout acc when mpc is lost
 *
 * @min 9.0
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(THR_TMO_ACC, 9.81f);


/**
 * thrust contrl Max acc when mpc is running
 *
 * thrust contrl Max acc when mpc is running
 *
 * @min 1.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(THR_SFT_ACC, 3.0f);

/**
 * thrust contrl Max rate when mpc is running
 *
 * thrust contrl Max rate when mpc is running
 *
 * @min 1.5
 * @max 3.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(THR_SFT_RATE, 2.5f);


/**
 * thrust contrl timeout time when mpc is lost  [0.1s]
 *
 * thrust contrl timeout time when mpc is lost, for example,  5 is 0.5s.
 *
 * @min 1
 * @max 10
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(THR_TMO_TIME, 5);



/**
 * thrust contrl simulation mode
 *
 * thrust contrl simulation mode, 1 is enable, 0 is disable.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(THR_SIM, 1);


/**
 * Pitch torque linear gain K
 *
 * Pitch torque linear gain K
 *
 * @min 0.000
 * @max 0.3
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(PITCH_TOR_K, 0.1f);

/**
 * Pitch torque Boundary
 *
 * Pitch torque Boundary
 *
 * @min 0.000
 * @max 0.080
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(PITCH_TOR_BD, 0.03f);

/**
 * MRAC Reference Model Coefficient A (Pole)
 *
 * Defines the reference model dynamics: ẋₘ = -am·xₘ + bm·r
 * Higher values = faster response. Typically am = bm for unit gain.
 *
 * @min 1.0
 * @max 20.0
 * @decimal 2
 * @increment 0.1
 * @group Thrust Acc Control
 */
PARAM_DEFINE_FLOAT(THR_MRAC_REF_MODEL_AM, 5.0f);

/**
 * MRAC Reference Model Coefficient B (Gain)
 *
 * Defines the reference model input gain.
 * Typically bm = am for unit DC gain.
 *
 * @min 1.0
 * @max 20.0
 * @decimal 2
 * @increment 0.1
 * @group Thrust Acc Control
 */
PARAM_DEFINE_FLOAT(THR_MRAC_REF_MODEL_BM, 5.0f);

/**
 * MRAC Adaptation Gain for Kr (Feedforward)
 *
 * Learning rate for the feedforward adaptive parameter Kr.
 * Higher = faster adaptation but may cause oscillation.
 *
 * @min 0.001
 * @max 5.0
 * @decimal 4
 * @increment 0.001
 * @group Thrust Acc Control
 */
PARAM_DEFINE_FLOAT(THR_MRAC_GAMMA_KR, 0.1f);

/**
 * MRAC Adaptation Gain for Kx (Feedback)
 *
 * Learning rate for the feedback adaptive parameter Kx.
 * Higher = faster adaptation but may cause oscillation.
 *
 * @min 0.001
 * @max 5.0
 * @decimal 4
 * @increment 0.001
 * @group Thrust Acc Control
 */
PARAM_DEFINE_FLOAT(THR_MRAC_GAMMA_KX, 0.1f);

/**
 * MRAC Low-Pass Filter Cutoff Frequency
 *
 * Cutoff frequency for filtering adaptive parameters.
 * Reduces high-frequency oscillations in control output.
 *
 * @min 1.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @unit Hz
 * @group Thrust Acc Control
 */
PARAM_DEFINE_FLOAT(THR_MRAC_LPF_CUTOFF, 8.0f);

/**
 * MRAC Kr Maximum Value
 *
 * Upper bound for feedforward adaptive parameter.
 * Prevents excessive feedforward action.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group Thrust Acc Control
 */
PARAM_DEFINE_FLOAT(THR_MRAC_KR_MAX, 2.0f);

/**
 * MRAC Kr Minimum Value
 *
 * Lower bound for feedforward adaptive parameter.
 * Prevents negative feedforward (reverse thrust).
 *
 * @min 0.0
 * @max 0.5
 * @decimal 4
 * @increment 0.001
 * @group Thrust Acc Control
 */
PARAM_DEFINE_FLOAT(THR_MRAC_KR_MIN, 0.01f);

/**
 * MRAC Kx Maximum Value
 *
 * Upper bound for feedback adaptive parameter.
 *
 * @min 0.1
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Thrust Acc Control
 */
PARAM_DEFINE_FLOAT(THR_MRAC_KX_MAX, 1.0f);

/**
 * MRAC Kx Minimum Value
 *
 * Lower bound for feedback adaptive parameter.
 * Allows negative feedback for damping.
 *
 * @min -5.0
 * @max -0.1
 * @decimal 3
 * @increment 0.01
 * @group Thrust Acc Control
 */
PARAM_DEFINE_FLOAT(THR_MRAC_KX_MIN, -1.0f);