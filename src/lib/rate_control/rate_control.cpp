/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * @file RateControl.cpp
 */

#include "rate_control.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

void RateControl::setPidGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void RateControl::setLPFGains(const Vector3f &D_fc, const Vector3f &LPF_fc)
{
	_D_frq = D_fc;
	_rate_error_lpf_frq = LPF_fc;
}

void RateControl::GetCtrlTerm(Vector3f &P_term, Vector3f &I_term, Vector3f &D_term, Vector3f &FF_term)
{
	P_term = _rate_prop;
	I_term = _rate_int;
	D_term = _rate_diff;
	FF_term = _rate_ff;
}

void RateControl::resetFilters(const Vector3f &rate_err, const float dt)
{
	float sample_rate_hz = 1.0f/dt;
	for (int axis = 0; axis < 3; axis++) {
		// angular velocity low pass
		_lp_filter_rate_err[axis].set_cutoff_frequency(sample_rate_hz, _rate_error_lpf_frq(axis));
		_lp_filter_rate_err[axis].reset(rate_err(axis));
	}

	_reset_filters = false;
}

void RateControl::setSaturationStatus(const Vector3<bool> &saturation_positive,
				      const Vector3<bool> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

void RateControl::setPositiveSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_positive(axis) = is_saturated;
	}
}

void RateControl::setNegativeSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_negative(axis) = is_saturated;
	}
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	const bool dt_changed = (fabsf(dt - _dt_prev) / dt) > 0.1f;

	if (dt_changed) {
		_reset_filters = true;
	}
	_dt_prev  = dt;

	if (_reset_filters) {
		resetFilters(rate_error, dt);
		// if resetFilter success, then _reset_filters -> false;
		if (_reset_filters) {
			return Vector3f(0.0f, 0.0f, 0.0f);
		}
	}
	//update D term
	updateDifferential(rate_error, dt);
	//update P term
	_rate_prop = _gain_p.emult(rate_error);
	_rate_ff = _gain_ff.emult(rate_sp);

	// PID control with feed forward
	const Vector3f torque = _rate_ff + _rate_prop + _rate_int + _rate_diff;

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	_rate_error_prev = rate_error;

	return torque;
}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method, use tustin integral: 0.5*dt*(z+1)/(z-1)
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * (rate_error(i)+_rate_error_prev(i))*0.5f*dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void RateControl::updateDifferential(Vector3f &rate_error, const float dt)
{
	Vector3f rate_error_lpf, a, b;
	// lyu: do data filtering
	if (!_reset_filters) {
		for (int axis = 0; axis < 3; axis++) {
			rate_error_lpf(axis) = _lp_filter_rate_err[axis].apply(rate_error(axis));
			a(axis) = _gain_d(axis)*(M_TWOPI_F*_D_frq(axis))/(dt*M_PI_F*_D_frq(axis) + 1.0f);
			b(axis) = (dt*M_PI_F*_D_frq(axis) - 1.0f)/(dt*M_PI_F*_D_frq(axis) + 1.0f);
		}
	} else {
		rate_error_lpf = rate_error;
	}

	// lyu: add the D-term compensator
	if (!_reset_filters) {
		_rate_diff = a.emult(rate_error_lpf)-a.emult(_rate_error_lpf_prev)-b.emult(_rate_diff_prev);
	} else {
		_rate_diff.zero();
	}

	_rate_diff_prev = _rate_diff;
	_rate_error_lpf_prev = rate_error_lpf;
}


void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}
