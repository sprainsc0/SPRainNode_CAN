#include "utils.h"
#include "pll.h"

void PLL::run(float phase, float dt)
{
	float delta_theta = wrap_PI(phase - _pll_phase);

	_pll_phase += (_pll_spd + _kp * delta_theta) * dt;
	_pll_phase = wrap_PI(_pll_phase);

	_pll_spd += _ki * delta_theta * dt;
}