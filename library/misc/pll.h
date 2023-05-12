#ifndef __PLL_H__
#define __PLL_H__

#include <stdlib.h>
#include <cmath>

class PLL {
public:

    PLL(float kp, float ki)
    {
        _kp = kp;
        _ki = ki;
        _pll_phase = 0.0f;
        _pll_spd   = 0.0f;
    }

    void    run(float phase, float dt);

    float   spd(void) const { return _pll_spd; }
    float   rad(void) const { return _pll_phase; }

private:
    float _pll_phase;
	float _pll_spd;
	float _kp;
	float _ki;
};

#endif