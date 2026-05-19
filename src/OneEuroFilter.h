#pragma once

#include <cmath>

/// 1D One Euro filter for jitter-free tracking with low lag on fast motion.
class OneEuroFilter {
public:
	void setup(float minCutoff, float beta, float dCutoff = 1.0f) {
		this->minCutoff = minCutoff;
		this->beta = beta;
		this->dCutoff = dCutoff;
		reset();
	}

	/// Live tuning: update cutoff/beta without touching the running filter state.
	void setParams(float minCutoff, float beta) {
		this->minCutoff = minCutoff;
		this->beta = beta;
	}

	void reset(float value = 0.0f) {
		first = true;
		xPrev = value;
		dxPrev = 0.0f;
	}

	float filter(float value, float dt) {
		if (dt <= 0.0f) {
			return first ? value : xPrev;
		}
		if (first) {
			first = false;
			xPrev = value;
			dxPrev = 0.0f;
			return value;
		}

		const float dx = (value - xPrev) / dt;
		const float edx = lowPass(dx, dxPrev, alpha(dt, dCutoff));
		const float cutoff = minCutoff + beta * std::abs(edx);
		const float filtered = lowPass(value, xPrev, alpha(dt, cutoff));
		xPrev = filtered;
		dxPrev = edx;
		return filtered;
	}

private:
	static float lowPass(float x, float xPrev, float a) {
		return a * x + (1.0f - a) * xPrev;
	}

	static float alpha(float dt, float cutoff) {
		const float tau = 1.0f / (2.0f * static_cast<float>(M_PI) * cutoff);
		return 1.0f / (1.0f + tau / dt);
	}

	float minCutoff = 1.0f;
	float beta = 0.0f;
	float dCutoff = 1.0f;
	bool first = true;
	float xPrev = 0.0f;
	float dxPrev = 0.0f;
};
