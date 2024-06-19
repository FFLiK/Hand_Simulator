#pragma once
#include <Hand.h>
#include <Joint.h>
class Progress {
private:
	const double unit = 0.01;
	double progress;
	bool sign;
public:
	Progress() : progress(0), sign(false) {}
	double operator()() {
		return progress;
	}
	void operator++() {
		progress += unit * (sign ? -1 : 1);
		if (progress > 1) {
			progress = 1;
			sign = !sign;
		}
		if (progress < 0) {
			progress = 0;
			sign = !sign;
		}
	}
};

class HandGenerator {
public:
	static Hand* Generate();
};