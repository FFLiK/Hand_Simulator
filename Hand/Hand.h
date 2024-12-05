#pragma once
#include <Joint.h>
#include <Muscle.h>
#include <vector>
#include <Window.h>
#include <functional>

enum GraphicMode {
	FRAME = 0,
	SPECIFIC_FRAME = 1,
	SOLID = 3,
	COORDINATE = 2
};

class SpecialMotionFunction {
private:
	Muscle* muscle;
	int index = 0;
	double power = 0.0;
	double power_unit = 0.05;
	std::string txt;

public:
	SpecialMotionFunction(Muscle* muscle, int index, std::string txt) {
		this->muscle = muscle;
		this->index = index;
		this->txt = txt;
	}

	void operator()() {
		this->muscle->SetPower(this->power, this->index);
		Log::Debug("[Manual Control]", txt, power);
		power += power_unit;
		if (power > 1.0) {
			power = 1.0;
			power_unit = -0.05;
		}
		else if (power < 0.0) {
			power = 0.0;
			power_unit = 0.05;
		}
	}
};

class Hand {
private:
	std::vector<Joint<>*> joints;
	std::vector<Muscle*> muscles;

	std::vector<pair<Joint<>*, Joint<>*> > additional_false_line;

	Vector3D orientation;

	vector<function<double()>*> result_function_set;

	std::vector<Vector3D> previous_joints_position_data;
	std::vector<Vector3D> current_joints_position_data;

	bool is_free_moving = false;
	atomic<bool> stop_external_call = false;
	atomic<bool> is_stop_safely = false;

	void ComputeInternally();

public:
	Hand();
	~Hand();

	Hand* AddJoint(Joint<>* joint);
	Hand* AddMuscle(Muscle* muscle);
	Hand* AddFalseLine(Joint<>* joint1, Joint<>* joint2);
	void Compute();
	void Render(SDL_Renderer* renderer, GraphicMode mode);

	std::vector<Joint<>*> GetJoints();

	void SetOrientation(double x, double y, double z);

	std::vector<function<void()>*> press_motion_function_set;
	std::vector<function<void()>*> release_motion_function_set;
	std::vector<SpecialMotionFunction*> motion_function_set;

	void GetCurrentPose(vector<Vector3D>& args);
	void SetFinalPose(vector<Vector3D> args);
	void Optimization();

	bool IsStable();

	void SetFreeMoving(bool is_free_moving);
	bool IsFreeMoving();

	double ObjectiveFunction();
};