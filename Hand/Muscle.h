#pragma once
#include <vector>
#include <functional>
#include <Joint.h>
#include <Constant.h>
using namespace std;

class Muslce;
class UnitMuscle;

class Muscle {
private:
	vector<UnitMuscle> muscles;
	int num = 1;
	double strength = 1.0;

public:
	Muscle();
	~Muscle();

	Muscle* SetMuscleUnitSize(int num);
	Muscle* SetMuscleStrength(double strength);
	UnitMuscle* GetMuscleUnit(int index);

	void SetPower(double power);
	double GetPower();

	void RotatePoints(Eigen::Matrix3d rotation_matrix);

	void Compute();
	void Render(SDL_Renderer* renderer);
};

class UnitMuscle {
friend class Muscle;

private:
	class JointFunctionGroup {
	public:
		function<void(double)> Force;
		function<double()> Angle;

		function<Vector3D()> Point;
	};

	int contracting_angle_sum = 0;
	vector<JointFunctionGroup> joint_functions;

	double power;

	vector<Vector3D> muscle_visualizing_points;

public:
	UnitMuscle();
	~UnitMuscle();

	UnitMuscle* AddJoint(Joint<>* joint, double theta, double visualizing_distance);
	UnitMuscle* SetContractingAngleSummation(int sum);

	void RotatePoints(Eigen::Matrix3d rotation_matrix);

	void Compute();
	void Render(SDL_Renderer* renderer);
};