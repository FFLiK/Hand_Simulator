#include "Muscle.h"
#include "Hand.h"
#include "RenderingTool.h"

Muscle::Muscle() {
	this->num = 0;
}

Muscle::~Muscle() {
}

Muscle* Muscle::SetMuscleUnitSize(int num) {
	this->num = num;
	this->muscles.resize(num);
	return this;
}

Muscle* Muscle::SetMuscleStrength(double strength) {
	this->strength = strength;
	return this;
}

UnitMuscle* Muscle::GetMuscleUnit(int index) {
	return &(this->muscles[index]);
}

void Muscle::SetPower(double power) {
	for (int i = 0; i < this->num; i++) {
		this->muscles[i].power = power * this->strength;
	}
}

double Muscle::GetPower() {
	double power = 0;
	for (int i = 0; i < this->num; i++) {
		power += this->muscles[i].power;
	}
	return power / this->num;
}

void Muscle::RotatePoints(Eigen::Matrix3d rotation_matrix) {
	for (int i = 0; i < this->num; i++) {
		this->muscles[i].RotatePoints(rotation_matrix);
	}
}

void Muscle::Compute() {
	for (int i = 0; i < this->num; i++) {
		this->muscles[i].Compute();
	}
}

void Muscle::Render(SDL_Renderer* renderer) {
	for (int i = 0; i < this->num; i++) {
		this->muscles[i].Render(renderer);
	}
}



UnitMuscle::UnitMuscle() {
	this->contracting_angle_sum = 0;
	this->power = 0;
}

UnitMuscle::~UnitMuscle() {
}

UnitMuscle* UnitMuscle::AddJoint(Joint<>* joint, double theta, double visualizing_distance) {
	JointFunctionGroup fn;
	double x_factor = cos(Constant::RAD(theta));
	double z_factor = sin(Constant::RAD(theta));

	fn.Force = [=](double t) {
		joint->SetForce(t * x_factor, 0, t * z_factor);
	};

	fn.Angle = [=]() {
		double x, y, z;
		joint->GetAngle(x, y, z);
		return x * x_factor + z * z_factor;
	};

	fn.Point = [=]() {
		double x_angle, y_angle, z_angle;
		joint->GetAngle(x_angle, y_angle, z_angle);
		DH_Matrix dh_matrix = Calculate::DH_Parameters(Vector3D(Constant::RAD(90 + (x_angle) / 2), 0, Constant::RAD(theta)), visualizing_distance, joint->GetDHMatrix(), false);
		return Calculate::DHMatrixToPosition(dh_matrix);
	};

	this->joint_functions.push_back(fn);

	return this;
}

UnitMuscle* UnitMuscle::SetContractingAngleSummation(int sum) {
	this->contracting_angle_sum = sum;
	return this;
}

void UnitMuscle::RotatePoints(Eigen::Matrix3d rotation_matrix) {
	for (int i = 0; i < this->muscle_visualizing_points.size(); i++) {
		Vector3D point = this->muscle_visualizing_points[i];
		Vector3D rotated_point = Calculate::Rotate(point, rotation_matrix);
		this->muscle_visualizing_points[i] = rotated_point;
	}
}

void UnitMuscle::Compute() {
	double angle_sum = 0;
	for (int i = 0; i < this->joint_functions.size(); i++) {
		angle_sum += this->joint_functions[i].Angle();
	}
	double delta = abs(angle_sum - this->contracting_angle_sum) / 360;
	double sign = angle_sum < this->contracting_angle_sum ? 1 : -1;
	double force = sign * delta * this->power;
	for (int i = 0; i < this->joint_functions.size(); i++) {
		this->joint_functions[i].Force(force);
	}

	this->muscle_visualizing_points.clear();
	for (int i = 0; i < this->joint_functions.size(); i++) {
		this->muscle_visualizing_points.push_back(this->joint_functions[i].Point());
	}
}

void UnitMuscle::Render(SDL_Renderer* renderer) {
	RenderingTool* rendering_tool = RenderingTool::GetInstance();

	auto color = GraphicColor::MUSCLE_COLOR;

	color.s += (100 - color.s) * (this->power > 1.0 ? 1.0 : this->power);

	auto rgb = color.to_RGB();

	rendering_tool->SetPointColor(rgb);
	rendering_tool->SetLineColor(rgb);
	/*for (int i = 0; i < this->muscle_visualizing_points.size(); i++) {
		Vector3D point = this->muscle_visualizing_points[i];
		rendering_tool->DrawPoint(renderer, point.x, point.y, point.z);
	}*/
	for (int i = 0; i < this->muscle_visualizing_points.size() - 1; i++) {
		Vector3D point1 = this->muscle_visualizing_points[i];
		Vector3D point2 = this->muscle_visualizing_points[i + 1];
		rendering_tool->DrawLine(renderer, point1.x, point1.y, point1.z, point2.x, point2.y, point2.z);
	}
}
