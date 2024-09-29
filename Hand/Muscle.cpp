#include "Muscle.h"
#include "Hand.h"
#include "RenderingTool.h"

Muscle::Muscle() {
}

Muscle::~Muscle() {
}

Muscle* Muscle::SetMuscleUnitSize(int num) {
	this->muscles.resize(num);
	return this;
}

Muscle* Muscle::SetMuscleStrength(double strength) {
	this->strength = strength;
	return this;
}

int Muscle::GetMuscleUnitSize() {
	return this->muscles.size();
}

UnitMuscle* Muscle::GetMuscleUnit(int index) {
	return &(this->muscles[index]);
}

void Muscle::SetPower(double power, int index) {
	if (index == -1) {
		for (int i = 0; i < this->muscles.size(); i++) {
			this->muscles[i].core_power = power;
		}
	}
	else {
		this->muscles[index].core_power = power;
	}
}

double Muscle::GetPower() {
	double power = 0;
	for (int i = 0; i < this->muscles.size(); i++) {
		power += this->muscles[i].core_power;
	}
	return power / this->muscles.size();
}

void Muscle::RotatePoints(Eigen::Matrix3d rotation_matrix) {
	for (int i = 0; i < this->muscles.size(); i++) {
		this->muscles[i].RotatePoints(rotation_matrix);
	}
}

void Muscle::Compute() {
	for (int i = 0; i < this->muscles.size(); i++) {
		this->muscles[i].dependent_power = 0;
		for (int j = 0; j < this->muscles.size(); j++) {
			this->muscles[i].dependent_power += this->muscles[j].core_power * pow(HandParameter::MUSCLE_UNIT_DEPENDENCY, abs(i - j));
		}
		if (this->muscles[i].dependent_power > 1.0) {
			this->muscles[i].dependent_power = 1.0;
		}
		this->muscles[i].Compute();
	}
}

void Muscle::Render(SDL_Renderer* renderer) {
	for (int i = 0; i < this->muscles.size(); i++) {
		this->muscles[i].Render(renderer);
	}
}



UnitMuscle::UnitMuscle() {
	this->contracting_angle_sum = 0;
	this->core_power = 0;
	this->dependent_power = 0;
}

UnitMuscle::~UnitMuscle() {
}

UnitMuscle* UnitMuscle::AddJoint(Joint<>* joint, double theta, double visualizing_distance, MuscleMotionDirection x_motion, MuscleMotionDirection y_motion, MuscleMotionDirection z_motion) {
	JointFunctionGroup fn;
	double x_factor = sin(Constant::RAD(theta)) * (x_motion == MuscleMotionDirection::SIN) + cos(Constant::RAD(theta)) * (x_motion == MuscleMotionDirection::COS);
	double y_factor = sin(Constant::RAD(theta)) * (y_motion == MuscleMotionDirection::SIN) + cos(Constant::RAD(theta)) * (y_motion == MuscleMotionDirection::COS);
	double z_factor = sin(Constant::RAD(theta)) * (z_motion == MuscleMotionDirection::SIN) + cos(Constant::RAD(theta)) * (z_motion == MuscleMotionDirection::COS);

	fn.Force = [=](double t) {
		joint->SetForce(t * x_factor, t * y_factor, t * z_factor);
		};

	fn.Angle = [=]() {
		double x, y, z;
		joint->GetAngle(x, y, z);
		return x * x_factor + y * y_factor + z * z_factor;
		};

	fn.Point = [=]() {
		double x_angle, y_angle, z_angle;
		joint->GetAngle(x_angle, y_angle, z_angle);
		DH_Matrix dh_matrix = Calculate::DH_Parameters(Vector3D(Constant::RAD(90 + (x_angle) / 2), 0, Constant::RAD(theta)), visualizing_distance, joint->GetDHMatrix(), false);
		return Calculate::DHMatrixToPosition(dh_matrix);
		};

	fn.Range = [=]() {
		double x_range, y_range, z_range;
		if (x_factor > 0.0) {
			double min, max;
			joint->GetRangeX(min, max);
			x_range = max * x_factor;
		}
		else {
			double min, max;
			joint->GetRangeX(min, max);
			x_range = min * x_factor;
		}
		if (y_factor > 0.0) {
			double min, max;
			joint->GetRangeY(min, max);
			y_range = max * y_factor;
		}
		else {
			double min, max;
			joint->GetRangeY(min, max);
			y_range = min * y_factor;
		}
		if (z_factor > 0.0) {
			double min, max;
			joint->GetRangeZ(min, max);
			z_range = max * z_factor;
		}
		else {
			double min, max;
			joint->GetRangeZ(min, max);
			z_range = min * z_factor;
		}
		return x_range + y_range + z_range;
		};

	this->joint_functions.push_back(fn);

	return this;
}

UnitMuscle* UnitMuscle::SetContractingAngleSummation(int sum) {
	this->contracting_angle_sum = sum;
	if (sum == -1) {
		this->contracting_angle_sum = 0;
		for (int i = 0; i < this->joint_functions.size(); i++) {
			this->contracting_angle_sum += this->joint_functions[i].Range();
		}
		this->contracting_angle_sum *= HandParameter::MUSCLE_CONTRACTING_ANGLE_SUM_APLIFICATION_FACTOR;
	}
	return this;
}

void UnitMuscle::RotatePoints(Eigen::Matrix3d rotation_matrix) {
	for (int i = 0; i < this->muscle_visualizing_points.size(); i++) {
		Vector3D point = this->muscle_visualizing_points[i];
		Vector3D rotated_point = Calculate::Rotate(point, rotation_matrix);
		this->muscle_visualizing_points[i] = rotated_point;
	}
}

// Compute the force of the muscle
//tex:
//For all Muscles $M_i$, for all Joints $J_j$ related to $M,
//tex:
//$F_j$ is Force of Joint $J_j$, $P_i$ is Power of Muscle $M_i$,
//tex:
//$C_i$ is Contracting Angle Summation of Muscle $M_i$

//tex:
//$$f_j(i) = \Delta \theta \cdot P_i$$
//$$\text{where } \Delta \theta = \frac{\sum_{j=1}^{n} \theta_j - C_i}{360}$$
//$$F_j = \sum_{i=1}^{m} f_j(i)$$

void UnitMuscle::Compute() {
	double angle_sum = 0;
	for (int i = 0; i < this->joint_functions.size(); i++) {
		angle_sum += this->joint_functions[i].Angle();
	}
	double delta = abs(angle_sum - this->contracting_angle_sum) / 360;
	double sign = angle_sum < this->contracting_angle_sum ? 1 : -1;
	double force = sign * delta * this->dependent_power;
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

	color.s += (100 - color.s) * (this->dependent_power > 1.0 ? 1.0 : this->dependent_power);

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
