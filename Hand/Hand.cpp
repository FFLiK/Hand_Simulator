#include "Hand.h"
#include <RenderingTool.h>
#include <Log.h>

Hand::Hand() {
}

Hand::~Hand() {
	for(int i=0;i<joints.size();i++) {
		delete joints[i];
	}
	for (int i = 0; i < muscles.size(); i++) {
		delete muscles[i];
	}
	for (int i = 0; i < press_motion_function_set.size(); i++) {
		delete press_motion_function_set[i];
	}
	for (int i = 0; i < release_motion_function_set.size(); i++) {
		delete release_motion_function_set[i];
	}
	joints.clear();
	muscles.clear();
	press_motion_function_set.clear();
	release_motion_function_set.clear();
	previous_joints_position_data.clear();
	current_joints_position_data.clear();
}

Hand* Hand::AddJoint(Joint<>* joint) {
	joints.push_back(joint);
	return this;
}

Hand* Hand::AddMuscle(Muscle* muscle) {
	muscles.push_back(muscle);
	return this;
}

Hand* Hand::AddFalseLine(Joint<>* joint1, Joint<>* joint2) {
	additional_false_line.push_back(make_pair(joint1, joint2));
	return this;
}

void Hand::Compute() {

	for (int i = 0; i < muscles.size(); i++) {
		muscles[i]->Compute();
	}

	Joint<>::NewComputation();
	for (int i = 0; i < joints.size(); i++) {
		joints[i]->Compute();
	}

	Eigen::Matrix3d rotation_matrix = Calculate::RotationMatrix(this->orientation);
	for (int i = 0; i < joints.size(); i++) {
		joints[i]->RotatePoints(rotation_matrix);
	}
	for (int i = 0; i < muscles.size(); i++) {
		muscles[i]->RotatePoints(rotation_matrix);
	}

	this->previous_joints_position_data = this->current_joints_position_data;
	this->current_joints_position_data.clear();
	for (int i = 0; i < this->joints.size(); i++) {
		Vector3D pos;
		this->joints[i]->GetPosition(pos.x, pos.y, pos.z);
		this->current_joints_position_data.push_back(pos);
	}
}

void Hand::Render(SDL_Renderer *renderer, GraphicMode mode) {
	RenderingTool *rendering_tool = RenderingTool::GetInstance();
	

	double prev_metacarpa_x = 0, prev_metacarpa_y = 0, prev_metacarpa_z = 0;
	double prev_carpal_x = 0, prev_carpal_y = 0, prev_carpal_z = 0;
	double first_carpa_x = 0, first_carpa_y = 0, first_carpa_z = 0;

	rendering_tool->SetLineColor(GraphicColor::FRAME_COLOR_RGB);
	rendering_tool->SetPointColor(GraphicColor::FRAME_COLOR_RGB);

	if (mode == GraphicMode::SPECIFIC_FRAME || mode == GraphicMode::COORDINATE) {
		for (int i = 0; i < muscles.size(); i++) {
			muscles[i]->Render(renderer);
		}
	}

	if (mode == GraphicMode::SPECIFIC_FRAME || mode == GraphicMode::COORDINATE) {
		rendering_tool->SetLineColor(GraphicColor::FALSE_COLOR_RGB);
	}
	for (int i = 0; i < this->additional_false_line.size(); i++) {
		Joint<>* joint1 = additional_false_line[i].first;
		Joint<>* joint2 = additional_false_line[i].second;
		double x1, y1, z1;
		double x2, y2, z2;
		joint1->GetPosition(x1, y1, z1);
		joint2->GetPosition(x2, y2, z2);
		rendering_tool->DrawLine(renderer, x1, y1, z1, x2, y2, z2);
	}

	for(int i=0;i<joints.size();i++) {
		Joint<>* joint = joints[i];
		double x, y, z;
		joint->GetPosition(x, y, z);

		if (mode == GraphicMode::SPECIFIC_FRAME || mode == GraphicMode::COORDINATE) {
			if (joint->IsEndEffector()) {
				rendering_tool->SetPointColor(GraphicColor::BONE_COLOR_RGB);
			}
			else if (joint->IsRealJoint()) {
				rendering_tool->SetPointColor(GraphicColor::JOINT_COLOR_RGB);
			}
			else {
				rendering_tool->SetPointColor(GraphicColor::FALSE_COLOR_RGB);
			}
		}
		rendering_tool->DrawPoint(renderer, x, y, z);
		Joint<>* parent_joint = joint->GetParentJoint();

		if (parent_joint != nullptr) {
			double x_parent, y_parent, z_parent;
			parent_joint->GetPosition(x_parent, y_parent, z_parent);

			Eigen::Vector3d original_vector(x_parent, y_parent, z_parent);

			if (mode == GraphicMode::SPECIFIC_FRAME || mode == GraphicMode::COORDINATE) {
				if (joint->IsRealJoint()) {
					rendering_tool->SetLineColor(GraphicColor::BONE_COLOR_RGB);
				}
				else {
					rendering_tool->SetLineColor(GraphicColor::FALSE_COLOR_RGB);
				}
			}
			rendering_tool->DrawLine(renderer, x_parent, y_parent, z_parent, x, y, z);
		}

		if (mode == GraphicMode::COORDINATE) {
			Vector3D normal;
			joint->GetNormal(normal.x, normal.y, normal.z);
			rendering_tool->SetLineColor(GraphicColor::FRAME_COLOR_RGB);
			rendering_tool->DrawLine(renderer, normal.x, normal.y, normal.z, x, y, z);
		}
	}
	if (mode == GraphicMode::SPECIFIC_FRAME || mode == GraphicMode::COORDINATE) {
		rendering_tool->SetLineColor(GraphicColor::FALSE_COLOR_RGB);
	}
	rendering_tool->DrawLine(renderer, prev_carpal_x, prev_carpal_y, prev_carpal_z, prev_metacarpa_x, prev_metacarpa_y, prev_metacarpa_z);
}

std::vector<Joint<>*> Hand::GetJoints() {
	return this->joints;
}

void Hand::SetOrientation(double x, double y, double z) {
	this->orientation.x += Constant::RAD(x);
	this->orientation.y += Constant::RAD(y);
	this->orientation.z += Constant::RAD(z);
}

void Hand::SetFinalPose(vector<Vector3D> args) {
	for (int i = 0; i < this->result_function_set.size(); i++) {
		delete this->result_function_set[i];
	}
	this->result_function_set.clear();

	int k = 0;
	for (int i = 0; i < this->joints.size(); i++) {
		if (this->joints[i]->IsEndEffector()) {
			this->result_function_set.push_back(
				new function<double()>(
					[=]()->double {
						Vector3D target_position = args[k];
						Vector3D current_position;
						this->joints[i]->GetPosition(current_position.x, current_position.y, current_position.z);
						Vector3D delta;
						delta.x = target_position.x - current_position.x;
						delta.y = target_position.y - current_position.y;
						delta.z = target_position.z - current_position.z;
						double distance = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
						return distance;
					}
				)
			);
			k++;
		}
	}
}

void Hand::Optimization() {
	Log::Hand("Start Optimization . . .");

	double delta = OptimizationParameter::LEARNING_RATE;
	double epsilon = OptimizationParameter::ERROR_THRESHOLD;
	
	bool optimizing = true;
	int optimize_step = 0;

	while (optimizing) {
		// 1. Jacobian Matrix (Muslcle Forces -> End Effectors (Result Function))
		Eigen::MatrixXd jacobian_matrix = Eigen::MatrixXd::Zero(this->result_function_set.size(), this->muscles.size());

		for (int i = 0; i < this->muscles.size(); i++) {
			for (int j = 0; j < this->result_function_set.size(); j++) {
				double original = (*this->result_function_set[j])();

				this->muscles[i]->SetPower(this->muscles[i]->GetPower() + delta);
				do this->Compute(); while (!this->IsStable());
				double changed = (*this->result_function_set[j])();

				jacobian_matrix(j, i) = (changed - original) / delta;

				this->muscles[i]->SetPower(this->muscles[i]->GetPower() - delta);
				do this->Compute(); while (!this->IsStable());
			}
		}

		// 2. Optimization by Newton-Raphson Method
		// Newton-Raphson Method: x_{n+1} = x_n - (Jacobian Matrix)^{-1} * f(x_n)
		Eigen::MatrixXd jacobian_inverse = jacobian_matrix.inverse();
		Eigen::VectorXd f_vector = Eigen::VectorXd::Zero(this->result_function_set.size());
		for (int i = 0; i < this->result_function_set.size(); i++) {
			f_vector(i) = (*this->result_function_set[i])();
		}
		Eigen::VectorXd delta_vector = jacobian_inverse * f_vector;

		for (int i = 0; i < this->muscles.size(); i++) {
			this->muscles[i]->SetPower(this->muscles[i]->GetPower() + delta_vector(i));
		}

		do this->Compute(); while (!this->IsStable());

		// 3. Check Result
		optimizing = false;
		for (int i = 0; i < this->result_function_set.size(); i++) {
			if (abs((*this->result_function_set[i])()) > epsilon) {
				optimizing = true;
				break;
			}
		}

		optimize_step++;

		if (optimize_step > OptimizationParameter::OPTIMIZING_LIMIT) {
			Log::Error("Optimization Failed");
			break;
		}
		else {
			Log::Debug("Optimization Step: " + to_string(optimize_step) + " / " + to_string(OptimizationParameter::OPTIMIZING_LIMIT) + " (" + to_string(100 * optimize_step / OptimizationParameter::OPTIMIZING_LIMIT) + "%)");
		}
	}

	Log::Hand("Finished");
}

bool Hand::IsStable() {
	if (this->current_joints_position_data.size() != this->previous_joints_position_data.size())
		return false;
	for (int i = 0; i < this->current_joints_position_data.size(); i++) {
		double dx = this->current_joints_position_data[i].x - this->previous_joints_position_data[i].x;
		double dy = this->current_joints_position_data[i].y - this->previous_joints_position_data[i].y;
		double dz = this->current_joints_position_data[i].z - this->previous_joints_position_data[i].z;
		double distance = dx * dx + dy * dy + dz * dz;
		if (distance > HandParameter::STABLE_CONDITION) {
			return false;
		}
	}
	return true;
}
