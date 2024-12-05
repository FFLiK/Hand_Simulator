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
	if (this->stop_external_call) {
		this->is_stop_safely = true;
		return;
	}
	this->ComputeInternally();
}

void Hand::ComputeInternally() {
	for (int i = 0; i < muscles.size(); i++) {
		muscles[i]->Compute();
	}

	Joint<>::NewComputation();
	for (int i = 0; i < joints.size(); i++) {
		joints[i]->Compute(!this->IsFreeMoving());
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

void Hand::GetCurrentPose(vector<Vector3D>& args) {
	args.clear();
	for (int i = 0; i < this->joints.size(); i++) {
		if (this->joints[i]->IsEndEffector()) {
			Vector3D pos;
			this->joints[i]->GetPosition(pos.x, pos.y, pos.z);
			args.push_back(pos);
		}
	}
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
	Log::Debug(this->muscles[0]->GetPower());

	this->is_stop_safely = false;
	this->stop_external_call = true;
	while (!this->is_stop_safely) {
		// Wait for stopping rendering
	}

	Log::Hand("Start Optimization . . .");
	
	double delta = OptimizationParameter::INFINITESIMAL_CHANGE_FOR_DERIVATIVE;
	double epsilon = OptimizationParameter::ERROR_THRESHOLD;
	double mu = OptimizationParameter::DAMPING_CONSTANT;
	
	bool optimizing = true;
	int optimize_step = 0;

	this->is_free_moving = false;

	while (optimizing) {
		Log::Hand("[Optimization]", "Optimizing... (", optimize_step, ")");
		//! Calculate Gradient
		Eigen::VectorXd gradient(this->muscles.size());
		for (int i = 0; i < this->muscles.size(); i++) {
			double original = this->muscles[i]->GetPower();

			this->muscles[i]->SetPower(original + delta);
			do this->ComputeInternally(); while (!this->IsStable());
			double b = this->ObjectiveFunction();

			double minus = this->ObjectiveFunction();
			this->muscles[i]->SetPower(original);
			do this->ComputeInternally(); while (!this->IsStable());
			double a = this->ObjectiveFunction();

			gradient(i) = (b - a) / delta;
		}
		Log::Hand("[Optimization]", "Gradient Calculated");

		//! Calculate Hessian
		Eigen::MatrixXd hessian(this->muscles.size(), this->muscles.size());
		for (int i = 0; i < this->muscles.size(); i++) {
			for (int j = 0; j < this->muscles.size(); j++) {
				double original_i = this->muscles[i]->GetPower();
				double original_j = this->muscles[j]->GetPower();

				this->muscles[i]->SetPower(original_i + delta);
				this->muscles[j]->SetPower(original_j);
				do this->ComputeInternally(); while (!this->IsStable());
				double b = this->ObjectiveFunction();

				this->muscles[i]->SetPower(original_i + delta);
				this->muscles[j]->SetPower(original_j + delta);
				do this->ComputeInternally(); while (!this->IsStable());
				double a = this->ObjectiveFunction();

				this->muscles[i]->SetPower(original_i);
				this->muscles[j]->SetPower(original_j + delta);
				do this->ComputeInternally(); while (!this->IsStable());
				double c = this->ObjectiveFunction();

				this->muscles[i]->SetPower(original_i);
				this->muscles[j]->SetPower(original_j);
				do this->ComputeInternally(); while (!this->IsStable());
				double d = this->ObjectiveFunction();

				hessian(i, j) = (a - b - c + d) / (delta * delta);
			}
		}
		Log::Hand("[Optimization]", "Hessian Calculated");

		//! Calculate Damping Factor
		Eigen::MatrixXd damping_matrix = mu * Eigen::MatrixXd::Identity(this->muscles.size(), this->muscles.size());
		
		Log::Hand("[Optimization]", "Damping Factor Calculated");

		auto positive_matrix = [](Eigen::MatrixXd matrix)->Eigen::MatrixXd {
			double epsilon = 1e-6;
			Eigen::MatrixXd symMatrix = (matrix + matrix.transpose()) / 2;

			Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symMatrix);

			Eigen::VectorXd eigenvalues = solver.eigenvalues();
			for (int i = 0; i < eigenvalues.size(); ++i) {
				if (eigenvalues(i) < 0) {
					eigenvalues(i) = -eigenvalues(i);
				}
			}

			Eigen::MatrixXd D = eigenvalues.asDiagonal();
			Eigen::MatrixXd U = solver.eigenvectors();
			Eigen::MatrixXd positive_matrix = U * D * U.transpose();
			return positive_matrix;
		};

		//! Optimization
		// by Newton's Method with Damping
		Eigen::MatrixXd step = (positive_matrix(hessian) + damping_matrix).inverse() * gradient;
		Log::Debug("Steps");
		for (int i = 0; i < this->muscles.size(); i++) {
			double power = this->muscles[i]->GetPower() - step(i);
			if (power > 1.0) power = 1.0;
			if (power < 0.0) power = 0.0;
			this->muscles[i]->SetPower(power);
		}
		do this->ComputeInternally(); while (!this->IsStable());

		Log::Debug("Muscle Foreces");
		for (int i = 0; i < this->muscles.size(); i++) {
			Log::Debug(i, ":", this->muscles[i]->GetPower());
		}

		Log::Hand("[Optimization]", "Optimizing (One Step) Completed");
		Log::Hand("[Optimization]", "Objective Function (x) = ", this->ObjectiveFunction());

		// Print Current Pose
		vector<Vector3D> current_pose;
		this->GetCurrentPose(current_pose);
		Log::Hand("[Optimization]", "Current Pose");
		for (int i = 0; i < current_pose.size(); i++) {
			Log::Hand("[Optimization]", "¤¤", current_pose[i].x, current_pose[i].y, current_pose[i].z);
		}

		// Check Convergence
		double step_size = step.norm();
		if (step_size < epsilon) {
			optimizing = false;
		}
		
		optimize_step++;

		if (optimize_step > OptimizationParameter::OPTIMIZING_LIMIT) {
			Log::Error("Optimization Failed");
			break;
		}
	}

	Log::Hand("Finished");
	this->stop_external_call = false;
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

void Hand::SetFreeMoving(bool is_free_moving) {
	this->is_free_moving = is_free_moving;
}

bool Hand::IsFreeMoving() {
	return this->is_free_moving;
}

double Hand::ObjectiveFunction() {
	double result = 0;
	for (int i = 0; i < this->result_function_set.size(); i++) {
		result += (*this->result_function_set[i])();
	}
	return result / (double)this->result_function_set.size();
}
