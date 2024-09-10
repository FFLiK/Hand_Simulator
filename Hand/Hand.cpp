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
}

Hand* Hand::AddJoint(Joint<>* joint) {
	joints.push_back(joint);
	return this;
}

Hand* Hand::AddMuscle(Muscle* muscle) {
	muscles.push_back(muscle);
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
		Joint<>* joint = joints[i];
		Vector3D point;
		joint->GetPosition(point.x, point.y, point.z);
		point = Calculate::Rotate(point, rotation_matrix);
		joint->SetPosition(point.x, point.y, point.z);
	}
	for (int i = 0; i < muscles.size(); i++) {
		muscles[i]->RotatePoints(rotation_matrix);
	}
}

void Hand::Render(SDL_Renderer *renderer, GraphicMode mode) {
	RenderingTool *rendering_tool = RenderingTool::GetInstance();
	

	double prev_metacarpa_x = 0, prev_metacarpa_y = 0, prev_metacarpa_z = 0;
	double prev_carpal_x = 0, prev_carpal_y = 0, prev_carpal_z = 0;
	double first_carpa_x = 0, first_carpa_y = 0, first_carpa_z = 0;

	rendering_tool->SetLineColor(GraphicColor::FRAME_COLOR_RGB);
	rendering_tool->SetPointColor(GraphicColor::FRAME_COLOR_RGB);

	if (mode == GraphicMode::SPECIFIC_FRAME) {
		for (int i = 0; i < muscles.size(); i++) {
			muscles[i]->Render(renderer);
		}
	}

	for(int i=0;i<joints.size();i++) {
		Joint<>* joint = joints[i];
		double x, y, z;
		joint->GetPosition(x, y, z);

		if (mode == GraphicMode::SPECIFIC_FRAME) {
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

			if (mode == GraphicMode::SPECIFIC_FRAME) {
				if (joint->IsRealJoint()) {
					rendering_tool->SetLineColor(GraphicColor::BONE_COLOR_RGB);
				}
				else {
					rendering_tool->SetLineColor(GraphicColor::FALSE_COLOR_RGB);
				}
			}
			rendering_tool->DrawLine(renderer, x_parent, y_parent, z_parent, x, y, z);

			if (parent_joint->GetParentJoint() == nullptr) {
				if (prev_carpal_x + prev_carpal_y + prev_carpal_z) {
					if (mode == GraphicMode::SPECIFIC_FRAME) {
						rendering_tool->SetLineColor(GraphicColor::FALSE_COLOR_RGB);
					}
					rendering_tool->DrawLine(renderer, prev_carpal_x, prev_carpal_y, prev_carpal_z, x, y, z);
				}
				else {
					first_carpa_x = x;
					first_carpa_y = y;
					first_carpa_z = z;
				}
				prev_carpal_x = x;
				prev_carpal_y = y;
				prev_carpal_z = z;
			}
			else if (parent_joint->GetParentJoint()->GetParentJoint() == nullptr) {
				if (prev_metacarpa_x + prev_metacarpa_y + prev_metacarpa_z) {
					if (mode == GraphicMode::SPECIFIC_FRAME) {
						rendering_tool->SetLineColor(GraphicColor::FALSE_COLOR_RGB);
					}
					rendering_tool->DrawLine(renderer, prev_metacarpa_x, prev_metacarpa_y, prev_metacarpa_z, x, y, z);
				}
				else if (first_carpa_x + first_carpa_y + first_carpa_z) {
					if (mode == GraphicMode::SPECIFIC_FRAME) {
						rendering_tool->SetLineColor(GraphicColor::FALSE_COLOR_RGB);
					}
					rendering_tool->DrawLine(renderer, first_carpa_x, first_carpa_y, first_carpa_z, x, y, z);
				}
				prev_metacarpa_x = x;
				prev_metacarpa_y = y;
				prev_metacarpa_z = z;
			}
		}
	}
	if (mode == GraphicMode::SPECIFIC_FRAME) {
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
