#include "Hand.h"
#include <RenderingTool.h>
#include <Log.h>

Hand::Hand() {
}

Hand::~Hand() {
	for(int i=0;i<joints.size();i++) {
		delete joints[i];
	}
	for (int i = 0; i < clicking_motion_function_set.size(); i++) {
		delete clicking_motion_function_set[i];
	}
	joints.clear();
}

Hand* Hand::AddJoint(Joint<>* joint) {
	joints.push_back(joint);
	return this;
}

void Hand::Calculate() {
	Joint<>::NewCalculation();
	for(int i=0;i<joints.size();i++) {
		joints[i]->Calculate();
	}

	Eigen::Matrix3d rotation_matrix;
	rotation_matrix =
		Eigen::AngleAxisd(this->orientation_x, Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(this->orientation_y, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(this->orientation_z, Eigen::Vector3d::UnitZ());
	for (int i = 0; i < joints.size(); i++) {
		Joint<>* joint = joints[i];
		double x, y, z;
		joint->GetPosition(x, y, z);

		Eigen::Vector3d original_vector(x, y, z);
		Eigen::Vector3d rotated_vector = original_vector.transpose() * rotation_matrix;
		x = rotated_vector.x();
		y = rotated_vector.y();
		z = rotated_vector.z();

		joint->SetPosition(x, y, z);
	}
}

void Hand::Render(SDL_Renderer *renderer, GraphicMode mode) {
	RenderingTool *rendering_tool = RenderingTool::GetInstance();
	

	double prev_metacarpa_x = 0, prev_metacarpa_y = 0, prev_metacarpa_z = 0;
	double prev_carpal_x = 0, prev_carpal_y = 0, prev_carpal_z = 0;
	double first_carpa_x = 0, first_carpa_y = 0, first_carpa_z = 0;

	rendering_tool->SetLineColor(GraphicColor::FRAME_COLOR_RGB);
	rendering_tool->SetPointColor(GraphicColor::FRAME_COLOR_RGB);

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
	this->orientation_x += Constant::RAD(x);
	this->orientation_y += Constant::RAD(y);
	this->orientation_z += Constant::RAD(z);
}
