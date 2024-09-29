#include <Joint.h>

// Primary Joint ========================================

Joint<JointType::PRIMARY>::Joint(double distance, Joint<>* parent) : Joint<>(distance, parent) {
	this->type = JointType::PRIMARY;
};

void Joint<JointType::PRIMARY>::SetAngle(double x_rot) {
	this->rotation.x += Constant::RAD(x_rot);
	this->AdjustOverAngle();
}

void Joint<JointType::PRIMARY>::SetAngleAsProgress(double x_rot) {
	this->rotation.x = this->range_x_min + (this->range_x_max - this->range_x_min) * x_rot;
}

// Secondary Joint ========================================

Joint<JointType::SECONDARY>::Joint(double distance, Joint<>* parent) : Joint<>(distance, parent) {
	this->type = JointType::SECONDARY;
};

void Joint<JointType::SECONDARY>::SetAngle(double x_rot, double z_rot) {
	this->rotation.x += Constant::RAD(x_rot);
	this->rotation.z += Constant::RAD(z_rot);
	this->AdjustOverAngle();
}

void Joint<JointType::SECONDARY>::SetAngleAsProgress(double x_rot, double z_rot) {
	this->rotation.x = this->range_x_min + (this->range_x_max - this->range_x_min) * x_rot;
	this->rotation.z = this->range_z_min + (this->range_z_max - this->range_z_min) * z_rot;
}

// Tirtary Joint ========================================

Joint<JointType::TIRTARY>::Joint(double distance, Joint<>* parent) : Joint<>(distance, parent) {
	this->type = JointType::TIRTARY;
};

void Joint<JointType::TIRTARY>::SetAngle(double x_rot, double y_rot, double z_rot) {
	this->rotation.x += Constant::RAD(x_rot);
	this->rotation.y += Constant::RAD(y_rot);
	this->rotation.z += Constant::RAD(z_rot);
	this->AdjustOverAngle();
}

void Joint<JointType::TIRTARY>::SetAngleAsProgress(double x_rot, double y_rot, double z_rot) {
	this->rotation.x = this->range_x_min + (this->range_x_max - this->range_x_min) * x_rot;
	this->rotation.y = this->range_y_min + (this->range_y_max - this->range_y_min) * y_rot;
	this->rotation.z = this->range_z_min + (this->range_z_max - this->range_z_min) * z_rot;
}
