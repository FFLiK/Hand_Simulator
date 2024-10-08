#pragma once
#include <Constant.h>

enum class JointType {
	NONE, PRIMARY, SECONDARY, TIRTARY
};

enum class JointFlag {
	REAL_JOINT = 0, VIRTUAL_JOINT = 1, END_EFFECTOR = 2
};

template<JointType T = JointType::NONE>
class Joint {
protected:
	JointType type;

	// Initial Value
	Vector3D initial_angle;

	// Input
	double distance;
	Vector3D rotation;

	// Output
	Vector3D position;
	Vector3D normal;

	// Force
	Vector3D force;
	DH_Matrix dh_matrix; // Denavit-Hartenberg matrix, M = Tz * Rz * Tx * Rx
	Joint<>* parent_joint;

	// Limitation
	double range_x_min = 0, range_x_max = 0;
	double range_y_min = 0, range_y_max = 0;
	double range_z_min = 0, range_z_max = 0;

	int update_seed;

	static int current_update_seed;

	int flag;

	void AdjustOverAngle();

public:
	Joint(double distance, Joint* parent = nullptr);
	virtual ~Joint();

	void Compute();

	Joint<>* GetPosition(double& x, double& y, double& z);
	Joint<>* GetNormal(double& x, double& y, double& z);
	Joint<>* SetPosition(double x, double y, double z);
	Joint<>* GetParentJoint();
	Joint<>* GetAngle(double& x_rot, double& y_rot, double& z_rot);
	Joint<>* InitAngle(double x_rot, double y_rot, double z_rot);
	Joint<>* SetRangeX(double min, double max);
	Joint<>* SetRangeY(double min, double max);
	Joint<>* SetRangeZ(double min, double max);
	Joint<>* GetRangeX(double& min, double& max);
	Joint<>* GetRangeY(double& min, double& max);
	Joint<>* GetRangeZ(double& min, double& max);
	Joint<>* SetFlag(JointFlag flag);
	void SetAngle(double x_rot, double y_rot, double z_rot);
	void SetAngleAsProgress(double x_rot, double y_rot, double z_rot);
	void SetForce(double x, double y, double z);
	void RotatePoints(Eigen::Matrix3d rotation_matrix);

	DH_Matrix GetDHMatrix();

	JointType Type() const;

	bool IsRealJoint();
	bool IsEndEffector();

	static void NewComputation();

	friend class Joint<JointType::PRIMARY>;
	friend class Joint<JointType::SECONDARY>;
	friend class Joint<JointType::TIRTARY>;
};

template<>
class Joint<JointType::PRIMARY> : public Joint<> {
public:
	Joint(double distance, Joint<>* parent = nullptr);
	void SetAngle(double x_rot);
	void SetAngleAsProgress(double x_rot);
};

template<>
class Joint<JointType::SECONDARY> : public Joint<> {
public:
	Joint(double distance, Joint<>* parent = nullptr);
	void SetAngle(double x_rot, double z_rot);
	void SetAngleAsProgress(double x_rot, double z_rot);
};

template<>
class Joint<JointType::TIRTARY> : public Joint<> {
public:
	Joint(double distance, Joint<>* parent = nullptr);
	void SetAngle(double x_rot, double y_rot, double z_rot);
	void SetAngleAsProgress(double x_rot, double y_rot, double z_rot);
};



template<JointType T>
int Joint<T>::current_update_seed = 0;

template<JointType T>
Joint<T>::Joint(double distance, Joint* parent) {
	this->type = JointType::NONE;

	if (parent == this) {
		throw "Joint cannot be parent of itself";
	}
	if (!parent)
		distance = 0;

	this->flag = 0;

	this->parent_joint = parent;

	this->rotation = { 0, 0, 0 };
	this->position = { 0, 0, 0 };
	this->normal = { 0, 0, 0 };
	this->distance = distance;

	this->dh_matrix = EMPTY_MAT;

	this->update_seed = Joint::current_update_seed;
}

template<JointType T>
Joint<T>::~Joint() {

}

template<JointType T>
void Joint<T>::Compute() {
	// Force Update
	this->force.x *= HandParameter::MUSCLE_FORCE_AMPLIFICATION_FACTOR;
	this->force.y *= HandParameter::MUSCLE_FORCE_AMPLIFICATION_FACTOR;
	this->force.z *= HandParameter::MUSCLE_FORCE_AMPLIFICATION_FACTOR;

	double x_delta = (Constant::DEG(this->initial_angle.x) - Constant::DEG(this->rotation.x)) / 360;
	double y_delta = (Constant::DEG(this->initial_angle.y) - Constant::DEG(this->rotation.y)) / 360;
	double z_delta = (Constant::DEG(this->initial_angle.z) - Constant::DEG(this->rotation.z)) / 360;

	this->force.x += x_delta * HandParameter::NEUTRAL_FORCE_AMPLIFICATION_FACTOR;
	this->force.y += y_delta * HandParameter::NEUTRAL_FORCE_AMPLIFICATION_FACTOR;
	this->force.z += z_delta * HandParameter::NEUTRAL_FORCE_AMPLIFICATION_FACTOR;

	this->SetAngle(this->force.x, this->force.y, this->force.z);
	this->force = { 0, 0, 0 };

	if (this->update_seed != Joint::current_update_seed) {
		// Parent Calculation
		DH_Matrix parent_matrix;
		if (this->parent_joint) {
			this->parent_joint->Compute();
			parent_matrix = this->parent_joint->dh_matrix;
		}
		else {
			parent_matrix = EMPTY_MAT;
		}

		// Calculate DH Matrix
		this->dh_matrix = Calculate::DH_Parameters(this->rotation, this->distance, parent_matrix, !this->parent_joint);

		this->position = Calculate::DHMatrixToPosition(this->dh_matrix);

		auto x_matrix = Calculate::DH_Parameters(Vector3D(Constant::RAD(90 + (this->rotation.x) / 2), 0, 0), 50, this->dh_matrix, false);
		this->normal = Calculate::DHMatrixToPosition(x_matrix);

		this->update_seed = Joint::current_update_seed;
	}
}

template<JointType T>
inline void Joint<T>::AdjustOverAngle() {
	if (this->rotation.x > this->range_x_max) this->rotation.x = this->range_x_max;
	if (this->rotation.x < this->range_x_min) this->rotation.x = this->range_x_min;
	if (this->rotation.y > this->range_y_max) this->rotation.y = this->range_y_max;
	if (this->rotation.y < this->range_y_min) this->rotation.y = this->range_y_min;
	if (this->rotation.z > this->range_z_max) this->rotation.z = this->range_z_max;
	if (this->rotation.z < this->range_z_min) this->rotation.z = this->range_z_min;
}

template<JointType T>
inline Joint<>* Joint<T>::GetPosition(double& x, double& y, double& z) {
	x = this->position.x;
	y = this->position.y;
	z = this->position.z;
	return this;
}

template<JointType T>
inline Joint<>* Joint<T>::GetNormal(double& x, double& y, double& z) {
	x = this->normal.x;
	y = this->normal.y;
	z = this->normal.z;
	return this;
}

template<JointType T>
inline Joint<>* Joint<T>::SetPosition(double x, double y, double z) {
	this->position.x = x;
	this->position.y = y;
	this->position.z = z;
	return this;
}

template<JointType T>
Joint<JointType::NONE>* Joint<T>::GetParentJoint() {
	return this->parent_joint;
}

template<JointType T>
inline Joint<>* Joint<T>::GetAngle(double& x_rot, double& y_rot, double& z_rot) {
	x_rot = Constant::DEG(this->rotation.x);
	y_rot = Constant::DEG(this->rotation.y);
	z_rot = Constant::DEG(this->rotation.z);
	return this;
}

template<JointType T>
Joint<>* Joint<T>::InitAngle(double x_rot, double y_rot, double z_rot) {
	this->initial_angle.x = Constant::RAD(x_rot);
	this->initial_angle.y = Constant::RAD(y_rot);
	this->initial_angle.z = Constant::RAD(z_rot);

	this->rotation = this->initial_angle;

	this->SetRangeX(x_rot, x_rot);
	this->SetRangeY(y_rot, y_rot);
	this->SetRangeZ(z_rot, z_rot);
	return this;
}

template<JointType T>
inline Joint<>* Joint<T>::SetRangeX(double min, double max) {
	min = Constant::RAD(min);
	max = Constant::RAD(max);
	this->range_x_min = min;
	this->range_x_max = max;
	return this;
}

template<JointType T>
inline Joint<>* Joint<T>::SetRangeY(double min, double max) {
	min = Constant::RAD(min);
	max = Constant::RAD(max);
	this->range_y_min = min;
	this->range_y_max = max;
	return this;
}

template<JointType T>
inline Joint<>* Joint<T>::SetRangeZ(double min, double max) {
	min = Constant::RAD(min);
	max = Constant::RAD(max);
	this->range_z_min = min;
	this->range_z_max = max;
	return this;
}

template<JointType T>
inline Joint<>* Joint<T>::GetRangeZ(double& min, double& max) {
	min = Constant::DEG(this->range_z_min);
	max = Constant::DEG(this->range_z_max);
	return this;
}

template<JointType T>
inline Joint<>* Joint<T>::GetRangeY(double& min, double& max) {
	min = Constant::DEG(this->range_y_min);
	max = Constant::DEG(this->range_y_max);
	return this;
}

template<JointType T>
inline Joint<>* Joint<T>::GetRangeX(double& min, double& max) {
	min = Constant::DEG(this->range_x_min);
	max = Constant::DEG(this->range_x_max);
	return this;
}

template<JointType T>
inline void Joint<T>::SetForce(double x, double y, double z) {
	this->force.x += x;
	this->force.y += y;
	this->force.z += z;
}

template<JointType T>
inline void Joint<T>::RotatePoints(Eigen::Matrix3d rotation_matrix) {
	this->position = Calculate::Rotate(this->position, rotation_matrix);
	this->normal = Calculate::Rotate(this->normal, rotation_matrix);
}

template<JointType T>
inline Joint<>* Joint<T>::SetFlag(JointFlag flag) {
	this->flag = static_cast<int>(flag);
	return this;
}

template<JointType T>
inline void Joint<T>::SetAngle(double x_rot, double y_rot, double z_rot) {
	this->rotation.x += Constant::RAD(x_rot);
	this->rotation.y += Constant::RAD(y_rot);
	this->rotation.z += Constant::RAD(z_rot);
	this->AdjustOverAngle();
}

template<JointType T>
inline DH_Matrix Joint<T>::GetDHMatrix() {
	return this->dh_matrix;
}

template<JointType T>
inline JointType Joint<T>::Type() const {
	return this->type;
}

template<JointType T>
inline bool Joint<T>::IsRealJoint() {
	return !(this->flag & (int)JointFlag::VIRTUAL_JOINT);
}

template<JointType T>
inline bool Joint<T>::IsEndEffector() {
	return this->flag & (int)JointFlag::END_EFFECTOR;
}

template<JointType T>
void Joint<T>::NewComputation() {
	int new_seed = SDL_GetTicks();
	Joint::current_update_seed =
		new_seed == Joint::current_update_seed ? new_seed + 1 : new_seed;
}