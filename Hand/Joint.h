#pragma once
#ifdef LOG_H
#else
#define LOG_H
#include <Log.h>
#endif
#ifdef CONSTANT_H
#else
#define CONSTANT_H
#include <Constant.h>
#endif
#ifdef EIGEN_DENSE_H
#else
#include <Eigen/Dense>
#define EIGEN_DENSE_H
#endif
#ifdef SDL_H
#else
#include <SDL.h>
#define SDL_H
#endif


enum class JointType {
	NONE, PRIMARY, SECONDARY, TIRTARY
};

enum class JointFlag {
	REAL_JOINT = 0, VIRTUAL_JOINT = 1, END_EFFECTOR = 2
};

template<JointType type = JointType::NONE>
class Joint {
protected:
	JointType TYPE = JointType::NONE;

	// Initial Value
	Vector3D initial_angle;

	// Input
	double distance;
	Vector3D rotation;

	// Output
	Vector3D position;

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
	~Joint();
	
	void Compute();
	void GetPosition(double& x, double& y, double& z);
	Joint<>* SetPosition(double x, double y, double z);
	Joint<>* GetParentJoint();
	Joint<>* GetAngle(double& x_rot, double& y_rot, double& z_rot);
	Joint<>* InitAngle(double x_rot, double y_rot, double z_rot);
	Joint<>* SetRangeX(double min, double max);
	Joint<>* SetRangeY(double min, double max);
	Joint<>* SetRangeZ(double min, double max);
	Joint<>* SetFlag(JointFlag flag);
	void SetAngle(double x_rot, double y_rot, double z_rot);
	void SetAngleAsProgress(double x_rot, double y_rot, double z_rot);
	void SetForce(double x, double y, double z);

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
	Joint(double distance, Joint<>* parent = nullptr) : Joint<>(distance, parent) {
		this->TYPE = JointType::PRIMARY;
	};
	void SetAngle(double x_rot) {
		this->rotation.x += Constant::RAD(x_rot);
		this->AdjustOverAngle();
	}
	void SetAngleAsProgress(double x_rot) {
		this->rotation.x = this->range_x_min + (this->range_x_max - this->range_x_min) * x_rot;
	}
};

template<>
class Joint<JointType::SECONDARY> : public Joint<> {
public:
	Joint(double distance, Joint<>* parent = nullptr) : Joint<>(distance, parent) {
		this->TYPE = JointType::SECONDARY;
	};
	void SetAngle(double x_rot, double z_rot) {
		this->rotation.x += Constant::RAD(x_rot);
		this->rotation.z += Constant::RAD(z_rot);
		this->AdjustOverAngle();
	}
	void SetAngleAsProgress(double x_rot, double z_rot) {
		this->rotation.x = this->range_x_min + (this->range_x_max - this->range_x_min) * x_rot;
		this->rotation.z = this->range_z_min + (this->range_z_max - this->range_z_min) * z_rot;
	}
};

template<>
class Joint<JointType::TIRTARY> : public Joint<> {
public:
	Joint(double distance, Joint<>* parent = nullptr) : Joint<>(distance, parent) {
		this->TYPE = JointType::TIRTARY;
	};
	void SetAngle(double x_rot, double y_rot, double z_rot) {
		this->rotation.x += Constant::RAD(x_rot);
		this->rotation.y += Constant::RAD(y_rot);
		this->rotation.z += Constant::RAD(z_rot);
		this->AdjustOverAngle();
	}
	void SetAngleAsProgress(double x_rot, double y_rot, double z_rot) {
		this->rotation.x = this->range_x_min + (this->range_x_max - this->range_x_min) * x_rot;
		this->rotation.y = this->range_y_min + (this->range_y_max - this->range_y_min) * y_rot;
		this->rotation.z = this->range_z_min + (this->range_z_max - this->range_z_min) * z_rot;
	}
};

template<JointType T>
int Joint<T>::current_update_seed = 0;

template<JointType type>
inline void Joint<type>::AdjustOverAngle() {
	if (this->rotation.x > this->range_x_max) this->rotation.x = this->range_x_max;
	if (this->rotation.x < this->range_x_min) this->rotation.x = this->range_x_min;
	if (this->rotation.y > this->range_y_max) this->rotation.y = this->range_y_max;
	if (this->rotation.y < this->range_y_min) this->rotation.y = this->range_y_min;
	if (this->rotation.z > this->range_z_max) this->rotation.z = this->range_z_max;
	if (this->rotation.z < this->range_z_min) this->rotation.z = this->range_z_min;
}

template<JointType T>
Joint<T>::Joint(double distance, Joint* parent) {
	if (parent == this) {
		throw "Joint cannot be parent of itself";
	}
	if (!parent) 
		distance = 0;

	this->flag = 0;
	
	this->parent_joint = parent;

	this->rotation = { 0, 0, 0 };
	this->position = { 0, 0, 0 };
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
	double FORCE_AMPLIFICATION_FACTOR = 5.0;
	this->force.x *= FORCE_AMPLIFICATION_FACTOR;
	this->force.y *= FORCE_AMPLIFICATION_FACTOR;
	this->force.z *= FORCE_AMPLIFICATION_FACTOR;

	double x_delta = (Constant::DEG(this->initial_angle.x) - Constant::DEG(this->rotation.x)) / 360;
	double y_delta = (Constant::DEG(this->initial_angle.y) - Constant::DEG(this->rotation.y)) / 360;
	double z_delta = (Constant::DEG(this->initial_angle.z) - Constant::DEG(this->rotation.z)) / 360;
	
	this->force.x += x_delta;
	this->force.y += y_delta;
	this->force.z += z_delta;

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
		
		this->update_seed = Joint::current_update_seed;
	}
}

template<JointType T>
void Joint<T>::GetPosition(double& x, double& y, double& z) {
	x = this->position.x;
	y = this->position.y;
	z = this->position.z;
}

template<JointType type>
inline Joint<>* Joint<type>::SetPosition(double x, double y, double z) {
	this->position.x = x;
	this->position.y = y;
	this->position.z = z;
	return this;
}

template<JointType type>
Joint<JointType::NONE>* Joint<type>::GetParentJoint() {
	return this->parent_joint;
}

template<JointType type>
inline Joint<>* Joint<type>::GetAngle(double& x_rot, double& y_rot, double& z_rot) {
	x_rot = Constant::DEG(this->rotation.x);
	y_rot = Constant::DEG(this->rotation.y);
	z_rot = Constant::DEG(this->rotation.z);
	return this;
}

template<JointType type>
Joint<>* Joint<type>::InitAngle(double x_rot, double y_rot, double z_rot) {
	this->initial_angle.x = Constant::RAD(x_rot);
	this->initial_angle.y = Constant::RAD(y_rot);
	this->initial_angle.z = Constant::RAD(z_rot);

	this->rotation = this->initial_angle;

	this->SetRangeX(0, 0);
	this->SetRangeY(0, 0);
	this->SetRangeZ(0, 0);
	return this;
}

template<JointType type>
inline Joint<>* Joint<type>::SetRangeX(double min, double max) {
	min = Constant::RAD(min);
	max = Constant::RAD(max);
	this->range_x_min = min;
	this->range_x_max = max;
	return this;
}

template<JointType type>
inline Joint<>* Joint<type>::SetRangeY(double min, double max) {
	min = Constant::RAD(min);
	max = Constant::RAD(max);
	this->range_y_min = min;
	this->range_y_max = max;
	return this;
}

template<JointType type>
inline Joint<>* Joint<type>::SetRangeZ(double min, double max) {
	min = Constant::RAD(min);
	max = Constant::RAD(max);
	this->range_z_min = min;
	this->range_z_max = max;
	return this;
}

template<JointType type>
void Joint<type>::SetAngle(double x_rot, double y_rot, double z_rot) {
	this->rotation.x += Constant::RAD(x_rot);
	this->rotation.y += Constant::RAD(y_rot);
	this->rotation.z += Constant::RAD(z_rot);
	this->AdjustOverAngle();
}

template<JointType type>
void Joint<type>::SetAngleAsProgress(double x_rot, double y_rot, double z_rot) {
	this->rotation.x = this->range_x_min + (this->range_x_max - this->range_x_min) * x_rot;
	this->rotation.y = this->range_y_min + (this->range_y_max - this->range_y_min) * y_rot;
	this->rotation.z = this->range_z_min + (this->range_z_max - this->range_z_min) * z_rot;
	this->AdjustOverAngle();
}

template<JointType type>
inline void Joint<type>::SetForce(double x, double y, double z) {
	this->force.x += x;
	this->force.y += y;
	this->force.z += z;
}

template<JointType type>
inline Joint<>* Joint<type>::SetFlag(JointFlag flag) {
	this->flag = static_cast<int>(flag);
	return this;
}

template<JointType type>
inline DH_Matrix Joint<type>::GetDHMatrix() {
	return this->dh_matrix;
}

template<JointType type>
inline JointType Joint<type>::Type() const {
	return this->TYPE;
}

template<JointType type>
inline bool Joint<type>::IsRealJoint() {
	return !(this->flag & (int)JointFlag::VIRTUAL_JOINT);
}

template<JointType type>
inline bool Joint<type>::IsEndEffector() {
	return this->flag & (int)JointFlag::END_EFFECTOR;
}

template<JointType T>
void Joint<T>::NewComputation() {
	int new_seed = SDL_GetTicks();
	Joint::current_update_seed =
		new_seed == Joint::current_update_seed ? new_seed + 1 : new_seed;
}
