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

using DH_Matrix = Eigen::Matrix<double, 4, 4>;
const DH_Matrix EMPTY_MAT = Eigen::Matrix<double, 4, 4>::Identity();

template<JointType type = JointType::NONE>
class Joint {
protected:
	JointType TYPE = JointType::NONE;

	struct Rotation { double x, y, z; };
	struct Position { double x, y, z; };

	// Input
	double distance;
	Rotation rotation;

	// Output
	Position position;

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
	
	void Calculate();
	void GetPosition(double& x, double& y, double& z);
	Joint<>* SetPosition(double x, double y, double z);
	Joint<>* GetParentJoint();
	Joint<>* InitAngle(double x_rot, double y_rot, double z_rot);
	Joint<>* SetRangeX(double min, double max);
	Joint<>* SetRangeY(double min, double max);
	Joint<>* SetRangeZ(double min, double max);
	Joint<>* SetFlag(JointFlag flag);
	void SetAngle(double x_rot, double y_rot, double z_rot);
	void SetAngleAsProgress(double x_rot, double y_rot, double z_rot);

	JointType Type() const;

	bool IsRealJoint();
	bool IsEndEffector();

	static void NewCalculation();

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
	Joint<JointType::PRIMARY>* InitAngle(double x_rot, double y_rot, double z_rot) {
		this->rotation.x += Constant::RAD(x_rot);
		this->rotation.y += Constant::RAD(y_rot);
		this->rotation.z += Constant::RAD(z_rot);
		this->SetRangeX(0, 0);
		this->SetRangeY(0, 0);
		this->SetRangeZ(0, 0);
		return this;
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
	Joint<JointType::SECONDARY>* InitAngle(double x_rot, double y_rot, double z_rot) {
		this->rotation.x += Constant::RAD(x_rot);
		this->rotation.y += Constant::RAD(y_rot);
		this->rotation.z += Constant::RAD(z_rot);
		this->SetRangeX(0, 0);
		this->SetRangeY(0, 0);
		this->SetRangeZ(0, 0);
		return this;
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
	Joint<JointType::TIRTARY>* InitAngle(double x_rot, double y_rot, double z_rot) {
		this->rotation.x += Constant::RAD(x_rot);
		this->rotation.y += Constant::RAD(y_rot);
		this->rotation.z += Constant::RAD(z_rot);
		this->SetRangeX(0, 0);
		this->SetRangeY(0, 0);
		this->SetRangeZ(0, 0);
		return this;
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
void Joint<T>::Calculate() {
	if (this->update_seed != Joint::current_update_seed) {
		// Parent Calculation
		DH_Matrix parent_matrix;
		if (this->parent_joint) {
			this->parent_joint->Calculate();
			parent_matrix = this->parent_joint->dh_matrix;
		}
		else {
			parent_matrix = EMPTY_MAT;
		}

		// Denavit-Hartenberg Matrix Calculation Functiuon (Lambda)
		auto Tz_Generator = [](double d_i) -> DH_Matrix {
			DH_Matrix Tz;
			Tz << 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, d_i,
				0, 0, 0, 1;
			return Tz;
		};

		auto Rz_Generator = [](double theta_i) -> DH_Matrix {
			DH_Matrix Rz;
			Rz << cos(theta_i), -sin(theta_i), 0, 0,
				sin(theta_i), cos(theta_i), 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;
			return Rz;
		};

		auto Tx_Generator = [](double a_i) -> DH_Matrix {
			DH_Matrix Tx;
			Tx << 1, 0, 0, a_i,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;
			return Tx;
		};

		auto Rx_Generator = [](double alpha_i) -> DH_Matrix {
			DH_Matrix Rx;
			Rx << 1, 0, 0, 0,
				0, cos(alpha_i), -sin(alpha_i), 0,
				0, sin(alpha_i), cos(alpha_i), 0,
				0, 0, 0, 1;
			return Rx;
		};

		// Calculating
		DH_Matrix Tz, Rz, Tx, Rx;
		double d_i, theta_i, a_i, alpha_i;

		d_i = 0;
		theta_i = this->rotation.x;
		a_i = 0;
		if (!this->parent_joint) alpha_i = 0;
		else alpha_i = Constant::PI * 0.5;
		
		Tz = Tz_Generator(d_i);
		Rz = Rz_Generator(theta_i);
		Tx = Tx_Generator(a_i);
		Rx = Rx_Generator(alpha_i);

		DH_Matrix M_x = Tz * Rz * Tx * Rx;

		d_i = 0; 
		theta_i = this->rotation.z; 
		a_i = this->distance; 
		alpha_i = -Constant::PI * 0.5 + this->rotation.y;

		Tz = Tz_Generator(d_i);
		Rz = Rz_Generator(theta_i);
		Tx = Tx_Generator(a_i);
		Rx = Rx_Generator(alpha_i);

		DH_Matrix M_z = Tz * Rz * Tx * Rx;

		this->dh_matrix = parent_matrix * M_x * M_z;

		this->position = {
			this->dh_matrix(0, 3),
			this->dh_matrix(1, 3),
			this->dh_matrix(2, 3)
		};
		
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
Joint<>* Joint<type>::InitAngle(double x_rot, double y_rot, double z_rot) {
	this->rotation.x = Constant::RAD(x_rot);
	this->rotation.y = Constant::RAD(y_rot);
	this->rotation.z = Constant::RAD(z_rot);
	this->SetRangeX(0, 0);
	this->SetRangeY(0, 0);
	this->SetRangeZ(0, 0);
	return this;
}

template<JointType type>
inline Joint<>* Joint<type>::SetRangeX(double min, double max) {
	min = Constant::RAD(min);
	max = Constant::RAD(max);
	this->range_x_min = min + this->rotation.x;
	this->range_x_max = max + this->rotation.x;
	return this;
}

template<JointType type>
inline Joint<>* Joint<type>::SetRangeY(double min, double max) {
	min = Constant::RAD(min);
	max = Constant::RAD(max);
	this->range_y_min = min + this->rotation.y;
	this->range_y_max = max + this->rotation.y;
	return this;
}

template<JointType type>
inline Joint<>* Joint<type>::SetRangeZ(double min, double max) {
	min = Constant::RAD(min);
	max = Constant::RAD(max);
	this->range_z_min = min + this->rotation.z;
	this->range_z_max = max + this->rotation.z;
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
inline Joint<>* Joint<type>::SetFlag(JointFlag flag) {
	this->flag = static_cast<int>(flag);
	return this;
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
void Joint<T>::NewCalculation() {
	int new_seed = SDL_GetTicks();
	Joint::current_update_seed =
		new_seed == Joint::current_update_seed ? new_seed + 1 : new_seed;
}
