#include "Constant.h"
#include <iostream>
using namespace std;
using namespace GraphicColor;

#define RET_HSV(h, s, v) {(int)h, (int)(s * 100.0), (int)(v * 100.0)}
#define RET_RGB(r, g, b) {(int)(r * 255.0), (int)(g * 255.0), (int)(b * 255.0)}

inline HSV RGBtoHSV(RGB in) {
	double h, s, v;
	double r, g, b;

	r = (double)in.r / 255.0;
	g = (double)in.g / 255.0;
	b = (double)in.b / 255.0;

	double min, max, delta;

	min = r < g ? r : g;
	min = min < b ? min : b;

	max = r > g ? r : g;
	max = max > b ? max : b;

	v = max;                                // v
	delta = max - min;
	if (max > 0.0) { // NOTE: if Max is == 0, this divide would cause a crash
		s = (delta / max);                  // s
	}
	else {
		// if max is 0, then r = g = b = 0
		// s = 0, v is undefined
		s = 0.0;
		h = 0.0;                            // its now undefined
		return RET_HSV(h, s, v);
	}
	if (r >= max)                           // > is bogus, just keeps compilor happy
		if (delta == 0) {
			h = 0.0;
		}
		else {
			h = (double)(g - b) / delta;        // between yellow & magenta
		}
	else
		if (g >= max)
			h = 2.0 + (double)(b - r) / delta;  // between cyan & yellow
		else
			h = 4.0 + (double)(r - g) / delta;  // between magenta & cyan

	h *= 60.0;                              // degrees

	if (h < 0.0)
		h += 360.0;

	return RET_HSV(h, s, v);
}

inline RGB HSVtoRGB(HSV in) {
	double h, s, v;
	double r, g, b;

	h = (double)in.h;
	s = (double)in.s / 100.0;
	v = (double)in.v / 100.0;

	double      hh, p, q, t, ff;
	long        i;

	if (s <= 0.0) {       // < is bogus, just shuts up warnings
		r = v;
		g = v;
		b = v;
		return RET_RGB(r, g, b);
	}
	hh = h;
	if (hh >= 360.0) hh = 0.0;
	hh /= 60.0;
	i = (long)hh;
	ff = hh - i;
	p = (double)v * (1.0 - (double)s);
	q = (double)v * (1.0 - ((double)s * ff));
	t = (double)v * (1.0 - ((double)s * (1.0 - ff)));

	switch (i) {
	case 0:
		r = v;
		g = t;
		b = p;
		break;
	case 1:
		r = q;
		g = v;
		b = p;
		break;
	case 2:
		r = p;
		g = v;
		b = t;
		break;

	case 3:
		r = p;
		g = q;
		b = v;
		break;
	case 4:
		r = t;
		g = p;
		b = v;
		break;
	case 5:
	default:
		r = v;
		g = p;
		b = q;
		break;
	}
	return RET_RGB(r, g, b);
}

GraphicColor::RGB::RGB(int r, int g, int b, int a) {
	this->r = r;
	this->g = g;
	this->b = b;
}

GraphicColor::RGB::RGB(const RGB& rgb) {
	this->r = rgb.r;
	this->g = rgb.g;
	this->b = rgb.b;
}

GraphicColor::RGB::RGB(HSV& hsv) {
	*this = HSVtoRGB(hsv);
}

HSV GraphicColor::RGB::to_HSV() const {
	return RGBtoHSV(*this);
}

GraphicColor::HSV::HSV(int h, int s, int v) {
	this->h = h;
	this->s = s;
	this->v = v;
}

GraphicColor::HSV::HSV(const HSV& hsv) {
	this->h = hsv.h;
	this->s = hsv.s;
	this->v = hsv.v;
}

GraphicColor::HSV::HSV(RGB& rgb) {
	*this = RGBtoHSV(rgb);
}

RGB GraphicColor::HSV::to_RGB() const {
	return HSVtoRGB(*this);
}

DH_Matrix Calculate::DH_Parameters(Vector3D angle, double distance, DH_Matrix prev_matrix, bool is_first) {
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
	theta_i = angle.x;
	a_i = 0;
	if (is_first) alpha_i = 0;
	else alpha_i = Constant::PI * 0.5;

	Tz = Tz_Generator(d_i);
	Rz = Rz_Generator(theta_i);
	Tx = Tx_Generator(a_i);
	Rx = Rx_Generator(alpha_i);

	DH_Matrix M_x = Tz * Rz * Tx * Rx;

	d_i = 0;
	theta_i = angle.z;
	a_i = distance;
	alpha_i = -Constant::PI * 0.5 + angle.y;

	Tz = Tz_Generator(d_i);
	Rz = Rz_Generator(theta_i);
	Tx = Tx_Generator(a_i);
	Rx = Rx_Generator(alpha_i);

	DH_Matrix M_z = Tz * Rz * Tx * Rx;

	DH_Matrix dh_matrix = prev_matrix * M_x * M_z;

	return dh_matrix;
}

Vector3D Calculate::DHMatrixToPosition(DH_Matrix dh_matrix) {
	Vector3D position;
	position.x = dh_matrix(0, 3);
	position.y = dh_matrix(1, 3);
	position.z = dh_matrix(2, 3);

	return position;
}

Eigen::Matrix3d Calculate::RotationMatrix(Vector3D orientation) {
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix =
		Eigen::AngleAxisd(orientation.x, Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(orientation.y, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(orientation.z, Eigen::Vector3d::UnitZ());
	return rotation_matrix;
}

Vector3D Calculate::Rotate(Vector3D point, Eigen::Matrix3d rotation_matrix) {
	Eigen::Vector3d original_vector(point.x, point.y, point.z);
	Eigen::Vector3d rotated_vector = original_vector.transpose() * rotation_matrix;
	point.x = rotated_vector.x();
	point.y = rotated_vector.y();
	point.z = rotated_vector.z();
	return point;
}

