#pragma once
#include <SDL.h>
#include <Eigen/Dense>

namespace GraphicColor {

	class RGB;
	class HSV;

	class RGB {
	public:
		RGB(int r, int g, int b, int a = 0);
		RGB(const RGB& rgb);
		RGB(HSV& hsv);
		HSV to_HSV() const;

		int r;
		int g;
		int b;
	};

	class HSV {
	public:
		HSV(int h, int s, int v);
		HSV(const HSV& hsv);
		HSV(RGB& rgb);
		RGB to_RGB() const;

		int h;
		int s;
		int v;
	};

	const GraphicColor::HSV BACKGROUND_COLOR = GraphicColor::HSV(0, 0, 0);
	const GraphicColor::RGB BACKGROUND_COLOR_RGB = BACKGROUND_COLOR.to_RGB();
	const GraphicColor::HSV FRAME_COLOR(0, 100, 100);
	const GraphicColor::RGB FRAME_COLOR_RGB = FRAME_COLOR.to_RGB();
	const GraphicColor::HSV FLESH_COLOR(0, 0, 100);
	const GraphicColor::RGB FLESH_COLOR_RGB = FLESH_COLOR.to_RGB();
	const GraphicColor::HSV BONE_COLOR(0, 0, 80);
	const GraphicColor::RGB BONE_COLOR_RGB = BONE_COLOR.to_RGB();
	const GraphicColor::HSV JOINT_COLOR(275, 30, 100);
	const GraphicColor::RGB JOINT_COLOR_RGB = JOINT_COLOR.to_RGB();
	const GraphicColor::HSV FALSE_COLOR(0, 0, 30);
	const GraphicColor::RGB FALSE_COLOR_RGB = FALSE_COLOR.to_RGB();
	const GraphicColor::HSV MUSCLE_COLOR(0, 30, 75);
	const GraphicColor::RGB MUSCLE_COLOR_RGB = MUSCLE_COLOR.to_RGB();
}

namespace Constant {
	// Program Status
	const int WINDOW_WIDTH = 1280;
	const int WINDOW_HEIGHT = 720;
	const int ORIGIN_X = WINDOW_WIDTH / 2;
	const int ORIGIN_Y = WINDOW_HEIGHT / 2 + 300;
	const int FPS = 60;
	const int FRAME_DELAY = 1000 / FPS;

	const int LINE_DENSITY_UNIT = 3;
	const int LINE_WIDTH = 80;

	// Mathematically Constant
	constexpr double PI = 3.14159265358979323846;
	constexpr double RAD(double deg) { return deg * PI / 180; }
	constexpr double DEG(double rad) { return rad * 180 / PI; }
};

namespace HandParameter {
	const double MUSCLE_FORCE_AMPLIFICATION_FACTOR = 10.0;
	const double NEUTRAL_FORCE_AMPLIFICATION_FACTOR = 1.0;

	const double MUSCLE_UNIT_DEPENDENCY = 0.6;
	const double MUSCLE_CONTRACTING_ANGLE_SUM_APLIFICATION_CONSTANT = 20;
};

class Vector3D {
public:
	Vector3D() : x(0), y(0), z(0) {}
	Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}
	Vector3D(const Vector3D& vec) : x(vec.x), y(vec.y), z(vec.z) {}

	double x;
	double y;
	double z;
};

using DH_Matrix = Eigen::Matrix<double, 4, 4>;
const DH_Matrix EMPTY_MAT = Eigen::Matrix<double, 4, 4>::Identity();

namespace Calculate {
	DH_Matrix DH_Parameters(Vector3D angle, double distance, DH_Matrix prev_matrix, bool is_first);
	Vector3D DHMatrixToPosition(DH_Matrix dh_matrix);

	Eigen::Matrix3d RotationMatrix(Vector3D orientation);
	Vector3D Rotate(Vector3D point, Eigen::Matrix3d rotation_matrix);
};