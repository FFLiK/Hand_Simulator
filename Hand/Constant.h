#pragma once

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
}

class Constant {
public:
	// Program Status
	const static int WINDOW_WIDTH = 1280;
	const static int WINDOW_HEIGHT = 720;
	const static int ORIGIN_X = WINDOW_WIDTH / 2;
	const static int ORIGIN_Y = WINDOW_HEIGHT / 2 + 300;
	const static int FPS = 60;
	const static int FRAME_DELAY = 1000 / FPS;

	const static int LINE_DENSITY_UNIT = 3;
	const static int LINE_WIDTH = 80;

	// Mathematically Constant
	constexpr static double PI = 3.14159265358979323846;
	constexpr static double RAD(double deg) { return deg * PI / 180; }
	constexpr static double DEG(double rad) { return rad * 180 / PI; }
};