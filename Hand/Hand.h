#pragma once
#include <Joint.h>
#include <vector>
#include <Window.h>
#include <functional>

enum GraphicMode {
	FRAME,
	SPECIFIC_FRAME,
	SOLID
};

class Hand {
private:
	std::vector<Joint<>*> joints;

	double orientation_x = 0, orientation_y = 0, orientation_z = 0;

public:
	Hand();
	~Hand();

	Hand* AddJoint(Joint<>* joint);
	void Calculate();
	void Render(SDL_Renderer* renderer, GraphicMode mode);

	std::vector<Joint<>*> GetJoints();

	void SetOrientation(double x, double y, double z);

	std::vector<function<void()>*> clicking_motion_function_set;
};