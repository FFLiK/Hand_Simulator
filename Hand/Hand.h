#pragma once
#include <Joint.h>
#include <Muscle.h>
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
	std::vector<Muscle*> muscles;

	Vector3D orientation;

public:
	Hand();
	~Hand();

	Hand* AddJoint(Joint<>* joint);
	Hand* AddMuscle(Muscle* muscle);
	void Compute();
	void Render(SDL_Renderer* renderer, GraphicMode mode);

	std::vector<Joint<>*> GetJoints();

	void SetOrientation(double x, double y, double z);

	std::vector<function<void()>*> press_motion_function_set;
	std::vector<function<void()>*> release_motion_function_set;
};