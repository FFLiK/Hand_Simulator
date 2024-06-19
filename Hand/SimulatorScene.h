#pragma once
#include <Scene.h>
#include <vector>
#include <Hand.h>
#include <functional>
using namespace std;

class SimulatorScene : public Scene {
public:
	SimulatorScene();
	~SimulatorScene();
	int Rendering();

	int rendering_mode = 2;
	EventMouse pressed_mouse = MOUSE_NONE;
	function<void()>* MovingFunction = nullptr;

	void AddHand(Hand* hand);

	double moving_value = 1.0;

private:
	int EventProcess(Event& evt);
	int NormalProcess();

	vector<Hand*> hands;

	int prev_time = 0;
};

