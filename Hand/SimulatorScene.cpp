#include "SimulatorScene.h"
#include "RenderingTool.h"

SimulatorScene::SimulatorScene() {
}

SimulatorScene::~SimulatorScene() {
}

int SimulatorScene::Rendering() {
	for (int i = 0; i < this->hands.size(); i++) {
		this->hands[i]->Calculate();
	}
	if (this->MovingFunction) {
		(*(this->MovingFunction))();
	}

	for (int i = 0; i < this->hands.size(); i++) {
		this->hands[i]->Render(this->ren, static_cast<GraphicMode>(this->rendering_mode));
	}
	RenderingTool* rt = RenderingTool::GetInstance();
	if (this->rendering_mode == GraphicMode::SOLID) {
		rt->Render(this->ren);
	}
	else {
		rt->Clear();
	}
	return 0;
}

void SimulatorScene::AddHand(Hand* hand) {
	this->hands.push_back(hand);
}

int SimulatorScene::EventProcess(Event& evt) {
	if (evt.type == EventType::KEY_UP) {
		if (evt.key == SDLK_ESCAPE) {
			this->rendering_mode++;
			this->rendering_mode %= 3;
		}
		else if (evt.key == SDLK_SPACE) {
			this->moving_value *= -1;
		}
	}
	else if (evt.type == EventType::KEY_DOWN) {
		if (evt.key == SDLK_h) {
			for (int i = 0; i < this->hands.size(); i++) {
				if (this->hands[i]->clicking_motion_function_set.size() < 1) continue;
				(*this->hands[i]->clicking_motion_function_set[0])();
			}
		}
		else if (evt.key == SDLK_j) {
			for (int i = 0; i < this->hands.size(); i++) {
				if (this->hands[i]->clicking_motion_function_set.size() < 2) continue;
				(*this->hands[i]->clicking_motion_function_set[1])();
			}
		}
		else if (evt.key == SDLK_k) {
			for (int i = 0; i < this->hands.size(); i++) {
				if (this->hands[i]->clicking_motion_function_set.size() < 3) continue;
				(*this->hands[i]->clicking_motion_function_set[2])();
			}
		}
		else if (evt.key == SDLK_l) {
			for (int i = 0; i < this->hands.size(); i++) {
				if (this->hands[i]->clicking_motion_function_set.size() < 4) continue;
				(*this->hands[i]->clicking_motion_function_set[3])();
			}
		}
		else if (evt.key == SDLK_SEMICOLON) {
			for (int i = 0; i < this->hands.size(); i++) {
				if (this->hands[i]->clicking_motion_function_set.size() < 5) continue;
				(*this->hands[i]->clicking_motion_function_set[4])();
			}
		}
	}
	else if (evt.type == EventType::MOUSE_DOWN) {
		this->pressed_mouse = evt.mouse;
		int x = evt.x;
		int y = evt.y;
		for (int i = 0; i < this->hands.size(); i++) {
			auto joints = this->hands[i]->GetJoints();
			for (int j = 0; j < joints.size(); j++) {
				double joint_x, joint_y, joint_z;
				if (joints[j]->GetParentJoint() == nullptr) continue;
				joints[j]->GetParentJoint()->GetPosition(joint_x, joint_y, joint_z);
				if (x > joint_x - 5 && x < joint_x + 5 && y > joint_y - 5 && y < joint_y + 5) {
					Joint<>* joint = joints[j];
					if (this->pressed_mouse == MOUSE_LEFT) {
						if (joint->Type() == JointType::PRIMARY 
							|| joint->Type() == JointType::SECONDARY 
							|| joint->Type() == JointType::TIRTARY)
							this->MovingFunction = new function<void()>([=]() {joint->SetAngle(this->moving_value, 0, 0); });
					}
					else if (this->pressed_mouse == MOUSE_RIGHT) {
						if (joint->Type() == JointType::SECONDARY 
							|| joint->Type() == JointType::TIRTARY)
							this->MovingFunction = new function<void()>([=]() {joint->SetAngle(0, 0, this->moving_value); });
					}
					goto ESCAPE;
				}
			}
		}
		ESCAPE:;
	}
	else if (evt.type == EventType::MOUSE_UP) {
		delete this->MovingFunction;
		this->MovingFunction = nullptr;
		this->pressed_mouse = MOUSE_NONE;
	}
	else if (evt.type == EventType::MOUSE_MOVE) {
		delete this->MovingFunction;
		this->MovingFunction = nullptr;
		if (this->pressed_mouse != MOUSE_NONE) {
			for (int i = 0; i < this->hands.size(); i++) {
				this->hands[i]->SetOrientation(evt.y_rel, evt.x_rel, 0);
			}
		}
	}
	return 0;
}

int SimulatorScene::NormalProcess() {
	return 0;
}
