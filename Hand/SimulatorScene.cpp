#include "SimulatorScene.h"
#include "RenderingTool.h"
#include "Log.h"

SimulatorScene::SimulatorScene() {
}

SimulatorScene::~SimulatorScene() {
}

int SimulatorScene::Rendering() {
	for (int i = 0; i < this->hands.size(); i++) {
		this->hands[i]->Compute();
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
	if (evt.T == EventType::KEY_UP) {
		if (evt.key == SDLK_ESCAPE) {
			this->rendering_mode++;
			this->rendering_mode %= 4;
		}
		else if (evt.key == SDLK_SPACE) {
			this->moving_value *= -1;
		}
		else if (SDLK_0 <= evt.key && evt.key <= SDLK_9) {
			int NUM = evt.key - SDLK_0;
			if (NUM == 0) NUM = 10;
			NUM--;
			for (int i = 0; i < this->hands.size(); i++) {
				if (this->hands[i]->release_motion_function_set.size() <= NUM) continue;
				(*this->hands[i]->release_motion_function_set[NUM])();
			}
		}
		else if (SDLK_s == evt.key) {
			for (int i = 0; i < this->hands.size(); i++) {
				vector<Vector3D> pose;
				this->hands[i]->GetCurrentPose(pose);
				this->hands[i]->SetFinalPose(pose);
			}
			Log::Hand("Pose Saved");
		}
		else if (SDLK_p == evt.key) {
			for (int i = 0; i < this->hands.size(); i++) {
				vector<Vector3D> pose;
				this->hands[i]->GetCurrentPose(pose);
				Log::Hand("Current Pose");
				for (int j = 0; j < pose.size(); j++) {
					Log::Hand("¤¤", pose[i].x, pose[i].y, pose[i].z);
				}
			}
		}
		else if (SDLK_f == evt.key) {
			for (int i = 0; i < this->hands.size(); i++)
				this->hands[i]->SetFreeMoving(!this->hands[i]->IsFreeMoving());
			Log::Hand("Free Moving Mode : ", this->hands[0]->IsFreeMoving());
		}
		else if (SDLK_o == evt.key) {
			for (int i = 0; i < this->hands.size(); i++)
				this->hands[i]->Optimization();
		}
		else if (SDLK_d == evt.key) {
			Log::Hand("Difference : ", this->hands[0]->ObjectiveFunction());
		}
	}
	else if (evt.T == EventType::KEY_DOWN) {
		if (SDLK_0 <= evt.key && evt.key <= SDLK_9) {
			int NUM = evt.key - SDLK_0;
			if(NUM == 0) NUM = 10;
			NUM--;
			for (int i = 0; i < this->hands.size(); i++) {
				if (this->hands[i]->press_motion_function_set.size() <= NUM) continue;
				(*this->hands[i]->press_motion_function_set[NUM])();
			}
		}
	}
	else if (evt.T == EventType::MOUSE_DOWN) {
		this->pressed_mouse = evt.mouse;
		int x = evt.x;
		int y = evt.y;
		for (int i = 0; i < this->hands.size(); i++) {
			if (this->hands[i]->IsFreeMoving()) {
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
		}
		ESCAPE:;
	}
	else if (evt.T == EventType::MOUSE_UP) {
		delete this->MovingFunction;
		this->MovingFunction = nullptr;
		this->pressed_mouse = MOUSE_NONE;
	}
	else if (evt.T == EventType::MOUSE_MOVE) {
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
