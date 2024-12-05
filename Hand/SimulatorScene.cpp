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
		else if (evt.key == SDLK_RETURN) {
			this->manual_mode = !this->manual_mode;
			Log::Hand("Manual Mode : ", this->manual_mode);
		}
		if (!this->manual_mode) {
			if (evt.key == SDLK_SPACE) {
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
	}
	else if (evt.T == EventType::KEY_DOWN) {
		if (this->manual_mode) {
			if (this->hands[0]->motion_function_set.size() == 36) {
				switch (evt.key) {
					// Flexor Digitorum Superficialis : ¾èÀº ¼Õ°¡¶ô ±ÁÈù±Ù
				case SDLK_q: (*this->hands[0]->motion_function_set[0])(); break;
				case SDLK_w: (*this->hands[0]->motion_function_set[1])(); break;
				case SDLK_e: (*this->hands[0]->motion_function_set[2])(); break;
				case SDLK_r: (*this->hands[0]->motion_function_set[3])(); break;
					// Flexor Digitorum Profundus : ±íÀº ¼Õ°¡¶ô ±ÁÈù±Ù
				case SDLK_a: (*this->hands[0]->motion_function_set[4])(); break;
				case SDLK_s: (*this->hands[0]->motion_function_set[5])(); break;
				case SDLK_d: (*this->hands[0]->motion_function_set[6])(); break;
				case SDLK_f: (*this->hands[0]->motion_function_set[7])(); break;
					// Extensor Digitorum : ¼Õ°¡¶ô Æï±Ù
				case SDLK_z: (*this->hands[0]->motion_function_set[8])(); break;
				case SDLK_x: (*this->hands[0]->motion_function_set[9])(); break;
				case SDLK_c: (*this->hands[0]->motion_function_set[10])(); break;
				case SDLK_v: (*this->hands[0]->motion_function_set[11])(); break;
					//  Flexor Pollicis Longus : ±ä¾öÁö±ÁÈû±Ù
				case SDLK_t: (*this->hands[0]->motion_function_set[12])(); break;
					//  Aductor Pollicis Longus : ±ä¾öÁö¹ú¸²±Ù
				case SDLK_g: (*this->hands[0]->motion_function_set[13])(); break;
					//  Extensor Pollicis Longus : ±ä¾öÁöÆï±Ù
				case SDLK_b: (*this->hands[0]->motion_function_set[14])(); break;
					//  Extensor Pollicis Brevis : ÂªÀº¾öÁöÆï±Ù
				case SDLK_y: (*this->hands[0]->motion_function_set[15])(); break;
					//  Extensor Indicis : °ËÁöÆï±Ù
				case SDLK_h: (*this->hands[0]->motion_function_set[16])(); break;
					//  Extensor Digiti Minimi : »õ³¢Æï±Ù
				case SDLK_n: (*this->hands[0]->motion_function_set[17])(); break;
					// Aductor Pollicis Brevis : ÂªÀº¾öÁö¹ú¸²±Ù
				case SDLK_u: (*this->hands[0]->motion_function_set[18])(); break;
					// Flexor Pollicis Brevis : ÂªÀº¾öÁö±ÁÈû±Ù
				case SDLK_j: (*this->hands[0]->motion_function_set[19])(); break;
					// Opponens Pollicis : ¾öÁö¸Â¼¶±Ù
				case SDLK_m: (*this->hands[0]->motion_function_set[20])(); break;
					// Adductor Pollicis : ¾öÁö¸ðÀ½±Ù
				case SDLK_i: (*this->hands[0]->motion_function_set[21])(); break;
					// Adductor Digiti Minimi : »õ³¢¹ú¸²±Ù
				case SDLK_k: (*this->hands[0]->motion_function_set[22])(); break;
					// Flexor Digiti Minimi Brevis : ÂªÀº»õ³¢±ÁÈû±Ù
				case SDLK_o: (*this->hands[0]->motion_function_set[23])(); break;
					// Opponens Digiti Minimi : »õ³¢¸Â¼¶±Ù
				case SDLK_l: (*this->hands[0]->motion_function_set[24])(); break;
					// Lumbricals : ¹ú·¹±Ù
				case SDLK_1: (*this->hands[0]->motion_function_set[25])(); break;
				case SDLK_2: (*this->hands[0]->motion_function_set[26])(); break;
				case SDLK_3: (*this->hands[0]->motion_function_set[27])(); break;
				case SDLK_4: (*this->hands[0]->motion_function_set[28])(); break;
					// Dorsal Interossei : µîÂÊ»À»çÀÌ±Ù
				case SDLK_5: (*this->hands[0]->motion_function_set[29])(); break;
				case SDLK_6: (*this->hands[0]->motion_function_set[30])(); break;
				case SDLK_7: (*this->hands[0]->motion_function_set[31])(); break;
				case SDLK_8: (*this->hands[0]->motion_function_set[32])(); break;
					// Palmar Interossei : ¼Õ¹Ù´Ú»À»çÀÌ±Ù
				case SDLK_9: (*this->hands[0]->motion_function_set[33])(); break;
				case SDLK_0: (*this->hands[0]->motion_function_set[34])(); break;
				case SDLK_MINUS: (*this->hands[0]->motion_function_set[35])(); break;
				}
			}
		}
		else {
			if (SDLK_0 <= evt.key && evt.key <= SDLK_9) {
				int NUM = evt.key - SDLK_0;
				if (NUM == 0) NUM = 10;
				NUM--;
				for (int i = 0; i < this->hands.size(); i++) {
					if (this->hands[i]->press_motion_function_set.size() <= NUM) continue;
					(*this->hands[i]->press_motion_function_set[NUM])();
				}
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
