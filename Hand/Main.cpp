#include <SDL.h>
#include <iostream>
#include <Window.h>
#include <Scene.h>
#include <Log.h>
#include <Constant.h>
#include <SimulatorScene.h>
#include <RenderingTool.h>
#include <HandGenerator.h>
using namespace std;

int main(int argc, char *argv[]) {
	SDL_Init(SDL_INIT_EVERYTHING);
	Window* win = new Window({"Hand Simulator", Constant::WINDOW_WIDTH, Constant::WINDOW_HEIGHT, Constant::FPS});
	win->Rendering();

	Scene* scene;
	scene = new SimulatorScene();
	win->AddScene(scene, 0);

	Hand* hand = HandGenerator::Generate();

	((SimulatorScene*)scene)->AddHand(hand);

	bool run = true;
	while (run) {
		switch (win->PollEvent()) {
		case QUIT:
			run = false;
			break;
		}
	}
	win->Destroy();
	delete win;
	RenderingTool::ReleaseInstance();
	SDL_Quit();
	return 0;
}