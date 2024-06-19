#include "RenderingTool.h"
#include <Log.h>
#include <algorithm>
using namespace std;

RenderingTool* RenderingTool::instance = new RenderingTool();

RenderingTool::RenderingTool() {
	this->center_x = Constant::ORIGIN_X;
	this->center_y = Constant::ORIGIN_Y;
}

RenderingTool::~RenderingTool() {
	if (this->point_texture) {
		SDL_DestroyTexture(this->point_texture);
	}
}

RenderingTool* RenderingTool::GetInstance() {
	return instance;
}

void RenderingTool::SetCenter(int x, int y) {
	this->center_x = x;
	this->center_y = y;
}

void RenderingTool::DrawPoint(SDL_Renderer* renderer, float x, float y, float z) {
	y *= -1;
	
	x += center_x;
	y += center_y;

	SDL_SetRenderDrawColor(
		renderer, 
		this->point_color.r, 
		this->point_color.g, 
		this->point_color.b, 
		255
	);
	SDL_Rect rect = { x - 5, y - 5, 10, 10 };
	SDL_RenderFillRect(renderer, &rect);
}

void RenderingTool::DrawLine(SDL_Renderer* renderer, float x1, float y1, float z1, float x2, float y2, float z2) {
	y1 *= -1;
	y2 *= -1;
	
	x1 += center_x;
	y1 += center_y;
	x2 += center_x;
	y2 += center_y;
	
	SDL_SetRenderDrawColor(
		renderer,
		this->line_color.r,
		this->line_color.g,
		this->line_color.b,
		255
	);	SDL_RenderDrawLine(renderer, x1, y1, x2, y2);

	int dx = x2 - x1;
	int dy = y2 - y1;
	int dz = z2 - z1;

	int xs = (dx < 0) ? -Constant::LINE_DENSITY_UNIT : Constant::LINE_DENSITY_UNIT;
	int ys = (dy < 0) ? -Constant::LINE_DENSITY_UNIT : Constant::LINE_DENSITY_UNIT;
	int zs = (dz < 0) ? -Constant::LINE_DENSITY_UNIT : Constant::LINE_DENSITY_UNIT;

	dx = abs(dx);
	dy = abs(dy);
	dz = abs(dz);

	int doubleDx = dx * 2;
	int doubleDy = dy * 2;
	int doubleDz = dz * 2;

	if (dx >= dy && dx >= dz) {
		int err1 = doubleDy - dx;
		int err2 = doubleDz - dx;
		for (; (x1 - x2) * xs / Constant::LINE_DENSITY_UNIT < 0; x1 += xs) {
			this->AddPoint(x1, y1, z1);
			if (err1 > 0) {
				y1 += ys;
				err1 -= doubleDx;
			}
			if (err2 > 0) {
				z1 += zs;
				err2 -= doubleDx;
			}
			err1 += doubleDy;
			err2 += doubleDz;
		}
	}
	else if (dy >= dx && dy >= dz) {
		int err1 = doubleDx - dy;
		int err2 = doubleDz - dy;
		for (; (y1 - y2) * ys / Constant::LINE_DENSITY_UNIT < 0; y1 += ys) {
			this->AddPoint(x1, y1, z1);
			if (err1 > 0) {
				x1 += xs;
				err1 -= doubleDy;
			}
			if (err2 > 0) {
				z1 += zs;
				err2 -= doubleDy;
			}
			err1 += doubleDx;
			err2 += doubleDz;
		}
	}
	else {
		int err1 = doubleDy - dz;
		int err2 = doubleDx - dz;
		for (; (z1 - z2) * zs / Constant::LINE_DENSITY_UNIT < 0; z1 += zs) {
			this->AddPoint(x1, y1, z1);
			if (err1 > 0) {
				y1 += ys;
				err1 -= doubleDz;
			}
			if (err2 > 0) {
				x1 += xs;
				err2 -= doubleDz;
			}
			err1 += doubleDy;
			err2 += doubleDx;
		}
	}
	this->AddPoint(x1, y1, z1);
}

void RenderingTool::AddPoint(float x, float y, float z) {
	Point p = { x, y, z };
	if (find(this->points.begin(), this->points.end(), p) == this->points.end()) {
		auto iter = lower_bound(this->points.begin(), this->points.end(), p);
		this->points.insert(iter, p);
	}
}

void SetColorByDepth(GraphicColor::HSV& color, float z) {
	double z_ = -z * 0.01;
	color.v = 50.0 + atan(z_) * 50 / 1.57;
}

void RenderingTool::Render(SDL_Renderer* renderer) {
	for (register int i = this->points.size() - 1; i >= 0; i--) {
		register auto p = this->points[i];
		SDL_Rect rect = { p.x - Constant::LINE_WIDTH / 2, p.y - Constant::LINE_WIDTH / 2, Constant::LINE_WIDTH, Constant::LINE_WIDTH };
		GraphicColor::HSV hsv = GraphicColor::FLESH_COLOR;
		SetColorByDepth(hsv, p.z);
		GraphicColor::RGB clr(hsv);
		SDL_SetTextureColorMod(this->point_texture, clr.r, clr.g, clr.b);
		SDL_RenderCopy(renderer, this->point_texture, nullptr, &rect);
	}
	this->points.clear();

	//SDL_RenderCopy(renderer, this->point_texture, nullptr, nullptr);
	//cout << this->point_texture << endl;
}

void RenderingTool::SetLineColor(const GraphicColor::RGB color) {
	this->line_color = color;
}

void RenderingTool::SetPointColor(const GraphicColor::RGB color) {
	this->point_color = color;
}

void RenderingTool::Clear() {
	this->points.clear();
}

void RenderingTool::Initialize(SDL_Renderer* renderer) {
	if (RenderingTool::instance->point_texture) {
		SDL_DestroyTexture(RenderingTool::instance->point_texture);
	}
	int size = 500;
	bool show = false;
	SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
	auto texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, size, size);
	SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);
	SDL_SetRenderTarget(renderer, texture);
	SDL_SetRenderDrawColor(renderer, 255, 255, 255, 0);
	SDL_RenderClear(renderer);
	for (int x = 0; x < size; x++) {
		for (int y = 0; y < size; y++) {
			float dis = sqrt(pow((float)x - (float)size / 2, 2) + pow((float)y - (float)size / 2, 2));
			if (dis < size / 2) {
				SDL_SetRenderDrawColor(renderer, 255, 255, 255, (1.0 - dis / ((float)size / 2)) * 255);
				SDL_RenderDrawPoint(renderer, x, y);
			}
		}
	}
	SDL_SetRenderTarget(renderer, nullptr);
	SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
	RenderingTool::instance->point_texture = texture;
}

void RenderingTool::ReleaseInstance() {
	delete instance;
	instance = nullptr;
}
