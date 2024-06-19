#pragma once
#include <SDL.h>
#include <Window.h>
#include <Constant.h>

class RenderingTool {
private :
	RenderingTool();
	~RenderingTool();

	static RenderingTool* instance;

	SDL_Texture* point_texture;

	int center_x, center_y;

	class Point {
	public :
		float x, y, z;
		bool operator< (const Point& a) const {
			return this->z < a.z;
		}
		bool operator== (const Point& a) const {
			return this->x == a.x && this->y == a.y && this->z == a.z;
		}
	};
	vector<Point> points;

	void AddPoint(float x, float y, float z);

	GraphicColor::RGB line_color = GraphicColor::FRAME_COLOR_RGB;
	GraphicColor::RGB point_color = GraphicColor::FRAME_COLOR_RGB;

public :
	static RenderingTool* GetInstance();
	static void Initialize(SDL_Renderer *ren);
	static void ReleaseInstance();

	void SetCenter(int x, int y);
	void SetLineColor(const GraphicColor::RGB color);
	void SetPointColor(const GraphicColor::RGB color);

	void DrawPoint(SDL_Renderer* renderer, float x, float y, float z);
	void DrawLine(SDL_Renderer* renderer, float x1, float y1, float z1, float x2, float y2, float z2);

	void Render(SDL_Renderer* renderer);
	void Clear();
};