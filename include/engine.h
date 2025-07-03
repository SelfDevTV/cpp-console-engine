#pragma once
#include "../src/pixel_renderer.cpp"

class Engine {
private:
  HalfHeightRenderer renderer;
  bool stop = false;
  void render();
  void clear();
  void initialize();
  void run();

protected:
  int w, h;

public:
  Engine(int w, int h);
  virtual ~Engine();
  virtual void init();
  virtual void update(float dt);
  virtual void draw();

  void start();
  void pset(int x, int y, Color col = WHITE);
};
