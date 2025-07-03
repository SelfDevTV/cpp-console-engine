#pragma once
#include "engine.h"

class Game : public Engine {
public:
  Game(int x, int y);
  void draw() override;
  void update(float dt) override;
private:
  int playerX = 10;
  int playerY = 10;
};
