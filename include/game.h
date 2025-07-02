#pragma once
#include "engine.h"

class Game : public Engine {
public:
  Game(int x, int y);
  void draw() override;
};