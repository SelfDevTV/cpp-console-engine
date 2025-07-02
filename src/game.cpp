#include "../include/game.h"

Game::Game(int x, int y) : Engine(x, y) {
  start(); // Now call start after Game is fully constructed
}

void Game::draw() {
  // Set multiple pixels to make sure we see something
  pset(1, 1, RED);
  
}
