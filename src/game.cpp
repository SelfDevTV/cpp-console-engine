#include "../include/game.h"

Game::Game(int x, int y) : Engine(x, y) {

  start(); // Now call start after Game is fully constructed
}


void Game::update(float dt) {
  playerX += 100 * dt;
  if(playerX >=w) {
    playerX = 0;
  } 
}

void Game::draw() {
  // Set yellow background
  for (int x = 0; x < w; x++) {
    for (int y = 0; y < h; y++) {
      pset(x, y, YELLOW);
    }
  }
  // Draw player as a black square pixel
  pset(playerX, playerY, BLACK);
}
