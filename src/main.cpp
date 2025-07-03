#include <termios.h>
#include <unistd.h>
#include "../include/game.h"
int main() {
  Game game(100, 100);  // Much smaller dimensions that fit terminal
  // Keep the object alive so the game loop can continue
  return 0;
}
