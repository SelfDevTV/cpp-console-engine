#include <termios.h>
#include <unistd.h>
#include "../include/game.h"
int main() {
  Game game(20, 20);
  // Keep the object alive so the game loop can continue
  return 0;
}
