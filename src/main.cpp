#include "./pixel_renderer.cpp"
#include <iostream>
#include <iterator>
#include <string>
#include <termios.h>
#include <unistd.h>
int main() {
  HalfHeightRenderer renderer(40, 40);
  struct termios old_tio, new_tio;
  tcgetattr(STDIN_FILENO, &old_tio);
  new_tio = old_tio;
  // disable echo
  new_tio.c_lflag &= ~ECHO;
  // disable signals like ctrl + c
  // new_tio.c_lflag &= ~ISIG;
  // disable canonical mode (raw input mode - read char by char instead of line
  // by line)
  new_tio.c_lflag &= ~ICANON;
  // minimum number of characters to read (0 = non-blocking read)
  new_tio.c_cc[VMIN] = 0;
  // timeout for read in deciseconds (0 = no timeout)
  new_tio.c_cc[VTIME] = 0;
  // disable cursor
  std::cout << "\e[?25l";
  tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
  bool stop = false;

  char key;
  std::string clearScreen = "\033[2J\033[H";
  std::string player = "O.O";
  int x = 5;
  int y = 0;

  void setPixel = renderer.setPixel;
  std::cout << clearScreen;
  while (!stop) {
    renderer.clear();

    // Draw just borders instead of solid background
    for (int xx = 0; xx < 40; xx++) {
      renderer.setPixel(xx, 0, true, YELLOW);  // Top
      renderer.setPixel(xx, 39, true, YELLOW); // Bottom
    }
    for (int yy = 1; yy < 39; yy++) {
      renderer.setPixel(0, yy, true, YELLOW);  // Left
      renderer.setPixel(39, yy, true, YELLOW); // Right
    }

    // Draw 2×2 red player
    renderer.setPixel(x, y, true, RED);

    renderer.render();
    usleep(16000);

    y++;
    if (y >= 38)
      y = 0; // Reset at bottom (leave space for 2×2 player)
    renderer.clear();
  }
  tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
  return 0;
}
