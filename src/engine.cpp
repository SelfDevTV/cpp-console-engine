#include "../include/engine.h"
#include <ostream>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <algorithm>

// Get terminal size with fallback
std::pair<int, int> getTerminalSize() {
  struct winsize ws;
  if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0 && ws.ws_row > 0) {
    return {ws.ws_col, ws.ws_row};
  }
  // Fallback to reasonable defaults
  return {80, 24};
}

// Calculate width that fits in terminal
int getTerminalFitWidth(int requestedWidth) {
  auto [termWidth, termHeight] = getTerminalSize();
  return std::min(requestedWidth, termWidth);
}

// Calculate height that fits in terminal (accounting for HalfHeightRenderer)
int getTerminalFitHeight(int requestedHeight) {
  auto [termWidth, termHeight] = getTerminalSize();
  // HalfHeightRenderer uses 2 pixel rows per terminal row
  // Leave 1 row for potential status/debug info
  int maxPixelHeight = (termHeight - 1) * 2;
  return std::min(requestedHeight, maxPixelHeight);
}

Engine::Engine(int w, int h) : renderer{getTerminalFitWidth(w), getTerminalFitHeight(h)}, w(getTerminalFitWidth(w)), h(getTerminalFitHeight(h)) {
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

  std::string clearScreen = "\033[2J\033[H";
  std::cout << clearScreen;
  init();
  // Don't call run() here - let derived class call it
  // run();
  // tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
};

void Engine::run() {
  while (!stop) {
    clear();
    update((float)1 / 60);
    draw();
    render();
    usleep(16000);
  }
}

void Engine::pset(int x, int y, Color color) {
  renderer.setPixel(x, y, true, color);
}

void Engine::start() {
  run();
  // Restore terminal when done
  struct termios old_tio;
  tcgetattr(STDIN_FILENO, &old_tio);
  old_tio.c_lflag |= ECHO | ICANON;
  tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

Engine::~Engine() {}

void Engine::init() {}

void Engine::update(float dt) {
  char key;
  if (read(STDIN_FILENO, &key, 1) > 0) {
    if (key == 'q' || key == 'x') {
      stop = true;
    }
  }
}

void Engine::draw() {
}

void Engine::clear() {
  renderer.clear();
}

void Engine::render() {
  renderer.render();
}
