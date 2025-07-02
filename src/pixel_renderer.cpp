#include <iostream>
#include <vector>

enum Color {
  BLACK = 0,
  RED = 1,
  GREEN = 2,
  YELLOW = 3,
  BLUE = 4,
  MAGENTA = 5,
  CYAN = 6,
  WHITE = 7
};

struct Pixel {
  bool state;
  Color color;
};

class HalfHeightRenderer {
private:
  int pixelWidth, pixelHeight;
  std::vector<std::vector<Pixel>> pixels;

public:
  HalfHeightRenderer(int w, int h) : pixelWidth(w), pixelHeight(h) {
    pixels.resize(h, std::vector<Pixel>(w, {false, WHITE}));
  }

  void setPixel(int x, int y, bool state, Color color = WHITE) {
    if (x >= 0 && x < pixelWidth && y >= 0 && y < pixelHeight) {
      pixels[y][x] = {state, color};
    }
  }

  void clear() {
    for (auto &row : pixels) {
      std::fill(row.begin(), row.end(), Pixel{false, WHITE});
    }
  }

  void render() {
    std::cout << "\033[H"; // Move to top

    // Process 2 pixel rows at a time
    for (int y = 0; y < pixelHeight; y += 2) {
      for (int x = 0; x < pixelWidth; x++) {
        Pixel topPixel = pixels[y][x];
        Pixel bottomPixel =
            (y + 1 < pixelHeight) ? pixels[y + 1][x] : Pixel{false, BLACK};

        // Convert 2 pixels into 1 character
        if (topPixel.state && bottomPixel.state) {
          std::cout << "\033[3" << topPixel.color << "m█";
        } else if (topPixel.state) {
          std::cout << "\033[3" << topPixel.color << "m▀";
        } else if (bottomPixel.state) {
          std::cout << "\033[3" << bottomPixel.color << "m▄";
        } else {
          std::cout << " "; // Both pixels off
        }
      }
      std::cout << "\n";
    }
  }

  // Example: Draw a simple pattern
  void drawCheckerboard() {
    for (int y = 0; y < pixelHeight; y++) {
      for (int x = 0; x < pixelWidth; x++) {
        setPixel(x, y, (x + y) % 2 == 0);
      }
    }
  }
};
