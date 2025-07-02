# Phase 2.2: Renderer Class Design

## C++ Learning Focus
- **Encapsulation**: Hiding implementation details behind clean interfaces
- **Method overloading**: Multiple versions of drawing functions
- **Const correctness**: Read-only methods and parameters
- **Smart pointers**: `std::unique_ptr` and `std::shared_ptr`
- **Template basics**: Generic drawing functions

## Implementation Overview
Extract all rendering logic from main.cpp into a dedicated Renderer class:
1. Manage frame buffers and double buffering
2. Provide high-level drawing primitives
3. Handle color management and console output
4. Abstract platform-specific rendering details

## Class Interface Design

### Core Renderer Interface
```cpp
class Renderer {
public:
    Renderer(int width, int height);
    ~Renderer();
    
    // Frame management
    void beginFrame();
    void endFrame();
    void present();
    
    // Basic drawing
    void clear(char fillChar = ' ');
    void setPixel(int x, int y, char c);
    char getPixel(int x, int y) const;
    
    // Primitive drawing
    void drawLine(int x1, int y1, int x2, int y2, char c);
    void drawRect(int x, int y, int width, int height, char c);
    void drawText(int x, int y, const std::string& text);
    
    // Color support
    void setColor(int foreground, int background = -1);
    void resetColor();
    
private:
    // Implementation details hidden
};
```

## Key C++ Concepts to Learn

### Const Correctness
```cpp
class Renderer {
public:
    // Read-only methods should be const
    char getPixel(int x, int y) const;
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    bool isValidPosition(int x, int y) const;
    
    // Methods that modify state are not const
    void setPixel(int x, int y, char c);
    void clear();
};
```

### Method Overloading
```cpp
class Renderer {
public:
    // Multiple versions of drawing functions
    void drawRect(int x, int y, int w, int h, char c);
    void drawRect(int x, int y, int w, int h, char c, bool filled);
    void drawRect(const Rect& rect, char c);
    
    void drawText(int x, int y, const std::string& text);
    void drawText(int x, int y, const std::string& text, int color);
};
```

### Buffer Management
```cpp
class Renderer {
private:
    std::unique_ptr<char[]> frontBuffer;
    std::unique_ptr<char[]> backBuffer;
    std::unique_ptr<int[]> colorBuffer;
    
    int width, height;
    int bufferSize;
    
    void allocateBuffers();
    void deallocateBuffers();
    void swapBuffers();
};
```

## Drawing Algorithm Implementation

### Line Drawing (Bresenham's Algorithm)
- Learn basic computer graphics algorithms
- Understand coordinate systems and transformations
- Handle edge cases (vertical/horizontal lines)
- Consider performance for console rendering

### Rectangle Drawing
- Hollow vs filled rectangles
- Bounds checking and clipping
- Efficient memory access patterns

### Text Rendering
- String positioning and alignment
- Text wrapping and clipping
- Font/character set considerations

## Color Management

### Color System Design
```cpp
enum class ConsoleColor {
    Black = 0, Red = 1, Green = 2, Yellow = 3,
    Blue = 4, Magenta = 5, Cyan = 6, White = 7,
    Bright_Black = 8, // ... etc
};

class Renderer {
private:
    int currentForeground;
    int currentBackground;
    
public:
    void setForegroundColor(ConsoleColor color);
    void setBackgroundColor(ConsoleColor color);
    void setColors(ConsoleColor fg, ConsoleColor bg);
};
```

### Platform-Specific Color Handling
- ANSI escape sequences for Unix/Linux
- Windows Console API colors
- Color capability detection
- Graceful degradation for monochrome terminals

## Performance Optimization

### Efficient Buffer Access
```cpp
// Inline for performance-critical functions
inline void Renderer::setPixel(int x, int y, char c) {
    if(isValidPosition(x, y)) {
        backBuffer[y * width + x] = c;
    }
}

// Row-major access pattern for cache efficiency
void Renderer::clear(char fillChar) {
    for(int i = 0; i < bufferSize; ++i) {
        backBuffer[i] = fillChar;
    }
}
```

### Dirty Rectangle Optimization
```cpp
class Renderer {
private:
    struct DirtyRegion {
        int minX, minY, maxX, maxY;
        bool isDirty;
    } dirtyRegion;
    
    void markDirty(int x, int y);
    void expandDirtyRegion(int x, int y);
    void renderDirtyRegion();
};
```

## Implementation Tips

### Bounds Checking Strategy
```cpp
class Renderer {
private:
    bool isValidPosition(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }
    
public:
    void setPixel(int x, int y, char c) {
        // Option 1: Silent clipping
        if(!isValidPosition(x, y)) return;
        
        // Option 2: Debug assertions
        assert(isValidPosition(x, y));
        
        // Option 3: Exception throwing
        if(!isValidPosition(x, y)) 
            throw std::out_of_range("Pixel position out of bounds");
    }
};
```

### Memory Management
```cpp
class Renderer {
public:
    Renderer(int w, int h) : width(w), height(h), bufferSize(w * h) {
        allocateBuffers();
    }
    
    ~Renderer() {
        // Smart pointers handle cleanup automatically
    }
    
private:
    void allocateBuffers() {
        frontBuffer = std::make_unique<char[]>(bufferSize);
        backBuffer = std::make_unique<char[]>(bufferSize);
        colorBuffer = std::make_unique<int[]>(bufferSize);
        
        clear(); // Initialize to known state
    }
};
```

## Common Design Decisions

### Coordinate System
- Origin at (0,0) - top-left vs bottom-left?
- Y-axis direction - up or down?
- Document your choice clearly!

### Error Handling Philosophy
- Silent clipping vs exceptions vs assertions
- Performance vs safety trade-offs
- Debug vs release behavior

### API Design Choices
- Immediate mode vs retained mode rendering
- State-based vs stateless API
- Method chaining support: `renderer.setColor(RED).drawRect(...)`

## Testing Strategy
- Unit tests for drawing primitives
- Visual tests with known patterns
- Performance benchmarks
- Edge case testing (boundaries, empty shapes)

## Integration with Engine

### Ownership Model
```cpp
class Engine {
private:
    std::unique_ptr<Renderer> renderer;
    
public:
    bool initialize() {
        renderer = std::make_unique<Renderer>(config.width, config.height);
        return renderer->initialize();
    }
    
    void render() {
        renderer->beginFrame();
        // Game rendering here...
        renderer->endFrame();
        renderer->present();
    }
};
```

## Resources
- [Bresenham's Line Algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
- [Computer Graphics Algorithms](https://www.cs.helsinki.fi/group/goa/mallinnus/lines/bresenh.html)
- [Console Color Codes](https://stackoverflow.com/questions/287871/how-to-print-colored-text-in-terminal-in-python)
- [Smart Pointers in C++](https://en.cppreference.com/w/cpp/memory)

## Fun Challenges
- Implement circle/ellipse drawing
- Add sprite/bitmap rendering
- Create ASCII art font system
- Implement screen effects (shake, fade, flash)
- Add viewport/camera system for larger worlds
- Create particle system renderer

## Debugging Tips
- Add debug visualization for dirty regions
- Implement renderer statistics (draw calls, pixels set)
- Create test patterns for visual verification
- Add performance profiling for drawing operations

## Common Gotchas
- **Buffer overruns**: Always check bounds
- **Memory leaks**: Use smart pointers or RAII
- **Color bleeding**: Reset colors properly
- **Performance**: Avoid unnecessary buffer copies
- **Coordinate confusion**: Be consistent with X/Y ordering

## Refactoring Process

### Step 1: Extract Basic Renderer
1. Create Renderer class with constructor/destructor
2. Move buffer management from main.cpp
3. Add basic setPixel/getPixel/clear methods

### Step 2: Add Drawing Primitives
1. Implement drawLine method
2. Add drawRect (hollow and filled)
3. Add drawText method

### Step 3: Add Color Support
1. Extend buffers to support color
2. Add color setting methods
3. Update rendering to output colors

### Step 4: Optimize Performance
1. Add dirty region tracking
2. Optimize buffer access patterns
3. Profile and optimize hot paths