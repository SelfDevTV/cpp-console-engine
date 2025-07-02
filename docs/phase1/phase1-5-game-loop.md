# Phase 1.5: Game Loop Structure

## C++ Learning Focus
- **Main function structure**: Organizing your `main()` effectively
- **Loop design**: `while`, `for`, and loop control
- **Timing functions**: `chrono` library for time management
- **RAII pattern**: Resource cleanup even with early exits

## Implementation Overview
The game loop is the heart of your engine:
1. **Initialize**: Set up console, buffers, initial game state
2. **Main Loop**: Input → Update → Render → Timing
3. **Cleanup**: Restore console settings, free resources

## Game Loop Pattern

### Basic Structure
```cpp
int main() {
    // 1. Initialize
    setupConsole();
    initializeBuffers();
    initializeGame();
    
    // 2. Main loop
    bool running = true;
    while(running) {
        // Input
        updateInput();
        if(shouldQuit()) running = false;
        
        // Update
        updateGame();
        
        // Render
        clearBuffer();
        drawGame();
        renderBuffer();
        
        // Timing
        limitFrameRate();
    }
    
    // 3. Cleanup
    cleanupGame();
    restoreConsole();
    return 0;
}
```

## Key C++ Concepts to Learn
- **Scope management**: Variables living only as long as needed
- **Early returns**: Multiple exit points and cleanup
- **Exception handling**: `try`/`catch` for robust cleanup
- **Static variables**: Frame counters and timing data

## Timing and Frame Rate

### Using `std::chrono`
```cpp
#include <chrono>
#include <thread>

auto lastFrame = std::chrono::high_resolution_clock::now();
const double targetFPS = 60.0;
const double frameTime = 1.0 / targetFPS;

while(running) {
    auto frameStart = std::chrono::high_resolution_clock::now();
    
    // Game logic here...
    
    // Calculate sleep time
    auto frameEnd = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration<double>(frameEnd - frameStart).count();
    
    if(elapsed < frameTime) {
        auto sleepTime = frameTime - elapsed;
        std::this_thread::sleep_for(std::chrono::duration<double>(sleepTime));
    }
}
```

### Delta Time Calculation
```cpp
double deltaTime = 0.0;
auto lastFrame = std::chrono::high_resolution_clock::now();

while(running) {
    auto currentFrame = std::chrono::high_resolution_clock::now();
    deltaTime = std::chrono::duration<double>(currentFrame - lastFrame).count();
    lastFrame = currentFrame;
    
    // Use deltaTime for frame-rate independent movement
    updateGame(deltaTime);
}
```

## Loop Organization Tips

### Phase Structure
```cpp
// Clear separation of concerns
while(running) {
    // 1. INPUT PHASE
    handleInput();
    
    // 2. UPDATE PHASE  
    updateGameLogic(deltaTime);
    updatePhysics(deltaTime);
    updateAnimations(deltaTime);
    
    // 3. RENDER PHASE
    clearBuffers();
    drawBackground();
    drawGameObjects();
    drawUI();
    presentFrame();
    
    // 4. TIMING PHASE
    limitFrameRate();
    updateFrameCounter();
}
```

### Exit Conditions
- **Escape key pressed**
- **Window closed**
- **Game over state**
- **Error conditions**

## Performance Considerations

### Frame Rate Strategies
- **Fixed timestep**: Consistent physics, may drop frames
- **Variable timestep**: Smooth rendering, physics can be unstable
- **Hybrid**: Fixed physics updates, variable rendering

### Optimization Tips
- Profile each phase to find bottlenecks
- Consider skipping rendering if running too slow
- Cache expensive calculations outside the loop
- Use efficient data structures

## Error Handling and Cleanup

### RAII Pattern Example
```cpp
class ConsoleManager {
public:
    ConsoleManager() { setupConsole(); }
    ~ConsoleManager() { restoreConsole(); }
};

int main() {
    ConsoleManager console; // Automatic cleanup on exit
    
    // Game loop here...
    
    // console cleanup happens automatically
}
```

### Exception Safety
```cpp
int main() {
    try {
        setupConsole();
        gameLoop();
    }
    catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    restoreConsole(); // Always restore console
    return 0;
}
```

## Common Gotchas
- **Infinite loops**: Missing exit conditions
- **Resource leaks**: Not cleaning up on all exit paths
- **Timing issues**: Not accounting for processing time
- **Input lag**: Processing input at wrong time in loop

## Loop Debugging

### Frame Rate Monitoring
```cpp
static int frameCount = 0;
static auto lastSecond = std::chrono::high_resolution_clock::now();

frameCount++;
auto now = std::chrono::high_resolution_clock::now();
if(std::chrono::duration<double>(now - lastSecond).count() >= 1.0) {
    // Display FPS: frameCount
    frameCount = 0;
    lastSecond = now;
}
```

### Performance Profiling
- Time each phase of the loop
- Log slow frames
- Monitor memory usage
- Track input responsiveness

## Testing Strategy
- Test with different frame rate limits
- Verify smooth movement at various speeds
- Test exit conditions work properly
- Check resource cleanup on abnormal exit

## Implementation Tips

### Start Simple
```cpp
// Phase 1 simple loop
while(running) {
    if(keyPressed('q')) running = false;
    
    // Simple animation
    static int x = 0;
    clearBuffer();
    setPixel(x % WIDTH, 10, '*');
    x++;
    
    renderBuffer();
    sleep(100); // Simple timing
}
```

### Gradually Add Features
1. Basic input/output
2. Proper timing
3. Frame rate limiting
4. Performance monitoring
5. Error handling

## Resources
- [Game Loop Patterns](https://gameprogrammingpatterns.com/game-loop.html)
- [C++ Chrono Library](https://en.cppreference.com/w/cpp/chrono)
- [RAII in C++](https://en.cppreference.com/w/cpp/language/raii)

## Fun Challenges
- Implement pause/resume functionality
- Add slow-motion and fast-forward modes
- Create a simple profiler to measure loop phases
- Add support for variable quality settings
- Implement frame skipping for slow systems

## Refactoring Notes
In Phase 2, this becomes an `Engine` class:
- Encapsulate loop logic in `Engine::run()`
- Add state management for different game modes
- Implement clean startup/shutdown sequences
- Add support for multiple scenes/states