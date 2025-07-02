# Phase 2.1: Engine Class Design

## C++ Learning Focus
- **Class design**: Public vs private interfaces
- **Constructor/Destructor**: RAII pattern implementation
- **Singleton pattern**: Global access vs dependency injection
- **Member initialization lists**: Efficient initialization
- **Static members**: Class-level data and functions

## Implementation Overview
Transform your main.cpp functions into a cohesive Engine class:
1. Encapsulate console setup/cleanup
2. Manage the main game loop
3. Coordinate other subsystems (renderer, input)
4. Provide clean initialization and shutdown

## Class Structure Design

### Basic Engine Interface
```cpp
class Engine {
public:
    Engine(int width, int height);
    ~Engine();
    
    bool initialize();
    void run();
    void shutdown();
    
    bool isRunning() const;
    void stop();
    
private:
    // Implementation details hidden
};
```

### Singleton vs Instance
**Singleton Approach:**
- Global access: `Engine::getInstance()`
- Automatic lifetime management
- Can complicate testing

**Instance Approach:**
- Created in main(), passed around
- Clear ownership model
- Better for testing and flexibility

## Key C++ Concepts to Learn

### RAII (Resource Acquisition Is Initialization)
```cpp
class Engine {
private:
    bool consoleInitialized;
    
public:
    Engine() : consoleInitialized(false) {
        // Constructor acquires resources
        if(setupConsole()) {
            consoleInitialized = true;
        }
    }
    
    ~Engine() {
        // Destructor automatically releases resources
        if(consoleInitialized) {
            restoreConsole();
        }
    }
};
```

### Member Initialization Lists
```cpp
class Engine {
private:
    int width;
    int height;
    bool running;
    double targetFPS;
    
public:
    Engine(int w, int h) 
        : width(w), height(h), running(false), targetFPS(60.0) {
        // Constructor body - prefer initialization list over assignment
    }
};
```

## Engine Responsibilities

### Core Functions
1. **Lifecycle Management**: Initialize, run, shutdown
2. **Loop Control**: Main game loop timing and flow
3. **System Coordination**: Manage renderer, input, etc.
4. **Error Handling**: Graceful failure and recovery

### What NOT to Put in Engine
- Game-specific logic (that goes in Scene/GameState)
- Detailed rendering code (that goes in Renderer)
- Input handling details (that goes in InputManager)

## Design Patterns to Consider

### State Pattern Preparation
```cpp
class Engine {
private:
    enum class EngineState {
        Uninitialized,
        Initializing, 
        Running,
        Paused,
        Shutting_Down
    };
    
    EngineState currentState;
    
public:
    void setState(EngineState newState);
    EngineState getState() const { return currentState; }
};
```

### Observer Pattern Hooks
```cpp
class Engine {
public:
    // Allow other systems to hook into engine events
    void onInitialize(std::function<void()> callback);
    void onShutdown(std::function<void()> callback);
    void onFrameStart(std::function<void(double)> callback);
};
```

## Implementation Tips

### Error Handling Strategy
```cpp
class Engine {
public:
    bool initialize() {
        try {
            if(!setupConsole()) return false;
            if(!initializeRenderer()) return false;
            if(!initializeInput()) return false;
            return true;
        }
        catch(const std::exception& e) {
            std::cerr << "Engine initialization failed: " << e.what() << std::endl;
            return false;
        }
    }
};
```

### Configuration Management
```cpp
struct EngineConfig {
    int windowWidth = 80;
    int windowHeight = 25;
    double targetFPS = 60.0;
    bool enableColors = true;
    bool fullscreen = false;
};

class Engine {
private:
    EngineConfig config;
    
public:
    Engine(const EngineConfig& cfg) : config(cfg) {}
};
```

## Common Design Pitfalls
- **God Object**: Don't put everything in Engine
- **Tight Coupling**: Engine shouldn't know about specific game objects
- **Initialization Order**: Be careful about dependencies between systems
- **Exception Safety**: Ensure cleanup happens even with exceptions

## Testing Strategy
- Test initialization/shutdown cycles
- Verify engine state transitions
- Test error conditions and recovery
- Measure startup/shutdown times

## Header File Organization

### engine.h Structure
```cpp
#ifndef ENGINE_H
#define ENGINE_H

#include <memory>
#include <functional>

// Forward declarations
class Renderer;
class InputManager;

class Engine {
public:
    // Public interface
    
private:
    // Private implementation
    std::unique_ptr<Renderer> renderer;
    std::unique_ptr<InputManager> input;
};

#endif // ENGINE_H
```

### Include Strategy
- Forward declare what you can in headers
- Include implementation details only in .cpp files
- Minimize header dependencies

## Performance Considerations
- Don't allocate/deallocate in the main loop
- Pre-allocate resources during initialization
- Consider object pooling for frequently created objects
- Profile initialization time

## Resources
- [RAII in C++](https://en.cppreference.com/w/cpp/language/raii)
- [Singleton Pattern](https://refactoring.guru/design-patterns/singleton/cpp)
- [Game Engine Architecture](https://www.gameenginebook.com/)
- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/)

## Fun Challenges
- Add engine configuration from file
- Implement engine statistics tracking (uptime, frame count)
- Add plugin/module loading system
- Create engine benchmarking tools
- Implement hot-reloading of engine components

## Debugging Tips
- Add logging to track engine state changes
- Implement engine console commands
- Create debug overlays showing engine stats
- Add memory usage tracking

## Refactoring Process

### Step 1: Extract Engine Class
1. Create empty Engine class
2. Move console setup/cleanup to constructor/destructor
3. Move main loop to Engine::run()
4. Test that everything still works

### Step 2: Add Configuration
1. Create EngineConfig struct
2. Pass config to Engine constructor
3. Use config throughout engine

### Step 3: Add Error Handling
1. Make initialization return bool
2. Add proper exception handling
3. Ensure cleanup on all exit paths

### Step 4: Add State Management
1. Add EngineState enum
2. Track current state
3. Add state transition validation

## Integration with main.cpp
```cpp
// main.cpp after refactoring
#include "engine.h"

int main() {
    EngineConfig config;
    config.windowWidth = 80;
    config.windowHeight = 25;
    config.targetFPS = 60.0;
    
    Engine engine(config);
    
    if(!engine.initialize()) {
        std::cerr << "Failed to initialize engine" << std::endl;
        return 1;
    }
    
    engine.run(); // Contains the main loop
    
    return 0;
}
```