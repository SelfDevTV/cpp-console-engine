# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a console-based game engine written in C++ designed for educational purposes. The engine runs in the existing terminal/console (not a separate window) and uses POSIX system calls for direct terminal control. The project is structured as a progressive learning exercise with 6 phases of increasing complexity.

## Building and Running

### Basic Build Process
```bash
# Compile the current prototype
g++ -o mygame src/main.cpp

# Run the game
./mygame
```

### Current Implementation
The `src/main.cpp` contains a basic terminal game loop that:
- Configures terminal for raw input mode using `termios.h`
- Disables echo, canonical mode, and signals
- Implements non-blocking character input with `read()`
- Uses ANSI escape sequences for screen control
- Runs at 60 FPS with `usleep(16000)`

## Architecture Overview

### Development Phases
The codebase follows a structured 6-phase approach documented in `docs/`:

1. **Phase 1**: Foundation (console window, buffers, rendering, input, game loop)
2. **Phase 2**: Core Classes (Engine, Renderer, Component system)
3. **Phase 3**: Objects & Scenes (Scene management, GameObjects, Transform system)
4. **Phase 4**: Input & Math (InputManager, Vector2, collision detection)
5. **Phase 5**: Advanced Features (Audio, GameState management, 2D physics)
6. **Phase 6**: Optimization (Memory management, performance, cross-platform)

### Current State - UPDATED
- **Phase 1 COMPLETED**: Working game prototype with full terminal control, character buffer rendering, and player movement
- **Current Implementation**: 40×20 world with borders, selective character doubling for visual consistency, wrap-around movement, 30 FPS game loop
- **Technical Achievements**: Raw terminal input, non-blocking I/O, ANSI escape sequences, double-buffered rendering, memory-efficient string building
- **Input System**: Basic movement (d=right, w=up, x=quit) with wrap-around teleportation at boundaries
- **Rendering Features**: Smart border rendering (single-width borders, double-width interior), clean screen updates
- All phases have comprehensive documentation with C++ learning objectives
- **Ready for Phase 2**: Refactoring from single-file prototype to class-based architecture (Engine, Renderer, InputManager)

### Platform Specifics
- **Primary target**: Unix/Linux systems using POSIX APIs
- **Terminal APIs**: `termios.h`, `unistd.h` for raw input/output
- **Cross-platform**: Windows support planned via conditional compilation
- **Input method**: Direct `read()` system calls for performance
- **Output method**: Mix of `std::cout` and ANSI escape sequences

## Key Technical Decisions

### Input System
- Uses `read(STDIN_FILENO, &key, 1)` for immediate key detection
- Non-blocking I/O (`VMIN=0, VTIME=0`) to maintain 60 FPS game loop
- Raw terminal mode (`~ICANON, ~ECHO, ~ISIG`) for game-appropriate input

### Rendering Approach
- ASCII/Unicode character-based rendering
- ANSI escape sequences for cursor positioning and colors
- Double buffering concept for flicker-free updates
- Target: 20x20 character grid initially, expandable

### Learning Focus
Each phase emphasizes specific C++ concepts:
- Memory management and RAII patterns
- Object-oriented design and component systems
- Template programming and STL usage
- Platform abstraction and cross-platform development
- Performance optimization and profiling

## Development Notes

### Current main.cpp Structure - WORKING PROTOTYPE
The completed Phase 1 prototype demonstrates all core engine concepts:
- **Terminal Control**: Complete raw mode setup with `termios`, cursor hiding, non-blocking input
- **Game Loop**: Input polling → buffer clearing → border rendering → player placement → string building → screen output
- **Rendering System**: Selective character doubling (borders single-width, interior double-width), ANSI positioning
- **Movement System**: Basic WASD-style movement with boundary wrap-around teleportation
- **Memory Management**: Pre-allocated string buffers, efficient 2D vector usage
- **Frame Rate**: 30 FPS with `usleep(32000)` timing

### Next Steps (Based on Phase Documentation)
1. Extract Engine class from main.cpp
2. Implement Renderer class for buffer management
3. Create Component system for game objects
4. Add Scene management and GameObject hierarchy
5. Implement InputManager for action mapping
6. Add math utilities (Vector2, collision detection)

### Documentation Structure
Each phase document includes:
- C++ learning objectives
- Implementation overview with code examples
- Common pitfalls and debugging tips
- Testing strategies
- Integration examples
- Refactoring guidance from previous phase