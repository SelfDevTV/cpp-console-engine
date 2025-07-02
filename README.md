# Console Game Engine

A lightweight, educational C++ game engine that runs in the terminal using ASCII/Unicode characters and ANSI color codes.

## Features

- **Half-height pixel rendering** - Achieves 2x vertical resolution using Unicode block characters
- **Color support** - 8 basic colors using ANSI escape sequences  
- **Raw terminal input** - Non-blocking keyboard input with POSIX termios
- **60 FPS game loop** - Smooth animation with frame rate limiting
- **Clean API** - Pico-8/Love2D inspired drawing functions

## Quick Start

```bash
# Compile and run
g++ -o game src/main.cpp
./game
```

## Architecture

The engine follows a 6-phase development plan:

1. **Phase 1 ✅**: Foundation (terminal control, buffers, input, game loop)
2. **Phase 2**: Core Classes (Engine, Renderer, Component system)  
3. **Phase 3**: Objects & Scenes (GameObject, Scene management)
4. **Phase 4**: Input & Math (InputManager, Vector2, collision)
5. **Phase 5**: Advanced Features (Audio, GameState, Physics)
6. **Phase 6**: Optimization (Memory, performance, cross-platform)

## Current Status

- ✅ Working pixel renderer with half-height blocks
- ✅ Color support and terminal control
- ✅ Basic game loop with player movement
- 🚧 Refactoring into Engine base class

## Documentation

Comprehensive development documentation is available in the `docs/` directory, covering each phase with C++ learning objectives and implementation details.

## Platform Support

- **Linux/macOS**: Full support using POSIX termios
- **Windows**: Planned (Phase 6) using conditional compilation
- **WSL**: Works perfectly

## Learning Focus

This project serves as a C++ learning exercise, progressively covering:
- POSIX system calls and terminal programming
- Object-oriented design and RAII patterns
- STL containers and modern C++ features
- Game engine architecture and design patterns