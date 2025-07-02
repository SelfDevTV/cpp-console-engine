# Phase 1.4: Input Handling

## C++ Learning Focus
- **Non-blocking I/O**: Understanding blocking vs non-blocking input
- **Platform differences**: Windows `_kbhit()` vs Linux `select()`
- **Bitwise operations**: Key state flags and combinations
- **Function pointers**: Preparing for callback systems later

## Implementation Overview
Create responsive input that doesn't block your game loop:
1. Check if key is pressed without waiting
2. Track key states (just pressed, held down, just released)
3. Map physical keys to logical game actions
4. Handle special keys (arrows, function keys, etc.)

## Platform-Specific Input

### Windows (`conio.h`)
```cpp
#include <conio.h>

// Check if key is available
if (_kbhit()) {
    char key = _getch(); // Get the key without echo
    // Process key...
}
```

### Linux/Mac (`termios.h`, `sys/select.h`)
```cpp
#include <termios.h>
#include <sys/select.h>

// Set up non-blocking input
struct termios old_tio, new_tio;
tcgetattr(STDIN_FILENO, &old_tio);
new_tio = old_tio;
new_tio.c_lflag &= ~(ICANON | ECHO);
tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

// Check for input
fd_set set;
struct timeval timeout;
FD_ZERO(&set);
FD_SET(STDIN_FILENO, &set);
timeout.tv_sec = 0;
timeout.tv_usec = 0;

if (select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout) > 0) {
    char key = getchar();
    // Process key...
}
```

## Key C++ Concepts to Learn
- **Enumeration**: `enum class KeyState { Up, Down, Pressed, Released }`
- **Arrays/Maps**: Storing key states efficiently
- **Boolean logic**: Combining key states and modifiers
- **Conditional compilation**: `#ifdef` for platform-specific code

## Key State Management

### Key State Types
- **Up**: Key is not pressed
- **Down**: Key is currently held down
- **Pressed**: Key was just pressed this frame
- **Released**: Key was just released this frame

### Implementation Pattern
```cpp
enum class KeyState { Up, Down, Pressed, Released };

struct InputManager {
    KeyState previousKeys[256];
    KeyState currentKeys[256];
    
    void update() {
        // Copy current to previous
        // Read new input
        // Calculate pressed/released states
    }
};
```

## Input Mapping

### Physical vs Logical Keys
- **Physical**: 'W', 'A', 'S', 'D', Arrow keys
- **Logical**: MoveUp, MoveLeft, MoveDown, MoveRight, Jump, Fire

### Mapping Implementation
```cpp
enum class GameAction {
    MoveUp, MoveDown, MoveLeft, MoveRight,
    Jump, Fire, Pause, Quit
};

bool isActionPressed(GameAction action) {
    switch(action) {
        case GameAction::MoveUp: 
            return isKeyPressed('W') || isKeyPressed(KEY_UP);
        // ... etc
    }
}
```

## Special Key Handling

### Arrow Keys and Function Keys
- Often send multiple bytes (escape sequences)
- May need special detection logic
- Platform-specific key codes

### Common Key Codes
- **Escape**: 27 (often followed by more bytes)
- **Enter**: 13 (Windows) or 10 (Unix)
- **Backspace**: 8
- **Space**: 32
- **Arrow keys**: Usually escape sequences like `[A`, `[B`, `[C`, `[D`

## Implementation Tips

### Input Polling Pattern
```cpp
void updateInput() {
    // Save previous frame's input state
    
    // Clear current frame input
    
    // Poll for new input (non-blocking)
    
    // Update key states based on previous vs current
}
```

### Frame-Based Input
- Call input update once per frame
- Store "pressed this frame" flags
- Reset pressed/released flags each frame

## Common Gotchas
- **Blocking input**: Using `cin` or `getchar()` without non-blocking setup
- **Key repeat**: OS key repeat can cause multiple "pressed" events
- **Platform key codes**: Different values for same keys on different systems
- **Terminal settings**: Not restoring original settings on exit

## Testing Strategy
- Test all mapped keys work correctly
- Verify non-blocking behavior (game doesn't freeze)
- Test key combinations (Ctrl+C, etc.)
- Check that key states reset properly between frames

## Performance Considerations
- Input polling is usually fast, but don't overdo it
- Consider input buffering for precise timing
- Avoid string comparisons for key checking

## Input Responsiveness Tips
- Update input at start of game loop
- Process input immediately after reading
- Consider input prediction for fast-paced games
- Handle input even during rendering/loading

## Resources
- [Cross-platform Input Handling](https://stackoverflow.com/questions/7469139/what-is-the-equivalent-to-getch-getche-in-linux)
- [Terminal Programming Guide](https://viewsourcecode.org/snaptoken/kilo/02.enteringRawMode.html)
- [Windows Console Input](https://docs.microsoft.com/en-us/windows/console/reading-input-buffer-events)

## Fun Challenges
- Add mouse support (if terminal supports it)
- Implement key combinations (Ctrl+S, Alt+F4)
- Create input recording/playback system
- Add configurable key bindings
- Implement input sensitivity/timing adjustments

## Debugging Tips
- Display current key states on screen
- Log all input events to a file
- Add input visualization (show pressed keys)
- Test with different terminals/consoles

## Refactoring Notes
In Phase 2, this becomes an `InputManager` class:
- Encapsulate all platform-specific input code
- Provide clean action-based interface
- Add support for input configuration
- Consider observer pattern for input events