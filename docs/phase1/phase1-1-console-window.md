# Phase 1.1: Console Window Management

## C++ Learning Focus
- **Platform-specific APIs**: Understanding conditional compilation with `#ifdef`
- **Header files**: `<windows.h>` on Windows, `<termios.h>` and `<unistd.h>` on Unix/Linux
- **Function calls**: Learning to call system APIs
- **Error handling**: Checking return values from system calls

## Implementation Overview
You'll need to:
1. Detect the operating system at compile time
2. Set console window to a fixed size (e.g., 80x25 characters)
3. Hide the cursor so it doesn't flicker during rendering
4. Configure input mode to capture keypresses immediately
5. Store original console settings to restore them later

## Platform-Specific Tips

### Windows (`windows.h`)
- Use `GetConsoleScreenBufferInfo()` to get current settings
- Use `SetConsoleScreenBufferSize()` and `SetConsoleWindowInfo()` for sizing
- Use `SetConsoleCursorInfo()` to hide cursor
- Use `SetConsoleMode()` for input configuration

### Linux/Mac (`termios.h`)
- Use `tcgetattr()` to save original terminal settings
- Use `tcsetattr()` to configure raw input mode
- Use ANSI escape sequences like `\033[?25l` to hide cursor
- Use `ioctl()` with `TIOCGWINSZ` to get terminal size

## Key C++ Concepts to Learn
- **Preprocessor directives**: `#ifdef _WIN32`, `#else`, `#endif`
- **Global variables**: Storing console state
- **Function organization**: Separate functions for setup/cleanup
- **RAII principle**: Constructor/destructor pattern (prepare for Phase 2)

## Implementation Tips
1. **Start simple**: Just clear screen and hide cursor first
2. **Test incrementally**: Add one feature at a time
3. **Handle errors gracefully**: Always check if API calls succeed
4. **Document platform differences**: Comment why you use different approaches

## Common Gotchas
- Console size vs buffer size difference on Windows
- Terminal settings not being restored on program crash
- Unicode/encoding issues with special characters
- Different key codes between platforms

## Testing Strategy
- Test on your target platform first
- Verify cursor is hidden
- Check that console size is what you expect
- Ensure settings are restored when program exits

## Resources
- [Windows Console API Reference](https://docs.microsoft.com/en-us/windows/console/console-reference)
- [Linux Terminal I/O Programming](https://tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html)
- [ANSI Escape Sequences](https://en.wikipedia.org/wiki/ANSI_escape_code)
- [Cross-platform C++ console tutorial](https://stackoverflow.com/questions/6856635/hide-cursor-in-console)

## Fun Challenges
- Add support for getting actual terminal size dynamically
- Implement console title setting
- Add color palette detection
- Create a "console capabilities" detection system

## Refactoring Notes
In Phase 2, you'll move this into a `Console` or `Platform` class. Think about:
- Which functions should be public vs private?
- What state needs to be stored?
- How to handle cleanup automatically?