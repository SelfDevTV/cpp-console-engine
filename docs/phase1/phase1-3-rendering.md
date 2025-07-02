# Phase 1.3: Basic Rendering System

## C++ Learning Focus
- **I/O streams**: `cout`, `printf`, and their performance differences
- **String manipulation**: Building output strings efficiently
- **Loop optimization**: Minimizing console I/O calls
- **Escape sequences**: ANSI codes for cursor positioning and colors

## Implementation Overview
Transform your buffer into visible console output:
1. Position cursor at top-left (0,0)
2. Output each character from buffer
3. Handle colors if implemented
4. Avoid screen flicker through proper techniques

## Rendering Strategies

### Strategy 1: Character-by-Character
```
for(int y = 0; y < HEIGHT; y++) {
    for(int x = 0; x < WIDTH; x++) {
        cout << buffer[y][x];
    }
    cout << '\n'; // newline at end of each row
}
```

### Strategy 2: String Building (More Efficient)
```
string output;
output.reserve(HEIGHT * (WIDTH + 1)); // pre-allocate memory
for(int y = 0; y < HEIGHT; y++) {
    for(int x = 0; x < WIDTH; x++) {
        output += buffer[y][x];
    }
    output += '\n';
}
cout << output;
```

## Key C++ Concepts to Learn
- **String performance**: `string` vs `char*` vs `stringstream`
- **Memory pre-allocation**: `reserve()` for performance
- **Operator overloading**: `+=` for strings
- **Stream manipulators**: `cout << flush` vs `cout << endl`

## Console Positioning Techniques

### ANSI Escape Sequences (Cross-platform)
- `\033[H` - Move cursor to top-left
- `\033[2J` - Clear entire screen
- `\033[{row};{col}H` - Move to specific position
- `\033[0m` - Reset colors

### Platform-Specific APIs
- **Windows**: `SetConsoleCursorPosition()`
- **Linux/Mac**: ANSI sequences work better

## Color Support

### ANSI Color Codes
- Foreground: `\033[30m` (black) through `\033[37m` (white)
- Background: `\033[40m` (black) through `\033[47m` (white)
- Reset: `\033[0m`
- Bright colors: `\033[90m` through `\033[97m`

### Implementation Tips
- Store color codes in your color buffer
- Build color strings: `\033[31m` + character + `\033[0m`
- Consider color pooling to avoid string building overhead

## Performance Optimization

### Minimize Console Calls
- Build entire frame as one string
- Use `cout << string` once instead of many small outputs
- Avoid `endl` (use `'\n'` instead - `endl` forces flush)

### Reduce Flicker
- Clear screen once at start, then overwrite
- Use cursor positioning instead of clearing
- Consider double buffering at console level

### String Building Performance
```cpp
// Fast: pre-allocate
string output;
output.reserve(estimated_size);

// Slow: many reallocations
string output;
// many += operations without reserve()
```

## Implementation Tips

### Frame Rendering Flow
1. Move cursor to (0,0)
2. For each row:
   - Build row string with colors
   - Output entire row at once
3. Flush output if needed

### Error Handling
- Check if console output succeeds
- Handle cases where console is resized
- Graceful degradation if colors not supported

## Common Gotchas
- **Newline handling**: Extra newlines can scroll screen
- **Color bleeding**: Not resetting colors properly
- **Performance**: Too many small console writes
- **Unicode issues**: Non-ASCII characters may not display correctly

## Testing Strategy
- Render test patterns (solid fill, checkerboard, borders)
- Test color rendering with different combinations
- Measure frame rate and rendering time
- Test on different terminal sizes

## Debugging Tips
- Add frame counters to verify rendering is happening
- Use distinctive characters to verify buffer contents
- Test rendering with sleep delays to see individual frames
- Log rendering time for performance analysis

## Resources
- [ANSI Escape Codes Reference](https://gist.github.com/fnky/458719343aabd01cfb17a3a4f7296797)
- [Console I/O Performance in C++](https://stackoverflow.com/questions/213907/c-stdendl-vs-n)
- [String Performance in C++](https://www.cplusplus.com/reference/string/string/reserve/)

## Fun Challenges
- Implement fade-in/fade-out effects using color intensity
- Add support for background patterns or textures
- Create a simple bitmap font system for text
- Implement screen shake effects
- Add support for transparency/blending

## Refactoring Notes
In Phase 2, this becomes a `Renderer` class:
- Encapsulate rendering logic and state
- Provide high-level drawing functions (draw_line, draw_rect, draw_text)
- Abstract away platform-specific code
- Add render state management (current color, position, etc.)