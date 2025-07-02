# Phase 1.2: Buffer System Design

## C++ Learning Focus
- **2D Arrays**: Understanding `char buffer[HEIGHT][WIDTH]` syntax
- **Memory layout**: Row-major ordering in C++
- **Array indexing**: `buffer[y][x]` vs `buffer[x][y]` - mind the coordinate system!
- **Static vs dynamic allocation**: Start with static arrays, prepare for dynamic

## Implementation Overview
You'll create multiple buffers:
1. **Character buffer**: What character to display at each position
2. **Color buffer**: What color each character should be
3. **Front/back buffers**: Double buffering to prevent flicker

## Buffer Design Tips

### Character Buffer
```
char frontBuffer[HEIGHT][WIDTH];
char backBuffer[HEIGHT][WIDTH];
```
- Use `char` for ASCII characters (32-126 are printable)
- Consider using `unsigned char` if you want extended ASCII (128-255)
- Initialize with space character (32) or your background character

### Color Buffer (Optional for Phase 1)
```
int colorBuffer[HEIGHT][WIDTH];
```
- Store color codes (ANSI color numbers or platform-specific)
- Start with just foreground colors, add background later

## Key C++ Concepts to Learn
- **Array initialization**: `{0}` vs `memset()` vs loops
- **Nested loops**: Iterating through 2D arrays
- **Function parameters**: Passing 2D arrays to functions
- **Const correctness**: `const char buffer[][WIDTH]` for read-only access

## Implementation Tips

### Coordinate System
- Decide early: Is (0,0) top-left or bottom-left?
- Be consistent: `buffer[y][x]` where y=row, x=column
- Document your choice clearly!

### Buffer Operations
1. **Clear buffer**: Fill with background character/color
2. **Set pixel**: `buffer[y][x] = character`
3. **Get pixel**: `char c = buffer[y][x]`
4. **Bounds checking**: Always verify x,y are within range

### Double Buffering Flow
1. Clear back buffer
2. Draw everything to back buffer
3. Copy back buffer to front buffer (or swap pointers)
4. Display front buffer to console

## Memory Management Tips
- Start with static arrays (fixed size at compile time)
- Use `const int HEIGHT = 20; const int WIDTH = 20;` for dimensions
- Consider cache-friendly access patterns (row-wise iteration)
- Initialize buffers in a dedicated function

## Common Gotchas
- **Index confusion**: `buffer[x][y]` vs `buffer[y][x]`
- **Bounds checking**: Accessing `buffer[20][20]` when max is `[19][19]`
- **Uninitialized memory**: Always clear buffers before use
- **Character encoding**: Stick to basic ASCII initially

## Testing Strategy
- Fill buffer with test patterns (checkerboard, borders)
- Test bounds checking with debug assertions
- Verify clearing works properly
- Test individual pixel setting/getting

## Performance Considerations
- **Row-major access**: Iterate `for(y) for(x)` not `for(x) for(y)`
- **Minimize buffer copies**: Consider pointer swapping instead
- **Batch operations**: Clear entire buffer at once, not pixel by pixel

## Resources
- [C++ 2D Arrays Tutorial](https://www.learncpp.com/cpp-tutorial/multidimensional-arrays/)
- [Memory Layout in C++](https://stackoverflow.com/questions/2151084/how-are-multi-dimensional-arrays-formatted-in-memory)
- [Double Buffering Explanation](https://en.wikipedia.org/wiki/Multiple_buffering)

## Fun Challenges
- Implement buffer scrolling (shift all content up/down/left/right)
- Add a "dirty rectangle" system to only update changed areas
- Create buffer load/save functions for debugging
- Implement buffer blending operations

## Refactoring Notes
In Phase 2, this becomes a `Buffer` or `FrameBuffer` class:
- Encapsulate buffer data as private members
- Provide clean public interface for operations
- Add proper constructors/destructors for dynamic allocation
- Consider template parameters for buffer type (`Buffer<char>`, `Buffer<int>`)