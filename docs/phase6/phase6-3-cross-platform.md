# Phase 6.3: Cross-Platform Support

## C++ Learning Focus
- **Conditional Compilation**: Using preprocessor directives for platform-specific code
- **Platform Abstraction**: Creating unified interfaces for platform-dependent functionality
- **Build Systems**: CMake, Makefiles, and cross-platform build configuration
- **Compatibility Strategies**: Writing portable C++ code across different systems
- **API Wrapping**: Abstracting platform-specific APIs behind common interfaces
- **Deployment Techniques**: Building and packaging for multiple target platforms

## Implementation Overview
Architect your engine for seamless cross-platform operation:
1. Implement platform detection and conditional compilation
2. Create abstraction layers for platform-specific APIs
3. Design flexible build system configuration
4. Establish compatibility testing procedures
5. Develop deployment strategies for each platform
6. Optimize performance for platform-specific features

## Platform Detection Strategies

### Preprocessor-Based Detection
```cpp
// Platform.h - Central platform detection
#pragma once

// Primary platform detection
#if defined(_WIN32) || defined(_WIN64)
    #define PLATFORM_WINDOWS
    #ifdef _WIN64
        #define PLATFORM_64BIT
    #else
        #define PLATFORM_32BIT
    #endif
#elif defined(__APPLE__)
    #include <TargetConditionals.h>
    #if TARGET_OS_MAC
        #define PLATFORM_MACOS
        #define PLATFORM_64BIT
    #endif
#elif defined(__linux__)
    #define PLATFORM_LINUX
    #if defined(__x86_64__) || defined(__aarch64__)
        #define PLATFORM_64BIT
    #else
        #define PLATFORM_32BIT
    #endif
#elif defined(__unix__)
    #define PLATFORM_UNIX
#else
    #error "Unsupported platform"
#endif

// Compiler detection
#if defined(_MSC_VER)
    #define COMPILER_MSVC
    #define FORCE_INLINE __forceinline
#elif defined(__GNUC__)
    #define COMPILER_GCC
    #define FORCE_INLINE __attribute__((always_inline))
#elif defined(__clang__)
    #define COMPILER_CLANG
    #define FORCE_INLINE __attribute__((always_inline))
#endif

// Debug/Release detection
#ifdef NDEBUG
    #define BUILD_RELEASE
#else
    #define BUILD_DEBUG
#endif
```

### Runtime Platform Information
```cpp
// PlatformInfo.h
#pragma once
#include <string>

class PlatformInfo {
public:
    enum class Platform {
        Windows,
        macOS,
        Linux,
        Unix,
        Unknown
    };
    
    enum class Architecture {
        x86,
        x64,
        ARM,
        ARM64,
        Unknown
    };
    
    static Platform GetPlatform();
    static Architecture GetArchitecture();
    static std::string GetPlatformString();
    static std::string GetArchitectureString();
    static std::string GetCompilerInfo();
    static bool IsLittleEndian();
    static size_t GetPageSize();
    static size_t GetCacheLineSize();
};

// PlatformInfo.cpp
#include "PlatformInfo.h"
#include <sstream>

#ifdef PLATFORM_WINDOWS
    #include <windows.h>
#elif defined(PLATFORM_LINUX) || defined(PLATFORM_UNIX)
    #include <unistd.h>
    #include <sys/utsname.h>
#endif

PlatformInfo::Platform PlatformInfo::GetPlatform() {
#ifdef PLATFORM_WINDOWS
    return Platform::Windows;
#elif defined(PLATFORM_MACOS)
    return Platform::macOS;
#elif defined(PLATFORM_LINUX)
    return Platform::Linux;
#elif defined(PLATFORM_UNIX)
    return Platform::Unix;
#else
    return Platform::Unknown;
#endif
}

PlatformInfo::Architecture PlatformInfo::GetArchitecture() {
#ifdef PLATFORM_64BIT
    #if defined(__x86_64__) || defined(_M_X64)
        return Architecture::x64;
    #elif defined(__aarch64__) || defined(_M_ARM64)
        return Architecture::ARM64;
    #endif
#else
    #if defined(__i386__) || defined(_M_IX86)
        return Architecture::x86;
    #elif defined(__arm__) || defined(_M_ARM)
        return Architecture::ARM;
    #endif
#endif
    return Architecture::Unknown;
}
```

## API Abstraction Layers

### Console Interface Abstraction
```cpp
// ConsoleAPI.h - Platform-agnostic console interface
#pragma once
#include "Platform.h"
#include <string>

struct Color {
    uint8_t r, g, b, a = 255;
    
    Color(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0, uint8_t alpha = 255)
        : r(red), g(green), b(blue), a(alpha) {}
};

struct ConsoleSize {
    int width, height;
};

struct CursorPosition {
    int x, y;
};

class IConsoleAPI {
public:
    virtual ~IConsoleAPI() = default;
    
    // Console setup and cleanup
    virtual bool Initialize() = 0;
    virtual void Shutdown() = 0;
    
    // Buffer management
    virtual void Clear() = 0;
    virtual void Present() = 0;
    virtual void SetBufferSize(int width, int height) = 0;
    virtual ConsoleSize GetBufferSize() const = 0;
    
    // Character and color output
    virtual void SetCharacter(int x, int y, char ch) = 0;
    virtual void SetColor(int x, int y, const Color& foreground, const Color& background) = 0;
    virtual void WriteString(int x, int y, const std::string& text, const Color& color) = 0;
    
    // Cursor management
    virtual void SetCursorPosition(int x, int y) = 0;
    virtual CursorPosition GetCursorPosition() const = 0;
    virtual void ShowCursor(bool visible) = 0;
    
    // Input handling
    virtual bool HasInput() const = 0;
    virtual int GetKeyPress() = 0;
    virtual bool IsKeyPressed(int keyCode) const = 0;
    
    // Window management
    virtual void SetTitle(const std::string& title) = 0;
    virtual bool SetFullscreen(bool fullscreen) = 0;
};

// ConsoleFactory.h
class ConsoleFactory {
public:
    static std::unique_ptr<IConsoleAPI> CreateConsole();
};
```

### Windows Implementation
```cpp
// WindowsConsole.h
#pragma once
#include "ConsoleAPI.h"

#ifdef PLATFORM_WINDOWS
#include <windows.h>
#include <vector>

class WindowsConsole : public IConsoleAPI {
private:
    HANDLE hConsoleOutput;
    HANDLE hConsoleInput;
    CONSOLE_SCREEN_BUFFER_INFO originalBufferInfo;
    DWORD originalConsoleMode;
    
    std::vector<CHAR_INFO> buffer;
    int bufferWidth, bufferHeight;
    COORD bufferSize;
    SMALL_RECT writeRegion;
    
public:
    WindowsConsole();
    ~WindowsConsole();
    
    // IConsoleAPI implementation
    bool Initialize() override;
    void Shutdown() override;
    void Clear() override;
    void Present() override;
    void SetBufferSize(int width, int height) override;
    ConsoleSize GetBufferSize() const override;
    void SetCharacter(int x, int y, char ch) override;
    void SetColor(int x, int y, const Color& foreground, const Color& background) override;
    void WriteString(int x, int y, const std::string& text, const Color& color) override;
    void SetCursorPosition(int x, int y) override;
    CursorPosition GetCursorPosition() const override;
    void ShowCursor(bool visible) override;
    bool HasInput() const override;
    int GetKeyPress() override;
    bool IsKeyPressed(int keyCode) const override;
    void SetTitle(const std::string& title) override;
    bool SetFullscreen(bool fullscreen) override;
    
private:
    WORD ColorToWindowsAttribute(const Color& foreground, const Color& background);
    int WindowsKeyToStandard(DWORD virtualKey);
};
#endif
```

### Linux/Unix Implementation
```cpp
// UnixConsole.h
#pragma once
#include "ConsoleAPI.h"

#if defined(PLATFORM_LINUX) || defined(PLATFORM_UNIX) || defined(PLATFORM_MACOS)
#include <termios.h>
#include <vector>
#include <string>

class UnixConsole : public IConsoleAPI {
private:
    struct termios originalTermios;
    bool terminalModified;
    
    struct Cell {
        char character;
        Color foreground;
        Color background;
    };
    
    std::vector<std::vector<Cell>> buffer;
    int bufferWidth, bufferHeight;
    int cursorX, cursorY;
    bool cursorVisible;
    
public:
    UnixConsole();
    ~UnixConsole();
    
    // IConsoleAPI implementation
    bool Initialize() override;
    void Shutdown() override;
    void Clear() override;
    void Present() override;
    void SetBufferSize(int width, int height) override;
    ConsoleSize GetBufferSize() const override;
    void SetCharacter(int x, int y, char ch) override;
    void SetColor(int x, int y, const Color& foreground, const Color& background) override;
    void WriteString(int x, int y, const std::string& text, const Color& color) override;
    void SetCursorPosition(int x, int y) override;
    CursorPosition GetCursorPosition() const override;
    void ShowCursor(bool visible) override;
    bool HasInput() const override;
    int GetKeyPress() override;
    bool IsKeyPressed(int keyCode) const override;
    void SetTitle(const std::string& title) override;
    bool SetFullscreen(bool fullscreen) override;
    
private:
    void EnableRawMode();
    void DisableRawMode();
    std::string ColorToANSI(const Color& foreground, const Color& background);
    void MoveCursor(int x, int y);
    int UnixKeyToStandard(int key);
    void GetTerminalSize(int& width, int& height);
};
#endif
```

## Build System Configuration

### CMake Cross-Platform Setup
```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(ConsoleGameEngine VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Platform detection
if(WIN32)
    set(PLATFORM_WINDOWS TRUE)
    add_compile_definitions(PLATFORM_WINDOWS)
elseif(APPLE)
    set(PLATFORM_MACOS TRUE)
    add_compile_definitions(PLATFORM_MACOS)
elseif(UNIX)
    set(PLATFORM_LINUX TRUE)
    add_compile_definitions(PLATFORM_LINUX)
endif()

# Compiler-specific settings
if(MSVC)
    add_compile_definitions(COMPILER_MSVC)
    add_compile_options(/W4 /WX)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(/O2)
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_definitions(COMPILER_GCC)
    add_compile_options(-Wall -Wextra -Werror)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(-O3)
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_definitions(COMPILER_CLANG)
    add_compile_options(-Wall -Wextra -Werror)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(-O3)
    endif()
endif()

# Source files organization
set(CORE_SOURCES
    src/Engine/GameEngine.cpp
    src/Engine/Scene.cpp
    src/Engine/GameObject.cpp
    src/Rendering/Renderer.cpp
    src/Input/InputManager.cpp
    src/Utils/Timer.cpp
)

set(PLATFORM_SOURCES)
if(PLATFORM_WINDOWS)
    list(APPEND PLATFORM_SOURCES
        src/Platform/Windows/WindowsConsole.cpp
        src/Platform/Windows/WindowsInput.cpp
        src/Platform/Windows/WindowsTimer.cpp
    )
elseif(PLATFORM_LINUX OR PLATFORM_MACOS)
    list(APPEND PLATFORM_SOURCES
        src/Platform/Unix/UnixConsole.cpp
        src/Platform/Unix/UnixInput.cpp
        src/Platform/Unix/UnixTimer.cpp
    )
endif()

# Create executable
add_executable(${PROJECT_NAME}
    ${CORE_SOURCES}
    ${PLATFORM_SOURCES}
    src/main.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    src/
    src/Engine/
    src/Rendering/
    src/Input/
    src/Platform/
    src/Utils/
)

# Platform-specific linking
if(PLATFORM_WINDOWS)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        kernel32
        user32
        gdi32
    )
elseif(PLATFORM_LINUX)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        pthread
        dl
    )
elseif(PLATFORM_MACOS)
    find_library(CORE_FOUNDATION CoreFoundation)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ${CORE_FOUNDATION}
        pthread
    )
endif()

# Installation
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

# Platform-specific installation
if(PLATFORM_WINDOWS)
    install(FILES $<TARGET_PDB_FILE:${PROJECT_NAME}>
        DESTINATION bin
        OPTIONAL
    )
endif()
```

### Platform-Specific Build Scripts
```bash
#!/bin/bash
# build.sh - Cross-platform build script

set -e

# Default values
BUILD_TYPE="Release"
PLATFORM=""
CLEAN=false
INSTALL=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --release)
            BUILD_TYPE="Release"
            shift
            ;;
        --platform)
            PLATFORM="$2"
            shift 2
            ;;
        --clean)
            CLEAN=true
            shift
            ;;
        --install)
            INSTALL=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Detect platform if not specified
if [ -z "$PLATFORM" ]; then
    case "$(uname -s)" in
        Linux*)
            PLATFORM="linux"
            ;;
        Darwin*)
            PLATFORM="macos"
            ;;
        CYGWIN*|MINGW*|MSYS*)
            PLATFORM="windows"
            ;;
        *)
            echo "Unsupported platform: $(uname -s)"
            exit 1
            ;;
    esac
fi

echo "Building for platform: $PLATFORM"
echo "Build type: $BUILD_TYPE"

# Create build directory
BUILD_DIR="build-$PLATFORM-$(echo $BUILD_TYPE | tr '[:upper:]' '[:lower:]')"

if [ "$CLEAN" = true ]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure with CMake
cmake .. \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Build
cmake --build . --config "$BUILD_TYPE" --parallel

# Install if requested
if [ "$INSTALL" = true ]; then
    cmake --install . --config "$BUILD_TYPE"
fi

echo "Build completed successfully!"
```

## Testing Across Platforms

### Automated Testing Framework
```cpp
// PlatformTest.h
#pragma once
#include "Platform.h"
#include <string>
#include <vector>
#include <functional>

class PlatformTest {
public:
    struct TestResult {
        std::string name;
        bool passed;
        std::string error;
        double executionTime;
    };
    
    static void RunAllTests();
    static std::vector<TestResult> GetResults();
    
private:
    static void TestPlatformDetection();
    static void TestConsoleOperations();
    static void TestInputHandling();
    static void TestFileSystem();
    static void TestThreading();
    static void TestMemoryOperations();
    static void TestPerformance();
    
    static void RunTest(const std::string& name, std::function<void()> test);
    static std::vector<TestResult> results;
};

// PlatformTest.cpp
#include "PlatformTest.h"
#include "ConsoleAPI.h"
#include "PlatformInfo.h"
#include <chrono>
#include <iostream>
#include <thread>

std::vector<PlatformTest::TestResult> PlatformTest::results;

void PlatformTest::RunAllTests() {
    results.clear();
    
    std::cout << "Running platform compatibility tests...\n";
    std::cout << "Platform: " << PlatformInfo::GetPlatformString() << "\n";
    std::cout << "Architecture: " << PlatformInfo::GetArchitectureString() << "\n";
    std::cout << "Compiler: " << PlatformInfo::GetCompilerInfo() << "\n\n";
    
    RunTest("Platform Detection", TestPlatformDetection);
    RunTest("Console Operations", TestConsoleOperations);
    RunTest("Input Handling", TestInputHandling);
    RunTest("File System", TestFileSystem);
    RunTest("Threading", TestThreading);
    RunTest("Memory Operations", TestMemoryOperations);
    RunTest("Performance", TestPerformance);
    
    // Print results
    int passed = 0;
    for (const auto& result : results) {
        std::cout << "[" << (result.passed ? "PASS" : "FAIL") << "] "
                  << result.name << " (" << result.executionTime << "ms)";
        if (!result.passed) {
            std::cout << " - " << result.error;
        }
        std::cout << "\n";
        if (result.passed) passed++;
    }
    
    std::cout << "\nResults: " << passed << "/" << results.size() << " tests passed\n";
}

void PlatformTest::RunTest(const std::string& name, std::function<void()> test) {
    TestResult result;
    result.name = name;
    result.passed = false;
    result.error = "";
    
    auto start = std::chrono::high_resolution_clock::now();
    
    try {
        test();
        result.passed = true;
    } catch (const std::exception& e) {
        result.error = e.what();
    } catch (...) {
        result.error = "Unknown exception";
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    result.executionTime = duration.count() / 1000.0;
    
    results.push_back(result);
}
```

## Deployment Strategies

### Windows Deployment
```batch
@echo off
REM deploy-windows.bat
setlocal enabledelayedexpansion

set BUILD_TYPE=Release
set PLATFORM=windows
set VERSION=1.0.0

echo Building Windows deployment package...

REM Build the application
call build.bat --release --platform windows

REM Create deployment directory
set DEPLOY_DIR=deploy\%PLATFORM%-%VERSION%
if exist %DEPLOY_DIR% rmdir /s /q %DEPLOY_DIR%
mkdir %DEPLOY_DIR%

REM Copy executable and dependencies
copy build-windows-release\ConsoleGameEngine.exe %DEPLOY_DIR%\
copy README.md %DEPLOY_DIR%\
copy LICENSE %DEPLOY_DIR%\

REM Copy Visual C++ redistributables if needed
if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\VC\Redist\MSVC\14.29.30133\x64\Microsoft.VC142.CRT\" (
    xcopy "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\VC\Redist\MSVC\14.29.30133\x64\Microsoft.VC142.CRT\*" %DEPLOY_DIR%\ /Y
)

REM Create installer script
echo @echo off > %DEPLOY_DIR%\install.bat
echo echo Installing Console Game Engine... >> %DEPLOY_DIR%\install.bat
echo copy ConsoleGameEngine.exe "%ProgramFiles%\ConsoleGameEngine\" >> %DEPLOY_DIR%\install.bat
echo echo Installation complete! >> %DEPLOY_DIR%\install.bat

echo Windows deployment package created in %DEPLOY_DIR%
```

### Linux/macOS Deployment
```bash
#!/bin/bash
# deploy-unix.sh

set -e

BUILD_TYPE="Release"
VERSION="1.0.0"

case "$(uname -s)" in
    Linux*)
        PLATFORM="linux"
        ;;
    Darwin*)
        PLATFORM="macos"
        ;;
    *)
        echo "Unsupported platform"
        exit 1
        ;;
esac

echo "Building $PLATFORM deployment package..."

# Build the application
./build.sh --release --platform "$PLATFORM"

# Create deployment directory
DEPLOY_DIR="deploy/$PLATFORM-$VERSION"
rm -rf "$DEPLOY_DIR"
mkdir -p "$DEPLOY_DIR"

# Copy executable and resources
cp "build-$PLATFORM-release/ConsoleGameEngine" "$DEPLOY_DIR/"
cp README.md "$DEPLOY_DIR/"
cp LICENSE "$DEPLOY_DIR/"

# Create installation script
cat > "$DEPLOY_DIR/install.sh" << 'EOF'
#!/bin/bash
echo "Installing Console Game Engine..."

# Install to /usr/local/bin
sudo cp ConsoleGameEngine /usr/local/bin/
sudo chmod +x /usr/local/bin/ConsoleGameEngine

# Create desktop entry (Linux only)
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    mkdir -p ~/.local/share/applications
    cat > ~/.local/share/applications/console-game-engine.desktop << 'DESKTOP_EOF'
[Desktop Entry]
Version=1.0
Type=Application
Name=Console Game Engine
Comment=A cross-platform console-based game engine
Exec=/usr/local/bin/ConsoleGameEngine
Icon=applications-games
Terminal=true
Categories=Game;Development;
DESKTOP_EOF
fi

echo "Installation complete!"
EOF

chmod +x "$DEPLOY_DIR/install.sh"

# Create tarball
tar -czf "deploy/console-game-engine-$PLATFORM-$VERSION.tar.gz" -C deploy "$PLATFORM-$VERSION"

echo "$PLATFORM deployment package created in $DEPLOY_DIR"
echo "Tarball created: deploy/console-game-engine-$PLATFORM-$VERSION.tar.gz"
```

## Platform-Specific Optimizations

### Performance Tuning
```cpp
// PlatformOptimizations.h
#pragma once
#include "Platform.h"

class PlatformOptimizations {
public:
    // Memory alignment for optimal performance
    static constexpr size_t GetOptimalAlignment() {
#ifdef PLATFORM_WINDOWS
        return 16; // SIMD alignment
#elif defined(PLATFORM_MACOS)
        return 16; // Apple Silicon optimization
#else
        return 8;  // Standard alignment
#endif
    }
    
    // Thread count optimization
    static size_t GetOptimalThreadCount();
    
    // Memory prefetching
    template<typename T>
    static void Prefetch(const T* ptr) {
#ifdef COMPILER_MSVC
        _mm_prefetch(reinterpret_cast<const char*>(ptr), _MM_HINT_T0);
#elif defined(COMPILER_GCC) || defined(COMPILER_CLANG)
        __builtin_prefetch(ptr, 0, 3);
#endif
    }
    
    // Branch prediction hints
    static bool Likely(bool condition) {
#if defined(COMPILER_GCC) || defined(COMPILER_CLANG)
        return __builtin_expect(condition, 1);
#else
        return condition;
#endif
    }
    
    static bool Unlikely(bool condition) {
#if defined(COMPILER_GCC) || defined(COMPILER_CLANG)
        return __builtin_expect(condition, 0);
#else
        return condition;
#endif
    }
};

// Platform-specific memory allocators
class PlatformAllocator {
public:
    static void* AllocateAligned(size_t size, size_t alignment = PlatformOptimizations::GetOptimalAlignment());
    static void DeallocateAligned(void* ptr);
    
#ifdef PLATFORM_WINDOWS
    static void* AllocateVirtual(size_t size);
    static void DeallocateVirtual(void* ptr, size_t size);
#endif
    
private:
    static void* AllocateAlignedImpl(size_t size, size_t alignment);
    static void DeallocateAlignedImpl(void* ptr);
};
```

## Compatibility Maintenance

### Version Compatibility System
```cpp
// CompatibilityManager.h
#pragma once
#include <string>
#include <vector>
#include <functional>

struct Version {
    int major, minor, patch;
    
    Version(int maj = 0, int min = 0, int pat = 0)
        : major(maj), minor(min), patch(pat) {}
    
    std::string ToString() const;
    bool IsCompatibleWith(const Version& other) const;
    
    bool operator<(const Version& other) const;
    bool operator==(const Version& other) const;
    bool operator>(const Version& other) const;
};

class CompatibilityManager {
public:
    struct CompatibilityCheck {
        std::string name;
        Version minVersion;
        Version maxVersion;
        std::function<bool()> checkFunction;
    };
    
    static void RegisterCheck(const CompatibilityCheck& check);
    static bool RunCompatibilityChecks();
    static std::vector<std::string> GetFailedChecks();
    static Version GetEngineVersion();
    static Version GetPlatformVersion();
    
private:
    static std::vector<CompatibilityCheck> checks;
    static std::vector<std::string> failedChecks;
};

// Usage example
void RegisterCompatibilityChecks() {
    // Console API compatibility
    CompatibilityManager::RegisterCheck({
        "Console Buffer Operations",
        Version(1, 0, 0),
        Version(2, 0, 0),
        []() {
            auto console = ConsoleFactory::CreateConsole();
            return console->Initialize();
        }
    });
    
    // Threading support
    CompatibilityManager::RegisterCheck({
        "Threading Support",
        Version(1, 0, 0),
        Version(2, 0, 0),
        []() {
            try {
                std::thread t([](){});
                t.join();
                return true;
            } catch (...) {
                return false;
            }
        }
    });
}
```

## Integration Examples

### Cross-Platform Game Example
```cpp
// CrossPlatformGame.cpp
#include "GameEngine.h"
#include "PlatformInfo.h"
#include "CompatibilityManager.h"
#include <iostream>

class CrossPlatformGame : public Game {
private:
    std::unique_ptr<IConsoleAPI> console;
    bool platformOptimized;
    
public:
    bool Initialize() override {
        // Check platform compatibility
        std::cout << "Initializing on " << PlatformInfo::GetPlatformString() 
                  << " (" << PlatformInfo::GetArchitectureString() << ")\n";
        
        if (!CompatibilityManager::RunCompatibilityChecks()) {
            std::cerr << "Compatibility check failed!\n";
            auto failed = CompatibilityManager::GetFailedChecks();
            for (const auto& check : failed) {
                std::cerr << "  - " << check << "\n";
            }
            return false;
        }
        
        // Initialize platform-specific console
        console = ConsoleFactory::CreateConsole();
        if (!console->Initialize()) {
            std::cerr << "Failed to initialize console\n";
            return false;
        }
        
        // Apply platform-specific optimizations
        ApplyPlatformOptimizations();
        
        console->SetTitle("Cross-Platform Console Game Engine");
        console->SetBufferSize(80, 25);
        console->Clear();
        
        return true;
    }
    
    void Update(float deltaTime) override {
        // Platform-agnostic game logic
        if (console->HasInput()) {
            int key = console->GetKeyPress();
            if (key == 27) { // ESC key
                SetRunning(false);
            }
        }
    }
    
    void Render() override {
        console->Clear();
        
        // Display platform information
        std::string info = "Platform: " + PlatformInfo::GetPlatformString();
        console->WriteString(1, 1, info, Color(255, 255, 255));
        
        info = "Architecture: " + PlatformInfo::GetArchitectureString();
        console->WriteString(1, 2, info, Color(255, 255, 255));
        
        info = "Optimized: " + std::string(platformOptimized ? "Yes" : "No");
        console->WriteString(1, 3, info, Color(255, 255, 255));
        
        console->WriteString(1, 5, "Press ESC to exit", Color(128, 128, 128));
        
        console->Present();
    }
    
    void Shutdown() override {
        if (console) {
            console->Shutdown();
        }
    }
    
private:
    void ApplyPlatformOptimizations() {
        platformOptimized = false;
        
#ifdef PLATFORM_WINDOWS
        // Windows-specific optimizations
        SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED);
        platformOptimized = true;
#elif defined(PLATFORM_LINUX)
        // Linux-specific optimizations
        // Enable high-resolution timers if available
        platformOptimized = true;
#elif defined(PLATFORM_MACOS)
        // macOS-specific optimizations
        platformOptimized = true;
#endif
    }
};

int main() {
    CrossPlatformGame game;
    
    if (!game.Initialize()) {
        std::cerr << "Failed to initialize game\n";
        return 1;
    }
    
    game.Run();
    game.Shutdown();
    
    return 0;
}
```

## Key C++ Concepts

### 1. Preprocessor Directives
```cpp
// Advanced conditional compilation
#if defined(PLATFORM_WINDOWS) && defined(COMPILER_MSVC)
    #pragma warning(push)
    #pragma warning(disable: 4996) // Disable deprecated function warnings
    #include <windows.h>
    #pragma warning(pop)
#elif defined(PLATFORM_LINUX) || defined(PLATFORM_MACOS)
    #include <unistd.h>
    #include <termios.h>
#else
    #error "Unsupported platform configuration"
#endif

// Feature detection
#ifdef __has_include
    #if __has_include(<filesystem>)
        #include <filesystem>
        #define HAS_FILESYSTEM 1
    #else
        #include <experimental/filesystem>
        #define HAS_FILESYSTEM 0
    #endif
#endif
```

### 2. Template Specialization for Platforms
```cpp
// Platform-specific template specializations
template<typename T>
struct PlatformTraits {
    static constexpr size_t alignment = alignof(T);
    static constexpr bool is_copyable = std::is_trivially_copyable_v<T>;
};

#ifdef PLATFORM_WINDOWS
template<>
struct PlatformTraits<float> {
    static constexpr size_t alignment = 16; // SSE alignment
    static constexpr bool is_copyable = true;
};
#endif

template<typename T>
class PlatformVector {
private:
    alignas(PlatformTraits<T>::alignment) std::vector<T> data;
    
public:
    void push_back(const T& value) {
        if constexpr (PlatformTraits<T>::is_copyable) {
            // Optimized path for copyable types
            data.push_back(value);
        } else {
            // Generic path
            data.emplace_back(value);
        }
    }
};
```

## Resources and Tools

### Development Tools
- **CMake**: Cross-platform build system generator
- **Conan**: C++ package manager for dependencies
- **vcpkg**: Microsoft's C++ package manager
- **Docker**: Containerized build environments
- **GitHub Actions**: Automated CI/CD for multiple platforms

### Testing Frameworks
- **GoogleTest**: Cross-platform C++ testing framework
- **Catch2**: Modern C++ testing framework
- **doctest**: Lightweight testing framework

### Debugging Tools
- **GDB**: GNU Debugger for Linux/macOS
- **LLDB**: LLVM Debugger for macOS/Linux
- **Visual Studio Debugger**: Windows debugging
- **Valgrind**: Memory debugging for Linux

### Profiling Tools
- **Intel VTune**: Performance profiling
- **Perf**: Linux performance analysis
- **Instruments**: macOS profiling tool
- **Visual Studio Profiler**: Windows performance analysis

## Fun Challenges

1. **Universal Input Mapper**: Create a system that maps platform-specific input codes to universal game actions
2. **Performance Benchmark Suite**: Build comprehensive benchmarks that compare performance across platforms
3. **Automatic Platform Detection**: Implement runtime detection of platform capabilities and features
4. **Cross-Platform Asset Pipeline**: Design a system for converting assets to platform-optimal formats
5. **Universal Color Management**: Create consistent color representation across different console implementations

## Debugging Tips

### Platform-Specific Debugging
```cpp
// Debug output macro that works on all platforms
#ifdef BUILD_DEBUG
    #ifdef PLATFORM_WINDOWS
        #define DEBUG_PRINT(msg) OutputDebugStringA((msg "\n"))
    #else
        #define DEBUG_PRINT(msg) std::cout << msg << std::endl
    #endif
#else
    #define DEBUG_PRINT(msg)
#endif

// Platform-specific breakpoints
#ifdef BUILD_DEBUG
    #ifdef PLATFORM_WINDOWS
        #define DEBUG_BREAK() __debugbreak()
    #elif defined(COMPILER_GCC) || defined(COMPILER_CLANG)
        #define DEBUG_BREAK() __builtin_trap()
    #else
        #define DEBUG_BREAK() abort()
    #endif
#else
    #define DEBUG_BREAK()
#endif
```

### Common Platform Issues
1. **Endianness**: Different byte ordering on different architectures
2. **Path Separators**: Windows uses backslashes, Unix uses forward slashes
3. **Line Endings**: CRLF vs LF differences
4. **Case Sensitivity**: Windows filesystem is case-insensitive
5. **Thread Stack Sizes**: Different default stack sizes across platforms

## Porting Process

### Step-by-Step Porting Guide
1. **Audit Current Code**: Identify platform-specific dependencies
2. **Create Abstraction Layer**: Design interfaces for platform-specific functionality
3. **Implement Platform Modules**: Create implementations for each target platform
4. **Update Build System**: Configure cross-platform build scripts
5. **Test Thoroughly**: Run comprehensive tests on all target platforms
6. **Optimize Per Platform**: Apply platform-specific optimizations
7. **Document Differences**: Record platform-specific behaviors and limitations

This comprehensive guide provides the foundation for creating truly cross-platform C++ applications, emphasizing clean abstraction, robust testing, and maintainable code organization.