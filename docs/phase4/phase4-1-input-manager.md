# Phase 4.1: InputManager Class Design

## C++ Learning Focus
- **Singleton pattern**: Global input access and lifetime management
- **Event systems**: Observer pattern and callback mechanisms
- **Function pointers/callbacks**: std::function and lambda expressions
- **STL containers**: std::unordered_map, std::vector for efficient lookups
- **Enums and type safety**: Strong typing for keys and actions
- **Template programming**: Generic callback system design

## Implementation Overview
You'll create a centralized input management system that:
1. Abstracts platform-specific input handling
2. Maps raw keys to logical actions
3. Tracks key states (pressed, held, released)
4. Dispatches events to registered listeners
5. Buffers input for frame-consistent processing
6. Provides both polling and event-driven interfaces

## Class Structure Design

### Basic InputManager Interface
```cpp
class InputManager {
public:
    static InputManager& getInstance();
    
    // Lifecycle
    bool initialize();
    void shutdown();
    void update(); // Call once per frame
    
    // Key state queries
    bool isKeyPressed(Key key) const;
    bool isKeyHeld(Key key) const;
    bool isKeyReleased(Key key) const;
    
    // Action mapping
    void mapKeyToAction(Key key, const std::string& action);
    bool isActionPressed(const std::string& action) const;
    bool isActionHeld(const std::string& action) const;
    
    // Event registration
    using KeyCallback = std::function<void(Key, KeyState)>;
    using ActionCallback = std::function<void(const std::string&, bool)>;
    
    void registerKeyCallback(KeyCallback callback);
    void registerActionCallback(const std::string& action, ActionCallback callback);
    
private:
    InputManager() = default;
    ~InputManager() = default;
    InputManager(const InputManager&) = delete;
    InputManager& operator=(const InputManager&) = delete;
};
```

### Key and State Enumerations
```cpp
enum class Key {
    Unknown = 0,
    
    // Letters
    A, B, C, D, E, F, G, H, I, J, K, L, M,
    N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
    
    // Numbers
    Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
    
    // Arrow keys
    Up, Down, Left, Right,
    
    // Function keys
    F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12,
    
    // Special keys
    Enter, Escape, Space, Tab, Backspace, Delete,
    Shift, Ctrl, Alt,
    
    // Custom engine keys
    Quit, Pause, Debug
};

enum class KeyState {
    Released = 0,
    Pressed,    // Just pressed this frame
    Held        // Held down (was pressed last frame too)
};
```

## Key C++ Concepts to Learn

### Singleton Implementation
```cpp
class InputManager {
private:
    static std::unique_ptr<InputManager> instance;
    static std::once_flag initFlag;
    
public:
    static InputManager& getInstance() {
        std::call_once(initFlag, []() {
            instance = std::make_unique<InputManager>();
        });
        return *instance;
    }
    
    // Alternative simpler singleton (not thread-safe in C++98)
    static InputManager& getInstanceSimple() {
        static InputManager instance;
        return instance;
    }
};
```

### Callback System with std::function
```cpp
class InputManager {
private:
    std::vector<KeyCallback> keyCallbacks;
    std::unordered_map<std::string, std::vector<ActionCallback>> actionCallbacks;
    
public:
    void registerKeyCallback(KeyCallback callback) {
        keyCallbacks.push_back(std::move(callback));
    }
    
    void dispatchKeyEvent(Key key, KeyState state) {
        for(const auto& callback : keyCallbacks) {
            callback(key, state);
        }
    }
    
    // Lambda usage example
    void setupPlayerInput() {
        registerActionCallback("move_up", [](const std::string& action, bool pressed) {
            if(pressed) player.startMovingUp();
            else player.stopMovingUp();
        });
    }
};
```

### Container Performance Optimization
```cpp
class InputManager {
private:
    // Fast key state lookup - O(1) access
    std::array<KeyState, static_cast<size_t>(Key::COUNT)> keyStates;
    std::array<KeyState, static_cast<size_t>(Key::COUNT)> previousKeyStates;
    
    // Action mapping - O(1) average lookup
    std::unordered_map<Key, std::vector<std::string>> keyToActions;
    std::unordered_map<std::string, std::vector<Key>> actionToKeys;
    
public:
    bool isKeyPressed(Key key) const {
        size_t index = static_cast<size_t>(key);
        return keyStates[index] == KeyState::Pressed;
    }
    
    void updateKeyStates() {
        // Copy current to previous for next frame comparison
        previousKeyStates = keyStates;
        
        // Update current states from platform input
        updatePlatformInput();
        
        // Determine press/release events
        for(size_t i = 0; i < keyStates.size(); ++i) {
            if(keyStates[i] == KeyState::Held && previousKeyStates[i] == KeyState::Released) {
                keyStates[i] = KeyState::Pressed;
            }
            else if(keyStates[i] == KeyState::Pressed && previousKeyStates[i] == KeyState::Pressed) {
                keyStates[i] = KeyState::Held;
            }
        }
    }
};
```

## Implementation Strategies

### Input Mapping System
```cpp
class InputManager {
private:
    struct ActionBinding {
        std::string name;
        std::vector<Key> keys;
        bool requiresAllKeys;  // AND vs OR logic
    };
    
    std::vector<ActionBinding> actionBindings;
    
public:
    void createAction(const std::string& name) {
        actionBindings.push_back({name, {}, false});
    }
    
    void bindKeyToAction(const std::string& action, Key key) {
        auto it = std::find_if(actionBindings.begin(), actionBindings.end(),
            [&action](const ActionBinding& binding) {
                return binding.name == action;
            });
        
        if(it != actionBindings.end()) {
            it->keys.push_back(key);
        }
    }
    
    bool isActionActive(const std::string& action) const {
        auto it = std::find_if(actionBindings.begin(), actionBindings.end(),
            [&action](const ActionBinding& binding) {
                return binding.name == action;
            });
        
        if(it == actionBindings.end()) return false;
        
        if(it->requiresAllKeys) {
            // All keys must be pressed (combo)
            return std::all_of(it->keys.begin(), it->keys.end(),
                [this](Key key) { return isKeyHeld(key) || isKeyPressed(key); });
        } else {
            // Any key can trigger action
            return std::any_of(it->keys.begin(), it->keys.end(),
                [this](Key key) { return isKeyPressed(key); });
        }
    }
};
```

### Key State Tracking
```cpp
class InputManager {
private:
    struct KeyInfo {
        KeyState currentState = KeyState::Released;
        KeyState previousState = KeyState::Released;
        double pressTime = 0.0;
        double releaseTime = 0.0;
    };
    
    std::array<KeyInfo, static_cast<size_t>(Key::COUNT)> keyInfo;
    double currentTime = 0.0;
    
public:
    void update(double deltaTime) {
        currentTime += deltaTime;
        
        // Update key states
        for(auto& info : keyInfo) {
            info.previousState = info.currentState;
        }
        
        // Get platform input and update current states
        updatePlatformInput();
        
        // Detect state changes and record timing
        for(size_t i = 0; i < keyInfo.size(); ++i) {
            auto& info = keyInfo[i];
            
            if(info.currentState != info.previousState) {
                if(info.currentState == KeyState::Pressed) {
                    info.pressTime = currentTime;
                    dispatchKeyEvent(static_cast<Key>(i), KeyState::Pressed);
                }
                else if(info.currentState == KeyState::Released) {
                    info.releaseTime = currentTime;
                    dispatchKeyEvent(static_cast<Key>(i), KeyState::Released);
                }
            }
        }
    }
    
    double getKeyHoldDuration(Key key) const {
        const auto& info = keyInfo[static_cast<size_t>(key)];
        if(info.currentState == KeyState::Held || info.currentState == KeyState::Pressed) {
            return currentTime - info.pressTime;
        }
        return 0.0;
    }
};
```

### Platform Abstraction
```cpp
class InputManager {
private:
    std::unique_ptr<PlatformInput> platformInput;
    
    void updatePlatformInput() {
        platformInput->pollEvents();
        
        // Convert platform-specific key codes to our Key enum
        auto platformKeys = platformInput->getPressedKeys();
        
        // Reset all keys to released first
        for(auto& info : keyInfo) {
            if(info.currentState != KeyState::Released) {
                info.currentState = KeyState::Released;
            }
        }
        
        // Set currently pressed keys
        for(auto platformKey : platformKeys) {
            Key engineKey = platformInput->convertToEngineKey(platformKey);
            if(engineKey != Key::Unknown) {
                auto& info = keyInfo[static_cast<size_t>(engineKey)];
                if(info.previousState == KeyState::Released) {
                    info.currentState = KeyState::Pressed;
                } else {
                    info.currentState = KeyState::Held;
                }
            }
        }
    }
};

// Platform abstraction interface
class PlatformInput {
public:
    virtual ~PlatformInput() = default;
    virtual void pollEvents() = 0;
    virtual std::vector<int> getPressedKeys() const = 0;
    virtual Key convertToEngineKey(int platformKey) const = 0;
};
```

## Event-Driven Architecture

### Observer Pattern Implementation
```cpp
class InputManager {
public:
    class InputObserver {
    public:
        virtual ~InputObserver() = default;
        virtual void onKeyPressed(Key key) {}
        virtual void onKeyReleased(Key key) {}
        virtual void onActionTriggered(const std::string& action) {}
    };
    
private:
    std::vector<std::weak_ptr<InputObserver>> observers;
    
public:
    void registerObserver(std::shared_ptr<InputObserver> observer) {
        observers.push_back(observer);
    }
    
    void notifyKeyPressed(Key key) {
        // Clean up expired weak_ptrs and notify valid ones
        auto it = std::remove_if(observers.begin(), observers.end(),
            [key](const std::weak_ptr<InputObserver>& weakObs) {
                if(auto obs = weakObs.lock()) {
                    obs->onKeyPressed(key);
                    return false; // Keep valid observers
                }
                return true; // Remove expired observers
            });
        observers.erase(it, observers.end());
    }
};

// Usage example
class Player : public InputManager::InputObserver {
public:
    void onActionTriggered(const std::string& action) override {
        if(action == "move_up") startMovingUp();
        else if(action == "jump") jump();
        else if(action == "attack") attack();
    }
};
```

### Event Queue System
```cpp
class InputManager {
private:
    struct InputEvent {
        enum Type { KeyPressed, KeyReleased, ActionTriggered };
        Type type;
        Key key;
        std::string action;
        double timestamp;
    };
    
    std::queue<InputEvent> eventQueue;
    std::mutex eventQueueMutex; // For thread safety
    
public:
    void queueEvent(InputEvent::Type type, Key key, const std::string& action = "") {
        std::lock_guard<std::mutex> lock(eventQueueMutex);
        eventQueue.push({type, key, action, currentTime});
    }
    
    void processEvents() {
        std::lock_guard<std::mutex> lock(eventQueueMutex);
        
        while(!eventQueue.empty()) {
            const auto& event = eventQueue.front();
            
            switch(event.type) {
                case InputEvent::KeyPressed:
                    notifyKeyPressed(event.key);
                    break;
                case InputEvent::KeyReleased:
                    notifyKeyReleased(event.key);
                    break;
                case InputEvent::ActionTriggered:
                    notifyActionTriggered(event.action);
                    break;
            }
            
            eventQueue.pop();
        }
    }
};
```

## Input Buffering and Frame Consistency

### Frame-Based Input Processing
```cpp
class InputManager {
private:
    struct FrameInput {
        std::bitset<static_cast<size_t>(Key::COUNT)> pressedThisFrame;
        std::bitset<static_cast<size_t>(Key::COUNT)> releasedThisFrame;
        std::bitset<static_cast<size_t>(Key::COUNT)> heldThisFrame;
    };
    
    FrameInput currentFrame;
    FrameInput previousFrame;
    
public:
    void beginFrame() {
        previousFrame = currentFrame;
        currentFrame.pressedThisFrame.reset();
        currentFrame.releasedThisFrame.reset();
        // heldThisFrame is updated during input polling
    }
    
    void endFrame() {
        // Process any remaining events
        processEvents();
        
        // Update held states based on press/release events
        for(size_t i = 0; i < currentFrame.heldThisFrame.size(); ++i) {
            if(currentFrame.pressedThisFrame[i]) {
                currentFrame.heldThisFrame[i] = true;
            }
            else if(currentFrame.releasedThisFrame[i]) {
                currentFrame.heldThisFrame[i] = false;
            }
        }
    }
    
    bool wasKeyPressedThisFrame(Key key) const {
        return currentFrame.pressedThisFrame[static_cast<size_t>(key)];
    }
    
    bool wasKeyReleasedThisFrame(Key key) const {
        return currentFrame.releasedThisFrame[static_cast<size_t>(key)];
    }
};
```

### Input Prediction and Buffering
```cpp
class InputManager {
private:
    static constexpr size_t BUFFER_SIZE = 60; // 1 second at 60 FPS
    
    struct BufferedInput {
        std::array<FrameInput, BUFFER_SIZE> frames;
        size_t currentIndex = 0;
        size_t frameCount = 0;
    };
    
    BufferedInput inputBuffer;
    
public:
    void bufferCurrentFrame() {
        inputBuffer.frames[inputBuffer.currentIndex] = currentFrame;
        inputBuffer.currentIndex = (inputBuffer.currentIndex + 1) % BUFFER_SIZE;
        inputBuffer.frameCount = std::min(inputBuffer.frameCount + 1, BUFFER_SIZE);
    }
    
    bool wasKeyPressedInLastFrames(Key key, size_t frameCount) const {
        frameCount = std::min(frameCount, inputBuffer.frameCount);
        
        for(size_t i = 0; i < frameCount; ++i) {
            size_t index = (inputBuffer.currentIndex - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
            if(inputBuffer.frames[index].pressedThisFrame[static_cast<size_t>(key)]) {
                return true;
            }
        }
        return false;
    }
    
    // Input prediction for networked games
    FrameInput predictNextFrame() const {
        // Simple prediction: assume held keys continue to be held
        FrameInput predicted;
        predicted.heldThisFrame = currentFrame.heldThisFrame;
        return predicted;
    }
};
```

## Configuration System

### Input Configuration Management
```cpp
struct InputConfig {
    struct KeyBinding {
        std::string action;
        std::vector<Key> keys;
        bool requiresAllKeys = false;
    };
    
    std::vector<KeyBinding> keyBindings;
    double keyRepeatDelay = 0.5;      // Time before key repeat starts
    double keyRepeatRate = 0.1;       // Time between repeats
    bool enableKeyRepeat = true;
    
    // Load from file
    bool loadFromFile(const std::string& filename);
    bool saveToFile(const std::string& filename) const;
    
    // Create default bindings
    static InputConfig createDefault();
};

class InputManager {
private:
    InputConfig config;
    
public:
    void loadConfiguration(const InputConfig& cfg) {
        config = cfg;
        rebuildActionMappings();
    }
    
private:
    void rebuildActionMappings() {
        // Clear existing mappings
        actionBindings.clear();
        
        // Rebuild from configuration
        for(const auto& binding : config.keyBindings) {
            ActionBinding actionBinding;
            actionBinding.name = binding.action;
            actionBinding.keys = binding.keys;
            actionBinding.requiresAllKeys = binding.requiresAllKeys;
            actionBindings.push_back(actionBinding);
        }
    }
};

// JSON-based configuration example
InputConfig InputConfig::loadFromFile(const std::string& filename) {
    InputConfig config;
    // Pseudo-code for JSON loading
    /*
    {
        "keyBindings": [
            {
                "action": "move_up",
                "keys": ["W", "Up"],
                "requiresAllKeys": false
            },
            {
                "action": "special_attack",
                "keys": ["Ctrl", "Space"],
                "requiresAllKeys": true
            }
        ],
        "keyRepeatDelay": 0.5,
        "keyRepeatRate": 0.1
    }
    */
    return config;
}
```

## Thread Safety Considerations

### Thread-Safe Input Access
```cpp
class InputManager {
private:
    mutable std::shared_mutex inputMutex;
    std::atomic<bool> inputSystemActive{false};
    
public:
    bool isKeyPressed(Key key) const {
        std::shared_lock<std::shared_mutex> lock(inputMutex);
        return keyStates[static_cast<size_t>(key)] == KeyState::Pressed;
    }
    
    void update() {
        std::unique_lock<std::shared_mutex> lock(inputMutex);
        
        if(!inputSystemActive.load()) return;
        
        updatePlatformInput();
        processEvents();
        bufferCurrentFrame();
    }
    
    void setActive(bool active) {
        inputSystemActive.store(active);
    }
};
```

## Testing Strategies

### Unit Testing Input Logic
```cpp
class MockPlatformInput : public PlatformInput {
private:
    std::vector<int> simulatedKeys;
    
public:
    void simulateKeyPress(Key key) {
        int platformKey = convertFromEngineKey(key);
        simulatedKeys.push_back(platformKey);
    }
    
    void simulateKeyRelease(Key key) {
        int platformKey = convertFromEngineKey(key);
        auto it = std::find(simulatedKeys.begin(), simulatedKeys.end(), platformKey);
        if(it != simulatedKeys.end()) {
            simulatedKeys.erase(it);
        }
    }
    
    std::vector<int> getPressedKeys() const override {
        return simulatedKeys;
    }
};

// Test example
void testKeyPressSequence() {
    InputManager& input = InputManager::getInstance();
    auto mockInput = std::make_unique<MockPlatformInput>();
    auto* mock = mockInput.get();
    
    input.setPlatformInput(std::move(mockInput));
    
    // Test sequence: press W, hold for 3 frames, release
    mock->simulateKeyPress(Key::W);
    input.update(1.0/60.0); // Frame 1
    assert(input.isKeyPressed(Key::W));
    assert(!input.isKeyHeld(Key::W));
    
    input.update(1.0/60.0); // Frame 2
    assert(!input.isKeyPressed(Key::W));
    assert(input.isKeyHeld(Key::W));
    
    input.update(1.0/60.0); // Frame 3
    assert(input.isKeyHeld(Key::W));
    
    mock->simulateKeyRelease(Key::W);
    input.update(1.0/60.0); // Frame 4
    assert(input.isKeyReleased(Key::W));
    assert(!input.isKeyHeld(Key::W));
}
```

### Integration Testing
```cpp
class InputTestScene : public Scene {
private:
    std::vector<std::string> testResults;
    
public:
    void initialize() override {
        // Register for input events
        auto& input = InputManager::getInstance();
        
        input.mapKeyToAction(Key::W, "move_up");
        input.mapKeyToAction(Key::S, "move_down");
        
        input.registerActionCallback("move_up", [this](const std::string& action, bool pressed) {
            testResults.push_back("move_up: " + std::to_string(pressed));
        });
    }
    
    void update(double deltaTime) override {
        // Display test results on screen
        for(size_t i = 0; i < testResults.size(); ++i) {
            renderer.drawText(10, 10 + i * 20, testResults[i]);
        }
        
        // Clear old results
        if(testResults.size() > 10) {
            testResults.erase(testResults.begin());
        }
    }
};
```

## Integration Examples

### Player Controller Integration
```cpp
class PlayerController : public InputManager::InputObserver {
private:
    Player* player;
    
public:
    PlayerController(Player* p) : player(p) {
        auto& input = InputManager::getInstance();
        
        // Setup action mappings
        input.createAction("move_up");
        input.createAction("move_down");
        input.createAction("move_left");
        input.createAction("move_right");
        input.createAction("jump");
        input.createAction("attack");
        
        // Bind keys to actions
        input.bindKeyToAction("move_up", Key::W);
        input.bindKeyToAction("move_up", Key::Up);
        input.bindKeyToAction("move_down", Key::S);
        input.bindKeyToAction("move_down", Key::Down);
        input.bindKeyToAction("move_left", Key::A);
        input.bindKeyToAction("move_left", Key::Left);
        input.bindKeyToAction("move_right", Key::D);
        input.bindKeyToAction("move_right", Key::Right);
        input.bindKeyToAction("jump", Key::Space);
        input.bindKeyToAction("attack", Key::Enter);
        
        // Register for callbacks
        auto observer = std::shared_ptr<InputObserver>(this, [](InputObserver*){});
        input.registerObserver(observer);
    }
    
    void onActionTriggered(const std::string& action) override {
        if(action == "move_up") player->setVelocityY(-1);
        else if(action == "move_down") player->setVelocityY(1);
        else if(action == "move_left") player->setVelocityX(-1);
        else if(action == "move_right") player->setVelocityX(1);
        else if(action == "jump") player->jump();
        else if(action == "attack") player->attack();
    }
    
    void update() {
        auto& input = InputManager::getInstance();
        
        // Handle continuous movement
        float moveX = 0, moveY = 0;
        
        if(input.isActionHeld("move_left")) moveX -= 1;
        if(input.isActionHeld("move_right")) moveX += 1;
        if(input.isActionHeld("move_up")) moveY -= 1;
        if(input.isActionHeld("move_down")) moveY += 1;
        
        player->setVelocity(moveX, moveY);
    }
};
```

### Menu System Integration
```cpp
class MenuSystem {
private:
    std::vector<std::string> menuItems;
    int selectedIndex = 0;
    
public:
    void initialize() {
        auto& input = InputManager::getInstance();
        
        input.registerKeyCallback([this](Key key, KeyState state) {
            if(state == KeyState::Pressed) {
                handleMenuInput(key);
            }
        });
    }
    
private:
    void handleMenuInput(Key key) {
        switch(key) {
            case Key::Up:
                selectedIndex = (selectedIndex - 1 + menuItems.size()) % menuItems.size();
                break;
                
            case Key::Down:
                selectedIndex = (selectedIndex + 1) % menuItems.size();
                break;
                
            case Key::Enter:
                activateMenuItem(selectedIndex);
                break;
                
            case Key::Escape:
                exitMenu();
                break;
        }
    }
};
```

## Performance Optimizations

### Memory Pool for Events
```cpp
class InputManager {
private:
    // Pre-allocated event pool
    static constexpr size_t EVENT_POOL_SIZE = 1000;
    std::array<InputEvent, EVENT_POOL_SIZE> eventPool;
    std::queue<size_t> freeEventIndices;
    std::queue<size_t> activeEventIndices;
    
public:
    void initialize() {
        // Initialize free event pool
        for(size_t i = 0; i < EVENT_POOL_SIZE; ++i) {
            freeEventIndices.push(i);
        }
    }
    
    void queueEvent(InputEvent::Type type, Key key) {
        if(freeEventIndices.empty()) {
            // Pool exhausted, could expand or drop event
            return;
        }
        
        size_t index = freeEventIndices.front();
        freeEventIndices.pop();
        
        eventPool[index] = {type, key, "", currentTime};
        activeEventIndices.push(index);
    }
    
    void processEvents() {
        while(!activeEventIndices.empty()) {
            size_t index = activeEventIndices.front();
            activeEventIndices.pop();
            
            const auto& event = eventPool[index];
            processEvent(event);
            
            // Return to free pool
            freeEventIndices.push(index);
        }
    }
};
```

### Cache-Friendly Data Layout
```cpp
class InputManager {
private:
    // Structure of Arrays for better cache performance
    struct KeyStateData {
        alignas(64) std::array<KeyState, 256> currentStates;
        alignas(64) std::array<KeyState, 256> previousStates;
        alignas(64) std::array<double, 256> pressTimes;
        alignas(64) std::array<double, 256> releaseTimes;
    };
    
    KeyStateData keyData;
    
public:
    void updateKeyStates() {
        // Process in chunks for better vectorization
        constexpr size_t CHUNK_SIZE = 64;
        
        for(size_t chunk = 0; chunk < 256; chunk += CHUNK_SIZE) {
            size_t end = std::min(chunk + CHUNK_SIZE, size_t(256));
            
            for(size_t i = chunk; i < end; ++i) {
                keyData.previousStates[i] = keyData.currentStates[i];
                
                // Update logic here
                if(keyData.currentStates[i] != keyData.previousStates[i]) {
                    if(keyData.currentStates[i] == KeyState::Pressed) {
                        keyData.pressTimes[i] = currentTime;
                    }
                }
            }
        }
    }
};
```

## Common Design Pitfalls

### Avoiding Common Mistakes
1. **Global State Pollution**: Don't store game-specific state in InputManager
2. **Platform Leakage**: Keep platform-specific code contained
3. **Event Flooding**: Limit event generation and processing
4. **Memory Leaks**: Properly manage observer lifetimes
5. **Thread Safety**: Protect shared state in multi-threaded environments

### Error Handling Strategies
```cpp
class InputManager {
private:
    bool isInitialized = false;
    std::string lastError;
    
public:
    bool initialize() {
        try {
            platformInput = createPlatformInput();
            if(!platformInput) {
                lastError = "Failed to create platform input";
                return false;
            }
            
            if(!platformInput->initialize()) {
                lastError = "Platform input initialization failed";
                return false;
            }
            
            isInitialized = true;
            return true;
        }
        catch(const std::exception& e) {
            lastError = "Exception during initialization: " + std::string(e.what());
            return false;
        }
    }
    
    const std::string& getLastError() const { return lastError; }
    
    bool isKeyPressed(Key key) const {
        if(!isInitialized) {
            // Log warning but don't crash
            return false;
        }
        
        size_t index = static_cast<size_t>(key);
        if(index >= keyStates.size()) {
            return false;
        }
        
        return keyStates[index] == KeyState::Pressed;
    }
};
```

## Resources

- [Design Patterns: Observer Pattern](https://refactoring.guru/design-patterns/observer/cpp)
- [C++ std::function Documentation](https://en.cppreference.com/w/cpp/utility/functional/function)
- [Thread Safety in C++](https://en.cppreference.com/w/cpp/thread)
- [STL Container Performance](https://github.com/danluu/post-mortems/blob/master/stl-performance.md)
- [Game Programming Patterns - Command Pattern](https://gameprogrammingpatterns.com/command.html)
- [Input Handling in Game Engines](https://www.gamedev.net/tutorials/programming/general-and-gameplay-programming/designing-an-input-handling-system-for-fast-paced-games-r2975/)

## Fun Challenges

- **Gesture Recognition**: Implement combo detection (like fighting game moves)
- **Macro System**: Record and playback input sequences
- **Accessibility Features**: Add key repeat, sticky keys, mouse emulation
- **Input Analytics**: Track player input patterns and statistics
- **Network Input**: Synchronize input across multiple clients
- **AI Input**: Create virtual players that generate input events
- **Mobile Touch**: Extend system to handle touch and gesture input
- **Hardware Integration**: Support game controllers and specialized input devices

## Debugging Tips

### Input State Visualization
```cpp
class InputDebugOverlay {
private:
    bool enabled = false;
    
public:
    void render(Renderer& renderer) {
        if(!enabled) return;
        
        auto& input = InputManager::getInstance();
        
        // Display currently pressed keys
        int y = 10;
        renderer.drawText(10, y, "Pressed Keys:");
        y += 20;
        
        for(int i = 0; i < static_cast<int>(Key::COUNT); ++i) {
            Key key = static_cast<Key>(i);
            if(input.isKeyHeld(key) || input.isKeyPressed(key)) {
                std::string keyName = getKeyName(key);
                std::string state = input.isKeyPressed(key) ? " (PRESSED)" : " (HELD)";
                renderer.drawText(20, y, keyName + state);
                y += 15;
            }
        }
        
        // Display active actions
        y += 10;
        renderer.drawText(10, y, "Active Actions:");
        y += 20;
        
        for(const auto& action : getAllActionNames()) {
            if(input.isActionActive(action)) {
                renderer.drawText(20, y, action);
                y += 15;
            }
        }
    }
    
    void toggle() { enabled = !enabled; }
};
```

### Input Event Logging
```cpp
class InputLogger {
private:
    std::ofstream logFile;
    bool enabled = false;
    
public:
    void initialize(const std::string& filename) {
        logFile.open(filename);
        enabled = logFile.is_open();
        
        if(enabled) {
            auto& input = InputManager::getInstance();
            input.registerKeyCallback([this](Key key, KeyState state) {
                logEvent(key, state);
            });
        }
    }
    
private:
    void logEvent(Key key, KeyState state) {
        if(!enabled) return;
        
        auto now = std::chrono::steady_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
        
        logFile << timestamp << ": " << getKeyName(key) << " " 
                << getStateName(state) << std::endl;
    }
};
```

## Refactoring Process

### Step 1: Extract Input Management
1. Create InputManager class with singleton pattern
2. Move platform input code from main loop
3. Add basic key state tracking
4. Test that input still works

### Step 2: Add Action Mapping
1. Create action mapping system
2. Define common game actions
3. Allow runtime key binding changes
4. Test action callbacks

### Step 3: Implement Event System
1. Add observer pattern for input events
2. Create event queue for frame-consistent processing
3. Add callback registration system
4. Test event dispatch

### Step 4: Add Configuration
1. Create InputConfig structure
2. Add file loading/saving
3. Allow runtime configuration changes
4. Test configuration persistence

### Step 5: Optimize Performance
1. Profile input processing performance
2. Add memory pooling for events
3. Optimize container access patterns
4. Add thread safety if needed

### Step 6: Add Advanced Features
1. Implement input buffering
2. Add gesture/combo recognition
3. Create debugging tools
4. Add accessibility features

## Integration with Engine
```cpp
// In Engine class
class Engine {
private:
    std::unique_ptr<InputManager> inputManager;
    
public:
    bool initialize() {
        // Initialize input system
        inputManager = std::make_unique<InputManager>();
        if(!inputManager->initialize()) {
            return false;
        }
        
        // Setup engine-level input bindings
        setupEngineInputBindings();
        
        return true;
    }
    
    void update(double deltaTime) {
        // Update input first each frame
        inputManager->update(deltaTime);
        
        // Handle engine-level input
        handleEngineInput();
        
        // Update other systems...
    }
    
private:
    void setupEngineInputBindings() {
        inputManager->createAction("quit");
        inputManager->createAction("pause");
        inputManager->createAction("debug_toggle");
        
        inputManager->bindKeyToAction("quit", Key::Escape);
        inputManager->bindKeyToAction("pause", Key::P);
        inputManager->bindKeyToAction("debug_toggle", Key::F3);
        
        inputManager->registerActionCallback("quit", [this](const std::string&, bool pressed) {
            if(pressed) requestShutdown();
        });
        
        inputManager->registerActionCallback("pause", [this](const std::string&, bool pressed) {
            if(pressed) togglePause();
        });
    }
    
    void handleEngineInput() {
        // Engine can also poll for input directly
        if(inputManager->isKeyPressed(Key::F1)) {
            toggleDebugOverlay();
        }
    }
};
```

This InputManager design provides a solid foundation for handling input in your console game engine while teaching essential C++ concepts like singleton pattern, event systems, callbacks, and STL container usage. The system is designed to be extensible, performant, and maintainable.