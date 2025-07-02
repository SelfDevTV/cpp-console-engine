# Phase 5.2: GameStateManager Design

## C++ Learning Focus
- **State pattern**: Implementing the classic Gang of Four state design pattern
- **Polymorphism**: Virtual functions and interface-based design
- **Stack-based management**: Using STL stack for state hierarchy
- **Smart pointers**: Managing state objects with `std::unique_ptr` and `std::shared_ptr`
- **Template patterns**: Generic state management containers
- **Object lifecycle**: Constructor/destructor patterns for state transitions
- **Abstract base classes**: Pure virtual interfaces for state behavior

## Implementation Overview
Design a GameStateManager to handle game state transitions and management:
1. Implement a stack-based state system for hierarchical states
2. Create polymorphic state interfaces with virtual methods
3. Handle state transitions with validation and cleanup
4. Manage state lifecycle with proper resource management
5. Enable inter-state communication and data sharing
6. Provide state persistence and restoration capabilities
7. Support pause/resume functionality for nested states

## State Machine Design Concepts

### What is a Game State?
A game state represents a distinct mode or screen in your game:
- **Menu State**: Main menu, settings, options
- **Gameplay State**: Active game simulation
- **Pause State**: Game paused overlay
- **Loading State**: Asset loading screen
- **Dialog State**: Conversation or cutscene
- **Inventory State**: Item management screen

### State Stack vs State Machine
```cpp
// Traditional State Machine (single active state)
enum class GameStateType {
    MENU, GAMEPLAY, PAUSE, SETTINGS
};

// Stack-based State System (multiple active states)
// Stack: [Menu] -> [Gameplay] -> [Pause]
// All states can be active, but only top state receives input
```

## Class Structure Design

### Abstract GameState Interface
```cpp
class GameState {
public:
    virtual ~GameState() = default;
    
    // Core lifecycle methods
    virtual bool initialize() = 0;
    virtual void cleanup() = 0;
    
    // State updates
    virtual void update(double deltaTime) = 0;
    virtual void render(Renderer& renderer) = 0;
    
    // State transitions
    virtual void pause() = 0;
    virtual void resume() = 0;
    
    // Input handling
    virtual bool handleInput(const InputEvent& event) = 0;
    
    // State identification
    virtual std::string getStateName() const = 0;
    virtual StateType getStateType() const = 0;
    
    // State properties
    virtual bool isPauseable() const { return true; }
    virtual bool isTransparent() const { return false; }
    virtual bool blocksInput() const { return true; }
    
protected:
    bool isActive = true;
    bool isPaused = false;
    GameStateManager* stateManager = nullptr;
    
    friend class GameStateManager;
};
```

### GameStateManager Class
```cpp
class GameStateManager {
public:
    GameStateManager();
    ~GameStateManager();
    
    bool initialize();
    void shutdown();
    void update(double deltaTime);
    void render(Renderer& renderer);
    
    // State management
    void pushState(std::unique_ptr<GameState> state);
    void popState();
    void changeState(std::unique_ptr<GameState> state);
    void clearAllStates();
    
    // State queries
    GameState* getCurrentState() const;
    GameState* getPreviousState() const;
    size_t getStateCount() const;
    bool hasState(StateType type) const;
    
    // Input handling
    bool handleInput(const InputEvent& event);
    
    // State communication
    void sendMessage(const StateMessage& message);
    void broadcastMessage(const StateMessage& message);
    
    // State validation
    bool canPushState(StateType type) const;
    bool canPopState() const;
    
private:
    std::stack<std::unique_ptr<GameState>> stateStack;
    std::vector<StateMessage> messageQueue;
    bool isTransitioning = false;
    
    void processMessages();
    void validateStateStack();
    void notifyStateChange();
};
```

### State Types and Identification
```cpp
enum class StateType {
    MAIN_MENU,
    GAMEPLAY,
    PAUSE_MENU,
    SETTINGS,
    INVENTORY,
    DIALOG,
    LOADING,
    GAME_OVER,
    CREDITS
};

struct StateMessage {
    std::string type;
    std::string sender;
    std::string recipient;
    std::any data;
    double timestamp;
    
    template<typename T>
    StateMessage(const std::string& msgType, const std::string& from, 
                const std::string& to, const T& messageData)
        : type(msgType), sender(from), recipient(to), 
          data(messageData), timestamp(getCurrentTime()) {}
};
```

## Key C++ Concepts to Learn

### Virtual Inheritance and Polymorphism
```cpp
// Base state with virtual methods
class GameState {
public:
    virtual ~GameState() = default;  // Virtual destructor
    virtual void update(double deltaTime) = 0;  // Pure virtual
    virtual void render(Renderer& renderer) = 0;
    
    // Non-pure virtual with default implementation
    virtual bool handleInput(const InputEvent& event) {
        return false;  // Default: don't handle input
    }
};

// Derived state implementation
class MenuState : public GameState {
public:
    void update(double deltaTime) override {
        // Menu-specific update logic
        animationTimer += deltaTime;
        updateMenuAnimations();
    }
    
    void render(Renderer& renderer) override {
        // Menu-specific rendering
        renderBackground(renderer);
        renderMenuItems(renderer);
    }
    
private:
    double animationTimer = 0.0;
    void updateMenuAnimations();
    void renderMenuItems(Renderer& renderer);
};
```

### Smart Pointer Management
```cpp
class GameStateManager {
private:
    // Stack owns the states
    std::stack<std::unique_ptr<GameState>> stateStack;
    
public:
    void pushState(std::unique_ptr<GameState> state) {
        if (state) {
            // Setup state
            state->stateManager = this;
            
            // Pause current state if exists
            if (!stateStack.empty()) {
                stateStack.top()->pause();
            }
            
            // Initialize and push new state
            if (state->initialize()) {
                stateStack.push(std::move(state));
            }
        }
    }
    
    void popState() {
        if (!stateStack.empty()) {
            // Cleanup current state
            stateStack.top()->cleanup();
            stateStack.pop();
            
            // Resume previous state
            if (!stateStack.empty()) {
                stateStack.top()->resume();
            }
        }
    }
};
```

### Stack Container Usage
```cpp
// STL stack for state management
#include <stack>
#include <memory>

class GameStateManager {
private:
    std::stack<std::unique_ptr<GameState>> stateStack;
    
    // Helper methods for stack operations
    bool isStackEmpty() const {
        return stateStack.empty();
    }
    
    GameState* getTopState() const {
        return stateStack.empty() ? nullptr : stateStack.top().get();
    }
    
    void clearStack() {
        while (!stateStack.empty()) {
            stateStack.top()->cleanup();
            stateStack.pop();
        }
    }
};
```

## State Pattern Implementation

### Classic State Pattern Design
```cpp
// Context class (Game or specific object)
class Player {
private:
    std::unique_ptr<PlayerState> currentState;
    
public:
    void setState(std::unique_ptr<PlayerState> newState) {
        if (currentState) {
            currentState->exit(this);
        }
        currentState = std::move(newState);
        if (currentState) {
            currentState->enter(this);
        }
    }
    
    void update(double deltaTime) {
        if (currentState) {
            currentState->update(this, deltaTime);
        }
    }
    
    void handleInput(const InputEvent& event) {
        if (currentState) {
            currentState->handleInput(this, event);
        }
    }
};

// State interface
class PlayerState {
public:
    virtual ~PlayerState() = default;
    virtual void enter(Player* player) = 0;
    virtual void exit(Player* player) = 0;
    virtual void update(Player* player, double deltaTime) = 0;
    virtual void handleInput(Player* player, const InputEvent& event) = 0;
};

// Concrete states
class IdleState : public PlayerState {
public:
    void enter(Player* player) override {
        // Setup idle animation
    }
    
    void handleInput(Player* player, const InputEvent& event) override {
        if (event.key == KEY_SPACE) {
            player->setState(std::make_unique<JumpingState>());
        }
    }
};
```

### State Factory Pattern
```cpp
class StateFactory {
public:
    static std::unique_ptr<GameState> createState(StateType type) {
        switch (type) {
            case StateType::MAIN_MENU:
                return std::make_unique<MainMenuState>();
            case StateType::GAMEPLAY:
                return std::make_unique<GameplayState>();
            case StateType::PAUSE_MENU:
                return std::make_unique<PauseMenuState>();
            case StateType::SETTINGS:
                return std::make_unique<SettingsState>();
            default:
                return nullptr;
        }
    }
    
    template<typename StateClass, typename... Args>
    static std::unique_ptr<GameState> createState(Args&&... args) {
        return std::make_unique<StateClass>(std::forward<Args>(args)...);
    }
};
```

## State Lifecycle Management

### State Initialization and Cleanup
```cpp
class GameplayState : public GameState {
public:
    bool initialize() override {
        try {
            // Load game assets
            if (!loadGameAssets()) {
                return false;
            }
            
            // Initialize game world
            world = std::make_unique<GameWorld>();
            if (!world->initialize()) {
                return false;
            }
            
            // Setup input handlers
            setupInputHandlers();
            
            isInitialized = true;
            return true;
        }
        catch (const std::exception& e) {
            cleanup();
            return false;
        }
    }
    
    void cleanup() override {
        // Cleanup in reverse order of initialization
        if (world) {
            world->shutdown();
            world.reset();
        }
        
        unloadGameAssets();
        isInitialized = false;
    }
    
private:
    std::unique_ptr<GameWorld> world;
    bool isInitialized = false;
    
    bool loadGameAssets();
    void unloadGameAssets();
    void setupInputHandlers();
};
```

### Pause and Resume Handling
```cpp
class GameState {
public:
    virtual void pause() {
        if (isActive && !isPaused) {
            onPause();
            isPaused = true;
        }
    }
    
    virtual void resume() {
        if (isActive && isPaused) {
            onResume();
            isPaused = false;
        }
    }
    
protected:
    virtual void onPause() {}
    virtual void onResume() {}
    
    bool isActive = true;
    bool isPaused = false;
};

class GameplayState : public GameState {
protected:
    void onPause() override {
        // Pause game simulation
        gameWorld->pause();
        
        // Save game state for quick resume
        saveCurrentState();
        
        // Stop audio/animations
        audioManager->pauseAll();
    }
    
    void onResume() override {
        // Resume game simulation
        gameWorld->resume();
        
        // Resume audio/animations
        audioManager->resumeAll();
        
        // Reset input state to prevent stuck keys
        inputManager->clearInputState();
    }
};
```

## State Transition Validation

### Transition Rules and Validation
```cpp
class StateTransitionValidator {
public:
    struct TransitionRule {
        StateType fromState;
        StateType toState;
        bool isAllowed;
        std::string reason;
    };
    
    bool canTransition(StateType from, StateType to) const {
        auto it = transitionRules.find({from, to});
        return it != transitionRules.end() && it->second.isAllowed;
    }
    
    std::string getTransitionReason(StateType from, StateType to) const {
        auto it = transitionRules.find({from, to});
        return it != transitionRules.end() ? it->second.reason : "Unknown transition";
    }
    
private:
    std::map<std::pair<StateType, StateType>, TransitionRule> transitionRules;
    
    void initializeRules() {
        // Define allowed transitions
        addRule(StateType::MAIN_MENU, StateType::GAMEPLAY, true, "Start game");
        addRule(StateType::GAMEPLAY, StateType::PAUSE_MENU, true, "Pause game");
        addRule(StateType::PAUSE_MENU, StateType::GAMEPLAY, true, "Resume game");
        addRule(StateType::PAUSE_MENU, StateType::MAIN_MENU, true, "Quit to menu");
        
        // Prevent invalid transitions
        addRule(StateType::LOADING, StateType::PAUSE_MENU, false, "Cannot pause during loading");
    }
};
```

### Safe State Transitions
```cpp
class GameStateManager {
public:
    bool requestStateChange(StateType newStateType) {
        auto currentStateType = getCurrentState() ? 
            getCurrentState()->getStateType() : StateType::MAIN_MENU;
        
        if (!validator.canTransition(currentStateType, newStateType)) {
            std::string reason = validator.getTransitionReason(currentStateType, newStateType);
            logWarning("Invalid state transition: " + reason);
            return false;
        }
        
        auto newState = StateFactory::createState(newStateType);
        if (!newState) {
            logError("Failed to create state of type: " + toString(newStateType));
            return false;
        }
        
        changeState(std::move(newState));
        return true;
    }
    
private:
    StateTransitionValidator validator;
    
    void changeState(std::unique_ptr<GameState> newState) {
        // Mark as transitioning to prevent concurrent changes
        isTransitioning = true;
        
        // Cleanup current state
        if (!stateStack.empty()) {
            stateStack.top()->cleanup();
            stateStack.pop();
        }
        
        // Initialize new state
        if (newState && newState->initialize()) {
            stateStack.push(std::move(newState));
        }
        
        isTransitioning = false;
        notifyStateChange();
    }
};
```

## Stack-Based State System

### Hierarchical State Management
```cpp
class GameStateManager {
public:
    void pushState(std::unique_ptr<GameState> state) {
        if (!state || isTransitioning) {
            return;
        }
        
        isTransitioning = true;
        
        // Pause current state (but keep it active for rendering)
        if (!stateStack.empty() && stateStack.top()->isPauseable()) {
            stateStack.top()->pause();
        }
        
        // Initialize and push new state
        if (state->initialize()) {
            state->stateManager = this;
            stateStack.push(std::move(state));
        }
        
        isTransitioning = false;
    }
    
    void render(Renderer& renderer) {
        // Render states from bottom to top
        std::vector<GameState*> statesToRender;
        
        // Collect transparent states for rendering
        auto tempStack = stateStack;
        while (!tempStack.empty()) {
            GameState* state = tempStack.top().get();
            statesToRender.push_back(state);
            tempStack.pop();
            
            // Stop if this state blocks rendering below it
            if (!state->isTransparent()) {
                break;
            }
        }
        
        // Render from bottom to top
        for (auto it = statesToRender.rbegin(); it != statesToRender.rend(); ++it) {
            (*it)->render(renderer);
        }
    }
    
    bool handleInput(const InputEvent& event) {
        // Only top state receives input by default
        if (!stateStack.empty()) {
            GameState* topState = stateStack.top().get();
            if (!topState->isPaused && topState->blockInput()) {
                return topState->handleInput(event);
            }
        }
        return false;
    }
};
```

### State Stack Utilities
```cpp
class GameStateManager {
public:
    // Find state by type in stack
    GameState* findState(StateType type) const {
        auto tempStack = stateStack;
        while (!tempStack.empty()) {
            if (tempStack.top()->getStateType() == type) {
                return tempStack.top().get();
            }
            tempStack.pop();
        }
        return nullptr;
    }
    
    // Remove specific state from stack
    bool removeState(StateType type) {
        std::stack<std::unique_ptr<GameState>> tempStack;
        bool found = false;
        
        // Move states to temp stack until we find the target
        while (!stateStack.empty()) {
            if (stateStack.top()->getStateType() == type && !found) {
                stateStack.top()->cleanup();
                stateStack.pop();
                found = true;
            } else {
                tempStack.push(std::move(const_cast<std::unique_ptr<GameState>&>(stateStack.top())));
                stateStack.pop();
            }
        }
        
        // Restore remaining states
        while (!tempStack.empty()) {
            stateStack.push(std::move(const_cast<std::unique_ptr<GameState>&>(tempStack.top())));
            tempStack.pop();
        }
        
        return found;
    }
    
    // Get state stack depth
    size_t getStackDepth() const {
        return stateStack.size();
    }
};
```

## State Communication Patterns

### Message-Based Communication
```cpp
struct StateMessage {
    std::string type;
    std::string sender;
    std::string recipient;  // Empty for broadcast
    std::any data;
    double timestamp;
    bool handled = false;
    
    template<typename T>
    T getData() const {
        return std::any_cast<T>(data);
    }
    
    bool isType(const std::string& msgType) const {
        return type == msgType;
    }
};

class GameState {
public:
    virtual void receiveMessage(const StateMessage& message) {
        // Default: ignore message
    }
    
protected:
    void sendMessage(const std::string& type, const std::string& recipient, 
                    const std::any& data) {
        if (stateManager) {
            StateMessage msg{type, getStateName(), recipient, data, getCurrentTime()};
            stateManager->sendMessage(msg);
        }
    }
    
    void broadcastMessage(const std::string& type, const std::any& data) {
        sendMessage(type, "", data);  // Empty recipient = broadcast
    }
};

class GameStateManager {
private:
    std::queue<StateMessage> messageQueue;
    
    void processMessages() {
        while (!messageQueue.empty()) {
            StateMessage& msg = messageQueue.front();
            
            if (msg.recipient.empty()) {
                // Broadcast to all states
                auto tempStack = stateStack;
                while (!tempStack.empty()) {
                    tempStack.top()->receiveMessage(msg);
                    tempStack.pop();
                }
            } else {
                // Send to specific recipient
                GameState* recipient = findStateByName(msg.recipient);
                if (recipient) {
                    recipient->receiveMessage(msg);
                    msg.handled = true;
                }
            }
            
            messageQueue.pop();
        }
    }
};
```

### Shared Data Context
```cpp
class GameContext {
public:
    // Game data accessible to all states
    PlayerData playerData;
    GameSettings settings;
    InventoryData inventory;
    SaveGameData currentSave;
    
    // Shared resources
    std::shared_ptr<AssetManager> assetManager;
    std::shared_ptr<AudioManager> audioManager;
    std::shared_ptr<InputManager> inputManager;
    
    // State persistence
    void saveState(const std::string& key, const std::any& value) {
        persistentData[key] = value;
    }
    
    template<typename T>
    T loadState(const std::string& key, const T& defaultValue = T{}) const {
        auto it = persistentData.find(key);
        if (it != persistentData.end()) {
            try {
                return std::any_cast<T>(it->second);
            } catch (const std::bad_any_cast&) {
                return defaultValue;
            }
        }
        return defaultValue;
    }
    
private:
    std::map<std::string, std::any> persistentData;
};

class GameState {
protected:
    GameContext* gameContext = nullptr;
    
public:
    void setGameContext(GameContext* context) {
        gameContext = context;
    }
};
```

## State Persistence

### Save and Restore State
```cpp
class PersistentGameState : public GameState {
public:
    virtual void saveState(SaveData& saveData) const {
        // Default implementation - override in derived classes
    }
    
    virtual bool loadState(const SaveData& saveData) {
        // Default implementation - override in derived classes
        return true;
    }
    
    virtual std::string getStateId() const = 0;
};

class GameplayState : public PersistentGameState {
public:
    void saveState(SaveData& saveData) const override {
        saveData.setValue("gameplay.playerPosition", player->getPosition());
        saveData.setValue("gameplay.playerHealth", player->getHealth());
        saveData.setValue("gameplay.currentLevel", currentLevel);
        saveData.setValue("gameplay.gameTime", gameTime);
        
        // Save world state
        world->saveState(saveData);
    }
    
    bool loadState(const SaveData& saveData) override {
        try {
            auto playerPos = saveData.getValue<Vector2>("gameplay.playerPosition");
            auto playerHealth = saveData.getValue<int>("gameplay.playerHealth");
            auto level = saveData.getValue<int>("gameplay.currentLevel");
            auto time = saveData.getValue<double>("gameplay.gameTime");
            
            player->setPosition(playerPos);
            player->setHealth(playerHealth);
            currentLevel = level;
            gameTime = time;
            
            return world->loadState(saveData);
        }
        catch (const std::exception& e) {
            logError("Failed to load gameplay state: " + std::string(e.what()));
            return false;
        }
    }
    
    std::string getStateId() const override {
        return "gameplay";
    }
};
```

### Quick Save/Load System
```cpp
class GameStateManager {
public:
    bool quickSave() {
        SaveData saveData;
        
        // Save current state stack
        saveData.setValue("stateManager.stackSize", stateStack.size());
        
        std::stack<std::unique_ptr<GameState>> tempStack = stateStack;
        int index = 0;
        
        while (!tempStack.empty()) {
            auto* persistentState = dynamic_cast<PersistentGameState*>(tempStack.top().get());
            if (persistentState) {
                std::string key = "state." + std::to_string(index);
                saveData.setValue(key + ".type", persistentState->getStateType());
                saveData.setValue(key + ".id", persistentState->getStateId());
                
                persistentState->saveState(saveData);
            }
            tempStack.pop();
            index++;
        }
        
        return saveData.writeToFile("quicksave.dat");
    }
    
    bool quickLoad() {
        SaveData saveData;
        if (!saveData.readFromFile("quicksave.dat")) {
            return false;
        }
        
        // Clear current states
        clearAllStates();
        
        // Restore state stack
        size_t stackSize = saveData.getValue<size_t>("stateManager.stackSize");
        
        for (int i = stackSize - 1; i >= 0; --i) {
            std::string key = "state." + std::to_string(i);
            auto stateType = saveData.getValue<StateType>(key + ".type");
            
            auto state = StateFactory::createState(stateType);
            if (state) {
                auto* persistentState = dynamic_cast<PersistentGameState*>(state.get());
                if (persistentState && persistentState->loadState(saveData)) {
                    pushState(std::move(state));
                }
            }
        }
        
        return !stateStack.empty();
    }
};
```

## Testing Strategies

### Unit Testing States
```cpp
#include <gtest/gtest.h>

class MockRenderer : public Renderer {
public:
    MOCK_METHOD(void, clear, (), (override));
    MOCK_METHOD(void, drawText, (int x, int y, const std::string& text), (override));
    MOCK_METHOD(void, present, (), (override));
};

class GameStateTest : public ::testing::Test {
protected:
    void SetUp() override {
        stateManager = std::make_unique<GameStateManager>();
        stateManager->initialize();
        
        mockRenderer = std::make_unique<MockRenderer>();
        gameContext = std::make_unique<GameContext>();
    }
    
    void TearDown() override {
        stateManager->shutdown();
    }
    
    std::unique_ptr<GameStateManager> stateManager;
    std::unique_ptr<MockRenderer> mockRenderer;
    std::unique_ptr<GameContext> gameContext;
};

TEST_F(GameStateTest, PushStateInitializesCorrectly) {
    auto menuState = std::make_unique<MainMenuState>();
    menuState->setGameContext(gameContext.get());
    
    EXPECT_EQ(stateManager->getStateCount(), 0);
    
    stateManager->pushState(std::move(menuState));
    
    EXPECT_EQ(stateManager->getStateCount(), 1);
    EXPECT_NE(stateManager->getCurrentState(), nullptr);
    EXPECT_EQ(stateManager->getCurrentState()->getStateType(), StateType::MAIN_MENU);
}

TEST_F(GameStateTest, StateStackManagement) {
    // Push menu state
    auto menuState = std::make_unique<MainMenuState>();
    stateManager->pushState(std::move(menuState));
    
    // Push gameplay state
    auto gameplayState = std::make_unique<GameplayState>();
    stateManager->pushState(std::move(gameplayState));
    
    EXPECT_EQ(stateManager->getStateCount(), 2);
    EXPECT_EQ(stateManager->getCurrentState()->getStateType(), StateType::GAMEPLAY);
    
    // Pop gameplay state - should return to menu
    stateManager->popState();
    
    EXPECT_EQ(stateManager->getStateCount(), 1);
    EXPECT_EQ(stateManager->getCurrentState()->getStateType(), StateType::MAIN_MENU);
}
```

### Integration Testing
```cpp
class GameStateIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        engine = std::make_unique<GameEngine>();
        engine->initialize();
        
        stateManager = engine->getStateManager();
    }
    
    std::unique_ptr<GameEngine> engine;
    GameStateManager* stateManager;
};

TEST_F(GameStateIntegrationTest, CompleteGameFlow) {
    // Start at main menu
    auto menuState = std::make_unique<MainMenuState>();
    stateManager->pushState(std::move(menuState));
    
    // Simulate menu interaction
    InputEvent startGameEvent{InputType::KEY_PRESS, KEY_ENTER};
    stateManager->handleInput(startGameEvent);
    
    // Should transition to gameplay
    engine->update(0.016);  // One frame
    
    EXPECT_EQ(stateManager->getCurrentState()->getStateType(), StateType::GAMEPLAY);
    
    // Simulate pause
    InputEvent pauseEvent{InputType::KEY_PRESS, KEY_ESCAPE};
    stateManager->handleInput(pauseEvent);
    
    engine->update(0.016);
    
    EXPECT_EQ(stateManager->getStateCount(), 2);
    EXPECT_EQ(stateManager->getCurrentState()->getStateType(), StateType::PAUSE_MENU);
}
```

## Integration Examples

### Basic Implementation Example
```cpp
// main.cpp
int main() {
    GameEngine engine;
    if (!engine.initialize()) {
        return -1;
    }
    
    // Create initial state
    auto menuState = std::make_unique<MainMenuState>();
    engine.getStateManager()->pushState(std::move(menuState));
    
    // Main game loop
    while (engine.isRunning()) {
        engine.update();
        engine.render();
    }
    
    engine.shutdown();
    return 0;
}

// Example MenuState implementation
class MainMenuState : public GameState {
private:
    std::vector<std::string> menuItems = {"Start Game", "Settings", "Exit"};
    int selectedItem = 0;
    
public:
    bool initialize() override {
        // Load menu resources
        return true;
    }
    
    void update(double deltaTime) override {
        // Update menu animations
    }
    
    void render(Renderer& renderer) override {
        renderer.clear();
        renderer.drawText(10, 5, "=== MAIN MENU ===");
        
        for (size_t i = 0; i < menuItems.size(); ++i) {
            std::string prefix = (i == selectedItem) ? "> " : "  ";
            renderer.drawText(10, 8 + i * 2, prefix + menuItems[i]);
        }
        
        renderer.present();
    }
    
    bool handleInput(const InputEvent& event) override {
        if (event.type == InputType::KEY_PRESS) {
            switch (event.key) {
                case KEY_UP:
                    selectedItem = (selectedItem - 1 + menuItems.size()) % menuItems.size();
                    return true;
                case KEY_DOWN:
                    selectedItem = (selectedItem + 1) % menuItems.size();
                    return true;
                case KEY_ENTER:
                    handleMenuSelection();
                    return true;
            }
        }
        return false;
    }
    
private:
    void handleMenuSelection() {
        switch (selectedItem) {
            case 0: // Start Game
                stateManager->pushState(std::make_unique<GameplayState>());
                break;
            case 1: // Settings
                stateManager->pushState(std::make_unique<SettingsState>());
                break;
            case 2: // Exit
                // Send exit message to engine
                broadcastMessage("engine.exit", true);
                break;
        }
    }
};
```

### Advanced State Communication
```cpp
class InventoryState : public GameState {
public:
    void receiveMessage(const StateMessage& message) override {
        if (message.isType("inventory.addItem")) {
            auto item = message.getData<Item>();
            addItemToInventory(item);
        } else if (message.isType("inventory.removeItem")) {
            auto itemId = message.getData<int>();
            removeItemFromInventory(itemId);
        }
    }
    
    bool handleInput(const InputEvent& event) override {
        if (event.type == InputType::KEY_PRESS && event.key == KEY_ESCAPE) {
            // Close inventory and return to gameplay
            stateManager->popState();
            return true;
        }
        
        if (event.key == KEY_ENTER && selectedItem >= 0) {
            // Use selected item
            useItem(inventory[selectedItem]);
            
            // Notify gameplay state about item usage
            sendMessage("gameplay.itemUsed", "gameplay", inventory[selectedItem]);
            
            return true;
        }
        
        return false;
    }
    
private:
    std::vector<Item> inventory;
    int selectedItem = -1;
    
    void addItemToInventory(const Item& item);
    void removeItemFromInventory(int itemId);
    void useItem(const Item& item);
};
```

## Common Implementation Patterns

### State Registration System
```cpp
class StateRegistry {
public:
    template<typename StateClass>
    void registerState(StateType type) {
        stateCreators[type] = []() -> std::unique_ptr<GameState> {
            return std::make_unique<StateClass>();
        };
    }
    
    std::unique_ptr<GameState> createState(StateType type) const {
        auto it = stateCreators.find(type);
        return it != stateCreators.end() ? it->second() : nullptr;
    }
    
private:
    std::map<StateType, std::function<std::unique_ptr<GameState>()>> stateCreators;
};

// Usage in engine initialization
void GameEngine::registerStates() {
    stateRegistry.registerState<MainMenuState>(StateType::MAIN_MENU);
    stateRegistry.registerState<GameplayState>(StateType::GAMEPLAY);
    stateRegistry.registerState<PauseMenuState>(StateType::PAUSE_MENU);
    stateRegistry.registerState<SettingsState>(StateType::SETTINGS);
}
```

### Async State Loading
```cpp
class LoadingState : public GameState {
public:
    LoadingState(StateType nextStateType) : nextState(nextStateType) {}
    
    bool initialize() override {
        loadingProgress = 0.0f;
        
        // Start async loading
        loadingFuture = std::async(std::launch::async, [this]() {
            return loadNextStateAsync();
        });
        
        return true;
    }
    
    void update(double deltaTime) override {
        // Update loading progress
        if (loadingFuture.valid()) {
            if (loadingFuture.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready) {
                // Loading complete
                auto loadedState = loadingFuture.get();
                if (loadedState) {
                    stateManager->changeState(std::move(loadedState));
                }
            }
        }
    }
    
    void render(Renderer& renderer) override {
        renderer.clear();
        renderer.drawText(10, 10, "Loading...");
        
        // Draw progress bar
        int barWidth = 50;
        int filledWidth = static_cast<int>(loadingProgress * barWidth);
        
        std::string progressBar = "[" + std::string(filledWidth, '=') + 
                                 std::string(barWidth - filledWidth, ' ') + "]";
        renderer.drawText(10, 12, progressBar);
        
        renderer.present();
    }
    
private:
    StateType nextState;
    std::future<std::unique_ptr<GameState>> loadingFuture;
    float loadingProgress = 0.0f;
    
    std::unique_ptr<GameState> loadNextStateAsync() {
        // Simulate resource loading with progress updates
        for (int i = 0; i <= 100; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            loadingProgress = i / 100.0f;
        }
        
        return StateFactory::createState(nextState);
    }
};
```

## Resources and Further Learning

### Essential C++ Concepts to Master
1. **Virtual functions and polymorphism**: Understanding vtables and dynamic dispatch
2. **Smart pointers**: `std::unique_ptr`, `std::shared_ptr`, and `std::weak_ptr`
3. **STL containers**: `std::stack`, `std::queue`, `std::vector`, `std::map`
4. **Design patterns**: State pattern, Factory pattern, Observer pattern
5. **RAII principles**: Resource management and exception safety
6. **Template programming**: Generic state management and type safety

### Recommended Reading
- "Design Patterns" by Gang of Four (State pattern chapter)
- "Effective C++" by Scott Meyers (Smart pointers and RAII)
- "Game Programming Patterns" by Robert Nystrom (State pattern in games)
- "C++ Primer" by Lippman, Lajoie, and Moo (STL containers and templates)

### Online Resources
- **CppReference**: https://en.cppreference.com/ (STL documentation)
- **GameDev.net**: State management articles and tutorials
- **Stack Overflow**: Common state management problems and solutions
- **GitHub**: Open source game engines for reference implementations

## Fun Challenges

### Beginner Challenges
1. **Simple Menu System**: Create a basic menu with navigation and selection
2. **Pause/Resume**: Implement pause functionality that works from any state
3. **State Transition Animation**: Add fade effects between state changes
4. **Save/Load States**: Implement quick save and load for your game states

### Intermediate Challenges
1. **Nested State Machines**: Implement states within states (e.g., player states within gameplay state)
2. **State History System**: Track state history and implement "back" functionality
3. **Conditional Transitions**: Create complex state transition rules and validation
4. **State Performance Metrics**: Add timing and performance monitoring to states

### Advanced Challenges
1. **Threaded State Loading**: Implement background state loading with progress reporting
2. **State Serialization**: Full save/load system with versioning and migration
3. **Network State Sync**: Synchronize states across network for multiplayer
4. **State Pattern Templates**: Create generic state machines for different game objects

## Debugging Tips

### Common State Management Issues
```cpp
// Debug logging for state transitions
class DebugGameStateManager : public GameStateManager {
public:
    void pushState(std::unique_ptr<GameState> state) override {
        if (state) {
            logDebug("Pushing state: " + state->getStateName() + 
                    " (Stack size: " + std::to_string(getStateCount()) + ")");
        }
        GameStateManager::pushState(std::move(state));
    }
    
    void popState() override {
        if (!stateStack.empty()) {
            logDebug("Popping state: " + getCurrentState()->getStateName() + 
                    " (Stack size: " + std::to_string(getStateCount()) + ")");
        }
        GameStateManager::popState();
    }
};

// State validation helper
void validateStateStack() {
    if (stateStack.empty()) {
        logWarning("State stack is empty!");
        return;
    }
    
    // Check for circular references
    std::set<std::string> stateNames;
    auto tempStack = stateStack;
    
    while (!tempStack.empty()) {
        std::string name = tempStack.top()->getStateName();
        if (stateNames.find(name) != stateNames.end()) {
            logError("Circular state reference detected: " + name);
        }
        stateNames.insert(name);
        tempStack.pop();
    }
}
```

### Memory Leak Detection
```cpp
class StateMemoryTracker {
public:
    static void trackState(const std::string& stateName) {
        std::lock_guard<std::mutex> lock(mutex);
        activeStates[stateName]++;
        logDebug("State created: " + stateName + " (Count: " + 
                std::to_string(activeStates[stateName]) + ")");
    }
    
    static void untrackState(const std::string& stateName) {
        std::lock_guard<std::mutex> lock(mutex);
        if (activeStates[stateName] > 0) {
            activeStates[stateName]--;
            logDebug("State destroyed: " + stateName + " (Count: " + 
                    std::to_string(activeStates[stateName]) + ")");
        }
    }
    
    static void printReport() {
        std::lock_guard<std::mutex> lock(mutex);
        logInfo("=== State Memory Report ===");
        for (const auto& pair : activeStates) {
            if (pair.second > 0) {
                logWarning("Potential leak - " + pair.first + ": " + 
                          std::to_string(pair.second) + " instances");
            }
        }
    }
    
private:
    static std::map<std::string, int> activeStates;
    static std::mutex mutex;
};

// Usage in state constructors/destructors
GameState::GameState() {
    StateMemoryTracker::trackState(getStateName());
}

GameState::~GameState() {
    StateMemoryTracker::untrackState(getStateName());
}
```

## Refactoring and Evolution

### From Simple to Complex States
```cpp
// Phase 1: Simple state enum
enum class GameState { MENU, PLAYING, PAUSED };
GameState currentState = GameState::MENU;

// Phase 2: State classes with polymorphism
class GameState {
public:
    virtual void update() = 0;
    virtual void render() = 0;
};

// Phase 3: Full state manager with stack
class GameStateManager {
    std::stack<std::unique_ptr<GameState>> stateStack;
public:
    void pushState(std::unique_ptr<GameState> state);
    void popState();
};

// Phase 4: Advanced features (messaging, persistence, etc.)
class AdvancedGameStateManager : public GameStateManager {
    // Message system, save/load, transition validation, etc.
};
```

### Refactoring Guidelines
1. **Start Simple**: Begin with basic state switching, add complexity gradually
2. **Extract Interfaces**: Move common functionality to base classes
3. **Add Validation**: Implement transition rules and error checking
4. **Optimize Performance**: Profile state updates and transitions
5. **Improve Debugging**: Add logging, metrics, and debugging tools
6. **Document Patterns**: Keep track of your state design decisions

This comprehensive guide covers the essential aspects of implementing a GameStateManager for your console game engine. The state pattern is fundamental to game development, and mastering these concepts will greatly improve your C++ programming skills and game architecture understanding.