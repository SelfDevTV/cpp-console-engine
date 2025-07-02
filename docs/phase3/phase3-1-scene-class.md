# Phase 3.1: Scene Class Design

## C++ Learning Focus
- **Container management**: `std::vector`, `std::list`, `std::map` performance characteristics
- **Object lifetime**: Stack vs heap allocation, object ownership
- **Iterator patterns**: Range-based for loops, iterator invalidation
- **Memory management**: Object pools, smart pointers with containers
- **Design patterns**: Observer, Command, State patterns

## Implementation Overview
Create a Scene class that manages collections of GameObjects and provides:
1. GameObject lifecycle management (creation, destruction, updates)
2. Scene-specific resources and configuration
3. Scene transitions and state management
4. Efficient object queries and spatial organization

## Scene Class Interface

### Basic Scene Structure
```cpp
class Scene {
public:
    Scene(const std::string& name);
    virtual ~Scene();
    
    // Lifecycle
    virtual void onEnter() {}
    virtual void onExit() {}
    virtual void update(double deltaTime);
    virtual void render(Renderer& renderer);
    
    // GameObject management
    GameObject* createGameObject(const std::string& name = "");
    void destroyGameObject(GameObject* obj);
    GameObject* findGameObject(const std::string& name);
    
    // Scene properties
    const std::string& getName() const { return sceneName; }
    size_t getObjectCount() const { return gameObjects.size(); }
    
private:
    std::string sceneName;
    std::vector<std::unique_ptr<GameObject>> gameObjects;
    std::vector<GameObject*> objectsToDestroy;
};
```

## Key C++ Concepts to Learn

### Container Choice and Performance
```cpp
class Scene {
private:
    // Option 1: Vector - good cache locality, fast iteration
    std::vector<std::unique_ptr<GameObject>> gameObjects;
    
    // Option 2: List - fast insertion/deletion, poor cache performance
    std::list<std::unique_ptr<GameObject>> gameObjects;
    
    // Option 3: Map - fast lookup by name, but overhead
    std::map<std::string, std::unique_ptr<GameObject>> namedObjects;
    
    // Hybrid approach - vector for iteration, map for lookup
    std::vector<std::unique_ptr<GameObject>> gameObjects;
    std::unordered_map<std::string, GameObject*> objectsByName;
};
```

### Safe Object Destruction
```cpp
class Scene {
private:
    std::vector<std::unique_ptr<GameObject>> gameObjects;
    std::vector<GameObject*> pendingDestruction;
    
public:
    void destroyGameObject(GameObject* obj) {
        // Don't destroy immediately - mark for destruction
        pendingDestruction.push_back(obj);
    }
    
    void update(double deltaTime) {
        // Update all objects first
        for(auto& obj : gameObjects) {
            obj->update(deltaTime);
        }
        
        // Then handle destructions
        processPendingDestructions();
    }
    
private:
    void processPendingDestructions() {
        for(GameObject* obj : pendingDestruction) {
            auto it = std::find_if(gameObjects.begin(), gameObjects.end(),
                [obj](const std::unique_ptr<GameObject>& ptr) {
                    return ptr.get() == obj;
                });
            if(it != gameObjects.end()) {
                gameObjects.erase(it);
            }
        }
        pendingDestruction.clear();
    }
};
```

### Range-Based For Loops
```cpp
class Scene {
public:
    void update(double deltaTime) {
        // Modern C++ range-based for loop
        for(auto& gameObject : gameObjects) {
            gameObject->update(deltaTime);
        }
    }
    
    void render(Renderer& renderer) {
        for(const auto& gameObject : gameObjects) {
            gameObject->render(renderer);
        }
    }
};
```

## Scene Management Patterns

### Scene Factory Pattern
```cpp
class SceneFactory {
public:
    static std::unique_ptr<Scene> createScene(const std::string& sceneType) {
        if(sceneType == "MainMenu") return std::make_unique<MainMenuScene>();
        if(sceneType == "GamePlay") return std::make_unique<GamePlayScene>();
        if(sceneType == "GameOver") return std::make_unique<GameOverScene>();
        return nullptr;
    }
};
```

### Scene Inheritance
```cpp
// Base scene for common functionality
class Scene { /* base implementation */ };

// Specific scene types
class MainMenuScene : public Scene {
public:
    MainMenuScene() : Scene("MainMenu") {}
    
    void onEnter() override {
        // Create menu UI objects
        auto title = createGameObject("Title");
        // ... setup menu
    }
    
    void update(double deltaTime) override {
        Scene::update(deltaTime); // Call base update
        // Handle menu-specific logic
    }
};
```

## Advanced Scene Features

### Object Queries and Filtering
```cpp
class Scene {
public:
    // Find objects by component type
    template<typename ComponentType>
    std::vector<GameObject*> findObjectsWithComponent() {
        std::vector<GameObject*> result;
        for(auto& obj : gameObjects) {
            if(obj->getComponent<ComponentType>()) {
                result.push_back(obj.get());
            }
        }
        return result;
    }
    
    // Find objects by tag
    std::vector<GameObject*> findObjectsWithTag(const std::string& tag) {
        std::vector<GameObject*> result;
        for(auto& obj : gameObjects) {
            if(obj->hasTag(tag)) {
                result.push_back(obj.get());
            }
        }
        return result;
    }
};
```

### Layer System
```cpp
class Scene {
private:
    struct Layer {
        std::string name;
        std::vector<GameObject*> objects;
        bool visible = true;
        int renderOrder = 0;
    };
    
    std::map<std::string, Layer> layers;
    
public:
    void addObjectToLayer(GameObject* obj, const std::string& layerName);
    void setLayerVisible(const std::string& layerName, bool visible);
    void setLayerRenderOrder(const std::string& layerName, int order);
};
```

## Performance Optimization

### Object Pooling Integration
```cpp
class Scene {
private:
    ObjectPool<GameObject> objectPool;
    
public:
    GameObject* createGameObject(const std::string& name = "") {
        GameObject* obj = objectPool.acquire();
        obj->reset(); // Clear previous state
        obj->setName(name);
        gameObjects.push_back(std::unique_ptr<GameObject>(obj));
        return obj;
    }
    
    void destroyGameObject(GameObject* obj) {
        // Return to pool instead of actual destruction
        objectPool.release(obj);
        // Remove from active objects
        removeFromActiveObjects(obj);
    }
};
```

### Spatial Partitioning
```cpp
class Scene {
private:
    struct Quadrant {
        int x, y, width, height;
        std::vector<GameObject*> objects;
    };
    
    std::vector<Quadrant> spatialGrid;
    
public:
    std::vector<GameObject*> getObjectsInRegion(int x, int y, int w, int h);
    void updateSpatialIndex(GameObject* obj);
};
```

## Scene Resource Management

### Resource Loading/Unloading
```cpp
class Scene {
private:
    std::map<std::string, std::shared_ptr<Texture>> textures;
    std::map<std::string, std::shared_ptr<Sound>> sounds;
    
public:
    void onEnter() override {
        loadResources();
        setupGameObjects();
    }
    
    void onExit() override {
        unloadResources();
    }
    
private:
    virtual void loadResources() {}
    virtual void unloadResources() {
        textures.clear();
        sounds.clear();
    }
};
```

## Implementation Tips

### GameObject Creation Patterns
```cpp
class Scene {
public:
    // Simple creation
    GameObject* createPlayer(float x, float y) {
        auto player = createGameObject("Player");
        auto transform = player->addComponent<Transform>();
        auto renderable = player->addComponent<Renderable>();
        auto movement = player->addComponent<MovementComponent>();
        
        transform->setPosition(x, y);
        renderable->character = '@';
        
        return player;
    }
    
    // Template-based creation
    template<typename SceneObjectType>
    SceneObjectType* createSceneObject() {
        auto obj = std::make_unique<SceneObjectType>();
        SceneObjectType* result = obj.get();
        sceneObjects.push_back(std::move(obj));
        return result;
    }
};
```

### Update Order Management
```cpp
enum class UpdatePhase {
    PreUpdate,
    Update,
    PostUpdate,
    Render
};

class Scene {
public:
    void update(double deltaTime) {
        updatePhase(UpdatePhase::PreUpdate, deltaTime);
        updatePhase(UpdatePhase::Update, deltaTime);
        updatePhase(UpdatePhase::PostUpdate, deltaTime);
    }
    
private:
    void updatePhase(UpdatePhase phase, double deltaTime) {
        for(auto& obj : gameObjects) {
            obj->updatePhase(phase, deltaTime);
        }
    }
};
```

## Common Design Decisions

### Object Ownership
- Scene owns GameObjects
- GameObjects own Components
- Clear ownership hierarchy prevents leaks

### Update/Render Separation
- Update modifies state
- Render only reads state
- Allows for different update/render frequencies

## Testing Strategy
- Unit tests for object lifecycle
- Performance tests for large object counts
- Memory leak detection
- Scene transition testing

## Integration with Engine

### Scene Manager Integration
```cpp
class Engine {
private:
    std::unique_ptr<Scene> currentScene;
    std::unique_ptr<Scene> nextScene;
    
public:
    void setScene(std::unique_ptr<Scene> scene) {
        nextScene = std::move(scene);
    }
    
    void update(double deltaTime) {
        if(nextScene) {
            if(currentScene) currentScene->onExit();
            currentScene = std::move(nextScene);
            currentScene->onEnter();
        }
        
        if(currentScene) {
            currentScene->update(deltaTime);
        }
    }
};
```

## Resources
- [STL Containers Performance](https://en.cppreference.com/w/cpp/container)
- [Object Lifetime in C++](https://en.cppreference.com/w/cpp/language/lifetime)
- [Iterator Invalidation Rules](https://stackoverflow.com/questions/6438086/iterator-invalidation-rules)

## Fun Challenges
- Implement scene serialization/deserialization
- Add scene scripting support
- Create visual scene editor
- Implement scene streaming for large worlds
- Add scene networking/synchronization

## Debugging Tips
- Add scene object visualization
- Implement scene inspection tools
- Create object lifecycle logging
- Add performance profiling per scene

## Common Gotchas
- **Iterator invalidation**: Modifying containers while iterating
- **Object lifetime**: Accessing destroyed objects
- **Memory leaks**: Circular references between objects
- **Performance**: O(n) searches for objects
- **Threading**: Scene access from multiple threads

## Refactoring Process

### Step 1: Basic Scene Structure
1. Create Scene class with GameObject vector
2. Implement basic update/render loops
3. Add simple object creation/destruction

### Step 2: Add Object Management
1. Implement safe destruction pattern
2. Add object lookup by name
3. Add object queries by component type

### Step 3: Add Scene Lifecycle
1. Implement onEnter/onExit methods
2. Add scene inheritance support
3. Test scene transitions

### Step 4: Performance Optimization
1. Add object pooling
2. Implement spatial partitioning if needed
3. Profile and optimize hot paths