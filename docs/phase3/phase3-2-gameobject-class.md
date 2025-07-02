# Phase 3.2: GameObject Class Design

## C++ Learning Focus
- **Composition pattern**: Building complex objects from simple components
- **RAII with containers**: Managing component lifetime automatically
- **Template metaprogramming**: Type-safe component access
- **Move semantics**: Efficient object transfer and std::move
- **Friend classes**: Controlled access between related classes

## Implementation Overview
Create a GameObject class that serves as a container for components:
1. Component storage and lifecycle management
2. Unique identification and naming system
3. Hierarchical relationships (parent/child)
4. Tag system for categorization and queries
5. Transform caching and optimization

## GameObject Class Interface

### Core GameObject Structure
```cpp
class GameObject {
public:
    GameObject(const std::string& name = "GameObject");
    ~GameObject();
    
    // Component management
    template<typename T, typename... Args>
    T* addComponent(Args&&... args);
    
    template<typename T>
    T* getComponent();
    
    template<typename T>
    bool hasComponent() const;
    
    template<typename T>
    void removeComponent();
    
    // Lifecycle
    void update(double deltaTime);
    void render(Renderer& renderer);
    
    // Properties
    const std::string& getName() const { return name; }
    void setName(const std::string& newName) { name = newName; }
    
    bool isActive() const { return active; }
    void setActive(bool state) { active = state; }
    
    // Unique identification
    uint32_t getId() const { return id; }
    
private:
    std::string name;
    uint32_t id;
    bool active = true;
    
    std::vector<std::unique_ptr<Component>> components;
    
    static uint32_t nextId;
    
    friend class Scene; // Allow Scene to manage GameObject lifecycle
};
```

## Key C++ Concepts to Learn

### Template Member Functions
```cpp
class GameObject {
public:
    template<typename T, typename... Args>
    T* addComponent(Args&&... args) {
        // Perfect forwarding of constructor arguments
        auto component = std::make_unique<T>(std::forward<Args>(args)...);
        T* result = component.get();
        
        // Set up component-object relationship
        component->owner = this;
        components.push_back(std::move(component));
        
        // Call component lifecycle
        result->onAttach();
        
        return result;
    }
    
    template<typename T>
    T* getComponent() {
        for(auto& comp : components) {
            // Use dynamic_cast for type safety
            if(T* casted = dynamic_cast<T*>(comp.get())) {
                return casted;
            }
        }
        return nullptr;
    }
};
```

### Move Semantics and Perfect Forwarding
```cpp
class GameObject {
    // Move constructor
    GameObject(GameObject&& other) noexcept
        : name(std::move(other.name))
        , id(other.id)
        , active(other.active)
        , components(std::move(other.components)) {
        
        // Update component ownership
        for(auto& comp : components) {
            comp->owner = this;
        }
    }
    
    // Move assignment
    GameObject& operator=(GameObject&& other) noexcept {
        if(this != &other) {
            name = std::move(other.name);
            id = other.id;
            active = other.active;
            components = std::move(other.components);
            
            // Update component ownership
            for(auto& comp : components) {
                comp->owner = this;
            }
        }
        return *this;
    }
    
    // Delete copy operations (GameObjects should be unique)
    GameObject(const GameObject&) = delete;
    GameObject& operator=(const GameObject&) = delete;
};
```

### Static Member Management
```cpp
class GameObject {
private:
    static uint32_t nextId;
    static std::unordered_map<uint32_t, GameObject*> allObjects;
    
public:
    GameObject() : id(nextId++), name("GameObject_" + std::to_string(id)) {
        allObjects[id] = this;
    }
    
    ~GameObject() {
        allObjects.erase(id);
    }
    
    static GameObject* findById(uint32_t objectId) {
        auto it = allObjects.find(objectId);
        return (it != allObjects.end()) ? it->second : nullptr;
    }
    
    static size_t getTotalObjectCount() {
        return allObjects.size();
    }
};

// Static member definition (in .cpp file)
uint32_t GameObject::nextId = 1;
std::unordered_map<uint32_t, GameObject*> GameObject::allObjects;
```

## Advanced GameObject Features

### Tag System
```cpp
class GameObject {
private:
    std::set<std::string> tags;
    
public:
    void addTag(const std::string& tag) { tags.insert(tag); }
    void removeTag(const std::string& tag) { tags.erase(tag); }
    bool hasTag(const std::string& tag) const { return tags.count(tag) > 0; }
    
    const std::set<std::string>& getTags() const { return tags; }
    
    // Utility methods
    bool hasAnyTag(const std::vector<std::string>& tagList) const {
        for(const auto& tag : tagList) {
            if(hasTag(tag)) return true;
        }
        return false;
    }
};
```

### Hierarchical Relationships
```cpp
class GameObject {
private:
    GameObject* parent = nullptr;
    std::vector<GameObject*> children;
    
public:
    void setParent(GameObject* newParent) {
        if(parent) {
            parent->removeChild(this);
        }
        parent = newParent;
        if(parent) {
            parent->addChild(this);
        }
    }
    
    void addChild(GameObject* child) {
        if(child && std::find(children.begin(), children.end(), child) == children.end()) {
            children.push_back(child);
            child->parent = this;
        }
    }
    
    void removeChild(GameObject* child) {
        auto it = std::find(children.begin(), children.end(), child);
        if(it != children.end()) {
            (*it)->parent = nullptr;
            children.erase(it);
        }
    }
    
    // Recursive operations
    void updateHierarchy(double deltaTime) {
        if(active) {
            update(deltaTime);
            for(GameObject* child : children) {
                child->updateHierarchy(deltaTime);
            }
        }
    }
};
```

### Component Caching for Performance
```cpp
class GameObject {
private:
    // Cache frequently accessed components
    mutable Transform* cachedTransform = nullptr;
    mutable bool transformCacheValid = false;
    
public:
    Transform* getTransform() const {
        if(!transformCacheValid) {
            cachedTransform = getComponent<Transform>();
            transformCacheValid = true;
        }
        return cachedTransform;
    }
    
    template<typename T>
    T* addComponent() {
        T* result = addComponentImpl<T>();
        
        // Invalidate caches when components change
        transformCacheValid = false;
        
        return result;
    }
    
    template<typename T>
    void removeComponent() {
        removeComponentImpl<T>();
        transformCacheValid = false;
    }
};
```

## Component Communication Patterns

### Direct Component Access
```cpp
// In a component's update method
void MovementComponent::update(double deltaTime) {
    Transform* transform = owner->getComponent<Transform>();
    if(transform) {
        transform->x += velocity.x * deltaTime;
        transform->y += velocity.y * deltaTime;
    }
}
```

### Message System
```cpp
class GameObject {
public:
    void sendMessage(const std::string& message, void* data = nullptr) {
        for(auto& comp : components) {
            comp->onMessage(message, data);
        }
    }
    
    void broadcastToChildren(const std::string& message, void* data = nullptr) {
        sendMessage(message, data);
        for(GameObject* child : children) {
            child->broadcastToChildren(message, data);
        }
    }
};
```

### Event System Integration
```cpp
class GameObject {
private:
    std::map<std::string, std::vector<std::function<void(void*)>>> eventHandlers;
    
public:
    void addEventListener(const std::string& event, std::function<void(void*)> handler) {
        eventHandlers[event].push_back(handler);
    }
    
    void triggerEvent(const std::string& event, void* data = nullptr) {
        auto it = eventHandlers.find(event);
        if(it != eventHandlers.end()) {
            for(auto& handler : it->second) {
                handler(data);
            }
        }
    }
};
```

## Performance Optimization

### Component Type Indexing
```cpp
class GameObject {
private:
    // Fast component lookup by type
    std::unordered_map<std::type_index, Component*> componentMap;
    
public:
    template<typename T>
    T* getComponent() {
        auto it = componentMap.find(std::type_index(typeid(T)));
        return (it != componentMap.end()) ? static_cast<T*>(it->second) : nullptr;
    }
    
    template<typename T>
    T* addComponent() {
        auto component = std::make_unique<T>();
        T* result = component.get();
        
        componentMap[std::type_index(typeid(T))] = result;
        components.push_back(std::move(component));
        
        return result;
    }
};
```

### Memory Pool Integration
```cpp
class GameObjectPool {
private:
    std::stack<std::unique_ptr<GameObject>> availableObjects;
    std::vector<std::unique_ptr<GameObject>> allObjects;
    
public:
    GameObject* acquire() {
        if(availableObjects.empty()) {
            allObjects.push_back(std::make_unique<GameObject>());
            return allObjects.back().get();
        } else {
            GameObject* obj = availableObjects.top().release();
            availableObjects.pop();
            obj->reset(); // Clear previous state
            return obj;
        }
    }
    
    void release(GameObject* obj) {
        obj->reset();
        availableObjects.push(std::unique_ptr<GameObject>(obj));
    }
};
```

## Implementation Tips

### Component Lifecycle Management
```cpp
class GameObject {
public:
    template<typename T>
    void removeComponent() {
        for(auto it = components.begin(); it != components.end(); ++it) {
            if(T* comp = dynamic_cast<T*>(it->get())) {
                comp->onDetach();
                components.erase(it);
                break;
            }
        }
    }
    
    void removeAllComponents() {
        for(auto& comp : components) {
            comp->onDetach();
        }
        components.clear();
    }
};
```

### Debug and Inspection Support
```cpp
class GameObject {
public:
    void debugPrint() const {
        std::cout << "GameObject: " << name << " (ID: " << id << ")\n";
        std::cout << "  Active: " << (active ? "true" : "false") << "\n";
        std::cout << "  Components: " << components.size() << "\n";
        for(const auto& comp : components) {
            std::cout << "    - " << comp->getTypeName() << "\n";
        }
        std::cout << "  Tags: ";
        for(const auto& tag : tags) {
            std::cout << tag << " ";
        }
        std::cout << "\n";
    }
    
    std::vector<std::string> getComponentTypeNames() const {
        std::vector<std::string> names;
        for(const auto& comp : components) {
            names.push_back(comp->getTypeName());
        }
        return names;
    }
};
```

## Testing Strategy
- Unit tests for component management
- Integration tests with different component combinations
- Performance tests for large numbers of GameObjects
- Memory leak detection for component lifecycle

## Common Design Decisions

### GameObject Identity
- Name vs ID vs both for identification
- Unique names enforced or allowed duplicates
- Global GameObject registry or scene-local

### Component Limits
- Allow multiple components of same type or enforce uniqueness
- Component dependency validation
- Component initialization order

## Integration Examples

### Simple GameObject Usage
```cpp
// Create a player character
GameObject player("Player");
player.addTag("Player");
player.addTag("Controllable");

auto transform = player.addComponent<Transform>();
auto sprite = player.addComponent<SpriteRenderer>();
auto controller = player.addComponent<PlayerController>();

transform->setPosition(10, 10);
sprite->setCharacter('@');

// In game loop
player.update(deltaTime);
player.render(renderer);
```

### Factory Pattern Integration
```cpp
class GameObjectFactory {
public:
    static GameObject* createPlayer(float x, float y) {
        auto player = std::make_unique<GameObject>("Player");
        player->addTag("Player");
        
        auto transform = player->addComponent<Transform>();
        transform->setPosition(x, y);
        
        auto renderable = player->addComponent<Renderable>();
        renderable->character = '@';
        
        auto movement = player->addComponent<MovementComponent>();
        
        return player.release();
    }
    
    static GameObject* createEnemy(float x, float y) {
        auto enemy = std::make_unique<GameObject>("Enemy");
        enemy->addTag("Enemy");
        
        // ... setup enemy components
        
        return enemy.release();
    }
};
```

## Resources
- [C++ Templates](https://en.cppreference.com/w/cpp/language/templates)
- [Move Semantics](https://en.cppreference.com/w/cpp/utility/move)
- [RAII Pattern](https://en.cppreference.com/w/cpp/language/raii)
- [Perfect Forwarding](https://eli.thegreenplace.net/2014/perfect-forwarding-and-universal-references-in-c/)

## Fun Challenges
- Implement GameObject serialization
- Add GameObject scripting support
- Create visual GameObject inspector
- Implement GameObject networking
- Add GameObject animation system

## Debugging Tips
- Add component visualization
- Implement GameObject inspection tools
- Create component dependency checking
- Add GameObject lifecycle logging

## Common Gotchas
- **Component ownership**: Components outliving GameObjects
- **Circular references**: Parent-child cycles
- **Memory leaks**: Not properly cleaning up components
- **Performance**: Excessive dynamic_cast usage
- **Thread safety**: Accessing GameObject from multiple threads

## Refactoring Process

### Step 1: Basic GameObject Structure
1. Create GameObject class with component vector
2. Implement basic component add/get/remove
3. Add name and ID system

### Step 2: Add Advanced Features
1. Implement tag system
2. Add parent-child relationships
3. Add component caching

### Step 3: Optimize Performance
1. Add component type indexing
2. Implement object pooling
3. Profile and optimize hot paths

### Step 4: Add Debugging Support
1. Implement debug visualization
2. Add component inspection
3. Create GameObject factory patterns