# Phase 2.3: Component System Foundation

## C++ Learning Focus
- **Inheritance**: Base classes and virtual functions
- **Polymorphism**: Virtual function calls and dynamic dispatch
- **Container classes**: `std::vector`, `std::map`, `std::unordered_map`
- **Type identification**: `typeid` and RTTI basics
- **Template specialization**: Component storage optimization

## Implementation Overview
Create a flexible system where game objects can have different combinations of behavior:
1. Define base Component class
2. Create specific component types (Transform, Renderable, etc.)
3. Implement component storage and management
4. Design component communication patterns

## Component System Design

### Base Component Class
```cpp
class Component {
public:
    Component() = default;
    virtual ~Component() = default;
    
    // Pure virtual functions for polymorphism
    virtual void update(double deltaTime) = 0;
    virtual void render(Renderer& renderer) {}
    
    // Component lifecycle
    virtual void onAttach() {}
    virtual void onDetach() {}
    
    // RTTI support
    virtual const char* getTypeName() const = 0;
    
protected:
    // Reference to owner (set by GameObject)
    class GameObject* owner = nullptr;
    friend class GameObject;
};
```

### Component Types

#### Transform Component
```cpp
class Transform : public Component {
public:
    float x = 0.0f, y = 0.0f;
    float rotation = 0.0f;
    float scaleX = 1.0f, scaleY = 1.0f;
    
    void update(double deltaTime) override {}
    const char* getTypeName() const override { return "Transform"; }
    
    // Helper methods
    void translate(float dx, float dy);
    void rotate(float angle);
    void setPosition(float newX, float newY);
};
```

#### Renderable Component
```cpp
class Renderable : public Component {
public:
    char character = '*';
    int color = 7; // White
    bool visible = true;
    
    void update(double deltaTime) override {}
    void render(Renderer& renderer) override;
    const char* getTypeName() const override { return "Renderable"; }
};
```

## Key C++ Concepts to Learn

### Virtual Functions and Polymorphism
```cpp
// Base class defines interface
class Component {
public:
    virtual void update(double deltaTime) = 0; // Pure virtual
    virtual void render(Renderer& renderer) {} // Virtual with default
    virtual ~Component() = default; // Virtual destructor essential!
};

// Derived classes implement specific behavior
class MovementComponent : public Component {
public:
    void update(double deltaTime) override {
        // Move the object based on velocity
    }
};
```

### Container Management
```cpp
class GameObject {
private:
    std::vector<std::unique_ptr<Component>> components;
    
public:
    template<typename T>
    T* addComponent() {
        auto component = std::make_unique<T>();
        T* result = component.get();
        component->owner = this;
        components.push_back(std::move(component));
        component->onAttach();
        return result;
    }
    
    template<typename T>
    T* getComponent() {
        for(auto& comp : components) {
            if(T* casted = dynamic_cast<T*>(comp.get())) {
                return casted;
            }
        }
        return nullptr;
    }
};
```

## Component Communication Patterns

### Owner Reference Pattern
```cpp
class MovementComponent : public Component {
public:
    void update(double deltaTime) override {
        if(Transform* transform = owner->getComponent<Transform>()) {
            transform->x += velocity.x * deltaTime;
            transform->y += velocity.y * deltaTime;
        }
    }
    
private:
    Vector2 velocity;
};
```

### Event System Pattern
```cpp
class Component {
public:
    virtual void onMessage(const std::string& message, void* data) {}
};

class GameObject {
public:
    void sendMessage(const std::string& message, void* data = nullptr) {
        for(auto& comp : components) {
            comp->onMessage(message, data);
        }
    }
};
```

## Advanced Component Features

### Component Dependencies
```cpp
class Renderable : public Component {
public:
    void onAttach() override {
        // Ensure Transform component exists
        if(!owner->getComponent<Transform>()) {
            throw std::runtime_error("Renderable requires Transform component");
        }
    }
};
```

### Component Caching
```cpp
class MovementComponent : public Component {
private:
    Transform* cachedTransform = nullptr;
    
public:
    void onAttach() override {
        cachedTransform = owner->getComponent<Transform>();
    }
    
    void update(double deltaTime) override {
        if(cachedTransform) {
            cachedTransform->x += velocity.x * deltaTime;
            cachedTransform->y += velocity.y * deltaTime;
        }
    }
};
```

## Performance Considerations

### Memory Layout
- Components stored in vectors for cache locality
- Consider data-oriented design for high-performance systems
- Pool allocation for frequently created/destroyed components

### Type-Safe Component Access
```cpp
// Fast type identification without RTTI
enum class ComponentType {
    Transform = 0,
    Renderable = 1,
    Movement = 2,
    // Add more as needed
};

class Component {
public:
    virtual ComponentType getType() const = 0;
};

class Transform : public Component {
public:
    ComponentType getType() const override { return ComponentType::Transform; }
};
```

## Implementation Tips

### Component Factory Pattern
```cpp
class ComponentFactory {
public:
    static std::unique_ptr<Component> create(const std::string& typeName) {
        if(typeName == "Transform") return std::make_unique<Transform>();
        if(typeName == "Renderable") return std::make_unique<Renderable>();
        // ... etc
        return nullptr;
    }
};
```

### Component Serialization Preparation
```cpp
class Component {
public:
    virtual void serialize(std::ostream& out) const {}
    virtual void deserialize(std::istream& in) {}
};
```

## Common Design Patterns

### Component as Interface
- Components define what an object can do
- Multiple inheritance alternative
- Composition over inheritance

### Update Order Management
```cpp
enum class UpdatePhase {
    PreUpdate,
    Update, 
    PostUpdate,
    Render
};

class Component {
public:
    virtual UpdatePhase getUpdatePhase() const { return UpdatePhase::Update; }
};
```

## Testing Strategy
- Unit tests for individual component types
- Integration tests for component interactions
- Performance tests for component lookup/iteration
- Memory leak detection for component lifecycle

## Common Gotchas
- **Virtual destructor**: Essential for proper cleanup
- **Dangling pointers**: Component references to destroyed objects
- **Circular dependencies**: Components referencing each other
- **Performance**: Excessive use of `dynamic_cast`
- **Memory management**: Proper ownership of components

## Integration Examples

### Simple GameObject Usage
```cpp
// Create a moving, renderable object
GameObject player;
auto transform = player.addComponent<Transform>();
auto renderable = player.addComponent<Renderable>();
auto movement = player.addComponent<MovementComponent>();

transform->setPosition(10, 10);
renderable->character = '@';
movement->setVelocity(5.0f, 0.0f);

// In game loop
player.update(deltaTime);
player.render(renderer);
```

### Scene Integration
```cpp
class Scene {
private:
    std::vector<GameObject> gameObjects;
    
public:
    void update(double deltaTime) {
        for(auto& obj : gameObjects) {
            obj.update(deltaTime);
        }
    }
    
    void render(Renderer& renderer) {
        for(auto& obj : gameObjects) {
            obj.render(renderer);
        }
    }
};
```

## Resources
- [Component-Entity-System Architecture](https://en.wikipedia.org/wiki/Entity_component_system)
- [C++ Virtual Functions](https://en.cppreference.com/w/cpp/language/virtual)
- [Smart Pointers and Ownership](https://en.cppreference.com/w/cpp/memory)
- [Dynamic Cast Performance](https://stackoverflow.com/questions/579887/how-expensive-is-rtti)

## Fun Challenges
- Implement component scripting support
- Add component hot-reloading
- Create visual component editor
- Implement component networking/synchronization
- Add component profiling and debugging tools

## Debugging Tips
- Add component type visualization
- Implement component dependency checking
- Create component inspection tools
- Add component lifecycle logging

## Refactoring Process

### Step 1: Create Base Component
1. Define Component base class
2. Add virtual destructor and basic interface
3. Test with simple derived class

### Step 2: Add Basic Components
1. Implement Transform component
2. Add Renderable component
3. Test component creation and basic functionality

### Step 3: Implement Component Management
1. Add component storage to GameObject
2. Implement addComponent/getComponent templates
3. Add component lifecycle management

### Step 4: Add Communication
1. Implement owner references
2. Add component message system
3. Test component interactions

This component system provides the foundation for flexible game object behavior while teaching important C++ concepts like inheritance, polymorphism, and template programming.