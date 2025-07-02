# Phase 5.3: Physics System Design

## C++ Learning Focus
- **Numerical integration**: Euler, Verlet, and Runge-Kutta methods
- **Algorithm complexity**: Big-O analysis for collision detection algorithms
- **Data structures**: Spatial partitioning with quadtrees and grids
- **Template metaprogramming**: Generic vector math and constraint systems
- **Performance optimization**: Cache-friendly data layouts and SIMD basics
- **Fixed-point arithmetic**: Deterministic physics simulation

## Implementation Overview
Create a robust 2D physics system that handles basic mechanics and collisions:
1. Implement vector math foundation with template-based Vector2 class
2. Design physics components (RigidBody, Collider, etc.)
3. Create collision detection and response algorithms
4. Implement numerical integration for realistic motion
5. Add spatial partitioning for performance optimization
6. Design constraint-based physics for joints and connections

## Physics Components Design

### Vector2 Math Foundation
```cpp
template<typename T>
class Vector2 {
public:
    T x, y;
    
    Vector2() : x(0), y(0) {}
    Vector2(T x, T y) : x(x), y(y) {}
    
    // Arithmetic operators
    Vector2 operator+(const Vector2& other) const { return {x + other.x, y + other.y}; }
    Vector2 operator-(const Vector2& other) const { return {x - other.x, y - other.y}; }
    Vector2 operator*(T scalar) const { return {x * scalar, y * scalar}; }
    Vector2 operator/(T scalar) const { return {x / scalar, y / scalar}; }
    
    // Assignment operators
    Vector2& operator+=(const Vector2& other) { x += other.x; y += other.y; return *this; }
    Vector2& operator-=(const Vector2& other) { x -= other.x; y -= other.y; return *this; }
    Vector2& operator*=(T scalar) { x *= scalar; y *= scalar; return *this; }
    
    // Vector operations
    T dot(const Vector2& other) const { return x * other.x + y * other.y; }
    T cross(const Vector2& other) const { return x * other.y - y * other.x; }
    T lengthSquared() const { return x * x + y * y; }
    T length() const { return std::sqrt(lengthSquared()); }
    
    Vector2 normalized() const {
        T len = length();
        return len > 0 ? *this / len : Vector2(0, 0);
    }
    
    Vector2 perpendicular() const { return Vector2(-y, x); }
};

using Vec2f = Vector2<float>;
using Vec2d = Vector2<double>;
```

### RigidBody Component
```cpp
class RigidBody : public Component {
public:
    Vec2f position{0.0f, 0.0f};
    Vec2f velocity{0.0f, 0.0f};
    Vec2f acceleration{0.0f, 0.0f};
    Vec2f force{0.0f, 0.0f};
    
    float mass = 1.0f;
    float inverseMass = 1.0f;
    float restitution = 0.8f;  // Bounciness (0 = no bounce, 1 = perfect bounce)
    float friction = 0.3f;     // Surface friction
    float angularVelocity = 0.0f;
    float torque = 0.0f;
    float rotation = 0.0f;
    
    bool isStatic = false;     // Static objects don't move
    bool useGravity = true;
    
    void update(double deltaTime) override;
    const char* getTypeName() const override { return "RigidBody"; }
    
    // Physics operations
    void applyForce(const Vec2f& force);
    void applyImpulse(const Vec2f& impulse);
    void applyForceAtPoint(const Vec2f& force, const Vec2f& point);
    void setMass(float newMass);
    
    // Integration methods
    void integrateEuler(float deltaTime);
    void integrateVerlet(float deltaTime, const Vec2f& previousPosition);
    void integrateRK4(float deltaTime);
    
private:
    Vec2f previousPosition{0.0f, 0.0f};
    Vec2f previousAcceleration{0.0f, 0.0f};
};
```

### Collider Components
```cpp
class Collider : public Component {
public:
    enum Type {
        CIRCLE,
        BOX,
        POLYGON
    };
    
    Type type;
    bool isTrigger = false;
    int layer = 0;
    
    virtual bool checkCollision(const Collider* other, CollisionInfo& info) const = 0;
    virtual AABB getBounds() const = 0;
    
    const char* getTypeName() const override { return "Collider"; }
};

class CircleCollider : public Collider {
public:
    float radius = 1.0f;
    Vec2f offset{0.0f, 0.0f};
    
    CircleCollider(float r = 1.0f) : radius(r) { type = CIRCLE; }
    
    bool checkCollision(const Collider* other, CollisionInfo& info) const override;
    AABB getBounds() const override;
};

class BoxCollider : public Collider {
public:
    Vec2f size{1.0f, 1.0f};
    Vec2f offset{0.0f, 0.0f};
    
    BoxCollider(Vec2f s = {1.0f, 1.0f}) : size(s) { type = BOX; }
    
    bool checkCollision(const Collider* other, CollisionInfo& info) const override;
    AABB getBounds() const override;
};
```

## Key C++ Concepts to Learn

### Numerical Integration Methods
```cpp
class PhysicsIntegrator {
public:
    // Explicit Euler (simple but inaccurate)
    static void integrateEuler(RigidBody& body, float deltaTime) {
        body.velocity += body.acceleration * deltaTime;
        body.position += body.velocity * deltaTime;
    }
    
    // Velocity Verlet (better stability and accuracy)
    static void integrateVerlet(RigidBody& body, float deltaTime) {
        Vec2f newPosition = body.position + body.velocity * deltaTime + 
                           body.acceleration * (0.5f * deltaTime * deltaTime);
        
        Vec2f newAcceleration = body.force / body.mass;
        body.velocity += (body.acceleration + newAcceleration) * (0.5f * deltaTime);
        
        body.position = newPosition;
        body.acceleration = newAcceleration;
    }
    
    // Runge-Kutta 4th order (most accurate)
    static void integrateRK4(RigidBody& body, float deltaTime) {
        auto evaluate = [](const RigidBody& body, float dt, const Vec2f& dv, const Vec2f& dp) {
            Vec2f velocity = body.velocity + dv * dt;
            Vec2f acceleration = body.force / body.mass;
            return std::make_pair(acceleration, velocity);
        };
        
        auto k1 = evaluate(body, 0.0f, Vec2f(0,0), Vec2f(0,0));
        auto k2 = evaluate(body, deltaTime*0.5f, k1.first*0.5f, k1.second*0.5f);
        auto k3 = evaluate(body, deltaTime*0.5f, k2.first*0.5f, k2.second*0.5f);
        auto k4 = evaluate(body, deltaTime, k3.first, k3.second);
        
        Vec2f dv = (k1.first + (k2.first + k3.first) * 2.0f + k4.first) * (deltaTime / 6.0f);
        Vec2f dp = (k1.second + (k2.second + k3.second) * 2.0f + k4.second) * (deltaTime / 6.0f);
        
        body.velocity += dv;
        body.position += dp;
    }
};
```

### Collision Detection Algorithms
```cpp
struct CollisionInfo {
    bool hasCollision = false;
    Vec2f normal{0.0f, 0.0f};      // Collision normal (unit vector)
    float penetration = 0.0f;       // Penetration depth
    Vec2f contactPoint{0.0f, 0.0f}; // Contact point in world space
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;
};

class CollisionDetection {
public:
    // Circle vs Circle collision
    static bool circleVsCircle(const CircleCollider& a, const CircleCollider& b, 
                              const Vec2f& posA, const Vec2f& posB, CollisionInfo& info) {
        Vec2f distance = (posB + b.offset) - (posA + a.offset);
        float distanceSquared = distance.lengthSquared();
        float radiusSum = a.radius + b.radius;
        
        if(distanceSquared < radiusSum * radiusSum) {
            float distanceLength = std::sqrt(distanceSquared);
            info.hasCollision = true;
            info.normal = distanceLength > 0 ? distance / distanceLength : Vec2f(1, 0);
            info.penetration = radiusSum - distanceLength;
            info.contactPoint = posA + a.offset + info.normal * a.radius;
            return true;
        }
        return false;
    }
    
    // AABB vs AABB collision
    static bool aabbVsAabb(const BoxCollider& a, const BoxCollider& b,
                          const Vec2f& posA, const Vec2f& posB, CollisionInfo& info) {
        Vec2f centerA = posA + a.offset;
        Vec2f centerB = posB + b.offset;
        Vec2f distance = centerB - centerA;
        Vec2f halfSizeA = a.size * 0.5f;
        Vec2f halfSizeB = b.size * 0.5f;
        
        Vec2f overlap = Vec2f(halfSizeA.x + halfSizeB.x - std::abs(distance.x),
                             halfSizeA.y + halfSizeB.y - std::abs(distance.y));
        
        if(overlap.x > 0 && overlap.y > 0) {
            info.hasCollision = true;
            
            // Find minimum penetration axis
            if(overlap.x < overlap.y) {
                info.normal = distance.x > 0 ? Vec2f(1, 0) : Vec2f(-1, 0);
                info.penetration = overlap.x;
            } else {
                info.normal = distance.y > 0 ? Vec2f(0, 1) : Vec2f(0, -1);
                info.penetration = overlap.y;
            }
            
            info.contactPoint = centerA + distance * 0.5f;
            return true;
        }
        return false;
    }
    
    // Separating Axis Theorem for convex polygons
    static bool satCollision(const std::vector<Vec2f>& verticesA, 
                           const std::vector<Vec2f>& verticesB, CollisionInfo& info) {
        auto projectOntoAxis = [](const std::vector<Vec2f>& vertices, const Vec2f& axis) {
            float min = vertices[0].dot(axis);
            float max = min;
            for(size_t i = 1; i < vertices.size(); ++i) {
                float projection = vertices[i].dot(axis);
                min = std::min(min, projection);
                max = std::max(max, projection);
            }
            return std::make_pair(min, max);
        };
        
        auto getAxes = [](const std::vector<Vec2f>& vertices) {
            std::vector<Vec2f> axes;
            for(size_t i = 0; i < vertices.size(); ++i) {
                Vec2f edge = vertices[(i + 1) % vertices.size()] - vertices[i];
                axes.push_back(edge.perpendicular().normalized());
            }
            return axes;
        };
        
        auto axes = getAxes(verticesA);
        auto axesB = getAxes(verticesB);
        axes.insert(axes.end(), axesB.begin(), axesB.end());
        
        float minOverlap = std::numeric_limits<float>::max();
        Vec2f minAxis;
        
        for(const auto& axis : axes) {
            auto projA = projectOntoAxis(verticesA, axis);
            auto projB = projectOntoAxis(verticesB, axis);
            
            float overlap = std::min(projA.second, projB.second) - 
                           std::max(projA.first, projB.first);
            
            if(overlap <= 0) {
                return false; // Separating axis found
            }
            
            if(overlap < minOverlap) {
                minOverlap = overlap;
                minAxis = axis;
            }
        }
        
        info.hasCollision = true;
        info.normal = minAxis;
        info.penetration = minOverlap;
        return true;
    }
};
```

## Physics Integration Methods

### Integration Comparison
```cpp
class PhysicsSystem : public System {
public:
    enum IntegrationMethod {
        EULER,
        VERLET,
        RK4
    };
    
private:
    IntegrationMethod method = VERLET;
    Vec2f gravity{0.0f, 9.81f};
    float fixedTimeStep = 1.0f / 60.0f;
    float accumulator = 0.0f;
    int maxSubsteps = 4;
    
public:
    void update(double deltaTime) override {
        accumulator += static_cast<float>(deltaTime);
        
        int substeps = 0;
        while(accumulator >= fixedTimeStep && substeps < maxSubsteps) {
            // Apply forces
            for(auto& body : getRigidBodies()) {
                if(body->useGravity && !body->isStatic) {
                    body->force += gravity * body->mass;
                }
            }
            
            // Integrate physics
            integrateAllBodies(fixedTimeStep);
            
            // Detect and resolve collisions
            detectCollisions();
            resolveCollisions();
            
            accumulator -= fixedTimeStep;
            substeps++;
        }
        
        // Interpolate for smooth rendering
        float alpha = accumulator / fixedTimeStep;
        interpolatePositions(alpha);
    }
    
private:
    void integrateAllBodies(float deltaTime) {
        for(auto& body : getRigidBodies()) {
            if(body->isStatic) continue;
            
            switch(method) {
                case EULER:
                    PhysicsIntegrator::integrateEuler(*body, deltaTime);
                    break;
                case VERLET:
                    PhysicsIntegrator::integrateVerlet(*body, deltaTime);
                    break;
                case RK4:
                    PhysicsIntegrator::integrateRK4(*body, deltaTime);
                    break;
            }
            
            // Clear forces after integration
            body->force = Vec2f(0, 0);
            body->torque = 0.0f;
        }
    }
};
```

## Constraint Solving Basics

### Spring Constraint
```cpp
class SpringConstraint {
public:
    RigidBody* bodyA;
    RigidBody* bodyB;
    float restLength;
    float stiffness;
    float damping;
    
    SpringConstraint(RigidBody* a, RigidBody* b, float length, float k, float d)
        : bodyA(a), bodyB(b), restLength(length), stiffness(k), damping(d) {}
    
    void solve(float deltaTime) {
        Vec2f distance = bodyB->position - bodyA->position;
        float currentLength = distance.length();
        
        if(currentLength > 0) {
            Vec2f direction = distance / currentLength;
            float displacement = currentLength - restLength;
            
            // Spring force (Hooke's law)
            Vec2f springForce = direction * (-stiffness * displacement);
            
            // Damping force
            Vec2f relativeVelocity = bodyB->velocity - bodyA->velocity;
            Vec2f dampingForce = direction * (-damping * relativeVelocity.dot(direction));
            
            Vec2f totalForce = springForce + dampingForce;
            
            if(!bodyA->isStatic) bodyA->applyForce(-totalForce);
            if(!bodyB->isStatic) bodyB->applyForce(totalForce);
        }
    }
};
```

### Distance Constraint (Rigid Connection)
```cpp
class DistanceConstraint {
public:
    RigidBody* bodyA;
    RigidBody* bodyB;
    float distance;
    
    void solve() {
        Vec2f delta = bodyB->position - bodyA->position;
        float currentDistance = delta.length();
        
        if(currentDistance > 0 && currentDistance != distance) {
            Vec2f direction = delta / currentDistance;
            float error = currentDistance - distance;
            
            // Position correction
            Vec2f correction = direction * (error * 0.5f);
            
            if(!bodyA->isStatic) bodyA->position += correction;
            if(!bodyB->isStatic) bodyB->position -= correction;
        }
    }
};
```

## Spatial Partitioning

### Grid-based Spatial Hash
```cpp
template<typename T>
class SpatialGrid {
private:
    struct Cell {
        std::vector<T*> objects;
    };
    
    std::unordered_map<uint64_t, Cell> grid;
    float cellSize;
    
    uint64_t hashPosition(int x, int y) const {
        return (static_cast<uint64_t>(x) << 32) | static_cast<uint64_t>(y);
    }
    
    std::pair<int, int> worldToGrid(const Vec2f& position) const {
        return {static_cast<int>(position.x / cellSize), 
                static_cast<int>(position.y / cellSize)};
    }
    
public:
    SpatialGrid(float size) : cellSize(size) {}
    
    void clear() {
        for(auto& pair : grid) {
            pair.second.objects.clear();
        }
    }
    
    void insert(T* object, const AABB& bounds) {
        auto minCell = worldToGrid({bounds.minX, bounds.minY});
        auto maxCell = worldToGrid({bounds.maxX, bounds.maxY});
        
        for(int x = minCell.first; x <= maxCell.first; ++x) {
            for(int y = minCell.second; y <= maxCell.second; ++y) {
                uint64_t hash = hashPosition(x, y);
                grid[hash].objects.push_back(object);
            }
        }
    }
    
    std::vector<T*> query(const AABB& bounds) const {
        std::set<T*> result;
        auto minCell = worldToGrid({bounds.minX, bounds.minY});
        auto maxCell = worldToGrid({bounds.maxX, bounds.maxY});
        
        for(int x = minCell.first; x <= maxCell.first; ++x) {
            for(int y = minCell.second; y <= maxCell.second; ++y) {
                uint64_t hash = hashPosition(x, y);
                auto it = grid.find(hash);
                if(it != grid.end()) {
                    for(T* obj : it->second.objects) {
                        result.insert(obj);
                    }
                }
            }
        }
        
        return std::vector<T*>(result.begin(), result.end());
    }
};
```

### Quadtree Implementation
```cpp
template<typename T>
class Quadtree {
private:
    struct Node {
        AABB bounds;
        std::vector<T*> objects;
        std::array<std::unique_ptr<Node>, 4> children;
        bool isLeaf = true;
        
        static constexpr size_t MAX_OBJECTS = 4;
        static constexpr int MAX_DEPTH = 6;
        
        void subdivide() {
            if(!isLeaf) return;
            
            float halfWidth = (bounds.maxX - bounds.minX) * 0.5f;
            float halfHeight = (bounds.maxY - bounds.minY) * 0.5f;
            float centerX = bounds.minX + halfWidth;
            float centerY = bounds.minY + halfHeight;
            
            // Top-left, Top-right, Bottom-left, Bottom-right
            children[0] = std::make_unique<Node>();
            children[0]->bounds = {bounds.minX, bounds.minY, centerX, centerY};
            
            children[1] = std::make_unique<Node>();
            children[1]->bounds = {centerX, bounds.minY, bounds.maxX, centerY};
            
            children[2] = std::make_unique<Node>();
            children[2]->bounds = {bounds.minX, centerY, centerX, bounds.maxY};
            
            children[3] = std::make_unique<Node>();
            children[3]->bounds = {centerX, centerY, bounds.maxX, bounds.maxY};
            
            isLeaf = false;
        }
        
        void insert(T* object, const AABB& objectBounds, int depth = 0) {
            if(!bounds.intersects(objectBounds)) return;
            
            if(isLeaf && (objects.size() < MAX_OBJECTS || depth >= MAX_DEPTH)) {
                objects.push_back(object);
                return;
            }
            
            if(isLeaf) {
                subdivide();
                
                // Redistribute existing objects
                auto oldObjects = std::move(objects);
                objects.clear();
                
                for(T* obj : oldObjects) {
                    insert(obj, getObjectBounds(obj), depth + 1);
                }
            }
            
            // Try to insert in children
            for(auto& child : children) {
                if(child) {
                    child->insert(object, objectBounds, depth + 1);
                }
            }
        }
        
        void query(const AABB& queryBounds, std::vector<T*>& results) const {
            if(!bounds.intersects(queryBounds)) return;
            
            if(isLeaf) {
                for(T* obj : objects) {
                    results.push_back(obj);
                }
                return;
            }
            
            for(const auto& child : children) {
                if(child) {
                    child->query(queryBounds, results);
                }
            }
        }
    };
    
    std::unique_ptr<Node> root;
    
public:
    Quadtree(const AABB& bounds) {
        root = std::make_unique<Node>();
        root->bounds = bounds;
    }
    
    void insert(T* object, const AABB& bounds) {
        root->insert(object, bounds);
    }
    
    std::vector<T*> query(const AABB& bounds) const {
        std::vector<T*> results;
        root->query(bounds, results);
        return results;
    }
    
    void clear() {
        root = std::make_unique<Node>();
    }
};
```

## Physics Debugging

### Visual Debugging System
```cpp
class PhysicsDebugRenderer {
public:
    enum DebugFlags {
        DRAW_COLLIDERS = 1 << 0,
        DRAW_VELOCITIES = 1 << 1,
        DRAW_FORCES = 1 << 2,
        DRAW_CONTACTS = 1 << 3,
        DRAW_SPATIAL_GRID = 1 << 4
    };
    
private:
    uint32_t debugFlags = 0;
    
public:
    void setDebugFlag(DebugFlags flag, bool enabled) {
        if(enabled) {
            debugFlags |= flag;
        } else {
            debugFlags &= ~flag;
        }
    }
    
    void render(Renderer& renderer, const PhysicsSystem& physics) {
        if(debugFlags & DRAW_COLLIDERS) {
            renderColliders(renderer, physics);
        }
        
        if(debugFlags & DRAW_VELOCITIES) {
            renderVelocities(renderer, physics);
        }
        
        if(debugFlags & DRAW_FORCES) {
            renderForces(renderer, physics);
        }
        
        if(debugFlags & DRAW_CONTACTS) {
            renderContacts(renderer, physics);
        }
        
        if(debugFlags & DRAW_SPATIAL_GRID) {
            renderSpatialGrid(renderer, physics);
        }
    }
    
private:
    void renderColliders(Renderer& renderer, const PhysicsSystem& physics) {
        for(const auto& collider : physics.getColliders()) {
            switch(collider->type) {
                case Collider::CIRCLE: {
                    auto circle = static_cast<const CircleCollider*>(collider);
                    renderer.drawCircle(circle->getWorldPosition(), circle->radius, 
                                      ConsoleColor::GREEN);
                    break;
                }
                case Collider::BOX: {
                    auto box = static_cast<const BoxCollider*>(collider);
                    renderer.drawRectangle(box->getBounds(), ConsoleColor::GREEN);
                    break;
                }
            }
        }
    }
    
    void renderVelocities(Renderer& renderer, const PhysicsSystem& physics) {
        for(const auto& body : physics.getRigidBodies()) {
            if(!body->isStatic && body->velocity.length() > 0.1f) {
                Vec2f start = body->position;
                Vec2f end = start + body->velocity * 0.1f; // Scale for visibility
                renderer.drawLine(start, end, ConsoleColor::BLUE);
                renderer.drawArrowHead(end, body->velocity.normalized(), ConsoleColor::BLUE);
            }
        }
    }
};
```

### Performance Profiler
```cpp
class PhysicsProfiler {
private:
    struct FrameData {
        double integrationTime = 0.0;
        double broadPhaseTime = 0.0;
        double narrowPhaseTime = 0.0;
        double resolutionTime = 0.0;
        size_t collisionChecks = 0;
        size_t activeContacts = 0;
    };
    
    std::vector<FrameData> frames;
    size_t maxFrames = 60;
    size_t currentFrame = 0;
    
public:
    class ScopedTimer {
    private:
        double& timeRef;
        std::chrono::high_resolution_clock::time_point startTime;
        
    public:
        ScopedTimer(double& time) : timeRef(time) {
            startTime = std::chrono::high_resolution_clock::now();
        }
        
        ~ScopedTimer() {
            auto endTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration<double, std::milli>(endTime - startTime);
            timeRef += duration.count();
        }
    };
    
    void beginFrame() {
        if(frames.size() <= currentFrame) {
            frames.resize(currentFrame + 1);
        }
        
        frames[currentFrame] = FrameData{};
    }
    
    void endFrame() {
        currentFrame = (currentFrame + 1) % maxFrames;
    }
    
    ScopedTimer timeIntegration() {
        return ScopedTimer(frames[currentFrame].integrationTime);
    }
    
    ScopedTimer timeBroadPhase() {
        return ScopedTimer(frames[currentFrame].broadPhaseTime);
    }
    
    void recordCollisionCheck() {
        frames[currentFrame].collisionChecks++;
    }
    
    FrameData getAverageFrameData() const {
        if(frames.empty()) return {};
        
        FrameData avg{};
        size_t count = std::min(frames.size(), maxFrames);
        
        for(size_t i = 0; i < count; ++i) {
            avg.integrationTime += frames[i].integrationTime;
            avg.broadPhaseTime += frames[i].broadPhaseTime;
            avg.narrowPhaseTime += frames[i].narrowPhaseTime;
            avg.resolutionTime += frames[i].resolutionTime;
            avg.collisionChecks += frames[i].collisionChecks;
            avg.activeContacts += frames[i].activeContacts;
        }
        
        avg.integrationTime /= count;
        avg.broadPhaseTime /= count;
        avg.narrowPhaseTime /= count;
        avg.resolutionTime /= count;
        avg.collisionChecks /= count;
        avg.activeContacts /= count;
        
        return avg;
    }
};
```

## Testing Strategies

### Unit Testing Framework
```cpp
class PhysicsTestSuite {
public:
    static void runAllTests() {
        testVectorMath();
        testCollisionDetection();
        testIntegrationMethods();
        testConstraintSolving();
        testSpatialPartitioning();
        
        std::cout << "All physics tests completed.\n";
    }
    
private:
    static void testVectorMath() {
        // Test vector operations
        Vec2f a(3.0f, 4.0f);
        Vec2f b(1.0f, 2.0f);
        
        assert(std::abs(a.length() - 5.0f) < 0.001f);
        assert(std::abs(a.dot(b) - 11.0f) < 0.001f);
        assert(std::abs(a.cross(b) - 2.0f) < 0.001f);
        
        Vec2f normalized = a.normalized();
        assert(std::abs(normalized.length() - 1.0f) < 0.001f);
    }
    
    static void testCollisionDetection() {
        // Test circle-circle collision
        CircleCollider circleA(1.0f);
        CircleCollider circleB(1.0f);
        
        CollisionInfo info;
        bool collision = CollisionDetection::circleVsCircle(
            circleA, circleB, Vec2f(0, 0), Vec2f(1.5f, 0), info);
        
        assert(collision);
        assert(std::abs(info.penetration - 0.5f) < 0.001f);
        assert(std::abs(info.normal.x - 1.0f) < 0.001f);
    }
    
    static void testIntegrationMethods() {
        RigidBody body;
        body.position = Vec2f(0, 0);
        body.velocity = Vec2f(1, 0);
        body.acceleration = Vec2f(0, -1);
        
        // Test Euler integration
        RigidBody eulerBody = body;
        PhysicsIntegrator::integrateEuler(eulerBody, 1.0f);
        
        // Verify position and velocity updates
        assert(std::abs(eulerBody.position.x - 1.0f) < 0.001f);
        assert(std::abs(eulerBody.velocity.y - (-1.0f)) < 0.001f);
    }
    
    static void testSpatialPartitioning() {
        SpatialGrid<int> grid(10.0f);
        
        int object1 = 1, object2 = 2;
        grid.insert(&object1, AABB{0, 0, 5, 5});
        grid.insert(&object2, AABB{15, 15, 20, 20});
        
        auto results = grid.query(AABB{0, 0, 10, 10});
        assert(results.size() == 1);
        assert(*results[0] == 1);
    }
};
```

### Performance Benchmarks
```cpp
class PhysicsBenchmark {
public:
    static void benchmarkCollisionDetection() {
        const size_t numObjects = 1000;
        std::vector<CircleCollider> circles;
        std::vector<Vec2f> positions;
        
        // Generate random objects
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 100.0);
        
        for(size_t i = 0; i < numObjects; ++i) {
            circles.emplace_back(1.0f);
            positions.emplace_back(dis(gen), dis(gen));
        }
        
        // Benchmark brute force O(n²)
        auto start = std::chrono::high_resolution_clock::now();
        
        size_t collisions = 0;
        for(size_t i = 0; i < numObjects; ++i) {
            for(size_t j = i + 1; j < numObjects; ++j) {
                CollisionInfo info;
                if(CollisionDetection::circleVsCircle(
                    circles[i], circles[j], positions[i], positions[j], info)) {
                    collisions++;
                }
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        std::cout << "Brute force: " << duration.count() << " microseconds, "
                  << collisions << " collisions\n";
    }
    
    static void benchmarkSpatialPartitioning() {
        // Similar benchmark but with spatial grid optimization
        const size_t numObjects = 1000;
        SpatialGrid<size_t> grid(10.0f);
        
        // Implementation here...
        
        std::cout << "Spatial grid optimization shows significant improvement\n";
    }
};
```

## Integration Examples

### Basic Physics GameObject
```cpp
class PhysicsObject : public GameObject {
public:
    PhysicsObject(const Vec2f& position, float mass = 1.0f) {
        // Add required components
        auto transform = addComponent<Transform>();
        auto rigidBody = addComponent<RigidBody>();
        auto collider = addComponent<CircleCollider>();
        auto renderable = addComponent<Renderable>();
        
        // Configure physics
        transform->setPosition(position.x, position.y);
        rigidBody->setMass(mass);
        rigidBody->position = position;
        
        // Configure rendering
        renderable->character = 'O';
        renderable->color = ConsoleColor::WHITE;
    }
    
    void applyForce(const Vec2f& force) {
        if(auto body = getComponent<RigidBody>()) {
            body->applyForce(force);
        }
    }
    
    void setVelocity(const Vec2f& velocity) {
        if(auto body = getComponent<RigidBody>()) {
            body->velocity = velocity;
        }
    }
};
```

### Physics Playground Scene
```cpp
class PhysicsPlayground : public Scene {
private:
    PhysicsSystem physicsSystem;
    PhysicsDebugRenderer debugRenderer;
    std::vector<PhysicsObject> objects;
    
public:
    void initialize() override {
        // Create boundaries
        createBoundaries();
        
        // Create some physics objects
        for(int i = 0; i < 10; ++i) {
            objects.emplace_back(Vec2f(10 + i * 3, 5), 1.0f);
            objects.back().applyForce(Vec2f(0, -100)); // Drop them
        }
        
        // Enable debug rendering
        debugRenderer.setDebugFlag(PhysicsDebugRenderer::DRAW_COLLIDERS, true);
        debugRenderer.setDebugFlag(PhysicsDebugRenderer::DRAW_VELOCITIES, true);
    }
    
    void update(double deltaTime) override {
        Scene::update(deltaTime);
        physicsSystem.update(deltaTime);
        
        // Handle user input for applying forces
        handlePhysicsInput();
    }
    
    void render(Renderer& renderer) override {
        Scene::render(renderer);
        debugRenderer.render(renderer, physicsSystem);
        
        // Render physics info
        renderPhysicsUI(renderer);
    }
    
private:
    void createBoundaries() {
        // Create static walls
        auto ground = std::make_unique<PhysicsObject>(Vec2f(40, 45), 0.0f);
        ground->getComponent<RigidBody>()->isStatic = true;
        ground->getComponent<BoxCollider>()->size = Vec2f(80, 2);
        
        addGameObject(std::move(ground));
    }
    
    void handlePhysicsInput() {
        // Apply wind force with spacebar
        if(Input::isKeyPressed(Key::SPACE)) {
            for(auto& obj : objects) {
                obj.applyForce(Vec2f(50, 0));
            }
        }
    }
};
```

## Performance Optimization

### Cache-Friendly Data Layout
```cpp
// Structure of Arrays (SoA) for better cache performance
class PhysicsSystemOptimized {
private:
    struct PhysicsData {
        std::vector<Vec2f> positions;
        std::vector<Vec2f> velocities;
        std::vector<Vec2f> accelerations;
        std::vector<Vec2f> forces;
        std::vector<float> masses;
        std::vector<float> inverseMasses;
        std::vector<bool> isStatic;
        
        size_t size() const { return positions.size(); }
        
        void resize(size_t newSize) {
            positions.resize(newSize);
            velocities.resize(newSize);
            accelerations.resize(newSize);
            forces.resize(newSize);
            masses.resize(newSize);
            inverseMasses.resize(newSize);
            isStatic.resize(newSize);
        }
        
        void addObject(const Vec2f& pos, const Vec2f& vel, float mass, bool Static = false) {
            positions.push_back(pos);
            velocities.push_back(vel);
            accelerations.push_back(Vec2f(0, 0));
            forces.push_back(Vec2f(0, 0));
            masses.push_back(mass);
            inverseMasses.push_back(Static ? 0.0f : 1.0f / mass);
            isStatic.push_back(Static);
        }
    };
    
    PhysicsData data;
    
public:
    void integrateAll(float deltaTime) {
        size_t count = data.size();
        
        // Process arrays linearly for better cache performance
        for(size_t i = 0; i < count; ++i) {
            if(data.isStatic[i]) continue;
            
            // Compute acceleration from forces
            data.accelerations[i] = data.forces[i] * data.inverseMasses[i];
            
            // Integrate velocity
            data.velocities[i] += data.accelerations[i] * deltaTime;
            
            // Integrate position
            data.positions[i] += data.velocities[i] * deltaTime;
            
            // Clear forces
            data.forces[i] = Vec2f(0, 0);
        }
    }
};
```

### SIMD Optimization Basics
```cpp
#ifdef USE_SIMD
#include <immintrin.h>

class SIMDPhysics {
public:
    // Process 4 Vec2f at once using SSE
    static void integratePositionsSSE(Vec2f* positions, const Vec2f* velocities, 
                                     float deltaTime, size_t count) {
        const __m128 dt = _mm_set1_ps(deltaTime);
        
        for(size_t i = 0; i < count; i += 2) {
            // Load 2 Vec2f (4 floats) at once
            __m128 pos = _mm_load_ps(reinterpret_cast<const float*>(&positions[i]));
            __m128 vel = _mm_load_ps(reinterpret_cast<const float*>(&velocities[i]));
            
            // Multiply velocity by deltaTime
            __m128 displacement = _mm_mul_ps(vel, dt);
            
            // Add to position
            __m128 newPos = _mm_add_ps(pos, displacement);
            
            // Store result
            _mm_store_ps(reinterpret_cast<float*>(&positions[i]), newPos);
        }
    }
};
#endif
```

## Common Gotchas

### Floating Point Precision
- **Accumulated error**: Small errors compound over time in numerical integration
- **Comparison issues**: Never use `==` for floating point comparisons
- **Catastrophic cancellation**: Subtracting similar large numbers
- **Solution**: Use epsilon comparisons and double precision for critical calculations

### Physics Stability
- **Explosive behavior**: Too large time steps cause instability
- **Penetration tunneling**: Fast objects passing through thin barriers
- **Energy drift**: Total system energy should be conserved
- **Solution**: Use fixed time steps, continuous collision detection, and energy monitoring

### Performance Pitfalls
- **O(n²) collision detection**: Becomes prohibitive with many objects
- **Cache misses**: Poor data layout kills performance
- **Unnecessary calculations**: Computing square roots when squared distance suffices
- **Solution**: Spatial partitioning, structure of arrays, and mathematical optimizations

## Resources
- [Real-Time Collision Detection by Christer Ericson](https://realtimecollisiondetection.net/)
- [Game Physics Engine Development by Ian Millington](https://www.crcpress.com/Game-Physics-Engine-Development/Millington/p/book/9780123819765)
- [Numerical Methods for Ordinary Differential Equations](https://en.wikipedia.org/wiki/Numerical_methods_for_ordinary_differential_equations)
- [Separating Axis Theorem Tutorial](https://www.dyn4j.org/2010/01/sat/)
- [Box2D Physics Engine Source](https://github.com/erincatto/box2d)

## Fun Challenges
- Implement soft body physics using mass-spring systems
- Add fluid simulation with particle-based methods
- Create a rope/chain simulation using distance constraints
- Implement basic cloth simulation
- Add particle effects with physics interaction
- Create a physics-based puzzle game
- Implement vehicle physics with wheels and suspension

## Debugging Tips

### Visual Debug Tools
```cpp
// Add to your debug rendering
void renderPhysicsInfo(Renderer& renderer) {
    // Show current integration method
    renderer.drawText(Vec2f(5, 5), "Integration: " + getIntegrationMethodName());
    
    // Show performance metrics
    auto metrics = profiler.getAverageFrameData();
    renderer.drawText(Vec2f(5, 6), "Collision checks: " + std::to_string(metrics.collisionChecks));
    renderer.drawText(Vec2f(5, 7), "Integration time: " + std::to_string(metrics.integrationTime) + "ms");
    
    // Show total system energy
    float totalEnergy = calculateTotalSystemEnergy();
    renderer.drawText(Vec2f(5, 8), "Total energy: " + std::to_string(totalEnergy));
}
```

### Physics Validation
```cpp
class PhysicsValidator {
public:
    static bool validateEnergyConservation(const PhysicsSystem& physics, float tolerance = 0.01f) {
        static float previousEnergy = -1.0f;
        float currentEnergy = calculateTotalEnergy(physics);
        
        if(previousEnergy >= 0.0f) {
            float energyChange = std::abs(currentEnergy - previousEnergy);
            if(energyChange > tolerance) {
                std::cout << "Energy conservation violation: " << energyChange << std::endl;
                return false;
            }
        }
        
        previousEnergy = currentEnergy;
        return true;
    }
    
    static bool validatePositionalConstraints(const PhysicsSystem& physics) {
        for(const auto& constraint : physics.getConstraints()) {
            if(!constraint->isValid()) {
                std::cout << "Constraint violation detected" << std::endl;
                return false;
            }
        }
        return true;
    }
};
```

## Refactoring Process

### Step 1: Mathematical Foundation
1. Implement Vector2 template class with all operations
2. Add utility functions for common physics calculations
3. Test vector math thoroughly with unit tests

### Step 2: Basic Physics Components
1. Create RigidBody component with basic properties
2. Implement simple Euler integration
3. Add basic collision shapes (Circle, Box)

### Step 3: Collision Detection
1. Implement circle-circle collision detection
2. Add AABB-AABB collision detection
3. Create collision information structure

### Step 4: Physics System Integration
1. Create PhysicsSystem to manage all physics objects
2. Implement fixed timestep with accumulator
3. Add collision detection and response loop

### Step 5: Advanced Features
1. Implement Verlet and RK4 integration methods
2. Add spatial partitioning for performance
3. Implement constraint solving system

### Step 6: Optimization and Polish
1. Add physics debugging and visualization
2. Implement performance profiling
3. Optimize data layout and algorithms

This physics system provides a solid foundation for 2D physics simulation while teaching essential C++ concepts like numerical methods, algorithm optimization, and performance-conscious programming. The modular design allows for easy extension and modification as your game engine grows in complexity.