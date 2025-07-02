# Phase 3.3: Transform System Design

## C++ Learning Focus
- **Vector math**: 2D vectors, dot product, cross product, magnitude calculations
- **Operator overloading**: Mathematical operators for intuitive vector operations
- **Reference semantics**: Const references, return value optimization
- **Coordinate systems**: Local vs world space transformations
- **Mathematical precision**: Floating point considerations and epsilon comparisons

## Implementation Overview
Create a Transform system that manages position, rotation, and scale for GameObjects:
1. 2D vector math foundation with Vec2 class
2. Transform component with position, rotation, scale
3. Local vs world space coordinate transformations
4. Parent-child transform hierarchies
5. Efficient transformation matrices for 2D operations

## Vector Math Foundation

### Vec2 Class Implementation
```cpp
class Vec2 {
public:
    float x, y;
    
    // Constructors
    Vec2() : x(0), y(0) {}
    Vec2(float x, float y) : x(x), y(y) {}
    Vec2(const Vec2& other) : x(other.x), y(other.y) {}
    
    // Assignment operators
    Vec2& operator=(const Vec2& other) {
        x = other.x;
        y = other.y;
        return *this;
    }
    
    // Arithmetic operators
    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }
    
    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }
    
    Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }
    
    Vec2 operator/(float scalar) const {
        return Vec2(x / scalar, y / scalar);
    }
    
    // Compound assignment operators
    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    
    Vec2& operator-=(const Vec2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    
    Vec2& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }
    
    // Vector operations
    float magnitude() const {
        return std::sqrt(x * x + y * y);
    }
    
    float magnitudeSquared() const {
        return x * x + y * y;
    }
    
    Vec2 normalized() const {
        float mag = magnitude();
        if(mag > 0.001f) {
            return *this / mag;
        }
        return Vec2(0, 0);
    }
    
    void normalize() {
        float mag = magnitude();
        if(mag > 0.001f) {
            x /= mag;
            y /= mag;
        }
    }
    
    float dot(const Vec2& other) const {
        return x * other.x + y * other.y;
    }
    
    // Cross product in 2D returns scalar (z-component)
    float cross(const Vec2& other) const {
        return x * other.y - y * other.x;
    }
    
    // Utility functions
    float distance(const Vec2& other) const {
        return (*this - other).magnitude();
    }
    
    float angle() const {
        return std::atan2(y, x);
    }
    
    Vec2 rotate(float radians) const {
        float cos_a = std::cos(radians);
        float sin_a = std::sin(radians);
        return Vec2(x * cos_a - y * sin_a, x * sin_a + y * cos_a);
    }
    
    // Static utility functions
    static Vec2 zero() { return Vec2(0, 0); }
    static Vec2 one() { return Vec2(1, 1); }
    static Vec2 up() { return Vec2(0, 1); }
    static Vec2 down() { return Vec2(0, -1); }
    static Vec2 left() { return Vec2(-1, 0); }
    static Vec2 right() { return Vec2(1, 0); }
};

// Non-member operators for commutative operations
Vec2 operator*(float scalar, const Vec2& vec) {
    return vec * scalar;
}

// Comparison operators
bool operator==(const Vec2& a, const Vec2& b) {
    const float epsilon = 0.001f;
    return std::abs(a.x - b.x) < epsilon && std::abs(a.y - b.y) < epsilon;
}

bool operator!=(const Vec2& a, const Vec2& b) {
    return !(a == b);
}

// Stream output for debugging
std::ostream& operator<<(std::ostream& os, const Vec2& vec) {
    os << "(" << vec.x << ", " << vec.y << ")";
    return os;
}
```

## Key C++ Concepts to Learn

### Operator Overloading Best Practices
```cpp
class Vec2 {
public:
    // Return by value for arithmetic operations
    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }
    
    // Return by reference for compound assignments
    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this; // Enable chaining: v1 += v2 += v3
    }
    
    // Const member functions don't modify object
    float magnitude() const {
        return std::sqrt(x * x + y * y);
    }
    
    // Non-const member functions can modify object
    void normalize() {
        float mag = magnitude();
        if(mag > 0.001f) {
            x /= mag;
            y /= mag;
        }
    }
};

// Free function operators for symmetry
Vec2 operator*(float scalar, const Vec2& vec) {
    return vec * scalar; // Allows both: vec * 2.0f and 2.0f * vec
}
```

### Reference Semantics and Performance
```cpp
class Vec2 {
public:
    // Pass by const reference for input parameters
    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }
    
    // Return by reference for chaining operations
    Vec2& normalize() {
        float mag = magnitude();
        if(mag > 0.001f) {
            x /= mag;
            y /= mag;
        }
        return *this;
    }
    
    // Return by value when creating new objects
    Vec2 normalized() const {
        Vec2 result = *this;
        result.normalize();
        return result; // RVO (Return Value Optimization) applies
    }
    
    // Use const member functions for read-only operations
    float distance(const Vec2& other) const {
        return (*this - other).magnitude();
    }
};
```

### Floating Point Precision Handling
```cpp
class Vec2 {
private:
    static constexpr float EPSILON = 0.001f;
    
public:
    bool isZero() const {
        return std::abs(x) < EPSILON && std::abs(y) < EPSILON;
    }
    
    bool equals(const Vec2& other, float epsilon = EPSILON) const {
        return std::abs(x - other.x) < epsilon && 
               std::abs(y - other.y) < epsilon;
    }
    
    Vec2 normalized() const {
        float mag = magnitude();
        if(mag > EPSILON) {
            return *this / mag;
        }
        return Vec2::zero(); // Return zero vector for near-zero magnitude
    }
};
```

## Transform Component Implementation

### Basic Transform Class
```cpp
class Transform : public Component {
private:
    Vec2 localPosition = Vec2::zero();
    float localRotation = 0.0f; // Radians
    Vec2 localScale = Vec2::one();
    
    // Cached world transform data
    mutable Vec2 cachedWorldPosition;
    mutable float cachedWorldRotation = 0.0f;
    mutable Vec2 cachedWorldScale;
    mutable bool worldTransformDirty = true;
    
public:
    Transform() = default;
    
    // Local space accessors
    const Vec2& getLocalPosition() const { return localPosition; }
    void setLocalPosition(const Vec2& pos) {
        localPosition = pos;
        markWorldTransformDirty();
    }
    
    float getLocalRotation() const { return localRotation; }
    void setLocalRotation(float rotation) {
        localRotation = rotation;
        markWorldTransformDirty();
    }
    
    const Vec2& getLocalScale() const { return localScale; }
    void setLocalScale(const Vec2& scale) {
        localScale = scale;
        markWorldTransformDirty();
    }
    
    // World space accessors (computed from hierarchy)
    const Vec2& getWorldPosition() const {
        updateWorldTransform();
        return cachedWorldPosition;
    }
    
    float getWorldRotation() const {
        updateWorldTransform();
        return cachedWorldRotation;
    }
    
    const Vec2& getWorldScale() const {
        updateWorldTransform();
        return cachedWorldScale;
    }
    
    // Transformation operations
    void translate(const Vec2& offset) {
        localPosition += offset;
        markWorldTransformDirty();
    }
    
    void rotate(float radians) {
        localRotation += radians;
        markWorldTransformDirty();
    }
    
    void scale(const Vec2& factor) {
        localScale.x *= factor.x;
        localScale.y *= factor.y;
        markWorldTransformDirty();
    }
    
    // Direction vectors in world space
    Vec2 getForward() const {
        return Vec2(std::cos(getWorldRotation()), std::sin(getWorldRotation()));
    }
    
    Vec2 getRight() const {
        Vec2 forward = getForward();
        return Vec2(forward.y, -forward.x); // 90-degree rotation
    }
    
private:
    void updateWorldTransform() const {
        if(!worldTransformDirty) return;
        
        GameObject* parent = owner->getParent();
        if(parent) {
            Transform* parentTransform = parent->getComponent<Transform>();
            if(parentTransform) {
                // Apply parent transformation
                Vec2 parentWorldPos = parentTransform->getWorldPosition();
                float parentWorldRot = parentTransform->getWorldRotation();
                Vec2 parentWorldScale = parentTransform->getWorldScale();
                
                // Transform local position by parent
                Vec2 rotatedPos = localPosition.rotate(parentWorldRot);
                rotatedPos.x *= parentWorldScale.x;
                rotatedPos.y *= parentWorldScale.y;
                
                cachedWorldPosition = parentWorldPos + rotatedPos;
                cachedWorldRotation = parentWorldRot + localRotation;
                cachedWorldScale.x = parentWorldScale.x * localScale.x;
                cachedWorldScale.y = parentWorldScale.y * localScale.y;
            }
        } else {
            // No parent - local space is world space
            cachedWorldPosition = localPosition;
            cachedWorldRotation = localRotation;
            cachedWorldScale = localScale;
        }
        
        worldTransformDirty = false;
    }
    
    void markWorldTransformDirty() {
        worldTransformDirty = true;
        // Mark children dirty too
        markChildrenDirty();
    }
    
    void markChildrenDirty() {
        if(owner) {
            for(GameObject* child : owner->getChildren()) {
                if(Transform* childTransform = child->getComponent<Transform>()) {
                    childTransform->worldTransformDirty = true;
                    childTransform->markChildrenDirty();
                }
            }
        }
    }
};
```

## Transformation Matrices (2D)

### Matrix2D Class for Advanced Transformations
```cpp
class Matrix2D {
private:
    float m[9]; // 3x3 matrix stored in row-major order
    
public:
    Matrix2D() {
        // Initialize as identity matrix
        for(int i = 0; i < 9; ++i) m[i] = 0.0f;
        m[0] = m[4] = m[8] = 1.0f; // Diagonal elements
    }
    
    Matrix2D(float m00, float m01, float m02,
             float m10, float m11, float m12,
             float m20, float m21, float m22) {
        m[0] = m00; m[1] = m01; m[2] = m02;
        m[3] = m10; m[4] = m11; m[5] = m12;
        m[6] = m20; m[7] = m21; m[8] = m22;
    }
    
    // Access operators
    float& operator()(int row, int col) {
        return m[row * 3 + col];
    }
    
    const float& operator()(int row, int col) const {
        return m[row * 3 + col];
    }
    
    // Matrix multiplication
    Matrix2D operator*(const Matrix2D& other) const {
        Matrix2D result;
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                result(i, j) = 0;
                for(int k = 0; k < 3; ++k) {
                    result(i, j) += (*this)(i, k) * other(k, j);
                }
            }
        }
        return result;
    }
    
    // Transform a point
    Vec2 transformPoint(const Vec2& point) const {
        float x = m[0] * point.x + m[1] * point.y + m[2];
        float y = m[3] * point.x + m[4] * point.y + m[5];
        return Vec2(x, y);
    }
    
    // Transform a vector (ignore translation)
    Vec2 transformVector(const Vec2& vector) const {
        float x = m[0] * vector.x + m[1] * vector.y;
        float y = m[3] * vector.x + m[4] * vector.y;
        return Vec2(x, y);
    }
    
    // Static factory methods
    static Matrix2D identity() {
        return Matrix2D();
    }
    
    static Matrix2D translation(const Vec2& offset) {
        return Matrix2D(1, 0, offset.x,
                       0, 1, offset.y,
                       0, 0, 1);
    }
    
    static Matrix2D rotation(float radians) {
        float cos_r = std::cos(radians);
        float sin_r = std::sin(radians);
        return Matrix2D(cos_r, -sin_r, 0,
                       sin_r,  cos_r, 0,
                       0,      0,     1);
    }
    
    static Matrix2D scale(const Vec2& scale) {
        return Matrix2D(scale.x, 0, 0,
                       0, scale.y, 0,
                       0, 0,       1);
    }
    
    static Matrix2D trs(const Vec2& translation, float rotation, const Vec2& scale) {
        return Matrix2D::translation(translation) * 
               Matrix2D::rotation(rotation) * 
               Matrix2D::scale(scale);
    }
};
```

### Matrix-Based Transform Implementation
```cpp
class Transform : public Component {
public:
    Matrix2D getLocalToWorldMatrix() const {
        updateWorldTransform();
        return Matrix2D::trs(getWorldPosition(), getWorldRotation(), getWorldScale());
    }
    
    Matrix2D getWorldToLocalMatrix() const {
        // Inverse transformation: Scale^-1 * Rotation^-1 * Translation^-1
        Vec2 worldPos = getWorldPosition();
        float worldRot = getWorldRotation();
        Vec2 worldScale = getWorldScale();
        
        return Matrix2D::scale(Vec2(1.0f / worldScale.x, 1.0f / worldScale.y)) *
               Matrix2D::rotation(-worldRot) *
               Matrix2D::translation(-worldPos);
    }
    
    Vec2 transformPoint(const Vec2& localPoint) const {
        return getLocalToWorldMatrix().transformPoint(localPoint);
    }
    
    Vec2 inverseTransformPoint(const Vec2& worldPoint) const {
        return getWorldToLocalMatrix().transformPoint(worldPoint);
    }
};
```

## Coordinate System Management

### Local vs World Space Conversions
```cpp
class Transform : public Component {
public:
    // Convert point from local space to world space
    Vec2 localToWorld(const Vec2& localPoint) const {
        return transformPoint(localPoint);
    }
    
    // Convert point from world space to local space
    Vec2 worldToLocal(const Vec2& worldPoint) const {
        return inverseTransformPoint(worldPoint);
    }
    
    // Convert direction from local space to world space
    Vec2 localToWorldDirection(const Vec2& localDirection) const {
        return getLocalToWorldMatrix().transformVector(localDirection);
    }
    
    // Convert direction from world space to local space
    Vec2 worldToLocalDirection(const Vec2& worldDirection) const {
        return getWorldToLocalMatrix().transformVector(worldDirection);
    }
    
    // Utility methods for common operations
    void lookAt(const Vec2& target) {
        Vec2 direction = (target - getWorldPosition()).normalized();
        setWorldRotation(direction.angle());
    }
    
    void setWorldPosition(const Vec2& worldPos) {
        GameObject* parent = owner->getParent();
        if(parent && parent->getComponent<Transform>()) {
            Vec2 localPos = parent->getComponent<Transform>()->worldToLocal(worldPos);
            setLocalPosition(localPos);
        } else {
            setLocalPosition(worldPos);
        }
    }
    
    void setWorldRotation(float worldRotation) {
        GameObject* parent = owner->getParent();
        if(parent && parent->getComponent<Transform>()) {
            float parentWorldRotation = parent->getComponent<Transform>()->getWorldRotation();
            setLocalRotation(worldRotation - parentWorldRotation);
        } else {
            setLocalRotation(worldRotation);
        }
    }
};
```

### Coordinate System Utilities
```cpp
namespace CoordinateSystem {
    // Screen to world coordinate conversion
    Vec2 screenToWorld(const Vec2& screenPos, const Vec2& cameraPos, float cameraZoom) {
        return (screenPos / cameraZoom) + cameraPos;
    }
    
    // World to screen coordinate conversion
    Vec2 worldToScreen(const Vec2& worldPos, const Vec2& cameraPos, float cameraZoom) {
        return (worldPos - cameraPos) * cameraZoom;
    }
    
    // Grid snapping
    Vec2 snapToGrid(const Vec2& position, float gridSize) {
        return Vec2(
            std::round(position.x / gridSize) * gridSize,
            std::round(position.y / gridSize) * gridSize
        );
    }
    
    // Distance functions accounting for coordinate system
    float manhattanDistance(const Vec2& a, const Vec2& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }
    
    float chebyshevDistance(const Vec2& a, const Vec2& b) {
        return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
    }
}
```

## Performance Considerations

### Transform Caching Strategy
```cpp
class Transform : public Component {
private:
    // Dirty flag system
    mutable bool localTransformDirty = true;
    mutable bool worldTransformDirty = true;
    
    // Cached matrices
    mutable Matrix2D cachedLocalMatrix;
    mutable Matrix2D cachedWorldMatrix;
    
public:
    const Matrix2D& getLocalMatrix() const {
        if(localTransformDirty) {
            cachedLocalMatrix = Matrix2D::trs(localPosition, localRotation, localScale);
            localTransformDirty = false;
        }
        return cachedLocalMatrix;
    }
    
    const Matrix2D& getWorldMatrix() const {
        if(worldTransformDirty) {
            GameObject* parent = owner->getParent();
            if(parent && parent->getComponent<Transform>()) {
                cachedWorldMatrix = parent->getComponent<Transform>()->getWorldMatrix() * getLocalMatrix();
            } else {
                cachedWorldMatrix = getLocalMatrix();
            }
            worldTransformDirty = false;
        }
        return cachedWorldMatrix;
    }
    
private:
    void markDirty() {
        localTransformDirty = true;
        markWorldTransformDirty();
    }
};
```

### Batch Transform Updates
```cpp
class TransformSystem {
private:
    std::vector<Transform*> dirtyTransforms;
    
public:
    void markTransformDirty(Transform* transform) {
        if(std::find(dirtyTransforms.begin(), dirtyTransforms.end(), transform) == dirtyTransforms.end()) {
            dirtyTransforms.push_back(transform);
        }
    }
    
    void updateDirtyTransforms() {
        // Sort by hierarchy depth to ensure parents update before children
        std::sort(dirtyTransforms.begin(), dirtyTransforms.end(), 
                 [](Transform* a, Transform* b) {
                     return getHierarchyDepth(a) < getHierarchyDepth(b);
                 });
        
        for(Transform* transform : dirtyTransforms) {
            transform->updateWorldTransform();
        }
        
        dirtyTransforms.clear();
    }
    
private:
    int getHierarchyDepth(Transform* transform) {
        int depth = 0;
        GameObject* current = transform->getOwner()->getParent();
        while(current) {
            depth++;
            current = current->getParent();
        }
        return depth;
    }
};
```

## Common Coordinate System Gotchas

### Screen vs World Coordinates
```cpp
class InputHandler {
public:
    Vec2 getMouseWorldPosition(const Vec2& mouseScreenPos, const Camera& camera) {
        // Common mistake: forgetting to account for camera transform
        // Wrong: return mouseScreenPos;
        
        // Correct: transform screen coordinates to world coordinates
        Vec2 cameraPos = camera.getPosition();
        float cameraZoom = camera.getZoom();
        Vec2 screenCenter = Vec2(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
        
        Vec2 relativeToCenter = mouseScreenPos - screenCenter;
        Vec2 worldOffset = relativeToCenter / cameraZoom;
        
        return cameraPos + worldOffset;
    }
};
```

### Rotation Order and Gimbal Lock
```cpp
class Transform : public Component {
private:
    float rotationRadians = 0.0f;
    
public:
    // Store rotation as single angle for 2D to avoid gimbal lock
    void setRotationDegrees(float degrees) {
        rotationRadians = degrees * (M_PI / 180.0f);
        markDirty();
    }
    
    float getRotationDegrees() const {
        return rotationRadians * (180.0f / M_PI);
    }
    
    // Normalize angle to [-π, π] range
    void normalizeRotation() {
        while(rotationRadians > M_PI) rotationRadians -= 2 * M_PI;
        while(rotationRadians < -M_PI) rotationRadians += 2 * M_PI;
    }
    
    // Rotate by shortest path to target
    void rotateTowards(float targetRadians, float maxRadians) {
        float diff = targetRadians - rotationRadians;
        
        // Find shortest rotation direction
        if(diff > M_PI) diff -= 2 * M_PI;
        if(diff < -M_PI) diff += 2 * M_PI;
        
        float rotation = std::min(maxRadians, std::abs(diff));
        if(diff < 0) rotation = -rotation;
        
        rotationRadians += rotation;
        normalizeRotation();
        markDirty();
    }
};
```

### Scale and Hierarchy Issues
```cpp
class Transform : public Component {
public:
    Vec2 getLocalScale() const { return localScale; }
    
    void setLocalScale(const Vec2& scale) {
        // Prevent zero or negative scale to avoid matrix singularity
        localScale.x = std::max(0.001f, std::abs(scale.x));
        localScale.y = std::max(0.001f, std::abs(scale.y));
        markDirty();
    }
    
    Vec2 getLossyScale() const {
        // Lossy scale accounts for parent scaling
        updateWorldTransform();
        return cachedWorldScale;
    }
    
    bool hasUniformScale() const {
        const float epsilon = 0.001f;
        return std::abs(localScale.x - localScale.y) < epsilon;
    }
};
```

## Testing Strategies

### Unit Tests for Vector Math
```cpp
#include <gtest/gtest.h>

class Vec2Test : public ::testing::Test {
protected:
    const float EPSILON = 0.001f;
    
    void expectVecEqual(const Vec2& a, const Vec2& b) {
        EXPECT_NEAR(a.x, b.x, EPSILON);
        EXPECT_NEAR(a.y, b.y, EPSILON);
    }
};

TEST_F(Vec2Test, BasicArithmetic) {
    Vec2 a(1, 2);
    Vec2 b(3, 4);
    
    expectVecEqual(a + b, Vec2(4, 6));
    expectVecEqual(a - b, Vec2(-2, -2));
    expectVecEqual(a * 2, Vec2(2, 4));
}

TEST_F(Vec2Test, VectorOperations) {
    Vec2 a(3, 4);
    
    EXPECT_NEAR(a.magnitude(), 5.0f, EPSILON);
    EXPECT_NEAR(a.magnitudeSquared(), 25.0f, EPSILON);
    
    Vec2 normalized = a.normalized();
    EXPECT_NEAR(normalized.magnitude(), 1.0f, EPSILON);
}

TEST_F(Vec2Test, DotAndCrossProduct) {
    Vec2 a(1, 0);
    Vec2 b(0, 1);
    
    EXPECT_NEAR(a.dot(b), 0.0f, EPSILON);
    EXPECT_NEAR(a.cross(b), 1.0f, EPSILON);
}
```

### Transform Hierarchy Tests
```cpp
TEST(TransformTest, HierarchyTransformation) {
    GameObject parent("Parent");
    GameObject child("Child");
    
    auto parentTransform = parent.addComponent<Transform>();
    auto childTransform = child.addComponent<Transform>();
    
    parentTransform->setLocalPosition(Vec2(10, 20));
    parentTransform->setLocalRotation(M_PI / 4); // 45 degrees
    
    child.setParent(&parent);
    childTransform->setLocalPosition(Vec2(5, 0));
    
    // Child's world position should be rotated and translated
    Vec2 expectedWorldPos = Vec2(10, 20) + Vec2(5, 0).rotate(M_PI / 4);
    expectVecEqual(childTransform->getWorldPosition(), expectedWorldPos);
}
```

### Performance Benchmarks
```cpp
#include <chrono>

class TransformBenchmark {
public:
    void benchmarkMatrixMultiplication(int iterations) {
        auto start = std::chrono::high_resolution_clock::now();
        
        Matrix2D m1 = Matrix2D::rotation(0.1f);
        Matrix2D m2 = Matrix2D::scale(Vec2(1.1f, 1.1f));
        
        for(int i = 0; i < iterations; ++i) {
            Matrix2D result = m1 * m2;
            // Prevent optimization
            volatile float dummy = result(0, 0);
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        std::cout << "Matrix multiplication: " << duration.count() << " microseconds for " 
                  << iterations << " iterations\n";
    }
};
```

## Integration Examples

### Movement System Integration
```cpp
class MovementComponent : public Component {
private:
    Vec2 velocity = Vec2::zero();
    float acceleration = 100.0f;
    float maxSpeed = 50.0f;
    
public:
    void update(double deltaTime) override {
        Transform* transform = owner->getComponent<Transform>();
        if(!transform) return;
        
        // Apply movement in local space
        Vec2 movement = velocity * static_cast<float>(deltaTime);
        transform->translate(movement);
        
        // Apply friction
        velocity *= 0.95f;
        
        // Clamp to max speed
        if(velocity.magnitude() > maxSpeed) {
            velocity = velocity.normalized() * maxSpeed;
        }
    }
    
    void addForce(const Vec2& force) {
        velocity += force;
    }
    
    void moveInDirection(const Vec2& direction, float speed) {
        velocity = direction.normalized() * speed;
    }
};
```

### Camera System Integration
```cpp
class Camera {
private:
    Transform* transform;
    float zoom = 1.0f;
    Vec2 viewportSize = Vec2(800, 600);
    
public:
    Camera(Transform* cameraTransform) : transform(cameraTransform) {}
    
    Matrix2D getViewMatrix() const {
        if(!transform) return Matrix2D::identity();
        
        // Camera view matrix is inverse of camera transform
        return transform->getWorldToLocalMatrix();
    }
    
    Matrix2D getProjectionMatrix() const {
        // Simple orthographic projection with zoom
        return Matrix2D::scale(Vec2(2.0f / viewportSize.x * zoom, 
                                   2.0f / viewportSize.y * zoom));
    }
    
    Matrix2D getViewProjectionMatrix() const {
        return getProjectionMatrix() * getViewMatrix();
    }
    
    Vec2 worldToScreen(const Vec2& worldPos) const {
        Vec2 viewPos = getViewMatrix().transformPoint(worldPos);
        Vec2 projPos = getProjectionMatrix().transformPoint(viewPos);
        
        // Convert from [-1,1] to screen coordinates
        return Vec2(
            (projPos.x + 1) * viewportSize.x * 0.5f,
            (1 - projPos.y) * viewportSize.y * 0.5f
        );
    }
    
    Vec2 screenToWorld(const Vec2& screenPos) const {
        // Convert screen coordinates to [-1,1] range
        Vec2 normalized = Vec2(
            (screenPos.x / viewportSize.x) * 2 - 1,
            1 - (screenPos.y / viewportSize.y) * 2
        );
        
        // Apply inverse transformations
        Matrix2D invProjection = getProjectionMatrix(); // Simplified inverse
        invProjection(0, 0) = 1.0f / invProjection(0, 0);
        invProjection(1, 1) = 1.0f / invProjection(1, 1);
        
        Vec2 viewPos = invProjection.transformPoint(normalized);
        return transform->getLocalToWorldMatrix().transformPoint(viewPos);
    }
};
```

### Physics Integration
```cpp
class PhysicsBody : public Component {
private:
    Vec2 velocity = Vec2::zero();
    Vec2 acceleration = Vec2::zero();
    float mass = 1.0f;
    float drag = 0.98f;
    
public:
    void update(double deltaTime) override {
        Transform* transform = owner->getComponent<Transform>();
        if(!transform) return;
        
        // Physics integration using Verlet integration
        Vec2 newAcceleration = getTotalForces() / mass;
        Vec2 newVelocity = velocity + (acceleration + newAcceleration) * 0.5f * deltaTime;
        Vec2 newPosition = transform->getLocalPosition() + velocity * deltaTime + 
                          acceleration * 0.5f * deltaTime * deltaTime;
        
        // Update transform
        transform->setLocalPosition(newPosition);
        
        // Update physics state
        velocity = newVelocity * drag;
        acceleration = newAcceleration;
    }
    
    void addForce(const Vec2& force) {
        acceleration += force / mass;
    }
    
    Vec2 getTotalForces() const {
        Vec2 totalForce = Vec2::zero();
        
        // Gravity
        totalForce += Vec2(0, -9.81f) * mass;
        
        // Add other forces as needed
        
        return totalForce;
    }
};
```

## Resources
- [Linear Algebra for Game Developers](https://www.gamedev.net/tutorials/_/technical/math-and-physics/a-simple-and-practical-approach-to-interpolation-r5030/)
- [C++ Operator Overloading Best Practices](https://en.cppreference.com/w/cpp/language/operators)
- [Floating Point Math](https://floating-point-gui.de/)
- [2D Game Math](https://www.redblobgames.com/articles/vector-math/)

## Fun Challenges
- Implement smooth interpolation between transforms
- Add support for 3D transformations
- Create a visual transform gizmo system
- Implement constraint systems (distance, angle constraints)
- Add spline-based movement paths

## Debugging Tips
- Visualize transform hierarchies with debug drawing
- Add transform inspector showing local/world values
- Implement transform animation recording/playback
- Create coordinate system visualization tools
- Add matrix decomposition debugging utilities

## Common Gotchas
- **Floating point precision**: Use epsilon comparisons for equality
- **Matrix order**: Remember matrix multiplication is not commutative
- **Rotation wrapping**: Keep angles in [-π, π] range
- **Scale singularities**: Avoid zero or near-zero scale values
- **Hierarchy depth**: Deep hierarchies can accumulate floating point errors

## Refactoring Process

### Step 1: Vector Math Foundation
1. Implement Vec2 class with basic operations
2. Add operator overloading for intuitive usage
3. Test mathematical operations thoroughly

### Step 2: Basic Transform Component
1. Create Transform component with position, rotation, scale
2. Implement local space getters and setters
3. Add basic transformation methods

### Step 3: Add World Space Support
1. Implement world space coordinate calculations
2. Add parent-child transform hierarchy support
3. Create local-to-world and world-to-local conversions

### Step 4: Performance Optimization
1. Add transform caching with dirty flags
2. Implement batch transform update system
3. Profile and optimize transformation calculations

### Step 5: Matrix Integration
1. Implement Matrix2D class for advanced operations
2. Add matrix-based transformation methods
3. Integrate with rendering and physics systems