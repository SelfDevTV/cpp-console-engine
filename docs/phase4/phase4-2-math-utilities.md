# Phase 4.2: Math Utilities Design

## C++ Learning Focus
- **Operator overloading**: Implementing arithmetic operators for custom types
- **Template programming**: Generic math functions and specializations
- **Namespace organization**: Logical grouping of mathematical utilities
- **Mathematical precision**: Floating-point considerations and epsilon comparisons
- **Const correctness**: Immutable mathematical operations
- **Header-only libraries**: Inline functions and templates

## Implementation Overview
Build a comprehensive math utilities system to support game mathematics:
1. Vector2 class with full operator support
2. Collision detection algorithms
3. Interpolation and easing functions
4. Utility math functions (clamp, lerp, distance)
5. Random number generation utilities
6. Mathematical constants and helper functions

## Math Utilities Structure

### Core Components
```cpp
namespace Math {
    // Vector mathematics
    class Vector2;
    
    // Collision detection
    namespace Collision {
        bool pointInRect(const Vector2& point, const Vector2& rectPos, const Vector2& rectSize);
        bool rectIntersect(const Vector2& pos1, const Vector2& size1, const Vector2& pos2, const Vector2& size2);
    }
    
    // Interpolation functions
    namespace Interpolation {
        double linear(double t);
        double easeInQuad(double t);
        double easeOutQuad(double t);
        double easeInOutQuad(double t);
    }
    
    // Utility functions
    template<typename T>
    T clamp(T value, T min, T max);
    
    template<typename T>
    T lerp(T a, T b, double t);
    
    // Constants
    constexpr double PI = 3.14159265358979323846;
    constexpr double TAU = 2.0 * PI;
    constexpr double EPSILON = 1e-9;
}
```

## Key C++ Concepts to Learn

### Operator Overloading Best Practices
```cpp
class Vector2 {
private:
    double x, y;
    
public:
    Vector2(double x = 0.0, double y = 0.0) : x(x), y(y) {}
    
    // Arithmetic operators - return new objects
    Vector2 operator+(const Vector2& other) const {
        return Vector2(x + other.x, y + other.y);
    }
    
    Vector2 operator-(const Vector2& other) const {
        return Vector2(x - other.x, y - other.y);
    }
    
    Vector2 operator*(double scalar) const {
        return Vector2(x * scalar, y * scalar);
    }
    
    // Compound assignment operators - modify this object
    Vector2& operator+=(const Vector2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    
    Vector2& operator-=(const Vector2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    
    Vector2& operator*=(double scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }
    
    // Comparison operators
    bool operator==(const Vector2& other) const {
        return std::abs(x - other.x) < Math::EPSILON && 
               std::abs(y - other.y) < Math::EPSILON;
    }
    
    bool operator!=(const Vector2& other) const {
        return !(*this == other);
    }
    
    // Access operators
    double& operator[](int index) {
        return (index == 0) ? x : y;
    }
    
    const double& operator[](int index) const {
        return (index == 0) ? x : y;
    }
};

// Free function operators for commutativity
Vector2 operator*(double scalar, const Vector2& vec) {
    return vec * scalar;
}
```

### Template Programming Patterns
```cpp
namespace Math {
    // Generic clamp function
    template<typename T>
    constexpr T clamp(T value, T min, T max) {
        return (value < min) ? min : (value > max) ? max : value;
    }
    
    // Template specialization for floating point
    template<>
    constexpr float clamp<float>(float value, float min, float max) {
        return std::fmaxf(min, std::fminf(max, value));
    }
    
    template<>
    constexpr double clamp<double>(double value, double min, double max) {
        return std::fmax(min, std::fmin(max, value));
    }
    
    // Generic linear interpolation
    template<typename T>
    constexpr T lerp(T a, T b, double t) {
        return a + static_cast<T>((b - a) * t);
    }
    
    // Variadic template for multiple values
    template<typename T, typename... Args>
    constexpr T min(T first, Args... args) {
        if constexpr (sizeof...(args) == 0) {
            return first;
        } else {
            return std::min(first, min(args...));
        }
    }
}
```

### Namespace Design Principles
```cpp
// math_utilities.h
#ifndef MATH_UTILITIES_H
#define MATH_UTILITIES_H

namespace Math {
    // Core math functions
    constexpr double PI = 3.14159265358979323846;
    constexpr double EPSILON = 1e-9;
    
    // Nested namespaces for organization
    namespace Collision {
        // Collision detection functions
    }
    
    namespace Interpolation {
        // Easing and interpolation functions
    }
    
    namespace Random {
        // Random number generation utilities
    }
}

// Usage:
// Math::PI
// Math::Collision::pointInRect()
// Math::Interpolation::easeInQuad()
```

### Floating Point Considerations
```cpp
namespace Math {
    // Safe floating point comparisons
    constexpr double EPSILON = 1e-9;
    
    bool isEqual(double a, double b, double epsilon = EPSILON) {
        return std::abs(a - b) < epsilon;
    }
    
    bool isZero(double value, double epsilon = EPSILON) {
        return std::abs(value) < epsilon;
    }
    
    // Handle edge cases in division
    double safeDivide(double numerator, double denominator, double defaultValue = 0.0) {
        return isZero(denominator) ? defaultValue : numerator / denominator;
    }
    
    // Normalize angles to [0, 2π) range
    double normalizeAngle(double angle) {
        while (angle < 0.0) angle += TAU;
        while (angle >= TAU) angle -= TAU;
        return angle;
    }
}
```

## Vector2 Implementation

### Complete Vector2 Class
```cpp
class Vector2 {
private:
    double x, y;
    
public:
    // Constructors
    Vector2() : x(0.0), y(0.0) {}
    Vector2(double x, double y) : x(x), y(y) {}
    Vector2(const Vector2& other) = default;
    
    // Accessors
    double getX() const { return x; }
    double getY() const { return y; }
    void setX(double newX) { x = newX; }
    void setY(double newY) { y = newY; }
    void set(double newX, double newY) { x = newX; y = newY; }
    
    // Vector operations
    double magnitude() const {
        return std::sqrt(x * x + y * y);
    }
    
    double magnitudeSquared() const {
        return x * x + y * y;
    }
    
    Vector2 normalized() const {
        double mag = magnitude();
        return Math::isZero(mag) ? Vector2() : Vector2(x / mag, y / mag);
    }
    
    void normalize() {
        double mag = magnitude();
        if (!Math::isZero(mag)) {
            x /= mag;
            y /= mag;
        }
    }
    
    double dot(const Vector2& other) const {
        return x * other.x + y * other.y;
    }
    
    double cross(const Vector2& other) const {
        return x * other.y - y * other.x;
    }
    
    double angle() const {
        return std::atan2(y, x);
    }
    
    double angleTo(const Vector2& other) const {
        return std::atan2(cross(other), dot(other));
    }
    
    double distanceTo(const Vector2& other) const {
        return (*this - other).magnitude();
    }
    
    double distanceSquaredTo(const Vector2& other) const {
        return (*this - other).magnitudeSquared();
    }
    
    Vector2 rotated(double angle) const {
        double cos_a = std::cos(angle);
        double sin_a = std::sin(angle);
        return Vector2(x * cos_a - y * sin_a, x * sin_a + y * cos_a);
    }
    
    Vector2 perpendicular() const {
        return Vector2(-y, x);
    }
    
    // Static utility functions
    static Vector2 fromAngle(double angle, double magnitude = 1.0) {
        return Vector2(std::cos(angle) * magnitude, std::sin(angle) * magnitude);
    }
    
    static Vector2 zero() { return Vector2(0.0, 0.0); }
    static Vector2 one() { return Vector2(1.0, 1.0); }
    static Vector2 up() { return Vector2(0.0, -1.0); }
    static Vector2 down() { return Vector2(0.0, 1.0); }
    static Vector2 left() { return Vector2(-1.0, 0.0); }
    static Vector2 right() { return Vector2(1.0, 0.0); }
};
```

## Collision Detection Algorithms

### Point-in-Shape Tests
```cpp
namespace Math::Collision {
    bool pointInRect(const Vector2& point, const Vector2& rectPos, const Vector2& rectSize) {
        return point.getX() >= rectPos.getX() && 
               point.getX() < rectPos.getX() + rectSize.getX() &&
               point.getY() >= rectPos.getY() && 
               point.getY() < rectPos.getY() + rectSize.getY();
    }
    
    bool pointInCircle(const Vector2& point, const Vector2& center, double radius) {
        return point.distanceSquaredTo(center) <= radius * radius;
    }
    
    bool pointInTriangle(const Vector2& point, const Vector2& a, const Vector2& b, const Vector2& c) {
        // Use barycentric coordinates
        Vector2 v0 = c - a;
        Vector2 v1 = b - a;
        Vector2 v2 = point - a;
        
        double dot00 = v0.dot(v0);
        double dot01 = v0.dot(v1);
        double dot02 = v0.dot(v2);
        double dot11 = v1.dot(v1);
        double dot12 = v1.dot(v2);
        
        double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
        
        return (u >= 0) && (v >= 0) && (u + v < 1);
    }
}
```

### Shape Intersection Tests
```cpp
namespace Math::Collision {
    bool rectIntersect(const Vector2& pos1, const Vector2& size1, 
                      const Vector2& pos2, const Vector2& size2) {
        return !(pos1.getX() >= pos2.getX() + size2.getX() ||
                pos2.getX() >= pos1.getX() + size1.getX() ||
                pos1.getY() >= pos2.getY() + size2.getY() ||
                pos2.getY() >= pos1.getY() + size1.getY());
    }
    
    bool circleIntersect(const Vector2& center1, double radius1,
                        const Vector2& center2, double radius2) {
        double distanceSquared = center1.distanceSquaredTo(center2);
        double radiusSum = radius1 + radius2;
        return distanceSquared <= radiusSum * radiusSum;
    }
    
    bool lineIntersect(const Vector2& line1Start, const Vector2& line1End,
                      const Vector2& line2Start, const Vector2& line2End,
                      Vector2* intersection = nullptr) {
        Vector2 dir1 = line1End - line1Start;
        Vector2 dir2 = line2End - line2Start;
        
        double cross = dir1.cross(dir2);
        if (Math::isZero(cross)) {
            return false; // Lines are parallel
        }
        
        Vector2 diff = line2Start - line1Start;
        double t1 = diff.cross(dir2) / cross;
        double t2 = diff.cross(dir1) / cross;
        
        bool intersects = (t1 >= 0.0 && t1 <= 1.0 && t2 >= 0.0 && t2 <= 1.0);
        
        if (intersects && intersection) {
            *intersection = line1Start + dir1 * t1;
        }
        
        return intersects;
    }
}
```

## Interpolation and Easing Functions

### Linear and Polynomial Easing
```cpp
namespace Math::Interpolation {
    // Basic interpolation
    double linear(double t) {
        return Math::clamp(t, 0.0, 1.0);
    }
    
    // Quadratic easing
    double easeInQuad(double t) {
        t = Math::clamp(t, 0.0, 1.0);
        return t * t;
    }
    
    double easeOutQuad(double t) {
        t = Math::clamp(t, 0.0, 1.0);
        return 1.0 - (1.0 - t) * (1.0 - t);
    }
    
    double easeInOutQuad(double t) {
        t = Math::clamp(t, 0.0, 1.0);
        return t < 0.5 ? 2.0 * t * t : 1.0 - 2.0 * (1.0 - t) * (1.0 - t);
    }
    
    // Cubic easing
    double easeInCubic(double t) {
        t = Math::clamp(t, 0.0, 1.0);
        return t * t * t;
    }
    
    double easeOutCubic(double t) {
        t = Math::clamp(t, 0.0, 1.0);
        double f = 1.0 - t;
        return 1.0 - f * f * f;
    }
    
    // Sine easing
    double easeInSine(double t) {
        t = Math::clamp(t, 0.0, 1.0);
        return 1.0 - std::cos(t * Math::PI * 0.5);
    }
    
    double easeOutSine(double t) {
        t = Math::clamp(t, 0.0, 1.0);
        return std::sin(t * Math::PI * 0.5);
    }
}
```

### Advanced Interpolation
```cpp
namespace Math::Interpolation {
    // Smooth step interpolation
    double smoothStep(double t) {
        t = Math::clamp(t, 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    }
    
    double smootherStep(double t) {
        t = Math::clamp(t, 0.0, 1.0);
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    }
    
    // Bounce easing
    double easeOutBounce(double t) {
        t = Math::clamp(t, 0.0, 1.0);
        if (t < 1.0 / 2.75) {
            return 7.5625 * t * t;
        } else if (t < 2.0 / 2.75) {
            t -= 1.5 / 2.75;
            return 7.5625 * t * t + 0.75;
        } else if (t < 2.5 / 2.75) {
            t -= 2.25 / 2.75;
            return 7.5625 * t * t + 0.9375;
        } else {
            t -= 2.625 / 2.75;
            return 7.5625 * t * t + 0.984375;
        }
    }
    
    // Elastic easing
    double easeOutElastic(double t) {
        t = Math::clamp(t, 0.0, 1.0);
        if (Math::isZero(t) || Math::isEqual(t, 1.0)) {
            return t;
        }
        
        constexpr double p = 0.3;
        constexpr double s = p / 4.0;
        return std::pow(2.0, -10.0 * t) * std::sin((t - s) * Math::TAU / p) + 1.0;
    }
}
```

## Random Number Generation

### Modern C++ Random Utilities
```cpp
namespace Math::Random {
    // Thread-local random engine
    thread_local std::mt19937 generator(std::random_device{}());
    
    // Seed the generator
    void seed(unsigned int seedValue) {
        generator.seed(seedValue);
    }
    
    // Random integers
    int range(int min, int max) {
        std::uniform_int_distribution<int> distribution(min, max);
        return distribution(generator);
    }
    
    // Random floating point
    double range(double min, double max) {
        std::uniform_real_distribution<double> distribution(min, max);
        return distribution(generator);
    }
    
    // Random boolean with probability
    bool chance(double probability) {
        std::bernoulli_distribution distribution(probability);
        return distribution(generator);
    }
    
    // Random from list
    template<typename T>
    const T& choice(const std::vector<T>& items) {
        if (items.empty()) {
            throw std::invalid_argument("Cannot choose from empty container");
        }
        std::uniform_int_distribution<size_t> distribution(0, items.size() - 1);
        return items[distribution(generator)];
    }
    
    // Random Vector2
    Vector2 vector2(double minX, double maxX, double minY, double maxY) {
        return Vector2(range(minX, maxX), range(minY, maxY));
    }
    
    Vector2 unitVector2() {
        double angle = range(0.0, Math::TAU);
        return Vector2::fromAngle(angle);
    }
    
    Vector2 insideCircle(double radius) {
        double angle = range(0.0, Math::TAU);
        double r = std::sqrt(range(0.0, 1.0)) * radius;
        return Vector2::fromAngle(angle, r);
    }
}
```

## Mathematical Constants and Utilities

### Constants and Helper Functions
```cpp
namespace Math {
    // Mathematical constants
    constexpr double PI = 3.14159265358979323846;
    constexpr double TAU = 2.0 * PI;
    constexpr double E = 2.71828182845904523536;
    constexpr double SQRT_2 = 1.41421356237309504880;
    constexpr double SQRT_3 = 1.73205080756887729353;
    constexpr double GOLDEN_RATIO = 1.61803398874989484820;
    constexpr double EPSILON = 1e-9;
    
    // Angle conversions
    constexpr double DEG_TO_RAD = PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / PI;
    
    double toRadians(double degrees) {
        return degrees * DEG_TO_RAD;
    }
    
    double toDegrees(double radians) {
        return radians * RAD_TO_DEG;
    }
    
    // Utility functions
    template<typename T>
    T sign(T value) {
        return (value > T{0}) - (value < T{0});
    }
    
    template<typename T>
    T abs(T value) {
        return value < T{0} ? -value : value;
    }
    
    template<typename T>
    T square(T value) {
        return value * value;
    }
    
    template<typename T>
    T cube(T value) {
        return value * value * value;
    }
    
    // Fast inverse square root (Quake algorithm)
    float fastInverseSqrt(float number) {
        const float threehalfs = 1.5f;
        float x2 = number * 0.5f;
        float y = number;
        
        // Evil floating point bit level hacking
        long i = *(long*)&y;
        i = 0x5f3759df - (i >> 1);
        y = *(float*)&i;
        
        // Newton's method iteration
        y = y * (threehalfs - (x2 * y * y));
        y = y * (threehalfs - (x2 * y * y)); // Optional second iteration
        
        return y;
    }
}
```

## Performance Optimization

### Optimized Mathematical Operations
```cpp
namespace Math {
    // Lookup table for trigonometric functions
    class TrigLookup {
    private:
        static constexpr size_t TABLE_SIZE = 1024;
        static constexpr double SCALE = TABLE_SIZE / TAU;
        
        std::array<double, TABLE_SIZE> sinTable;
        std::array<double, TABLE_SIZE> cosTable;
        
    public:
        TrigLookup() {
            for (size_t i = 0; i < TABLE_SIZE; ++i) {
                double angle = (static_cast<double>(i) / TABLE_SIZE) * TAU;
                sinTable[i] = std::sin(angle);
                cosTable[i] = std::cos(angle);
            }
        }
        
        double sin(double angle) const {
            angle = normalizeAngle(angle);
            size_t index = static_cast<size_t>(angle * SCALE) % TABLE_SIZE;
            return sinTable[index];
        }
        
        double cos(double angle) const {
            angle = normalizeAngle(angle);
            size_t index = static_cast<size_t>(angle * SCALE) % TABLE_SIZE;
            return cosTable[index];
        }
    };
    
    // Global lookup table instance
    inline const TrigLookup& getTrigLookup() {
        static TrigLookup instance;
        return instance;
    }
    
    // Fast approximation functions
    double fastSin(double x) {
        return getTrigLookup().sin(x);
    }
    
    double fastCos(double x) {
        return getTrigLookup().cos(x);
    }
    
    // Integer power function
    template<typename T>
    constexpr T ipow(T base, unsigned int exp) {
        T result = T{1};
        while (exp > 0) {
            if (exp & 1) result *= base;
            base *= base;
            exp >>= 1;
        }
        return result;
    }
}
```

### Memory Pool for Vector Operations
```cpp
namespace Math {
    class Vector2Pool {
    private:
        std::vector<Vector2> pool;
        size_t nextIndex = 0;
        
    public:
        Vector2Pool(size_t initialSize = 1000) : pool(initialSize) {}
        
        Vector2* allocate() {
            if (nextIndex >= pool.size()) {
                pool.resize(pool.size() * 2);
            }
            return &pool[nextIndex++];
        }
        
        void reset() {
            nextIndex = 0;
        }
        
        size_t getUsedCount() const { return nextIndex; }
        size_t getTotalCount() const { return pool.size(); }
    };
    
    // Global pool instance
    inline Vector2Pool& getVector2Pool() {
        static Vector2Pool instance;
        return instance;
    }
}
```

## Testing Strategies

### Unit Testing Math Functions
```cpp
#include <gtest/gtest.h>
#include "math_utilities.h"

class MathUtilitiesTest : public ::testing::Test {
protected:
    const double EPSILON = 1e-9;
    
    void assertNearEqual(double a, double b, double epsilon = EPSILON) {
        ASSERT_NEAR(a, b, epsilon);
    }
    
    void assertVectorNearEqual(const Vector2& a, const Vector2& b, double epsilon = EPSILON) {
        ASSERT_NEAR(a.getX(), b.getX(), epsilon);
        ASSERT_NEAR(a.getY(), b.getY(), epsilon);
    }
};

TEST_F(MathUtilitiesTest, Vector2BasicOperations) {
    Vector2 v1(3.0, 4.0);
    Vector2 v2(1.0, 2.0);
    
    // Addition
    Vector2 sum = v1 + v2;
    assertVectorNearEqual(sum, Vector2(4.0, 6.0));
    
    // Magnitude
    assertNearEqual(v1.magnitude(), 5.0);
    
    // Normalization
    Vector2 normalized = v1.normalized();
    assertNearEqual(normalized.magnitude(), 1.0);
}

TEST_F(MathUtilitiesTest, InterpolationFunctions) {
    // Linear interpolation
    ASSERT_NEAR(Math::lerp(0.0, 10.0, 0.5), 5.0, EPSILON);
    ASSERT_NEAR(Math::lerp(0.0, 10.0, 0.0), 0.0, EPSILON);
    ASSERT_NEAR(Math::lerp(0.0, 10.0, 1.0), 10.0, EPSILON);
    
    // Easing functions
    ASSERT_NEAR(Math::Interpolation::easeInQuad(0.0), 0.0, EPSILON);
    ASSERT_NEAR(Math::Interpolation::easeInQuad(1.0), 1.0, EPSILON);
    ASSERT_NEAR(Math::Interpolation::easeInQuad(0.5), 0.25, EPSILON);
}

TEST_F(MathUtilitiesTest, CollisionDetection) {
    // Point in rectangle
    Vector2 point(5.0, 5.0);
    Vector2 rectPos(0.0, 0.0);
    Vector2 rectSize(10.0, 10.0);
    
    ASSERT_TRUE(Math::Collision::pointInRect(point, rectPos, rectSize));
    
    Vector2 outsidePoint(15.0, 5.0);
    ASSERT_FALSE(Math::Collision::pointInRect(outsidePoint, rectPos, rectSize));
}

TEST_F(MathUtilitiesTest, RandomGeneration) {
    // Test range function produces values in bounds
    for (int i = 0; i < 100; ++i) {
        int value = Math::Random::range(1, 10);
        ASSERT_GE(value, 1);
        ASSERT_LE(value, 10);
    }
    
    // Test floating point range
    for (int i = 0; i < 100; ++i) {
        double value = Math::Random::range(0.0, 1.0);
        ASSERT_GE(value, 0.0);
        ASSERT_LT(value, 1.0);
    }
}
```

### Performance Testing
```cpp
TEST_F(MathUtilitiesTest, PerformanceBenchmarks) {
    const int ITERATIONS = 1000000;
    
    // Benchmark vector operations
    Vector2 v1(1.0, 2.0);
    Vector2 v2(3.0, 4.0);
    
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < ITERATIONS; ++i) {
        Vector2 result = v1 + v2;
        result = result * 2.0;
        double mag = result.magnitude();
        (void)mag; // Prevent optimization
    }
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Vector operations: " << duration.count() << " microseconds" << std::endl;
}
```

## Integration Examples

### Using Math Utilities in Game Objects
```cpp
class Player : public GameObject {
private:
    Vector2 velocity;
    double maxSpeed = 100.0;
    Vector2 targetPosition;
    
public:
    void update(double deltaTime) override {
        // Move towards target with easing
        Vector2 currentPos = getPosition();
        Vector2 direction = (targetPosition - currentPos).normalized();
        
        // Apply easing to movement
        double distance = currentPos.distanceTo(targetPosition);
        double t = Math::clamp(distance / 100.0, 0.0, 1.0);
        double easeValue = Math::Interpolation::easeOutQuad(t);
        
        velocity = direction * maxSpeed * easeValue;
        
        // Update position
        Vector2 newPosition = currentPos + velocity * deltaTime;
        setPosition(newPosition);
        
        // Check bounds
        Vector2 screenSize(80, 25);
        Vector2 clampedPos(
            Math::clamp(newPosition.getX(), 0.0, screenSize.getX() - 1.0),
            Math::clamp(newPosition.getY(), 0.0, screenSize.getY() - 1.0)
        );
        setPosition(clampedPos);
    }
    
    void setTarget(const Vector2& target) {
        targetPosition = target;
    }
};
```

### Collision System Integration
```cpp
class CollisionSystem {
private:
    std::vector<GameObject*> collidableObjects;
    
public:
    void checkCollisions() {
        for (size_t i = 0; i < collidableObjects.size(); ++i) {
            for (size_t j = i + 1; j < collidableObjects.size(); ++j) {
                GameObject* obj1 = collidableObjects[i];
                GameObject* obj2 = collidableObjects[j];
                
                if (Math::Collision::rectIntersect(
                    obj1->getPosition(), obj1->getSize(),
                    obj2->getPosition(), obj2->getSize())) {
                    
                    handleCollision(obj1, obj2);
                }
            }
        }
    }
    
private:
    void handleCollision(GameObject* obj1, GameObject* obj2) {
        // Calculate collision response
        Vector2 center1 = obj1->getPosition() + obj1->getSize() * 0.5;
        Vector2 center2 = obj2->getPosition() + obj2->getSize() * 0.5;
        
        Vector2 separationDirection = (center2 - center1).normalized();
        
        // Apply separation
        double separationDistance = 1.0; // Minimum separation
        Vector2 separation = separationDirection * separationDistance * 0.5;
        
        obj1->setPosition(obj1->getPosition() - separation);
        obj2->setPosition(obj2->getPosition() + separation);
    }
};
```

## Resources
- [C++ Operator Overloading Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-overload)
- [Template Programming Best Practices](https://en.cppreference.com/w/cpp/language/templates)
- [Floating Point Arithmetic](https://docs.oracle.com/cd/E19957-01/806-3568/ncg_goldberg.html)
- [Game Mathematics](https://gamemath.com/)
- [Real-Time Collision Detection](https://realtimecollisiondetection.net/)
- [Easing Functions Reference](https://easings.net/)

## Fun Challenges
- Implement 3D vector math (Vector3 class)
- Add matrix math for transformations
- Create a spatial partitioning system (quadtree)
- Implement Bezier curve calculations
- Add noise generation functions (Perlin, Simplex)
- Create a physics constraint solver
- Implement advanced collision shapes (oriented bounding boxes, capsules)
- Add spline interpolation support

## Debugging Tips
- Use assertions to validate mathematical constraints
- Implement debug visualization for vectors and shapes
- Add logging for mathematical operations during development
- Create unit tests with known mathematical results
- Use epsilon comparisons for floating-point equality
- Implement debug modes that show collision boundaries
- Add performance profiling for math-heavy operations

## Refactoring Process

### Step 1: Create Math Namespace
1. Create `math_utilities.h` with basic namespace structure
2. Define mathematical constants and epsilon values
3. Add basic utility functions (clamp, lerp, etc.)
4. Test compilation and basic functionality

### Step 2: Implement Vector2 Class
1. Create Vector2 with basic constructor and accessors
2. Add arithmetic operators (start with +, -, *)
3. Implement magnitude and normalization functions
4. Add dot product and cross product operations
5. Test all vector operations thoroughly

### Step 3: Add Collision Detection
1. Create Collision namespace within Math
2. Implement point-in-shape tests
3. Add shape intersection algorithms
4. Create comprehensive collision test suite

### Step 4: Implement Interpolation Functions
1. Create Interpolation namespace
2. Add basic linear interpolation
3. Implement easing functions (quadratic, cubic, sine)
4. Add advanced easing (bounce, elastic, back)

### Step 5: Add Random Utilities
1. Create Random namespace with modern C++ generators
2. Implement range functions for integers and floats
3. Add specialized random functions (unit vectors, etc.)
4. Create utilities for random choices and probabilities

### Step 6: Performance Optimization
1. Profile mathematical operations
2. Add lookup tables for expensive functions
3. Implement fast approximation algorithms where appropriate
4. Create memory pools for temporary mathematical objects

### Step 7: Integration and Testing
1. Integrate math utilities into existing game objects
2. Create comprehensive unit test suite
3. Add performance benchmarks
4. Write usage examples and documentation