# Phase 6.1: Memory Management Optimization

## C++ Learning Focus
- **Smart Pointers**: Understanding `unique_ptr`, `shared_ptr`, and `weak_ptr`
- **RAII Principles**: Resource Acquisition Is Initialization for memory safety
- **Object Pooling**: Reducing allocation overhead with reusable object pools
- **Memory Profiling**: Tools and techniques for identifying memory issues
- **Custom Allocators**: Implementing specialized memory allocation strategies
- **Memory Debugging**: Detecting leaks, corruption, and performance bottlenecks

## Implementation Overview
Transform your engine's memory usage for optimal performance:
1. Replace raw pointers with smart pointers throughout the codebase
2. Implement object pooling for frequently created/destroyed objects
3. Add custom allocators for specific use cases
4. Integrate memory profiling and debugging tools
5. Establish memory leak prevention strategies
6. Optimize memory layout for cache efficiency

## Memory Allocation Strategies

### Smart Pointer Migration
```cpp
// Before: Raw pointer management
class GameEngine {
private:
    Renderer* renderer;
    InputManager* input;
    Scene* currentScene;
    
public:
    GameEngine() {
        renderer = new Renderer();
        input = new InputManager();
        currentScene = nullptr;
    }
    
    ~GameEngine() {
        delete renderer;
        delete input;
        delete currentScene;
    }
};

// After: Smart pointer management
class GameEngine {
private:
    std::unique_ptr<Renderer> renderer;
    std::unique_ptr<InputManager> input;
    std::shared_ptr<Scene> currentScene;
    
public:
    GameEngine() 
        : renderer(std::make_unique<Renderer>())
        , input(std::make_unique<InputManager>())
        , currentScene(nullptr) {
    }
    
    // Destructor automatically handles cleanup
    ~GameEngine() = default;
};
```

### Object Lifetime Management
```cpp
class ResourceManager {
private:
    std::unordered_map<std::string, std::shared_ptr<Texture>> textures;
    std::unordered_map<std::string, std::weak_ptr<Texture>> textureCache;
    
public:
    std::shared_ptr<Texture> getTexture(const std::string& path) {
        // Check if texture is still alive in cache
        if (auto cached = textureCache[path].lock()) {
            return cached;
        }
        
        // Load new texture
        auto texture = std::make_shared<Texture>(path);
        textures[path] = texture;
        textureCache[path] = texture;
        return texture;
    }
    
    void cleanup() {
        // Remove expired weak references
        for (auto it = textureCache.begin(); it != textureCache.end();) {
            if (it->second.expired()) {
                it = textureCache.erase(it);
            } else {
                ++it;
            }
        }
    }
};
```

### Pool Allocation Implementation
```cpp
template<typename T, size_t PoolSize = 100>
class ObjectPool {
private:
    std::array<T, PoolSize> pool;
    std::queue<T*> available;
    std::bitset<PoolSize> used;
    
public:
    ObjectPool() {
        for (size_t i = 0; i < PoolSize; ++i) {
            available.push(&pool[i]);
        }
    }
    
    T* acquire() {
        if (available.empty()) {
            return nullptr; // Pool exhausted
        }
        
        T* obj = available.front();
        available.pop();
        
        size_t index = obj - &pool[0];
        used[index] = true;
        
        return obj;
    }
    
    void release(T* obj) {
        if (!obj) return;
        
        size_t index = obj - &pool[0];
        if (index >= PoolSize || !used[index]) {
            return; // Invalid object or already released
        }
        
        obj->reset(); // Custom reset method
        used[index] = false;
        available.push(obj);
    }
    
    size_t available_count() const {
        return available.size();
    }
    
    size_t used_count() const {
        return used.count();
    }
};

// Usage example for game entities
class GameObject {
public:
    void reset() {
        position = {0, 0};
        velocity = {0, 0};
        active = false;
        components.clear();
    }
    
private:
    Vector2 position;
    Vector2 velocity;
    bool active;
    std::vector<std::unique_ptr<Component>> components;
};

class EntityManager {
private:
    ObjectPool<GameObject, 1000> gameObjectPool;
    
public:
    GameObject* createGameObject() {
        auto* obj = gameObjectPool.acquire();
        if (obj) {
            obj->reset();
            return obj;
        }
        return nullptr; // Pool exhausted
    }
    
    void destroyGameObject(GameObject* obj) {
        gameObjectPool.release(obj);
    }
};
```

## Key C++ Concepts to Learn

### Smart Pointer Best Practices
```cpp
// Prefer make_unique/make_shared over new
auto renderer = std::make_unique<Renderer>();
auto scene = std::make_shared<Scene>();

// Use unique_ptr for exclusive ownership
class Engine {
private:
    std::unique_ptr<Renderer> renderer; // Engine owns renderer
    std::unique_ptr<AudioManager> audio; // Engine owns audio
};

// Use shared_ptr for shared ownership
class Scene {
private:
    std::vector<std::shared_ptr<GameObject>> objects; // Multiple containers can share objects
};

// Use weak_ptr to break circular references
class GameObject {
private:
    std::weak_ptr<Scene> parentScene; // Avoids circular reference
    std::vector<std::shared_ptr<Component>> components;
};

class Component {
private:
    std::weak_ptr<GameObject> owner; // Avoids circular reference
};
```

### RAII Memory Patterns
```cpp
class BufferManager {
private:
    struct Buffer {
        std::unique_ptr<char[]> data;
        size_t size;
        
        Buffer(size_t s) : data(std::make_unique<char[]>(s)), size(s) {}
        
        // Move constructor for efficiency
        Buffer(Buffer&& other) noexcept 
            : data(std::move(other.data)), size(other.size) {
            other.size = 0;
        }
        
        // Disable copy to prevent accidental duplication
        Buffer(const Buffer&) = delete;
        Buffer& operator=(const Buffer&) = delete;
    };
    
    std::vector<Buffer> buffers;
    
public:
    size_t allocateBuffer(size_t size) {
        buffers.emplace_back(size);
        return buffers.size() - 1;
    }
    
    char* getBuffer(size_t index) {
        return (index < buffers.size()) ? buffers[index].data.get() : nullptr;
    }
    
    // Automatic cleanup when BufferManager is destroyed
};
```

### Custom Allocators
```cpp
// Stack allocator for temporary allocations
template<size_t Size>
class StackAllocator {
private:
    std::array<char, Size> memory;
    size_t offset = 0;
    
public:
    template<typename T>
    T* allocate(size_t count = 1) {
        size_t bytes = sizeof(T) * count;
        size_t aligned_offset = (offset + alignof(T) - 1) & ~(alignof(T) - 1);
        
        if (aligned_offset + bytes > Size) {
            throw std::bad_alloc();
        }
        
        T* result = reinterpret_cast<T*>(&memory[aligned_offset]);
        offset = aligned_offset + bytes;
        return result;
    }
    
    void reset() {
        offset = 0;
    }
    
    size_t bytes_used() const { return offset; }
    size_t bytes_remaining() const { return Size - offset; }
};

// Frame allocator - resets every frame
class FrameAllocator {
private:
    StackAllocator<1024 * 1024> frameMemory; // 1MB per frame
    
public:
    template<typename T>
    T* allocate(size_t count = 1) {
        return frameMemory.allocate<T>(count);
    }
    
    void endFrame() {
        frameMemory.reset();
    }
};
```

## Memory Profiling Techniques

### Memory Tracking System
```cpp
class MemoryTracker {
private:
    struct AllocationInfo {
        size_t size;
        std::string file;
        int line;
        std::chrono::time_point<std::chrono::steady_clock> timestamp;
    };
    
    std::unordered_map<void*, AllocationInfo> allocations;
    std::mutex allocation_mutex;
    size_t total_allocated = 0;
    size_t peak_allocated = 0;
    
public:
    void recordAllocation(void* ptr, size_t size, const char* file, int line) {
        std::lock_guard<std::mutex> lock(allocation_mutex);
        
        allocations[ptr] = {size, file, line, std::chrono::steady_clock::now()};
        total_allocated += size;
        peak_allocated = std::max(peak_allocated, total_allocated);
    }
    
    void recordDeallocation(void* ptr) {
        std::lock_guard<std::mutex> lock(allocation_mutex);
        
        auto it = allocations.find(ptr);
        if (it != allocations.end()) {
            total_allocated -= it->second.size;
            allocations.erase(it);
        }
    }
    
    void reportLeaks() {
        std::lock_guard<std::mutex> lock(allocation_mutex);
        
        if (!allocations.empty()) {
            std::cout << "Memory leaks detected:\n";
            for (const auto& [ptr, info] : allocations) {
                std::cout << "  " << info.size << " bytes at " 
                         << info.file << ":" << info.line << "\n";
            }
        }
    }
    
    size_t getCurrentUsage() const { return total_allocated; }
    size_t getPeakUsage() const { return peak_allocated; }
};

// Global memory tracker
extern MemoryTracker g_memoryTracker;

// Override global new/delete for tracking
void* operator new(size_t size, const char* file, int line) {
    void* ptr = std::malloc(size);
    g_memoryTracker.recordAllocation(ptr, size, file, line);
    return ptr;
}

void operator delete(void* ptr) noexcept {
    g_memoryTracker.recordDeallocation(ptr);
    std::free(ptr);
}

#define TRACKED_NEW new(__FILE__, __LINE__)
```

### Performance Monitoring
```cpp
class MemoryProfiler {
private:
    struct FrameStats {
        size_t allocations_count = 0;
        size_t allocations_size = 0;
        size_t deallocations_count = 0;
        size_t deallocations_size = 0;
        std::chrono::microseconds allocation_time{0};
    };
    
    FrameStats current_frame;
    std::vector<FrameStats> frame_history;
    
public:
    void startFrame() {
        current_frame = {};
    }
    
    void endFrame() {
        frame_history.push_back(current_frame);
        if (frame_history.size() > 60) { // Keep last 60 frames
            frame_history.erase(frame_history.begin());
        }
    }
    
    void recordAllocation(size_t size, std::chrono::microseconds time) {
        current_frame.allocations_count++;
        current_frame.allocations_size += size;
        current_frame.allocation_time += time;
    }
    
    void recordDeallocation(size_t size) {
        current_frame.deallocations_count++;
        current_frame.deallocations_size += size;
    }
    
    void printStats() {
        if (frame_history.empty()) return;
        
        auto& stats = frame_history.back();
        std::cout << "Frame Memory Stats:\n";
        std::cout << "  Allocations: " << stats.allocations_count 
                 << " (" << stats.allocations_size << " bytes)\n";
        std::cout << "  Deallocations: " << stats.deallocations_count 
                 << " (" << stats.deallocations_size << " bytes)\n";
        std::cout << "  Allocation time: " << stats.allocation_time.count() << " μs\n";
    }
};
```

## Allocation Strategies

### Memory Layout Optimization
```cpp
// Structure of Arrays (SoA) for better cache performance
class ParticleSystemSoA {
private:
    std::vector<Vector2> positions;
    std::vector<Vector2> velocities;
    std::vector<float> lifetimes;
    std::vector<Color> colors;
    size_t active_count = 0;
    
public:
    void update(float deltaTime) {
        // Process in chunks for better cache usage
        for (size_t i = 0; i < active_count; ++i) {
            positions[i] += velocities[i] * deltaTime;
            lifetimes[i] -= deltaTime;
        }
        
        // Remove dead particles
        removeDeadParticles();
    }
    
private:
    void removeDeadParticles() {
        size_t write_index = 0;
        for (size_t read_index = 0; read_index < active_count; ++read_index) {
            if (lifetimes[read_index] > 0) {
                if (write_index != read_index) {
                    positions[write_index] = positions[read_index];
                    velocities[write_index] = velocities[read_index];
                    lifetimes[write_index] = lifetimes[read_index];
                    colors[write_index] = colors[read_index];
                }
                write_index++;
            }
        }
        active_count = write_index;
    }
};

// Memory pool with alignment
template<typename T, size_t Alignment = alignof(T)>
class AlignedObjectPool {
private:
    struct alignas(Alignment) AlignedStorage {
        char data[sizeof(T)];
    };
    
    std::vector<AlignedStorage> storage;
    std::queue<T*> available;
    
public:
    explicit AlignedObjectPool(size_t size) : storage(size) {
        for (auto& slot : storage) {
            available.push(reinterpret_cast<T*>(&slot));
        }
    }
    
    template<typename... Args>
    T* construct(Args&&... args) {
        if (available.empty()) return nullptr;
        
        T* obj = available.front();
        available.pop();
        
        new(obj) T(std::forward<Args>(args)...);
        return obj;
    }
    
    void destroy(T* obj) {
        if (!obj) return;
        
        obj->~T();
        available.push(obj);
    }
};
```

## Garbage Collection Alternatives

### Reference Counting with Cycle Detection
```cpp
template<typename T>
class CyclicPtr {
private:
    T* ptr = nullptr;
    std::atomic<int>* ref_count = nullptr;
    std::atomic<bool>* in_cycle_check = nullptr;
    
public:
    CyclicPtr() = default;
    
    explicit CyclicPtr(T* p) : ptr(p) {
        if (ptr) {
            ref_count = new std::atomic<int>(1);
            in_cycle_check = new std::atomic<bool>(false);
        }
    }
    
    CyclicPtr(const CyclicPtr& other) : ptr(other.ptr), ref_count(other.ref_count), in_cycle_check(other.in_cycle_check) {
        if (ref_count) {
            (*ref_count)++;
        }
    }
    
    ~CyclicPtr() {
        release();
    }
    
    CyclicPtr& operator=(const CyclicPtr& other) {
        if (this != &other) {
            release();
            ptr = other.ptr;
            ref_count = other.ref_count;
            in_cycle_check = other.in_cycle_check;
            if (ref_count) {
                (*ref_count)++;
            }
        }
        return *this;
    }
    
    T* get() const { return ptr; }
    T& operator*() const { return *ptr; }
    T* operator->() const { return ptr; }
    
private:
    void release() {
        if (ref_count && --(*ref_count) == 0) {
            delete ptr;
            delete ref_count;
            delete in_cycle_check;
        }
    }
};

// Generational garbage collector for game objects
class GenerationalGC {
private:
    std::vector<std::unique_ptr<GameObject>> young_generation;
    std::vector<std::unique_ptr<GameObject>> old_generation;
    std::vector<GameObject*> root_objects;
    
public:
    void addRoot(GameObject* obj) {
        root_objects.push_back(obj);
    }
    
    void minorCollection() {
        // Mark phase - start from roots
        std::unordered_set<GameObject*> reachable;
        for (auto* root : root_objects) {
            markReachable(root, reachable);
        }
        
        // Sweep young generation
        auto it = std::remove_if(young_generation.begin(), young_generation.end(),
            [&](const std::unique_ptr<GameObject>& obj) {
                return reachable.find(obj.get()) == reachable.end();
            });
        young_generation.erase(it, young_generation.end());
        
        // Promote survivors to old generation
        for (auto& obj : young_generation) {
            old_generation.push_back(std::move(obj));
        }
        young_generation.clear();
    }
    
private:
    void markReachable(GameObject* obj, std::unordered_set<GameObject*>& reachable) {
        if (!obj || reachable.find(obj) != reachable.end()) return;
        
        reachable.insert(obj);
        
        // Mark all referenced objects
        for (auto* child : obj->getChildren()) {
            markReachable(child, reachable);
        }
    }
};
```

## Memory Debugging Tools

### Debug Memory Allocator
```cpp
class DebugAllocator {
private:
    struct DebugHeader {
        uint32_t magic = 0xDEADBEEF;
        size_t size;
        const char* file;
        int line;
    };
    
    struct DebugFooter {
        uint32_t magic = 0xBEEFDEAD;
    };
    
    static constexpr uint8_t FREED_PATTERN = 0xDD;
    static constexpr uint8_t UNINITIALIZED_PATTERN = 0xCD;
    
public:
    static void* allocate(size_t size, const char* file, int line) {
        size_t total_size = sizeof(DebugHeader) + size + sizeof(DebugFooter);
        void* raw_ptr = std::malloc(total_size);
        
        if (!raw_ptr) return nullptr;
        
        // Fill with uninitialized pattern
        std::memset(raw_ptr, UNINITIALIZED_PATTERN, total_size);
        
        DebugHeader* header = static_cast<DebugHeader*>(raw_ptr);
        header->magic = 0xDEADBEEF;
        header->size = size;
        header->file = file;
        header->line = line;
        
        void* user_ptr = header + 1;
        
        DebugFooter* footer = reinterpret_cast<DebugFooter*>(
            static_cast<char*>(user_ptr) + size);
        footer->magic = 0xBEEFDEAD;
        
        return user_ptr;
    }
    
    static void deallocate(void* ptr) {
        if (!ptr) return;
        
        DebugHeader* header = static_cast<DebugHeader*>(ptr) - 1;
        
        // Check for corruption
        if (header->magic != 0xDEADBEEF) {
            std::cerr << "Memory corruption detected: invalid header magic\n";
            std::abort();
        }
        
        DebugFooter* footer = reinterpret_cast<DebugFooter*>(
            static_cast<char*>(ptr) + header->size);
        
        if (footer->magic != 0xBEEFDEAD) {
            std::cerr << "Buffer overflow detected at " << header->file 
                     << ":" << header->line << "\n";
            std::abort();
        }
        
        // Fill with freed pattern
        size_t total_size = sizeof(DebugHeader) + header->size + sizeof(DebugFooter);
        std::memset(header, FREED_PATTERN, total_size);
        
        std::free(header);
    }
};

#ifdef DEBUG
#define DEBUG_NEW(size) DebugAllocator::allocate(size, __FILE__, __LINE__)
#define DEBUG_DELETE(ptr) DebugAllocator::deallocate(ptr)
#else
#define DEBUG_NEW(size) std::malloc(size)
#define DEBUG_DELETE(ptr) std::free(ptr)
#endif
```

### Memory Leak Detection
```cpp
class LeakDetector {
private:
    struct AllocationRecord {
        size_t size;
        std::string stack_trace;
        std::chrono::time_point<std::chrono::steady_clock> timestamp;
    };
    
    std::unordered_map<void*, AllocationRecord> active_allocations;
    std::mutex mutex;
    bool enabled = true;
    
public:
    static LeakDetector& instance() {
        static LeakDetector detector;
        return detector;
    }
    
    void recordAllocation(void* ptr, size_t size) {
        if (!enabled) return;
        
        std::lock_guard<std::mutex> lock(mutex);
        active_allocations[ptr] = {
            size,
            captureStackTrace(),
            std::chrono::steady_clock::now()
        };
    }
    
    void recordDeallocation(void* ptr) {
        if (!enabled) return;
        
        std::lock_guard<std::mutex> lock(mutex);
        active_allocations.erase(ptr);
    }
    
    void checkForLeaks() {
        std::lock_guard<std::mutex> lock(mutex);
        
        if (active_allocations.empty()) {
            std::cout << "No memory leaks detected.\n";
            return;
        }
        
        std::cout << "Memory leaks detected:\n";
        size_t total_leaked = 0;
        
        for (const auto& [ptr, record] : active_allocations) {
            total_leaked += record.size;
            auto duration = std::chrono::steady_clock::now() - record.timestamp;
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
            
            std::cout << "  Leak: " << record.size << " bytes (age: " 
                     << seconds.count() << "s)\n";
            std::cout << "    Stack trace: " << record.stack_trace << "\n";
        }
        
        std::cout << "Total leaked: " << total_leaked << " bytes\n";
    }
    
private:
    std::string captureStackTrace() {
        // Platform-specific stack trace capture
        // This is a simplified version
        return "Stack trace capture not implemented";
    }
};
```

## Testing Strategies

### Memory Test Framework
```cpp
class MemoryTestSuite {
private:
    size_t initial_memory_usage;
    
public:
    void setUp() {
        initial_memory_usage = getCurrentMemoryUsage();
    }
    
    void tearDown() {
        size_t final_memory_usage = getCurrentMemoryUsage();
        if (final_memory_usage > initial_memory_usage) {
            size_t leaked = final_memory_usage - initial_memory_usage;
            std::cerr << "Memory leak detected: " << leaked << " bytes\n";
        }
    }
    
    void testObjectPooling() {
        ObjectPool<GameObject, 10> pool;
        
        // Test acquisition
        std::vector<GameObject*> objects;
        for (int i = 0; i < 10; ++i) {
            auto* obj = pool.acquire();
            assert(obj != nullptr);
            objects.push_back(obj);
        }
        
        // Pool should be exhausted
        assert(pool.acquire() == nullptr);
        
        // Test release
        for (auto* obj : objects) {
            pool.release(obj);
        }
        
        // Should be able to acquire again
        assert(pool.acquire() != nullptr);
    }
    
    void testSmartPointers() {
        auto shared1 = std::make_shared<GameObject>();
        assert(shared1.use_count() == 1);
        
        {
            auto shared2 = shared1;
            assert(shared1.use_count() == 2);
        }
        
        assert(shared1.use_count() == 1);
    }
    
    void testMemoryAlignment() {
        AlignedObjectPool<GameObject, 64> pool(10);
        
        auto* obj = pool.construct();
        assert(reinterpret_cast<uintptr_t>(obj) % 64 == 0);
        
        pool.destroy(obj);
    }
    
private:
    size_t getCurrentMemoryUsage() {
        // Platform-specific memory usage query
        return 0; // Simplified
    }
};
```

### Benchmark Framework
```cpp
class MemoryBenchmark {
public:
    static void benchmarkAllocation() {
        const size_t iterations = 100000;
        
        // Benchmark malloc/free
        auto start = std::chrono::high_resolution_clock::now();
        for (size_t i = 0; i < iterations; ++i) {
            void* ptr = std::malloc(64);
            std::free(ptr);
        }
        auto malloc_time = std::chrono::high_resolution_clock::now() - start;
        
        // Benchmark object pool
        ObjectPool<AllocTest, 1000> pool;
        start = std::chrono::high_resolution_clock::now();
        for (size_t i = 0; i < iterations; ++i) {
            auto* obj = pool.acquire();
            pool.release(obj);
        }
        auto pool_time = std::chrono::high_resolution_clock::now() - start;
        
        std::cout << "malloc/free: " << malloc_time.count() << " ns\n";
        std::cout << "object pool: " << pool_time.count() << " ns\n";
        std::cout << "Speedup: " << (double)malloc_time.count() / pool_time.count() << "x\n";
    }
    
private:
    struct AllocTest {
        char data[64];
        void reset() { /* no-op */ }
    };
};
```

## Optimization Examples

### Cache-Friendly Memory Layout
```cpp
// Component storage optimized for iteration
class ComponentManager {
private:
    // Structure of Arrays for better cache performance
    struct TransformComponents {
        std::vector<Vector3> positions;
        std::vector<Quaternion> rotations;
        std::vector<Vector3> scales;
        std::vector<EntityID> entities;
        
        void addComponent(EntityID entity, const Transform& transform) {
            positions.push_back(transform.position);
            rotations.push_back(transform.rotation);
            scales.push_back(transform.scale);
            entities.push_back(entity);
        }
        
        void removeComponent(size_t index) {
            // Swap with last element to avoid shifting
            if (index < positions.size() - 1) {
                std::swap(positions[index], positions.back());
                std::swap(rotations[index], rotations.back());
                std::swap(scales[index], scales.back());
                std::swap(entities[index], entities.back());
            }
            
            positions.pop_back();
            rotations.pop_back();
            scales.pop_back();
            entities.pop_back();
        }
    };
    
    TransformComponents transforms;
    
public:
    void updateTransforms() {
        // SIMD-friendly loop
        const size_t count = transforms.positions.size();
        for (size_t i = 0; i < count; ++i) {
            // Process 4 transforms at once using SIMD
            // transforms.positions[i] += velocity * deltaTime;
        }
    }
};

// Memory-mapped file for large assets
class AssetCache {
private:
    struct MappedFile {
        void* data;
        size_t size;
        int fd;
        
        MappedFile(const std::string& filename) {
            fd = open(filename.c_str(), O_RDONLY);
            if (fd == -1) throw std::runtime_error("Failed to open file");
            
            struct stat sb;
            if (fstat(fd, &sb) == -1) {
                close(fd);
                throw std::runtime_error("Failed to get file size");
            }
            
            size = sb.st_size;
            data = mmap(nullptr, size, PROT_READ, MAP_PRIVATE, fd, 0);
            if (data == MAP_FAILED) {
                close(fd);
                throw std::runtime_error("Failed to map file");
            }
        }
        
        ~MappedFile() {
            if (data != MAP_FAILED) {
                munmap(data, size);
            }
            if (fd != -1) {
                close(fd);
            }
        }
    };
    
    std::unordered_map<std::string, std::unique_ptr<MappedFile>> mapped_files;
    
public:
    const void* loadAsset(const std::string& filename) {
        auto it = mapped_files.find(filename);
        if (it != mapped_files.end()) {
            return it->second->data;
        }
        
        auto mapped = std::make_unique<MappedFile>(filename);
        const void* data = mapped->data;
        mapped_files[filename] = std::move(mapped);
        return data;
    }
};
```

## Common Memory Pitfalls
- **Memory leaks**: Forgetting to release allocated memory
- **Dangling pointers**: Using pointers after memory is freed
- **Buffer overflows**: Writing past allocated boundaries
- **Double deletion**: Calling delete twice on same pointer
- **Circular references**: Smart pointers creating reference cycles
- **Fragmentation**: Poor allocation patterns causing memory fragmentation
- **Cache misses**: Poor memory layout affecting performance

## Testing Strategy
- Test memory allocation/deallocation cycles
- Verify smart pointer reference counting
- Benchmark allocation performance
- Test pool exhaustion scenarios
- Validate memory alignment requirements
- Check for memory leaks in long-running tests
- Profile memory usage patterns

## Memory Debugging Tools Integration
```cpp
class MemoryManager {
private:
    bool debug_mode = false;
    std::unique_ptr<MemoryTracker> tracker;
    std::unique_ptr<LeakDetector> leak_detector;
    
public:
    void enableDebugMode() {
        debug_mode = true;
        tracker = std::make_unique<MemoryTracker>();
        leak_detector = std::make_unique<LeakDetector>();
    }
    
    void* allocate(size_t size, const char* file, int line) {
        void* ptr = std::malloc(size);
        
        if (debug_mode && tracker) {
            tracker->recordAllocation(ptr, size, file, line);
        }
        
        return ptr;
    }
    
    void deallocate(void* ptr) {
        if (debug_mode && tracker) {
            tracker->recordDeallocation(ptr);
        }
        
        std::free(ptr);
    }
    
    void generateReport() {
        if (tracker) {
            tracker->reportLeaks();
        }
        if (leak_detector) {
            leak_detector->checkForLeaks();
        }
    }
};

// Global memory manager
extern MemoryManager g_memoryManager;

// Macros for tracked allocation
#define TRACKED_MALLOC(size) g_memoryManager.allocate(size, __FILE__, __LINE__)
#define TRACKED_FREE(ptr) g_memoryManager.deallocate(ptr)
```

## Resources
- [C++ Smart Pointers](https://en.cppreference.com/w/cpp/memory)
- [Memory Management in C++](https://isocpp.org/wiki/faq/freestore-mgmt)
- [Object Pooling Patterns](https://gameprogrammingpatterns.com/object-pool.html)
- [Memory Debugging Tools](https://valgrind.org/)
- [CPU Cache and Memory](https://mechanical-sympathy.blogspot.com/)
- [RAII and Smart Pointers](https://docs.microsoft.com/en-us/cpp/cpp/smart-pointers-modern-cpp)

## Fun Challenges
- Implement a garbage collector for your engine
- Create a memory defragmentation system
- Build a memory usage visualizer
- Implement copy-on-write semantics
- Create a memory-mapped asset loading system
- Build a custom allocator for specific data types
- Implement weak reference cycle detection

## Debugging Tips
- Use memory sanitizers (AddressSanitizer, MemorySanitizer)
- Profile memory usage with tools like Valgrind or Visual Studio Diagnostics
- Implement custom memory tracking for development builds
- Add memory usage overlays to your engine
- Create memory usage graphs over time
- Test with artificial memory pressure
- Use static analysis tools to detect memory issues

## Refactoring Process

### Step 1: Smart Pointer Migration
1. Replace raw pointers with appropriate smart pointers
2. Update constructors to use make_unique/make_shared
3. Remove manual delete calls
4. Test that functionality remains unchanged

### Step 2: Implement Object Pooling
1. Identify frequently allocated/deallocated objects
2. Create object pools for these types
3. Replace direct allocation with pool acquisition
4. Measure performance improvements

### Step 3: Add Memory Tracking
1. Implement memory tracking system
2. Add allocation/deallocation hooks
3. Create memory usage reporting
4. Identify and fix memory leaks

### Step 4: Optimize Memory Layout
1. Analyze cache usage patterns
2. Restructure data for better locality
3. Implement custom allocators where beneficial
4. Measure cache performance improvements

### Step 5: Integration Testing
1. Run extensive memory tests
2. Profile memory usage under load
3. Validate leak detection systems
4. Document memory management patterns

## Integration with Engine Architecture
```cpp
// Updated engine with memory management
class GameEngine {
private:
    std::unique_ptr<MemoryManager> memory_manager;
    std::unique_ptr<Renderer> renderer;
    std::unique_ptr<AssetManager> asset_manager;
    std::unique_ptr<EntityManager> entity_manager;
    
public:
    GameEngine() {
        memory_manager = std::make_unique<MemoryManager>();
        memory_manager->enableDebugMode();
        
        renderer = std::make_unique<Renderer>();
        asset_manager = std::make_unique<AssetManager>();
        entity_manager = std::make_unique<EntityManager>();
    }
    
    ~GameEngine() {
        // Generate memory report before cleanup
        memory_manager->generateReport();
        
        // Smart pointers handle cleanup automatically
    }
    
    void update(float deltaTime) {
        // Update subsystems
        entity_manager->update(deltaTime);
        renderer->render();
        
        // Clean up temporary allocations
        memory_manager->endFrame();
    }
};
```