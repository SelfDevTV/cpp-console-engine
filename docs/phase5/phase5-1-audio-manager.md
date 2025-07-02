# Phase 5.1: AudioManager Design

## C++ Learning Focus
- **Resource management**: RAII for audio resources and cleanup
- **Factory pattern**: Creating different types of sound objects
- **Platform-specific APIs**: Conditional compilation for audio systems
- **Template patterns**: Generic audio resource containers
- **Timing and synchronization**: Managing audio playback timing
- **Queue management**: STL containers for audio effect queuing

## Implementation Overview
Design an AudioManager to handle console-based audio in your game engine:
1. Manage console beep generation and timing
2. Queue and play sound effects sequentially
3. Handle platform-specific audio APIs
4. Provide audio configuration and volume control
5. Abstract audio operations behind a clean interface
6. Implement basic audio timing and synchronization

## Console Audio Capabilities

### What Console Audio Can Do
- **System beeps**: Short duration tones
- **Frequency control**: Different pitch beeps
- **Duration control**: Variable length sounds
- **Sequential playback**: Queue multiple sounds
- **Volume control**: System volume adjustment (limited)

### Console Audio Limitations
- No complex audio formats (MP3, WAV, etc.)
- Limited frequency range
- No simultaneous sound mixing
- Platform-dependent capabilities
- System volume limitations

## Class Structure Design

### Basic AudioManager Interface
```cpp
class AudioManager {
public:
    AudioManager();
    ~AudioManager();
    
    bool initialize();
    void shutdown();
    void update(double deltaTime);
    
    // Sound effect management
    void playBeep(int frequency, int durationMs);
    void queueSound(const SoundEffect& effect);
    void clearSoundQueue();
    
    // Configuration
    void setVolume(float volume);
    float getVolume() const;
    void setEnabled(bool enabled);
    bool isEnabled() const;
    
    // Status
    bool isPlaying() const;
    size_t getQueueSize() const;
    
private:
    // Implementation details hidden
};
```

### Sound Effect Design
```cpp
struct SoundEffect {
    int frequency;          // Hz (e.g., 440 for A4)
    int durationMs;        // Duration in milliseconds
    int delayAfterMs;      // Pause after this sound
    
    SoundEffect(int freq, int duration, int delay = 100)
        : frequency(freq), durationMs(duration), delayAfterMs(delay) {}
};

// Factory functions for common sounds
namespace SoundEffects {
    SoundEffect beep();
    SoundEffect lowBeep();
    SoundEffect highBeep();
    SoundEffect errorSound();
    SoundEffect successSound();
    SoundEffect clickSound();
}
```

## Key C++ Concepts to Learn

### RAII for Audio Resources
```cpp
class AudioManager {
private:
    bool initialized;
    bool audioEnabled;
    std::queue<SoundEffect> soundQueue;
    std::chrono::steady_clock::time_point lastSoundTime;
    
public:
    AudioManager() : initialized(false), audioEnabled(true) {
        // Constructor - prepare for initialization
    }
    
    ~AudioManager() {
        // Destructor - automatic cleanup
        if (initialized) {
            shutdown();
        }
    }
    
    bool initialize() {
        if (initialized) return true;
        
        // Platform-specific audio system setup
        #ifdef _WIN32
            if (!initializeWindowsAudio()) return false;
        #else
            if (!initializeLinuxAudio()) return false;
        #endif
        
        initialized = true;
        return true;
    }
};
```

### Factory Pattern for Sound Effects
```cpp
class SoundEffectFactory {
public:
    // Factory methods for different categories
    static SoundEffect createUISound(const std::string& type) {
        if (type == "click") return SoundEffect(800, 50, 50);
        if (type == "hover") return SoundEffect(600, 30, 30);
        if (type == "select") return SoundEffect(1000, 100, 100);
        return SoundEffect(400, 100, 100); // default
    }
    
    static SoundEffect createGameSound(const std::string& type) {
        if (type == "pickup") return SoundEffect(1200, 150, 50);
        if (type == "damage") return SoundEffect(200, 200, 200);
        if (type == "victory") return SoundEffect(800, 500, 100);
        return SoundEffect(400, 100, 100); // default
    }
    
    // Sequence factory for complex sounds
    static std::vector<SoundEffect> createSequence(const std::string& sequence) {
        std::vector<SoundEffect> sounds;
        if (sequence == "startup") {
            sounds.push_back(SoundEffect(440, 100, 50));
            sounds.push_back(SoundEffect(550, 100, 50));
            sounds.push_back(SoundEffect(660, 200, 100));
        }
        return sounds;
    }
};
```

### Template Audio Container
```cpp
template<typename T>
class AudioQueue {
private:
    std::queue<T> queue;
    std::mutex queueMutex;  // Thread safety
    size_t maxSize;
    
public:
    AudioQueue(size_t max = 100) : maxSize(max) {}
    
    bool enqueue(const T& item) {
        std::lock_guard<std::mutex> lock(queueMutex);
        if (queue.size() >= maxSize) return false;
        queue.push(item);
        return true;
    }
    
    bool dequeue(T& item) {
        std::lock_guard<std::mutex> lock(queueMutex);
        if (queue.empty()) return false;
        item = queue.front();
        queue.pop();
        return true;
    }
    
    size_t size() const {
        std::lock_guard<std::mutex> lock(queueMutex);
        return queue.size();
    }
    
    void clear() {
        std::lock_guard<std::mutex> lock(queueMutex);
        std::queue<T> empty;
        queue.swap(empty);
    }
};
```

## Platform-Specific Implementations

### Windows Audio (windows.h)
```cpp
#ifdef _WIN32
class WindowsAudioImpl {
public:
    static void playBeep(int frequency, int durationMs) {
        // Use Windows Beep() function
        Beep(frequency, durationMs);
    }
    
    static bool isAudioAvailable() {
        // Check if audio hardware is available
        return true; // Windows console always supports Beep()
    }
    
    static void setSystemVolume(float volume) {
        // Use Windows volume APIs (more complex)
        // This is optional for console games
    }
};
#endif
```

### Linux Audio (ALSA/OSS)
```cpp
#ifdef __linux__
class LinuxAudioImpl {
public:
    static void playBeep(int frequency, int durationMs) {
        // Method 1: Use speaker-test utility
        std::string command = "speaker-test -t sine -f " + 
                             std::to_string(frequency) + " -l 1 -s 1 >/dev/null 2>&1 &";
        system(command.c_str());
        
        // Method 2: Direct /dev/dsp access (if available)
        // Method 3: ALSA library calls
        
        // Sleep for duration
        std::this_thread::sleep_for(std::chrono::milliseconds(durationMs));
    }
    
    static bool isAudioAvailable() {
        // Check for audio devices
        return access("/dev/dsp", F_OK) == 0 || 
               access("/proc/asound/cards", F_OK) == 0;
    }
};
#endif
```

### macOS Audio (AudioToolbox)
```cpp
#ifdef __APPLE__
class MacAudioImpl {
public:
    static void playBeep(int frequency, int durationMs) {
        // Use AudioToolbox framework
        // Or fallback to system beep
        NSBeep();
    }
    
    static bool isAudioAvailable() {
        return true; // macOS always has audio
    }
};
#endif
```

## Audio Queuing System

### Queue Management
```cpp
class AudioManager {
private:
    AudioQueue<SoundEffect> soundQueue;
    std::chrono::steady_clock::time_point nextSoundTime;
    bool currentlyPlaying;
    
public:
    void update(double deltaTime) {
        auto now = std::chrono::steady_clock::now();
        
        // Check if we can play the next sound
        if (!currentlyPlaying && now >= nextSoundTime) {
            SoundEffect sound;
            if (soundQueue.dequeue(sound)) {
                playSound(sound);
                
                // Calculate when next sound can play
                nextSoundTime = now + 
                    std::chrono::milliseconds(sound.durationMs + sound.delayAfterMs);
            }
        }
    }
    
private:
    void playSound(const SoundEffect& sound) {
        if (!audioEnabled) return;
        
        currentlyPlaying = true;
        
        // Platform-specific playback
        #ifdef _WIN32
            WindowsAudioImpl::playBeep(sound.frequency, sound.durationMs);
        #elif defined(__linux__)
            LinuxAudioImpl::playBeep(sound.frequency, sound.durationMs);
        #elif defined(__APPLE__)
            MacAudioImpl::playBeep(sound.frequency, sound.durationMs);
        #endif
        
        currentlyPlaying = false;
    }
};
```

### Sequence Playback
```cpp
class AudioSequencePlayer {
private:
    std::vector<SoundEffect> sequence;
    size_t currentIndex;
    bool playing;
    
public:
    void playSequence(const std::vector<SoundEffect>& sounds) {
        sequence = sounds;
        currentIndex = 0;
        playing = true;
    }
    
    void update(AudioManager& audioManager, double deltaTime) {
        if (!playing || currentIndex >= sequence.size()) {
            playing = false;
            return;
        }
        
        if (!audioManager.isPlaying() && audioManager.getQueueSize() == 0) {
            audioManager.queueSound(sequence[currentIndex++]);
        }
    }
    
    bool isPlaying() const { return playing; }
    void stop() { playing = false; }
};
```

## Timing and Synchronization

### Precise Timing Control
```cpp
class AudioTimer {
private:
    std::chrono::steady_clock::time_point startTime;
    std::chrono::milliseconds duration;
    bool active;
    
public:
    void start(int durationMs) {
        startTime = std::chrono::steady_clock::now();
        duration = std::chrono::milliseconds(durationMs);
        active = true;
    }
    
    bool isExpired() const {
        if (!active) return false;
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - startTime);
        
        return elapsed >= duration;
    }
    
    float getProgress() const {
        if (!active) return 0.0f;
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - startTime);
        
        return static_cast<float>(elapsed.count()) / duration.count();
    }
};
```

### Thread-Safe Audio Processing
```cpp
class ThreadSafeAudioManager {
private:
    std::mutex audioMutex;
    std::condition_variable audioCondition;
    std::thread audioThread;
    std::atomic<bool> shouldStop;
    AudioQueue<SoundEffect> pendingSounds;
    
public:
    void startAudioThread() {
        shouldStop = false;
        audioThread = std::thread(&ThreadSafeAudioManager::audioWorker, this);
    }
    
    void stopAudioThread() {
        shouldStop = true;
        audioCondition.notify_all();
        if (audioThread.joinable()) {
            audioThread.join();
        }
    }
    
private:
    void audioWorker() {
        while (!shouldStop) {
            SoundEffect sound;
            if (pendingSounds.dequeue(sound)) {
                playSound(sound);
            } else {
                // Wait for new sounds or stop signal
                std::unique_lock<std::mutex> lock(audioMutex);
                audioCondition.wait_for(lock, std::chrono::milliseconds(10));
            }
        }
    }
};
```

## Audio Configuration

### Configuration Structure
```cpp
struct AudioConfig {
    bool enabled = true;
    float volume = 1.0f;            // 0.0 to 1.0
    int maxQueueSize = 50;
    int defaultFrequency = 440;     // A4 note
    int defaultDuration = 100;      // milliseconds
    int defaultDelay = 50;          // milliseconds between sounds
    bool enableThreadedPlayback = false;
    
    // Platform-specific settings
    #ifdef _WIN32
    bool useWindowsBeep = true;
    #endif
    
    #ifdef __linux__
    std::string audioDevice = "/dev/dsp";
    bool useSpeakerTest = true;
    #endif
};

class AudioManager {
private:
    AudioConfig config;
    
public:
    AudioManager(const AudioConfig& cfg = AudioConfig()) : config(cfg) {}
    
    void updateConfig(const AudioConfig& newConfig) {
        config = newConfig;
        // Apply configuration changes
        soundQueue.setMaxSize(config.maxQueueSize);
        audioEnabled = config.enabled;
    }
    
    const AudioConfig& getConfig() const { return config; }
};
```

### Configuration Loading
```cpp
class AudioConfigLoader {
public:
    static AudioConfig loadFromFile(const std::string& filename) {
        AudioConfig config;
        
        // Simple key-value parser
        std::ifstream file(filename);
        std::string line;
        
        while (std::getline(file, line)) {
            auto pos = line.find('=');
            if (pos != std::string::npos) {
                std::string key = line.substr(0, pos);
                std::string value = line.substr(pos + 1);
                
                if (key == "enabled") config.enabled = (value == "true");
                else if (key == "volume") config.volume = std::stof(value);
                else if (key == "maxQueueSize") config.maxQueueSize = std::stoi(value);
                // ... more config options
            }
        }
        
        return config;
    }
    
    static void saveToFile(const AudioConfig& config, const std::string& filename) {
        std::ofstream file(filename);
        file << "enabled=" << (config.enabled ? "true" : "false") << std::endl;
        file << "volume=" << config.volume << std::endl;
        file << "maxQueueSize=" << config.maxQueueSize << std::endl;
        // ... more config options
    }
};
```

## Testing Strategies

### Unit Testing
```cpp
class AudioManagerTest {
public:
    void testBasicPlayback() {
        AudioManager audio;
        assert(audio.initialize());
        
        audio.playBeep(440, 100);
        // Can't easily test audio output, but can test state
        assert(!audio.isPlaying()); // Should be false after short delay
    }
    
    void testQueueing() {
        AudioManager audio;
        audio.initialize();
        
        audio.queueSound(SoundEffect(440, 100));
        audio.queueSound(SoundEffect(550, 100));
        
        assert(audio.getQueueSize() == 2);
        
        // Process queue
        audio.update(0.1);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        audio.update(0.1);
        
        assert(audio.getQueueSize() == 1);
    }
    
    void testConfiguration() {
        AudioConfig config;
        config.enabled = false;
        
        AudioManager audio(config);
        audio.initialize();
        
        audio.playBeep(440, 100);
        // Should not play any sound when disabled
        assert(!audio.isPlaying());
    }
};
```

### Integration Testing
```cpp
class AudioIntegrationTest {
public:
    void testWithGameEngine() {
        Engine engine;
        AudioManager* audio = engine.getAudioManager();
        
        // Test audio integration with game events
        GameObject player;
        player.onCollision([audio](GameObject& other) {
            audio->queueSound(SoundEffects::clickSound());
        });
        
        // Simulate collision
        GameObject obstacle;
        player.handleCollision(obstacle);
        
        // Verify sound was queued
        assert(audio->getQueueSize() > 0);
    }
};
```

### Platform Testing
```cpp
class PlatformAudioTest {
public:
    void testPlatformCompatibility() {
        AudioManager audio;
        
        #ifdef _WIN32
        assert(WindowsAudioImpl::isAudioAvailable());
        #elif defined(__linux__)
        assert(LinuxAudioImpl::isAudioAvailable());
        #elif defined(__APPLE__)
        assert(MacAudioImpl::isAudioAvailable());
        #endif
        
        assert(audio.initialize());
    }
};
```

## Integration Examples

### UI Sound Integration
```cpp
class Menu {
private:
    AudioManager* audioManager;
    
public:
    Menu(AudioManager* audio) : audioManager(audio) {}
    
    void onButtonHover(Button& button) {
        audioManager->queueSound(SoundEffects::createUISound("hover"));
    }
    
    void onButtonClick(Button& button) {
        audioManager->queueSound(SoundEffects::createUISound("click"));
    }
    
    void onMenuOpen() {
        auto openSequence = SoundEffectFactory::createSequence("menu_open");
        for (const auto& sound : openSequence) {
            audioManager->queueSound(sound);
        }
    }
};
```

### Game Event Integration
```cpp
class GameEventSystem {
private:
    AudioManager* audioManager;
    
public:
    GameEventSystem(AudioManager* audio) : audioManager(audio) {}
    
    void onPlayerDamage(int damage) {
        if (damage > 50) {
            audioManager->queueSound(SoundEffects::createGameSound("heavy_damage"));
        } else {
            audioManager->queueSound(SoundEffects::createGameSound("light_damage"));
        }
    }
    
    void onItemPickup(const Item& item) {
        if (item.isRare()) {
            auto sequence = SoundEffectFactory::createSequence("rare_pickup");
            for (const auto& sound : sequence) {
                audioManager->queueSound(sound);
            }
        } else {
            audioManager->queueSound(SoundEffects::createGameSound("pickup"));
        }
    }
    
    void onLevelComplete() {
        auto victorySequence = SoundEffectFactory::createSequence("victory");
        for (const auto& sound : victorySequence) {
            audioManager->queueSound(sound);
        }
    }
};
```

### Scene-Based Audio
```cpp
class Scene {
protected:
    AudioManager* audioManager;
    std::unique_ptr<AudioSequencePlayer> backgroundPlayer;
    
public:
    Scene(AudioManager* audio) : audioManager(audio) {
        backgroundPlayer = std::make_unique<AudioSequencePlayer>();
    }
    
    virtual void onEnter() {
        // Play scene entry sound
        auto entrySequence = getSceneEntrySequence();
        backgroundPlayer->playSequence(entrySequence);
    }
    
    virtual void update(double deltaTime) {
        backgroundPlayer->update(*audioManager, deltaTime);
    }
    
    virtual std::vector<SoundEffect> getSceneEntrySequence() = 0;
};

class MainMenuScene : public Scene {
public:
    MainMenuScene(AudioManager* audio) : Scene(audio) {}
    
    std::vector<SoundEffect> getSceneEntrySequence() override {
        return SoundEffectFactory::createSequence("main_menu");
    }
};
```

## Performance Considerations

### Memory Management
- Pre-allocate sound effect objects during initialization
- Use object pools for frequently created temporary sounds
- Limit queue sizes to prevent memory bloat
- Clear unused sound sequences promptly

### CPU Optimization
- Avoid string comparisons in hot paths
- Cache frequently used sound effects
- Use move semantics for sound effect containers
- Minimize audio processing in main thread

### Platform Optimization
```cpp
class OptimizedAudioManager {
private:
    // Cache platform-specific implementations
    #ifdef _WIN32
    static constexpr bool useNativeBeep = true;
    #else
    static constexpr bool useNativeBeep = false;
    #endif
    
    // Pre-computed frequency tables
    static const std::array<int, 12> chromaticScale;
    
public:
    void playNote(int noteIndex, int durationMs) {
        if constexpr (useNativeBeep) {
            WindowsAudioImpl::playBeep(chromaticScale[noteIndex], durationMs);
        } else {
            LinuxAudioImpl::playBeep(chromaticScale[noteIndex], durationMs);
        }
    }
};
```

## Common Design Pitfalls

### Audio System Pitfalls
- **Blocking audio calls**: Don't block the main thread with long audio operations
- **Queue overflow**: Always check queue limits before adding sounds
- **Platform assumptions**: Don't assume all platforms support the same audio features
- **Resource leaks**: Ensure proper cleanup of audio resources
- **Thread safety**: Protect shared audio state with proper synchronization

### Performance Pitfalls
- **Frequent allocations**: Don't create new sound objects in update loops
- **Synchronous playback**: Avoid waiting for audio completion in main thread
- **String operations**: Minimize string processing in audio hot paths
- **Complex calculations**: Pre-compute frequency values and timing

## Resources
- [Windows Console Beep API](https://docs.microsoft.com/en-us/windows/win32/api/utilapiset/nf-utilapiset-beep)
- [Linux Audio Programming Guide](https://tldp.org/HOWTO/Sound-HOWTO/)
- [ALSA Project Documentation](https://www.alsa-project.org/wiki/Documentation)
- [Audio Programming Patterns](https://www.gameaudioimplementation.com/)
- [C++ Threading Tutorial](https://www.cplusplus.com/reference/thread/)
- [STL Containers Performance](https://en.cppreference.com/w/cpp/container)

## Fun Challenges

### Creative Sound Design
- Implement musical scale support (major, minor, pentatonic)
- Create procedural audio sequences based on game state
- Add rhythm pattern support for background "music"
- Implement audio compression for longer sequences

### Advanced Features
- Create audio scripting system with simple commands
- Implement audio fade-in/fade-out effects (volume ramping)
- Add audio visualization in console (ASCII waveforms)
- Create audio profile system for different game modes

### Cross-Platform Enhancement
- Add Bluetooth speaker support detection
- Implement audio device enumeration and selection
- Create fallback audio methods for silent environments
- Add audio accessibility features for hearing-impaired users

## Debugging Tips

### Audio Debugging Tools
```cpp
class AudioDebugger {
public:
    static void logSoundEvent(const SoundEffect& sound, const std::string& event) {
        std::cout << "[AUDIO] " << event << " - Freq: " << sound.frequency 
                  << "Hz, Duration: " << sound.durationMs << "ms" << std::endl;
    }
    
    static void printQueueStatus(const AudioManager& audio) {
        std::cout << "[AUDIO] Queue size: " << audio.getQueueSize() 
                  << ", Playing: " << (audio.isPlaying() ? "Yes" : "No") << std::endl;
    }
    
    static void printAudioConfig(const AudioConfig& config) {
        std::cout << "[AUDIO CONFIG]" << std::endl;
        std::cout << "  Enabled: " << (config.enabled ? "Yes" : "No") << std::endl;
        std::cout << "  Volume: " << config.volume << std::endl;
        std::cout << "  Max Queue: " << config.maxQueueSize << std::endl;
    }
};
```

### Common Issues and Solutions
- **No audio output**: Check platform audio availability and system volume
- **Delayed playback**: Verify timing calculations and thread synchronization
- **Queue not processing**: Ensure update() is called regularly in main loop
- **Audio cutting off**: Check for competing audio processes and resource conflicts

## Refactoring Process

### Step 1: Basic Audio Infrastructure
1. Create AudioManager class with basic beep functionality
2. Implement platform detection and basic sound playback
3. Add simple queue system for sound effects
4. Test basic audio output on target platform

### Step 2: Sound Effect System
1. Create SoundEffect structure and factory methods
2. Implement audio queuing and timing system
3. Add configuration management
4. Create common sound effect library

### Step 3: Advanced Features
1. Add sequence playback support
2. Implement thread-safe audio processing
3. Create audio debugging and profiling tools
4. Add integration hooks for game systems

### Step 4: Polish and Optimization
1. Optimize performance for target platforms
2. Add comprehensive error handling
3. Create extensive test suite
4. Document usage patterns and best practices

## Integration with Engine Architecture

### Engine Integration
```cpp
class Engine {
private:
    std::unique_ptr<AudioManager> audioManager;
    AudioConfig audioConfig;
    
public:
    bool initialize() {
        // Load audio configuration
        audioConfig = AudioConfigLoader::loadFromFile("config/audio.cfg");
        
        // Initialize audio system
        audioManager = std::make_unique<AudioManager>(audioConfig);
        if (!audioManager->initialize()) {
            std::cerr << "Warning: Audio system failed to initialize" << std::endl;
            // Continue without audio
        }
        
        return true;
    }
    
    void update(double deltaTime) {
        if (audioManager) {
            audioManager->update(deltaTime);
        }
    }
    
    AudioManager* getAudioManager() const {
        return audioManager.get();
    }
};
```

This AudioManager design provides a solid foundation for console-based audio in your game engine while teaching important C++ concepts like RAII, factory patterns, and platform abstraction. The system is designed to be extensible and can grow with your engine's complexity needs.