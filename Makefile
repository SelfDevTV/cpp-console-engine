# Console Game Engine Makefile

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -Iinclude
TARGET = game
SRCDIR = src
INCDIR = include
SOURCES = $(wildcard $(SRCDIR)/*.cpp)

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCES)

clean:
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)

debug: CXXFLAGS += -g -DDEBUG
debug: $(TARGET)

install:
	@echo "Console Game Engine"
	@echo "Compile with: make"
	@echo "Run with: make run"
	@echo "Clean with: make clean"