# CMake 없이도 바로 빌드할 수 있는 간단한 Makefile.
#   make          → run_tests 빌드
#   make run      → 빌드 후 실행
#   make clean    → 산출물 삭제

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Iinclude

SOURCES := \
	src/grid.cpp \
	src/validator.cpp \
	src/renderer.cpp \
	src/scenario.cpp \
	user/my_planner.cpp \
	tests/run_tests.cpp

TARGET := run_tests

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) -o $(TARGET)

.PHONY: run clean
run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
