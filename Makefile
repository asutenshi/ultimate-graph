CXX = g++
CXXFLAGS = -std=c++20 -Wall -Igraph-project/include
TARGET = app

# Укажите здесь все ваши исходные .cpp файлы
SOURCES = src/main.cpp src/graph/GraphWrapper.cpp src/pathFinding/LIAN.cpp

# Правило для сборки
all:
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCES)

# Правило для очистки
clean:
	rm -f $(TARGET)
