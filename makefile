# Компилятор и флаги
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
INCLUDES = -I/usr/include/eigen3
LIBS = -lsfml-graphics -lsfml-window -lsfml-system -lpthread

# Директории
SRC_DIR = .
BUILD_DIR = build
BIN_DIR = bin

# Исходные файлы
SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS = $(SOURCES:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)

# Имя исполняемого файла
TARGET = $(BIN_DIR)/uav_simulation

# Правило по умолчанию
all: directories $(TARGET)

# Создание директорий
directories:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BIN_DIR)

# Компиляция исполняемого файла
$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(TARGET) $(LIBS)

# Компиляция объектных файлов
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Очистка
clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)

# Пересборка
rebuild: clean all

# Создание тестового файла terrain.txt
terrain:
	@echo "Creating test terrain file..."
	@echo "0 0 100" > terrain.txt
	@echo "10 10 120" >> terrain.txt
	@echo "20 20 110" >> terrain.txt
	@echo "30 30 130" >> terrain.txt
	@echo "40 40 125" >> terrain.txt
	@echo "50 50 115" >> terrain.txt

# Запуск
run: all terrain
	./$(TARGET)

# Отладка
debug: CXXFLAGS += -g
debug: clean all

.PHONY: all clean rebuild run debug terrain directories