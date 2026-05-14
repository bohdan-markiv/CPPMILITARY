# Зручна точка входу для перевірки якості ДЗ 6.
# Перед PR достатньо запустити: make quality

# Файли, до яких застосовуються інструменти.
CXX_FILES = include/ballistics.hpp \
            src/ballistics.cpp \
            src/ballistics_cli.cpp \
            src/main.cpp \
            tests/ballistics_tests.cpp

CMAKE_FILES = CMakeLists.txt

BUILD_DIR = ../build/debug

# Форматування C++ і CMake файлів.
format:
	clang-format -i $(CXX_FILES)
	cmake-format -i $(CMAKE_FILES)

# Статичний аналіз змінених C++ файлів.
lint:
	clang-tidy -p $(BUILD_DIR) src/ballistics.cpp
	clang-tidy -p $(BUILD_DIR) src/ballistics_cli.cpp
	clang-tidy -p $(BUILD_DIR) src/main.cpp
	clang-tidy -p $(BUILD_DIR) tests/ballistics_tests.cpp

# Збірка і запуск тестів.
test:
	cd .. && cmake --build --preset debug
	cd .. && ctest --test-dir build/debug --output-on-failure

# Повна перевірка: формат -> аналіз -> тести.
quality: format lint test

.PHONY: format lint test quality