cmake_minimum_required(VERSION 3.8)
project(lab4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Znajdź zależności
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# Dodaj plik źródłowy test_nav.cpp
add_executable(test_nav src/test_nav.cpp)

# Dodaj katalogi z nagłówkami
target_include_directories(test_nav PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Wymagania dotyczące standardów C99 i C++17
target_compile_features(test_nav PUBLIC c_std_99 cxx_std_17)

# Połącz target z wymaganymi zależnościami
ament_target_dependencies(test_nav rclcpp nav_msgs)

# Instalacja pliku wykonywalnego
install(TARGETS test_nav
  DESTINATION lib/${PROJECT_NAME})

# Dodaj obsługę testów i linterów
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skoki w linterach (jeśli nie masz jeszcze licencji lub repozytorium git)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
