cmake_minimum_required(VERSION 3.10)

# 프로젝트 이름과 언어 설정
project(LTE-Drone-Communication LANGUAGES CXX)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 서브 디렉토리 추가
add_subdirectory(server)
add_subdirectory(client)
