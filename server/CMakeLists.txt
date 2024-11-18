# 실행 파일 생성
add_executable(server server.cpp)

# 공통 디렉토리의 헤더 파일 포함
target_include_directories(server PRIVATE ../common)
