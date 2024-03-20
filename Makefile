SRC_SOURCES = $(shell find src -type f -name "*.cpp")
SRC_HEADERS = $(shell find src -type f -name "*.h")

SRC_OBJ = ${SRC_SOURCES:.cpp=.o}
CC = g++ -lstdc++ -O3
all:main

main: main.cpp ${SRC_OBJ}
	${CC} ${SRC_OBJ} $< -o $@

clean:
	rm -rf ${SRC_OBJ} main

run: main
	time ./main

%.o: %.cpp ${SRC_HEADERS}
	${CC} -c $< -o $@
