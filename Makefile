CC=g++
RM=rm

peripheral_name = MCP23S08

CFLAGS = -g -ggdb -std=c++11 -Werror -Wall -Wfatal-errors -Wno-shift-count-overflow -Weffc++ -Wno-non-virtual-dtor -Wno-strict-aliasing -Wno-unused-private-field -O3

$(shell mkdir -p _build)

all: _build/$(peripheral_name).so

_build/$(peripheral_name).o: $(peripheral_name).cpp
	$(CC) -fPIC -c -o $@ $^ $(CFLAGS) -I./include -I./modeling-framework/include

_build/ModelingFramework.o: modeling-framework/src/ModelingFramework.cpp
	$(CC) -fPIC -c -o $@ $^ $(CFLAGS) -I./modeling-framework/include

_build/$(peripheral_name).so: _build/$(peripheral_name).o _build/ModelingFramework.o
	$(CC) -shared -o $@ $^

.PHONY: clean
clean:
	$(RM) -rf _build
