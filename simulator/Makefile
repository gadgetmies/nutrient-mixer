# Nutrient mixer simulator. Builds the firmware (../nutrient_mixer.ino) together with its Arduino libraries
# and the simulated ESP32/board/world into one program.
#
#   make          build
#   make run      build and start the simulator (opens the UI in the browser)
#   make test     run the scenarios in scenarios/ headless
#   make clean    remove the build (make distclean also removes the downloaded libraries)

CXX ?= c++
CC ?= cc
BUILD := build
DEPS := .deps
SKETCH := ../nutrient_mixer.ino
PROGRAM := $(BUILD)/nutrient-mixer-sim
PORT ?= 8737
PYTHON ?= python3

DEFINES := -DARDUINO=10819 -DESP32 -DARDUINO_ARCH_ESP32 -DGEM_DISABLE_U8G2
# The libraries are included as system headers so that their warnings do not drown ours
INCLUDES := -Ihal -Isrc -I$(BUILD)/gen \
	-isystem $(DEPS)/GEM/src -isystem $(DEPS)/Adafruit-GFX -isystem $(DEPS)/KeyDetector/src \
	-isystem $(DEPS)/AiEsp32RotaryEncoder/src -isystem $(DEPS)/PH4502C-Sensor/src -isystem $(DEPS)/QR-Code-generator/c
OPT ?= -O2 -g
CXXFLAGS += -std=gnu++17 $(OPT) $(DEFINES) $(INCLUDES) -MMD -MP
CFLAGS += -std=c11 $(OPT) $(INCLUDES) -MMD -MP
WARNINGS := -Wall -Wno-unused-function
LDLIBS += -lpthread

SIM_SOURCES := $(wildcard src/*.cpp) $(wildcard hal/*.cpp)
LIBRARY_SOURCES := $(DEPS)/GEM/src/GEM.cpp $(DEPS)/GEM/src/GEMItem.cpp $(DEPS)/GEM/src/GEMPage.cpp \
	$(DEPS)/GEM/src/GEMSelect.cpp $(DEPS)/GEM/src/GEMSpinner.cpp $(DEPS)/GEM/src/GEM_adafruit_gfx.cpp \
	$(DEPS)/Adafruit-GFX/Adafruit_GFX.cpp $(DEPS)/KeyDetector/src/KeyDetector.cpp \
	$(DEPS)/AiEsp32RotaryEncoder/src/AiEsp32RotaryEncoder.cpp $(DEPS)/PH4502C-Sensor/src/ph4502c_sensor.cpp
LIBRARY_C_SOURCES := $(DEPS)/QR-Code-generator/c/qrcodegen.c

SIM_OBJECTS := $(patsubst %.cpp,$(BUILD)/%.o,$(SIM_SOURCES))
LIBRARY_OBJECTS := $(patsubst $(DEPS)/%.cpp,$(BUILD)/lib/%.o,$(LIBRARY_SOURCES)) \
	$(patsubst $(DEPS)/%.c,$(BUILD)/lib/%.o,$(LIBRARY_C_SOURCES))
OBJECTS := $(SIM_OBJECTS) $(LIBRARY_OBJECTS)
DEPS_STAMP := $(DEPS)/.stamp

.PHONY: all run test clean distclean deps

all: $(PROGRAM)

deps: $(DEPS_STAMP)

$(DEPS_STAMP): scripts/fetch_deps.sh
	sh scripts/fetch_deps.sh $(DEPS)
	touch $@

$(BUILD)/gen/nutrient_mixer.ino.cpp: $(SKETCH) scripts/gen_prototypes.py
	@mkdir -p $(dir $@)
	$(PYTHON) scripts/gen_prototypes.py $(SKETCH) $@

$(BUILD)/src/firmware_unit.o: $(BUILD)/gen/nutrient_mixer.ino.cpp

$(BUILD)/src/%.o: src/%.cpp | $(DEPS_STAMP)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(WARNINGS) -c $< -o $@

$(BUILD)/hal/%.o: hal/%.cpp | $(DEPS_STAMP)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(WARNINGS) -c $< -o $@

$(BUILD)/lib/%.o: $(DEPS)/%.cpp | $(DEPS_STAMP)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -w -c $< -o $@

$(BUILD)/lib/%.o: $(DEPS)/%.c | $(DEPS_STAMP)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -w -c $< -o $@

$(LIBRARY_SOURCES) $(LIBRARY_C_SOURCES): $(DEPS_STAMP)

$(PROGRAM): $(OBJECTS)
	$(CXX) $(OBJECTS) $(LDFLAGS) $(LDLIBS) -o $@

run: $(PROGRAM)
	./$(PROGRAM) --port $(PORT) --open

test: $(PROGRAM)
	sh scripts/run_scenarios.sh $(PROGRAM) scenarios

clean:
	rm -rf $(BUILD)

distclean: clean
	rm -rf $(DEPS)

-include $(OBJECTS:.o=.d)
