# Haptic device controller
#
# Author: Dan O'Shea (c) 2013
# NPL Stanford University
#

CFG = debug
include Makefile.utils

# determine platform
ARCH   ?= $(shell echo `uname -m`)
SYSTEM ?= $(shell echo `uname -s`)
ifeq ($(SYSTEM), Linux)
OS = lin
endif
ifeq ($(SYSTEM), Darwin)
OS = mac
endif

CXX = clang++
MARCH = $(ARCH)


# project layout
#EXTERNAL_DIR = $(HOME)/Dropbox/shenoy_lab/code/rig5-external
EXTERNAL_DIR = $(HOME)/code/rig5-external
#CHAI_DIR = $(EXTERNAL_DIR)/chai3d-2.3.0
CHAI_DIR = $(EXTERNAL_DIR)/chai3d-3.0.0-RC1
CHAI_INC_DIR = $(CHAI_DIR)/src
#SDK_DIR = $(EXTERNAL_DIR)/sdk-3.4.5
#SDK_INC_DIR = $(SDK_DIR)/include
SDK_LIB_DIR = $(EXTERNAL_DIR)/chai3d-3.0.0-RC1/external/DHD/lib/$(ARCH)
SDK_INC_DIR = $(EXTERNAL_DIR)/chai3d-3.0.0-RC1/external/DHD/include 

GLEW_INC_DIR = $(CHAI_DIR)/external/glew/include

TOP_DIR = $(CHAI_DIR)


# include modified Chai makefile
include Makefile.common	
# platform-specific macros
include Makefile.common.$(OS)	

# use clang

# local configuration
SRC_DIR   = ./src
BIN_DIR   = ./bin
OBJ_DIR   = ./obj/$(CFG)/$(OS)-$(ARCH)

SOURCES   = $(wildcard $(SRC_DIR)/*.cpp  )
INCLUDES  = $(wildcard $(CHAI_INC_DIR)/*.h) $(wildcard $(SDK_INC_DIR)/*.h) $(wildcard $(SRC_DIR)/*.h)
OBJECTS   = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(notdir $(SOURCES)))
OUTPUT    = $(BIN_DIR)/hapticControllerPlane

all: $(OUTPUT)

# require .d file contents for each o
D_FILES = $(OBJECTS:.o=.d)
-include $(D_FILES)

LDFLAGS_EXT = $(LDFLAGS) -ldrd 

print-%:
	@echo '$*=$($*)'

# make the obj/ directory
$(OBJ_DIR):
	mkdir -p $@

# make the bin/ dir
$(BIN_DIR):
	mkdir -p $@

# link the executable
$(OUTPUT): $(BIN_DIR) $(OBJECTS)
	$(ECHO) "Linking $(OUTPUT)" $(ECHO_END)
	$(CXX) $(CXXFLAGS) $(INCLUDE_FLAGS) $(OBJECTS) $(LDFLAGS_EXT) $(LDLIBS) $(LDLIBS_GLUT) -o $(OUTPUT)

# compile cpp with .d dependency file as well
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(ECHO) "Compiling $<" $(ECHO_END)
	$(CXX) $(CXXFLAGS) $(INCLUDE_FLAGS) -MD -MF $(OBJ_DIR)/$*.d -c -o $@ $<

clean:
	$(ECHO) "Cleaning build" $(ECHO_END)
	@rm -f $(PROGS) $(OBJECTS) *~ TAGS core *.bak *.d #*#
	@rm -rf $(OBJ_DIR)
