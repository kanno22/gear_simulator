CC = gcc
CXX = g++
PKG = opencv4
CFLAGS = 
CXXFLAGS= -std=c++17 $(CFLAGS) 
SRCS = model.cpp simulation.cpp RungeKutta.cpp main.cpp visualize.cpp timer.cpp key_control.cpp
SRCSDIR = ../
OBJDIR = ../obj
OBJS = $(addprefix ../obj/, $(addsuffix .o, $(basename $(SRCS))))
LIBS += -lglut -lGLU -lGL -lm -pthread 
TARGET = simulation
$(TARGET): $(OBJS)
	$(CXX) $^ $(LIBS) -o $@

.PHONY: clean

# $(OBJDIR)/%.o: %.cpp
# 	$(CXX) $^ $(LIBS) -o $@

$(OBJDIR)/%.o: $(SRCSDIR)%.cpp
	@[ -d $(OBJDIR) ]
	$(CXX) -o $@ -c $<

clean:
	rm -rf ../*.o ../obj/*.o optimize simulation

optimize: $(OBJS_OPT)
	$(CXX) $^ $(LIBS) -o $@	