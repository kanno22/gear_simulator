CC = gcc
CXX = g++
PKG = opencv4
CXXFLAGS= -std=c++17 $(CFLAGS) 
SRCS = model.cpp simulation.cpp RungeKutta.cpp main.cpp visualize.cpp timer.cpp 
SRCSDIR = ./src
OBJDIR = ./obj
OBJS = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(basename $(SRCS))))
LIBS += -lglut -lGLU -lGL -lm -pthread 
TARGET = simulate.exe

$(TARGET): $(OBJS)
	$(CXX) $^ $(LIBS) -o $@

$(OBJDIR)/%.o: $(SRCSDIR)/%.cpp
	@[ -d $(OBJDIR) ]
	$(CXX) -o $@ -c $<

clean:
	rm -rf ./*.o $(OBJDIR)/*.o optimize simulate.exe

.PHONY: clean
	