
TARGET = main_exec
OBJS := ./*.o

all:$(TARGET)

$(TARGET): $(OBJS)
	$(CC)  *.c -o $(TARGET) -lm

clean:
	rm -f $(TARGET) ./*.o ./*.h.gch 

clear:
	rm -f $(filter-out main.o, $(wildcard *.o)) ./*.h.gch