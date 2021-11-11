CC=g++
CFLAGS=-g3
LFLAGS=-lsystemc -lm
DEPS=A1P1.h A1P2.h
OBJ1=A1P1.o
OBJ2=A1P2.o
%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

.PHONY: clean

tlm: $(OBJ1)
	$(CC) -o $@ $^ $(CFLAGS) $(LFLAGS)

caat: $(OBJ2)
	$(CC) -o $@ $^ $(CFLAGS) $(LFLAGS)

clean:
	rm -f $(OBJ1) $(OBJ2) tlm caat
