CC      = gcc
CFLAGS  = -Wall -O2
LDLIBS  = -lglut -lGLU -lGL -lm

robokick: robokick.c
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f robokick
