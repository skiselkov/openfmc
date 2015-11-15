all : openfmc

OBJS=openfmc.o airac.o helpers.o geom.o
CFLAGS=$(shell pkg-config --cflags cairo) -W -Wall -Werror
LDFLAGS=$(shell pkg-config --libs cairo)

openfmc : $(OBJS)
	$(LINK.c) -o $@ $(OBJS)

%.o : %.c
	$(COMPILE.c) -o $@ $<

clean :
	rm -f openfmc $(OBJS)
