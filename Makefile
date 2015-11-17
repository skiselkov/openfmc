all : openfmc

OBJS=openfmc.o airac.o helpers.o geom.o log.o list.o htbl.o libxxhash.o

DEPS=$(patsubst %.o, %.d, $(OBJS))
CFLAGS=$(shell pkg-config --cflags cairo) -W -Wall -Werror -g
LDFLAGS=$(shell pkg-config --libs cairo)

openfmc : $(OBJS)
	$(LINK.c) -o $@ $(OBJS)

%.o : %.c
	$(COMPILE.c) -MMD -o $@ $<

clean :
	rm -f openfmc $(OBJS) $(DEPS)

-include $(DEPS)
