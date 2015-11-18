all : openfmc

OBJS=openfmc.o airac.o helpers.o geom.o log.o list.o htbl.o

DEPS=$(patsubst %.o, %.d, $(OBJS))
CFLAGS=$(shell pkg-config --cflags cairo) -W -Wall -Werror -O2 -g
ifeq ($(debug),yes)
	CFLAGS += -DDEBUG
else
	CFLAGS += -O3
endif
LDFLAGS=$(shell pkg-config --libs cairo)

openfmc : $(OBJS)
	$(LINK.c) -o $@ $(OBJS)

%.o : %.c
	$(COMPILE.c) -MMD -o $@ $<

clean :
	rm -f openfmc $(OBJS) $(DEPS)

-include $(DEPS)
