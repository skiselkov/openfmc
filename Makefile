all : openfmc

OBJS=openfmc.o airac.o helpers.o geom.o log.o list.o htbl.o route.o fms.o err.o \
    wmm.o GeomagnetismLibrary.o

DEPS=$(patsubst %.o, %.d, $(OBJS))
CFLAGS=$(shell pkg-config --cflags cairo) $(shell pkg-config --cflags libpng) \
    -W -Wall -Werror -O0 -g
ifeq ($(debug),no)
	CFLAGS += -O3
else
	CFLAGS += -DDEBUG
endif
LDFLAGS=$(shell pkg-config --libs cairo) $(shell pkg-config --libs libpng)

openfmc : $(OBJS)
	$(LINK.c) -o $@ $(OBJS)

%.o : %.c
	$(COMPILE.c) -MMD -o $@ $<

clean :
	rm -f openfmc $(OBJS) $(DEPS)

-include $(DEPS)
