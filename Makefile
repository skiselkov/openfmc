all : openfmc

OBJS=wmm.o GeomagnetismLibrary.o list.o \
    helpers.o htbl.o geom.o err.o log.o \
    airac.o route.o fms.o \
    openfmc.o

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
