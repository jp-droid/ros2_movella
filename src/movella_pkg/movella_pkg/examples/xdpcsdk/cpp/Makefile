MOVELLA_INSTALL_PREFIX?=$(shell dirname $$(dirname $$(dirname $$(pwd))))
BASIC_CFLAGS := -g -Wall -Wextra
INCLUDE:=-I$(MOVELLA_INSTALL_PREFIX)/include -I$(MOVELLA_INSTALL_PREFIX)/include/movelladot_pc_sdk
LFLAGS:=-lm -lmovelladot_pc_sdk -lxstypes -lpthread -L$(MOVELLA_INSTALL_PREFIX)/lib '-Wl,-rpath,$$ORIGIN:$(MOVELLA_INSTALL_PREFIX)/lib'

CFLAGS:=$(BASIC_CFLAGS) $(INCLUDE) $(CFLAGS)
CXXFLAGS:=$(BASIC_CFLAGS) -std=c++17 $(INCLUDE) $(CXXFLAGS)

TARGETS:=example_receive_data example_write example_firmware_update example_magnetic_field_mapper example_data_export example_onboard_recording example_synchronization
all: $(TARGETS)

example_receive_data: example_receive_data.cpp xdpchandler.cpp.o conio.c.o
example_write: example_write.cpp xdpchandler.cpp.o conio.c.o
example_firmware_update: example_firmware_update.cpp xdpchandler.cpp.o conio.c.o
example_magnetic_field_mapper: example_magnetic_field_mapper.cpp xdpchandler.cpp.o conio.c.o
example_onboard_recording: example_onboard_recording.cpp xdpchandler.cpp.o conio.c.o
example_data_export: example_data_export.cpp xdpchandler.cpp.o conio.c.o
example_synchronization: example_synchronization.cpp xdpchandler.cpp.o conio.c.o

$(TARGETS):
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LFLAGS)

-include $(FILES:.cpp=.dpp)
%.cpp.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@
	@$(CXX) -MM $(CXXFLAGS) $< > $*.dpp
	@mv -f $*.dpp $*.dpp.tmp
	@sed -e 's|.*:|$*.cpp.o:|' < $*.dpp.tmp > $*.dpp
	@sed -e 's/.*://' -e 's/\\$$//' < $*.dpp.tmp | fmt -1 | \
	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.dpp
	@rm -f $*.dpp.tmp

-include $(FILES:.c=.d)
%.c.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@
	@$(CC) -MM $(CFLAGS) $< > $*.d
	@mv -f $*.d $*.d.tmp
	@sed -e 's|.*:|$*.c.o:|' < $*.d.tmp > $*.d
	@sed -e 's/.*://' -e 's/\\$$//' < $*.d.tmp | fmt -1 | \
	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.d
	@rm -f $*.d.tmp

clean:
	-$(RM) *.o *.d *.dpp $(TARGETS) $(PREBUILDARTIFACTS)
