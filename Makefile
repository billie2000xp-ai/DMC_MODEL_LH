obj-y  = AddressMapping.o ClockDomain.o Inline_ECC.o MemorySystem.o Transaction.o write_buffer.o TimingCalculate.o
obj-y += BankState.o IniReader.o LPMemorySystemTop.o MemoryController.o Rank.o
obj-y += parameter/

# 1. names
EXE_NAME=LPDRAMSim
STATIC_LIB_NAME := liblpddrsim.a
LIB_NAME=liblpddrsim.so
LIB_NAME_A=liblpddrsim.a
LIB_NAME_MACOS=liblpddrsim.dylib


SRC = $(wildcard *.cpp)
OBJ = $(addsuffix .o, $(basename $(SRC)))
INI_SRC = $(wildcard ./parameter/*.ini)
INI_OBJ = $(patsubst %.ini,%.o,$(INI_SRC))
EMBED_S = ./embed_asm.S

LIB_SRC := $(filter-out TraceBasedSim.cpp,$(SRC))
LIB_OBJ := $(addsuffix .o, $(basename $(LIB_SRC)))

#build portable objects (i.e. with -fPIC)
POBJ = $(addsuffix .po, $(basename $(LIB_SRC)))

REBUILDABLES=$(OBJ) ${POBJ} $(EXE_NAME) $(LIB_NAME) $(STATIC_LIB_NAME)


# 2. compiler flags
CXXFLAGS=-DNO_STORAGE -Wall -g -O0 -DDEBUG_BUILD -fPIC -std=c++0x
OPTFLAGS=${SOC_CFLAG}

ifdef DEBUG
ifeq ($(DEBUG), 1)
OPTFLAGS= -O0 -g
endif
endif
CXXFLAGS+=$(OPTFLAGS)

DEPDIR := .d
$(shell if not exist $(DEPDIR) mkdir $(DEPDIR) >nul 2>nul)
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.Td
BUILDSTATICLIB := ar -rcs

CC          := g++ -O3
CXX			:= g++ -O3
BB          := ar
RM			:= del /Q /F

COMPILE.c = $(CC) $(DEPFLAGS) $(CFLAGS) $(CPPFLAGS)  $(GPROF_OPT) $(INCPATH) -c
COMPILE.cc = $(CXX) $(DEPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(GPROF_OPT) $(INCPATH) -c
POSTCOMPILE = @copy /Y $(DEPDIR)\$*.Td $(DEPDIR)\$*.d >nul && type nul >> $@

%.o : %.cpp
%.o : %.cpp $(DEPDIR)/%.d
	$(COMPILE.cc) $(OUTPUT_OPTION) $<
	$(POSTCOMPILE)

# 8. Default
default:
	@echo "=========================================================================================="
	@echo " COMMAND                        | DESCRIPTION"
	@echo "================================+========================================================="
	@echo " make ini_obg or 0              | Compile ini.o For Linxi"
	@echo " make compile or 1              | Compile For LPDDR UT"
	@echo " make sim or 2                  | Run the simulation with output log print to terminal"
	@echo " make sim_log or 3              | Run the simulation with output log print to run.log"
	@echo " make compile_so or 4           | Compile Dynamic Library File For ESL System"
	@echo " make compile_a or 5            | Compile Static Library File For ESL System"
	@echo " make clean or 6                | Not delete LPDRAMSim"
	@echo " make clean_all or 7            | Delete the temp file"
	@echo " make all or 8                  | run compile -> sim_log"
	@echo " make leak or 9                 | run leak -> leak.log"
	@echo " make qcachegrind or 10         | run qcachegrind -> qcachegrind.log"
	@echo " make qcachegrind_log or 11     | run qcachegrind qcachegrind.log"
	@echo " make callgrind_control or 12   | update qcachegrind.log"
	@echo " make perf_stat or 13           | run perf stat"
	@echo " make perf_record or 14         | run perf record"
	@echo " make perf_report or 15         | run perf report"
	@echo "=========================================================================================="

ini_obg 0: ${INI_OBJ}

$(INI_OBJ): %.o: %.ini
	g++ -DFILE='"$<"' -c -o $@ $(EMBED_S)

compile 1: ${LIB_NAME} $(EXE_NAME)
	@mkdir log 2>nul || exit /b 0

#   $@ target name, $^ target deps, $< matched pattern
$(EXE_NAME): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ 
	@echo "Built $@ successfully" 

$(LIB_NAME): $(OBJ)
	g++ -shared -Wl,-soname,$@ -o $@ $^
	@echo "Built $@ successfully"

compile_so 4: $(OBJ)
	g++ -shared -Wl,-soname,$(LIB_NAME) -o $(LIB_NAME) AddressMapping.o BankState.o ClockDomain.o IniReader.o Inline_ECC.o LPMemorySystemTop.o MemoryController.o MemorySystem.o Rank.o Transaction.o write_buffer.o TimingCalculate.o Rmw.o
	@echo "Built $(LIB_NAME) successfully"

compile_a 5: $(OBJ)
	ar -rc $(LIB_NAME_A) AddressMapping.o BankState.o ClockDomain.o IniReader.o Inline_ECC.o LPMemorySystemTop.o MemoryController.o MemorySystem.o Rank.o Transaction.o write_buffer.o TimingCalculate.o Rmw.o

$(STATIC_LIB_NAME): $(LIB_OBJ)
	$(AR) crs $@ $^

$(LIB_NAME_MACOS): $(POBJ)
	g++ -dynamiclib -o $@ $^
	@echo "Built $@ successfully"

clean 6:
	-$(RM) $(subst /,\,$(OBJ))
	-$(RM) $(subst /,\,$(INI_OBJ))
	-$(RM) $(LIB_NAME)
	-del /Q /F run.log leak.log qcachegrind.log perf.data 2>nul
	-if exist log rmdir /S /Q log
	-if exist .d rmdir /S /Q .d

clean_all 7:
	-$(RM) $(subst /,\,$(OBJ))
	-$(RM) $(subst /,\,$(INI_OBJ))
	-$(RM) $(EXE_NAME) $(LIB_NAME)
	-del /Q /F run.log leak.log qcachegrind.log perf.data 2>nul
	-if exist log rmdir /S /Q log
	-if exist .d rmdir /S /Q .d

# 9. Auto dependency generation 
$(DEPDIR)/%.d: ;
.PRECIOUS: $(DEPDIR)/%.d
include $(wildcard $(patsubst %,$(DEPDIR)/%.d,$(basename $(SRC))))

# 10. UT sim
sim 2:
	@mkdir -p log
	LPDRAMSim

# 10. UT sim with run.log
sim_log 3:
	@mkdir -p log
	LPDRAMSim &> run.log

all 8: compile sim_log

leak 9:
	@mkdir -p log
	@valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --error-limit=no --verbose --log-file=leak.log LPDRAMSim

qcachegrind 10:
	@valgrind --tool=callgrind --trace-children=yes --callgrind-out-file=./qcachegrind.log LPDRAMSim

qcachegrind_log 11:
	@~/tools/qcachegrind qcachegrind.log

callgrind_control 12:
	@callgrind_control -d

perf_stat 13:
	@perf stat LPDRAMSim

perf_record 14:
	@perf record -F 999 LPDRAMSim

perf_report 15:
	@perf report