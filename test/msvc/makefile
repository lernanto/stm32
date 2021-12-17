CC = cl.exe /c
LINK = link

CMSIS_ROOT = D:\git\CMSIS_5\CMSIS
LIB_ROOT = ..\..\lib
INC = /I . /I $(LIB_ROOT) /I $(CMSIS_ROOT)\DSP\Include /I $(CMSIS_ROOT)\DSP\PrivateInclude
CFLAGS = $(INC)

all: test_dsp.exe

clean:
	del test_dsp.exe *.obj

test_dsp.obj: test_dsp.c
	$(CC) $(CFLAGS) $**

log.obj: $(LIB_ROOT)\log.c
	$(CC) $(CFLAGS) $**

dsp.obj: $(LIB_ROOT)\dsp.c
	$(CC) $(CFLAGS) $**

BasicMathFunctions.obj: $(CMSIS_ROOT)\DSP\Source\BasicMathFunctions\BasicMathFunctions.c
	$(CC) $(CFLAGS) $**

CommonTables.obj: $(CMSIS_ROOT)\DSP\Source\CommonTables\CommonTables.c
	$(CC) $(CFLAGS) $**

DistanceFunctions.obj: $(CMSIS_ROOT)\DSP\Source\DistanceFunctions\DistanceFunctions.c
	$(CC) $(CFLAGS) $**

FastMathFunctions.obj: $(CMSIS_ROOT)\DSP\Source\FastMathFunctions\FastMathFunctions.c
	$(CC) $(CFLAGS) $**

MatrixFunctions.obj: $(CMSIS_ROOT)\DSP\Source\MatrixFunctions\MatrixFunctions.c
	$(CC) $(CFLAGS) $**

StatisticsFunctions.obj: $(CMSIS_ROOT)\DSP\Source\StatisticsFunctions\StatisticsFunctions.c
	$(CC) $(CFLAGS) $**

SupportFunctions.obj: $(CMSIS_ROOT)\DSP\Source\SupportFunctions\SupportFunctions.c
	$(CC) $(CFLAGS) $**

test_dsp.exe: \
	test_dsp.obj \
	log.obj \
	dsp.obj \
	BasicMathFunctions.obj \
	CommonTables.obj \
	FastMathFunctions.obj \
	DistanceFunctions.obj \
	MatrixFunctions.obj \
	StatisticsFunctions.obj \
	SupportFunctions.obj
	$(LINK) -out:test_dsp.exe $**