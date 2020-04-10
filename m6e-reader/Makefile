M28X = 1
MYD  = 2
Board = M28X

# 1) Select cross toolchains
#CC=gcc
ifeq ($(Board), M28X)
CROSS = /opt/gcc-4.4.4-glibc-2.11.1-multilib-1.0/arm-fsl-linux-gnueabi/bin/arm-fsl-linux-gnueabi-
else
CROSS = /opt/myd_toolchains/gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
endif

CC = $(CROSS)gcc
STRIP = $(CROSS)strip
CFLAGS = -Wall -g -O2

# 2) What api ver you use now, just update don't move these files
YBT_INC_DIR := /include/YBT_include
INC_DIR := -I./include -I.$(YBT_INC_DIR)

# 3) Select the mercuryapi dir, must reference to 1)
YBT_LIB_PATH := ./lib/YBT

LIB_PATH := ./lib
LINK_LIB = -L$(LIB_PATH) -ldhcpcd -L$(YBT_LIB_PATH)
OBJ_DIR := ./obj
BIN_DIR := ./bin

EXEC = $(BIN_DIR)/YBT_reader
OBJS := $(OBJ_DIR)/reader.o \
		$(OBJ_DIR)/app_sys_setting.o \
		$(OBJ_DIR)/ind_cfg.o \
		$(OBJ_DIR)/ind_string.o \
		$(OBJ_DIR)/mid_msgq.o \
		$(OBJ_DIR)/mid_mutex.o \
		$(OBJ_DIR)/mid_net.o \
		$(OBJ_DIR)/mid_select.o \
		$(OBJ_DIR)/mid_task.o \
		$(OBJ_DIR)/mid_telnet.o \
		$(OBJ_DIR)/mid_timer.o \
		$(OBJ_DIR)/ringbuf.o \
		$(OBJ_DIR)/shell_play.o  \
		$(OBJ_DIR)/telnetd_port.o \
		$(OBJ_DIR)/osex_net.o \
		$(OBJ_DIR)/gpio_init.o \
		$(OBJ_DIR)/tcpserver.o \
		$(OBJ_DIR)/utils.o \
		$(OBJ_DIR)/serial.o \
		$(OBJ_DIR)/uart_raw.o \

all: clean $(EXEC) ; @echo OUTPUT into $(EXEC)

$(OBJ_DIR)/%.o: %.c $^
	$(CC) -c -o $@ $<

$(EXEC):$(OBJS)
	$(CC) $(INC_DIR) -o $@ $(OBJS) $(LINK_LIB) -lrt -lpthread 

clean:
	rm -f $(EXEC) $(OBJ_DIR)/*.o
