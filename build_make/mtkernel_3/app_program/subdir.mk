################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# app_program/*.c plus one level of subfolders (one per subsystem)
TEMP_SRCS = $(wildcard ../app_program/*.c) $(wildcard ../app_program/*/*.c)
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	@mkdir -p $(dir $@)
	$(GCC) $(CFLAGS) -std=c99 -Wall -Wextra -Werror -Wshadow -Wmissing-prototypes -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

# Application code follows BARR-C:2018; legacy kernel/SDK files keep their ABI.
.PHONY: app_style_check
app_style_check:
	@python3 ../tools/check_barr.py
	@python3 ../tools/generate_docs.py

$(subst ../, ./mtkernel_3/, $(TEMP_OBJS)): | app_style_check
all: app_style_check
