#
# Makefile for the vibrator driver ICs with vibrator driver
#

ifeq ($(CONFIG_MOTOR_DRV_MAX8997),y)
ifeq ($(CONFIG_AGNI_PURECM_MODE),y)
obj-$(CONFIG_MOTOR_DRV_MAX8997)		+= max8997_vibrator_cm.o
else
obj-$(CONFIG_MOTOR_DRV_MAX8997)		+= max8997_vibrator.o
endif
endif

ifeq ($(CONFIG_MOTOR_DRV_MAX77693),y)
ifeq ($(CONFIG_AGNI_PURECM_MODE),y)
obj-$(CONFIG_MOTOR_DRV_MAX77693)	+= max77693_haptic_cm.o
else
obj-$(CONFIG_MOTOR_DRV_MAX77693)	+= max77693_haptic.o
endif
endif

obj-$(CONFIG_MOTOR_DRV_ISA1200)		+= isa1200_vibrator.o
obj-$(CONFIG_MOTOR_DRV_DRV2603)		+= drv2603_vibrator.o
obj-$(CONFIG_DC_MOTOR)			+= dc_motor.o

obj-$(CONFIG_VIBETONZ) 			+= vibrator.o
vibrator-objs 				+= tspdrv.o
