#
# Makefile for the Cypress touchkey driver
#

ifeq ($(CONFIG_KEYBOARD_CYPRESS_TOUCH),y)
ifeq ($(CONFIG_AGNI_PURECM_MODE),y)
obj-$(CONFIG_KEYBOARD_CYPRESS_TOUCH) += cypress-touchkey_cm.o issp_driver_routines.o issp_main.o issp_routines.o
else
obj-$(CONFIG_KEYBOARD_CYPRESS_TOUCH) += cypress-touchkey.o issp_driver_routines.o issp_main.o issp_routines.o
endif
endif

