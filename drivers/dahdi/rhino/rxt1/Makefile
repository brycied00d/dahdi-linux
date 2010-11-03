obj-m += rxt1.o

FIRM_DIR        := ../../firmware
EXTRA_CFLAGS += -I$(src)/..  -w
rxt1-objs := rxt1_base.o GpakApi.o GpakCust.o GpakDsp.o

$(obj)/rxt1_base.o: $(src)/rxt1.h $(src)/GpakApi.h $(src)/GpakCust.h
$(obj)/GpakCust.o: $(src)/GpakCust.h $(src)/rxt1.h
$(obj)/GpakApi.o: $(src)/GpakApi.h $(src)/GpakCust.h $(src)/rxt1.h

$(obj)/GpakDsp.o: $(obj)/$(FIRM_DIR)/fw-rhino/GpakDsp.fw $(obj)/rxt1_base.o
	@echo Making firmware object file for $(notdir $<)
	sh $(obj)/$(FIRM_DIR)/fw-rhino/make_firmware_object $(obj)/$(FIRM_DIR)/fw-rhino/$(notdir $<) $@ $(obj)/../../dahdi.o


clean:
	@rm -f *.o
	@rm -f *.cmd *.o *.ko
	@rm -f .*.cmd .*.o
	@rm -rf .tmp_versions Module.symvers
	@rm -f core
