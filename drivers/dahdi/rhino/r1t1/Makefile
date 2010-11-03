obj-m += r1t1.o
FIRM_DIR	:= ../../firmware
EXTRA_CFLAGS += -I$(src)/..  -w

$(obj)/r1t1_base.o: $(src)/r1t1.h $(src)/GpakApi.h $(src)/GpakCust.h

$(obj)/GpakCust.o: $(src)/GpakCust.h $(src)/r1t1.h

$(obj)/GpakApi.o: $(src)/GpakApi.h $(src)/GpakCust.h $(src)/r1t1.h

$(obj)/GpakDsp.o: $(obj)/$(FIRM_DIR)/fw-rhino/GpakDsp.fw $(obj)/r1t1_base.o
	@echo Making firmware object file for $(notdir $<)
	@sh $(obj)/$(FIRM_DIR)/fw-rhino/make_firmware_object $(obj)/$(FIRM_DIR)/fw-rhino/$(notdir $<) $@ $(obj)/../../dahdi.o

r1t1-objs := r1t1_base.o GpakApi.o GpakCust.o GpakDsp.o


clean:
	@rm -f *.o
	@rm -f *.cmd *.o *.ko
	@rm -f .*.cmd .*.o
	@rm -rf .tmp_versions Module.symvers
	@rm -f core

