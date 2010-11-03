obj-m += rcbfx.o

FIRM_DIR        := ../../firmware
EXTRA_CFLAGS += -I$(src)/..  -w

$(obj)/rcbfx_base.o: $(src)/rcbfx.h $(src)/GpakApi.h $(src)/GpakCust.h
rcbfx-objs := rcbfx_base.o GpakApi.o GpakCust.o DspLoader.o GpakDsp10.o GpakDsp0704.o GpakDsp0708.o

$(obj)/rcbfx_base.o: $(src)/rcbfx.h $(src)/GpakApi.h $(src)/GpakCust.h
$(obj)/GpakCust.o: $(src)/GpakCust.h $(src)/rcbfx.h
$(obj)/GpakApi.o: $(src)/GpakApi.h $(src)/GpakCust.h $(src)/rcbfx.h


$(obj)/DspLoader.o: $(obj)/$(FIRM_DIR)/fw-rhino/DspLoader.fw $(obj)/rcbfx_base.o
	@echo Making firmware object file for $(notdir $<)
	@sh $(obj)/$(FIRM_DIR)/fw-rhino/make_firmware_object $(obj)/$(FIRM_DIR)/fw-rhino/$(notdir $<) $@ $(obj)/../../dahdi.o
$(obj)/GpakDsp10.o: $(obj)/$(FIRM_DIR)/fw-rhino/GpakDsp10.fw $(obj)/rcbfx_base.o
	@echo Making firmware object file for $(notdir $<)
	@sh $(obj)/$(FIRM_DIR)/fw-rhino/make_firmware_object $(obj)/$(FIRM_DIR)/fw-rhino/$(notdir $<) $@ $(obj)/../../dahdi.o
$(obj)/GpakDsp0704.o: $(obj)/$(FIRM_DIR)/fw-rhino/GpakDsp0704.fw $(obj)/rcbfx_base.o
	@echo Making firmware object file for $(notdir $<)
	@sh $(obj)/$(FIRM_DIR)/fw-rhino/make_firmware_object $(obj)/$(FIRM_DIR)/fw-rhino/$(notdir $<) $@ $(obj)/../../dahdi.o
$(obj)/GpakDsp0708.o: $(obj)/$(FIRM_DIR)/fw-rhino/GpakDsp0708.fw $(obj)/rcbfx_base.o
	@echo Making firmware object file for $(notdir $<)
	@sh $(obj)/$(FIRM_DIR)/fw-rhino/make_firmware_object $(obj)/$(FIRM_DIR)/fw-rhino/$(notdir $<) $@ $(obj)/../../dahdi.o


clean:
	@rm -f *.o
	@rm -f *.cmd *.o *.ko
	@rm -f .*.cmd .*.o
	@rm -rf .tmp_versions Module.symvers
	@rm -f core

