obj-m += r1t1.o

r1t1-objs := r1t1_base.o GpakApi.o GpakCust.o GpakDsp.o

$(obj)/r1t1_base.o: $(src)/r1t1.h $(src)/GpakApi.h $(src)/GpakCust.h

$(obj)/GpakCust.o: $(src)/GpakCust.h $(src)/r1t1.h

$(obj)/GpakApi.o: $(src)/GpakApi.h $(src)/GpakCust.h $(src)/r1t1.h

$(obj)/GpakDsp.o: $(src)/GpakDsp.fw $(obj)/r1t1_base.o
	@echo Making firmware object file for $(notdir $<)
	@cd $(src) && ../../../build_tools/make_firmware_object $(notdir $<) $@ $(obj)/r1t1_base.o

install-firmware:
ifeq ($(HOTPLUG_FIRMWARE),yes)
	@if [ -d $(INSTALL_PREFIX)/usr/lib/hotplug/firmware ]; then \
		install -m 644 GpakDsp.fw $(INSTALL_PREFIX)/usr/lib/hotplug/firmware; \
		echo "Rhino R1T1 firmware installed into $(INSTALL_PREFIX)/usr/lib/hotplug/firmware/"; \
	fi
	@if [ -d $(INSTALL_PREFIX)/lib/firmware ]; then \
		install -m 644 GpakDsp.fw $(INSTALL_PREFIX)/lib/firmware; \
		echo "Rhino R1T1 firmware installed into $(INSTALL_PREFIX)/lib/firmware/"; \
	fi
else
	@echo "Rhino R1T1 firmware not installed, hotplugging firmware is not supported by your kernel."
endif

clean:
	@rm -f *.o
	@rm -f *.cmd *.o *.ko
	@rm -f .*.cmd .*.o
	@rm -rf .tmp_versions Module.symvers
	@rm -f core

uninstall-firwamre:
	@if [ -d $(INSTALL_PREFIX)/usr/lib/hotplug/firmware ]; then \
		rm -fv $(INSTALL_PREFIX)/usr/lib/firmware/GpakDsp.fw; \
		echo "Rhino R1T1 firmware uninstalled from $(INSTALL_PREFIX)/usr/lib/hotplug/firmware/"; \
	fi
	@if [ -d $(INSTALL_PREFIX)/lib/firmware ]; then \
		rm -fv $(INSTALL_PREFIX)/lib/firmware/GpakDsp.fw; \
		echo "Rhino R1T1 firmware uninstalled from $(INSTALL_PREFIX)/lib/hotplug/firmware/"; \
	fi
