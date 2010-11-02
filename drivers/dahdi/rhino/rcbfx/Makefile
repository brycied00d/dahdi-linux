obj-m += rcbfx.o

$(obj)/rcbfx_base.o: $(src)/rcbfx.h $(src)/GpakApi.h $(src)/GpakCust.h

rcbfx-objs := rcbfx_base.o GpakApi.o GpakCust.o

$(obj)/rcbfx_base.o: $(src)/rcbfx.h $(src)/GpakApi.h $(src)/GpakCust.h
$(obj)/GpakCust.o: $(src)/GpakCust.h $(src)/rcbfx.h
$(obj)/GpakApi.o: $(src)/GpakApi.h $(src)/GpakCust.h $(src)/rcbfx.h


clean:
	@rm -f *.o
	@rm -f *.cmd *.o *.ko
	@rm -f .*.cmd .*.o
	@rm -rf .tmp_versions Module.symvers
	@rm -f core

