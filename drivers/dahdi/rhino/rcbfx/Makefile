obj-m += rcbfx.o

rcbfx-objs := rcbfx_base.o 

$(obj)/rcbfx_base.o: $(src)/rcbfx.h $(src)/GpakApi.h $(src)/GpakCust.h

clean:
	@rm -f *.o
	@rm -f *.cmd *.o *.ko
	@rm -f .*.cmd .*.o
	@rm -rf .tmp_versions Module.symvers
	@rm -f core

