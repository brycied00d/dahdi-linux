obj-m += r1t1.o

r1t1-objs := r1t1_base.o 

$(obj)/r1t1_base.o: $(src)/r1t1.h $(src)/GpakApi.h $(src)/GpakCust.h


clean:
	@rm -f *.o
	@rm -f *.cmd *.o *.ko
	@rm -f .*.cmd .*.o
	@rm -rf .tmp_versions Module.symvers
	@rm -f core

