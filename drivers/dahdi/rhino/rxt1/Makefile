obj-m += rxt1.o

rxt1-objs := rxt1_base.o

$(obj)/rxt1_base.o: $(src)/rxt1.h $(src)/GpakApi.h $(src)/GpakCust.h

clean:
	@rm -f *.o
	@rm -f *.cmd *.o *.ko
	@rm -f .*.cmd .*.o
	@rm -rf .tmp_versions Module.symvers
	@rm -f core
