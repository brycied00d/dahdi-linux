#!/bin/bash
modprobe dahdi_dynamic_loc
sleep 1
dahdi_cfg -c /usr/src/sruffell/dahdi-linux/sysfs-testing/dynamic.conf
