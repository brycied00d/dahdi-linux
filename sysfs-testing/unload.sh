#!/bin/bash
dahdi_cfg -c /usr/src/sruffell/dahdi-linux/sysfs-testing/dynamic.conf -s
/etc/init.d/dahdi stop
