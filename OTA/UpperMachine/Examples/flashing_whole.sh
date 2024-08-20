#!/bin/bash
sudo ./upper_monitor.out /dev/ttysWK3 115200 send cm0plus_app.bin 1 
sleep 6
sudo ./upper_monitor.out /dev/ttysWK3 115200 send cm7_0_app.bin 2
sleep 6
sudo ./upper_monitor.out /dev/ttysWK3 115200 send cm7_1_app.bin 3