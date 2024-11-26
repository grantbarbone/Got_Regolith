#!/bin/bash

echo "0.0 0.0" > store_coord.txt
sudo usermod -aG dialout $USER
while true 
do
	./ZED_Tutorial_1
done

