#/bin/bash

sudo touch /etc/udev/rules.d/imu.rules

sudo sh -c 'echo \"KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", MODE:=\"0666\", GROUP:=\"dialout\",  SYMLINK+=\"ch110\"\" > /etc/udev/rules.d/imu.rules'

sudo service udev restart

sudo service udev reload
