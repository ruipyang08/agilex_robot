#/bin/bash
echo 'KERNEL=="ttyUSB*",ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",MODE:="0666",SYMLINK+="DH_hand"' >> /etc/udev/rules.d/dh_hand.rules

service udev reload
sleep 2
service udev restart
