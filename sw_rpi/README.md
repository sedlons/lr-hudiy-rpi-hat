# README
Copy folder hudiy-discovery2 to /home/pi/, because these paths are used in systemd services.

Copy services hudiy-car-interface.service and encoder-volume.service to /etc/systemd/system/
Both services can be enabled and started with systemd

shutdown command must be enabled to run by user pi. So run command "sudo visudo" and add to the end:
pi ALL=(ALL) NOPASSWD: /sbin/shutdown, /sbin/poweroff, /sbin/reboot

If not exist yet, create new file in udev:
/etc/udev/rules.d/99-hudiy.rules

and add:

SUBSYSTEM=="usb", ATTR{idVendor}=="*", ATTR{idProduct}=="*", MODE="0666", GROUP="plugdev"
