# README

Install protobuf, websocket, websockets:
sudo apt install python3-protobuf python3-websocket python3-websocket

## Inlinometer plugin 

### Video example
[![Example](https://img.youtube.com/vi/qAAtmSkTd4U/sddefault.jpg)](https://www.youtube.com/watch?v=qAAtmSkTd4U)

This plugin can be configured in settings.json. It is posible to set two alert values. When alert is on, whole 
html on hudiy blinks red / or stay completly red.

Copy hudiy-inclinometer to /home/pi

Copy service hudiy-inclinometer.service to /etc/systemd/system/

Both service can be enabled / checked/  started with 
 - sudo systemctl enable hudiy-inclinometer
 - sudo systemctl start hudiy-inclinometer
 - systemctl status hudiy-inclinometer


### Add inclinometer to hudiy dash

edit file 
~/.hudiy/share/config/dashboards.json 
Example with inclinometer instead of phone

```
{
    "dashboards": [
        {
            "isDefault": true,
            "action": "hudiy_dashboard",
            "widgets": [
                {
                    "type": "date_time",
                    "size": "small_narrow"
                },
                {
                    "type": "navigation",
                    "size": "medium_narrow"
                },
                {
                    "type": "now_playing",
                    "size": "large_narrow"
                },
                {
                    "type": "web",
                    "size": "small_narrow",
                    "url": "file:///home/pi/hudiy-inclinometer/hudiy-inclinometer.html"
                }
            ]
        }
    ]
}

```

## volume-encoder and communication with arduino (steering-wheel, light-sensor...)

Copy folder hudiy-discovery2 to /home/pi/, because these paths are used in systemd services.

Copy services hudiy-car-interface.service and encoder-volume.service to /etc/systemd/system/
Both services can be enabled and started with systemctl

shutdown command must be enabled to run by user pi. So run command "sudo visudo" and add to the end:
pi ALL=(ALL) NOPASSWD: /sbin/shutdown, /sbin/poweroff, /sbin/reboot

If not exist yet, create new file in udev:
/etc/udev/rules.d/99-hudiy.rules

and add:

SUBSYSTEM=="usb", ATTR{idVendor}=="*", ATTR{idProduct}=="*", MODE="0666", GROUP="plugdev"

