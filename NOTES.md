# Notes

## Network
Set the WiFi regulatory country or 5 GHz AP won't start (default regdomain "00" forbids it). Verify: `iw reg get`
```bash
sudo raspi-config nonint do_wifi_country US
```
```bash
sudo nmcli con add type wifi ifname wlan0 con-name "Rover-Hotspot" autoconnect yes ssid "Rover"
sudo nmcli con modify "Rover-Hotspot" 802-11-wireless.mode ap 802-11-wireless.band a 802-11-wireless.channel 36 ipv4.method shared
sudo nmcli con modify "Rover-Hotspot" wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<YOUR-PASSWORD>'
sudo nmcli con modify "Rover-Hotspot" ipv4.addresses 10.42.0.1/24
```
```bash
sudo nmcli con up Rover-Hotspot
```

## LiDAR
These edits live in `/boot/firmware/config.txt`

```bash
# Full USB current budget — or the RPLIDAR motor browns out / the port over-currents
usb_max_current_enable=1

# Hardware PWM on GPIO12/13 for the motor driver (24 kHz = silent, no audible whine)
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```

## Xbox Controller
The Xbox pad pairs but then immediately disconnects unless Bluetooth ERTM is off. The bluetooth module is loadable, so persist it via `modprobe.d`:
```bash
# apply now (module is already loaded)
echo Y | sudo tee /sys/module/bluetooth/parameters/disable_ertm
# persist across reboots
echo 'options bluetooth disable_ertm=Y' | sudo tee /etc/modprobe.d/xbox-bt.conf
```
Pair once (then trust so it auto-reconnects on boot):
```bash
bluetoothctl
  scan on            # note the "Xbox Wireless Controller" MAC
  pair  <MAC>
  trust <MAC>
  connect <MAC>
```


## Parts
- 50:1 Metal Gearmotor 37Dx70L mm 24V with 64 CPR Encoder (Helical Pinion) ($60.95 x 4 = $122)
    - https://www.pololu.com/product/4693
- Pololu Dual G2 High-Power Motor Driver 24v14 for Raspberry Pi ($79.95)
    - https://www.pololu.com/product/3753
- Two Dewalt 20V 5Ah Batteries + Charger ($125)
    - https://a.co/d/055lrbni
- Power Wheels Adapter for Dewalt 20V Battery ($10)
    - https://a.co/d/04fbQ5KM
- Raspberry Pi 5 - 8 GB RAM ($200)
    - https://www.adafruit.com/product/5813
- SLAMTEC RPLIDAR C1 (~$71.93)
    - https://www.robotshop.com/products/slamtec-rplidar-c1-360-dtof-laser-scanner
- OAK-D Lite ($169)
    - https://shop.luxonis.com/products/oak-d-lite-1
- ~~Robot Drive Wheel - 6 inch pneumatic tire ($10 x 4 = $40)~~
    - https://www.superdroidrobots.com/store/robot-parts/mechanical-parts/wheels-shafts/all-terrain-wheel-shafts/product=1988
- ~~ATR Shaft 6mm DM - 6 inch Tire ($14.48 x 4 = $57.92)~~
    - https://www.superdroidrobots.com/store/robot-parts/mechanical-parts/wheels-shafts/all-terrain-wheel-shafts/product=2078
- 24V to 5V Step Down ($10)
    - https://a.co/d/06zMBUDL
- Dual-Row Barrier Terminal Strip