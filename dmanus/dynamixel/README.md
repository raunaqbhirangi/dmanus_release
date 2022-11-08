
# Python wrapper for dynamixels
Primarily designed for MX series

# Set up
0. Clone the [dynamixel robotis repo](https://github.com/ROBOTIS-GIT/DynamixelSDK.git). 
Note that the repo has gone through changes that aren't backward compatible. Checkout an earlier commit compatibility .
```
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git  
cd DynamixelSDK
git checkout 2ae37fc2390eedcd8fe356f40ece398720db0532 
```

1. build the c library

```
cd ~/Libraries/DynamixelSDK/c/build/linux64/ 
make 
```

2. add path to the ctype wrapper in the bashrc 

```export PYTHONPATH="/home/vik/Libraries/DynamixelSDK/python/dynamixel_functions_py:$PYTHONPATH"``` 

3. Edit `DynamixelSDK/python/dynamixel_functions_py/dynamixel_functions.py` to point to the right libraries (absolute path)


```dxl_lib = cdll.LoadLibrary("/home/vik/Libraries/DynamixelSDK/c/build/linux64/libdxl_x64_c.so")```

4. Edit user permissions to access ports

```
sudo usermod -a -G tty yourname
sudo usermod -a -G dialout yourname
```
Note that logout/login will be required for the group addition to take effect.

5. Port's latency can significantly affect your performance. You can check its value by:
 ```
 $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
 ```

 If you think that the communication is too slow, type following after plugging the usb in to change the latency timer

Method 1. 
 ```Type following (you should do this everytime when the usb once was plugged out or the connection was dropped)
 $ echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
 $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
 ```

 Method 2. 
 ```If you want to set it as be done automatically, and don't want to do above everytime, make rules file in /etc/udev/rules.d/. For example,
 $ echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"1\" > 99-dynamixelsdk-usb.rules
 $ sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
 ```
 Use commands below to reload the rules, or log-off+login for the new rules to take effect.
 ```
 $ sudo udevadm control --reload-rules
 $ sudo udevadm trigger --action=add
 $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
 ```

6. Create identifiable link for your usb devices now. Check the serial of the device using
```
udevadm info -a -p  $(udevadm info -q path -n /dev/ttyUSBX) | grep serial
```
Now add a synlink to your udev file `/etc/udev/rules.d/99-dynamixel-usb.rules`. The final file will look something like this
```
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
ACTION=="add", SUBSYSTEM=="tty", ENV{ID_MODEL_ID}=="6014", ENV{ID_VENDOR_ID}=="0403", ENV{ID_SERIAL_SHORT}=="FT3R4CCT", SYMLINK+="DKitty"
ACTION=="add", SUBSYSTEM=="tty", ENV{ID_MODEL_ID}=="6014", ENV{ID_VENDOR_ID}=="0403", ENV{ID_SERIAL_SHORT}=="FT2H2MX4", SYMLINK+="DLeg"
```
Note that logout/login will be required for the new rules to take effect. Alternatively one can also use `udevadm control -R`


# usage
1. Open dynamixel_utils.py and pick the connected dynamixel type 
2. `python dynamixel_utils.py`to test
