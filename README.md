# ros2_robotiq_gripper

This repository contains the ROS 2 driver, controller and description packages for working with a Robotiq Gripper.

## Dependency: serial communication library

This project requires the communication library "serial" from the Borg-Robotics repository (branch `ros2`): https://github.com/Borg-Robotics/serial/tree/ros2

## Hardware Setup: Robotiq 2F-85 Dual Gripper Configuration

This section describes how to set up two Robotiq 2F-85 grippers connected via USB-to-Serial (FTDI FT232) adapters on an Ubuntu host.

### USB Device Identification

Each Robotiq 2F-85 gripper uses an FTDI FT232 USB-to-Serial converter (Vendor ID: `0403`, Product ID: `6001`). When two grippers are connected, they appear as `/dev/ttyUSB0` and `/dev/ttyUSB1`, but these assignments can swap on reboot. To get stable, predictable device paths, we use udev rules based on the unique FTDI serial number of each adapter.

To find the serial numbers of your FTDI adapters, run:

```bash
for dev in /dev/ttyUSB*; do
  echo "=== $dev ==="
  udevadm info -a -n "$dev" | grep -E '{idVendor}|{idProduct}|{serial}' | head -3
done
```

### Udev Rules

Create a udev rules file to assign stable symlinks to each gripper:

```bash
sudo nano /etc/udev/rules.d/99-robotiq-grippers.rules
```

Add the following rules (replace the serial numbers with your own if different):

```
# Robotiq 2F-85 Gripper - Left Arm (serial: BG018PFD)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="BG018PFD", SYMLINK+="robotiq_gripper_left", MODE="0666"

# Robotiq 2F-85 Gripper - Right Arm (serial: BG03U2R5)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="BG03U2R5", SYMLINK+="robotiq_gripper_right", MODE="0666"
```

Reload the udev rules and trigger them:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Verify the symlinks were created:

```bash
ls -la /dev/robotiq_gripper_*
```

You should see:

```
/dev/robotiq_gripper_left -> ttyUSB0
/dev/robotiq_gripper_right -> ttyUSB1
```

### Gripper Modbus Slave Addresses

Each gripper must have a unique Modbus slave address. Our grippers are configured as follows:

| Gripper | Symlink | FTDI Serial | Slave Address |
|---------|---------|-------------|---------------|
| Left Arm | `/dev/robotiq_gripper_left` | `BG018PFD` | `2` (`0x02`) |
| Right Arm | `/dev/robotiq_gripper_right` | `BG03U2R5` | `1` (`0x01`) |

### Testing the Grippers

First, build the hardware test package:

```bash
colcon build --packages-select robotiq_hardware_tests
source install/setup.bash
```

Test each gripper individually:

```bash
# Test left gripper
ros2 run robotiq_hardware_tests full_test --port /dev/robotiq_gripper_left --slave-address 2

# Test right gripper
ros2 run robotiq_hardware_tests full_test --port /dev/robotiq_gripper_right --slave-address 1
```

A successful test will:
1. Connect to the gripper
2. Deactivate and reactivate the gripper
3. Close, open, half-close, and open the gripper
4. Test speed changes (slow close, fast open)