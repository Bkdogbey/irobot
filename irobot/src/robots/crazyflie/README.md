# USB Radio and Crazyflie 2 Setup on Linux

This guide explains how to set up your USB Radio and Crazyflie 2 to be used over USB without requiring root privileges on a Linux system.

## Prerequisites

Ensure you are running a Linux distribution that supports `udev` and you have root (sudo) privileges.

## Steps

### 1. Add User to `plugdev` Group

First, create a group called `plugdev` and add your user to it. This will allow non-root access to USB devices.

```bash
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER
```

After running these commands, log out and log back in to ensure your user is properly added to the `plugdev` group.

### 2. Create `99-bitcraze.rules`

Now, create the udev rule file that defines the necessary permissions for the USB devices (Crazyradio and Crazyflie).

You can use a text editor like `nano`, `gedit`, or `subl` to create the file:

* **Using `nano`:**

  ```bash
  sudo nano /etc/udev/rules.d/99-bitcraze.rules
  ```
* **Using `gedit`:**

  ```bash
  sudo gedit /etc/udev/rules.d/99-bitcraze.rules
  ```
* **Using `subl` (Sublime Text):**

  ```bash
  sudo subl /etc/udev/rules.d/99-bitcraze.rules
  ```

Once the editor is open, paste the following content:

```plaintext
# Crazyradio (normal operation)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
# Crazyflie (over USB)
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
```

Save the file and close the editor.

### 3. Reload udev Rules

To apply the changes and reload the udev rules, run the following commands:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Final Notes

Once these steps are completed, your USB Radio and Crazyflie 2 devices should be accessible over USB without requiring root privileges.

For more detailed information about the Crazyflie 2 and its Python library, visit the [Crazyflie Python library documentation](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/).
