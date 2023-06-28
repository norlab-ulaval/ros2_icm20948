# ros2_icm20948
Driver for the ICM-20948 IMU

## Dependencies
```bash
pip3 install sparkfun-qwiic-icm20948
```

## Permissions
In order to run this node, i2c access permissions must be granted to the user that runs it. To do so run the following command: 
```bash
sudo adduser <your_user> i2c
```