I2C Data Capture
========================================

![I2C Traffic from VL53L1X](https://github.com/sparkfunX/Qwiic_Distance_VL53L1X/raw/master/DataCapture/I2C%20Traffic.jpg)

ST has chosen not to release a full datasheet for the VL53L1X so we must reverse engineer how to talk to the sensor and get measurements. Luckily, ST has provided a C++ example for their STM32 line of processors as well as the X-NUCLEO-53L1A1 development and evaluation board for the VL53L1X. 

SparkFun has written a library based on the I2C traffic between the STM32F401RET6 processor on the Nucleo-64. This zip file contains the I2C traffic recorded when the sensor is configured and activated while using ST's Windows example program (found under [STSW-IMG007](http://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl53l1x.html#sw-tools-scroll)). The I2C data was found using a Saleae Logic 4. We've provided a CSV version so that you can use [PulseView](https://sigrok.org/wiki/PulseView) or other open source logic analyzation software.

We would love your help to improve this library. There are many features to this sensor including calibrating cross talk compensation, enabling reference SPADs, establishing the offset, etc. Please create issues and PRs as you go and we'll help out.

