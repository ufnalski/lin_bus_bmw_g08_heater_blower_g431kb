# BMW G08 heater blower motor drive (STM32G431KB)
After completing the steps related to [the RX scanner](https://github.com/ufnalski/lin_bus_rx_scanner_g431kb) and [the TX scanner](https://github.com/ufnalski/lin_bus_tx_scanner_g431kb), we are ready to measure voltage at the drive terminals, current drawn by the drive, and speed of the fun. This is to identify parts of the payload (potentially single bytes) that follow the trends.

![Valeo/BMW blower in action](/Assets/Images/bmw_blower_lin_bus_in_action.jpg)
![Valeo/BMW blower serial monitor](/Assets/Images/bmw_blower_serial_monitor.JPG)

# Missing files?
Don't worry :slightly_smiling_face: Just log in to MyST and hit Alt-K to generate /Drivers/CMCIS/ and /Drivers/STM32G4xx_HAL_Driver/ based on the .ioc file. After a couple of seconds your project will be ready for building.

# What next?
Play with a LIN slave device [^1], e.g. emulate the blower, or even develop its digital twin[^2]. Use the already developed LIN master node to test your slave device - it should be indistinguishable from the physical blower to the master device. Fingers crossed!

[^1]: [STM32 UART #10 || Lin Protocol PART3 || Master - Slave communication using the Linbus](https://www.youtube.com/watch?v=u1MbUQcbw0g) (ControllersTech)
[^2]: [Digital twin](https://en.wikipedia.org/wiki/Digital_twin) (Wikipedia)

# Call for action
Create your own [home laboratory/workshop/garage](http://ufnalski.edu.pl/control_engineering_for_hobbyists/2025_dzien_otwarty_we/Dzien_Otwarty_WE_2025_Control_Engineering_for_Hobbyists.pdf)! Get inspired by [ControllersTech](https://www.youtube.com/@ControllersTech), [DroneBot Workshop](https://www.youtube.com/@Dronebotworkshop), [Andreas Spiess](https://www.youtube.com/@AndreasSpiess), [GreatScott!](https://www.youtube.com/@greatscottlab), [ElectroBOOM](https://www.youtube.com/@ElectroBOOM), [Phil's Lab](https://www.youtube.com/@PhilsLab), [atomic14](https://www.youtube.com/@atomic14), [That Project](https://www.youtube.com/@ThatProject), [Paul McWhorter](https://www.youtube.com/@paulmcwhorter), [Max Imagination](https://www.youtube.com/@MaxImagination), [Nikodem Bartnik](https://www.youtube.com/@nikodembartnik), and many other professional hobbyists sharing their awesome projects and tutorials! Shout-out/kudos to all of them!

> [!WARNING]
> Automotive reverse engineering - do try this at home :exclamation:

200+ challenges to start from: [Control Engineering for Hobbyists at the Warsaw University of Technology](http://ufnalski.edu.pl/control_engineering_for_hobbyists/Control_Engineering_for_Hobbyists_list_of_challenges.pdf).

Stay tuned :sunglasses:
