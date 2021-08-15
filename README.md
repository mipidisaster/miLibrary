# miLibrary

This library contains the support of a majority of devices/interaces which will be used within my "mipidisaster" projects going forward. The design of this currently only supports the following devices:
 * Raspberry Pi (using the WiringPi interface -v2.50 - visit http://wiringpi.com/)
 * STM32Fx devices
 * STM32Lx devices
   * All STM32 devices require basic configuration via STM32CubeMX -v6.2.1 - https://www.st.com/en/development-tools/stm32cubemx.html

# Build Tree
![build tree](https://github.com/mipidisaster/miLibrary/blob/master/_image/Build_tree.png)

0.2.0
* Architecture diagrams introduced into "_image"
* Eclipse preferences captured
* All major driver and device classes update to align with code style in wiki, as well as to remove version number and dates from top of source and header files
   * Issue [#2](https://github.com/mipidisaster/miLibrary/issues/2) fixed for Stepper header

0.1.1
* README.md file image build updated to work correctly

0.1.0
* Initial release

# Configuration

Embedded Device | Configuration parameter
-- | --
Raspberry Pi | #define zz__MiRaspbPi__zz
STM32Fx devices | #define zz__MiSTM32Fx__zz
STM32Lx devices | #define zz__MiSTM32Lx__zz

Eclipse settings/preferences (or at least my preferences), can be located in the `_preferences` folder

# Wiki
The wiki for this can be found within the Github repository for this - https://github.com/mipidisaster/miLibrary/wiki

# Issues
If you find any issues with this library, please capture them within the Github repository issue area - https://github.com/mipidisaster/miLibrary/issues. Issue numbering system is still a bit TBD at this point...
