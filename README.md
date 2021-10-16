# miLibrary

This library contains the support of a majority of devices/interaces which will be used within my "mipidisaster" projects going forward. The design of this currently only supports the following devices:
 * Raspberry Pi (using the WiringPi interface -v2.50 - visit http://wiringpi.com/)
 * STM32Fx devices
 * STM32Lx devices
   * All STM32 devices require basic configuration via STM32CubeMX -v6.2.1 - https://www.st.com/en/development-tools/stm32cubemx.html

# Build Tree
![build tree](/_image/Build_tree.png)

develop
d0.2.1.1
* Worked issue [#9](https://github.com/mipidisaster/miLibrary/issues/9), through introduction of `zz__MiEmbedType__zz`
* RaspberryPi enabled functions (zz__MiEmbedType__zz  =  10) updated to work with change to code (wasn't modified/tested as part of the updates to v0.1.0 -> v0.2.1)

v0.2.1
* Introduces changes made in the branch 'wip-#5-stepper-improvements', these changes have been captured within Pull Request #6 (PR[#6](https://github.com/mipidisaster/miLibrary/pull/6/))
* Issue [#5](https://github.com/mipidisaster/miLibrary/issues/5) closed at this tag

v0.2.0 -> wip-#5 (actual name - 'wip-#5-stepper-improvements')
* Branch used to make improvements to the Stepper class, as part of fix for issue [#5](https://github.com/mipidisaster/miLibrary/issues/5)
* Additionally, first time making use of branches - still learning and figuring out how to use it

v0.2.0
* Architecture diagrams introduced into "_image"
* Eclipse preferences captured
* All major driver and device classes update to align with code style in wiki, as well as to remove version number and dates from top of source and header files
   * Issue [#2](https://github.com/mipidisaster/miLibrary/issues/2) fixed for Stepper header

v0.1.1
* README.md file image build updated to work correctly

v0.1.0
* Initial release

# Configuration

Embedded Device | Configuration parameter
-- | --
Linux (No Hardware) | #define zz__MiEmbedType__zz  =  0
Raspberry Pi | #define zz__MiEmbedType__zz  =  10
STM32Fx devices | #define zz__MiEmbedType__zz  =  50
STM32Lx devices | #define zz__MiEmbedType__zz  =  51

Eclipse settings/preferences (or at least my preferences), can be located in the `_preferences` folder

# Wiki
The wiki for this can be found within the Github repository for this - https://github.com/mipidisaster/miLibrary/wiki

# Issues
If you find any issues with this library, please capture them within the Github repository issue area - https://github.com/mipidisaster/miLibrary/issues.
