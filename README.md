# miLibrary

This library contains the support of a majority of devices/interaces which will be used within my "mipidisaster" projects going forward. The design of this currently only supports the following devices:
 * Raspberry Pi (using the WiringPi interface -v2.50 - visit http://wiringpi.com/)
 * STM32Fx devices
 * STM32Lx devices
   * All STM32 devices require basic configuration via STM32CubeMX -v6.2.1 - https://www.st.com/en/development-tools/stm32cubemx.html

# Contents
 - [Build Tree](#build-tree)
 - [Useful](#useful)

## Build Tree
![build tree](https://github.com/mipidisaster/Library/raw/master/_image/Build_tree.png)

0.1.1
* README.md file image build updated to work correctly

0.1.0
* Initial release

## Useful
![git repository image](https://github.com/mipidisaster/Library/raw/master/_image/git-commands.png)

### Clone Repositories
To clone this repository, use the following command:

    > git clone https://github.com/mipidisaster/Library.git

If you want it in the current directory, add ` .` to the end of the string

### Change Control

    git add .
    git add <directory>
Above will add any changes from the "workspace" (local area) to the "index"

    git commit -m "some changes have been made"
Will move the contents of the "index" to the "local repository"

    git tag -a <tag number> -m "my tagged version"
Will add a tag to the current commit within the "local respository"...i think

    git push
    git push origin <tag number>
Will move the contents of the "local repository" to the "remote repository" (in this case GitHub), account deatils would be required for this.
The push origin, will share the tag with the "remote repository"

### Basic commands

    git status
Provides the status of the current repository, when compared with the `Master` or which ever version has been cloned

    git diff
Similar to above, however shows any changes in the files

    git checkout
    git checkout <tag name>
Will update the "workspace" with either the contents of "index", or revert back to the contents as per `<tag name>`

### Guides
Suggestions on how to setup Git - https://guides.github.com/

Further guides - https://docs.github.com/en

Pro Git book - https://git-scm.com/book/en/v2 (a bit difficult to read)