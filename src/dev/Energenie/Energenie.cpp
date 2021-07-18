/**************************************************************************************************
 * @file        Energenie.cpp
 * @author      Thomas
 * @brief       << Manually Entered >>
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <Energenie/Energenie.h>

Energenie::Energenie()
{
    this->ModulationEnab    = new GPIO(GPIO_LOW, 25, GPIO_OUT); // Setup Modulation pin (LOW)
    this->ASKSelector       = new GPIO(GPIO_LOW, 24, GPIO_OUT); // Setup ASKselector (LOW)

    this->D0                = new GPIO(GPIO_LOW, 17, GPIO_OUT); // Setup D0
    this->D1                = new GPIO(GPIO_LOW, 22, GPIO_OUT); // Setup D1
    this->D2                = new GPIO(GPIO_LOW, 23, GPIO_OUT); // Setup D2
    this->D3                = new GPIO(GPIO_LOW, 27, GPIO_OUT); // Setup D3
}

void Energenie::SwitchOn(void)
{
    this->D0->setValue(GPIO_HIGH);  //  Setup for socket 1 selection
    this->D1->setValue(GPIO_HIGH);  //
    this->D2->setValue(GPIO_HIGH);  //
    this->D3->setValue(GPIO_HIGH);  //  Switch ON

    delay(100);     // Settle time for encoder

    this->ModulationEnab->setValue(GPIO_HIGH);   // Select Modulation

    delay(250);    // Keep enabled for a period

    this->ModulationEnab->setValue(GPIO_LOW);  // DeSelect
}

void Energenie::SwitchOff(void)
{
    this->D0->setValue(GPIO_HIGH);  //  Setup for socket 1 selection
    this->D1->setValue(GPIO_HIGH);  //
    this->D2->setValue(GPIO_HIGH);  //
    this->D3->setValue(GPIO_LOW);   //  SWITCH OFF

    delay(100);     // Settle time for encoder

    this->ModulationEnab->setValue(GPIO_HIGH);   // Select Modulation

    delay(250);    // Keep enabled for a period

    this->ModulationEnab->setValue(GPIO_LOW);  // DeSelect
}


Energenie::~Energenie()
{
    delete [] ModulationEnab;               // Delete the array "ModulationEnab"
    delete [] ASKSelector;               // Delete the array "ASKSelector"

    delete [] D0;               // Delete the array "D0"
    delete [] D1;               // Delete the array "D1"
    delete [] D2;               // Delete the array "D2"
    delete [] D3;               // Delete the array "D3"
}

