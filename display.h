#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "Arduino.h"
#include "lvgl.h"

#include <TFT_eSPI.h> // TFT display library


#define TFT_DIRECTION 1   // 1 = liggande (vänd 180° mot förra versionen). Byt till 3 för att vända tillbaka. (0 = stående)


class Display
{
private:

public:
    void init();
    void routine();
};

#endif
