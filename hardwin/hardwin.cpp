#include <hardwin.h>

namespace hardwin {

    void flashLed(int pin, int times, int wait) {
      for (int i = 0; i < times; i++) {
        digitalWrite(pin, HIGH);
        delay(wait);
        digitalWrite(pin, LOW);

        if (i + 1 < times) {
          delay(wait);
        }
      }
    }

}