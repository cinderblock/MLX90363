// Minimal MLX90363 example. Periodically requests an Alpha measurement and
// toggles a status LED on each fresh sample. Built by CI as a smoke test —
// proves the templated header compiles + links and the AVR++ submodule is
// wired correctly. Doesn't do anything useful by itself.

#include <avr/interrupt.h>
#include <util/delay.h>

#include <AVR++/IOpin.hpp>
#include <AVR++/Ports.hpp>

#include "../MLX90363.hpp"

using namespace AVR;
using namespace Basic;

// Slave-select for the MLX is on PB7 in this example. Choose any free pin.
using MySS = IOpin<Ports::B, 7>;
using MyMLX = MLX90363Lib::MLX90363<MySS>;

// Status LED on PB6. Toggled whenever the chip's ROLL counter advances.
using LED = IOpin<Ports::B, 6>;

// SPI complete vector — dispatch to the library's ISR handler. The library
// deliberately doesn't install this itself so multiple instantiations can
// coexist if needed.
ISR(SPI_STC_vect) { MyMLX::isr(); }

int main() {
  MyMLX::init();
  LED::output();
  LED::clr();
  sei();

  u1 lastRoll = 0xff;

  while (true) {
    MyMLX::prepareGET1Message(MyMLX::MessageType::Alpha);
    MyMLX::startTransmitting();
    _delay_us(MyMLX::tMeasurementAlpha_us + 100);

    if (MyMLX::hasNewData(lastRoll)) {
      LED::tgl();
      (void)MyMLX::getAlpha();
    }

    _delay_ms(10);
  }
}
