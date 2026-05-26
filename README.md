# MLX90363

Header-only C++17 driver for the [Melexis MLX90363][datasheet] magnetic-rotary
sensor over hardware SPI on an AVR.

[datasheet]: https://www.melexis.com/en/product/mlx90363/triaxis-magnetic-node

## Status

Builds via the [uMaker](https://github.com/cinderblock/uMaker) Makefile system
against [AVR++](https://github.com/cinderblock/AVR) on `atmega32u4`. See
`examples/` for a runnable smoke-test firmware.

## Layout

```
MLX90363.hpp           # the library (one header)
examples/
  main.cpp             # minimal example firmware
  Makefile             # builds the example via uMaker
AVR++/                 # submodule — pin/SPI/atomic primitives
uMaker/                # submodule — Makefile build system
.github/workflows/     # CI: builds the example, uploads .elf
```

## Usage

The library is header-only — drop it into your project (e.g. as a git submodule
at `your-project/MLX90363/`) and add the directory to your include path:

```cpp
#include <MLX90363.hpp>
#include <avr/interrupt.h>

using namespace AVR;
using MyMLX = MLX90363Lib::MLX90363<IOpin<Ports::B, 7>>;  // SS pin

// The library does NOT install the SPI interrupt vector. Dispatch it yourself:
ISR(SPI_STC_vect) { MyMLX::isr(); }

int main() {
  MyMLX::init();
  sei();

  while (true) {
    MyMLX::prepareGET1Message(MyMLX::MessageType::Alpha);
    MyMLX::startTransmitting();
    _delay_us(MyMLX::tMeasurementAlpha_us + 100);

    auto alpha = MyMLX::getAlpha();
    (void)alpha;
  }
}
```

### Template parameters

| Param  | Default              | Purpose                                  |
|--------|----------------------|------------------------------------------|
| `SS`   | (required)           | Slave-select pin type. Library drives it. |
| `SCLK` | `AVR::SPI::SCLK`     | SPI clock pin (hardware default).        |
| `MOSI` | `AVR::SPI::MOSI`     | SPI MOSI pin (hardware default).         |
| `MISO` | `AVR::SPI::MISO`     | SPI MISO pin (hardware default).         |

Pin parameters are types (e.g. `AVR::IOpin<AVR::Ports::B, 7>`), not instances.

### What the library does NOT do

- **Install the SPI interrupt vector.** User code defines `ISR(SPI_STC_vect)`
  and dispatches to `MyMLX::isr()`. Avoids name collisions across multiple
  instantiations and makes the cost explicit.
- **Wait between transmissions.** The chip needs ~1 ms after a measurement
  command. Datasheet timings are exposed as `constexpr u4 tXxx_us` constants
  for use with `_delay_us()` or your own scheduler.

## Building the example locally

Needs `gcc-avr`, `avr-libc`, `binutils-avr`, `make`:

```bash
git clone --recurse-submodules https://github.com/cinderblock/MLX90363.git
cd MLX90363/examples
make build
```

Output is `examples/out/mlx-example.elf`.
