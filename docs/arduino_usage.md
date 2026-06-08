# moteus Arduino Usage Guide

This guide explains how to use the moteus Arduino library from a sketch or other Arduino-based project.

## What the library provides

The library centers on `MoteusController<CanBus>`, which builds and sends moteus commands over a transport object that provides:

- `poll()`
- `available()`
- `receive(CANFDMessage&)`
- `tryToSend(const CANFDMessage&)`

For convenience, the library also provides transport-specific aliases named `Moteus`.

## Choosing a transport

Use the header that matches your hardware.

- `MoteusAcan2517fd.h` for MCP2517FD and MCP2518FD adapters using `ACAN2517FD`
- `MoteusTeensy.h` for Teensy 4.x CAN-FD using `ACAN_T4`
- `MoteusStm32Fdcan.h` for STM32 H7, G4, and G0 on-board FDCAN
- `MoteusUart.h` for UART transport using the moteus fdcanusb-style text protocol

Each transport header defines a `Moteus` alias so the controller type stays consistent across platforms.

## Basic CAN-FD usage

The simplest pattern is:

1. Create the transport object.
2. Create a `Moteus` controller bound to that transport.
3. Send commands with `Set*`, `Begin*`, or `Make*`.

```cpp
#include <MoteusAcan2517fd.h>

ACAN2517FD can_bus;
Moteus controller(can_bus);

void setup() {
  // Initialize the CAN-FD backend here.
}

void loop() {
  controller.SetStop();
  delay(1000);
}
```

## The three command styles

Most controller actions come in three forms.

### `Make*`

`Make*` methods build a `CanFdFrame` without sending it.

Use these when you want to:

- inspect the frame
- batch transmission through another layer
- send the same frame later

Example:

```cpp
auto frame = controller.MakeStop();
```

### `Set*`

`Set*` methods send a command and wait for a response.

Use these when you want a synchronous call and do not need to manage polling yourself.

Example:

```cpp
bool ok = controller.SetBrake();
```

### `Begin*`

`Begin*` methods send a command and return immediately.

Use these when you want to poll for completion later.

Example:

```cpp
controller.BeginQuery();
while (!controller.Poll()) {
  // Wait for the reply.
}
```

After a successful `Poll()`, the parsed response is available from `last_result()`.

## Reading results

`last_result()` returns the most recent received frame and parsed query values.

```cpp
if (controller.Poll()) {
  const auto& result = controller.last_result();
  Serial.print("Position: ");
  Serial.println(result.values.position);
}
```

The parsed values come from the query format configured in the controller options.

## Controller options

`MoteusController::Options` lets you customize addressing and query behavior.

Important fields include:

- `id`: the moteus servo ID
- `source`: the source ID used by the host
- `can_prefix`: the CAN prefix applied to arbitration IDs
- `disable_brs`: whether outgoing frames use bit-rate switching
- `default_query`: whether each command includes the configured query
- `min_rcv_wait_us`: minimum wait time for blocking commands

Example:

```cpp
Moteus::Options options;
options.id = 1;
options.source = 0;
options.default_query = true;

Moteus controller(can_bus, options);
```

## Common commands

### Stop and brake

```cpp
controller.SetStop();
controller.SetBrake();
```

### Query

Querying reads the configured registers without changing mode.

```cpp
controller.SetQuery();
```

### Position mode

Position mode is the most common control path.

```cpp
mjbots::moteus::PositionMode::Command cmd;
cmd.position = 1.0;
cmd.velocity = 0.0;
cmd.feedforward_torque = 0.0;

controller.SetPosition(cmd);
```

You can also override the per-field encoding format with a `Format` object.

```cpp
mjbots::moteus::PositionMode::Format format;
format.position = Moteus::kFloat;
format.velocity = Moteus::kFloat;

controller.SetPosition(cmd, &format);
```

### Current mode

```cpp
mjbots::moteus::CurrentMode::Command cmd;
cmd.d_A = 0.0;
cmd.q_A = 1.0;

controller.SetCurrent(cmd);
```

### Voltage FOC mode

```cpp
mjbots::moteus::VFOCMode::Command cmd;
cmd.theta_rad = 0.0;
cmd.voltage = 2.0;
cmd.theta_rad_rate = 0.0;

controller.SetVFOC(cmd);
```

### Zero velocity mode

```cpp
mjbots::moteus::ZeroVelocityMode::Command cmd;
cmd.kd_scale = 1.0;

controller.SetZeroVelocity(cmd);
```

### Stay within mode

```cpp
mjbots::moteus::StayWithinMode::Command cmd;
cmd.lower_bound = -1.0;
cmd.upper_bound = 1.0;

controller.SetStayWithin(cmd);
```

### GPIO read and auxiliary PWM write

```cpp
controller.SetGpioRead();

mjbots::moteus::AuxPwmWrite::Command pwm;
pwm.aux1_pwm1 = 0.5f;
pwm.aux2_pwm1 = 0.25f;
controller.SetAuxPwmWrite(pwm);
```

## Waiting for trajectory completion

`SetPositionWaitComplete()` repeatedly sends position commands until the controller reports that the trajectory is complete.

```cpp
mjbots::moteus::PositionMode::Command cmd;
cmd.position = 2.0;
cmd.velocity = 0.0;

bool done = controller.SetPositionWaitComplete(cmd, 0.02);
```

The second argument is the update period in seconds.

## Diagnostic commands on Arduino

Diagnostic channel helpers are available only when compiling with Arduino because they use the Arduino `String` class.

Useful methods include:

- `DiagnosticCommand()`
- `SetDiagnosticRead()`
- `SetDiagnosticFlush()`

Example:

```cpp
String response = controller.DiagnosticCommand("conf get");
```

If you only need a single diagnostic channel read:

```cpp
String data = controller.SetDiagnosticRead(1);
```

## UART transport

`MoteusUart<SerialPort>` lets moteus communicate over the fdcanusb-style text protocol through a serial port.

Example:

```cpp
#include <MoteusUart.h>

MoteusUart<HardwareSerial> uart_bus(Serial1);
MoteusController<MoteusUart<HardwareSerial>> controller(uart_bus);

void setup() {
  uart_bus.begin();
}
```

The UART transport:

- sends frames as text commands
- waits for `OK` replies
- buffers received `rcv` lines

## Teensy transport

`MoteusTeensyCanFD` wraps an `ACAN_T4` bus object.

```cpp
#include <MoteusTeensy.h>

MoteusTeensyCanFD can_bus(ACAN_T4::can3, settings);
Moteus controller(can_bus);
```

Call `beginFD(settings)` on the CAN object in your setup before using the controller.

## STM32 transport

`MoteusStm32FdCan` wraps the STM32 HAL FDCAN peripheral.

```cpp
#include <MoteusStm32Fdcan.h>

MoteusStm32FdCan can_bus;
Moteus controller(can_bus);

void setup() {
  can_bus.begin();
}
```

The default constructor configures FDCAN1 with board-specific pin and timing defaults for supported STM32 families.

## Bare-metal STM32

If you are not using Arduino on STM32, define:

- `moteus_micros()`
- `moteus_delay_ms(uint32_t ms)`

Then use `MoteusController<MoteusStm32FdCan>` directly.

Diagnostic `String` helpers are not available in that build style.

## Practical notes

- The library assumes the moteus controller is already calibrated.
- Query formatting controls which registers are returned and at what resolution.
- `default_query` is convenient for development, but you can disable it to reduce bus traffic.
- `Poll()` should be called regularly when using `Begin*` methods.

## Suggested starting point

If you are just getting started, this is a good first sketch shape:

```cpp
#include <MoteusAcan2517fd.h>

ACAN2517FD can_bus;
Moteus controller(can_bus);

void setup() {
  // Initialize your CAN hardware here.
  controller.SetStop();
}

void loop() {
  mjbots::moteus::PositionMode::Command cmd;
  cmd.position = 0.5;
  cmd.velocity = 0.0;
  controller.SetPosition(cmd);
  delay(20);
}
```

