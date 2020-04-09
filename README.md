# ST L9945 driver in C++

## General

The driver supports the ST L9945 bridge driver. I won’t provide here any information available in the [datasheet](https://www.st.com/en/automotive-analog-and-power/l9945.html).

## Features

The driver provides a complete low-level C++ interface to the L9945 bridge driver. Main features:
* The driver uses C++ 17. In fact, only some `std::optional` return types in the diagnostics part.
* The driver is implemented header-only.
* It is platform-independent, and uses an interface class to communicate with the actual MCU and HAL.
* The driver uses a cache for reading and writing operations to reduce the number of SPI device accesses. These caches are implemented using the same granularity as the device registers: 32 bits, which includes many individual device functions.
* The driver is designed to run in a single thread, possibly as part of an application queue reception loop which processes higher level motor commands.
* The driver contains setting-dependent diagnostics enabling the application to make a “snapshot” of the actual status, which
  * can be examined programatically to gain information about behavior
  * or can be output to a text logging system for development purposes.
* The driver supports various error reporting facilities implemented in application logic:
  * If the application decides so, it can throw exceptions.
  * Or it is possible to compile the whole stuff without exceptions, then for example a status variable can take ists role.

## API

The user-intended API consists **only of the public methods** of the L9945 class. The later presented methods of underlying application-dependent interface are not intended to be called from the application!

Bridge and channel numbering is taken from the L9945 datasheet.

Due to the large amount of methods I don’t provide a detailed description here. Please refer the source for the names and the parameter list.

All the functions perform basic parameter checks, so for example setting something on the non-existent channel 9 won’t corrupt neighborous registers. However, no error is signaled in such cases (fail-silent operation).

### PWM output

By design, the L9945 is unable to generate PWM signal for its bridge or channel output drives. These signals must be provided somehow by the hardware and the application.

The two `setPwm` methods make it possible to set PWM signals on a bridge or an individual channel.

### Register API

Method names for register access are intended to conform the datasheet, but also be more readable the often cryptic acronyms.

The driver uses bool type to manipulate enabled and on/off type entities, but provides own enum classes for every enumerated type which can’t be specified using built-in types in a semantically clear way.

Due to the large number of registers and enum classes, a good editor with efficient typing aids is desirable.

For almost all of the registers these four method types are available. The exceptions are the read-only or write-only registers.

Under _influencer register_ I mean other registers, whose value change the interpretation the value of this one.

Method type   | Operation
--------------|--------------------------
`get*`        | Gets the value from the read cache, including influencer registers.
`modify*`     | Sets the value in the write cache.
`read*`       | Reads directly from the device, including also the influencer registers. These values are then stored in the read cache.
`write*`      | Writes directly into the device, and also stores the value read back in the write cache.

#### Cache management

The following methods help the cache management:

Method                                   | Read cache                             | Write cache
-----------------------------------------|----------------------------------------|----------------------------------
`readAllIntoCache()`                     |All regs modified to device value.      |Retained
`readIntoCache(uint32_t const aCommand)` |Specified reg modified to device value. | Retained
`writeAllFromCache()`                    |All regs modified to write cache.       |Retained
`writeFromCache(uint32_t const aCommand)`|Specified reg modified to write cache.  |Retained
`getReadCache(uint32_t const aCommand)`  |Retained                                | Retained

#### Device reset

The following steps are carried out during the reset() call:

1. Device reset using the application interface.
1. Copy the initial register contents array to the write cache.
1. A fake SPI transfer call on a read-only register to mitigate some initial communication fault under STM32G4 HAL.
1. `writeAllFromCache()`
1. Enable / disable the whole device according to the success of the previous call.

#### SPI transfer

The `spiTransfer(uint32_t const aCommand, uint32_t const aDelay)` method performs transfer in read as well as write operations. The whole call does anything only if the SPI transfer has not failed since the last reset. This can be queried using the `hasSpiEverFailed()` call.

The data to the L9945 device is transferred in a bi-directional way. First the read or write command is sent, then a fake one. During the fake command the read view of the register specified in the first command arrives. If both transfer succeed, and there is no parity error on the read back value, it is put into the read cache.

In case of an SPI / parity error, the whole device is shut down, the internal SPI error flag is set and the interface’s `fatalError` method is called with the appropriate `L9945::Exception` value. It may then throw an exception or handle the error some other way.

### Exceptions or other error handling

The driver supports

* exceptions for error reporting
* or the application interface can be implemented to enable the error be queried using a function call
* or the interface can call an application callback to handle the error
* or anything one can use a single error callback for.

Suppose we have this imaginary call stack

Function                                   |Remark
-------------------------------------------|-----------------------------------------
`interface::fatalError`                    |Place to throw an exception or set a flag or call an application error handler callback.
`L9945::spiTransfer`                       |SPI transfer
`L9945::read`                              |L9945::getCurrentSourceStatus
`application::processCurrentSource`        |Level of individual command processor methods
`application::queueReceiveLoop`            |The L9945 command processor loop in the application, possible place for a try - catch block or query the error flag after the commands processed.

The application thus can have a central error processing place. Because the C++ exception management is almost unnoticable when nothing is thrown and propagated, normal operation will won’t suffer. SPI or parity errors can be considered fatal, and a single central place handling a fatal error can be afforded to be slow.

### Device diagnostics

The driver supports device diagnostic in different modes. In all modes it takes a full snapshot of the device registers and returns a `DiagnosticsResult` object with it. It can then be examined “offline” by the application or output to some text logging system.

The `DiagnosticsResult` class has all the relevant query functions to obtain various device status information. However, it does not define any getter to obtain device settings. Refer the source for the list. This class also has a `log` function to send its contents to the text logging system.

Diagnostics is performed using the `L9945::diagnose(DiagnosticsTest const aTest)` call, which returns a reference to an internal `DiagnosticsResultobject`. It must be copied if not processed immediately.

#### Diagnostic modes

The driver defines the following diagnostic modes. Please refer the datasheet for more information.

Mode                                       | Description
-------------------------------------------|-------------------------------------------
`L9945::DiagnosticsTest::cAuto`            | Automatic diagnostic intended to run under normal PWM operation.
`L9945::DiagnosticsTest::cOffPulse`        | Can be used to test channels that are constantly on at the moment. The device issues a short off pulse on them to be able to test the corresponding circuits.
`L9945::DiagnosticsTest::cOnPulse`         | Can be used to test channels that are constantly off at the moment. The device issues a short on pulse on them to be able to test the corresponding circuits.
`L9945::DiagnosticsTest::cBist`            | Built-in self test for the digital and analog parts of the L9945 device. After perofming it the device needs to be reprogrammed.

## Application interface

Here is the application interface class API with STM32G4 HAL example implementation:

```C++
class L9945interface final {
static_assert(static_cast<uint32_t>(L9945<L9945interface>::SpiResult::cOk) == static_cast<uint32_t>(HAL_OK));
  static_assert(static_cast<uint32_t>(L9945<L9945interface>::SpiResult::cError) == static_cast<uint32_t>(HAL_ERROR));
  static_assert(static_cast<uint32_t>(L9945<L9945interface>::SpiResult::cBusy) == static_cast<uint32_t>(HAL_BUSY));
  static_assert(static_cast<uint32_t>(L9945<L9945interface>::SpiResult::cTimeout) == static_cast<uint32_t>(HAL_TIMEOUT));

public:
  static constexpr uint32_t cPwmMaxValue = 0xFFFu;
  static constexpr uint32_t cPrescaler   = 1u;

private:
  static constexpr uint32_t cSpiTimeout = 10u;
  SPI_HandleTypeDef*        mSpiHandle  = &hspi2;

  LogShiftChainHelper mAppender;
  DigitalOut mReset                     = DigitalOut(*L9945_NRES_GPIO_Port, L9945_NRES_Pin, Gpio::SignalLevel::cLow);
  DigitalOut mChipSelect                = DigitalOut(*L9945_SPI_NCS_GPIO_Port, L9945_SPI_NCS_Pin, Gpio::SignalLevel::cLow);

public:
  /// Blocking delay in ms.
  static void delayMs(uint32_t const aDelay) noexcept {
    nowtech::OsUtil::taskDelayMillis(aDelay);
  }

  /// @param aEnable if true, makes the reset signal to the L9945 effective, otherwise ineffective
  void enableReset(bool const aEnable) noexcept {
    mReset.write(aEnable ? Gpio::State::cOn : Gpio::State::cOff);
  }

  /// @param aEnable if true, enables the SPI connection to the L9945, otherwise disables it
  void enableSpiTransfer(bool const aEnable) noexcept {
    mChipSelect.write(aEnable ? Gpio::State::cOn : Gpio::State::cOff);
  }

  /// Can be used to enable or disable the whole functionality of the L9945 chip using one of its dedicated inputs.
  void enableAll(bool const aEnable) noexcept {
    // TODO set the appropriate GPIO
  }

  /// Used to signal a fatal error specified in the parameter. The implementation may throw it as an exception
  /// if the application decides to throw exceptions, ohterwise use an other error signaling mechanism.
  void fatalError(L9945<ExampleL9945interface>::Exception const aException) {
    throw aException;
  }

  /// Simultaneous transfer and receive a given amount ov bytes via SPI. The logic in L9945 class performs
  /// calls to enableSpiTransfer to enable and disable the SPI as needed.
  /// @param aTxData pointer to staff to send
  /// @param aRxData pointer to buffer to write into the received data
  /// @param aSize   amount of bytes to send and receive.
  /// @returns L9945::SpiResult to indicate the result.
  L9945<ExampleL9945interface>::SpiResult spiTransmitReceive(uint8_t const* const aTxData, uint8_t* const aRxData, uint16_t const aSize) noexcept {
    return static_cast<L9945<L9945interface>::SpiResult>(HAL_SPI_TransmitReceive(mSpiHandle, const_cast<uint8_t*>(aTxData), aRxData, aSize, cSpiTimeout));
  }

  /// Sets the drive to the L9945 external PWM inputs for a bridge, if it has benn so configured.
  /// @param aValue -1 full speed reverse, 0 stop, 1 full speed forward
  /// @param aBridge the bridge to set.
  void setPwm(float const aValue, L9945<L9945interface>::Bridge const aBridge) noexcept {
    if (aBridge == L9945<L9945interface>::Bridge::c1) {
      float absValue;
      if (std::abs(aValue) < 1.0f / cPwmMaxValue) {
        absValue = 0.0f;
      }
      else {
        absValue = std::min<float>(std::abs(aValue), 1.0f);
        HAL_GPIO_WritePin(BRIDGE_A_DIR_GPIO_Port, BRIDGE_A_DIR_Pin, aValue > 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
      uint32_t pwmValue = static_cast<uint32_t>(cPwmMaxValue * absValue);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwmValue);
      __HAL_TIM_SET_PRESCALER(&htim1, cPrescaler);
    }
    else { // only one bridge now, nothing to do
    }
  }
  
  /// Sets the drive to the L9945 external PWM inputs for a channel, if it has benn so configured.
  /// @param aValue 0 completely closed, 1 full time open
  /// @param aChannel channel number from 1 to 8, inclusive.
  void setPwm(float const aValue, uint32_t const aChannel) noexcept {
    // TODO something similar to the other one
  }

  /// Opens a log session for logging diagnostics. The class instance should store a handle or whatever is neeeded for it.
  void open() noexcept {
    mAppender = Log::i(nowtech::LogTopics::l9945);
  }

  /// Logs something to the open log session.
  template<typename ToAppend>
  L9945interface& operator<<(ToAppend aWhat) noexcept {
    mAppender << aWhat;
    return *this;
  }

  /// Finishes the log sesison.
  void close() noexcept {
    mAppender << Log::end;
  }
};
```

## Example

```C++
void nowtech::Application::initL9945() {
  mL9945.reset();
  //Enable diagnostics
  mL9945.modifyEnableDiagnostics(true);
  for (uint32_t i = 1u; i <= 4u; i++) {
    //Set ch 1-4 FET to N type
    mL9945.modifyHsFet(L9945real::ChannelHsFet::cNmos, i);
  }
  //Set side for devboard
  mL9945.modifySide(L9945real::ChannelSide::cHs, 1u);
  mL9945.modifySide(L9945real::ChannelSide::cHs, 2u);
  //Set ch 1-4 to bridge mode
  mL9945.modifyBridgeConfig(true, L9945real::Bridge::c1);
  for (uint32_t i = 1u; i <= 8u; i++) {
    //Set SPI mode
    mL9945.modifySpiInputSelect(false, i);
    //Enable all channels
    mL9945.modifyOutputEnable(true, i);
    //Protection bit shall be 0
    mL9945.modifyProtectionDisable(false, i);
    //Set OC method to shunt
    mL9945.modifyOutputOcMeasure(L9945real::ChannelOutputOcMeasure::cShunt, i);
    //Set OC detect treshold
    mL9945.modifyOcDetectTreshold(1078, i);
  }
  //Set side for devboard
  mL9945.modifySide(L9945real::ChannelSide::cHs, 1u);
  mL9945.modifySide(L9945real::ChannelSide::cHs, 2u);
  Log::i(nowtech::LogTopics::system) << "Send L9945 config to device." << Log::end;
  mL9945.writeAllFromCache();
  //Disable communication watchdog
  Log::i(nowtech::LogTopics::system) << "CC OFF" << Log::end;
  mL9945.writeConfigCommCheck(L9945real::RequestCommCheck::cNo);

  HAL_GPIO_WritePin(BRIDGE_A_EN_GPIO_Port, BRIDGE_A_EN_Pin, GPIO_PIN_RESET);
}

void nowtech::Application::defaultLoop() noexcept {
  checkMemory();
  float step = 0.0006;
  
  try {
    initL9945();    
    while (true) {
      float temp = mL9945.readTemperature();
      bool ot = mL9945.getOverTempState();
      Log::i(nowtech::LogTopics::system) << "temperature: " << temp << ' ' << ot << Log::end;
      
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); //Toggle LED
    
      // Test spin forward and backward
      Log::i(nowtech::LogTopics::system) << "Before +" << Log::end;
      auto diagnostics = mL9945.diagnose(nowtech::L9945real::DiagnosticsTest::cAuto);
      diagnostics.log();
      float lv = 0.0;
      for (float i = 0.0f; i < 1.0f; i += step) {
        mL9945.setPwm(i, L9945real::Bridge::c1);
        nowtech::OsUtil::taskDelayMillis(1u);
        lv = i;
      }
      nowtech::OsUtil::taskDelayMillis(1000u);
      for (float i = lv; i > 0.0f; i -= step) {
        mL9945.setPwm(i, L9945real::Bridge::c1);
        nowtech::OsUtil::taskDelayMillis(1u);
      }
      mL9945.setPwm(0.0f, L9945real::Bridge::c1);
      Log::i(nowtech::LogTopics::system) << "After +" << Log::end;
      diagnostics = mL9945.diagnose(nowtech::L9945real::DiagnosticsTest::cAuto);
      diagnostics.log();
      nowtech::OsUtil::taskDelayMillis(1000u);
      for (float i = 0.0f; i > -1.0f; i -= step) {
        mL9945.setPwm(i, L9945real::Bridge::c1);
        nowtech::OsUtil::taskDelayMillis(1u);
        lv = i;
      }
      nowtech::OsUtil::taskDelayMillis(1000u);
      for (float i = lv; i < 0.0f; i += step) {
        mL9945.setPwm(i, L9945real::Bridge::c1);
        nowtech::OsUtil::taskDelayMillis(1u);
      }
      mL9945.setPwm(0.0f, L9945real::Bridge::c1);
      Log::i(nowtech::LogTopics::system) << "After -" << Log::end;
      diagnostics = mL9945.diagnose(nowtech::L9945real::DiagnosticsTest::cAuto);
      diagnostics.log();
      nowtech::OsUtil::taskDelayMillis(1000u);
    }
  }
  catch (L9945real::Exception &exc) {
    while (true) {
      Log::i(nowtech::LogTopics::system) << (exc == L9945real::Exception::cParity ? "parity error" : "HAL error") << Log::end;
      nowtech::OsUtil::taskDelayMillis(2000u);
    }
  }
}
```
