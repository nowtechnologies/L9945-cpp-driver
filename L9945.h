
#ifndef NOWTECH_L9945_H
#define NOWTECH_L9945_H

#include <cmath>
#include <cstdint>
#include <optional>
#include <algorithm>
#include "BanCopyMove.h"

namespace nowtech {

namespace l9945 {

constexpr uint32_t getRightmost1position(uint32_t const aIn) noexcept {
  uint32_t result = 0;
  for (uint32_t work = aIn; (work & 1u) == 0 && result < 32u; ++result) {
    work >>= 1u;
  }
  return result;
}

constexpr uint32_t calculateParity(uint32_t const aIn) noexcept {
  uint32_t result = aIn;
  result ^= result >> 1u;
  result ^= result >> 2u;
  result ^= result >> 4u;
  result ^= result >> 8u;
  result ^= result >> 16u;
  return ~result & 1u;
}

}

/*
/// Example class providing an interface from the L9945 class to the actual application.
/// The L9945 instance will contain an instance of this to avoid more template 
/// instantiation when using several L9945 chips.
class ExampleL9945interface final {
public:
  /// Blocking delay in ms.
  static void delayMs(uint32_t const aDelay) noexcept;

  /// @param aEnable if true, makes the reset signal to the L9945 effective, otherwise ineffective
  void enableReset(bool const aEnable) noexcept;

  /// @param aEnable if true, enables the SPI connection to the L9945, otherwise disables it
  void enableSpiTransfer(bool const aEnable) noexcept;

  /// Can be used to enable or disable the whole functionality of the L9945 chip using one of its dedicated inputs.
  void enableAll(bool const aEnable) noexcept;

  /// Used to signal a fatal error specified in the parameter. The implementation may throw it as an exception
  /// if the application decides to throw exceptions, ohterwise use an other error signaling mechanism.
  void fatalError(L9945<ExampleL9945interface>::Exception const aException);

  /// Simultaneous transfer and receive a given amount ov bytes via SPI. The logic in L9945 class performs
  /// calls to enableSpiTransfer to enable and disable the SPI as needed.
  /// @param aTxData pointer to staff to send
  /// @param aRxData pointer to buffer to write into the received data
  /// @param aSize   amount of bytes to send and receive.
  /// @returns L9945::SpiResult to indicate the result.
  L9945<ExampleL9945interface>::SpiResult spiTransmitReceive(uint8_t const* const aTxData, uint8_t* const aRxData, uint16_t const aSize) noexcept;

  /// Sets the drive to the L9945 external PWM inputs for a bridge, if it has benn so configured.
  /// @param aValue -1 full speed reverse, 0 stop, 1 full speed forward
  /// @param aBridge the bridge to set.
  void setPwm(float const aValue, L9945<L9945interface>::Bridge const aBridge) noexcept;

  /// Sets the drive to the L9945 external PWM inputs for a channel, if it has benn so configured.
  /// @param aValue 0 completely closed, 1 full time open
  /// @param aChannel channel number from 1 to 8, inclusive.
  void setPwm(float const aValue, uint32_t const aChannel) noexcept;

  /// Opens a log session for logging diagnostics. The class instance should store a handle or whatever is neeeded for it.
  void open() noexcept;

  /// Logs something to the open log session.
  template<typename ToAppend>
  L9945interface& operator<<(ToAppend aWhat) noexcept;

  /// Finishes the log sesison.
  void close() noexcept;
};
*/

template<typename tInterface>
class L9945 final : public BanCopyMove {
private:
  static constexpr uint32_t cResetDelay                 = 10u;
  static constexpr uint32_t cCsDelay                    =  1u;
  static constexpr uint32_t cSizeofRegister             = sizeof(uint32_t);
  static constexpr uint32_t cRegisterCount              = 14u;

  static constexpr uint32_t cCommand0                   =  0u;
  static constexpr uint32_t cCommand1                   =  1u;
  static constexpr uint32_t cCommand2                   =  2u;
  static constexpr uint32_t cCommand3                   =  3u;
  static constexpr uint32_t cCommand4                   =  4u;
  static constexpr uint32_t cCommand5                   =  5u;
  static constexpr uint32_t cCommand6                   =  6u;
  static constexpr uint32_t cCommand7                   =  7u;
  static constexpr uint32_t cCommand8                   =  8u;
  static constexpr uint32_t cCommand9                   =  9u;
  static constexpr uint32_t cCommand10                  = 10u;
  static constexpr uint32_t cCommand11                  = 11u;
  static constexpr uint32_t cCommand12                  = 12u;
  static constexpr uint32_t cCommand13                  = 13u;

  static constexpr uint32_t cInvalidResponse            = 0u;
  static constexpr uint32_t cInvalidParity              = 1u;

  static constexpr uint32_t cMaskCommand                = 0x0fu << 28u;
  static constexpr uint32_t cMaskRead                   = 0x01u << 27u;
  static constexpr uint32_t cMaskParity                 = 0x01u;
  static constexpr uint32_t cMaskChannel                = 0x07u;
  static constexpr uint32_t cMaskChannelDiagnostics     = 0x010101u;

  static constexpr uint32_t cChannelCount               = cMaskChannel + 1u;
  static constexpr uint32_t cNoDelay                    = 0u;
  
public:
  enum class SpiResult : uint32_t {
    cOk      = 0x00U,
    cError   = 0x01U,
    cBusy    = 0x02U,
    cTimeout = 0x03U
  };

  enum class Exception : uint32_t {
    cCommunication = 0u,
    cParity        = 1u
  };

  enum class Bridge : uint32_t {
    c1 = 0u,
    c2 = 4u
  };

  // Mask names come almost unmodified from the data sheet.
  // Enum names tend to be more descriptive.
  // Whenever a bit has not a clean on/off meaning, an enum will be used with descriptive names.
  static constexpr uint32_t cMask0spreadSpectrum        = 0x01u << 26u;
  static constexpr uint32_t cMask0enableDiagnostics     = 0x01u << 25u;
  static constexpr uint32_t cMask0spiInputSelect81      = 0xffu << 17u;
  static constexpr uint32_t cMask0protectionDisable81   = 0xffu <<  9u;
  static constexpr uint32_t cMask0spiOnOut81            = 0xffu <<  1u;
  static constexpr uint32_t cMask0outputVcompared81     = 0xffu <<  1u;
//--------------------------------------------------------------------------------------
  static constexpr uint32_t cMask1bridge1deadTime       = 0x03u << 25u;
  enum class BridgeDeadTime : uint32_t {
    c1us =  0u,
    c2us =  1u << l9945::getRightmost1position(cMask1bridge1deadTime),
    c4us = (1u << l9945::getRightmost1position(cMask1bridge1deadTime)) * 2u,
    c8us = (1u << l9945::getRightmost1position(cMask1bridge1deadTime)) * 3u,
  };

  static constexpr uint32_t cMask1bridge1tDiagExtConfig = 0x01u << 24u;
  enum class BridgeSelectTdiagTimer : uint32_t {
    cHbridge  =  0u,
    cStandard =  1u << l9945::getRightmost1position(cMask1bridge1tDiagExtConfig)
  };

  // bits 1-23 inclusive are under cMask81*
//--------------------------------------------------------------------------------------
  static constexpr uint32_t cMask2bridge1tOff           = 0x03u << 25u;
  enum class BridgeToff : uint32_t {
    c31us  =  0u,
    c48us  =  1u << l9945::getRightmost1position(cMask2bridge1tOff),
    c62us  = (1u << l9945::getRightmost1position(cMask2bridge1tOff)) * 2u,
    c125us = (1u << l9945::getRightmost1position(cMask2bridge1tOff)) * 3u,
  };

  static constexpr uint32_t cMask2battFactorConfig      = 0x01u << 24u;
  enum class BatteryFactor : uint32_t {
    cCv =  0u,
    cPv =  1u << l9945::getRightmost1position(cMask2battFactorConfig)
  };

  // bits 1-23 inclusive are under cMask81*
//--------------------------------------------------------------------------------------
  static constexpr uint32_t cMask3bridge1currentLimitEn = 0x01u << 26u;
  static constexpr uint32_t cMask3bridge1actFreewheelLs = 0x01u << 25u;
  enum class BridgeFreewheelLs : uint32_t {
    cPassive =  0u,
    cActive  =  1u << l9945::getRightmost1position(cMask3bridge1actFreewheelLs)
  };

  static constexpr uint32_t cMask3gccOverrideConfig     = 0x01u << 24u;
  enum class GccOverride : uint32_t {
    cSelective =  0u,
    cGlobal    =  1u << l9945::getRightmost1position(cMask3gccOverrideConfig)
  };

  // bits 1-23 inclusive are under cMask81*
//--------------------------------------------------------------------------------------
  static constexpr uint32_t cMask4bridge1config         = 0x01u << 26u;
  static constexpr uint32_t cMask4peakHold1diagStrategy = 0x01u << 25u;
  enum class PeakHoldDiagReport : uint32_t {
    cNoOlStgStbFailure =  0u,
    cNoDiagDone        =  1u << l9945::getRightmost1position(cMask4peakHold1diagStrategy)
  };

  static constexpr uint32_t cMask4peakHold1config       = 0x01u << 24u;

  // bits 1-23 inclusive are under cMask81*
//--------------------------------------------------------------------------------------
  static constexpr uint32_t cMask5bridge2deadTime       = 0x03u << 25u;
  // BridgeDeadTime used here
  static_assert(cMask1bridge1deadTime == cMask5bridge2deadTime);

  static constexpr uint32_t cMask5bridge2tDiagExtConfig = 0x01u << 24u;
  // BridgeSelectTdiagTimer used here
  static_assert(cMask1bridge1tDiagExtConfig == cMask5bridge2tDiagExtConfig);

  // bits 1-23 inclusive are under cMask81*
//--------------------------------------------------------------------------------------
  static constexpr uint32_t cMask6bridge2tOff           = 0x03u << 25u;
  // BridgeToff used here
  static_assert(cMask2bridge1tOff == cMask6bridge2tOff);

  // bits 1-23 inclusive are under cMask81*
//--------------------------------------------------------------------------------------
  static constexpr uint32_t cMask7bridge2currentLimitEn = 0x01u << 26u;
  static_assert(cMask3bridge1currentLimitEn == cMask7bridge2currentLimitEn);

  static constexpr uint32_t cMask7bridge2actFreewheelLs = 0x01u << 25u;
  // BridgeFreewheelLs used here
  static_assert(cMask3bridge1actFreewheelLs == cMask7bridge2actFreewheelLs);

  // bits 1-23 inclusive are under cMask81*
//--------------------------------------------------------------------------------------
  static constexpr uint32_t cMask8bridge2config         = 0x01u << 26u;
  static_assert(cMask4bridge1config == cMask8bridge2config);

  static constexpr uint32_t cMask8peakHold2diagStrategy = 0x01u << 25u;
  // PeakHoldDiagReport used here
  static_assert(cMask4peakHold1diagStrategy == cMask8peakHold2diagStrategy);

  static constexpr uint32_t cMask8peakHold2config       = 0x01u << 24u;
  static_assert(cMask4peakHold1config == cMask8peakHold2config);
  // bits 1-23 inclusive are under cMask81*
//--------------------------------------------------------------------------------------
  static constexpr uint32_t cMask9bridge2currentLimit   = 0x01u << 26u; // read
  static constexpr uint32_t cMask9bridge1currentLimit   = 0x01u << 25u; // read
  static constexpr uint32_t cMask9diagOffPulse81        = 0xffu <<  9u; // write
  static constexpr uint32_t cMask9diagOnPulse81         = 0xffu <<  1u; // write
  static constexpr uint32_t cMask9diagnosticBit2ch81    = 0xffu << 17u; // read   access thesed three per channel
  static constexpr uint32_t cMask9diagnosticBit1ch81    = 0xffu <<  9u; // read
  static constexpr uint32_t cMask9diagnosticBit0ch81    = 0xffu <<  1u; // read
  enum class ChannelDiagnostics : uint32_t {
    cOcPinFail      = 0x000000u,
    cOcFail         = 0x000001u,
    cStgStbFail     = 0x000100u,
    cOlFail         = 0x000101u,
    cNoFail         = 0x010000u,
    cNoOcFail       = 0x010001u,
    cNoOlStgStbFail = 0x010100u,
    cNoDiagDone     = 0x010101u
  };

//--------------------------------------------------------------------------------------
  static constexpr uint32_t cMask10bistHwscRequest      = 0x03u <<  5u; // write
  enum class RequestBist : uint32_t {
    cYes =  1u << l9945::getRightmost1position(cMask10bistHwscRequest),
    cNo  = (1u << l9945::getRightmost1position(cMask10bistHwscRequest)) * 2
  };

  static constexpr uint32_t cMask10configCommCheck      = 0x03u <<  3u; // write
  enum class RequestCommCheck : uint32_t {
    cYes =  1u << l9945::getRightmost1position(cMask10configCommCheck),
    cNo  = (1u << l9945::getRightmost1position(cMask10configCommCheck)) * 2
  };

  static constexpr uint32_t cMask10en6disableLatch      = 0x01u << 26u; // read
  static constexpr uint32_t cMask10en6disableState      = 0x01u << 25u; // read
  static constexpr uint32_t cMask10vddOvDisableLatch    = 0x01u << 24u; // read
  static constexpr uint32_t cMask10vddUvDisableState    = 0x01u << 23u; // read
  static constexpr uint32_t cMask10vddUvDisableLatch    = 0x01u << 22u; // read
  static constexpr uint32_t cMask10deviceDisState       = 0x01u << 21u; // read
  static constexpr uint32_t cMask10deviceDisLatch       = 0x01u << 20u; // read
  static constexpr uint32_t cMask10deviceNdisOnState    = 0x01u << 19u; // read
  static constexpr uint32_t cMask10deviceNdisOnLatch    = 0x01u << 18u; // read
  static constexpr uint32_t cMask10deviceNdisOutLatch   = 0x01u << 17u; // read
  static constexpr uint32_t cMask10configCommCheckState = 0x01u << 16u; // read
  static constexpr uint32_t cMask10commCheckLatch       = 0x01u << 15u; // read
  static constexpr uint32_t cMask10bistDone             = 0x01u << 14u; // read
  static constexpr uint32_t cMask10bistDisableLatch     = 0x01u << 13u; // read
  enum class BistResult : uint32_t {
    cPassed = 0u,
    cFailed = 1u << l9945::getRightmost1position(cMask10bistDisableLatch)
  };

  static constexpr uint32_t cMask10hwscDone             = 0x01u << 12u; // read
  static constexpr uint32_t cMask10hwscDisableLatch     = 0x01u << 11u; // read
  enum class HwscResult : uint32_t {
    cPassed = 0u,
    cFailed = 1u << l9945::getRightmost1position(cMask10hwscDisableLatch)
  };
  
  static constexpr uint32_t cMask10vddOvCompState       = 0x01u << 10u; // read
  static constexpr uint32_t cMask10vddOvCompLatch       = 0x01u <<  9u; // read
  static constexpr uint32_t cMask10vddUvCompState       = 0x01u <<  8u; // read
  static constexpr uint32_t cMask10vddUvCompLatch       = 0x01u <<  7u; // read
  static constexpr uint32_t cMask10powerOnResetLatch    = 0x01u <<  6u; // read
  static constexpr uint32_t cMask10nResLatch            = 0x01u <<  5u; // read
  static constexpr uint32_t cMask10vcpUvState           = 0x01u <<  4u; // read
  enum class VgbhiUvStatus : uint32_t {
    cVpsGtVvpsUv = 0u,
    cVpsLtVvpsUv = 1u << l9945::getRightmost1position(cMask10vcpUvState)
  };

  static constexpr uint32_t cMask10vcpUvLatch           = 0x01u <<  3u; // read
  static constexpr uint32_t cMask10vpsUvState           = 0x01u <<  2u; // read
  enum class VpsStatus : uint32_t {
    cVpsGtVvpsUv = 0u,
    cVpsLtVvpsUv = 1u << l9945::getRightmost1position(cMask10vpsUvState)
  };

  static constexpr uint32_t cMask10vpsUvLatch           = 0x01u <<  1u; // read
//--------------------------------------------------------------------------------------
  static constexpr uint32_t cMask1112externalFetState4185   = 0x0fu << 17u; // read
  static constexpr uint32_t cMask1112externalFetCommand4185 = 0x0fu << 13u; // read
  static constexpr uint32_t cMask1112channelPullUpDown48   = 0x0fu << 10u; // read
  static constexpr uint32_t cMask1112channelPullUpDown37   = 0x0fu <<  7u; // read
  static constexpr uint32_t cMask1112channelPullUpDown26   = 0x0fu <<  4u; // read
  static constexpr uint32_t cMask1112channelPullUpDown15   = 0x0fu <<  1u; // read
  enum class CurrentSource : uint32_t {
    cCompromised = 0u,
    cFetOn       = 1u,
    cFetOff      = 2u,
    cFetTriState = 3u
  };
  
//--------------------------------------------------------------------------------------
  static constexpr uint32_t cMask13ndisProtectLatch     = 0x01u << 23u; // read
  static constexpr uint32_t cMask13overTempState        = 0x01u << 22u; // read
  static constexpr uint32_t cMask13sdoOvLatch           = 0x01u << 21u; // read
  static constexpr uint32_t cMask13tempAdc             = 0x3ffu << 11u; // read
  static constexpr uint32_t cMask13vpsAdc              = 0x3ffu <<  1u; // read
//--------------------------------------------------------------------------------------
  // These apply to Commands 1-8 on channels 1-8
  static constexpr uint32_t cMask81tDiagConfig81        = 0x03u << 22u;
  enum class ChannelTdiagOff : uint32_t {
    c11_25us  =  0u,
    c28_61us  =  1u << l9945::getRightmost1position(cMask81tDiagConfig81),
    c40_105us = (1u << l9945::getRightmost1position(cMask81tDiagConfig81)) * 2u,
    c51_150us = (1u << l9945::getRightmost1position(cMask81tDiagConfig81)) * 3u,
  };

  static constexpr uint32_t cMask81ocRead81             = 0x01u << 21u;
  enum class ChannelOcThreasholdToRead : uint32_t {
    cFixed  =  0u,
    cActual =  1u << l9945::getRightmost1position(cMask81ocRead81)
  };

  static constexpr uint32_t cMask81ocConfig81           = 0x3fu << 15u;
  static constexpr uint32_t cMask81ocTempComp81         = 0x03u << 13u;
  enum class ChannelOcTempComp : uint32_t {
    cNone  =  0u,
    c60deg =  1u << l9945::getRightmost1position(cMask81ocTempComp81),
    c40deg = (1u << l9945::getRightmost1position(cMask81ocTempComp81)) * 2u,
    c25deg = (1u << l9945::getRightmost1position(cMask81ocTempComp81)) * 3u
  };

  static constexpr uint32_t cMask81ocBattComp81         = 0x01u << 12u;
  static constexpr uint32_t cMask81tBlankOc81           = 0x07u <<  9u;
  enum class ChannelOcBlankTime : uint32_t {
    c11us  =  0u,
    c15us  =  1u << l9945::getRightmost1position(cMask81tBlankOc81),
    c20us  = (1u << l9945::getRightmost1position(cMask81tBlankOc81)) * 2u,
    c31us  = (1u << l9945::getRightmost1position(cMask81tBlankOc81)) * 3u,
    c42us  = (1u << l9945::getRightmost1position(cMask81tBlankOc81)) * 4u,
    c53us  = (1u << l9945::getRightmost1position(cMask81tBlankOc81)) * 5u,
    c97us  = (1u << l9945::getRightmost1position(cMask81tBlankOc81)) * 6u,
    c142us = (1u << l9945::getRightmost1position(cMask81tBlankOc81)) * 7u
  };

  static constexpr uint32_t cMask81protConfig81         = 0x01u <<  8u;
  enum class ChannelOutputReEngage : uint32_t {
    cWithControlSignalSwitching  =  0u,
    cAfterControlSignalSwitching =  1u << l9945::getRightmost1position(cMask81protConfig81)
  };

  static constexpr uint32_t cMask81ocDsShunt81          = 0x01u <<  7u;
  enum class ChannelOutputOcMeasure : uint32_t {
    cDsm   =  0u,
    cShunt =  1u << l9945::getRightmost1position(cMask81ocDsShunt81)
  };

  static constexpr uint32_t cMask81diagIconfig81        = 0x01u <<  6u;
  enum class ChannelOlOutCurrCapability : uint32_t {
    c100uA =  0u,
    c1mA   =  1u << l9945::getRightmost1position(cMask81diagIconfig81)
  };

  static constexpr uint32_t cMask81gccConfig81          = 0x03u <<  4u;
  enum class ChannelGateCurrent : uint32_t {
    cExternalResistor = 0u,
    c20mA =  1u << l9945::getRightmost1position(cMask81gccConfig81),
    c5mA  = (1u << l9945::getRightmost1position(cMask81gccConfig81)) * 2u,
    c1mA  = (1u << l9945::getRightmost1position(cMask81gccConfig81)) * 3u
  };

  static constexpr uint32_t cMask81nPconfig81           = 0x01u <<  3u;
  enum class ChannelHsFet : uint32_t {
    cNmos =  0u,
    cPmos =  1u << l9945::getRightmost1position(cMask81nPconfig81)
  };
  
  static constexpr uint32_t cMask81lsHsConfig81         = 0x01u <<  2u;
  enum class ChannelSide : uint32_t {
    cLs =  0u,
    cHs =  1u << l9945::getRightmost1position(cMask81lsHsConfig81)
  };
  static constexpr uint32_t cMask81enOut81              = 0x01u <<  1u;

  static constexpr uint32_t cInitialRegisterValues[] = {
    //10987654321098765432109876543210
    0b00001000000000000000000000000001u,
    0x1EC00001u,
    0x2EC00001u,
    0x3BC00000u,
    0x48C00001u,
    0x5EC00000u,
    0x6EC00000u,
    0x7AC00000u,
    0x88C00001u,
    0b10011010101010111111111111111110u,
    0b10101010101010101010101010000000u, // I counted X bits as 0
    0xBAAAAAAAu,
    0xCAAAAAABu,
    0xDAAAAAAAu
    //10987654321098765432109876543210
  };

  static constexpr uint32_t cFixedPatternValues[] = {
    //10987654321098765432109876543210
    0x00000000u,
    0x10000000u,
    0x20000000u,
    0x30000000u,
    0x40000000u,
    0x50000000u,
    0x60000000u,
    0x70000000u,
    0x80000000u,
    0b10010010101010100000000000000000u,
    0b10100010101010101010101010000000u,
    0xBAAAAAAAu,
    0xCAAAAAABu,
    0xDAAAAAAAu
    //10987654321098765432109876543210
  };

  static constexpr uint32_t cFixedPatternMasks[] = {
    //10987654321098765432109876543210
    0xF0000000u,
    0xF0000000u,
    0xF0000000u,
    0xF0000000u,
    0xF0000000u,
    0xF0000000u,
    0xF1000000u,
    0xF1000000u,
    0xF0000000u,
    0b11110111111111100000000000000000u,
    0b11110111111111111111111110000000u,
    0xFFFFFFFFu,
    0xFFFFFFFFu,
    0xFFFFFFFFu
    //10987654321098765432109876543210
  };

  static constexpr CurrentSource cCurrentSourceDecoder[0x10] = {
    // HS and PMOS
    CurrentSource::cFetTriState, CurrentSource::cCompromised, CurrentSource::cFetOff, CurrentSource::cCompromised, CurrentSource::cFetOn, CurrentSource::cCompromised, CurrentSource::cCompromised, CurrentSource::cCompromised,
    // LS or NMOS
    CurrentSource::cFetTriState, CurrentSource::cFetOn, CurrentSource::cFetOff, CurrentSource::cCompromised, CurrentSource::cCompromised, CurrentSource::cCompromised, CurrentSource::cCompromised, CurrentSource::cCompromised
  };

  uint8_t mDataOut[cSizeofRegister * 2u] = { 0u, 0u, 0u, 0u, 0xf0u, 0u, 0u, 1u }; // last one is invalid command with parity error;
  uint8_t mDataIn[cSizeofRegister * 2u];

  tInterface&        mInterface;

  // Parity is not maintained in these values!
  std::array<uint32_t, cRegisterCount> mReadCache;
  std::array<uint32_t, cRegisterCount> mWriteCache;
  bool                                 mSpiFailed = false;
  uint32_t                             mWriteDelay = 0u;

public:
  L9945(tInterface &aInterface)
  : mInterface(aInterface)
  , mLastResult(this) {
  }

  void reset();

  bool hasSpiEverFailed() const noexcept {
    return mSpiFailed;
  }

  // @param aValue -1 full speed reverse, 0 stop, 1 full speed forward
  void setPwm(float const aValue, Bridge const aBridge);

  // @param aValue 0 closed, 1 full time open
  void setPwm(float const aValue, uint32_t const aChannel);

  bool getSpreadSpectrum()                     noexcept { return getBool(cCommand0, cMask0spreadSpectrum); } // 26
  void modifySpreadSpectrum(bool const aValue) noexcept { modifyBool(cCommand0, cMask0spreadSpectrum, aValue); }
  bool readSpreadSpectrum()                             { return readBool(cCommand0, cMask0spreadSpectrum); }
  bool writeSpreadSpectrum(bool const aValue)           { return writeBool(cCommand0, cMask0spreadSpectrum, aValue); }
 
  bool getEnableDiagnostics()                     noexcept { return getBool(cCommand0, cMask0enableDiagnostics); } // 25
  void modifyEnableDiagnostics(bool const aValue) noexcept { modifyBool(cCommand0, cMask0enableDiagnostics, aValue); }
  bool readEnableDiagnostics()                             { return readBool(cCommand0, cMask0enableDiagnostics); }
  bool writeEnableDiagnostics(bool const aValue)           { return writeBool(cCommand0, cMask0enableDiagnostics, aValue); }
 
  bool getSpiInputSelect(uint32_t const aChannel)                       noexcept { return getValue(cCommand0, cMask0spiInputSelect81, aChannel); } // 17-
  void modifySpiInputSelect(bool const aValue, uint32_t const aChannel) noexcept { modifyValue(cCommand0, cMask0spiInputSelect81, aValue, aChannel); }
  bool readSpiInputSelect(uint32_t const aChannel)                               { return readValue(cCommand0, cMask0spiInputSelect81, aChannel); }
  bool writeSpiInputSelect(bool const aValue, uint32_t const aChannel)           { return writeValue(cCommand0, cMask0spiInputSelect81, aValue, aChannel); }
 
  bool getProtectionDisable(uint32_t const aChannel)                       noexcept { return getValue(cCommand0, cMask0protectionDisable81, aChannel); } // 9-
  void modifyProtectionDisable(bool const aValue, uint32_t const aChannel) noexcept { modifyValue(cCommand0, cMask0protectionDisable81, aValue, aChannel); }
  bool readProtectionDisable(uint32_t const aChannel)                               { return readValue(cCommand0, cMask0protectionDisable81, aChannel); }
  bool writeProtectionDisable(bool const aValue, uint32_t const aChannel)           { return writeValue(cCommand0, cMask0protectionDisable81, aValue, aChannel); }

  uint32_t getSpiOnOutMask(uint32_t const aMask)                       noexcept { return getValue(cCommand0, cMask0spiOnOut81) & aMask; } // 1-
  void modifySpiOnOutMask(uint32_t const aValue, uint32_t const aMask) noexcept { modifyValue(cCommand0, cMask0spiOnOut81, aValue, aMask); }
  uint32_t readSpiOnOutMask(uint32_t const aMask)                               { return readValue(cCommand0, cMask0spiOnOut81) & aMask; };
  bool writeSpiOnOutMask(uint32_t const aValue, uint32_t const aMask)           { return writeValue(cCommand0, cMask0spiOnOut81, aValue, aMask); } 

  bool getSpiOnOut(uint32_t const aChannel) noexcept;
  void modifySpiOnOut(bool const aValue, uint32_t const aChannel)  noexcept { modifyValue(cCommand0, cMask0spiOnOut81, aValue, aChannel); }  // 1-
  bool readSpiOnOut(uint32_t const aChannel);
  bool writeSpiOnOut(bool const aValue, uint32_t const aChannel)            { return writeValue(cCommand0, cMask0spiOnOut81, aValue, aChannel); }
 
  BridgeDeadTime getBridgeDeadTime(Bridge const aBridge)                       noexcept { return static_cast<BridgeDeadTime>(getEnum(bridge2command1458(cCommand1, aBridge), cMask1bridge1deadTime)); } // 25-
  void modifyBridgeDeadTime(BridgeDeadTime const aValue, Bridge const aBridge) noexcept { modifyEnum(bridge2command1458(cCommand1, aBridge), cMask1bridge1deadTime, static_cast<uint32_t>(aValue)); }
  BridgeDeadTime readBridgeDeadTime(Bridge const aBridge)                               { return static_cast<BridgeDeadTime>(readEnum(bridge2command1458(cCommand1, aBridge), cMask1bridge1deadTime)); }
  bool writeBridgeDeadTime(BridgeDeadTime const aValue, Bridge const aBridge)           { return writeEnum(bridge2command1458(cCommand1, aBridge), cMask1bridge1deadTime, static_cast<uint32_t>(aValue)); }
 
  BridgeSelectTdiagTimer getBridgeTdiagExtConfig(Bridge const aBridge)                       noexcept { return static_cast<BridgeSelectTdiagTimer>(getEnum(bridge2command1458(cCommand1, aBridge), cMask1bridge1tDiagExtConfig)); } // 25-
  void modifyBridgeTdiagExtConfig(BridgeSelectTdiagTimer const aValue, Bridge const aBridge) noexcept { modifyEnum(bridge2command1458(cCommand1, aBridge), cMask1bridge1tDiagExtConfig, static_cast<uint32_t>(aValue)); }
  BridgeSelectTdiagTimer readBridgeTdiagExtConfig(Bridge const aBridge)                               { return static_cast<BridgeSelectTdiagTimer>(readEnum(bridge2command1458(cCommand1, aBridge), cMask1bridge1tDiagExtConfig)); }
  bool writeBridgeTdiagExtConfig(BridgeSelectTdiagTimer const aValue, Bridge const aBridge)           { return writeEnum(bridge2command1458(cCommand1, aBridge), cMask1bridge1tDiagExtConfig, static_cast<uint32_t>(aValue)); }
 
  BridgeToff getBridgeTOff(Bridge const aBridge)                       noexcept { return static_cast<BridgeToff>(getEnum(bridge2command1458(cCommand2, aBridge), cMask2bridge1tOff)); } // 25-
  void modifyBridgeTOff(BridgeToff const aValue, Bridge const aBridge) noexcept { modifyEnum(bridge2command1458(cCommand2, aBridge), cMask2bridge1tOff, static_cast<uint32_t>(aValue)); }
  BridgeToff readBridgeTOff(Bridge const aBridge)                               { return static_cast<BridgeToff>(readEnum(bridge2command1458(cCommand2, aBridge), cMask2bridge1tOff)); }
  bool writeBridgeTOff(BridgeToff const aValue, Bridge const aBridge)           { return writeEnum(bridge2command1458(cCommand2, aBridge), cMask2bridge1tOff, static_cast<uint32_t>(aValue)); }
 
  BatteryFactor getBattFactorConfig()                     noexcept { return static_cast<BatteryFactor>(getEnum(cCommand2, cMask2battFactorConfig)); } // 24-
  void modifyBattFactorConfig(BatteryFactor const aValue) noexcept { modifyEnum(cCommand2, cMask2battFactorConfig, static_cast<uint32_t>(aValue)); }
  BatteryFactor readBattFactorConfig()                             { return static_cast<BatteryFactor>(readEnum(cCommand2, cMask2battFactorConfig)); }
  bool writeBattFactorConfig(BatteryFactor const aValue)           { return writeEnum(cCommand2, cMask2battFactorConfig, static_cast<uint32_t>(aValue)); }
 
  bool getBridgeCurrentLimitEn(Bridge const aBridge)                       noexcept { return getBool(bridge2command1458(cCommand3, aBridge), cMask3bridge1currentLimitEn); } // 26
  void modifyBridgeCurrentLimitEn(bool const aValue, Bridge const aBridge) noexcept { modifyBool(bridge2command1458(cCommand3, aBridge), cMask3bridge1currentLimitEn, aValue); }
  bool readBridgeCurrentLimitEn(Bridge const aBridge)                               { return readBool(bridge2command1458(cCommand3, aBridge), cMask3bridge1currentLimitEn); }
  bool writeBridgeCurrentLimitEn(bool const aValue, Bridge const aBridge)           { return writeBool(bridge2command1458(cCommand3, aBridge), cMask3bridge1currentLimitEn, aValue); }
 
  BridgeFreewheelLs getBridgeActFreewheelLs(Bridge const aBridge)                       noexcept { return static_cast<BridgeFreewheelLs>(getEnum(bridge2command1458(cCommand3, aBridge), cMask3bridge1actFreewheelLs)); } // 25-
  void modifyBridgeActFreewheelLs(BridgeFreewheelLs const aValue, Bridge const aBridge) noexcept { modifyEnum(bridge2command1458(cCommand3, aBridge), cMask3bridge1actFreewheelLs, static_cast<uint32_t>(aValue)); }
  BridgeFreewheelLs readBridgeActFreewheelLs(Bridge const aBridge)                               { return static_cast<BridgeFreewheelLs>(readEnum(bridge2command1458(cCommand3, aBridge), cMask3bridge1actFreewheelLs)); }
  bool writeBridgeActFreewheelLs(BridgeFreewheelLs const aValue, Bridge const aBridge)           { return writeEnum(bridge2command1458(cCommand3, aBridge), cMask3bridge1actFreewheelLs, static_cast<uint32_t>(aValue)); }
 
  GccOverride getGccOverrideConfig()                     noexcept { return static_cast<GccOverride>(getEnum(cCommand3, cMask3gccOverrideConfig)); } // 24-
  void modifyGccOverrideConfig(GccOverride const aValue) noexcept { modifyEnum(cCommand3, cMask3gccOverrideConfig, static_cast<uint32_t>(aValue)); }
  GccOverride readGccOverrideConfig()                             { return static_cast<GccOverride>(readEnum(cCommand3, cMask3gccOverrideConfig)); }
  bool writeGccOverrideConfig(GccOverride const aValue)           { return writeEnum(cCommand3, cMask3gccOverrideConfig, static_cast<uint32_t>(aValue)); }
  
  bool getBridgeConfig(Bridge const aBridge)                       noexcept { return getBool(bridge2command1458(cCommand4, aBridge), cMask4bridge1config); } // 26
  void modifyBridgeConfig(bool const aValue, Bridge const aBridge) noexcept { modifyBool(bridge2command1458(cCommand4, aBridge), cMask4bridge1config, aValue); }
  bool readBridgeConfig(Bridge const aBridge)                               { return readBool(bridge2command1458(cCommand4, aBridge), cMask4bridge1config); }
  bool writeBridgeConfig(bool const aValue, Bridge const aBridge)           { return writeBool(bridge2command1458(cCommand4, aBridge), cMask4bridge1config, aValue); }
 
  PeakHoldDiagReport getPeakHoldDiagReport(Bridge const aBridge)                       noexcept { return static_cast<PeakHoldDiagReport>(getEnum(bridge2command1458(cCommand4, aBridge), cMask4peakHold1diagStrategy)); } // 25-
  void modifyPeakHoldDiagReport(PeakHoldDiagReport const aValue, Bridge const aBridge) noexcept { modifyEnum(bridge2command1458(cCommand4, aBridge), cMask4peakHold1diagStrategy, static_cast<uint32_t>(aValue)); }
  PeakHoldDiagReport readPeakHoldDiagReport(Bridge const aBridge)                               { return static_cast<PeakHoldDiagReport>(readEnum(bridge2command1458(cCommand4, aBridge), cMask4peakHold1diagStrategy)); }
  bool writePeakHoldDiagReport(PeakHoldDiagReport const aValue, Bridge const aBridge)           { return writeEnum(bridge2command1458(cCommand4, aBridge), cMask4peakHold1diagStrategy, static_cast<uint32_t>(aValue)); }
 
  bool getPeakHoldConfig(Bridge const aBridge)                       noexcept { return getBool(bridge2command1458(cCommand4, aBridge), cMask4peakHold1config); } // 24
  void modifyPeakHoldConfig(bool const aValue, Bridge const aBridge) noexcept { modifyBool(bridge2command1458(cCommand4, aBridge), cMask4peakHold1config, aValue); }
  bool readPeakHoldConfig(Bridge const aBridge)                               { return readBool(bridge2command1458(cCommand4, aBridge), cMask4peakHold1config); }
  bool writePeakHoldConfig(bool const aValue, Bridge const aBridge)           { return writeBool(bridge2command1458(cCommand4, aBridge), cMask4peakHold1config, aValue); }
 
  bool getBridgeCurrentLimit(Bridge const aBridge)                 noexcept { return getBool(cCommand9, aBridge == Bridge::c1 ? cMask9bridge1currentLimit : cMask9bridge2currentLimit); } // 26, 25
  bool readBridgeCurrentLimit(Bridge const aBridge)                         { return readBool(cCommand9, aBridge == Bridge::c1 ? cMask9bridge1currentLimit : cMask9bridge2currentLimit); }
  
  void modifyDiagOffPulse(bool const aValue, uint32_t const aChannel) noexcept { modifyValue(cCommand9, cMask9diagOffPulse81, aValue, aChannel); }
  bool writeDiagOffPulse(bool const aValue, uint32_t const aChannel)           { return writeValue(cCommand9, cMask9diagOffPulse81, aValue, aChannel); }
 
  void modifyDiagOnPulse(bool const aValue, uint32_t const aChannel) noexcept { modifyValue(cCommand9, cMask9diagOnPulse81, aValue, aChannel); }
  bool writeDiagOnPulse(bool const aValue, uint32_t const aChannel)           { return writeValue(cCommand9, cMask9diagOnPulse81, aValue, aChannel); }
 
  ChannelDiagnostics getChannelDiagnostics(uint32_t const aChannel) noexcept { 
    uint32_t all = getValue(cCommand9, cMask9diagnosticBit2ch81 | cMask9diagnosticBit1ch81 | cMask9diagnosticBit0ch81);
    return static_cast<ChannelDiagnostics>((all >> aChannel) & cMaskChannelDiagnostics);
  }

  ChannelDiagnostics readChannelDiagnostics(uint32_t const aChannel);

  /// There is no modify version, because it would cache this destructive command. After a subsequent write, an unintentional BIST would occur.
  /// The caller should check if the 3 ms has elapsed after issuing this command.
  bool writeBistHwscRequest(RequestBist const aValue) { 
    bool result = writeEnum(cCommand10, cMask10bistHwscRequest, static_cast<uint32_t>(aValue));
    mWriteCache[cCommand10] &= ~cMask10bistHwscRequest;
    return result;
  }
 
  void modifyConfigCommCheck(RequestCommCheck const aValue) noexcept { modifyEnum(cCommand10, cMask10configCommCheck, static_cast<uint32_t>(aValue)); } // 3-
  bool writeConfigCommCheck(RequestCommCheck const aValue)           { return writeEnum(cCommand10, cMask10configCommCheck, static_cast<uint32_t>(aValue)); }
 
  bool getEn6disableLatch()                noexcept { return getBool(cCommand10, cMask10en6disableLatch); } // 26
  bool readEn6disableLatch()                        { return readBool(cCommand10, cMask10en6disableLatch); }

  bool getEn6disableState()                noexcept { return getBool(cCommand10, cMask10en6disableState); } // 25
  bool readEn6disableState()                        { return readBool(cCommand10, cMask10en6disableState); }
  
  bool getVddOvDisableLatch()              noexcept { return getBool(cCommand10, cMask10vddOvDisableLatch); } // 24
  bool readVddOvDisableLatch()                      { return readBool(cCommand10, cMask10vddOvDisableLatch); }
  
  bool getVddUvDisableState()              noexcept { return getBool(cCommand10, cMask10vddUvDisableState); } // 23
  bool readVddUvDisableState()                      { return readBool(cCommand10, cMask10vddUvDisableState); }
  
  bool getVddUvDisableLatch()              noexcept { return getBool(cCommand10, cMask10vddUvDisableLatch); } // 22
  bool readVddUvDisableLatch()                      { return readBool(cCommand10, cMask10vddUvDisableLatch); }

  bool getDeviceDisState()                 noexcept { return getBool(cCommand10, cMask10deviceDisState); } // 21
  bool readDeviceDisState()                         { return readBool(cCommand10, cMask10deviceDisState); }
  
  bool getDeviceDisLatch()                 noexcept { return getBool(cCommand10, cMask10deviceDisLatch); } // 20
  bool readDeviceDisLatch()                         { return readBool(cCommand10, cMask10deviceDisLatch); }
  
  bool getDeviceNdisOnState()              noexcept { return getBool(cCommand10, cMask10deviceNdisOnState); } // 19
  bool readDeviceNdisOnState()                      { return readBool(cCommand10, cMask10deviceNdisOnState); }
  
  bool getDeviceNdisOnLatch()              noexcept { return getBool(cCommand10, cMask10deviceNdisOnLatch); } // 18
  bool readDeviceNdisOnLatch()                      { return readBool(cCommand10, cMask10deviceNdisOnLatch); }

  bool getDeviceNdisOutLatch()             noexcept { return getBool(cCommand10, cMask10deviceNdisOutLatch); } // 17
  bool readDeviceNdisOutLatch()                     { return readBool(cCommand10, cMask10deviceNdisOutLatch); }

  bool getConfigCommCheckState()           noexcept { return getBool(cCommand10, cMask10configCommCheckState); } // 16
  bool readConfigCommCheckState()                   { return readBool(cCommand10, cMask10configCommCheckState); }

  bool getCommCheckLatch()                 noexcept { return getBool(cCommand10, cMask10commCheckLatch); } // 15
  bool readCommCheckLatch()                         { return readBool(cCommand10, cMask10commCheckLatch); }

  bool getBistDone()                       noexcept { return getBool(cCommand10, cMask10bistDone); } // 14
  bool readBistDone()                               { return readBool(cCommand10, cMask10bistDone); }

  BistResult getBistResult()               noexcept { return static_cast<BistResult>(getEnum(cCommand10, cMask10bistDisableLatch)); } // 13
  BistResult readBistResult()                       { return static_cast<BistResult>(readEnum(cCommand10, cMask10bistDisableLatch)); }

  bool getHwscDone()                       noexcept { return getBool(cCommand10, cMask10hwscDone); } // 12
  bool readHwscDone()                               { return readBool(cCommand10, cMask10hwscDone); }

  HwscResult getHwscResult()               noexcept { return static_cast<HwscResult>(getEnum(cCommand10, cMask10hwscDisableLatch)); } // 13
  HwscResult readHwscResult()                       { return static_cast<HwscResult>(readEnum(cCommand10, cMask10hwscDisableLatch)); }
  
  bool getVddOvCompState()                 noexcept { return getBool(cCommand10, cMask10vddOvCompState); } // 10
  bool readVddOvCompState()                         { return readBool(cCommand10, cMask10vddOvCompState); }

  bool getVddOvCompLatch()                 noexcept { return getBool(cCommand10, cMask10vddOvCompLatch); } // 9
  bool readVddOvCompLatch()                         { return readBool(cCommand10, cMask10vddOvCompLatch); }

  bool getVddUvCompState()                 noexcept { return getBool(cCommand10, cMask10vddUvCompState); } // 8
  bool readVddUvCompState()                         { return readBool(cCommand10, cMask10vddUvCompState); }
  
  bool getVddUvCompLatch()                 noexcept { return getBool(cCommand10, cMask10vddUvCompLatch); } // 7
  bool readVddUvCompLatch()                         { return readBool(cCommand10, cMask10vddUvCompLatch); }
 
  bool getPowerOnResetLatch()              noexcept { return getBool(cCommand10, cMask10powerOnResetLatch); } // 6
  bool readPowerOnResetLatch()                      { return readBool(cCommand10, cMask10powerOnResetLatch); }

  bool getNresLatch()                      noexcept { return getBool(cCommand10, cMask10nResLatch); } // 5
  bool readNresLatch()                              { return readBool(cCommand10, cMask10nResLatch); }

  VgbhiUvStatus getVcpUvState()            noexcept { return static_cast<VgbhiUvStatus>(getEnum(cCommand10, cMask10vcpUvState)); } // 4
  VgbhiUvStatus readVcpUvState()                    { return static_cast<VgbhiUvStatus>(readEnum(cCommand10, cMask10vcpUvState)); }

  bool getVcpUvLatch()                     noexcept { return getBool(cCommand10, cMask10vcpUvLatch); } // 3
  bool readVcpUvLatch()                             { return readBool(cCommand10, cMask10vcpUvLatch); }

  VpsStatus getVpsUvState()                noexcept { return static_cast<VpsStatus>(getEnum(cCommand10, cMask10vpsUvState)); } // 2
  VpsStatus readVpsUvState()                        { return static_cast<VpsStatus>(readEnum(cCommand10, cMask10vpsUvState)); }

  bool getVpsUvLatch()                     noexcept { return getBool(cCommand10, cMask10vpsUvLatch); } // 1
  bool readVpsUvLatch()                             { return readBool(cCommand10, cMask10vpsUvLatch); }

  bool getExternalFetOnStatus(uint32_t const aChannel) noexcept;
  bool readExternalFetOnStatus(uint32_t const aChannel);

  bool getExternalFetCommand(uint32_t const aChannel)  noexcept { return getValue(channel2command1112(aChannel), cMask1112externalFetCommand4185, channel18toChannel1458(aChannel)); }
  bool readExternalFetCommand(uint32_t const aChannel)          { return readValue(channel2command1112(aChannel), cMask1112externalFetCommand4185, channel18toChannel1458(aChannel)); }

  CurrentSource getCurrentSourceStatus(uint32_t const aChannel) noexcept;
  CurrentSource readCurrentSourceStatus(uint32_t const aChannel);

  bool getNdisProtectLatch()               noexcept { return getBool(cCommand13, cMask13ndisProtectLatch); } // 23
  bool readNdisProtectLatch()                       { return readBool(cCommand13, cMask13ndisProtectLatch); }

  bool getOverTempState()                  noexcept { return getBool(cCommand13, cMask13overTempState); } // 22
  bool readOverTempState()                          { return readBool(cCommand13, cMask13overTempState); }

  bool getSdoOvLatch()                     noexcept { return getBool(cCommand13, cMask13sdoOvLatch); } // 21
  bool readSdoOvLatch()                             { return readBool(cCommand13, cMask13sdoOvLatch); }
  
  float getTemperature()                     noexcept { return bin2temperature(getValue(cCommand13, cMask13tempAdc)); } // 11-
  float readTemperature()                             { return bin2temperature(readValue(cCommand13, cMask13tempAdc)); }

  float getBatteryVoltage()                  noexcept { return bin2voltage(getValue(cCommand13, cMask13vpsAdc)); } // 11-
  float readBatteryVoltage()                          { return bin2voltage(readValue(cCommand13, cMask13vpsAdc)); }

  ChannelTdiagOff getTimerDiagOff(uint32_t const aChannel)                       noexcept { return static_cast<ChannelTdiagOff>(getEnum(channel2command18(aChannel), cMask81tDiagConfig81)); } // 22-
  void modifyTimerDiagOff(ChannelTdiagOff const aValue, uint32_t const aChannel) noexcept { modifyEnum(channel2command18(aChannel), cMask81tDiagConfig81, static_cast<uint32_t>(aValue)); }
  ChannelTdiagOff readTimerDiagOff(uint32_t const aChannel)                               { return static_cast<ChannelTdiagOff>(readEnum(channel2command18(aChannel), cMask81tDiagConfig81)); }
  bool writeTimerDiagOff(ChannelTdiagOff const aValue, uint32_t const aChannel)           { return writeEnum(channel2command18(aChannel), cMask81tDiagConfig81, static_cast<uint32_t>(aValue)); }

  ChannelOcThreasholdToRead getOcThreasholdToRead(uint32_t const aChannel)                       noexcept { return static_cast<ChannelOcThreasholdToRead>(getEnum(channel2command18(aChannel), cMask81ocRead81)); } // 21-
  void modifyOcThreasholdToRead(ChannelOcThreasholdToRead const aValue, uint32_t const aChannel) noexcept { modifyEnum(channel2command18(aChannel), cMask81ocRead81, static_cast<uint32_t>(aValue)); }
  ChannelOcThreasholdToRead readOcThreasholdToRead(uint32_t const aChannel)                               { return static_cast<ChannelOcThreasholdToRead>(readEnum(channel2command18(aChannel), cMask81ocRead81)); }
  bool writeOcThreasholdToRead(ChannelOcThreasholdToRead const aValue, uint32_t const aChannel)           { return writeEnum(channel2command18(aChannel), cMask81ocRead81, static_cast<uint32_t>(aValue)); }

  // These use one common parameter pair to do the conversions,
  // because the tolerance in each case is much higher than the difference between LS and HS values.
  float getOcDetectTreshold(uint32_t const aChannel)                       noexcept { return bin2ocDetectTreshold(getValue(channel2command18(aChannel), cMask81ocConfig81)); } // 15-
  void modifyOcDetectTreshold(float const aValue, uint32_t const aChannel) noexcept { modifyValue(channel2command18(aChannel), cMask81ocConfig81, ocDetectTreshold2bin(aValue)); }
  float readOcDetectTreshold(uint32_t const aChannel)                               { return bin2ocDetectTreshold(readValue(channel2command18(aChannel), cMask81ocConfig81)); }
  bool writeOcDetectTreshold(float const aValue, uint32_t const aChannel)           { return writeValue(channel2command18(aChannel), cMask81ocConfig81, ocDetectTreshold2bin(aValue)); }

  ChannelOcTempComp getOcTempCompensation(uint32_t const aChannel)                       noexcept { return static_cast<ChannelOcTempComp>(getEnum(channel2command18(aChannel), cMask81ocTempComp81)); } // 13-
  void modifyOcTempCompensation(ChannelOcTempComp const aValue, uint32_t const aChannel) noexcept { modifyEnum(channel2command18(aChannel), cMask81ocTempComp81, static_cast<uint32_t>(aValue)); }
  ChannelOcTempComp readOcTempCompensation(uint32_t const aChannel)                               { return static_cast<ChannelOcTempComp>(readEnum(channel2command18(aChannel), cMask81ocTempComp81)); }
  bool writeOcTempCompensation(ChannelOcTempComp const aValue, uint32_t const aChannel)           { return writeEnum(channel2command18(aChannel), cMask81ocTempComp81, static_cast<uint32_t>(aValue)); }

  bool getOcBatteryCompensation(uint32_t const aChannel)                       noexcept { return getBool(channel2command18(aChannel), cMask81ocBattComp81); } // 12
  void modifyOcBatteryCompensation(bool const aValue, uint32_t const aChannel) noexcept { modifyBool(channel2command18(aChannel), cMask81ocBattComp81, aValue); }
  bool readOcBatteryCompensation(uint32_t const aChannel)                               { return readBool(channel2command18(aChannel), cMask81ocBattComp81); }
  bool writeOcBatteryCompensation(bool const aValue, uint32_t const aChannel)           { return writeBool(channel2command18(aChannel), cMask81ocBattComp81, aValue); }

  ChannelOcBlankTime getOcBlankTime(uint32_t const aChannel)                       noexcept { return static_cast<ChannelOcBlankTime>(getEnum(channel2command18(aChannel), cMask81tBlankOc81)); } // 9-
  void modifyOcBlankTime(ChannelOcBlankTime const aValue, uint32_t const aChannel) noexcept { modifyEnum(channel2command18(aChannel), cMask81tBlankOc81, static_cast<uint32_t>(aValue)); }
  ChannelOcBlankTime readOcBlankTime(uint32_t const aChannel)                               { return static_cast<ChannelOcBlankTime>(readEnum(channel2command18(aChannel), cMask81tBlankOc81)); }
  bool writeOcBlankTime(ChannelOcBlankTime const aValue, uint32_t const aChannel)           { return writeEnum(channel2command18(aChannel), cMask81tBlankOc81, static_cast<uint32_t>(aValue)); }

  ChannelOutputReEngage getOutputReEngage(uint32_t const aChannel)                       noexcept { return static_cast<ChannelOutputReEngage>(getEnum(channel2command18(aChannel), cMask81protConfig81)); } // 8
  void modifyOutputReEngage(ChannelOutputReEngage const aValue, uint32_t const aChannel) noexcept { modifyEnum(channel2command18(aChannel), cMask81protConfig81, static_cast<uint32_t>(aValue)); }
  ChannelOutputReEngage readOutputReEngage(uint32_t const aChannel)                               { return static_cast<ChannelOutputReEngage>(readEnum(channel2command18(aChannel), cMask81protConfig81)); }
  bool writeOutputReEngage(ChannelOutputReEngage const aValue, uint32_t const aChannel)           { return writeEnum(channel2command18(aChannel), cMask81protConfig81, static_cast<uint32_t>(aValue)); }

  ChannelOutputOcMeasure getOutputOcMeasure(uint32_t const aChannel)                       noexcept { return static_cast<ChannelOutputOcMeasure>(getEnum(channel2command18(aChannel), cMask81ocDsShunt81)); } // 7
  void modifyOutputOcMeasure(ChannelOutputOcMeasure const aValue, uint32_t const aChannel) noexcept { modifyEnum(channel2command18(aChannel), cMask81ocDsShunt81, static_cast<uint32_t>(aValue)); }
  ChannelOutputOcMeasure readOutputOcMeasure(uint32_t const aChannel)                               { return static_cast<ChannelOutputOcMeasure>(readEnum(channel2command18(aChannel), cMask81ocDsShunt81)); }
  bool writeOutputOcMeasure(ChannelOutputOcMeasure const aValue, uint32_t const aChannel)           { return writeEnum(channel2command18(aChannel), cMask81ocDsShunt81, static_cast<uint32_t>(aValue)); }

  ChannelOlOutCurrCapability getOlOutCurrCapability(uint32_t const aChannel)                       noexcept { return static_cast<ChannelOlOutCurrCapability>(getEnum(channel2command18(aChannel), cMask81diagIconfig81)); } // 6
  void modifyOlOutCurrCapability(ChannelOlOutCurrCapability const aValue, uint32_t const aChannel) noexcept { modifyEnum(channel2command18(aChannel), cMask81diagIconfig81, static_cast<uint32_t>(aValue)); }
  ChannelOlOutCurrCapability readOlOutCurrCapability(uint32_t const aChannel)                               { return static_cast<ChannelOlOutCurrCapability>(readEnum(channel2command18(aChannel), cMask81diagIconfig81)); }
  bool writeOlOutCurrCapability(ChannelOlOutCurrCapability const aValue, uint32_t const aChannel)           { return writeEnum(channel2command18(aChannel), cMask81diagIconfig81, static_cast<uint32_t>(aValue)); }

  ChannelGateCurrent getGateCurrent(uint32_t const aChannel)                       noexcept { return static_cast<ChannelGateCurrent>(getEnum(channel2command18(aChannel), cMask81gccConfig81)); } // 4-
  void modifyGateCurrent(ChannelGateCurrent const aValue, uint32_t const aChannel) noexcept { modifyEnum(channel2command18(aChannel), cMask81gccConfig81, static_cast<uint32_t>(aValue)); }
  ChannelGateCurrent readGateCurrent(uint32_t const aChannel)                               { return static_cast<ChannelGateCurrent>(readEnum(channel2command18(aChannel), cMask81gccConfig81)); }
  bool writeGateCurrent(ChannelGateCurrent const aValue, uint32_t const aChannel)           { return writeEnum(channel2command18(aChannel), cMask81gccConfig81, static_cast<uint32_t>(aValue)); }

  ChannelHsFet getHsFet(uint32_t const aChannel)                       noexcept { return static_cast<ChannelHsFet>(getEnum(channel2command18(aChannel), cMask81nPconfig81)); } // 3
  void modifyHsFet(ChannelHsFet const aValue, uint32_t const aChannel) noexcept { modifyEnum(channel2command18(aChannel), cMask81nPconfig81, static_cast<uint32_t>(aValue)); }
  ChannelHsFet readHsFet(uint32_t const aChannel)                               { return static_cast<ChannelHsFet>(readEnum(channel2command18(aChannel), cMask81nPconfig81)); }
  bool writeHsFet(ChannelHsFet const aValue, uint32_t const aChannel)           { return writeEnum(channel2command18(aChannel), cMask81nPconfig81, static_cast<uint32_t>(aValue)); }

  ChannelSide getSide(uint32_t const aChannel)                       noexcept { return static_cast<ChannelSide>(getEnum(channel2command18(aChannel), cMask81lsHsConfig81)); } // 2
  void modifySide(ChannelSide const aValue, uint32_t const aChannel) noexcept { modifyEnum(channel2command18(aChannel), cMask81lsHsConfig81, static_cast<uint32_t>(aValue)); }
  ChannelSide readSide(uint32_t const aChannel)                               { return static_cast<ChannelSide>(readEnum(channel2command18(aChannel), cMask81lsHsConfig81)); }
  bool writeSide(ChannelSide const aValue, uint32_t const aChannel)           { return writeEnum(channel2command18(aChannel), cMask81lsHsConfig81, static_cast<uint32_t>(aValue)); }

  bool getOutputEnable(uint32_t const aChannel)                       noexcept { return getBool(channel2command18(aChannel), cMask81enOut81); } // 1
  void modifyOutputEnable(bool const aValue, uint32_t const aChannel) noexcept { modifyBool(channel2command18(aChannel), cMask81enOut81, aValue); }
  bool readOutputEnable(uint32_t const aChannel)                               { return readBool(channel2command18(aChannel), cMask81enOut81); }
  bool writeOutputEnable(bool const aValue, uint32_t const aChannel)           { return writeBool(channel2command18(aChannel), cMask81enOut81, aValue); }

  bool readAllIntoCache();
  bool readIntoCache(uint32_t const aCommand);
  bool writeAllFromCache();

  bool writeFromCache(uint32_t const aCommand) {
    return write(aCommand, mWriteCache[aCommand]);
  }

  uint32_t getReadCache(uint32_t const aCommand) const {
    return mReadCache[aCommand];
  }

  enum class DiagnosticsTest : uint8_t {
    cNone = 0u,
    cAuto = 1u,
    cOffPulse = 2u, // requires 1 ms to finish
    cOnPulse = 3u, // requires 1 ms to finish
    cBist = 4u  // requires 3 ms to finish
  };

  enum class StatusLatch : uint8_t {
    cBoth0 = 0u,
    cStatus1 = 1u,
    cLatch1 = 2u,
    cBoth1 = 3u
  };

  class DiagnosticsResult final {
    friend class L9945;
  private:
    static constexpr uint32_t            cWaitForTest[] = { 0u, 0u, 1u, 1u, 3u };
    static constexpr char                cTextDiagnosticsTest[][10u]    = { "None", "Auto", "OffPulse", "OnPulse", "Bist" };
    static constexpr char                cTextChannelDiagnostics[][16u] = { "OcPinFail", "OcFail", "StgStbFail", "OlFail", "NoFail", "NoOcFail", "NoOlStgStbFail", "NoDiagDone"};
    static constexpr char                cTextStatusLatch[][8u]         = { "Both0", "Status1", "Latch1", "Both1" };
    static constexpr char                cTextCurrentSource[][8u]       = { "Corrupt", "FetOn", "FetOff", "Fet3st"};
    L9945                               *mParent;
    DiagnosticsTest                      mTestPerformed = DiagnosticsTest::cNone;
    std::array<bool, 8u>                 mChannelsDiagnosed;
    std::array<uint32_t, cRegisterCount> mReadCache;

  public:
    DiagnosticsResult(L9945 *aParent) noexcept : mParent(aParent) {
    }

    DiagnosticsResult(DiagnosticsResult const&) = default;
    DiagnosticsResult(DiagnosticsResult&&) = default;
    DiagnosticsResult& operator=(DiagnosticsResult const&) = default;
    DiagnosticsResult& operator=(DiagnosticsResult&&) = default;

    uint8_t getSpiOnOut() const noexcept;
    uint8_t getHsFetIsP() const noexcept;
    uint8_t getSideIsHs() const noexcept;
    uint8_t getOutputEnable() const noexcept;
    std::optional<bool> getBridgeCurrentLimit(Bridge const aBridge) const noexcept;
    std::optional<ChannelDiagnostics> getChannelDiagnostics(uint32_t const aChannel) const noexcept;

    StatusLatch getEn6disable() const noexcept {
      return getStatusLatch10(cMask10en6disableState, cMask10en6disableLatch);
    }

    bool getVddOvDisableLatch() const noexcept {
      return (mReadCache[cCommand10] & cMask10vddOvDisableLatch) > 0u;
    }

    StatusLatch getVddUvDisable() const noexcept {
      return getStatusLatch10(cMask10vddUvDisableState, cMask10vddUvDisableLatch);
    }

    StatusLatch getDeviceDis() const noexcept {
      return getStatusLatch10(cMask10deviceDisState, cMask10deviceDisLatch);
    }

    StatusLatch getDeviceNdisOn() const noexcept {
      return getStatusLatch10(cMask10deviceNdisOnState, cMask10deviceNdisOnLatch);
    }

    bool getDeviceNdisOutLatch() const noexcept {
      return (mReadCache[cCommand10] & cMask10deviceNdisOutLatch) > 0u;
    }
    
    std::optional<bool> getCommCheckLatch() const noexcept;
    std::optional<bool> getBistResult() const noexcept;
    std::optional<bool> getHwscResult() const noexcept;

    StatusLatch getVddOvComp() const noexcept {
      return getStatusLatch10(cMask10vddOvCompState, cMask10vddOvCompLatch);
    }

    StatusLatch getVddUvComp() const noexcept {
      return getStatusLatch10(cMask10vddUvCompState, cMask10vddUvCompLatch);
    }

    bool getPowerOnResetLatch() const noexcept {
      return (mReadCache[cCommand10] & cMask10powerOnResetLatch) > 0u;
    }

    bool getNresLatch() const noexcept {
      return (mReadCache[cCommand10] & cMask10nResLatch) > 0u;
    }

    StatusLatch getVcpUv() const noexcept {
      return getStatusLatch10(cMask10vcpUvState, cMask10vcpUvLatch);
    }

    StatusLatch getVpsUv() const noexcept {
      return getStatusLatch10(cMask10vpsUvState, cMask10vpsUvLatch);
    }

    uint8_t getExternalFetOnStatus() const noexcept;

    uint8_t getExternalFetCommand() const noexcept {
      return  (mReadCache[cCommand11] & cMask1112externalFetCommand4185) >> l9945::getRightmost1position(cMask1112externalFetCommand4185)
           | ((mReadCache[cCommand12] & cMask1112externalFetCommand4185) << 4u) >> l9945::getRightmost1position(cMask1112externalFetCommand4185);
    }

    CurrentSource getCurrentSourceStatus(uint32_t const aChannel) const noexcept;

    bool getNdisProtectLatch() const noexcept {
      return (mReadCache[cCommand13] & cMask13ndisProtectLatch) > 0u;
    }

    bool getOverTempState() const noexcept {
      return (mReadCache[cCommand13] & cMask13overTempState) > 0u;
    }

    bool getSdoOvLatch() const noexcept {
      return (mReadCache[cCommand13] & cMask13sdoOvLatch) > 0u;
    }

    float getTemperature() const noexcept {
      return mParent->bin2temperature((mReadCache[cCommand13] & cMask13tempAdc) >> l9945::getRightmost1position(cMask13tempAdc));
    }

    float getBatteryVoltage() const noexcept {
      return mParent->bin2voltage((mReadCache[cCommand13] & cMask13vpsAdc) >> l9945::getRightmost1position(cMask13vpsAdc));
    }

    void log() noexcept;

  private:
    void appendBoolOptional(std::optional<bool> const aResult, char const* const aPresent, char const* const aMissing) noexcept;
    void perform(DiagnosticsTest const aTest);
    uint32_t gatherChannels(ChannelOcBlankTime const aTimeLimit, bool const aFetNow) noexcept;

    StatusLatch getStatusLatch10(uint32_t const aStatus, uint32_t const aLatch) const noexcept {
      return static_cast<StatusLatch>(((mReadCache[cCommand10] & aStatus) > 0u ? 1u : 0u) | ((mReadCache[cCommand10] & aLatch) > 0u ? 2u : 0u));
    }
  };

  DiagnosticsResult& diagnose(DiagnosticsTest const aTest) {
    mLastResult.perform(aTest);
    return mLastResult;
  }

private:
  DiagnosticsResult mLastResult;

private:
  void setWriteDelay(uint32_t const aWriteDelay) noexcept {
    mWriteDelay = aWriteDelay;
  }

  uint32_t getEnum(uint32_t const aCommand, uint32_t const aFunction) const noexcept {
    return mReadCache[aCommand] & aFunction;
  }

  bool getBool(uint32_t const aCommand, uint32_t const aFunction) const noexcept {
    return (mReadCache[aCommand] & aFunction) > 0u ? true : false;
  }

  uint32_t getValue(uint32_t const aCommand, uint32_t const aFunction) const noexcept {
    return (mReadCache[aCommand] & aFunction) >> l9945::getRightmost1position(aFunction);
  }

  bool getValue(uint32_t const aCommand, uint32_t const aFunction, uint32_t const aChannel) const noexcept {
    return static_cast<bool>(((mReadCache[aCommand] & aFunction) >> (l9945::getRightmost1position(aFunction) + aChannel - 1u)) & 1u);
  }

  void modifyEnum(uint32_t const aCommand, uint32_t const aFunction, uint32_t const aInput) noexcept {
    mWriteCache[aCommand] = (mWriteCache[aCommand] & ~aFunction) | aInput;
  }

  void modifyBool(uint32_t const aCommand, uint32_t const aFunction, bool const aInput) noexcept {
    uint32_t value = (aInput ? 1u : 0u);
    mWriteCache[aCommand] = (mWriteCache[aCommand] & ~aFunction) | (value << l9945::getRightmost1position(aFunction));
  }

  void modifyValue(uint32_t const aCommand, uint32_t const aFunction, uint32_t const aInput) noexcept {
    mWriteCache[aCommand] = (mWriteCache[aCommand] & ~aFunction) | (aInput << l9945::getRightmost1position(aFunction) & aFunction);
  }

  void modifyValue(uint32_t const aCommand, uint32_t const aFunction, uint32_t const aInput, uint32_t const aMask) noexcept {
    mWriteCache[aCommand] = (mWriteCache[aCommand] & ~(aFunction & (aMask << l9945::getRightmost1position(aFunction)))) 
    | ((aInput & aMask) << l9945::getRightmost1position(aFunction));
  }

  void modifyValue(uint32_t const aCommand, uint32_t const aFunction, bool const aInput, uint32_t const aChannel) noexcept {
    mWriteCache[aCommand] = (mWriteCache[aCommand] & ~(aFunction & (1u << (l9945::getRightmost1position(aFunction) + aChannel - 1u)))) 
    | ((aInput ? 1u : 0u) << (l9945::getRightmost1position(aFunction) + aChannel - 1u));
  }

  uint32_t readEnum(uint32_t const aCommand, uint32_t const aFunction)  { return read(aCommand) & aFunction; }
  bool     readBool(uint32_t const aCommand, uint32_t const aFunction)  { return (read(aCommand) & aFunction) > 0u ? true : false; };
  uint32_t readValue(uint32_t const aCommand, uint32_t const aFunction) { return (read(aCommand) & aFunction) >> l9945::getRightmost1position(aFunction); };

  bool     readValue(uint32_t const aCommand, uint32_t const aFunction, uint32_t const aChannel) { 
    return static_cast<bool>(((read(aCommand) & aFunction) >> (l9945::getRightmost1position(aFunction) + aChannel - 1u)) & 1u);
  }

  bool writeEnum(uint32_t const aCommand, uint32_t const aFunction, uint32_t const aInput) {
    return write(aCommand, (mWriteCache[aCommand] & ~aFunction) | aInput);
  }

  bool writeBool(uint32_t const aCommand, uint32_t const aFunction, bool const aInput) {
    uint32_t value = (aInput ? 1u : 0u);
    return write(aCommand, (mWriteCache[aCommand] & ~aFunction) | (value << l9945::getRightmost1position(aFunction)));
  }

  bool writeValue(uint32_t const aCommand, uint32_t const aFunction, uint32_t const aInput) {
    return write(aCommand, (mWriteCache[aCommand] & ~aFunction) | (aInput << l9945::getRightmost1position(aFunction) & aFunction));
  }

  bool writeValue(uint32_t const aCommand, uint32_t const aFunction, uint32_t const aInput, uint32_t const aMask) {
    return write(aCommand, (mWriteCache[aCommand] & ~(aFunction & (aMask << l9945::getRightmost1position(aFunction))))
    | ((aInput & aMask) << l9945::getRightmost1position(aFunction)));
  }

  bool writeValue(uint32_t const aCommand, uint32_t const aFunction, bool const aInput, uint32_t const aChannel) {
    return write(aCommand, (mWriteCache[aCommand] & ~(aFunction & (1u << (l9945::getRightmost1position(aFunction) + aChannel - 1u))))
    | ((aInput ? 1u : 0u) << (l9945::getRightmost1position(aFunction) + aChannel - 1u)));
  }

  uint32_t bridge2command1458(uint32_t const aCommand, Bridge const aBridge) const noexcept {
    return aCommand + static_cast<uint32_t>(aBridge);
  }

  uint32_t channel2command18(uint32_t const aChannel) const noexcept {
    return cCommand1 + ((aChannel - 1u) & cMaskChannel);
  }

  uint32_t channel2command1112(uint32_t const aChannel) const noexcept {
    return cCommand11 + ((aChannel - 1u) / 4u & 1u);
  }

  uint32_t channel18toChannel1458(uint32_t const aChannel) const noexcept {
    return ((aChannel - 1u) % 4u + 1u);
  }

  float bin2ocDetectTreshold(uint32_t const aValue) const noexcept {
    return 60.5f + 15.25f * aValue;
  }

  uint32_t ocDetectTreshold2bin(float const aValue) const noexcept {
    int32_t iRaw = std::min<int32_t>(lround((aValue - 60.5f) / 15.25f), cMask81ocConfig81 >> l9945::getRightmost1position(cMask81ocConfig81));
    return static_cast<uint32_t>(std::max<int32_t>(iRaw, 0));
  }

  float bin2temperature(uint32_t const aValue) const noexcept {
    return (0.28f * aValue) - 65.0f;
  }

  float bin2voltage(uint32_t const aValue) const noexcept {
    return 0.048f * aValue;
  }

  // Any combination of concurrent read and write calls have to be avoided
  uint32_t read(uint32_t const aCommand);

  // Any combination of concurrent read and write calls have to be avoided
  bool write(uint32_t const aCommand, uint32_t const aValue);
  uint32_t spiTransfer(uint32_t const aCommand, uint32_t const aDelay);
  void prepareDataToSend(uint32_t const aValue) noexcept;
  void avoidInitialCommunicationFailure() noexcept;
};

template<typename tInterface>
constexpr uint32_t L9945<tInterface>::cInitialRegisterValues[];

template<typename tInterface>
void L9945<tInterface>::reset() {
  mInterface.enableReset(true);
  tInterface::delayMs(cResetDelay);
  mInterface.enableReset(false);
  tInterface::delayMs(cResetDelay);
  std::copy(cInitialRegisterValues, cInitialRegisterValues + cRegisterCount, mWriteCache.begin());
  avoidInitialCommunicationFailure();
  mSpiFailed = false;
  writeAllFromCache();
  mInterface.enableAll(!mSpiFailed);
}

template<typename tInterface>
void L9945<tInterface>::setPwm(float const aValue, Bridge const aBridge) {
  if (getBridgeConfig(aBridge)) {
    if (!mSpiFailed) {
      mInterface.setPwm(aValue, aBridge);
    }
    else {
      mInterface.setPwm(0u, aBridge);
    }
  }
  else { // nothing to do
  }
}

template<typename tInterface>
void L9945<tInterface>::setPwm(float const aValue, uint32_t const aChannel) {
  Bridge bridge = (aChannel <= 4u ? Bridge::c1 : Bridge::c2);
  if (!getSpiInputSelect(aChannel) && !getBridgeConfig(bridge)) {
    if (!mSpiFailed) {
      mInterface.setPwm(aValue, aChannel);
    }
    else {
      mInterface.setPwm(0u, aChannel);
    }
  }
}

template<typename tInterface>
bool L9945<tInterface>::getSpiOnOut(uint32_t const aChannel) noexcept {
  ChannelSide side = getSide(aChannel);
  bool bit = getValue(cCommand0, cMask0outputVcompared81, aChannel);
  return (bit && side == ChannelSide::cHs) || (!bit && side == ChannelSide::cLs);
}

template<typename tInterface>
bool L9945<tInterface>::readSpiOnOut(uint32_t const aChannel) {
  readIntoCache(cCommand0);
  readIntoCache(channel2command18(aChannel));
  return getSpiOnOut(aChannel);
}

template<typename tInterface>
typename L9945<tInterface>::ChannelDiagnostics L9945<tInterface>::readChannelDiagnostics(uint32_t const aChannel) {
  uint32_t all = readValue(cCommand9, cMask9diagnosticBit2ch81 | cMask9diagnosticBit1ch81 | cMask9diagnosticBit0ch81);
  return static_cast<ChannelDiagnostics>((all >> aChannel) & cMaskChannelDiagnostics);
}

template<typename tInterface>
bool L9945<tInterface>::getExternalFetOnStatus(uint32_t const aChannel) noexcept {
  ChannelSide side = getSide(aChannel);
  bool bit = getValue(channel2command1112(aChannel), cMask1112externalFetState4185, channel18toChannel1458(aChannel));
  return (bit && side == ChannelSide::cHs) || (!bit && side == ChannelSide::cLs);
}

template<typename tInterface>
bool L9945<tInterface>::readExternalFetOnStatus(uint32_t const aChannel) {
  readIntoCache(channel2command1112(aChannel));
  readIntoCache(channel2command18(aChannel));
  return getExternalFetOnStatus(aChannel);
}

template<typename tInterface>
typename L9945<tInterface>::CurrentSource L9945<tInterface>::getCurrentSourceStatus(uint32_t const aChannel) noexcept {
  uint32_t effectiveChannel = channel18toChannel1458(aChannel) - 1u;
  ChannelHsFet hsFetPolarity = getHsFet(aChannel);
  ChannelSide side = getSide(aChannel);
  uint32_t value = getValue(channel2command1112(aChannel), cMask1112channelPullUpDown15 << (3u * effectiveChannel));
  return cCurrentSourceDecoder[value | (side == ChannelSide::cHs && hsFetPolarity == ChannelHsFet::cPmos ? 0u : 8u)];
}

template<typename tInterface>
typename L9945<tInterface>::CurrentSource L9945<tInterface>::readCurrentSourceStatus(uint32_t const aChannel) {
  readIntoCache(channel2command1112(aChannel));
  readIntoCache(channel2command18(aChannel));
  return getCurrentSourceStatus(aChannel);
}

template <typename tInterface>
bool L9945<tInterface>::readAllIntoCache() { // TODO can be implemented in chained HAL_SPI_Transmit calls without dummy word if needed
  bool result = true;
  for (size_t command = 0u; result && command < cRegisterCount; ++command) {
    mReadCache[command] = read(command);
  }
  return mSpiFailed;
}

template<typename tInterface>
bool L9945<tInterface>::readIntoCache(uint32_t const aCommand) {
  mReadCache[aCommand] = read(aCommand);
  return mSpiFailed;
}

template<typename tInterface>
bool L9945<tInterface>::writeAllFromCache() { // TODO can be implemented in chained HAL_SPI_Transmit calls without dummy word if needed
  bool result = true;
  for (size_t command = 0u; result && command < cRegisterCount; ++command) {
    if (!write(command, mWriteCache[command])) {
      result = false;
    }
    else { // nothing to do
    }
  }
  return result;
}

template<typename tInterface>
uint32_t L9945<tInterface>::read(uint32_t const aCommand) {
  prepareDataToSend(cFixedPatternValues[aCommand] | cMaskRead);
  return spiTransfer(aCommand, cNoDelay);
}

// Any combination of concurrent read and write calls have to be avoided
template<typename tInterface>
bool L9945<tInterface>::write(uint32_t const aCommand, uint32_t const aValue) {
  uint32_t result;
  uint32_t toWrite = (aValue & ~(cMaskRead | cFixedPatternMasks[aCommand])) | cFixedPatternValues[aCommand];
  mWriteCache[aCommand] = toWrite;
  prepareDataToSend(toWrite);
  uint32_t delay = mWriteDelay;
  mWriteDelay = cNoDelay;        // prepare for possible exception
  return spiTransfer(aCommand, delay) != cInvalidResponse;
}

template<typename tInterface>
uint32_t L9945<tInterface>::spiTransfer(uint32_t const aCommand, uint32_t const aDelay) {
  uint32_t result = cInvalidResponse;
  mInterface.enableSpiTransfer(true);
  SpiResult spiResult1 = SpiResult::cOk;
  SpiResult spiResult2 = SpiResult::cOk;
  bool success = (!mSpiFailed && aCommand < cRegisterCount && (spiResult1 = mInterface.spiTransmitReceive(mDataOut, mDataIn, cSizeofRegister)) == SpiResult::cOk);
  mInterface.enableSpiTransfer(false);
  tInterface::delayMs(aDelay);
  mInterface.enableSpiTransfer(true);
  if (success && (spiResult2 = mInterface.spiTransmitReceive(mDataOut + cSizeofRegister, mDataIn, cSizeofRegister)) == SpiResult::cOk) {
    result = (static_cast<uint32_t>(mDataIn[0]) << 24u) |
      (static_cast<uint32_t>(mDataIn[1]) << 16u) |
      (static_cast<uint32_t>(mDataIn[2]) << 8u) |
      mDataIn[3];
  }
  else { // nothing to do
  }
  mInterface.enableSpiTransfer(false);
  if (!mSpiFailed) {
    if(spiResult1 != SpiResult::cOk || spiResult2 != SpiResult::cOk) {
      mSpiFailed = true;
      mInterface.enableAll(false);
      mInterface.fatalError(Exception::cCommunication);
      result = cInvalidResponse;
    }
    else if (l9945::calculateParity(result) == cInvalidParity) {
      mSpiFailed = true;
      mInterface.enableAll(false);
      mInterface.fatalError(Exception::cParity);
      result = cInvalidResponse;
    }
    else { // nothing to do
    }
  }
  else { // nothing to do
  }
  mReadCache[aCommand] = result;
  return result;
}

template<typename tInterface>
void L9945<tInterface>::prepareDataToSend(uint32_t const aValue) noexcept {
  uint32_t actual = aValue ^ l9945::calculateParity(aValue);
  mDataOut[0u] = actual >> 24u;
  mDataOut[1u] = actual >> 16u;
  mDataOut[2u] = actual >> 8u;
  mDataOut[3u] = actual;
}

template<typename tInterface>
void L9945<tInterface>::avoidInitialCommunicationFailure() noexcept {
  uint32_t toWrite = (cInitialRegisterValues[cCommand13] & ~(cMaskRead | cFixedPatternMasks[cCommand13])) | cFixedPatternValues[cCommand13];
  mWriteCache[cCommand13] = toWrite;
  prepareDataToSend(toWrite);
  mInterface.enableSpiTransfer(true);
  mInterface.spiTransmitReceive(mDataOut, mDataIn, cSizeofRegister);
  mInterface.enableSpiTransfer(false);
  mInterface.enableSpiTransfer(true);
  mInterface.spiTransmitReceive(mDataOut + cSizeofRegister, mDataIn, cSizeofRegister);
  mInterface.enableSpiTransfer(false);
}

template<typename tInterface>
uint8_t L9945<tInterface>::DiagnosticsResult::getSpiOnOut() const noexcept {
  uint8_t result = (mReadCache[cCommand0] & cMask0outputVcompared81) >> l9945::getRightmost1position(cMask0outputVcompared81);
  for (uint32_t i = 0; i < cChannelCount; ++i) {             // LS 0   HS 1
    result ^= ((mReadCache[i + 1u] & cMask81lsHsConfig81) == 0u ? 1u : 0u) << i;
  }
  return result;
}

template<typename tInterface>
uint8_t L9945<tInterface>::DiagnosticsResult::getHsFetIsP() const noexcept {
  uint8_t result = 0u;
  for (uint32_t i = 0; i < cChannelCount; ++i) {
    result |= (((mReadCache[i + 1u] & cMask81nPconfig81) >> l9945::getRightmost1position(cMask81nPconfig81)) & 1u) << i;
  }
  return result;
}
  
template<typename tInterface>
uint8_t L9945<tInterface>::DiagnosticsResult::getSideIsHs() const noexcept {
  uint8_t result = 0u;
  for (uint32_t i = 0; i < cChannelCount; ++i) {
    result |= (((mReadCache[i + 1u] & cMask81lsHsConfig81) >> l9945::getRightmost1position(cMask81lsHsConfig81)) & 1u) << i;
  }
  return result;
}
  
template<typename tInterface>
uint8_t L9945<tInterface>::DiagnosticsResult::getOutputEnable() const noexcept {
  uint8_t result = 0u;
  for (uint32_t i = 0; i < cChannelCount; ++i) {
    result |= (((mReadCache[i + 1u] & cMask81enOut81) >> l9945::getRightmost1position(cMask81enOut81)) & 1u) << i;
  }
  return result;
}

template<typename tInterface>
std::optional<bool> L9945<tInterface>::DiagnosticsResult::getBridgeCurrentLimit(Bridge const aBridge) const noexcept {
  uint32_t commandEnable = (aBridge == Bridge::c1 ? cCommand3 : cCommand7);
  uint32_t commandConfig = (aBridge == Bridge::c1 ? cCommand4 : cCommand8);
  uint32_t maskLimit     = (aBridge == Bridge::c1 ? cMask9bridge1currentLimit : cMask9bridge2currentLimit);
  std::optional<bool> result;
  if ((mReadCache[commandEnable] & cMask3bridge1currentLimitEn) > 0u &&
    (mReadCache[commandConfig] & cMask4bridge1config) > 0u) {
    result = (mReadCache[cCommand9] & maskLimit) > 0u;
  }
  else { // nothing to do
  }
  return result;
}

template<typename tInterface>
std::optional<typename L9945<tInterface>::ChannelDiagnostics> L9945<tInterface>::DiagnosticsResult::getChannelDiagnostics(uint32_t const aChannel) const noexcept {
  uint32_t all = mReadCache[cCommand9] & (cMask9diagnosticBit2ch81 | cMask9diagnosticBit1ch81 | cMask9diagnosticBit0ch81);
  std::optional<ChannelDiagnostics> result;
  if (aChannel > 0u && aChannel <= cChannelCount && mChannelsDiagnosed[aChannel - 1u] > 0u &&
    (mTestPerformed == DiagnosticsTest::cAuto || mTestPerformed == DiagnosticsTest::cOffPulse || mTestPerformed == DiagnosticsTest::cOnPulse)) {
    result = static_cast<ChannelDiagnostics>((all >> aChannel) & cMaskChannelDiagnostics);
  }
  else { // nothing to do
  }
  return result;
}

template<typename tInterface>
std::optional<bool> L9945<tInterface>::DiagnosticsResult::getCommCheckLatch() const noexcept {
  std::optional<bool> result;
  if ((mReadCache[cCommand10] & cMask10configCommCheckState) > 0u) {
    result = (mReadCache[cCommand10] & cMask10commCheckLatch) > 0u;
  }
  else { // nothing to do
  }
  return result;
}

template<typename tInterface>
std::optional<bool> L9945<tInterface>::DiagnosticsResult::getBistResult() const noexcept {
  std::optional<bool> result;
  if (mTestPerformed == DiagnosticsTest::cBist && (mReadCache[cCommand10] & cMask10bistDone) > 0u) {
    result = (mReadCache[cCommand10] & cMask10bistDisableLatch) > 0u;
  }
  else { // nothing to do
  }
  return result;
}

template<typename tInterface>
std::optional<bool> L9945<tInterface>::DiagnosticsResult::getHwscResult() const noexcept {
  std::optional<bool> result;
  if (mTestPerformed == DiagnosticsTest::cBist && (mReadCache[cCommand10] & cMask10hwscDone) > 0u) {
    result = (mReadCache[cCommand10] & cMask10hwscDisableLatch) > 0u;
  }
  else { // nothing to do
  }
  return result;
}

template<typename tInterface>
uint8_t L9945<tInterface>::DiagnosticsResult::getExternalFetOnStatus() const noexcept {
  uint8_t result = (mReadCache[cCommand11] & cMask1112externalFetState4185) >> l9945::getRightmost1position(cMask1112externalFetState4185);
  result |= ((mReadCache[cCommand12] & cMask1112externalFetState4185) << 4u) >> l9945::getRightmost1position(cMask1112externalFetState4185);
  for (uint32_t i = 0; i < cChannelCount; ++i) {             // LS 0   HS 1
    result ^= ((mReadCache[i + 1u] & cMask81lsHsConfig81) == 0u ? 1u : 0u) << i;
  }
  return result;
}

template<typename tInterface>
typename L9945<tInterface>::CurrentSource L9945<tInterface>::DiagnosticsResult::getCurrentSourceStatus(uint32_t const aChannel) const noexcept {
  CurrentSource result;
  if (aChannel > 0u && aChannel <= cChannelCount) {
    uint32_t effectiveChannel = mParent->channel18toChannel1458(aChannel) - 1u;
    ChannelHsFet hsFetPolarity = static_cast<ChannelHsFet>(mReadCache[aChannel] & cMask81nPconfig81);
    ChannelSide side = static_cast<ChannelSide>(mReadCache[aChannel] & cMask81lsHsConfig81);
    uint32_t value = mReadCache[mParent->channel2command1112(aChannel)] & cMask1112channelPullUpDown15 << (3u * effectiveChannel);
    value >>= l9945::getRightmost1position(cMask1112channelPullUpDown15) + 3u * effectiveChannel;
    result = cCurrentSourceDecoder[value | (side == ChannelSide::cHs && hsFetPolarity == ChannelHsFet::cPmos ? 0u : 8u)];
  }
  else {
    result = CurrentSource::cCompromised;
  }
  return result;
}

template<typename tInterface>
void L9945<tInterface>::DiagnosticsResult::log() noexcept {
  mParent->mInterface.open();
  mParent->mInterface << "Test: " << cTextDiagnosticsTest[static_cast<size_t>(mTestPerformed)] << '\n';
  mParent->mInterface << "HS PFET: " << getHsFetIsP() << '\n';
  mParent->mInterface << "channel HS: " << getSideIsHs() << '\n';
  mParent->mInterface << "out on: " << getSpiOnOut() << '\n';
  mParent->mInterface << "out en: " << getOutputEnable() << '\n';
  appendBoolOptional(getBridgeCurrentLimit(Bridge::c1), "bridge 1 curr lim: ", "bridge 1 curr lim N/A");
  appendBoolOptional(getBridgeCurrentLimit(Bridge::c2), "bridge 2 curr lim: ", "bridge 2 curr lim N/A");
  for (uint32_t i = 1u; i <= cChannelCount; ++i) {
    std::optional<ChannelDiagnostics> result = getChannelDiagnostics(i);
    if (result) {
      uint32_t value = static_cast<uint32_t>(*result);
      uint32_t compacted = (value | value >> 7u | value >> 14u) & 7u;
      mParent->mInterface << "channel " << i << " diag: " << cTextChannelDiagnostics[compacted] << '\n';
    }
    else {
      mParent->mInterface << "channel " << i << " diag: N/A\n";
    }
  }
  mParent->mInterface << "dis: EN6: " << cTextStatusLatch[static_cast<size_t>(getEn6disable())] << '\n';
  mParent->mInterface << "dis: Vdd OV latch: " << getVddOvDisableLatch() << '\n';
  mParent->mInterface << "dis: Vdd UV: " << cTextStatusLatch[static_cast<size_t>(getVddUvDisable())] << '\n';
  mParent->mInterface << "dis: pin DIS: " << cTextStatusLatch[static_cast<size_t>(getDeviceDis())] << '\n';
  mParent->mInterface << "dis: pin NDIS: " << cTextStatusLatch[static_cast<size_t>(getDeviceNdisOn())] << '\n';
  mParent->mInterface << "nDIS out latch: " << getDeviceNdisOutLatch() << '\n';
  appendBoolOptional(getCommCheckLatch(), "comm error: ", "comm error N/A");
  appendBoolOptional(getBistResult(), "BIST: ", "BIST N/A");
  appendBoolOptional(getHwscResult(), "HWSC: ", "HWSC N/A");
  mParent->mInterface << "Vdd OV: " << cTextStatusLatch[static_cast<size_t>(getVddOvComp())] << '\n';
  mParent->mInterface << "Vdd UV: " << cTextStatusLatch[static_cast<size_t>(getVddUvComp())] << '\n';
  mParent->mInterface << "POR latch: " << getPowerOnResetLatch() << '\n';
  mParent->mInterface << "nRES latch: " << getNresLatch() << '\n';
  mParent->mInterface << "CP UV: " << cTextStatusLatch[static_cast<size_t>(getVcpUv())] << '\n';
  mParent->mInterface << "Vps UV: " << cTextStatusLatch[static_cast<size_t>(getVpsUv())] << '\n';
  mParent->mInterface << "FET status: " << getExternalFetOnStatus() << '\n';
  mParent->mInterface << "FET cmd: " << getExternalFetCommand() << '\n';
  for (uint32_t i = 1u; i <= cChannelCount; ++i) {
    mParent->mInterface << "channel " << i << " curr src: " << cTextCurrentSource[static_cast<size_t>(getCurrentSourceStatus(i))] << '\n';
  }
  mParent->mInterface << "nDIS protect latch: " << getNdisProtectLatch() << '\n';
  mParent->mInterface << "overtemp latch: " << getOverTempState() << '\n';
  mParent->mInterface << "SPI SDO OV latch: " << getSdoOvLatch() << '\n';
  mParent->mInterface << "temperature: " << getTemperature() << '\n';
  mParent->mInterface << "Vps: " << getBatteryVoltage() << '\n';
  mParent->mInterface.close();
}

template<typename tInterface>
void L9945<tInterface>::DiagnosticsResult::appendBoolOptional(std::optional<bool> const aResult, char const * const aPresent, char const* const aMissing) noexcept {
  if (aResult) {
    mParent->mInterface << aPresent << *aResult;
  }
  else {
    mParent->mInterface << aMissing;
  }
  mParent->mInterface << '\n';
}

template<typename tInterface>
void L9945<tInterface>::DiagnosticsResult::perform(DiagnosticsTest const aTest) {
  mTestPerformed = aTest;
  uint32_t willTest = 0u;
  if (aTest == DiagnosticsTest::cAuto) {
    mParent->readAllIntoCache();
    willTest = ((mParent->mReadCache[cCommand0] & cMask0enableDiagnostics) > 0u ? 0xffu : 0u);
    willTest &= ~(mParent->mReadCache[cCommand0] >> l9945::getRightmost1position(cMask0protectionDisable81));
  }
  else if (aTest == DiagnosticsTest::cOffPulse) {
    mParent->readAllIntoCache();
    willTest = gatherChannels(ChannelOcBlankTime::c142us, true);
    if (willTest != 0u) {
      mParent->setWriteDelay(cWaitForTest[static_cast<size_t>(aTest)]);
      mParent->write(cCommand9, cFixedPatternValues[cCommand9] | willTest << l9945::getRightmost1position(cMask9diagOffPulse81));
      mParent->mWriteCache[cCommand9] = cFixedPatternValues[cCommand9];
    }
    else { // nothing to do
    }
  }
  else if (aTest == DiagnosticsTest::cOnPulse) {
    mParent->readAllIntoCache();
    willTest = gatherChannels(ChannelOcBlankTime::c97us, false);
    if (willTest != 0u) {
      mParent->setWriteDelay(cWaitForTest[static_cast<size_t>(aTest)]);
      mParent->write(cCommand9, cFixedPatternValues[cCommand9] | willTest << l9945::getRightmost1position(cMask9diagOnPulse81));
      mParent->mWriteCache[cCommand9] = cFixedPatternValues[cCommand9];
    }
    else { // nothing to do
    }
  } 
  else if (aTest == DiagnosticsTest::cBist) {
    mParent->setWriteDelay(cWaitForTest[static_cast<size_t>(aTest)]);
    mParent->writeBistHwscRequest(RequestBist::cYes);
    mParent->readAllIntoCache();
  }
  else {
    mParent->readAllIntoCache(); // no separate tests, but read current statuses and latches
  }
  mReadCache = mParent->mReadCache;
  for (uint32_t i = 0; i < cChannelCount; ++i) {
    mChannelsDiagnosed[i] = (willTest & 1u << i) > 0u;
  }
}

template<typename tInterface>
uint32_t L9945<tInterface>::DiagnosticsResult::gatherChannels(ChannelOcBlankTime const aTimeLimit, bool const aFetNow) noexcept {
  uint32_t willTest = (mParent->getBridgeConfig(Bridge::c1) ? 0u : 0x0fu);
  willTest |= (mParent->getBridgeConfig(Bridge::c2) ? 0u : 0xf0u);
  willTest &= ((mParent->mReadCache[cCommand0] & cMask0enableDiagnostics) > 0u ? 0xffu : 0u);
  willTest &= mParent->mReadCache[cCommand0] >> l9945::getRightmost1position(cMask0spiInputSelect81);
  willTest &= ~(mParent->mReadCache[cCommand0] >> l9945::getRightmost1position(cMask0protectionDisable81));
  for (uint32_t i = 0; i < cChannelCount; ++i) {
    bool ok = (mParent->getOcBlankTime(i + 1u) < aTimeLimit);
    ok = ok && mParent->getSpiOnOut(i + 1u) == aFetNow;
    willTest &= (ok ? 1u : 0u) << i;
  }
  return willTest & 0xff;
}

}

#endif
