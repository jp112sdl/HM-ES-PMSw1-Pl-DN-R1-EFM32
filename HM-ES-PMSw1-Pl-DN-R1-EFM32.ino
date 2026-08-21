//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2017-10-24 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2018-09-29 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// ci-test=yes board=644p aes=no

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER
#define NDEBUG
#define USE_HW_SERIAL
#define HIDE_IGNORE_MSG

#include <SPI.h>
#include <AskSinPP.h>
#include <Switch.h>
#include <CS5490.h>

#define CS5490_MEASURE_INTERVAL         1
#define POWERMETER_CYCLIC_INTERVAL    120

#define CONFIG_BUTTON_PIN     PB8
#define TRX_CS                PC14
#define TRX_GDO0              PC15
#define LED1_PIN              PA1 //GRN
#define LED2_PIN              PA0 //ROT
#define CS5490_RST            PB11
#define CS5490_DO             PA2
#define R7                    0.75
#define R4                    390.0
#define R5                    390.0
#define R6                    390.0
#define V_FS_RMS_V            0.17678
#define V_FS                  0.6
#define V_GAIN_S              0.9003636
#define I_GAIN_S              0.3808553
#define MULTIPLIER            414.09759786948


#define RELAY_PIN             PB7

// number of available peers per channel
#define PEERS_PER_SWCHANNEL     8
#define PEERS_PER_PMCHANNEL     8
#define PEERS_PER_SENSORCHANNEL 8

// all library classes are placed in the namespace 'as'
using namespace as;

uint32_t floatToUint32_reinterpret(float val) {
  return *reinterpret_cast<uint32_t*>(&val);
}

typedef struct {
  uint32_t E_Counter = 0;
  uint32_t Power     = 0;
  uint16_t Current   = 0;
  uint16_t Voltage   = 0;
  uint8_t  Frequency = 0;
} energyMeterValues;

energyMeterValues actualValues ;
energyMeterValues lastValues;
uint8_t averaging = 1;
bool    resetAverageCounting = false;
// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0x00, 0xac, 0x20},     // Device ID
  "HMESPMSw20",           // Device Serial
  {0x00, 0xac},           // Device Model
  0x26,                   // Firmware Version
  as::DeviceType::Switch, // Device Type
  {0x01, 0x00}            // Info Bytes
};

// Configure the used hardware
typedef LibSPI<TRX_CS> SPIType;
typedef CC1101Radio<SPIType, TRX_GDO0> RadioType;
typedef StatusLed<LED1_PIN> LedType;
typedef AskSin<LedType, NoBattery, RadioType> Hal;
typedef StatusLed<LED2_PIN> RedLedType;
CS5490 cs5490(Serial, CS5490_RST);


DEFREGISTER(Reg0, MASTERID_REGS, DREG_INTKEY, DREG_CONFBUTTONTIME, DREG_LOCALRESETDISABLE)
class PMSw1List0 : public RegList0<Reg0> {
  public:
    PMSw1List0(uint16_t addr) : RegList0<Reg0>(addr) {}
    void defaults () {
      clear();
    }
};

bool relayOn() {
  return (digitalRead(RELAY_PIN) == HIGH);
}
//typedef SwitchChannel<Hal, PEERS_PER_SWCHANNEL, PMSw1List0> SwChannel;
class SwChannel : public SwitchChannel<Hal, PEERS_PER_SWCHANNEL, PMSw1List0>  {

  protected:
    typedef SwitchChannel<Hal, PEERS_PER_SWCHANNEL, PMSw1List0> BaseChannel;
    RedLedType RedLed;

  public:
    SwChannel () : BaseChannel() {}
    virtual ~SwChannel() {}

    void init (uint8_t p) {
      RedLed.init();
      BaseChannel::init(p);
    }

    virtual void switchState(__attribute__((unused)) uint8_t oldstate, uint8_t newstate,uint32_t delay) {
      BaseChannel::switchState(oldstate, newstate, delay);
      if ( newstate == AS_CM_JT_ON ) {
        resetAverageCounting = true;
        RedLed.ledOn();
      }
      else if ( newstate == AS_CM_JT_OFF ) {
        RedLed.ledOff();
      }
    }
};


DEFREGISTER(MReg1, CREG_AES_ACTIVE, CREG_AVERAGING, CREG_TX_MINDELAY, CREG_TX_THRESHOLD_POWER, CREG_TX_THRESHOLD_CURRENT, CREG_TX_THRESHOLD_VOLTAGE, CREG_TX_THRESHOLD_FREQUENCY)
class MeasureList1 : public RegList1<MReg1> {
  public:
    MeasureList1 (uint16_t addr) : RegList1<MReg1>(addr) {}
    void defaults () {
      clear();
      txMindelay(8);
      txThresholdPower(10000);
      txThresholdCurrent(100);
      txThresholdVoltage(100);
      txThresholdFrequency(100);
      averaging(1);
    }
};

DEFREGISTER(SensorReg1, CREG_AES_ACTIVE, CREG_LEDONTIME, CREG_TRANSMITTRYMAX, CREG_COND_TX_THRESHOLD_HI, CREG_COND_TX_THRESHOLD_LO, CREG_COND_TX, CREG_COND_TX_DECISION_ABOVE, CREG_COND_TX_DECISION_BELOW)
class SensorList1 : public RegList1<SensorReg1> {
  public:
    SensorList1 (uint16_t addr) : RegList1<SensorReg1>(addr) {}
    void defaults () {
      clear();
      transmitTryMax(6);
      condTxDecisionAbove(200);
      condTxDecisionBelow(0);
      condTxFalling(false);
      condTxRising(false);
      condTxCyclicBelow(false);
      condTxCyclicAbove(false);
      condTxThresholdHi(0);
      condTxThresholdLo(0);
    }
};

class PowerEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, uint8_t typ, bool boot, uint32_t e_counter, uint32_t power, uint16_t current, uint16_t voltage, uint8_t frequency ) {

      uint8_t ec1 = (e_counter >> 16) & 0x7f;
      if (boot == true) {
        ec1 |= 0x80;
      }

      Message::init(0x16, msgcnt, typ, (typ == AS_MESSAGE_POWER_EVENT_CYCLIC) ? BCAST : BIDI | BCAST, ec1, (e_counter >> 8) & 0xff);
      pload[0] = (e_counter) & 0xff;
      pload[1] = (power >> 16) & 0xff;
      pload[2] = (power >> 8) & 0xff;
      pload[3] = (power) & 0xff;
      pload[4] = (current >> 8) & 0xff;
      pload[5] = (current) & 0xff;
      pload[6] = (voltage >> 8) & 0xff;
      pload[7] = (voltage) & 0xff;
      pload[8] = (frequency) & 0xff;
    }
};

class PowerMeterChannel : public Channel<Hal, MeasureList1, EmptyList, List4, PEERS_PER_PMCHANNEL, PMSw1List0>, public Alarm {
    PowerEventMsg msg;
    bool boot;
    uint32_t txThresholdPower;
    uint16_t txThresholdCurrent;
    uint16_t txThresholdVoltage;
    uint8_t txThresholdFrequency;
    uint8_t txMindelay;
  public:
    PowerMeterChannel () : Channel(), Alarm(0), boot(false), txThresholdPower(0), txThresholdCurrent(0), txThresholdVoltage(0), txThresholdFrequency(0), txMindelay(8) {}
    virtual ~PowerMeterChannel () {}
    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      static uint8_t tickCount = 0;
      tick = seconds2ticks(delay());
      tickCount++;

      uint8_t msgType = 0;

      if (tickCount > (POWERMETER_CYCLIC_INTERVAL / delay()) || boot == false)  {
        msgType = AS_MESSAGE_POWER_EVENT_CYCLIC;
      }

      if ((txThresholdCurrent   > 0) && (abs((int)(actualValues.Current   - lastValues.Current)  ) >= (int)txThresholdCurrent))   msgType = AS_MESSAGE_POWER_EVENT;
      if ((txThresholdFrequency > 0) && (abs((int)(actualValues.Frequency - lastValues.Frequency)) >= (int)txThresholdFrequency)) msgType = AS_MESSAGE_POWER_EVENT;
      if ((txThresholdPower     > 0) && (abs((int)(actualValues.Power     - lastValues.Power)    ) >= (int)txThresholdPower))     msgType = AS_MESSAGE_POWER_EVENT;
      if ((txThresholdVoltage   > 0) && (abs((int)(actualValues.Voltage   - lastValues.Voltage)  ) >= (int)txThresholdVoltage))   msgType = AS_MESSAGE_POWER_EVENT;

      if ((msgType != AS_MESSAGE_POWER_EVENT) && (actualValues.Voltage > 0) && (lastValues.Voltage == 0)) msgType = AS_MESSAGE_POWER_EVENT;

      msg.init(device().nextcount(), msgType, boot, actualValues.E_Counter, actualValues.Power, actualValues.Current, actualValues.Voltage, actualValues.Frequency);
      switch (msgType) {
        case AS_MESSAGE_POWER_EVENT_CYCLIC:
          DPRINTLN(F("PowerMeterChannel - SENDING CYCLIC MESSAGE"));
          tickCount = 0;
          device().broadcastEvent(msg);
        break;
        case AS_MESSAGE_POWER_EVENT:
          if (device().getMasterID() > 0) {
            DPRINTLN(F("PowerMeterChannel - SENDING EVENT MESSAGE"));
            device().sendMasterEvent(msg);
          }
        break;
      }

      lastValues.Current = actualValues.Current;
      lastValues.Frequency = actualValues.Frequency;
      lastValues.Power = actualValues.Power;
      lastValues.Voltage = actualValues.Voltage;
      lastValues.E_Counter = actualValues.E_Counter;

      boot = true;
      sysclock.add(*this);
    }

    uint32_t delay() {
      return max(1, txMindelay);
    }

    void configChanged() {
      DPRINTLN(F("PowerMeterChannel Config changed List1"));
      txThresholdPower     = this->getList1().txThresholdPower();        // 1.00 W = 100
      txThresholdCurrent   = this->getList1().txThresholdCurrent();      // 1 mA   = 1
      txThresholdVoltage   = this->getList1().txThresholdVoltage();      // 10.0V  = 100
      txThresholdFrequency = this->getList1().txThresholdFrequency();    // 1 Hz   = 100
      txMindelay           = this->getList1().txMindelay();
      averaging            = this->getList1().averaging();
      DPRINT(F("txMindelay           = ")); DDECLN(txMindelay);
      DPRINT(F("txThresholdPower     = ")); DDECLN(txThresholdPower);
      DPRINT(F("txThresholdCurrent   = ")); DDECLN(txThresholdCurrent);
      DPRINT(F("txThresholdVoltage   = ")); DDECLN(txThresholdVoltage);
      //DPRINT(F("txThresholdFrequency = ")); DDECLN(txThresholdFrequency);
    }

    void setup(Device<Hal, PMSw1List0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      sysclock.add(*this);
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      return 0;
    }
};

class SensorChannel : public Channel<Hal, SensorList1, EmptyList, List4, PEERS_PER_SENSORCHANNEL, PMSw1List0>, public Alarm {
  public:

    SensorChannel () : Channel(), Alarm(0) {}
    virtual ~SensorChannel () {}

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      static uint8_t evcnt = 0;
      bool sendMsg = false;
      tick = seconds2ticks(10);

      if (relayOn() == true) {
        SensorEventMsg& rmsg = (SensorEventMsg&)device().message();
        static uint8_t aboveMsgSent = false;
        static uint8_t belowMsgSent = false;
        switch (number()) {
          case 3:
            // Leistungssensor
            if (this->getList1().condTxRising() == true)  {
              if (actualValues.Power > this->getList1().condTxThresholdHi()) {
                if ((aboveMsgSent == false && this->getList1().condTxCyclicAbove() == false) || this->getList1().condTxCyclicAbove() == true) {
                  rmsg.init(device().nextcount(), number(), evcnt++, this->getList1().condTxDecisionAbove(), false , false);
                  sendMsg = true;
                  aboveMsgSent = true;
                }
              } else {
                aboveMsgSent = false;
              }
            }
            if (this->getList1().condTxFalling() == true) {
              if (actualValues.Power < this->getList1().condTxThresholdLo()) {
                if ((belowMsgSent == false && this->getList1().condTxCyclicBelow() == false) || this->getList1().condTxCyclicBelow() == true) {
                  rmsg.init(device().nextcount(), number(), evcnt++, this->getList1().condTxDecisionBelow(), false , false);
                  sendMsg = true;
                  belowMsgSent = true;
                }
              } else {
                belowMsgSent = false;
              }
            }
            break;
          case 4:
            // Stromsensor
            if (this->getList1().condTxRising() == true)  {
              if (actualValues.Current > this->getList1().condTxThresholdHi()) {
                if ((aboveMsgSent == false && this->getList1().condTxCyclicAbove() == false) || this->getList1().condTxCyclicAbove() == true) {
                  rmsg.init(device().nextcount(), number(), evcnt++, this->getList1().condTxDecisionAbove(), false , false);
                  sendMsg = true;
                  aboveMsgSent = true;
                }
              } else {
                aboveMsgSent = false;
              }
            }
            if (this->getList1().condTxFalling() == true) {
              if (actualValues.Current < this->getList1().condTxThresholdLo()) {
                if ((belowMsgSent == false && this->getList1().condTxCyclicBelow() == false) || this->getList1().condTxCyclicBelow() == true) {
                  rmsg.init(device().nextcount(), number(), evcnt++, this->getList1().condTxDecisionBelow(), false , false);
                  sendMsg = true;
                  belowMsgSent = true;
                }
              } else {
                belowMsgSent = false;
              }
            }
            break;
          case 5:
            // Spannungssensor
            if (this->getList1().condTxRising() == true)  {
              if (actualValues.Voltage > this->getList1().condTxThresholdHi()) {
                if ((aboveMsgSent == false && this->getList1().condTxCyclicAbove() == false) || this->getList1().condTxCyclicAbove() == true) {
                  rmsg.init(device().nextcount(), number(), evcnt++, this->getList1().condTxDecisionAbove(), false , false);
                  sendMsg = true;
                  aboveMsgSent = true;
                }
              } else {
                aboveMsgSent = false;
              }
            }
            if (this->getList1().condTxFalling() == true) {
              if (actualValues.Voltage < this->getList1().condTxThresholdLo()) {
                if ((belowMsgSent == false && this->getList1().condTxCyclicBelow() == false) || this->getList1().condTxCyclicBelow() == true) {
                  rmsg.init(device().nextcount(), number(), evcnt++, this->getList1().condTxDecisionBelow(), false , false);
                  sendMsg = true;
                  belowMsgSent = true;
                }
              } else {
                belowMsgSent = false;
              }
            }
            break;
          case 6:
            // Frequenzsensor
            break;
        }

        if (sendMsg) {
          device().sendPeerEvent(rmsg, *this);
          sendMsg = false;
        }
      }
      sysclock.add(*this);
    }

    void configChanged() {
      DPRINT(F("SensorChannel ")); DDEC(number()); DPRINTLN(F(" Config changed List1"));

      //DPRINT(F("transmitTryMax      = ")); DDECLN(this->getList1().transmitTryMax());
      //DPRINT(F("condTxDecisionAbove = ")); DDECLN(this->getList1().condTxDecisionAbove());
      //DPRINT(F("condTxDecisionBelow = ")); DDECLN(this->getList1().condTxDecisionBelow());
      //DPRINT(F("condTxFalling       = ")); DDECLN(this->getList1().condTxFalling());
      //DPRINT(F("condTxRising        = ")); DDECLN(this->getList1().condTxRising());
      //DPRINT(F("condTxCyclicAbove   = ")); DDECLN(this->getList1().condTxCyclicAbove());
      //DPRINT(F("condTxCyclicBelow   = ")); DDECLN(this->getList1().condTxCyclicBelow());
      //DPRINT(F("condTxThresholdHi   = ")); DDECLN(this->getList1().condTxThresholdHi());
      //DPRINT(F("condTxThresholdLo   = ")); DDECLN(this->getList1().condTxThresholdLo());
    }

    void setup(Device<Hal, PMSw1List0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      sysclock.add(*this);
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      return 0;
    }
};

class MixDevice : public ChannelDevice<Hal, VirtBaseChannel<Hal, PMSw1List0>, 6, PMSw1List0> {
    class MeasureAlarm : public Alarm {
        MixDevice& dev;
      public:
        MeasureAlarm (MixDevice& d) : Alarm (0), dev(d) {}
        virtual ~MeasureAlarm () {}
        void trigger (AlarmClock& clock)  {
          set(seconds2ticks(CS5490_MEASURE_INTERVAL));
          static uint32_t Power      = 0;
          static uint32_t Current    = 0;
          static uint32_t Voltage    = 0;
          static uint32_t Frequency  = 0;
          static uint8_t  avgCounter = 0;


          if (resetAverageCounting == true) {
            //DPRINTLN(F("*** RESETTING AVERAGING ***"));
            resetAverageCounting = false;
            avgCounter = 0;
            Power = 0;
            Voltage = 0;
            Current = 0;
            Frequency = 0;
          }

          if (avgCounter < averaging) {

            float status0 = cs5490.readRegister(CS5490::STATUS0);
            if (status0 == 0xC00030) {
              double Vrms = cs5490.readRegister(CS5490::V_RMS);
              double V = Vrms * MULTIPLIER;//V_FS_RMS_V / V_FS *  V_GAIN_S / (R7 / ((R4+R5+R6)+R7) );
              Voltage += (double)V * 10UL;

              double Freq = cs5490.readRegister(CS5490::EPSILON);
              Frequency += Freq * 4000UL;

              double C = cs5490.readRegister(CS5490::I_RMS) * I_GAIN_S;
              Current += C * 10UL;

              double Pavg = cs5490.readRegister(CS5490::P_AVG);
              Power += Pavg * 100UL;


              avgCounter++;
            }

          } else {
            actualValues.Power   = relayOn() ? (Power / averaging) : 0;
            actualValues.Voltage = (Voltage / averaging);
            actualValues.Frequency = (Frequency / averaging);
            actualValues.Current = relayOn() ? (Current / averaging) : 0;
            resetAverageCounting = true;

          }

          /* actualValues.E_Counter = (hlw8012.getEnergy()  / 3600.0)   * 10; */

          clock.add(*this);
        }
    } energyMeterMeasure;
  public:
    VirtChannel<Hal, SwChannel,         PMSw1List0> c1;
    VirtChannel<Hal, PowerMeterChannel, PMSw1List0> c2;
    VirtChannel<Hal, SensorChannel,     PMSw1List0> c3, c4, c5, c6;
  public:
    typedef ChannelDevice<Hal, VirtBaseChannel<Hal, PMSw1List0>, 6, PMSw1List0> DeviceType;
    MixDevice (const DeviceInfo& info, uint16_t addr) : DeviceType(info, addr), energyMeterMeasure(*this) {
      DeviceType::registerChannel(c1, 1);
      DeviceType::registerChannel(c2, 2);
      DeviceType::registerChannel(c3, 3);
      DeviceType::registerChannel(c4, 4);
      DeviceType::registerChannel(c5, 5);
      DeviceType::registerChannel(c6, 6);
    }
    virtual ~MixDevice () {}

    SwChannel& switchChannel ()  {
      return c1;
    }
    PowerMeterChannel& powermeterChannel ()  {
      return c2;
    }
    SensorChannel& sensorChannel3Power ()  {
      return c3;
    }
    SensorChannel& sensorChannel4Current ()  {
      return c4;
    }
    SensorChannel& sensorChannel5Voltage ()  {
      return c5;
    }
    SensorChannel& sensorChannel6Frequency ()  {
      return c6;
    }

    MeasureAlarm&  em ()  {
      return energyMeterMeasure;
    }

    virtual void configChanged () {
      DPRINTLN(F("MixDevice Config changed List0"));
    }
};

Hal hal;
MixDevice sdev(devinfo, 0x20);
ConfigToggleButton<MixDevice> cfgBtn(sdev);

void initPeerings (bool first) {
  if( first == true ) {
    sdev.switchChannel().peer(cfgBtn.peer());
  }
}

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  bool first = sdev.init(hal);
  sdev.switchChannel().init(RELAY_PIN);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  initPeerings(first);
  sdev.initDone();
  DDEVINFO(sdev);

  cs5490.begin(115200,false);

  cs5490.writeRegister(CS5490::V_GAIN     , V_GAIN_S);
  cs5490.writeRegister(CS5490::I_GAIN     , I_GAIN_S);

  cs5490.writeRegister(CS5490::T_SETTLE   , 8000);
  cs5490.writeRegister(CS5490::SAMPLECOUNT, 400);

  cs5490.writeRegister(CS5490::CONFIG2    , 0x10020A);
  cs5490.writeRegister(CS5490::CONFIG0    , 0xC02020);
  cs5490.writeRegister(CS5490::CONFIG1    , 0x00EEEB);
  cs5490.writeRegister(CS5490::MASK       , 0x800010);
  cs5490.writeRegister(CS5490::STATUS0    , 0x800010);

  cs5490.sendInstruction(CS5490::CONT_CONV);

  sdev.em().set(seconds2ticks(CS5490_MEASURE_INTERVAL));
  sysclock.add(sdev.em());
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    // hal.activity.savePower<Idle<true> >(hal);
  }
}
