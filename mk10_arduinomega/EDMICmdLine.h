#include "config.h"
#include "Arduino.h"
// #include <cstddef>
// #include <cstdint>
// #include <map>
// #include <HardwareSerial.h>
// #include <pgmspace.h>

typedef uint8_t byte;

class EdmiCMDReader {
public:
  enum class Status : uint8_t {
    Connect,
    Disconnect,
    Ready,
    LoggedIn,
    NotLogin,
    Busy,
    Finish,
    TimeoutError,
    ProtocolError,
    ChecksumError
  };

// explicit EdmiCMDReader(HardwareSerial &serial) : serial_(serial);
// EdmiCMDReader(EdmiCMDReader const &) = delete;
// EdmiCMDReader(EdmiCMDReader &&) = delete;

// void begin(unsigned long baud);
#if defined(ESP32)
  /* ESP32 Hardware serial interface requires the receive and transmit pin specified */
  EdmiCMDReader(HardwareSerial &port, uint8_t receivePin, uint8_t transmitPin);

  // Deprecate passing pointer
  EdmiCMDReader(HardwareSerial *port, uint8_t receivePin, uint8_t transmitPin)
    : EdmiCMDReader(*port, receivePin, transmitPin){};
#else
  EdmiCMDReader(HardwareSerial &port);

  EdmiCMDReader(HardwareSerial *port)
    : EdmiCMDReader(*port){};
#endif
  // Empty constructor for creating arrays
  EdmiCMDReader(){};

  ~EdmiCMDReader();

  void keepAlive();
  void read_looping();

  void TX_raw(uint8_t d);
  void TX_cmd(uint8_t *cmd, unsigned short len);

  void send_cmdR(const byte *reg);

  void step_start();
  bool read_default();

  String serialNumber();
  float voltR();
  float voltS();
  float voltT();
  float currentR();
  float currentS();
  float currentT();
  float wattR();
  float wattS();
  float wattT();
  float pf();
  float frequency();
  float kVarh();
  float kwhWBP();
  float kwhLWBP();
  float kwhTotal();

  float read_voltR();
  float read_voltS();
  float read_voltT();
  float read_currentR();
  float read_currentS();
  float read_currentT();
  float read_wattR();
  float read_wattS();
  float read_wattT();
  float read_pf();
  float read_frequency();
  float read_kVarh();
  float read_kwhWBP();
  float read_kwhLWBP();
  float read_kwhTotal();

  String read_Serialnumber(/*char *output, int len*/);

  Status status() const {
    return status_;
  }

  void acknowledge() {
    Serial.println("ACKNOWLEDGE");
    if (status_ != Status::Busy and status_ != Status::Disconnect)
      status_ = Status::Ready;
  }

protected:
  Stream* serial_; // Serial interface
  enum class Step : uint8_t;
  enum class ErrorCode : uint8_t {
    NullError,
    CannotWrite,
    UnimplementedOperation,
    RegisterNotFound,
    AccessDenied,
    WrongLength,
    BadTypCode,
    DataNotReady,
    OutOfRange,
    NotLoggedIn,
  };

  struct
  {
    String serialNumber;
    float voltR;
    float voltS;
    float voltT;
    float currentR;
    float currentS;
    float currentT;
    float wattR;
    float wattS;
    float wattT;
    float pf;
    float frequency;
    float kVarh;
    float kwhWBP;
    float kwhLWBP;
    float kwhTotal;
  } _currentValues;  // Measured values

  void step_login();
  void step_logout();
  void step_read();
  void TX_byte(uint8_t d);
  bool RX_char(unsigned int timeout, byte *inChar);
  bool check_RXReg(uint8_t *buff, uint8_t typeReg, uint8_t compare);

  uint8_t RX_message(uint8_t *message, int maxMessageSize, unsigned int timeout);
  void change_status(Status to);
  void init(Stream* port); // Init common to all constructors


  Step step_;
  ErrorCode regError_;
  size_t errors_ = 0, checksum_errors_ = 0, successes_ = 0;
  Status status_ = Status::Ready;
  uint8_t rx_, tx_;
};