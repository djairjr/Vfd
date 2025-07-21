/*'''''''''''''''/*
   Interface completa DMX -> MODBUS
   -----------------------------------
   Def.: Permite o controle de um Motor trifásico WEG W22,
   utilizando um inversor de fase Schneider ATV12 com interface
   MODBUS feita por um Arduino Mega, com dois módulos RS485
*/
#include <esp_dmx.h>
#include <Preferences.h>
#include <ModbusMaster.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <vfd.h> // Biblioteca customizada com os VFDs de vários fabricantes

TFT_eSPI tft = TFT_eSPI();
#define TFT_BL 4

/*=============================  DMX  =============================*/
dmx_port_t dmxPort = DMX_NUM_1;
#define DMX_RX1_PIN 21
#define DMX_TX1_PIN 22
#define DMX_FIRST_CHANNEL 1
#define DMX_TOTAL_CHANNELS 4
#define DMX_MAX_CHANNEL (512 - DMX_TOTAL_CHANNELS)

int16_t dmxBaseChannel = DMX_FIRST_CHANNEL;
uint8_t dmxEnable   = 0;
uint8_t dmxDirection = 0;
uint8_t dmxSpeed    = 0;
uint8_t dmxTenths   = 0;

uint8_t lastDmxEnable    = 0;
uint8_t lastDmxDirection = 0;
uint8_t lastDmxSpeed     = 0;
uint8_t lastDmxTenths    = 0;

bool dmxOnline = false;
uint8_t dmxData[DMX_PACKET_SIZE];

/*=============================  MODBUS  =============================*/
#define MODBUS_RESPONSE_TIMEOUT 2000   // ms
#define STATE_TRANSITION_DELAY  500    // ms
#define MODBUS_TX 15
#define MODBUS_RX 17
ModbusMaster node;

/*=============================  ENCODER  =============================*/
#define DT_ENCODER 25
#define CLK_ENCODER 26
#define SW_ENCODER 27
#define ENCODER_DEBOUNCE_MS 5
ESP32Encoder encoder;
volatile bool encoderButtonFlag = false;

/*=============================  VFD  =============================*/
// Escolha apenas UM fabricante abaixo:
#define SCHNEIDER_ATV12 // Default
// #define SIEMENS_V20
// #define WEG_CFW500
// #define ABB_ACS550
// #define DANFOSS_FC302
// #define YASKAWA_V1000

#ifdef SCHNEIDER_ATV12
SchneiderATV12 vfd(&node, 1); // É possível usar um sistema por endereço: (ModbusMaster* node, uint8_t address)
#elif defined(SIEMENS_V20)
SiemensV20 vfd(&node, 1);
#elif defined(WEG_CFW500)
WEGCFW500 vfd(&node, 1);
#elif defined(ABB_ACS550)
ABBACS550 vfd(&node, 1);
#elif defined(DANFOSS_FC302)
DanfossFC302 vfd(&node, 1);
#elif defined(YASKAWA_V1000)
YaskawaV1000 vfd(&node, 1);
#else
#error "Defina um fabricante válido."
#endif
/*==============================   VFD =================================*/
uint16_t getMotorState() {
  return vfd.getMotorStatus() & 0x004F;
}


/*=============================  CONFIG  =============================*/
enum ConfigStep : uint8_t {
  IDLE = 0,
  SELECT_INVERTER = 1,
  SELECT_CHANNEL = 2,
  SAVE_AND_REBOOT = 3
};

enum InverterBrand : uint8_t {
  INV_SCHNEIDER_ATV12 = 0,
  INV_SIEMENS_V20,
  INV_WEG_CFW500,
  INV_ABB_ACS550,
  INV_DANFOSS_FC302,
  INV_YASKAWA_V1000,
  INV_COUNT
};

const char* brandNames[INV_COUNT] = {
  "Schneider ATV12",
  "Siemens V20",
  "WEG CFW500",
  "ABB ACS550",
  "Danfoss FC302",
  "Yaskawa V1000"
};

InverterBrand inverterBrand = INV_SCHNEIDER_ATV12;
ConfigStep configStep = IDLE;

/*=============================  EEPROM  =============================*/
Preferences prefs;

/*=============================  PROTÓTIPOS  =============================*/
void setup();
void loop();
void readDMX();
bool dmxChanged();
void processDMXCommands();
void checkEncoder();
void IRAM_ATTR onEncoderButton();
bool readEncoderButton();
void drawInverterScreen();
void drawChannelScreen();
void enterConfigMode();
void exitConfigMode();
void updateDisplayData();

/*======================================================================*/
/*                                SETUP                                 */
/*======================================================================*/
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  pinMode(SW_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SW_ENCODER), onEncoderButton, FALLING);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachSingleEdge(CLK_ENCODER, DT_ENCODER);
  encoder.clearCount();

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // Título principal "DMX MODBUS" em destaque
  tft.setTextFont(4);           // Fonte maior para destaque
  tft.setTextColor(TFT_CYAN);
  tft.setTextDatum(MC_DATUM);   // Centraliza o texto no meio da tela
  tft.drawString("DMX MODBUS", tft.width() / 2, 50);

  tft.setTextFont(2);           // Fonte menor para o subtítulo
  tft.setTextColor(TFT_WHITE);
  tft.drawString("@nicolaudosbrinquedos", tft.width() / 2, 70);
  delay(500);

  // EEPROM
  prefs.begin("dmx-config", false);
  inverterBrand = static_cast<InverterBrand>(prefs.getUChar("inverter", INV_SCHNEIDER_ATV12));
  dmxBaseChannel = prefs.getInt("baseChannel", DMX_FIRST_CHANNEL);
  prefs.end();
  dmxBaseChannel = constrain(dmxBaseChannel, DMX_FIRST_CHANNEL, DMX_MAX_CHANNEL);

  // DMX
  dmx_config_t cfg = DMX_CONFIG_DEFAULT;
  dmx_driver_install(dmxPort, &cfg, NULL, 0);
  dmx_set_pin(dmxPort, DMX_TX1_PIN, DMX_RX1_PIN, DMX_PIN_NO_CHANGE);
  readDMX(); // Added to update DMX values at boot

  // MODBUS
  Serial2.begin(19200, SERIAL_8E1, MODBUS_TX, MODBUS_RX);
  node.begin(1, Serial2);

  // VFD
  vfd.readFrequencyLimits();
  vfd.readMotorStatus();
  updateDisplayData();
}

/*======================================================================*/
/*                                LOOP                                  */
/*======================================================================*/
void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  checkEncoder();

  if (now - lastUpdate >= 20) {
    if (configStep == IDLE && readEncoderButton()) {
      configStep = SELECT_INVERTER;
      encoder.clearCount();
      drawInverterScreen();
      lastUpdate = now;
      return;
    }

    if (configStep == SELECT_INVERTER && readEncoderButton()) {
      configStep = SELECT_CHANNEL;
      encoder.clearCount();
      encoder.setCount(dmxBaseChannel);
      drawChannelScreen();
      lastUpdate = now;
      return;
    }

    if (configStep == SELECT_CHANNEL && readEncoderButton()) {
      prefs.begin("dmx-config", false);
      prefs.putUChar("inverter", inverterBrand);
      prefs.putInt("baseChannel", dmxBaseChannel);
      prefs.end();
      ESP.restart();
      lastUpdate = now;
      return;
    }

    if (configStep == IDLE) {
      readDMX();
      if (dmxChanged()) {
        lastDmxEnable    = dmxEnable;
        lastDmxDirection = dmxDirection;
        lastDmxSpeed     = dmxSpeed;
        lastDmxTenths    = dmxTenths;
        processDMXCommands();
        vfd.readMotorStatus();
        updateDisplayData();
      }
    }
    
    
    lastUpdate = now;
  }
}

/*======================================================================*/
/*                           FUNÇÕES                                    */
/*======================================================================*/
void readDMX() {
  dmx_packet_t packet;
  if (dmx_receive(dmxPort, &packet, 0)) {
    dmxOnline = (packet.err == DMX_OK);
    if (dmxOnline) {
      dmx_read(dmxPort, dmxData, packet.size);
      int idx = dmxBaseChannel;
      if (packet.size >= (idx + DMX_TOTAL_CHANNELS)) {
        dmxEnable   = dmxData[idx];
        dmxDirection = dmxData[idx + 1];
        dmxSpeed    = dmxData[idx + 2];
        dmxTenths   = dmxData[idx + 3];
      } else {
        dmxEnable = dmxDirection = dmxSpeed = dmxTenths = 0;
      }
    }
  }
}

bool dmxChanged() {
  return lastDmxEnable   != dmxEnable ||
         lastDmxDirection != dmxDirection ||
         lastDmxSpeed    != dmxSpeed ||
         lastDmxTenths   != dmxTenths;
}

void processDMXCommands() {
  bool enableMotor = (dmxEnable >= 128);
  uint16_t targetSpeed = 0;

  if (enableMotor) {
    static uint8_t lastSpeed = 0, lastTenths = 0;
    uint8_t speedVal = (dmxSpeed * 61) / 256;
    if (abs(speedVal - lastSpeed) >= 1 || dmxSpeed == 0 || dmxSpeed == 255) lastSpeed = speedVal;
    uint8_t tenthsVal = (dmxTenths * 10) / 256;
    if (abs(tenthsVal - lastTenths) >= 1 || dmxTenths == 0 || dmxTenths == 255) lastTenths = tenthsVal;

    targetSpeed = constrain(lastSpeed, 0, 60) * 10 + constrain(lastTenths, 0, 9);
    targetSpeed = constrain(targetSpeed, vfd.getMinFrequency(), vfd.getMaxFrequency());
  }

  bool newDir = (dmxDirection >= 128);
  bool dirChanged = newDir != (vfd.getMotorDirection() == 1);

  uint16_t st = vfd.getMotorStatus() & 0x004F;

  if (st == STATE_FAULT) {
    vfd.sendCommandWithRetry(CMD_RESET_FAULT);
    delay(1000);
    vfd.readMotorStatus();
    return;
  }

  vfd.setMotorSpeed(targetSpeed);

  if (!enableMotor) {
    switch (st) {
      case STATE_OPERATION_ENABLED:
        vfd.sendCommandWithRetry(CMD_DISABLE_OPERATION);
        delay(STATE_TRANSITION_DELAY);
        vfd.sendCommandWithRetry(CMD_SHUTDOWN);
        break;
      case STATE_SWITCHED_ON:
        vfd.sendCommandWithRetry(CMD_SHUTDOWN);
        break;
    }
    return;
  }

  if (dirChanged && st == STATE_OPERATION_ENABLED) {
    vfd.sendCommandWithRetry(CMD_DISABLE_OPERATION);
    delay(500);
  }

  vfd.setMotorDirection(newDir);

  switch (st) {
    case STATE_SWITCH_DISABLED:
      vfd.sendCommandWithRetry(CMD_SHUTDOWN);
      delay(STATE_TRANSITION_DELAY);
      vfd.sendCommandWithRetry(CMD_SWITCH_ON);
      delay(STATE_TRANSITION_DELAY);
      vfd.sendCommandWithRetry(newDir ? CMD_RUN_REVERSE : CMD_RUN_FORWARD);
      break;
    case STATE_READY:
      vfd.sendCommandWithRetry(CMD_SWITCH_ON);
      delay(STATE_TRANSITION_DELAY);
      vfd.sendCommandWithRetry(newDir ? CMD_RUN_REVERSE : CMD_RUN_FORWARD);
      break;
    case STATE_SWITCHED_ON:
      vfd.sendCommandWithRetry(newDir ? CMD_RUN_REVERSE : CMD_RUN_FORWARD);
      break;
    case STATE_OPERATION_ENABLED:
      if (dirChanged) vfd.sendCommandWithRetry(newDir ? CMD_RUN_REVERSE : CMD_RUN_FORWARD);
      break;
  }
}

/*======================================================================*/
/*                           ENCODER / CONFIG                           */
/*======================================================================*/
void IRAM_ATTR onEncoderButton() {
  static unsigned long last = 0;
  if (millis() - last > 100) {
    encoderButtonFlag = true;
    last = millis();
  }
}

bool readEncoderButton() {
  bool ret = encoderButtonFlag;
  encoderButtonFlag = false;
  return ret;
}

void checkEncoder() {
  static long lastPos = 0;
  static unsigned long lastChange = 0;

  long newPos = encoder.getCount();
  if (newPos != lastPos && millis() - lastChange > ENCODER_DEBOUNCE_MS) {
    long diff = newPos - lastPos;
    int inc = (diff > 0) ? 1 : -1;

    if (configStep == SELECT_INVERTER) {
      inverterBrand = static_cast<InverterBrand>((inverterBrand + INV_COUNT + inc) % INV_COUNT);
      drawInverterScreen();
    } else if (configStep == SELECT_CHANNEL) {
      dmxBaseChannel = constrain(dmxBaseChannel + inc, DMX_FIRST_CHANNEL, DMX_MAX_CHANNEL);
      drawChannelScreen();
    }

    lastPos = newPos;
    lastChange = millis();
  }
}

void drawInverterScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextFont(2);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Choose Inverter", tft.width() / 2, 15);
  tft.setTextFont(4);
  tft.drawString(brandNames[inverterBrand], tft.width() / 2, 75);
  tft.setTextFont(1);
  tft.setTextColor(TFT_YELLOW);
  tft.drawString("Press to Confirm", tft.width() / 2, 130);
}

void drawChannelScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextFont(2);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("DMX Channel", tft.width() / 2, 15);
  tft.setTextFont(7);
  tft.setTextColor(TFT_CYAN);
  tft.drawNumber(dmxBaseChannel, tft.width() / 2, 75);
  tft.setTextFont(1);
  tft.setTextColor(TFT_YELLOW);
  tft.drawString("Press to Save & Reboot", tft.width() / 2, 130);
}

/*======================================================================*/
/*                             DISPLAY                                  */
/*======================================================================*/
void updateDisplayData() {
  if (configStep != IDLE) return;

  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);

  int xLabel = 5, xValue = 120, xBar = 170;
  int barW = 60, barH = 10, yStart = 10, lineH = 15;

  const char* labels[4] = {"STATE", "DIR", "SPEED", ".DEC"};
  uint8_t vals[4] = {dmxEnable, dmxDirection, dmxSpeed, dmxTenths};
  uint16_t colors[4] = {TFT_CYAN, TFT_MAGENTA, TFT_ORANGE, TFT_YELLOW};

  for (int i = 0; i < 4; ++i) {
    int y = yStart + i * lineH;
    tft.setCursor(xLabel, y);
    tft.setTextColor(colors[i], TFT_BLACK);
    tft.printf("DMX %d > %s >", dmxBaseChannel + i, labels[i]);
    tft.setCursor(xValue, y);
    tft.printf("%3d", vals[i]);
    int len = map(vals[i], 0, 255, 0, barW);
    tft.fillRect(xBar, y + 2, len, barH, colors[i]);
    tft.drawRect(xBar, y + 2, barW, barH, TFT_DARKGREY);
  }

  int modY = yStart + 4 * lineH + 10;
  float dsp = vfd.getMotorSpeed() / 10.0f;
  if (vfd.getMotorDirection() == 1 && vfd.getMotorSpeed() > 32767)
    dsp = ((int16_t)(65535 - vfd.getMotorSpeed() + 1)) / -10.0f;

  tft.setCursor(xLabel, modY);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Motor: ");
  bool running = (vfd.getMotorStatus() & 0x004F) == STATE_OPERATION_ENABLED;
  tft.setTextColor(running ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.println(running ? "ON" : "OFF");

  tft.setCursor(xValue, modY);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.printf("Speed: %.1f Hz", abs(dsp));

  int r = 4, ySt = modY + 25;
  tft.setCursor(xLabel, ySt - 8);
  tft.print("DMX");
  tft.fillCircle(xLabel + 72, ySt - 2, r, dmxOnline ? TFT_GREEN : TFT_RED);

  tft.setCursor(xValue, ySt - 8);
  tft.print("MODBUS");
  tft.fillCircle(xValue + 72, ySt - 2, r, vfd.getMotorStatus() != 0 ? TFT_GREEN : TFT_RED);

  // Inverter status
  int inverterY = ySt + 5;  // 20px abaixo dos status de DMX e MODBUS
  tft.setCursor(xLabel, inverterY);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Inverter Model:");
  tft.setCursor(xLabel + 115, inverterY);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.print(brandNames[inverterBrand]);
}
// =============================================
// FUNÇÕES CONFIGURAÇÕES SALVAS E LIDAS NA EEPROM
// =============================================

void loadEEPROMConfig() {
  prefs.begin("dmx-config", false);
  inverterBrand  = static_cast<InverterBrand>(prefs.getUChar("inverter", INV_SCHNEIDER_ATV12));
  dmxBaseChannel = prefs.getInt("baseChannel", DMX_FIRST_CHANNEL);
  prefs.end();
}

void saveEEPROMConfig() {
  prefs.begin("dmx-config", false);
  prefs.putUChar("inverter", inverterBrand);
  prefs.putInt("baseChannel", dmxBaseChannel);
  prefs.end();
}
