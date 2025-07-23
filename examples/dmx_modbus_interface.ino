/*
   Interface completa DMX -> MODBUS
   -----------------------------------
   Def.: Permite o controle de um Motor trifásico WEG W22,
   utilizando um inversor de fase Schneider ATV12 com interface
   MODBUS feita por um Arduino Mega, com dois módulos RS485

   Criado por Djair Guilherme em Junho de 2025 para o espetáculo
   Macuco, da Digna Companhia, que necessitava de um controle
   do dispositivo a partir de uma mesa de luz DMX convencional;

   PARÂMETROS DO MOTOR WEG W22 (220V)
   -----------------------------------
   Motor de 3KW(HP-cv) 0.25 (0.33)
   (nSP) 1720 RPM  - (FrS) 60Hz
   Tensão Nominal (Uns): 220V
   Corrente Nominal (nCr): 1.65A
   Corrente de sobrecarga (SFA): 1.9A
   Fator de Potência: (Ncr) 0.25kW
   Fator Potência: (Cos) 0.64
   FSSF 1.15

   CONFIGURANDO PARÂMETROS DO MOTOR NO ATV12H037M2
   -----------------------------------
   Conf -> Full -> DrC- (Configuração do Motor):
   BFR 60
   NPR 0.2
   UNS 220
   NCR 1.6
   FRS 60
   NSP 1720 (Só aparece se NPC estiver como NPR)
   TFR 60 (default 72)
   CTT PERF
   UFR 100
   SLP 100
   STA 20
   FLG 20
   SFT HF1
   SFR 8.0 (Default 4.0 khz - Redução do ruído agudo)
   NRD YES (Redução de ruído agudo)
   TUN NO
   NPC NPR/COS
   COS 0.64 (Só aparece se NPC estiver como COS)

   CONFIGURANDO PARÂMETROS DE PROTEÇÃO TÉRMICA NO ATV12H037M2
   -----------------------------------
   FLT -> THT
   ITH 1.9

   CONFIGURANDO RAMPAS E VELOCIDADES NO ATV12H037M2
   -----------------------------------
   Conf ->
   ACC 3.0
   DEC 1.0
   LSP 0.0
   HSP 60.0

   CONFIGURANDO PARÂMETROS DE COMUNICAÇÃO MODBUS NO ATV12H037M2
   -----------------------------------
   Conf -> Full -> COM (Comunicação):
   ADD (Endereço Modbus): 1 (ou o desejado)
   TBR (Baud rate): 19.2 kbps (valor 19)
   TFO (Formato): 8E1 (valor 1)
   TTO (Timeout): 2.0 s (valor 20) padrão da Biblioteca ModbusMaster

   CONFIGURANDO CONTROLE POR MODBUS NO ATV12H037M2
   -----------------------------------
   Conf -> Full -> CTL (Comandos):
   CHCF: Sep
   CD1: Modbus
   FR1: Modbus

*/
#include <esp_dmx.h>
#include <ModbusMaster.h>
#include <Preferences.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <vfd.h> // Biblioteca customizada com os VFDs de vários fabricantes

TFT_eSPI tft = TFT_eSPI();
#define TFT_BL 4

/*=============================  MODBUS  =============================*/
#define MODBUS_RESPONSE_TIMEOUT 2000   // ms
#define STATE_TRANSITION_DELAY  500    // ms
#define MODBUS_TX 15
#define MODBUS_RX 17

ModbusMaster node;

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
  Serial2.begin(19200, SERIAL_8E1, MODBUS_RX, MODBUS_TX); // RX first TX second
  node.begin(1, Serial2);
  Serial.println("Checando conexão MODBUS");
  vfd.checkModbusAddress();
  vfd.verifyModbusConnection();
  vfd.resetFault();

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
    // Check Inverter Config
    if (configStep == IDLE && readEncoderButton()) {
      configStep = SELECT_INVERTER;
      encoder.clearCount();
      drawInverterScreen();
      lastUpdate = now;
      return;
    }
    // Check DMX first Channel
    if (configStep == SELECT_INVERTER && readEncoderButton()) {
      configStep = SELECT_CHANNEL;
      encoder.clearCount();
      encoder.setCount(dmxBaseChannel);
      drawChannelScreen();
      lastUpdate = now;
      return;
    }
    // Save both Configs
    if (configStep == SELECT_CHANNEL && readEncoderButton()) {
      prefs.begin("dmx-config", false);
      prefs.putUChar("inverter", inverterBrand);
      prefs.putInt("baseChannel", dmxBaseChannel);
      prefs.end();
      ESP.restart();
      lastUpdate = now;
      return;
    }
    // Not config. Read DMX, if changed, processDMXCommands and Update Display
    if (configStep == IDLE) {
      readDMX();
      if (dmxChanged()) {
        lastDmxEnable    = dmxEnable;
        lastDmxDirection = dmxDirection;
        lastDmxSpeed     = dmxSpeed;
        lastDmxTenths    = dmxTenths;
        processDMXCommands();
        updateDisplayData();
      }
    }
    // Constantly Check Motor Status at 0,5s
    static unsigned long lastStatusRead = 0;
    if (now - lastStatusRead > 500) {
      vfd.readMotorStatus();
      lastStatusRead = now;
      // updateDisplayData();
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
  // ------------------------------------------------------------------
  // 0. Sanity checks
  // ------------------------------------------------------------------
  if (!vfd.verifyModbusConnection()) {
    Serial.println("Erro: Sem comunicação MODBUS - Verifique ATV12");
    return;
  }
  vfd.readFrequencyLimits();

  // ------------------------------------------------------------------
  // 1. Decode DMX values
  // ------------------------------------------------------------------
  bool enableMotor = (dmxEnable >= 128); // if first channel is equal or bigger 128, activate motor

  uint16_t targetSpeed = 0;
  if (enableMotor) { // if motor active
    static uint8_t lastSpeed = 0, lastTenths = 0;

    uint8_t speedVal = (dmxSpeed * 61) / 256; // Integer part of Speed
    if (abs(speedVal - lastSpeed) >= 1 || dmxSpeed == 0 || dmxSpeed == 255)
      lastSpeed = speedVal;

    uint8_t tenthsVal = (dmxTenths * 10) / 256; // Decimal part of Speed
    if (abs(tenthsVal - lastTenths) >= 1 || dmxTenths == 0 || dmxTenths == 255)
      lastTenths = tenthsVal;

    targetSpeed = constrain(lastSpeed, 0, 60) * 10 + constrain(lastTenths, 0, 9); // Set new speed
    targetSpeed = constrain(targetSpeed, vfd.getMinFrequency(), vfd.getMaxFrequency()); // Constrain in limits
  }

  bool newDir      = (dmxDirection >= 128); // if second channel is equal or bigger than 128, change direction
  bool dirChanged  = newDir != (vfd.getMotorDirection() == 1);

  uint16_t st = vfd.getMotorStatus() & 0x004F;

  // ------------------------------------------------------------------
  // 2. Fault handling
  // ------------------------------------------------------------------
  if (st == STATE_FAULT) {
    if (!vfd.resetFault()) {
      Serial.println("Falha: Reset de falha não realizado");
      return;
    }
    delay(1000);
    vfd.readMotorStatus();
    st = vfd.getMotorStatus() & 0x004F;   // refresh
  }

  // ------------------------------------------------------------------
  // 3. Speed & direction
  // ------------------------------------------------------------------
  vfd.setMotorSpeed(targetSpeed);          // envia velocidade
  if (!vfd.setMotorDirection(newDir))      // ajusta direção
    Serial.println("Aviso: falha ao ajustar direção");
  delay(50); // Added to give time for direction change...
  
  // ------------------------------------------------------------------
  // 4. State machine
  // ------------------------------------------------------------------
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
  }

  if (dirChanged && st == STATE_OPERATION_ENABLED) {
    vfd.sendCommandWithRetry(CMD_DISABLE_OPERATION);
    delay(500);
    st = vfd.getMotorStatus() & 0x004F;   // refresh
  }
  if (enableMotor) {

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
        if (dirChanged) {
          vfd.sendCommandWithRetry(newDir ? CMD_RUN_REVERSE : CMD_RUN_FORWARD);
        }
        break;
    }
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
  tft.fillCircle(xValue + 72, ySt - 2, r,   vfd.verifyModbusConnection() != 0 ? TFT_GREEN : TFT_RED);

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
