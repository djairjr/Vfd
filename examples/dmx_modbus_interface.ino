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
  delay(1000);

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
  // readDMX(); // Added to update DMX values at boot
  delay(500);

  // MODBUS
  Serial2.begin(19200, SERIAL_8E1, MODBUS_RX, MODBUS_TX); // RX first TX second
  node.begin(1, Serial2);
  Serial.println("Checando conexão MODBUS");
  vfd.checkModbusAddress();
  vfd.verifyModbusConnection();
  delay(500);
  // vfd.resetFault();

  // VFD
  Serial.println ("Read Frequency Limits and Motor Status");
  vfd.readFrequencyLimits();
  vfd.readMotorStatus();
  updateDisplayData();
  delay(500);
}

/*======================================================================*/
/*                                LOOP                                  */
/*======================================================================*/
void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  checkEncoder();

  if (now - lastUpdate >= 50) { // 20ms é muito rápido para o ATV12
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
  vfd.readMotorStatus(); // Atualiza o status primeiro!

  // ------------------------------------------------------------------
  // 1. Decode DMX values
  // ------------------------------------------------------------------
  bool enableMotor = (dmxEnable >= 128);
  uint16_t targetSpeed = 0;

  if (enableMotor) {
    // Processamento do Canal 3 (velocidade inteira 0-60)
    static uint8_t lastSpeedValue = 0;
    uint8_t newSpeedValue = (dmxSpeed * 61) / 256;

    if (abs(newSpeedValue - lastSpeedValue) >= 1 || dmxSpeed == 0 || dmxSpeed == 255) {
      lastSpeedValue = newSpeedValue;
    }
    uint8_t speedValue = constrain(lastSpeedValue, 0, 60);

    // Processamento do Canal 4 (décimos 0-9)
    static uint8_t lastTenthsValue = 0;
    uint8_t newTenthsValue = (dmxTenths * 10) / 256;

    if (abs(newTenthsValue - lastTenthsValue) >= 1 || dmxTenths == 0 || dmxTenths == 255) {
      lastTenthsValue = newTenthsValue;
    }
    uint8_t tenthsValue = constrain(lastTenthsValue, 0, 9);

    targetSpeed = (speedValue * 10) + tenthsValue;
    targetSpeed = constrain(targetSpeed, vfd.getMinFrequency(), vfd.getMaxFrequency());
  }

  bool newDirection = (dmxDirection >= 128);
  bool directionChanged = newDirection != (vfd.getMotorDirection() == 1);

  // Debug
  Serial.print("DMX Values - Enable: "); Serial.print(dmxEnable);
  Serial.print(", Direction: "); Serial.print(dmxDirection);
  Serial.print(", Speed: "); Serial.print(dmxSpeed);
  Serial.print(", Tenths: "); Serial.println(dmxTenths);
  Serial.print("Target Speed: "); Serial.print(targetSpeed / 10); 
  Serial.print("."); Serial.print(targetSpeed % 10); Serial.println(" Hz");

  // ------------------------------------------------------------------
  // 2. Fault handling
  // ------------------------------------------------------------------
  if ((vfd.getMotorStatus() & 0x004F) == STATE_FAULT) {
    if (!vfd.resetFault()) {
      Serial.println("Falha: Reset de falha não realizado");
      return;
    }
    delay(1000);
    vfd.readMotorStatus();
  }

  // ------------------------------------------------------------------
  // 3. Speed control (independente do estado)
  // ------------------------------------------------------------------
  if (!vfd.setMotorSpeed(targetSpeed)) {
    Serial.println("Erro: Velocidade não configurada");
  }

  // ------------------------------------------------------------------
  // 4. State machine
  // ------------------------------------------------------------------
  if (!enableMotor) {
    switch (vfd.getMotorStatus() & 0x004F) {
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

  // Se motor habilitado, tratar direção e estado
  if (directionChanged && (vfd.getMotorStatus() & 0x004F) == STATE_OPERATION_ENABLED) {
    vfd.sendCommandWithRetry(CMD_DISABLE_OPERATION);
    delay(500);
    vfd.readMotorStatus(); // Atualiza status após mudança
  }

  // Configura direção (deve vir antes dos comandos de estado)
  if (!vfd.setMotorDirection(newDirection)) {
    Serial.println("Aviso: falha ao ajustar direção");
  }

  // Máquina de estados principal
  switch (vfd.getMotorStatus() & 0x004F) {
    case STATE_SWITCH_DISABLED:
      vfd.sendCommandWithRetry(CMD_SHUTDOWN);
      delay(STATE_TRANSITION_DELAY);
      vfd.sendCommandWithRetry(CMD_SWITCH_ON);
      delay(STATE_TRANSITION_DELAY);
      vfd.sendCommandWithRetry(newDirection ? CMD_RUN_REVERSE : CMD_RUN_FORWARD);
      break;

    case STATE_READY:
      vfd.sendCommandWithRetry(CMD_SWITCH_ON);
      delay(STATE_TRANSITION_DELAY);
      vfd.sendCommandWithRetry(newDirection ? CMD_RUN_REVERSE : CMD_RUN_FORWARD);
      break;

    case STATE_SWITCHED_ON:
      vfd.sendCommandWithRetry(newDirection ? CMD_RUN_REVERSE : CMD_RUN_FORWARD);
      break;

    case STATE_OPERATION_ENABLED:
      if (directionChanged) {
        vfd.sendCommandWithRetry(newDirection ? CMD_RUN_REVERSE : CMD_RUN_FORWARD);
      }
      break;
  }

  vfd.readMotorStatus(); // Atualiza status final
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

  // Margens e posicionamento ajustados
  int xLabel = 10;
  int xValue = 125;
  int xBar = 175;
  int barW = 60, barH = 10;
  
  // Ajuste vertical (valores reduzidos para subir o conteúdo)
  int yStart = 8;       // Reduzido de 12 para 8
  int lineH = 14;       // Reduzido de 15 para 14
  int sectionSpacing = 8; // Espaço entre seções

  const char* labels[4] = {"STATE", "DIR", "SPEED", ".DEC"};
  uint8_t vals[4] = {dmxEnable, dmxDirection, dmxSpeed, dmxTenths};
  uint16_t colors[4] = {TFT_CYAN, TFT_MAGENTA, TFT_ORANGE, TFT_YELLOW};

  // DMX Channels
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

  // Motor Status (subido 4 pixels)
  int modY = yStart + 4 * lineH + sectionSpacing - 4; // Ajuste para subir
  
  float motorSpeedHz = 0.0;
  uint16_t rawSpeed = vfd.getMotorSpeed();
  bool isReverse = (vfd.getMotorDirection() == 1);
  
  if (isReverse && rawSpeed > 32767) {
    motorSpeedHz = ((int16_t)(65535 - rawSpeed + 1)) / -10.0f;
  } else {
    motorSpeedHz = rawSpeed / 10.0f;
    if (isReverse) motorSpeedHz = -motorSpeedHz;
  }

  tft.setCursor(xLabel, modY);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Motor: ");
  tft.setTextColor((vfd.getMotorStatus() & 0x004F) == STATE_OPERATION_ENABLED ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.println((vfd.getMotorStatus() & 0x004F) == STATE_OPERATION_ENABLED ? "RUN" : "STOP");

  tft.setCursor(xValue, modY);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.printf("Speed: %s%.1f Hz", (motorSpeedHz < 0) ? "-" : "", abs(motorSpeedHz));

  // Connection Status (subido 4 pixels)
  int connY = modY + lineH + 2; // Reduzido o espaçamento
  int r = 3; // Raio menor dos círculos
  tft.setCursor(xLabel, connY);
  tft.print("DMX");
  tft.fillCircle(xLabel + 72, connY + 6, r, dmxOnline ? TFT_GREEN : TFT_RED);

  tft.setCursor(xValue, connY);
  tft.print("MODBUS");
  tft.fillCircle(xValue + 72, connY + 6, r, vfd.verifyModbusConnection() ? TFT_GREEN : TFT_RED);

  // Inverter Model (subido 4 pixels)
  int invY = connY + lineH - 2; // Reduzido o espaçamento
  tft.setCursor(xLabel, invY);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Inverter:");
  tft.setCursor(xValue - 10, invY);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.print(brandNames[inverterBrand]);

  // Error Display (com posição garantida)
  uint16_t errors = vfd.getMotorErrors();
  if (errors != 0) {
    int errY = invY + lineH - 2; // Posição ajustada
    tft.setCursor(xLabel, errY);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.printf("Error: 0x%04X", errors);
  }
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
