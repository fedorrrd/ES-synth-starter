#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>

HardwareTimer sampleTimer(TIM1);
// std::bitset<32> inputs;

// Constants
const uint32_t interval = 100;                                                                                                                         // Display update interval
const uint32_t frequencies[] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 444, 466, 494};                                                           // frequencies of notes for sawtooth
const uint32_t stepSizes[] = {51149156, 54077543, 57396381, 60715219, 64424509, 68133799, 72233541, 76528508, 81018701, 86680249, 90975216, 96441538}; // stepsizes for sawtooth
const std::string notes[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

// volatiles
volatile uint32_t currentStepSize;
volatile uint32_t knob3Rotation;

// struct to store variables between threads
struct
{
  std::bitset<32> inputs;
  uint32_t knobRotation[4] = {0, 0, 0, 0};
  SemaphoreHandle_t mutex;

} sysState;

// Pin definitions
// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// read inputs
std::bitset<4> readCols()
{
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);

  return result;
}

void setRow(uint8_t rowIdx)
{
  digitalWrite(REN_PIN, LOW);

  digitalWrite(RA0_PIN, rowIdx & 1);
  digitalWrite(RA1_PIN, rowIdx & 2);
  digitalWrite(RA2_PIN, rowIdx & 4);

  digitalWrite(REN_PIN, HIGH);
}

void sampleISR()
{
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - knob3Rotation);
  analogWrite(OUTR_PIN, Vout + 128);
}

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

void scanKeysTask(void *pvParameters)
{
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t localCurrentStepSize = 0;
  int knobDirection[] = {0, 0, 0, 0};
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    // make placeholder for inputs
    std::bitset<32> inputs;

    // read all inputs
    bool keypress = false;
    for (int i = 0; i < 8; i++)
    {
      setRow(i);
      delayMicroseconds(3);
      std::bitset<4> input_iteration = readCols();
      for (int j = 0; j < 4; j++)
      {
        inputs[i * 4 + j] = input_iteration[j];
        // if input is a key - change current step size
        if (i < 3)
        {
          if (inputs[i * 4 + j] == 0)
          {
            localCurrentStepSize = stepSizes[i * 4 + j];
            keypress = true;
          }
        }
      }
    }

    // if no keys are pressed - reset current step size
    if (!keypress)
    {
      localCurrentStepSize = 0;
    }
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

    // access systate for previous inputs and current knob rotation values
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    // since sysState.inputs is updated at the end of the code - it stores the input of the previous iteration
    std::bitset<32> previousInputs = sysState.inputs;
    uint32_t knobRotation[] = {sysState.knobRotation[0], sysState.knobRotation[1], sysState.knobRotation[2], sysState.knobRotation[3]};
    xSemaphoreGive(sysState.mutex);

    // read knobs' rotation
    for (int i = 12; i < 20; i += 2)
    {
      if (previousInputs[i] == 0 && previousInputs[i + 1] == 0)
      {
        if ((inputs[i] == 1 && inputs[i + 1] == 0) || (inputs[i] == 1 && inputs[i + 1] == 1 && knobDirection[(i - 12) / 2] == 1))
        {
          knobDirection[(i - 12) / 2] = 1;
        }
        else if ((inputs[i] == 0 && inputs[i + 1] == 1) || (inputs[i] == 1 && inputs[i + 1] == 1 && knobDirection[(i - 12) / 2] == -1))
        {
          knobDirection[(i - 12) / 2] = -1;
        }
        else
        {
          knobDirection[(i - 12) / 2] = 0;
        }
      }
      else if (previousInputs[i] == 1 && previousInputs[i + 1] == 1)
      {
        if ((inputs[i] == 0 && inputs[i + 1] == 1) || (inputs[i] == 0 && inputs[i + 1] == 0 && knobDirection[(i - 12) / 2] == 1))
        {
          knobDirection[(i - 12) / 2] = 1;
        }
        else if ((inputs[i] == 1 && inputs[i + 1] == 0) || (inputs[i] == 0 && inputs[i + 1] == 0 && knobDirection[(i - 12) / 2] == -1))
        {
          knobDirection[(i - 12) / 2] = -1;
        }
        else
        {
          knobDirection[(i - 12) / 2] = 0;
        }
      }
      else
      {
        knobDirection[(i - 12) / 2] = 0;
      }
    }

    // adjust the knob value according to the rotation
    if ((knobRotation[0] == 0 && knobDirection[0] == -1) || (knobRotation[0] == 8 && knobDirection[0] == 1))
    {
      knobDirection[0] = 0;
    }
    for (int i = 0; i < 4; i++)
    {
      knobRotation[i] = knobRotation[i] + knobDirection[i];
    }

    // volume control
    __atomic_store_n(&knob3Rotation, knobRotation[0], __ATOMIC_RELAXED);

    // store sysState variables
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = inputs;
    for (int i = 0; i < 4; i++)
    {
      sysState.knobRotation[i] = knobRotation[i];
    }
    xSemaphoreGive(sysState.mutex);

    // Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void displayUpdateTask(void *pvParameters)
{
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    // access systate
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    std::bitset<32> inputs = sysState.inputs;
    uint32_t knobRotation[] = {sysState.knobRotation[0], sysState.knobRotation[1], sysState.knobRotation[2], sysState.knobRotation[3]};
    xSemaphoreGive(sysState.mutex);

    bool keypress = false;
    for (int i = 0; i < 12; i++)
    {
      if (inputs[i] == 0)
      {
        u8g2.drawStr(2, 10, notes[i].c_str());
        keypress = true;
      }
    }

    if (!keypress)
    {
      u8g2.drawStr(2, 10, "no keypress");
    }

    u8g2.setCursor(2, 20);
    u8g2.print(knobRotation[0]);
    u8g2.print(" ");
    u8g2.print(knobRotation[1]);
    u8g2.print(" ");
    u8g2.print(knobRotation[2]);
    u8g2.print(" ");
    u8g2.print(knobRotation[3]);

    u8g2.setCursor(2, 30);
    u8g2.print(inputs.to_ulong(), HEX);

    u8g2.sendBuffer();
  }
}

void setup()
{
  // put your setup code here, to run once:

  // Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  // Initialise display
  setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

  // Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  // setup interrupt
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  sysState.mutex = xSemaphoreCreateMutex();

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask,     /* Function that implements the task */
      "scanKeys",       /* Text name for the task */
      256,              /* Stack size in words, not bytes */
      NULL,             /* Parameter passed into the task */
      2,                /* Task priority */
      &scanKeysHandle); /* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
      displayUpdateTask,     /* Function that implements the task */
      "scanKeys",            /* Text name for the task */
      256,                   /* Stack size in words, not bytes */
      NULL,                  /* Parameter passed into the task */
      1,                     /* Task priority */
      &displayUpdateHandle); /* Pointer to store the task handle */

  vTaskStartScheduler();
}

void loop()
{
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;

  while (millis() < next)
    ; // Wait for next interval

  next += interval;
}