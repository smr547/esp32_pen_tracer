#include <Arduino.h>
#include <SPI.h>

// ---------------------- PIN DEFINITIONS ----------------------
#define XPT_CS_PIN 21   // Chip Select for XPT2046
#define XPT_IRQ_PIN 14  // PENIRQ pin (active LOW
#define T_DIN 16
#define T_DOUT 18
#define T_SCLK 17

// ---------------------- XPT2046 COMMAND BYTES ----------------
// 12-bit, differential, PD=00 (power down between conversions)
#define CMD_X 0b10010000   // X position
#define CMD_Y 0b11010000   // Y position
#define CMD_Z1 0b10110000  // Z1 (pressure-related)
#define CMD_Z2 0b11000000  // Z2 (optional if you want fancier pressure)

// ---------------------- TOUCH / FILTER CONSTANTS -------------
#define Z1_TOUCH_THRESHOLD \
    5                  // Tune experimentally (smaller = stronger contact)
#define NUM_SAMPLES 5  // Median filter window

// ---------------------- GLOBALS ------------------------------
volatile bool touchIrqFlag = false;  // Set in ISR when PENIRQ falls
volatile uint16_t interrupt_count = 0;
bool touchActive = false;  // State: currently handling a touch?

SPIClass* vspi = &SPI;

// ---------------------- FORWARD DECLARATIONS -----------------
// void IRAM_ATTR penIrqISR();
void penIrqISR();
uint16_t xptReadADC(uint8_t cmd);
bool isTouchStillPresent();
uint16_t medianOfArray(uint16_t* arr, int n);
void touchSamplingLoopStep();
bool confirmPenUp();
void onPenDown(uint16_t x, uint16_t y, uint16_t z, uint16_t ic);
void reportRawValues(uint16_t x, uint16_t y, uint16_t z, uint16_t ic);
void onPenMove(uint16_t x, uint16_t y, uint16_t z, uint16_t ic);
void onPenUp();
void reArmIRQ(void);

// ---------------------- SETUP -------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("XPT2046 + ESP32 (interrupt start) demo");

    pinMode(XPT_CS_PIN, OUTPUT);
    digitalWrite(XPT_CS_PIN, HIGH);  // CS idle high

    pinMode(XPT_IRQ_PIN, INPUT_PULLUP);  // PENIRQ is open-drain, active low

    // Init SPI (VSPI)
    vspi->begin(T_SCLK, T_DOUT, T_DIN, XPT_CS_PIN);  // SCK, MISO, MOSI, SS
    vspi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

    // Attach interrupt on falling edge
    // attachInterrupt(digitalPinToInterrupt(XPT_IRQ_PIN), penIrqISR, FALLING);
    reArmIRQ();
}

// ---------------------- MAIN LOOP ----------------------------
void loop() {
    /*

      detachInterrupt(digitalPinToInterrupt(XPT_IRQ_PIN));
      uint16_t x = xptReadADC(CMD_X);
      if (x == 2047) {
          x = 0;
      }
      uint16_t y = xptReadADC(CMD_Y);
      uint16_t z = xptReadADC(CMD_Z1);
      uint16_t ic = interrupt_count;
      attachInterrupt(digitalPinToInterrupt(XPT_IRQ_PIN), penIrqISR, FALLING);

      reportRawValues(x, y, z, ic);
      if (x + y + z == 0) {
          // interrupt_count = 0;
          // attachInterrupt(digitalPinToInterrupt(XPT_IRQ_PIN), penIrqISR,
          // FALLING);
      }

      delay(500);
      */

    if (!touchActive) {
        // IDLE / WAIT FOR TOUCH
        if (touchIrqFlag) {
            touchIrqFlag = false;

            // Confirm it's a real touch (not noise)
            if (isTouchStillPresent()) {
                touchActive = true;

                // Optional: initial position sample at pen-down
                uint16_t x = xptReadADC(CMD_X);
                uint16_t y = xptReadADC(CMD_Y);
                uint16_t z = xptReadADC(CMD_Z1);

                onPenDown(x, y, z, interrupt_count);

                // We stay in touchActive and run the sampling loop
            } else {
                // False alarm: re-enable IRQ
                reArmIRQ();
            }
        }

        // You *could* put the ESP32 to light sleep here in a real design.

    } else {
        // TOUCH ACTIVE STATE: take samples and check for pen-up
        touchSamplingLoopStep();
    }
    delay(100);  // slow things down
}

void reArmIRQ(void) {
    touchActive = false;
    interrupt_count = 0;
    attachInterrupt(digitalPinToInterrupt(XPT_IRQ_PIN), penIrqISR, FALLING);
}

// ---------------------- ISR ----------------------------
// void IRAM_ATTR penIrqISR() {
void penIrqISR() {
    touchIrqFlag = true;
    interrupt_count = interrupt_count + 1;
    // Avoid re-entering: detach until main code handles it
    detachInterrupt(digitalPinToInterrupt(XPT_IRQ_PIN));
}

// ---------------------- LOW-LEVEL XPT READ -------------------
uint16_t xptReadADC(uint8_t cmd) {
    uint8_t hi, lo;

    digitalWrite(XPT_CS_PIN, LOW);
    vspi->transfer(cmd);
    hi = vspi->transfer(0x00);
    lo = vspi->transfer(0x00);
    digitalWrite(XPT_CS_PIN, HIGH);

    uint16_t value = ((uint16_t)hi << 8 | lo) >> 4;  // 12-bit result
    return value;
}

// ---------------------- TOUCH PRESENCE CHECK -----------------
bool isTouchStillPresent() {
    // Simple approach: use Z1
    uint16_t z1 = xptReadADC(CMD_Z1);
    return (z1 > Z1_TOUCH_THRESHOLD);
}

// ---------------------- MEDIAN FILTER ------------------------
uint16_t medianOfArray(uint16_t* arr, int n) {
    // Simple in-place bubble sort (fine for n=5)
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - 1 - i; j++) {
            if (arr[j] > arr[j + 1]) {
                uint16_t tmp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = tmp;
            }
        }
    }
    return arr[n / 2];  // Middle element
}

// ---------------------- TOUCH SAMPLING STEP ------------------
void touchSamplingLoopStep() {
    const int N = NUM_SAMPLES;

    uint16_t xSamples[N];
    uint16_t ySamples[N];
    uint16_t zSamples[N];

    // Take a small burst of samples
    for (int i = 0; i < N; i++) {
        xSamples[i] = xptReadADC(CMD_X);
        ySamples[i] = xptReadADC(CMD_Y);
        zSamples[i] = xptReadADC(CMD_Z1);
        delayMicroseconds(200);  // settle time; tweak as needed
    }

    uint16_t x = medianOfArray(xSamples, N);
    uint16_t y = medianOfArray(ySamples, N);
    uint16_t z = medianOfArray(zSamples, N);

    // Check if touch probably released
    if (z <= Z1_TOUCH_THRESHOLD) {
        if (confirmPenUp()) {
            onPenUp();
            reArmIRQ();
        }
    } else {
        // Still touching: report movement
        onPenMove(x, y, z, interrupt_count);
    }
}

// ---------------------- PEN-UP CONFIRMATION ------------------
bool confirmPenUp() {
    // Take a few quick checks to avoid jitter-based false pen-up
    for (int i = 0; i < 3; i++) {
        delay(2);
        if (isTouchStillPresent()) {
            return false;  // still touching
        }
    }
    return true;  // released
}

// ---------------------- CALLBACKS ----------------------------
// In a real app, you'd map raw X/Y to screen coordinates here
void onPenDown(uint16_t x, uint16_t y, uint16_t z, uint16_t ic) {
    Serial.printf("PEN DOWN:  X=%u  Y=%u  Z=%u  interrupts=%u\n", x, y, z, ic);
}

void reportRawValues(uint16_t x, uint16_t y, uint16_t z, uint16_t ic) {
    Serial.printf("RAW VALUES:  X=%u  Y=%u  Z=%u  interrupts=%u\n", x, y, z,
                  ic);
}

void onPenMove(uint16_t x, uint16_t y, uint16_t z, uint16_t ic) {
    static uint16_t last_x = 0;
    static uint16_t last_y = 0;
    if (last_x != x or last_y != y) {
        Serial.printf("MOVE:      X=%u  Y=%u  Z=%u  interrupts=%u\n", x, y, z,
                      ic);
        last_x = x;
        last_y = y;
    }
}

void onPenUp() { Serial.println("PEN UP"); }