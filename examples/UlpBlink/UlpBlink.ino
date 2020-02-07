#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "UlpDebug.h"

// Slow memory variable assignment
enum {
  SLOW_BLINK_STATE,     // Blink status

  SLOW_PROG_ADDR        // Program start address
};

void ULP_BLINK(uint32_t us) {
  // Set ULP activation interval
  ulp_set_wakeup_period(0, us);

  // Slow memory initialization
  memset(RTC_SLOW_MEM, 0, 8192);

  // Blink status initialization
  RTC_SLOW_MEM[SLOW_BLINK_STATE] = 0;

  // PIN to blink (specify by +14)
  const int pin_blink_bit = RTCIO_GPIO26_CHANNEL + 14;
  const gpio_num_t pin_blink = GPIO_NUM_26;

  // GPIO26 initialization (set to output and initial value is 0)
  rtc_gpio_init(pin_blink);
  rtc_gpio_set_direction(pin_blink, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(pin_blink, 0);

  // ULP Program
  const ulp_insn_t  ulp_prog[] = {
    I_MOVI(R3, SLOW_BLINK_STATE),           // R3 = SLOW_BLINK_STATE
    I_LD(R0, R3, 0),                        // R0 = RTC_SLOW_MEM[R3(SLOW_BLINK_STATE)]
    M_BL(1, 1),                             // IF R0 < 1 THAN GOTO M_LABEL(1)

    // R0 => 1 : run
    I_WR_REG(RTC_GPIO_OUT_REG, pin_blink_bit, pin_blink_bit, 1), // pin_blink_bit = 1
    I_MOVI(R0, 0),                          // R0 = 0
    I_ST(R0, R3, 0),                        // RTC_SLOW_MEM[R3(SLOW_BLINK_STATE)] = R0
    M_BX(2),                                // GOTO M_LABEL(2)

    // R0 < 1 : run
    M_LABEL(1),                             // M_LABEL(1)
    I_WR_REG(RTC_GPIO_OUT_REG, pin_blink_bit, pin_blink_bit, 0),// pin_blink_bit = 0
    I_MOVI(R0, 1),                          // R0 = 1
    I_ST(R0, R3, 0),                        // RTC_SLOW_MEM[R3(SLOW_BLINK_STATE)] = R0

    M_LABEL(2),                             // M_LABEL(2)
    I_DELAY(60000),
    I_DELAY(60000),
    I_DELAY(60000),
    I_DELAY(60000),
    I_DELAY(60000),
    I_DELAY(60000),
    I_DELAY(60000),
    I_DELAY(60000),
    I_DELAY(60000),
    I_DELAY(60000),
    I_DELAY(60000),
    I_DELAY(60000),
    I_DELAY(60000),
    I_DELAY(60000),
    I_HALT()                                // Stop the program
  };

  // Run the program shifted backward by the number of variables
  size_t size = sizeof(ulp_prog) / sizeof(ulp_insn_t);
  ulp_process_macros_and_load(SLOW_PROG_ADDR, ulp_prog, &size);
  ulp_run(SLOW_PROG_ADDR);
}

void setup() {
  // For debug output
  Serial.begin(115200);

  // Execute ULP program at 300ms intervals
  ULP_BLINK(300000);
}

void loop() {
  // For debug output
  ulpDump();

  // Wait
  delay(1000);
}
