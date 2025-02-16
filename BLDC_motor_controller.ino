#include "driver/gpio.h"
#include "hal/gpio_ll.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "math.h"
#include "AS5600.h"
#include "Wire.h"

#define MAX(a, b, c) ((a) > ((b) > (c) ? (b) : (c)) ? (a) : (((b) > (c) ? (b) : (c))))

#define MIN(a, b, c) ((a) > (b) ? ((b) > (c) ? c : b) : ((a) > (c) ? c : a))

#define CLAMP(x, a, b) constrain(x, a, b)//((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

#define SIN(x) (sinf(x))
#define COS(x) (cosf(x))

#define PWM_FREQ 60000   // 60kHz : up_down_counter

#define DUTY 60.0

#define POLES 40.0f

#define KP 0.00005f

int greenH = 25;
int greenL = 18;
int blueH = 26;
int blueL = 27;
int yellowH = 23;
int yellowL = 19;

int potpin = 15;
int potVal = 0;

float Vd = 0.01f;
float Vq = 1.0f;

float Va = 0.0f;
float Vb = 0.0f;
float Vc = 0.0f;

float Vbeta = 0.0f;
float Valpha = 0.0f;
float Valpha_fact = 0.0f;
float Vbeta_fact = 0.0f;

float Vcom = 0.0f;
float Vamp = 0.0f;

float target_mech_angle = 0.0f;
float real_angle = 0.0f;
float elec_angle = 0.0f;

float duty_a = 0.0f;
float duty_b = 0.0f;
float duty_c = 0.0f;

float Vmax = 0.0f;
float Vmin = 0.0f;

uint16_t mechraw = 0;

bool startup = true;

float elec_offset = 0.0f;

AS5600 as5600;

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);

  as5600.begin();

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, greenH); // Timer0, Operator A
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, greenL); // Timer0, Operator B

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, yellowH); // Timer1, Operator A
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, yellowL); // Timer1, Operator B

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, blueH); // Timer2, Operator A
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, blueL); // Timer2, Operator B

  mcpwm_config_t pwm_config0;
  pwm_config0.frequency    = PWM_FREQ;             // 60 kHz
  pwm_config0.cmpr_a       = 20.0;                 // Duty cycle % for Operator A
  pwm_config0.cmpr_b       = 20.0;                 // Duty cycle % for Operator B
  pwm_config0.counter_mode = MCPWM_UP_COUNTER;     // Count up
  pwm_config0.duty_mode    = MCPWM_DUTY_MODE_0;    // Active high

  // Initialize Timer0 with the above config
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config0);

  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 10, 10);

  // 3. Configure Timer1
  mcpwm_config_t pwm_config1;
  pwm_config1.frequency    = PWM_FREQ;             // 60 kHz
  pwm_config1.cmpr_a       = 20.0;                 // Duty cycle % for Operator A
  pwm_config1.cmpr_b       = 20.0;                  // Not used (Operator B)
  pwm_config1.counter_mode = MCPWM_UP_COUNTER;     
  pwm_config1.duty_mode    = MCPWM_DUTY_MODE_0;    

  // Initialize Timer1 with the above config
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config1);

  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 10, 10);

  mcpwm_config_t pwm_config2;
  pwm_config2.frequency    = PWM_FREQ;             // 60 kHz
  pwm_config2.cmpr_a       = 20.0;                 // Duty cycle % for Operator A
  pwm_config2.cmpr_b       = 20.0;                  // Not used (Operator B)
  pwm_config2.counter_mode = MCPWM_UP_COUNTER;     
  pwm_config2.duty_mode    = MCPWM_DUTY_MODE_0;    

  // Initialize Timer2 with the above config
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config2);

  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 10, 10);

  delay(500);

  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_2);

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  //mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
  //mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
  //mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, 0);
}

void loop() {
  mechraw = as5600.readAngle();

  real_angle = (float)mechraw * 0.00153398f;

  potVal = analogRead(potpin);
  //potAngleDeg = map(potValue, 0, 4095, 0, 360);
  target_mech_angle = (float)potVal * 0.00153398f;

  Vq = CLAMP(KP * (target_mech_angle - real_angle), -0.1f, 0.1f);

  elec_angle = real_angle * POLES - elec_offset;

  if(startup){
    Vq = 0.0f;
    Vd = 1.0f;
  }

  Valpha = Vd*COS(elec_angle) - Vq*SIN(elec_angle);
  Vbeta = Vd*SIN(elec_angle) + Vq*COS(elec_angle);

  Valpha_fact = Valpha * -0.5f;
  Vbeta_fact = Vbeta * 0.8660254038f;

  Va = Valpha;
  Vb = Valpha_fact + Vbeta_fact;
  Vc = Valpha_fact - Vbeta_fact;

  Vmax = MAX(Va, Vb, Vc);
  Vmin = MIN(Va, Vb, Vc);

  Vcom = (Vmax + Vmin) * 0.5f;
  Vamp = (Vmax - Vmin) * 0.5f;

  Va = Va - Vcom;
  Vb = Vb - Vcom;
  Vc = Vc - Vcom;

  duty_a = (Va + Vamp) * 0.5f * (DUTY/Vamp);
  duty_b = (Vb + Vamp) * 0.5f * (DUTY/Vamp);
  duty_c = (Vc + Vamp) * 0.5f * (DUTY/Vamp);
  
  duty_a = CLAMP(duty_a, 0.0f, DUTY);
  duty_b = CLAMP(duty_b, 0.0f, DUTY);
  duty_c = CLAMP(duty_c, 0.0f, DUTY);

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_a); // A high
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_a); // A low

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_b); // B high
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, duty_b); // B low

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, duty_c); // C high
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, duty_c); // C low

  if(startup){
    delay(500);
    elec_offset = real_angle;
    startup = false;
    Serial.println(elec_offset);
  }
}
