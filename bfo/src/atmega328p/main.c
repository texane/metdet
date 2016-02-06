#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define CONFIG_UART 1
#ifdef CONFIG_UART
#include "./uart.c"
#endif /* CONFIG_UART */


/* high resolution frequency counter
 */

/* timer2 interrupt handler. timer1 is an extended
   32 bits register (16 bits hard + 16 softs)
   incremented once per:
   1 / (fcpu / prescal) <=> prescal / fcpu
   thus, it will overflow at:
   2^16 * prescal / fcpu
   on tim2 overflow, the interrupt handler is called
   and stores the tim1 current value in tim1_cur_counter.
   thus, the tim1 value integrated over the whole
   tim1 period is:
   (tim1_ovf_counter * 2^16) + tim1_cur_counter.
   tim2_is_ovf is set to notify the application.
 */

static volatile uint8_t tim2_ovf_counter;
static volatile uint8_t tim2_is_ovf;
static volatile uint16_t tim1_cur_counter;

ISR(TIMER2_OVF_vect)
{
  if ((tim2_ovf_counter--) == 0)
  {
    /* disable tim1 before reading */
    TCCR1B = 0;
    tim1_cur_counter = TCNT1;

    /* disable tim2 */
    TCCR2B = 0;

    tim2_is_ovf = 1;
  }
}

/* timer1 interrupt handler. timer1 is a 8 bits counter
   incremented by the input signal rising edges. since
   8 bits are not enough to integrate, an auxiliary
   register (tim1_ovf_counter) is updated on overflow.
   tim1_ovf_counter is an 8 bits register, and will
   overflow without any notice past 0xff.
 */

static volatile uint8_t tim1_ovf_counter;

ISR(TIMER1_OVF_vect)
{
  ++tim1_ovf_counter;
}

static void hfc_start(void)
{
  /* the total acquisition time is given by: */
  /* tacq = tim2_max * 1 / (fcpu / tim2_prescal) */
  /* with: */
  /* tim2_max =  12500 */
  /* fcpu = 16MHz */
  /* tim2_prescal = 128 */
  /* tacq = 0.1 */

  /* disable interrupts */
  TIMSK1 = 0;
  TIMSK2 = 0;

  /* reset stuff */
  tim1_ovf_counter = 0;
  tim1_cur_counter = 0;
  tim2_is_ovf = 0;

  /* 0x100 overflows make 16 bits */
  /* 12500 = 256 * 49 */
  tim2_ovf_counter = 49;

  /* configure tim2
     normal operation
     prescaler 128
     enable interrupt on overflow
   */
  TCNT2 = 0;
  TIMSK2 = 1 << 0;
  TCCR2A = 0;
  TCCR2B = 0;

  /* configure tim1
     t1 pin (pd5) rising edge as external clock
   */
  DDRD &= ~(1 << 5);
  TCNT1 = 0;
  TIMSK1 = 1 << 0;
  TCCR1A = 0;
  TCCR1B = 0;

  /* start tim1, tim2 */
  TCCR1B = 7 << 0;
  TCCR2B = 5 << 0;
}

static uint8_t hfc_poll(void)
{
  return tim2_is_ovf;
}

static uint32_t hfc_wait(void)
{
  /* busy wait for tim1 to overflow. returns the resulting
     16 bits counter, to be multiplied by the frequency
     resolution (refer to hfc_start) to get the actual
     frequency.
   */

  /* force inline, do not use hfc_poll */
  while (tim2_is_ovf == 0) ;

  return ((uint32_t)tim1_ovf_counter << 16) | (uint32_t)tim1_cur_counter;
}

static inline uint32_t hfc_start_wait(void)
{
  hfc_start();
  return hfc_wait();
}

__attribute__((unused))
static inline uint32_t hfc_to_hz(uint32_t counter)
{
  /* refer to hfc_start notes */
  /* f = counter / tacq */
  /* f = counter * facq */
  /* with: */
  /* tacq = 0.1, facq = 10 */
  return 10 * counter;
}


/* pwm as piezo driver */
/* use portd6, d6 */

#define PWM_N_MIN 10
#define PWM_N_MAX 100

#define PWM_FLAG_START (1 << 0)
static uint8_t pwm_flags = 0;
static uint8_t pwm_n = 0;

static void pwm_update(uint8_t n)
{
  TCNT0 = 0;
  OCR0A = n / 2;
}

static void pwm_start(uint8_t n)
{
  TCCR0B = 0;

  DDRD |= 1 << 6;

  pwm_update(n);

  TIMSK0 = 0;

  TCCR0A = (1 << 6) | (1 << 1);
  TCCR0B = (1 << 3) | (5 << 0);
}

static void pwm_stop(void)
{
  TCCR0B = 0;
}


/* calibrate */

static void calibrate(uint32_t* x)
{
  uint8_t i;

  *x = 0;
  for (i = 0; i != 8; ++i) *x += hfc_start_wait();
  *x /= 8;
}


/* main */

int main(void)
{
#define CONFIG_THRESH 2

  uint32_t calib_counter;
  uint32_t counter;
  uint32_t diff;

#ifdef CONFIG_UART
  uart_setup();
#endif /* CONFIG_UART */

  sei();

  calibrate(&calib_counter);

#ifdef CONFIG_UART
  /* print calibration counter */
  uart_write((uint8_t*)"CALIB ", 6);
  uart_write((uint8_t*)uint32_to_string(calib_counter), 8);
  uart_write((uint8_t*)"\r\n", 2);
#endif /* CONFIG_UART */

  /* measure */
  while (1)
  {
    counter = hfc_start_wait();

    if (counter > calib_counter) diff = counter - calib_counter;
    else diff = calib_counter - counter;

    if (diff < CONFIG_THRESH)
    {
      if (pwm_flags & PWM_FLAG_START)
      {
	pwm_stop();
	pwm_flags &= ~PWM_FLAG_START;

#ifdef CONFIG_UART
	uart_write((uint8_t*)"PWM_STOP\r\n", 10);
#endif /* CONFIG_UART */
      }
    }
    else
    {
      /* compute pwm frequency */
      diff = diff / 4 + 10;
      if (diff >= PWM_N_MAX) diff = PWM_N_MAX;

      if ((pwm_flags & PWM_FLAG_START) == 0)
      {
	pwm_start((uint8_t)diff);
	pwm_flags |= PWM_FLAG_START;

#ifdef CONFIG_UART
	uart_write((uint8_t*)"PWM_START ", 10);
	uart_write((uint8_t*)uint32_to_string(counter), 8);
	uart_write((uint8_t*)"\r\n", 2);
#endif /* CONFIG_UART */
      }
      else if (pwm_n != (uint8_t)diff)
      {
	pwm_update((uint8_t)diff);
	pwm_n = (uint8_t)diff;

#ifdef CONFIG_UART
	uart_write((uint8_t*)"PWM_UPDATE ", 11);
	uart_write((uint8_t*)uint32_to_string(counter), 8);
	uart_write((uint8_t*)"\r\n", 2);
#endif /* CONFIG_UART */
      }
    }
  }

  return 0;
}
