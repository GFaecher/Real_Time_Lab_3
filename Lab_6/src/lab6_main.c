/*
 * Compile-time options:
 *	MOBA_MODE: 0 to send ASCII data to a USB port for a MOBA X-term
 *		   1 to send binary data to a USB port for a soft scope.
 *	N_ADC_CHANNELS 1 or 2
 *	FILTER can be either biquad_60Hz_notch, biquad_lowpass_50, or nothing
 *	Comment/uncomment lines in task_input_loop() to replace analogRead()
 *		with analogReadFromFile() for debugging. If so, then you need
 *	  	-DANALOG_READ_FILENAME=...	in platformio.ini
 *
 * Connections:
 *	- Reads an analog signal from Nano A0 (PA0); drives a filtered version
 *	  out to DAC channel 1 (Nano A3,PA4). The precise filter depends on the
 *	  compile-time constant FILTER.
 *	- If N_ADC_CHANNELS==2, it also reads an analog signal from Nano A1
 *	   (PA1) and drives a filtered version to DAC channel 2 (Nano A4,PA5) 
 *	- You can read input from a file rather than inputs/DAC by going into
 *	  task_input_loop(), commenting out the call to analogRead() and
 *	  uncommenting the call to analogReadFromFile(). If so, then you need
 *	  	-DANALOG_READ_FILENAME=...	in platformio.ini
 *	- Writes all filtered ADC output to UART #2 (which drives USB to the
 *	  host). This can either be ascii text (#define MOBA_MODE 1) or binary
 *	  data for a soft oscilloscope (#define MOBA_MODE 0).
 *
 * Usage as a 6-lead ECG:
 *	Use only one red electrode as RL and ignore the other one.
 *	Both black electrodes are soldered together, and are RA.
 *	Channel-1 blue=LA (the AD8232 driving Nano A0)
 *	Channel-2 blue=LL (the AD8232 driving Nano A1)
 *
 *	Lead 1 is LA - RA
 *	Lead 2 is LL - RA 
 *	Lead 3 is LL - LA
 *	Each output line is lead1,lead2 (which is what 6lead.py expects).
 *
 * Detailed usage:
 *	First set up the compile-time options listed above. Then the detailed
 *	usage depends on the mode.
 *	If MOBA_MODE=0, it immediately starts sending binary output to a soft
 *		scope. It does this forever.
 *	If MOBA_MODE=1, it prints "Type the letter 'r' to start recording."
 *		When you do so, it prints 5000 samples (10 seconds of data) to
 *		the UART in ASCII.

 *	In both modes:
 *		- the green LED should always, always blink.
 *		- filtering is applied if requested by FILTER
 *		- input channel 1 drives out to Nano A3, and input channel 2
 *		  (if N_ADC_CHANNELS==2) drives out to Nano A4.
 */

#define N_DATA_SAMPLES 5000	// Take this many samples in MOBA_MODE.
#define SAMPLE_DELAY 2		// Sample every 2ms.
#define MOBA_MODE 1
#define N_ADC_CHANNELS 1

// Define FILTER to biquad_60Hz_notch, or biquad_lowpass_50.
// Or don't define it at all, if you don't want any filtering.
#define FILTER biquad_60Hz_notch

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"
#include "timers.h"
#include "stm32l4xx.h"
#include "stm32l432xx.h"
#include <stdbool.h>
#include "lib_ee152.h"

//****************************************************
// Biquad filtering.
//****************************************************

// All DSP filters need state.
struct Biquadstate { float x_nm1, x_nm2, y_nm1, y_nm2; };

#define N_BIQUAD_SECS 2

// A Channel holds the state for a single sEMG input.
struct Channel {
    enum Pin input_pin;		// Which ADC input pin drives this channel.
    enum Pin drive_out_pin;	// Which pin to drive out on, if any.

    // Our ADC inputs are 12-bit unsigned. Filtering may change that a bit.
    uint16_t samples[N_DATA_SAMPLES];

    // State for the biquad filter.
    struct Biquadstate biquad_state[N_BIQUAD_SECS];
};

static struct Channel g_channels[N_ADC_CHANNELS];

// The difference equation is
//	yn = b0*xn + b1*x_nm1 + b2*x_nm2 - a1*y_nm1 - a2*y_nm2 (assuming
//	a0=1).
// In the Z domain, this is the filter
//	Y(z)/X(z) = (b0 + b1/z + b2/z**2) / (1 + a1/z + a2/z**2)
struct Biquadcoeffs {	// The coefficients of a single biquad section.
    float b0, b1, b2,	// Numerator
	  a0, a1, a2;	// Denominator; assume a0=1 and ignore it.
};

// 50Hz 4-pole lowpass.
static const struct Biquadcoeffs biquad_lowpass_50[2] = {
	{.00482434,  .00964869,  .00482434,  1, -1.04859958,  .29614036},
	{ 1,         2,          1,          1, -1.32091343,  .63273879}};
// [58,62]Hz 2nd-order notch filter. A 2nd-order notch filter has two 2nd-order
// sections => 4 poles (a 2nd-order lowpass would have just one section).
static const struct Biquadcoeffs biquad_60Hz_notch[2] = {
	{.96508099, -1.40747202, .96508099,  1., -1.40810535, .96443153},
	{1.,        -1.45839783, 1.,         1., -1.45687509, .96573127}};

// Initialize a channel: assign a pin to it, and zero out its state.
static void init_channel (struct Channel *c, enum Pin input_pin) {
    c->input_pin = input_pin;
    c->drive_out_pin = NO_PIN;

    for (int i=0; i<N_BIQUAD_SECS; ++i)
	c->biquad_state[i] = (struct Biquadstate) {0,0,0,0};
}

// Biquad filtering routine.
// - The input xn is assumed to be a float in the range [0,1).
// - Compute yn = b0*xn + b1*x_nm1 + b2*x_nm2 - a1*y_nm1 - a2*y_nm2 (assuming
//   a0=1).
// - Update x_nm1->x_nm2, xn->x_nm1, y_nm1->y_nm2, yn->y_nm1
// - Return yn as a float.
float biquad_filter (const struct Biquadcoeffs *const coeffs,
		     struct Biquadstate *state, float xn) {
    float yn = coeffs->b0*xn + coeffs->b1*state->x_nm1 + coeffs->b2*state->x_nm2
	     - coeffs->a1*state->y_nm1 - coeffs->a2*state->y_nm2; // output

    state->x_nm2 = state->x_nm1;
    state->x_nm1 = xn;
    state->y_nm2 = state->y_nm1;
    state->y_nm1 = yn;

    return (yn);
}

//****************************************************
// UART support.
//****************************************************

// Used in MOBA_MODE to print the ECG values in ascii to the terminal.
#define MAX_DIGITS 6
static char *int_to_string (int val) {
    static char buf[MAX_DIGITS+1];

    int pos = MAX_DIGITS;	// rightmost position.
    buf [MAX_DIGITS] = '\0';

    while ((val>0) && (--pos >= 0)) {
	int digit = val % 10;
	val /= 10;
	buf[pos] = digit + '0';
    }
    if (pos==MAX_DIGITS) 	// Special case the number 0;
	buf[--pos]='0';		// else it returns an empty string.
    return (&buf[pos]);
}

// Set by task_input_loop(), read by task_UART_write().
static volatile int g_n_samples_taken=0;

// The main UART task. Task_input_loop() fills up g_channels[*].samples[].
// We try to keep up with it, sending the data to the host. For soft-scope mode,
// the filling goes forever, turning .samples[] into a circular buffer.
void task_UART_write (void *pvParameters) {
    int n_lines_printed = 0;
    uint16_t samples[2]={0,0};		// For soft-scope mode.

    while (1) {
	if (n_lines_printed != g_n_samples_taken) {
	    if (MOBA_MODE) {
		for (int ch=0; ch<N_ADC_CHANNELS; ++ch) {
		    int sample = g_channels[ch].samples[n_lines_printed];
		    serial_write (USART2, int_to_string (sample));
		    serial_write(USART2, ", ");
		}
		serial_write (USART2, "\n\r");
	    } else {
	        for (int ch=0; ch<N_ADC_CHANNELS; ++ch)
		    samples[ch] = g_channels[ch].samples[n_lines_printed];
		UART_send_to_host (samples[0]/4096.0, samples[1]/4096.0);
	    }
	    ++n_lines_printed;
	}
	// In soft-scope mode, wrap around to do it again.
	if ((n_lines_printed == N_DATA_SAMPLES) && !MOBA_MODE)
	    n_lines_printed=0;	
    }
}

//****************************************************
// The main input loop.
//****************************************************

void task_input_loop (void *pvParameters) {
    // In soft-scope mode, we *always* send data to the scope/UART. But in MOBA
    // mode, we don't do so until the user types 'r'.
    bool send_to_UART = !MOBA_MODE;
    int LED_on=0, LED_cycles_since_flip=0;
    // The green LED is at Nano D13, or PB3.
    pinMode(D13, "OUTPUT");

    while (1) {		// The "forever" loop, consuming input.
	// In MOBA mode, we don't send anything to the UART until the user hits
	// "r" for "record."
	if (!send_to_UART && UART_has_input_data(USART2)
		&& (serial_read(USART2)=='r')) {
	    send_to_UART = 1;
	}
	// Blink the LED to show we're still alive. Always, always.
	if (++LED_cycles_since_flip == 500) {
	    LED_on = !LED_on;
	    digitalWrite (D13, LED_on);
	    LED_cycles_since_flip=0;
	}
	for (int ch=0; ch < N_ADC_CHANNELS; ++ch) {
	    int samp = analogRead (g_channels[ch].input_pin);
	    //int samp = analogReadFromFile (ch,N_ADC_CHANNELS);

#ifdef FILTER
	    // The filter works with floats, but the rest of this file works
	    // with 12-bit ints. So convert to/from floats here.
	    float filtered = ((float)samp) / (1<<12);
	    for (int s=0; s<N_BIQUAD_SECS; ++s)
		filtered=biquad_filter (&FILTER[s],
				     &g_channels[ch].biquad_state[s], filtered);
	    samp = filtered * (1<<12);
#endif

	    // Drive out the nicely-filtered signal if desired.
	    if (g_channels[ch].drive_out_pin != NO_PIN)
		analogWrite (g_channels[ch].drive_out_pin, samp>>4);

            if (send_to_UART) {
	        g_channels[ch].samples[g_n_samples_taken] = samp;
	        g_n_samples_taken += (ch+1==N_ADC_CHANNELS);
		if (!MOBA_MODE && (g_n_samples_taken==N_DATA_SAMPLES))
		    g_n_samples_taken=0; // Make .samples[] a circular buffer.
		if (g_n_samples_taken==N_DATA_SAMPLES) //In MOBA mode, and done
		    send_to_UART = 0;
	    }
	} // for each channel
	vTaskDelay(SAMPLE_DELAY);
    }	// for each sample
}

int main(void){
    clock_setup_80MHz();		// 80 MHz, AHB and APH1/2 prescale=1x

    serial_begin (USART2,100000);

    // Dummy read of any channel enables ADC so we pay this cost up front.
    analogRead (A0);

    int drive_canned_ECG=0;

    // Channel 1 is always present; it always drives A3.
    init_channel (&g_channels[0], A0);
    g_channels[0].drive_out_pin = A3;
    // Channel 2 may not be present.
    if (N_ADC_CHANNELS==2) {
	init_channel (&g_channels[1], A1);
	g_channels[1].drive_out_pin = A4;
    }

    // All control-mode has been taken. Start FreeRTOS, so that we get nice
    // output to the DACs.
    if (MOBA_MODE)
        serial_write (USART2, "Type the letter 'r' to start recording: ");

    // Create tasks.
    TaskHandle_t task_handle_UART = NULL;
    BaseType_t status = xTaskCreate (
	    task_UART_write, "Write data to the UART",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY+1, // priority
	    &task_handle_UART);
    if (status != pdPASS)
	for ( ;; );

    TaskHandle_t task_handle_input_loop = NULL;
    status = xTaskCreate (
	    task_input_loop, "Take ADC samples",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY+3, // priority
	    &task_handle_input_loop);
    if (status != pdPASS)
	for ( ;; );

    vTaskStartScheduler();
}
