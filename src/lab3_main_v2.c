// // Include FreeRTOS headers.
// #include "FreeRTOSConfig.h"
// #include "FreeRTOS.h"
// #include "portmacro.h"
// #include "task.h"
// #include "timers.h"
// #include "queue.h"

// #include "stm32l432xx.h"
// #include "lib_ee152.h"

// #include <string.h>

// //*****************************************************
// // Our big-hammer debugging facility. It works with macros in FreeRTOSConfig.h.
// // - At each 1ms SysTick interrupt, the macro traceTASK_INCREMENT_TICK() appends
// //   the number 4 to g_debug_notes[].
// // - Whenever we context switch to task_writer1() -- whether it's triggered by a
// //   SysTick interrupt, by a task calling vTaskDelay(), or whatever -- we append
// //   the number 1 to g_debug_notes[]
// // - Similarly, switch to task_writer2() appends 2, switching to
// //   task_UART_daemon() appends 3, and switching to the idle task appends 0.
// // - Then, at the end of the run, task_UART_daemon() dump g_debug_notes[]:
// //   printing out "*" for each SysTick interrupt, "1" for task_writer1, "2" for
// //   task_writer2, "D" for the UART daemon and "I" for the idle task.
// //*****************************************************
// unsigned char g_debug_notes [MAX_DEBUG_NOTES];
// unsigned g_n_debug_notes;

// // Each writer task writes its variable to say that its done: task_UART_daemon
// // monitors these globals and, when both writers are done, prints out the debug
// // info.
// static volatile int g_writer1_done=0;
// static volatile int g_writer2_done=0;

// // The global queue -- 32 bytes (not 32 ints).
// QueueHandle_t g_queue;
// #define QUEUE_SIZE 32

// // Error-handling routine. Why bother with an error handler that doesn't print
// // its message? Well, it's hard to print an error message when we're trying to
// // debug the UART! Instead, we can turn on the green LED to show that there was
// // some kind of error (though remember we're also using the green LED to
// // indicate that both writers are done).
// // Or, we can run in the debugger. Then, either set a breakpoint on error_152()
// // or just manually stop the program when it hangs. Either way, the debugger
// // will show us that we're in error_152(), who called it, and what 'message' is.
// void error_152 (char *message) {
//     //digitalWrite (D13, 1);	// Turn the green LED on if desired.
//     while(1);
// }

// //*****************************************************
// // This part of the file has our core routines for writing to the screen.
// // - The main shared data structure is a FreeRTOS queue g_queue.
// // - Task_UART_daemon() continually loops: pull a character from the queue
// //   using a blocking read, then print that character using
// //   UART_write_byte_nonB().
// // - How do characters get into the queue? The two writer tasks (which will come
// //   later) each call serial_write_lab3(string). Serial_write_lab3() takes
// //   a string and writes it into g_queue. Internally, serial_write_lab3()
// //   merely does blocking queue writes of 1B until its string is used up.
// //   Note that serial_write_lab3() is a function and not a task. It doesn't
// //   return until it has found space to write its entire string into g_queue.
// //   But there can easily be two instance of serial_write() active, that can be
// //   swapped in and out by the scheduler.
// // - Note that serial_write_lab3() is a special version of the usual Arduino
// //   serial_write(), just for this lab. It's special because it works with
// //   g_queue.
// //
// // As opposed to lab3_main_v1.c, these versions of serial_write_lab3() and
// // task_UART_daemon() don't ever put themselves to sleep with vTaskDelay(1).
// // Instead, they just use blocking queue writes and reads to accomplish the
// // same goal.
// // This way is simpler to code and more in the spirit of queue -- and
// // interestingly, has a very different end result.
// //*****************************************************

// static void serial_write_lab3 (const char *user_buf) {
//     // Dump into the circular buffer.
//     for (const char *cp=user_buf; *cp; ++cp) {
// 	if (xQueueSend (g_queue,cp,5000) != pdPASS)
// 	    error_152 ("Queue write failed");
//     }
// }

// static void task_UART_daemon (void *pvParameters) {
//     void UART_write_byte(USART_TypeDef *USARTx, char data);
//     vTaskSetApplicationTaskTag (NULL, (void *)3);
//     char ch;
//     while (!g_writer1_done || !g_writer2_done) {
        // 	// Returns an error condition if the read times out.
        // 	if (xQueueReceive (g_queue, &ch, 5) == pdPASS)
        // 	    //error_152 ("Queue read timed out");
        // 	    UART_write_byte (USART2, ch);
//     }
    
//     // Both writers are done. Print the accumulated debug info and then spin.
//     digitalWrite (D13, 1);	// Turn the green LED on.
//     UART_write_byte (USART2, '\r');
//     UART_write_byte (USART2, '\n');
//     for (int i=0; i<g_n_debug_notes; ++i) {
// 	char c = "I12D*????????"[g_debug_notes[i]];
// 	UART_write_byte (USART2, c);
//     }
//     UART_write_byte (USART2, '\n');
//     UART_write_byte (USART2, '\r');
//     while (1);
// }

// //*****************************************************
// // The two writers.
// // Each one just loops, calling serial_write_lab3() to print its "signature"
// // string (task_writer1 prints "11111111" and task_writer2 prints "22222222").
// // In a perfect world, they will work together fairly to display
// //	11111111
// //	22222222
// //	11111111
// //	22222222
// // In real life, it's not that simple.
// //*****************************************************

// #define N_LOOPS 25
// static void task_writer1 (void *pvParameters) {
//     vTaskSetApplicationTaskTag (NULL, (void *)1);
//     char buf[] = "11111111\n\r";
//     serial_write_lab3 ("start writer 1\n\r");
//     for (int i=0; i<N_LOOPS; ++i) {
// 	serial_write_lab3 (buf);
//     }
//     serial_write_lab3("done writer 1\n\r");
//     g_writer1_done=1;
//     while (1);
// }

// static void task_writer2 (void *pvParameters) {
//     vTaskSetApplicationTaskTag (NULL, (void *)2);
//     char buf[] = "22222222\n\r";
//     serial_write_lab3 ("start writer 2\n\r");
//     for (int i=0; i<N_LOOPS; ++i) {
// 	serial_write_lab3 (buf);
//     }
//     serial_write_lab3("done writer 2\n\r");
//     g_writer2_done=1;
//     while (1);
// }

// int main(void){
//     clock_setup_80MHz();	// 80 MHz, AHB and APH1/2 prescale=1x

//     // The green LED is at Nano D13, or PB3.
//     pinMode(D13, "OUTPUT");
//     digitalWrite (D13, 0);

//     serial_begin (USART2, 9600);
//     serial_write (USART2, "In main()\r\n");

//     // Create the global queue. The "1" means an element size of 1B -- a char.
//     if ((g_queue=xQueueCreate(QUEUE_SIZE,1)) == NULL)
// 	error_152 ("Cannot create queue");

//     // Create tasks.

//     // First, the UART daemon. It continually looks at the UART buffer and
//     // prints anything in it.
//     TaskHandle_t task_handle_UART_daemon = NULL;
//     BaseType_t task_create_OK = xTaskCreate (
// 	    task_UART_daemon, "UART daemon",
// 	    100, // stack size in words
// 	    NULL, // parameter passed into task, e.g. "(void *) 1"
// 	    3+tskIDLE_PRIORITY, // priority
// 	    &task_handle_UART_daemon);
//     if (task_create_OK != pdPASS) for ( ;; );

//     // Next, writer task #1, that just writes.
//     TaskHandle_t task_handle_writer1 = NULL;
//     task_create_OK = xTaskCreate (
// 	    task_writer1, "UART writer #1",
// 	    100, // stack size in words
// 	    NULL, // parameter passed into task, e.g. "(void *) 1"
// 	    1+tskIDLE_PRIORITY, // priority
// 	    &task_handle_writer1);
//     if (task_create_OK != pdPASS) for ( ;; );

//     // Next, writer task #2, that just writes.
//     TaskHandle_t task_handle_writer2 = NULL;
//     task_create_OK = xTaskCreate (
// 	    task_writer2, "UART writer #2",
// 	    100, // stack size in words
// 	    NULL, // parameter passed into task, e.g. "(void *) 1"
// 	    1+tskIDLE_PRIORITY, // priority
// 	    &task_handle_writer2);
//     if (task_create_OK != pdPASS) for ( ;; );

//     vTaskStartScheduler();
// }