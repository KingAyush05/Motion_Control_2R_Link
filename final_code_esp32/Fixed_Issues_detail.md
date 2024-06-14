# Details of the changes made in the final ESP32 code

<font size="4">
Errors facing initially:
<ul>
<li>Guru Meditation Error: Core  1 panic'ed (LoadProhibited). Exception was unhandled.
<li>For high speeds, the ESP32 was rebooting due to access of illegal memory
<li>Also for low speeds of motor, there was still the same issue but after some time interval.
</ul>
 
Due to these errors the angle rotated by the motor was always going back to zero.
 
Changes done to fix the issues:
<ul>
<li>Added portMUX_TYPE mux which is used to ensure only one task or interrupt can access the resource at a time.
<li>Used portENTER_CRITICAl & portEXIT_CRITICAL so that the critcal code is note interrupted.
<li>Used a local copy of counter in the main loop to minimize the time spent in critical sections.
<li>Used IRAM_ATTR for the ISR function. This tells the compiler to place the specific function into the Internal RAM(IRAM) instead of flash memory
</ul>

The line portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; is part of the FreeRTOS framework, which is used in the ESP32 environment. This line of code initializes a mutex (mutual exclusion) lock, which is used to manage access to shared resources and ensure that only one task or interrupt can access the resource at a time.
<br>
Breakdown of the Components:
<ul>
<li><u>portMUX_TYPE</u>: This is a data type defined by FreeRTOS in the ESP32 environment. It represents a mutex or a critical section lock that is used to protect shared resources. A mutex ensures that only one task or interrupt can access the critical section of code at a time, preventing race conditions.
<li><u>mux</u>:This is the variable name for the mutex you are creating. You can name it whatever you like, but mux is commonly used for brevity.
<li><u>portMUX_INITIALIZER_UNLOCKED</u>: This macro initializes the mux variable in an unlocked state, meaning that the mutex is initially available.
It sets up the mutex so that it is ready to be used in critical sections or ISRs (Interrupt Service Routines).
</li></ul>
<br>
Why Use IRAM_ATTR?
<ul>
<li><u>Speed</u>: The internal RAM is faster than the external flash memory. Functions placed in IRAM execute more quickly because the CPU can access them directly without the need for fetching them from slower flash memory.

<li><u>Reliability</u>: During an interrupt, the system needs to execute the ISR immediately. If the ISR is stored in flash memory, there could be a delay in fetching the code, especially if the flash memory is in power-down mode or busy with other operations. Storing the ISR in IRAM avoids this potential delay.

<li><u>Interrupt Context</u>: ISRs are time-critical functions. Placing them in IRAM ensures they run with minimal latency, which is crucial for handling interrupts promptly.

</ul>
</font>
