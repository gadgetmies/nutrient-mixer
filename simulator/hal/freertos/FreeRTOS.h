// Critical sections. Interrupt handlers run on the firmware thread in the simulator (between API calls), so
// they can never interrupt the code that holds a critical section and these need not do anything.
#pragma once
typedef struct {
  int owner;
} portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
#define portENTER_CRITICAL(mux) ((void)(mux))
#define portEXIT_CRITICAL(mux) ((void)(mux))
#define portENTER_CRITICAL_ISR(mux) ((void)(mux))
#define portEXIT_CRITICAL_ISR(mux) ((void)(mux))
#define portTICK_PERIOD_MS 1
