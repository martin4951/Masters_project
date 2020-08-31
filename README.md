# Masters_project



Master MCU code can be found under: "MasterMCU_code" - This is the scheduler code for distributing and migrating tasks based on the system tick timer (TIM2) and the feedback from the processors.

Slave processor code can be found under: "Processor_code" - This is simple code for accepting tasks from master MCU after a SPI based interrupt, running the tasks and returning to sleep mode.
