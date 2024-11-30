#include <can2040.h>
#include <hardware/regs/intctrl.h>
#include <stdio.h>
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include <queue.h>

#define MAIN_TASK_PRIORITY      ( tskIDLE_PRIORITY + 1UL )
#define MAIN_TASK_STACK_SIZE configMINIMAL_STACK_SIZE

static struct can2040 cbus;

QueueHandle_t msgs;

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    xQueueSendToBack(msgs, msg, portMAX_DELAY);
}

static void PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

void canbus_setup(void)
{
    uint32_t pio_num = 0;
    uint32_t sys_clock = 125000000, bitrate = 500000;
    uint32_t gpio_rx = 4, gpio_tx = 5;

    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
    irq_set_priority(PIO0_IRQ_0, PICO_DEFAULT_IRQ_PRIORITY - 1);
    irq_set_enabled(PIO0_IRQ_0, 1);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}

void receive_task(__unused void *params) {
    struct can2040_msg data;

    while (1) {
        xQueueReceive(msgs, &data, portMAX_DELAY);
        printf("WE GOT A MESSAGE");
    }
}

void send_string (struct can2040* cd, char* str, size_t len) {
    struct can2040_msg msg;
    msg.id = 0x1;
    msg.data[0] = str[0];
    msg.dlc = 1;

    if (can2040_check_transmit(cd)) {
        can2040_transmit(cd, &msg);
    }
}

void send_task(__unused void *params) {
    while (1) {
        vTaskDelay(5000*portTICK_PERIOD_MS);
        send_string(&cbus, "HELLO WORLD\n", 12);
    }
};

int main( void )
{
    stdio_init_all();
    const char *rtos_name;
    rtos_name = "FreeRTOS";

    xQueueCreate(100,sizeof(struct can2040_msg));

    canbus_setup();

    TaskHandle_t task;
    xTaskCreate(receive_task, "MainThread",
                MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, &task);
    vTaskStartScheduler();
    
    return 0;
}