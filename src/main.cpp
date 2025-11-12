#include <stdio.h>

#include <climits>

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"

#include "motors.hpp"
#include "pid.hpp"
#include "utils.hpp"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <rmw_microros/rmw_microros.h>

#include "pico_stdio_transport.h"

#define INT_PIN 14
#define SDA_PIN 12
#define SCL_PIN 13

// LIDAR DMA Buffer Structure
#define LIDAR_BUFFER_SIZE 1024u
#define LIDAR_PACKET_SIZE 22u

static uint8_t lidar_dma_buffer[LIDAR_BUFFER_SIZE] __attribute__((aligned(LIDAR_BUFFER_SIZE)));
static volatile size_t lidar_write_pos = 0;
static volatile size_t lidar_read_pos = 0;
static int lidar_dma_chan_a;
static int lidar_dma_chan_b;

// RTOS flags
bool stop_publishers = false;
bool error_detected = false;

// RTOS Tasks
TaskHandle_t main_task_handle = nullptr;
TaskHandle_t imu_task_handle = nullptr;
TaskHandle_t lidar_task_handle = nullptr;
TaskHandle_t tilt_sweep_task_handle = nullptr;

// RTOS Semaphores
SemaphoreHandle_t lidar_task_semaphore = nullptr;

// micro-ROS publishers
rcl_publisher_t statistics_publisher{};
rcl_publisher_t lidar_scan_publisher{};
rcl_publisher_t lidar_rpm_publisher{};

// Main micro-ROS communication task
void main_task(__unused void *params) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_node_t node{};
    rclc_support_t support{};
    rclc_executor_t executor{};

    // micro-ROS timers
    rcl_timer_t statistics_timer{};

    while (true) {
        stop_publishers = false;
        error_detected = false;

        // synchronize with the micro-ROS agent
        constexpr const int timeout_ms = 1000;
        constexpr const uint8_t attempts = 10;
        if (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK) {
            continue;
        }

        // start micro-ROS executor
        rclc_support_init(&support, 0, NULL, &allocator);
        rclc_node_init_default(&node, "firmware", "", &support);
        rclc_publisher_init_default(&statistics_publisher, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "firmware_runtime_stats");
        rclc_publisher_init_default(&lidar_rpm_publisher, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "laser_rpm");
        rclc_publisher_init_default(&lidar_scan_publisher, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "laser_scan");
        rclc_timer_init_default2(&statistics_timer, &support, RCL_MS_TO_NS(2500),
                [] (rcl_timer_t *timer, int64_t last_call_time) {
            char runtime_stats[1024];
            std_msgs__msg__String runtime_stats_msg = {
                .data = {
                    .data = runtime_stats,
                    .size = 0,
                    .capacity = sizeof runtime_stats
                }
            };

            // publish the FreeRTOS runtime statistics
            vTaskGetRunTimeStatistics(runtime_stats, sizeof runtime_stats);
            runtime_stats_msg.data.size = strnlen(runtime_stats, sizeof runtime_stats);

            auto rc = rcl_publish(&statistics_publisher, &runtime_stats_msg, NULL);
            if (rc != RCL_RET_OK) {
                error_detected = true;
            }
        }, true);
        rclc_executor_init(&executor, &support.context, 10, &allocator);
        rclc_executor_add_timer(&executor, &statistics_timer);

        // start the lidar task
        xSemaphoreGive(lidar_task_semaphore);

        // spin the executor
        while (!error_detected) {
            auto rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
                error_detected = true;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // stop the lidar task
        stop_publishers = true;
        while (xSemaphoreTake(lidar_task_semaphore, portMAX_DELAY) != pdTRUE) {
            // nothing to do
        }

        // stop micro-ROS executor
        rcl_timer_fini(&statistics_timer);
        rcl_publisher_fini(&lidar_rpm_publisher, &node);
        rcl_publisher_fini(&lidar_scan_publisher, &node);
        rcl_publisher_fini(&statistics_publisher, &node);
        rclc_executor_fini(&executor);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
    }
}

// Handle IMU data
void imu_task(__unused void *params) {
    // initialize I2C1 bus for the MPU6050
    auto rc = i2c_dma_init(&I2Cdev::i2c_dma, i2c0, 400 * 1000, SDA_PIN, SCL_PIN);
    if (rc != PICO_OK) {
        //printf_guarded("failed to initialize I2C\n");
        vTaskDelete(NULL);
    }

    // mpu initialization
    MPU6050 mpu;
    mpu.initialize();
    if (mpu.dmpInitialize()) {
        //printf_guarded("failed to initialize MPU6050 DMP\n");
        vTaskDelete(NULL);
    }

    // TODO: calibrate IMU
    mpu.setXAccelOffset(-387);
    mpu.setYAccelOffset(1135);
    mpu.setZAccelOffset(975);
    mpu.setXGyroOffset(106);
    mpu.setYGyroOffset(18);
    mpu.setZGyroOffset(7);

    mpu.setDMPEnabled(true);
    auto mpuIntStatus = mpu.getIntStatus();
    auto packetSize = mpu.dmpGetFIFOPacketSize();

    while (true) {
        auto fifoCount = mpu.getFIFOCount();
        if (fifoCount < packetSize) {
            // wait until IMU interrupt has been triggered
            ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        }

        // check for packets from the IMU
        fifoCount = mpu.getFIFOCount();
        if ((mpuIntStatus & 0x10) || fifoCount == 1024)  {
            mpu.resetFIFO();
            continue;
        } else if (fifoCount < packetSize) {
            continue;
        }

        // read the fifo
        uint8_t fifoBuffer[64];
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // convert to euler angles (in degrees)
        Quaternion q;
        VectorFloat gravity;
        //vec3 euler;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        //mpu.dmpGetYawPitchRoll(euler.d, &q, &gravity);

        //euler.yaw = euler.yaw * 180.f / PI;
        //euler.pitch = euler.pitch * 180.f / PI;
        //euler.roll = euler.roll * 180.f / PI;
    }
}

size_t lidar_buffer_available() {
    return (lidar_write_pos - lidar_read_pos) & (LIDAR_BUFFER_SIZE - 1);
}

uint8_t lidar_buffer_peek(size_t offset) {
    return lidar_dma_buffer[(lidar_read_pos + offset) & (LIDAR_BUFFER_SIZE - 1)];
}

uint8_t lidar_buffer_read() {
    auto b = lidar_dma_buffer[lidar_read_pos];
    lidar_read_pos = (lidar_read_pos + 1) & (LIDAR_BUFFER_SIZE - 1);
    return b;
}

void lidar_buffer_await(size_t length) {
    while (lidar_buffer_available() < length) {
        xTaskNotifyWait(0, ULONG_MAX, nullptr, portMAX_DELAY);
    }
}

uint16_t lidar_checksum(unsigned char *buffer) {
    uint32_t checksum = 0;
    for(int i = 0; i < 10; i++) {
        uint16_t word = buffer[2*i+1];
        word <<= 8;
        word |= buffer[2*i];
        checksum = (checksum << 1) + word;
    }

    checksum = (checksum & 0x7fff) + (checksum >> 15);
    return checksum & 0x7fff;
}

bool lidar_get_packet(uint8_t *buffer) {
    lidar_buffer_await(22);
    for (int i = 0; i < 22; i++) {
        buffer[i] = lidar_buffer_read();
    }

    uint16_t checksum = buffer[21];
    checksum <<= 8;
    checksum |= buffer[20];
    return checksum == lidar_checksum(buffer);
}

void lidar_synchronize() {
    while (true) {
        // detect the header for the last packet of a scan
        lidar_buffer_await(2);
        if (lidar_buffer_peek(0) != 0xFA || lidar_buffer_peek(1) != 0xF9) {
            lidar_buffer_read();
            continue;
        }

        // if we can successfully decode a packet, we're synchronized
        uint8_t buffer[22];
        if (lidar_get_packet(buffer)) {
            break;
        }
    }
}

struct lidar_ray {
    uint16_t distance : 14;
    bool strength_warning : 1;
    bool invalid_warning : 1;
    uint16_t intensity;
};

void lidar_task(__unused void* param) {
    // LiDAR scan storage
    sensor_msgs__msg__LaserScan lidar_scan_msg;
    lidar_scan_msg.angle_min = 0;
    lidar_scan_msg.angle_max = 2 * PI;
    lidar_scan_msg.angle_increment = PI / 180.f;
    lidar_scan_msg.time_increment = (1.f / 5.f) / 360.f;
    lidar_scan_msg.scan_time = (1.f / 5.f);
    lidar_scan_msg.range_min = 0.15f;
    lidar_scan_msg.range_max = 6.0f;

    char *frame_id = strdup("map");
    lidar_scan_msg.header.frame_id.data = frame_id;
    lidar_scan_msg.header.frame_id.size = 3;
    lidar_scan_msg.header.frame_id.capacity = 3;

    float scans[360];
    lidar_scan_msg.ranges.data = scans;
    lidar_scan_msg.ranges.size = 360;
    lidar_scan_msg.ranges.capacity = 360;

    float intensities[360];
    lidar_scan_msg.intensities.data = intensities;
    lidar_scan_msg.intensities.size = 360;
    lidar_scan_msg.intensities.capacity = 360;

    while (true) {
        constexpr float lidar_Kp = 0.02f;
        constexpr float lidar_Ki = 0.01f;
        constexpr float lidar_Kd = 0.000f;

        PositionalPID lidar_pid(lidar_Kp, lidar_Ki, lidar_Kd);
        lidar_pid.set_setpoint(300.f);
        set_motor_lidar_speed(0.0f);

        // wait for micro-ROS to start
        while (xSemaphoreTake(lidar_task_semaphore, portMAX_DELAY) != pdTRUE);
        set_motor_lidar_speed(0.2f);
        lidar_read_pos = 0;
        lidar_write_pos = 0;

        // LiDAR packet loop
        int index = 0;
        while (!stop_publishers && !error_detected) {
            uint8_t buffer[22];
            if (!lidar_get_packet(buffer)) {
                lidar_synchronize();
                index = 0;
                continue;
            }

            // LiDAR rpm control
            uint16_t rpm_raw = (buffer[2] + (buffer[3] << 8));
            float rpm = (float) rpm_raw / 64.f;
            float response = lidar_pid.compute(rpm, 1.f / 450.f, nullptr, nullptr, nullptr,
                    [] (float a, float b) {return b - a;});
            set_motor_lidar_speed(response);

            // header timestamp is acquisition time of first ray in scan
            if (index == 0) {
                auto now = time_us_64();
                lidar_scan_msg.header.stamp.sec = now / 1000000;
                lidar_scan_msg.header.stamp.nanosec = (now % 1000000) * 1000;
            }

            // Compute distances
            lidar_ray *rays = reinterpret_cast<lidar_ray*>(&buffer[4]);
            for (int j = 0; j < 4; j++) {
                scans[index + j] = ((float) rays[j].distance) / 1000.f;
                intensities[index + j] = rays[j].intensity;
            }

            // Publish message when scan is complete
            index += 4;
            if (index >= 360) {
                index = 0;

                // publish laser scan
                if (rpm > 280.f && rpm < 320.f) {
                    std_msgs__msg__Float32 rpm_msg = { .data = rpm };
                    auto rc = rcl_publish(&lidar_rpm_publisher, &rpm_msg, NULL);
                    if (rc != RCL_RET_OK) {
                        error_detected = true;
                        break;
                    }

                    rc = rcl_publish(&lidar_scan_publisher, &lidar_scan_msg, NULL);
                    if (rc != RCL_RET_OK) {
                        error_detected = true;
                    }
                }
            }
        }

        // shut down lidar
        set_motor_lidar_speed(0.0f);
        xSemaphoreGive(lidar_task_semaphore);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void tilt_sweep_task(__unused void* param) {
    while (1) {
        for (float i = -1.f; i <= 1.f; i += 0.001f) {
            set_motor_tilt_position(i);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        for (float i = 1.f; i >= -1.f; i -= 0.001f) {
            set_motor_tilt_position(i);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

void setup_imu_interrupt() {
    gpio_init(INT_PIN);
    gpio_set_input_enabled(INT_PIN, true);
    gpio_pull_up(INT_PIN);

    // subscribe to imu interrupt
    gpio_add_raw_irq_handler(INT_PIN, [] () {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (gpio_get_irq_event_mask(INT_PIN) & GPIO_IRQ_EDGE_FALL) {
            gpio_acknowledge_irq(INT_PIN, GPIO_IRQ_EDGE_FALL);
            vTaskNotifyGiveFromISR(imu_task_handle, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    });
    gpio_set_irq_enabled(INT_PIN, GPIO_IRQ_EDGE_FALL, true);
}

void setup_lidar() {
    // start uart0
    gpio_set_function(17, UART_FUNCSEL_NUM(uart0, 17));
    uart_init(uart0, 115200);

    // setup a dma-powered ring buffer with chained DMA channels
    // TODO: hunt the starting packet and synchronize (otherwise we're off by <= 1 packet interval)
    lidar_dma_chan_a = dma_claim_unused_channel(true);
    lidar_dma_chan_b = dma_claim_unused_channel(true);

    dma_channel_config config_a = dma_channel_get_default_config(lidar_dma_chan_a);
    channel_config_set_transfer_data_size(&config_a, DMA_SIZE_8);
    channel_config_set_read_increment(&config_a, false);
    channel_config_set_write_increment(&config_a, true);
    channel_config_set_dreq(&config_a, uart_get_dreq_num(uart0, false));
    channel_config_set_ring(&config_a, true, 10);
    channel_config_set_chain_to(&config_a, lidar_dma_chan_b);

    dma_channel_config config_b = dma_channel_get_default_config(lidar_dma_chan_b);
    channel_config_set_transfer_data_size(&config_b, DMA_SIZE_8);
    channel_config_set_read_increment(&config_b, false);
    channel_config_set_write_increment(&config_b, true);
    channel_config_set_dreq(&config_b, uart_get_dreq_num(uart0, false));
    channel_config_set_ring(&config_b, true, 10);
    channel_config_set_chain_to(&config_b, lidar_dma_chan_a);

    dma_channel_configure(lidar_dma_chan_a, &config_a, (void*) &lidar_dma_buffer[0],
            &uart_get_hw(uart0)->dr, LIDAR_PACKET_SIZE, false);
    dma_channel_configure(lidar_dma_chan_b, &config_b, (void*) &lidar_dma_buffer[LIDAR_PACKET_SIZE],
            &uart_get_hw(uart0)->dr, LIDAR_PACKET_SIZE, false);
    dma_channel_set_irq1_enabled(lidar_dma_chan_a, true);
    dma_channel_set_irq1_enabled(lidar_dma_chan_b, true);

    irq_set_exclusive_handler(DMA_IRQ_1, [] () {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (dma_irqn_get_channel_status(1, lidar_dma_chan_a)) {
            dma_irqn_acknowledge_channel(1, lidar_dma_chan_a);
            lidar_write_pos = (lidar_write_pos + LIDAR_PACKET_SIZE) & (LIDAR_BUFFER_SIZE - 1);
            const auto write_pos_next =
                    (lidar_write_pos + LIDAR_PACKET_SIZE) & (LIDAR_BUFFER_SIZE - 1);
            dma_channel_set_write_addr(lidar_dma_chan_a, &lidar_dma_buffer[write_pos_next], false);

            if (lidar_task_handle) {
                xTaskNotifyFromISR(lidar_task_handle, 0, eNoAction, &xHigherPriorityTaskWoken);
            }
        }

        if (dma_irqn_get_channel_status(1, lidar_dma_chan_b)) {
            dma_irqn_acknowledge_channel(1, lidar_dma_chan_b);
            lidar_write_pos = (lidar_write_pos + LIDAR_PACKET_SIZE) & (LIDAR_BUFFER_SIZE - 1);
            const auto write_pos_next =
                (lidar_write_pos + LIDAR_PACKET_SIZE) & (LIDAR_BUFFER_SIZE - 1);
            dma_channel_set_write_addr(lidar_dma_chan_b, &lidar_dma_buffer[write_pos_next], false);

            if (lidar_task_handle) {
                xTaskNotifyFromISR(lidar_task_handle, 0, eNoAction, &xHigherPriorityTaskWoken);
            }
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    });
    irq_set_enabled(DMA_IRQ_1, true);

    // kick off DMA transfers
    dma_channel_start(lidar_dma_chan_a);
}

void setup_microros() {
    rmw_uros_set_custom_transport(true, NULL, pico_stdio_transport_open,
        pico_stdio_transport_close, pico_stdio_transport_write, pico_stdio_transport_read
    );

    rcl_allocator_t allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate = [] (size_t size, void *) {
        return pvPortMalloc(size);
    };

    allocator.deallocate = [] (void * pointer, void *) {
        if (NULL != pointer){
            vPortFree(pointer);
        }
    };

    allocator.reallocate = [] (void * pointer, size_t size, void *) {
        if (NULL == pointer){
            return pvPortMalloc(size);
        } else {
            return pvPortRealloc(pointer,size);
        }
    };

    allocator.zero_allocate = [] (size_t number_of_elements, size_t size_of_element, void *) {
        return pvPortCalloc(number_of_elements, size_of_element);
    };

    if (!rcutils_set_default_allocator(&allocator)) {
        while (1) {
            tight_loop_contents();
        }
    }
}

extern "C" {

/* Static memory allocation for Idle task */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* Static memory allocation for Passive Idle task (if using SMP) */
static StaticTask_t xPassiveIdleTaskTCBBuffer;
static StackType_t xPassiveIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetPassiveIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                           StackType_t **ppxIdleTaskStackBuffer,
                                           uint32_t *pulIdleTaskStackSize,
                                           BaseType_t xCoreID)
{
    *ppxIdleTaskTCBBuffer = &xPassiveIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xPassiveIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
    *ppxTimerTaskStackBuffer = &xTimerStack[0];
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

} /* extern "C" */

int main(void) {
    stdio_init_all();

    // setup hardware
    setup_imu_interrupt();
    setup_encoders();
    setup_motors();
    setup_lidar();

    // setup micro-ROS
    setup_microros();

    // setup semaphores
    lidar_task_semaphore = xSemaphoreCreateBinary();

    // start tasks
    xTaskCreate(main_task, "Executor", 16384, nullptr, tskIDLE_PRIORITY + 1U, &main_task_handle);
    xTaskCreate(imu_task, "IMUThread", 8192, nullptr, tskIDLE_PRIORITY + 2U, &imu_task_handle);
    xTaskCreate(lidar_task, "LidarThread", 8192, nullptr, tskIDLE_PRIORITY + 2U, &lidar_task_handle);
    xTaskCreate(tilt_sweep_task, "TiltSweepThread", 8192, nullptr, tskIDLE_PRIORITY + 2U, &tilt_sweep_task_handle);
    vTaskStartScheduler();
    return 0;
}