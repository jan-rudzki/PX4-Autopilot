#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>  // For atof()

#include <uORB/uORB.h>
#include <uORB/topics/debug_vect.h>
//#include <uORB/topics/debug_key_value.h>
//#include <uORB/topics/debug_array.h>

#include <drivers/drv_hrt.h>  // For hrt_absolute_time()


extern "C" __EXPORT int uart_interface_main(int argc, char *argv[]);

static int uart_fd = -1;
static const char *uart_device = "/dev/ttyS1";  // --> ttyS1 = USART2 = TELEM3 Replace with the UART port (ttySx)


int uart_interface_main(int argc, char *argv[]) {
    PX4_INFO("Starting UART driver");

    /*
    // Create a uORB publisher
    struct debug_key_value_s dbg_v;
    strncpy(dbg_v.key, "pcu_v", sizeof(dbg_v.key));
    dbg_v.value = 0.0f;
    orb_advert_t pub_dbg_v = orb_advertise(ORB_ID(debug_key_value), &dbg_v);

    // Create a uORB publisher for an array
    struct debug_array_s dbg_array;
	dbg_array.id = 1;
	strncpy(dbg_array.name, "dbg_array", 10);
	orb_advert_t pub_dbg_array = orb_advertise(ORB_ID(debug_array), &dbg_array);
    */

    uint64_t loc_timestamp = hrt_absolute_time();
    //uint64_t pcu_timestamp = 0;

    uint16_t raw_voltage = 0;
    uint16_t raw_current = 0;

    float voltage = 0.0f;
    float current = 0.0f;
    float power = 0.0f;

    const float voltage_gain = 0.004882813f;  // Voltage gain factor
    const float current_gain = 0.040283203f;  // Current gain factor

    const float voltage_offset = 30.0f;  // Voltage offset
    const float current_offset = -15.0f;  // Current offset

	struct debug_vect_s pcu_vect;
	strncpy(pcu_vect.name, "PCU", 10);
	pcu_vect.x = 0.0f;
	pcu_vect.y = 0.0f;
	pcu_vect.z = 0.0f;
	orb_advert_t pub_pcu_vect = orb_advertise(ORB_ID(debug_vect), &pcu_vect);

    // Open UART device
    uart_fd = open(uart_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd < 0) {
        PX4_ERR("Failed to open UART device %s", uart_device);
        return -1;
    }

    // Configure UART baud rate and settings
    struct termios uart_config;
    tcgetattr(uart_fd, &uart_config);

    cfsetispeed(&uart_config, B115200);  // Set baud rate (adjust as needed)
    cfsetospeed(&uart_config, B115200);

    uart_config.c_cflag |= (CLOCAL | CREAD);  // Enable receiver and set local mode
    uart_config.c_cflag &= ~CSIZE;
    uart_config.c_cflag |= CS8;               // 8 data bits
    uart_config.c_cflag &= ~PARENB;           // No parity
    uart_config.c_cflag &= ~CSTOPB;           // 1 stop bit

    tcsetattr(uart_fd, TCSANOW, &uart_config);

    // Main loop for reading data
    char buffer[128];
    while (true) {
        int n = read(uart_fd, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            loc_timestamp = hrt_absolute_time();  // Get timestamp in microseconds

            buffer[n] = '\0';  // Null-terminate the received data

            // Print the received data to console
            PX4_INFO("@TELEM3: %s", buffer);

            // buffer is of the form "%u - %u %u %u %u %u %u %u %u"
            // where the first %u is the timestamp and the rest are the raw ADC values
            // we are interested in FC voltage, LiPo current, and their product the power
            // FC voltage is the second raw ADC value, LiPo current is the third raw ADC value

            // split the buffer into tokens
            char *token = strtok(buffer, " -");
            int value_counter = 0;

            while (token != NULL) {
                if (value_counter == 0) { // PCU timestamp
                    //pcu_timestamp = strtol(token, NULL, 10);
                    //PX4_INFO("PCU timestamp: %u", pcu_timestamp);
                }
                else if (value_counter == 2) { // FC voltage
                    raw_voltage = atoi(token);
                    //raw_voltage = strtol(token, NULL, 10);
                    //PX4_INFO("Raw FC-voltage: %u", raw_voltage);
                } else if (value_counter == 3) { // LiPo current
                    raw_current = atoi(token);
                    //raw_current = strtol(token, NULL, 10);
                    //PX4_INFO("Raw LiPo-current: %u", raw_current);

                    break; // Leave the loop
                }
                token = strtok(NULL, " -");
                value_counter++;
            }

            // Convert raw ADC values to physical values
            voltage = (float)raw_voltage * voltage_gain + voltage_offset;
            current = (float)raw_current * current_gain + current_offset;
            power = voltage * current;

            /*
            // Publish it via uORB --> Mavlink message
            float buffer_value = atof(buffer); // Convert the received data to a float

            dbg_v.timestamp = timestamp;
            dbg_v.value = buffer_value;
            orb_publish(ORB_ID(debug_key_value), pub_dbg_v, &dbg_v);

            for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
                dbg_array.data[i] = i * 0.01f;
            }

            dbg_array.timestamp = timestamp;
            orb_publish(ORB_ID(debug_array), pub_dbg_array, &dbg_array);
            */

            /* send one vector */
            pcu_vect.x = voltage;
            pcu_vect.y = current;
            pcu_vect.z = power;
            pcu_vect.timestamp = loc_timestamp;
            orb_publish(ORB_ID(debug_vect), pub_pcu_vect, &pcu_vect);
        }
        usleep(100000);  // Sleep for 100 ms
    }

    close(uart_fd);
    return 0;
}
