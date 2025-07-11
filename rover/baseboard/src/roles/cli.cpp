#include "./cli.h"
#include "./roles.h"

static CLI * selfCli = nullptr;

CLI::CLI() {
    selfCli = this;

    leds.setup();
    leds.status_led.blink3();

    USBSerial.begin(115200);

    // Start the CLI task
    xTaskCreate(
        cliTask,
        "cliTask",
        2048,
        this,
        1,
        NULL
    );
};

void CLI::loop() {

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1) {
        xTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);

        if (USBSerial.available()) {
            char command = USBSerial.read();
            switch (command) {
                case 'h':
                    USBSerial.println("Available commands:");
                    USBSerial.println("  h - Show this help");
                    USBSerial.println("  b - select base role");
                    USBSerial.println("  r - select rover role");
                    break;
                case 'b':
                    nvs.begin("core", false);
                    nvs.putInt("role", ROLE_BASESTATION);
                    nvs.end();
                    USBSerial.println("BASE mode selected.");
                    break;
                case 'r':
                    nvs.begin("core", false);
                    nvs.putInt("role", ROLE_ROVER);
                    nvs.end();
                    USBSerial.println("ROVER mode selected.");
                    break;
                default:
                    USBSerial.printf("Unknown command: %c\n", command);
            }
        }
    }
}
