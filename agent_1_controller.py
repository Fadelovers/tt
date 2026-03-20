#include <modbus/modbus.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    const char *device = "/tmp/ttyV0";
    int baud = 19200;
    char parity = 'E';
    int data_bits = 8;
    int stop_bits = 1;
    int slave_id = 14;

    printf("[SERVER] start\n");
    printf("[SERVER] device=%s baud=%d parity=%c data_bits=%d stop_bits=%d slave=%d\n",
           device, baud, parity, data_bits, stop_bits, slave_id);

    modbus_t *ctx = modbus_new_rtu(device, baud, parity, data_bits, stop_bits);
    if (ctx == NULL) {
        fprintf(stderr, "[SERVER] modbus_new_rtu failed\n");
        return 1;
    }

    if (modbus_set_slave(ctx, slave_id) == -1) {
        fprintf(stderr, "[SERVER] modbus_set_slave failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return 1;
    }

    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "[SERVER] modbus_connect failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return 1;
    }

    printf("[SERVER] connected successfully\n");

    modbus_mapping_t *mb_mapping = modbus_mapping_new(0, 0, 100, 0);
    if (mb_mapping == NULL) {
        fprintf(stderr, "[SERVER] modbus_mapping_new failed\n");
        modbus_close(ctx);
        modbus_free(ctx);
        return 1;
    }

    for (int i = 0; i < 100; i++) {
        mb_mapping->tab_registers[i] = i;
    }

    printf("[SERVER] holding registers initialized\n");

    uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];

    while (1) {
        printf("[SERVER] waiting for request...\n");

        int rc = modbus_receive(ctx, query);
        if (rc == -1) {
            fprintf(stderr, "[SERVER] modbus_receive error: %s\n", modbus_strerror(errno));
            continue;
        }

        printf("[SERVER] request received, size=%d bytes\n", rc);
        printf("[SERVER] raw query: ");
        for (int i = 0; i < rc; i++) {
            printf("%02X ", query[i]);
        }
        printf("\n");

        int func = query[1];
        int addr = (query[2] << 8) | query[3];
        int count = (query[4] << 8) | query[5];

        printf("[SERVER] function=0x%02X addr=%d count=%d\n", func, addr, count);

        int reply_rc = modbus_reply(ctx, query, rc, mb_mapping);
        if (reply_rc == -1) {
            fprintf(stderr, "[SERVER] modbus_reply failed: %s\n", modbus_strerror(errno));
        } else {
            printf("[SERVER] reply sent successfully\n");
        }
    }

    modbus_mapping_free(mb_mapping);
    modbus_close(ctx);
    modbus_free(ctx);
    return 0;
