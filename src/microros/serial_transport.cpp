#include "microros/serial_transport.hpp"

#include <rmw_microros/custom_transport.h>
#include <uxr/client/transport.h>

extern "C" {

static bool serial_transport_open(struct uxrCustomTransport *transport) {
    mbed::UARTSerial *serial = (mbed::UARTSerial *)transport->args;
    serial->set_flow_control(mbed::SerialBase::Disabled);
    serial->set_blocking(false);
    return true;
}

static bool serial_transport_close(struct uxrCustomTransport *transport) {
    return true;
}

static size_t serial_transport_write(struct uxrCustomTransport *transport,
                                     const uint8_t *buf, size_t len,
                                     uint8_t *err) {
    mbed::UARTSerial *serial = (mbed::UARTSerial *)transport->args;
    return serial->write(buf, len);
}

static size_t serial_transport_read(struct uxrCustomTransport *transport,
                                    uint8_t *buf, size_t len, int timeout,
                                    uint8_t *err) {
    mbed::UARTSerial *serial = (mbed::UARTSerial *)transport->args;
    return serial->read(buf, len);
}
}

void set_microros_serial_transports(mbed::UARTSerial *serial) {
    rmw_uros_set_custom_transport(true, serial, serial_transport_open,
                                  serial_transport_close, serial_transport_write,
                                  serial_transport_read);
}