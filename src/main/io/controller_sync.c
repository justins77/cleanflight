#include "io/controller_sync.h"

#include "io/serial.h"
#include "io/debug_console.h"

int sync_bytes_written = 0;
int sync_bytes_read = 0;

static serialPort_t *csyncPort;

void controllerSyncInit() {
  debugPrint("opening port\r\n");
  csyncPort = openSerialPort(SERIAL_PORT_USART2,
			     FUNCTION_VTX_SMARTAUDIO,  // TODO: use my own function name
			     NULL,
			     NULL,
			     115200,
			     MODE_RXTX,
			     0);
  debugPrintVar("csyncPort: ", (int)csyncPort);
}

extern int rx_cplt_count;

void controllerSyncUpdate() {
  if (!csyncPort) {
    return;
  }

  if (sync_bytes_written > 2000) {
    return;
  }

  while (serialRxBytesWaiting(csyncPort)) {
    int rx = serialRead(csyncPort);
    //debugPrintc(rx);
    sync_bytes_read++;
  }

  static int ds2 = 0;
  if (++ds2 >= 1) {
    ds2 = 0;
    static int out = 0;
    serialWrite(csyncPort, '0' + out);
    out = (out + 1) % 10;
    sync_bytes_written++;
  }

  static int downsample = 0;
  if (++downsample >= 100) {
    downsample = 0;
    debugPrint(" written: ");
    debugPrinti(sync_bytes_written);
    debugPrint(" read: ");
    debugPrinti(sync_bytes_read);
    debugPrint(" rx_callbacks: ");
    debugPrinti(rx_cplt_count);
    debugPrint("\r\n");
  }
}
