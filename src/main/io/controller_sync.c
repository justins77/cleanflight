#include "io/controller_sync.h"

#include <string.h>
#include "platform.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"

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

  // send a few bytes to intentionally misalign buffer
  for (int i=0; i<4; i++) {
    serialWrite(csyncPort, 'x');
  }
}

int rx_cplt_count = 0;
int validFrameCount = 0;
int frameErrorCount = 0;
int framesSentCount = 0;


extern uartPort_t* tmpDmaUartPort;

#define kFrameSize 8
#define kNumFrames 3
const int kTotalBufferSize = kFrameSize * kNumFrames;
const uint8_t kStartByte = 0xAA;

volatile int currentReadFrame = 0;
volatile int currentReadFrameStart = 0;

volatile int gapStart = -1;
volatile int gapEnd = -1;

volatile int requestedResyncBytes = -1;

// populated on the first callback
volatile uint8_t* buffer = NULL;

void printFrame(volatile uint8_t* frameBuffer) {
  for (int i=0; i<kFrameSize; i++) {
    debugPrintx(frameBuffer[i]);
    debugPrint(" ");
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart != &tmpDmaUartPort->Handle) {
    return;
  }

  buffer = tmpDmaUartPort->port.rxBuffer;

  debugPrint("----------------------------------------- read: ");
  printFrame(buffer + currentReadFrameStart);
  debugPrint("\r\n");

  rx_cplt_count++;
  currentReadFrame = (currentReadFrame + 1) % kNumFrames;
  currentReadFrameStart = currentReadFrame * kFrameSize;

  int size = kFrameSize;
  if (requestedResyncBytes > 0) {
    size = requestedResyncBytes;
    requestedResyncBytes = -1;

    gapStart = currentReadFrameStart + size;
    gapEnd = currentReadFrameStart + kFrameSize;
  }
  HAL_UART_Receive_DMA(&tmpDmaUartPort->Handle, (uint8_t*)buffer + currentReadFrameStart, size);
}

int bufferHead() {
  return currentReadFrameStart + kFrameSize - __HAL_DMA_GET_COUNTER(&tmpDmaUartPort->rxDMAHandle);
}

int tail = 0;

void advanceTail() {
  tail = (tail + 1) % kTotalBufferSize;
  if (tail == gapStart) {
    tail = gapEnd;
    gapStart = -1;
    gapEnd = -1;
  }
}

bool areBytesAvailable() {
  if (buffer == NULL) {
    return false;
  }
  //return bufferHead() != tail;
  return currentReadFrameStart != tail;
}

uint8_t readByte() {
  uint8_t result = buffer[tail];
  advanceTail();
  return result;
}

// For now a valid packet is an ascending sequence starting at 0xAA
bool isValidFrame(uint8_t* frameBuffer) {
  for (int i=0; i<kFrameSize; i++) {
    if (frameBuffer[i] != i+0xAA) {
      debugPrint("bad frame: ");
      printFrame(frameBuffer);
      debugPrint("\r\n");
      return false;
    }
  }
  return true;
}

void processAvailableData() {
  static uint8_t frameBuffer[kFrameSize];
  static int bufferPos = 0;

  while (true) {
    while (bufferPos < kFrameSize && areBytesAvailable()) {
      frameBuffer[bufferPos] = readByte();
      bufferPos++;
    }

    if (bufferPos < kFrameSize) {
      // Not enough data available for a frame; return for now
      return;
    }

    if (isValidFrame(frameBuffer)) {
      validFrameCount++;
      bufferPos = 0;
      // NOTE: we process at most one frame per cycle
      // TODO: verify that we are on a frame boundary for async reads
      //   if so, capture the timestamp and process
      //   if not, request a resync
      return;
    } else {
      frameErrorCount++;
    }

    // Find the next start byte in the buffered data
    int nextStartCandidate = -1;
    for (int i=1; i<kFrameSize; i++) {
      if (frameBuffer[i] == kStartByte) {
	nextStartCandidate = i;
	break;
      }
    }

    if (nextStartCandidate != -1) {
      // Shift the buffer to put the start byte at the beginning
      memmove(frameBuffer, frameBuffer + nextStartCandidate, kFrameSize - nextStartCandidate);
      bufferPos = kFrameSize - nextStartCandidate;
    } else {
      bufferPos = 0;
    }
  }
}

void controllerSyncUpdate() {
  if (!csyncPort) {
    return;
  }

  processAvailableData();

  static int ds2 = 0;
  if (++ds2 >= 10) {
    ds2 = 0;
    for (int i=0; i<kFrameSize; i++) {
      serialWrite(csyncPort, kStartByte + i);
    }
    framesSentCount++;
  }

  static int downsample = 0;
  if (++downsample >= 100) {
    downsample = 0;
    debugPrint("sent: ");
    debugPrinti(framesSentCount);
    debugPrint(" valid: ");
    debugPrinti(validFrameCount);
    debugPrint(" errors: ");
    debugPrinti(frameErrorCount);
    debugPrint(" rx_callbacks: ");
    debugPrinti(rx_cplt_count);
    debugPrint("\r\n");
  }
}
