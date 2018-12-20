#include "io/controller_sync.h"

#include <string.h>
#include "platform.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/time.h"

#include "io/serial.h"
#include "io/debug_console.h"
#include "io/fletcher.h"

// If payload is modified, make sure to change kExpectedTransmissionTime
typedef struct {
  uint32_t utime;
  int32_t clock_diff;
} payload_t;


int sync_bytes_written = 0;
int sync_bytes_read = 0;

static serialPort_t *csyncPort;

int rx_cplt_count = 0;
int validFrameCount = 0;
int frameErrorCount = 0;
int framesSentCount = 0;


extern uartPort_t* tmpDmaUartPort;

// frame is 1 start byte + payload + 2 checksum bytes
#define kFrameSize (sizeof(payload_t) + 3)

#define kNumFrames 3
const int kTotalBufferSize = kFrameSize * kNumFrames;
const uint8_t kStartByte = 0xAA;

volatile int currentReadFrame = 0;
volatile int currentReadFrameStart = 0;

volatile int gapStart = -1;
volatile int gapEnd = -1;

volatile int requestedResyncBytes = -1;

// populated on the first callback
//volatile uint8_t* buffer = NULL;
volatile uint8_t buffer[256];

uint32_t frameTxUtime = 0;
volatile uint32_t frameRxUtime = 0;

// Our receive utime minus peer send utime.  We are ahead of peer if this is positive, or
// behind if this is negative.
int32_t ourSendReceiveTimeDiff = 0;

// Difference in utime from one callback to the next on our side (uses no data from our peer).
int32_t callbackInterval = 0;

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

  //buffer = tmpDmaUartPort->port.rxBuffer;
  HAL_UART_Receive_DMA(&tmpDmaUartPort->Handle, (uint8_t*)buffer + currentReadFrameStart, kFrameSize);

  // send a few bytes to intentionally misalign buffer
  for (int i=0; i<6; i++) {
    serialWrite(csyncPort, 'x');
  }
}

void printFrame(volatile uint8_t* frameBuffer) {
  for (uint32_t i=0; i<kFrameSize; i++) {
    if (frameBuffer[i] == 0) {
      debugPrint("..");
    } else {
      debugPrintx(frameBuffer[i]);
    }
    debugPrint(" ");
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart != &tmpDmaUartPort->Handle) {
    return;
  }

  frameRxUtime = micros();

  /*
  debugPrint("----------------------------------------- buffer: ");
  //printFrame(buffer + currentReadFrameStart);
  printFrame(buffer);
  debugPrint(" | ");
  printFrame(buffer + kFrameSize);
  debugPrint(" | ");
  printFrame(buffer + (kFrameSize*2));
  debugPrint(" | ");
  debugPrinti(currentReadFrameStart);
  */

  rx_cplt_count++;
  currentReadFrame = (currentReadFrame + 1) % kNumFrames;
  currentReadFrameStart = currentReadFrame * kFrameSize;

  int size = kFrameSize;
  if (requestedResyncBytes > 0) {
    size = requestedResyncBytes;
    gapStart = currentReadFrameStart + size;
    gapEnd = (currentReadFrameStart + kFrameSize) % kTotalBufferSize;
    requestedResyncBytes = -1;

    // temporary for debugging:
    for (int i=gapStart; i != gapEnd; i = (i+1) % kTotalBufferSize) {
      buffer[i] = 0;
    }
  }

  //for (int i=0; i<kTotalBufferSize; i++) {
  //  buffer[i] = 0;
  //}

  /*
  debugPrint(" ");
  debugPrinti(currentReadFrameStart);
  debugPrint(" ");
  debugPrinti(size);
  debugPrint("\r\n");
  */

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

// For now a valid packet starts with 0xAA and consists of ascending values.
bool isValidFrame(uint8_t* frameBuffer) {
  if (frameBuffer[0] != kStartByte) {
      debugPrint("bad frame (bad start byte): ");
      printFrame(frameBuffer);
      debugPrint("\r\n");
      return false;
  }

  uint16_t computedChecksum = compute_fletcher16(frameBuffer + 1, sizeof(payload_t));
  uint16_t receivedChecksum = *((uint16_t*)(frameBuffer + 1 + sizeof(payload_t)));

  if (computedChecksum != receivedChecksum) {
    debugPrint("bad frame (bad checksum): ");
    printFrame(frameBuffer);
    debugPrint("\r\n");
    return false;
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
      payload_t* payload = (payload_t*)(frameBuffer + 1);
      validFrameCount++;
      bufferPos = 0;
      // NOTE: we process at most one frame per cycle
      // verify that we are on a frame boundary for async reads
      //   if so, capture the timestamp and process
      //   if not, request a resync
      int frameOffset = tail % kFrameSize;
      if (frameOffset == 0) {
	//uint32_t dt = frameRxUtime - payload->utime;
	//debugPrintVar("time in flight: ", dt);

	// TODO: need to verify that frameRxUtime corresponds to the correct frame (it could point to
	// a newer frame if our task has been starved for a while)
	ourSendReceiveTimeDiff = (int32_t)(frameRxUtime - payload->utime);
	int32_t theirSendReceiveTimeDiff = payload->clock_diff;

	int32_t estimatedTransmissionTime = (ourSendReceiveTimeDiff + theirSendReceiveTimeDiff) / 2;

	int32_t estimatedClockDelta = ourSendReceiveTimeDiff - estimatedTransmissionTime;
	int32_t theirEstimatedClockDelta = theirSendReceiveTimeDiff - estimatedTransmissionTime;

	uint32_t peerTxUtime = payload->utime + estimatedClockDelta;

	// TODO: This needs to be modded to the range (-50k, +50k) (i.e. +/- 0.5 of the cycle period)
	int32_t cycleStartDiff = frameTxUtime - peerTxUtime;

	// We print out (utime, our detla, peer delta negated) as that's a convenient tuple for graphing.
	debugPrint("clockDeltaData ");
	debugPrintu(payload->utime);
	debugPrint(",");
	debugPrinti(estimatedClockDelta);
	debugPrint(",");
	debugPrinti(-theirEstimatedClockDelta);
	debugPrint(",");
	debugPrinti(estimatedTransmissionTime);
	debugPrint(",");
	debugPrinti(cycleStartDiff);
	debugPrint(",");
	debugPrinti(callbackInterval);
	debugPrint("\r\n");
      } else {
	if (requestedResyncBytes <= 0 && gapStart <= 0) {
	  requestedResyncBytes = frameOffset;
	  debugPrintVar("requesting resync: ", requestedResyncBytes);
	}
      }
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

// periodically drop a byte or add a spurious byte
void unreliableWrite(serialPort_t *instance, uint8_t ch) {
  static int counter = 0;
  static bool dropOrAdd = false;

  if (counter++ >= 1000) {
    counter = 0;
    if (dropOrAdd) {
      dropOrAdd = false;
      return;
    } else {
      serialWrite(instance, 0xFF);
      dropOrAdd = true;
    }
  }
  serialWrite(instance, ch);
}

void sendFrame(serialPort_t* instance, payload_t* payload) {
  serialWrite(instance, kStartByte);
  uint8_t* payload_data = (uint8_t*)payload;
  for (int i=0; i<sizeof(payload_t); i++) {
    serialWrite(instance, payload_data[i]);
  }
  uint16_t checksum = compute_fletcher16(payload_data, sizeof(payload_t));
  serialWrite(instance, checksum & 0xFF);
  serialWrite(instance, (checksum >> 8) & 0xFF);
}

static uint32_t last_callback = 0;
static uint32_t longest_gap;

void controllerSyncUpdate() {
  if (!csyncPort) {
    return;
  }

  uint32_t utime = micros();
  if (last_callback != 0) {
    callbackInterval = utime - last_callback;
    if (callbackInterval > 15000) {
      debugPrintVar("LONG GAP: ", callbackInterval);
    }
    if (callbackInterval > longest_gap) {
      longest_gap = callbackInterval;
    }
  }
  last_callback = utime;

  // TODO: once we implement loop sync, we'll do this in the other order.  We transmit, then
  // wait up to TIMEOUT to receive data from peer(s).
  processAvailableData();

  static int ds2 = 0;
  if (++ds2 >= 10) {
    ds2 = 0;

    payload_t payload;
    payload.utime = micros();
    frameTxUtime = payload.utime;
    payload.clock_diff = ourSendReceiveTimeDiff;
    sendFrame(csyncPort, &payload);
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
    debugPrint(" longest gap: ");
    debugPrinti(longest_gap);
    debugPrint("\r\n");
  }
}
