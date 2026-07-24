#pragma once

#include <Arduino.h>

// Must be >= MAX_TRANS_UNIT(255) + 3 (logRxRaw header bytes: code + snr + rssi) so that
// logRxRaw() never silently drops overheard packets.  The old value of 176 caused any
// packet larger than 173 bytes (common after several mesh hops) to be discarded with no
// error — see github.com/meshcore-dev/MeshCore/issues/3022.
#define MAX_FRAME_SIZE  260

class BaseSerialInterface {
protected:
  BaseSerialInterface() { }

public:
  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual bool isEnabled() const = 0;

  virtual bool isConnected() const = 0;

  virtual bool isWriteBusy() const = 0;
  virtual size_t writeFrame(const uint8_t src[], size_t len) = 0;
  virtual size_t checkRecvFrame(uint8_t dest[]) = 0;
};

