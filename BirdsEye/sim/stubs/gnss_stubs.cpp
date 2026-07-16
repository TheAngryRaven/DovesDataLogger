///////////////////////////////////////////
// SparkFun u-blox GNSS v3 — sim stubs.
//
// The sim uses the REAL SparkFun header (for UBX_NAV_PVT_data_t and the
// SFE_UBLOX_GNSS_SERIAL type the sketch instantiates) but NOT the real
// 20k-line driver: there is no module to talk to, and PVT is injected by
// the host directly into the sketch's onPVTReceived() callback (see the
// handoff spec — "do NOT drive the SparkFun library through a fake
// serial Stream"). These no-op definitions satisfy the linker for
// exactly the methods the firmware references.
//
// Under SIM the sketch never calls begin()/probe paths (GPS_SETUP's SIM
// branch just sets gpsInitialized), but the probe/recovery functions are
// still compiled, so their callees must link. Every stub is a harmless
// success/failure constant; checkUblox()/checkCallbacks() run every
// GPS_LOOP() and must be cheap no-ops.
///////////////////////////////////////////

#include <SparkFun_u-blox_GNSS_v3.h>

DevUBLOXGNSS::DevUBLOXGNSS(void) {}
DevUBLOXGNSS::~DevUBLOXGNSS(void) {}

bool DevUBLOXGNSS::init(uint16_t, bool) { return false; }

void DevUBLOXGNSS::setCommunicationBus(SparkFun_UBLOX_GNSS::GNSSDeviceBus&) {}

bool DevUBLOXGNSS::checkUblox(uint8_t, uint8_t) { return false; }

void DevUBLOXGNSS::checkCallbacks(void) {}

bool DevUBLOXGNSS::setAutoPVTcallbackPtr(void (*)(UBX_NAV_PVT_data_t*),
                                         uint8_t, uint16_t) {
  return true;
}

bool DevUBLOXGNSS::setAutoNAVSATcallbackPtr(void (*)(UBX_NAV_SAT_data_t*),
                                            uint8_t, uint16_t) {
  return true;
}

bool DevUBLOXGNSS::setAutoNAVSAT(bool, uint8_t, uint16_t) { return true; }

bool DevUBLOXGNSS::setAutoNAVSATrate(uint8_t, bool, uint8_t, uint16_t) {
  return true;
}

bool DevUBLOXGNSS::setNavigationFrequency(uint8_t, uint8_t, uint16_t) {
  return true;
}

bool DevUBLOXGNSS::setDynamicModel(dynModel, uint8_t, uint16_t) {
  return true;
}

bool DevUBLOXGNSS::setUART1Output(uint8_t, uint8_t, uint16_t) { return true; }

bool DevUBLOXGNSS::setSerialRate(uint32_t, uint8_t, uint8_t, uint16_t) {
  return true;
}

bool DevUBLOXGNSS::enableGNSS(bool, sfe_ublox_gnss_ids_e, uint8_t, uint16_t) {
  return true;
}

bool DevUBLOXGNSS::powerOff(uint32_t, uint16_t) { return true; }
