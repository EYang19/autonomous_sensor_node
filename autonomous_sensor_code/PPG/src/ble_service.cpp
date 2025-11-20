#include "ble_service.h"

static NimBLEServer*        gServer = nullptr;
static NimBLECharacteristic* gChr   = nullptr;

void BLEPPG::begin(const char* name) {
  NimBLEDevice::init(name);
  NimBLEDevice::setPower(ESP_PWR_LVL_N0); // 降功耗
  NimBLEDevice::setMTU(100);

  gServer = NimBLEDevice::createServer();
  auto svc = gServer->createService(UUID_SVC_PPG);
  gChr = svc->createCharacteristic(UUID_CHR_PPG, NIMBLE_PROPERTY::NOTIFY);
  svc->start();

  auto adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(UUID_SVC_PPG);
  adv->setScanResponse(true);
  adv->start();
}

void BLEPPG::notifySamples(const uint8_t* payload, size_t len) {
  if (!gChr) return;
  if (gChr->getSubscribedCount() == 0) return;
  gChr->setValue((uint8_t*)payload, len);
  gChr->notify();
}
