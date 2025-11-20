#include "ble_service.h"

static NimBLEServer* pServer = nullptr;
static NimBLECharacteristic* pChr = nullptr;

void BLEAccel::begin(const char* deviceName) {
  NimBLEDevice::init(deviceName);
  NimBLEDevice::setPower(ESP_PWR_LVL_N0); // 降低发射功率以省电（按需调）
  NimBLEDevice::setMTU(100);

  pServer = NimBLEDevice::createServer();
  auto svc = pServer->createService(UUID_SVC_ACCEL);
  pChr = svc->createCharacteristic(UUID_CHR_SAMPLES, NIMBLE_PROPERTY::NOTIFY);

  // 推荐使用 Notify + payload（每notifySamples包打若干样本）
  svc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(UUID_SVC_ACCEL);
  adv->setScanResponse(true);
  adv->start();
}

void BLEAccel::notifySamples(const uint8_t* payload, size_t len) {
  if (!pChr) return;
  if (pChr->getSubscribedCount() == 0) return; // 没有订阅就不发，省电
  pChr->setValue((uint8_t*)payload, len);
  pChr->notify(); // 不关心返回值，兼容返回 void/ bool 的实现
}
