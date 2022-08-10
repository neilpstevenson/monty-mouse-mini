/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <Arduino.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include"esp_gap_bt_api.h"
#include "esp_err.h"
#include <BluetoothSerial.h>

extern "C" void scanAndPairBluetooth(void);

#define REMOVE_BONDED_DEVICES 1   // <- Set to 0 to view all paired devices addresses, set to 1 to remove

#define PAIR_MAX_DEVICES 20
uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];

char *bda2str(const uint8_t* bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}
 
void removedPairedDevices(bool showOnly)
{
  Serial.print("ESP32 bluetooth address: "); Serial.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));
  // Get the numbers of bonded/paired devices in the BT module
  int count = esp_bt_gap_get_bond_device_num();
  if(!count) 
  {
    Serial.println("No paired devices found.");
  } 
  else 
  {
    Serial.print("Paired device count: "); Serial.println(count);
    if(PAIR_MAX_DEVICES < count) 
    {
      count = PAIR_MAX_DEVICES; 
      Serial.print("Reset paired device count: "); Serial.println(count);
    }
    esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    if(ESP_OK == tError) 
    {
      for(int i = 0; i < count; i++) 
      {
        Serial.print("Found paired device # "); Serial.print(i); Serial.print(" -> ");
        Serial.println(bda2str(pairedDeviceBtAddr[i], bda_str, 18));     
        if(!showOnly) 
        {
          esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
          if(ESP_OK == tError) 
          {
            Serial.print("Removed paired device # "); 
          } 
          else 
          {
            Serial.print("Failed to remove paired device # ");
          }
          Serial.println(i);
        }
      }
    }
  }   
}

BluetoothSerial SerialBT;

extern void pairBluetooth(void)
{
  removedPairedDevices(!REMOVE_BONDED_DEVICES);
  //scanAndPairBluetooth();

  SerialBT.begin("ESP32test", true); //Bluetooth device name

  // Scan synchronously
  #define BT_DISCOVER_TIME  10000
  Serial.println("Starting discover...");
  BTScanResults *pResults = SerialBT.discover(BT_DISCOVER_TIME);
  if (pResults)
  {
    pResults->dump(&Serial);

    esp_err_t tError = esp_spp_connect(ESP_SPP_SEC_AUTHORIZE, ESP_SPP_ROLE_MASTER, 1, // remoteScn
                                        *pResults->getDevice(0)->getAddress().getNative());
    if(ESP_OK == tError) 
    {
      Serial.println("Connected to device ok"); 
    } 
    else 
    {
      Serial.print("Failed to connect to device: ");
      Serial.println(tError);
    }
    

    // Attempt to connect to the first
//    BTAdvertisedDevice* dev = pResults->getDevice(0);
//    SerialBT.connect(*dev->getAddress().getNative());
    
    //esp_hidh_dev_open(dev.getAddress().getNative(), pResults->transport, pResults->ble.addr_type);
  }
  else
    Serial.println("Error on BT Scan, no result!");

}
