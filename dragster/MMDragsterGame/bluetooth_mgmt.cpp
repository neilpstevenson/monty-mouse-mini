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
#include <TFT_eSPI.h> // TTGO T-Display library
extern TFT_eSPI tft;

#define BT_DISCOVER_TIME  5000
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

  // Perform discovery. BluetoothSerial is used for convenience only
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.print("Searching ...");

  // Scan synchronously
  Serial.println("Starting discover...");
  
  SerialBT.begin("HammerTime", true); //Bluetooth device name
  BTScanResults *pResults = SerialBT.discover(BT_DISCOVER_TIME);
  
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  if (pResults && pResults->getCount())
  {
    pResults->dump(&Serial);

    // Attempt to connect to the first
    BTAdvertisedDevice* dev = pResults->getDevice(0);

    esp_err_t tError = esp_spp_connect(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 1, // remoteScn
                                        *dev->getAddress().getNative());
    if(ESP_OK == tError) 
    {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.print("Connected\n");
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.print(dev->getName().c_str());
      Serial.println("Connected to device ok"); 
    } 
    else 
    {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.print("Failed to\nConnect");
      Serial.print("Failed to connect to device: ");
      Serial.println(tError);
    }

    //SerialBT.connect(*dev->getAddress().getNative());
    
    //esp_hidh_dev_open(dev.getAddress().getNative(), pResults->transport, pResults->ble.addr_type);
  }
  else
  {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("None Found");
    Serial.println("Error on BT Scan, no result!");
  }
  //SerialBT.end();
}
