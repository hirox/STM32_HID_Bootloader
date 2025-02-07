/*
* STM32 HID Bootloader - USB HID bootloader for STM32F10X
* Copyright (c) 2018 Bruno Freitas - bruno@brunofreitas.com
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
* Modified 20 April 2018
*	by Vassilis Serasidis <avrsite@yahoo.gr>
*	This HID bootloader work with bluepill + STM32duino + Arduino IDE <http://www.stm32duino.com/>
*
*/

#include <cstdint>
#include <optional>
#include <sstream>
#include <random>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include "hidapi.h"
#include "crypt.h"

#define SECTOR_SIZE  1024
#define HID_TX_SIZE    65
#define HID_RX_SIZE     9

#define VID           ((VID_HI << 8) + VID_LOW)
#define PID           ((PID_HI << 8) + PID_LOW)
#define FIRMWARE_VER  0x0300

static uint64_t extended_key[2 * ROUNDS];
static std::uint64_t iv[2];

static int usb_write(hid_device *device, uint8_t *buffer, int len) {
  int retries = 20;
  int retval;

  while(((retval = hid_write(device, buffer, len)) < len) && --retries) {
    if(retval < 0) {
      usleep(100 * 1000); // No data has been sent here. Delay and retry.
    } else {
      return 0; // Partial data has been sent. Firmware will be corrupted. Abort process.
    }
  }

  if(retries <= 0) {
    return 0;
  }

  return 1;
}

static std::optional<std::uint32_t> get_valid_devices() {
  struct hid_device_info *devs, *cur_dev;
  uint8_t valid_hid_devices = 0;
  
  for(int i=0;i<10;i++){ //Try up to 10 times to open the HID device.
    devs = hid_enumerate(VID, PID);
    cur_dev = devs;
    while (cur_dev) { //Search for valid HID Bootloader USB devices
      if((cur_dev->vendor_id == VID)&&(cur_dev->product_id = PID)){
        valid_hid_devices++;
        if(cur_dev->release_number < FIRMWARE_VER){ //The STM32 board has firmware lower than 3.00
          printf("\nError - Please update the firmware to the latest version (v3.00+)");
          return std::nullopt;
        }
      }
      cur_dev = cur_dev->next;
    }
    hid_free_enumeration(devs);
    printf("#");
    sleep(1);
    if(valid_hid_devices > 0) break;
  }
  return valid_hid_devices;
}

static int read_file(uint64_t page_data[SECTOR_SIZE / 8], FILE* in, FILE* out) {
  memset(page_data, 0, SECTOR_SIZE);
  size_t read_bytes = fread(page_data, 1, SECTOR_SIZE, in);
  if (read_bytes > 0) {
#if 0
    uint64_t tmp[SECTOR_SIZE / 8];
    uint64_t tmp_iv[2] = {iv[0], iv[1]};
    set_iv(tmp_iv);
    encrypt(tmp, page_data, SECTOR_SIZE, extended_key);
    set_iv(tmp_iv);
    decrypt(tmp, tmp, SECTOR_SIZE, extended_key);
    if (memcmp(tmp, page_data, SECTOR_SIZE) != 0) {
      printf("Enc/Dec error");
      return 0;
    }
#endif

    encrypt(page_data, page_data, SECTOR_SIZE, extended_key);
    fwrite(page_data, 1, SECTOR_SIZE, out);
  }
  return read_bytes;
}

int main(int argc, char *argv[]) {
  uint64_t page_data[SECTOR_SIZE / 8];
  uint8_t hid_tx_buf[HID_TX_SIZE];
  uint8_t hid_rx_buf[HID_RX_SIZE];
  uint8_t CMD_RESET_PAGES[8] = {'B','T','L','D','C','M','D', 0x00};
  uint8_t CMD_REBOOT_MCU[8] = {'B','T','L','D','C','M','D', 0x01};
  hid_device *handle = NULL;
  FILE *encrypted_file = NULL;
  FILE *firmware_file = NULL;
  int error = 0;
  uint32_t n_bytes = 0;
  setbuf(stdout, NULL);
  bool encryption_only = false;
  std::stringstream ss;

#if FIRMWARE_KEY1 == 0x0123456789ABCDEF
#warning Using default firmware key (0x0123456789ABCDEF). Please consider to use your own key for security
#endif
  static const uint64_t key[2] = {
    FIRMWARE_KEY1, FIRMWARE_KEY2
  };
  extend_key(extended_key, key);

  printf("\n+-----------------------------------------------------------------------+\n");
  printf  ("|         HID-Flash v2.2.1 - STM32 HID Bootloader Flash Tool            |\n");
  printf  ("|     (c)      2018 - Bruno Freitas       http://www.brunofreitas.com   |\n");
  printf  ("|     (c) 2018-2019 - Vassilis Serasidis  https://www.serasidis.gr      |\n");
  printf  ("|   Customized for STM32duino ecosystem   https://www.stm32duino.com    |\n");
  printf  ("+-----------------------------------------------------------------------+\n\n");
  
  if(argc <= 1) {
    printf("Usage: hid-flash <bin_firmware_file> <bool: encryption only>\n");
    return 1;
  }else if (argc >= 3) {
    encryption_only = true;
  }
  
  firmware_file = fopen(argv[1], "rb");
  if(!firmware_file) {
    printf("> Error opening firmware file: %s\n", argv[1]);
    return error;
  }
  
  std::random_device seed_gen;
  std::mt19937_64 engine(seed_gen());
  iv[0] = engine();
  iv[1] = engine();
  set_iv(iv);

  if (!encryption_only) {
    hid_init();
    printf("> Searching for [%04X:%04X] device...\n",VID,PID);

    auto valid_hid_devices = get_valid_devices();
    if (valid_hid_devices == std::nullopt) goto exit;
    if (*valid_hid_devices == 0){
      printf("\nError - [%04X:%04X] device is not found :(",VID,PID);
      error = 1;
      goto exit;
    } 
  
    handle = hid_open(VID, PID, NULL);
    
    if (handle == NULL) {
      printf("\n> Unable to open the [%04X:%04X] device.\n",VID,PID);
      error = 1;
      goto exit;
    }
  
    printf("\n> Opened device [%04X:%04X]\n",VID,PID);
  
    // Send RESET PAGES command to put HID bootloader in initial stage...
    memset(hid_tx_buf, 0, sizeof(hid_tx_buf)); //Fill the hid_tx_buf with zeros.
    memcpy(&hid_tx_buf[1], CMD_RESET_PAGES, sizeof(CMD_RESET_PAGES));

    *reinterpret_cast<std::uint64_t*>(&hid_tx_buf[1 + 8]) = iv[0];
    *reinterpret_cast<std::uint64_t*>(&hid_tx_buf[1 + 8 + 8]) = iv[1];

    printf("> Sending <reset pages> command...\n");

    // Flash is unavailable when writing to it, so USB interrupt may fail here
    if(!usb_write(handle, hid_tx_buf, HID_TX_SIZE)) {
      printf("> Error while sending <reset pages> command.\n");
      error = 1;
      goto exit;
    }
    memset(hid_tx_buf, 0, sizeof(hid_tx_buf));

    // Send Firmware File data
    printf("> Flashing firmware...\n");
  } else {
    printf("> Opening firmware...\n");
  }

  ss << argv[1] << ".encrypted";
  encrypted_file = fopen(ss.str().c_str(), "wb");
  if(!encrypted_file) {
    printf("> Error opening firmware file: %s\n", ss.str().c_str());
    goto exit;
  }
  fwrite(iv, 1, 16, encrypted_file);

  while(1) {
    size_t read_bytes = read_file(page_data, firmware_file, encrypted_file);
    if (read_bytes <= 0) break;

    for(int i = 0; i < SECTOR_SIZE; i += HID_TX_SIZE - 1) {
      memcpy(&hid_tx_buf[1], page_data + i / 8, HID_TX_SIZE - 1);

      if (!encryption_only) {
        if((i % 1024) == 0)
          printf(".");

        // Flash is unavailable when writing to it, so USB interrupt may fail here
        if(!usb_write(handle, hid_tx_buf, HID_TX_SIZE)) {
          printf("> Error while flashing firmware data.\n");
          error = 1;
          goto exit;
        }
        usleep(500);
      }
      n_bytes += (HID_TX_SIZE - 1);
    }

    if (!encryption_only)
      printf(" %d Bytes\n", n_bytes);

    if (!encryption_only) {
      do{
          hid_read(handle, hid_rx_buf, 9);
          usleep(500);
      } while(hid_rx_buf[7] != 0x02);
    }
  }

  printf("\n> Done! %d Bytes\n", n_bytes);
  
  if (!encryption_only) {
    // Send CMD_REBOOT_MCU command to reboot the microcontroller...
    memset(hid_tx_buf, 0, sizeof(hid_tx_buf));
    memcpy(&hid_tx_buf[1], CMD_REBOOT_MCU, sizeof(CMD_REBOOT_MCU));

    printf("> Sending <reboot mcu> command...\n");

    // Flash is unavailable when writing to it, so USB interrupt may fail here
    if(!usb_write(handle, hid_tx_buf, HID_TX_SIZE)) {
      printf("> Error while sending <reboot mcu> command.\n");
    }
  }
  
exit:
  if (!encryption_only) {
    if(handle) {
      hid_close(handle);
    }

    hid_exit();
  }

  if(encrypted_file) {
    fclose(encrypted_file);
  }

  if(firmware_file) {
    fclose(firmware_file);
  }

  printf("> Finish\n");
  
  return error;
}
