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
* Modified January 2019
*	by Michel Stempin <michel.stempin@wanadoo.fr>
*	Cleanup and optimizations
*
*/

#include <stm32f10x.h>
#include <string.h>
#include <stdbool.h>
#include "config.h"
#include "usb.h"
#include "hid.h"
#include "led.h"
#include "flash.h"
#include "crypt.h"

/* This should be <= MAX_EP_NUM defined in usb.h */
#define EP_NUM 			2

/* Flash memory base address */
#define FLASH_BASE_ADDRESS	0x08000000

/* *
 * PAGE_SIZE : Flash Page size
 *  Low and MEDIUM Density F103 devices have 1 kB Flash page
 *  High Density F103 devices have 2 kB Flash page
 *
 * PAGEMIN : This should be the last page taken by the bootloader
 *  (2 * 1024) or (1 * 2048) In any case, the bootloader size is 2048 bytes
 */
#if (PAGE_SIZE == 2048)
#define MIN_PAGE		1
#else
#define MIN_PAGE		2
#endif

/* Command size */
#define COMMAND_SIZE		64

/* Buffer table offsset in PMA memory */
#define BTABLE_OFFSET		(0x00)

/* EP0  */
/* RX/TX buffer base address */
#define ENDP0_RXADDR		(0x80)
#define ENDP0_TXADDR		(0xC0)

/* EP1  */
/* TX buffer base address */
#define ENDP1_TXADDR		(0x100)

/* Upload finished flag */
volatile bool UploadFinished;

static volatile uint8_t DeviceAddress;

/* Sent command (Received command is the same minus last byte) */
static const uint8_t Command[] = {'B', 'T', 'L', 'D', 'C', 'M', 'D', 2};

/* Flash page buffer */
static uint64_t PageData[PAGE_SIZE / 4];

/* Current page number (starts right after bootloader's end) */
static volatile uint8_t CurrentPage;

static volatile uint8_t CurrentWritePage;

/* Byte offset in flash page */
static volatile uint16_t CurrentPageOffset;

/* USB Descriptors */
static const uint8_t USB_DeviceDescriptor[] = {
	0x12,			// bLength
	0x01,			// bDescriptorType (Device)
	0x10, 0x01,		// bcdUSB 1.10
	0x00,			// bDeviceClass (Use class information in the Interface Descriptors)
	0x00,			// bDeviceSubClass
	0x00,			// bDeviceProtocol
	MAX_PACKET_SIZE,	// bMaxPacketSize0 8
	VID_LOW, VID_HI,	// idVendor VID_HI/VID_LOW
	PID_LOW, PID_HI,	// idProduct PID_HI/PID_LOW
	0x00, 0x03,		// bcdDevice 3.00
	0x01,			// iManufacturer (String Index)
	0x02,			// iProduct (String Index)
	0x00,			// iSerialNumber (String Index)
	0x01 			// bNumConfigurations 1
};

static const uint8_t USB_ConfigurationDescriptor[] = {
	0x09,			// bLength
	0x02,			// bDescriptorType (Configuration)
	0x22, 0x00,		// wTotalLength 34
	0x01,			// bNumInterfaces 1
	0x01,			// bConfigurationValue
	0x00,			// iConfiguration (String Index)
	0xC0,			// bmAttributes Self Powered
	0x32,			// bMaxPower 100mA

	0x09,			// bLength
	0x04,			// bDescriptorType (Interface)
	0x00,			// bInterfaceNumber 0
	0x00,			// bAlternateSetting
	0x01,			// bNumEndpoints 1
	0x03,			// bInterfaceClass (HID)
	0x00,			// bInterfaceSubClass
	0x00,			// bInterfaceProtocol
	0x00,			// iInterface (String Index)

	0x09,			// bLength
	0x21,			// bDescriptorType (HID)
	0x11, 0x01,		// bcdHID 1.11
	0x00,			// bCountryCode
	0x01,			// bNumDescriptors
	0x22,			// bDescriptorType[0] (HID)
	0x20, 0x00,		// wDescriptorLength[0] 32

	0x07,			// bLength
	0x05,			// bDescriptorType (Endpoint)
	0x81,			// bEndpointAddress (IN/D2H)
	0x03,			// bmAttributes (Interrupt)
	MAX_PACKET_SIZE, 0x00,	// wMaxPacketSize 8
	0x05 			// bInterval 5 (2^(5-1)=16 micro-frames)
};

static const uint8_t USB_ReportDescriptor[] = {
	0x06, 0x00, 0xFF,	// Usage Page (Vendor Defined 0xFF00)
	0x09, 0x01,		// Usage (0x01)
	0xA1, 0x01,		// Collection (Application)
	0x09, 0x02,		// 	Usage (0x02)
	0x15, 0x00,		// 	Logical Minimum (0)
	0x25, 0xFF,		// 	Logical Maximum (255)
	0x75, 0x08,		// 	Report Size (8)
	0x95, 0x08,		// 	Report Count (8)
	0x81, 0x02,		// 	Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x09, 0x03,		// 	Usage (0x03)
	0x15, 0x00,		// 	Logical Minimum (0)
	0x25, 0xFF,		// 	Logical Maximum (255)
	0x75, 0x08,		// 	Report Size (8)
	0x95, 0x40,		// 	Report Count (64)
	0x91, 0x02,		// 	Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
	0xC0 			// End Collection
};

/* USB String Descriptors */
static const uint8_t USB_LangIDStringDescriptor[] = {
	0x04,			// bLength
	0x03,			// bDescriptorType (String)
	0x09, 0x04		// English (United States)
};

static const uint8_t USB_VendorStringDescriptor[] = {
	0x0A,			// bLength
	0x03,			// bDescriptorType (String)
	'E', 0, 'V', 0, 'T', 0, '1', 0
};

static const uint8_t USB_ProductStringDescriptor[] = {
	0x28,			// bLength
	0x03,			// bDescriptorType (String)
	'A', 0, 'l', 0, 't', 0, '-', 0, 'L', 0, 'i', 0, 'n', 0, 'k', 0,
	' ', 0,	'B', 0, 'o', 0, 'o', 0, 't', 0, 'l', 0, 'o', 0, 'a', 0,
	'd', 0, 'e', 0,	'r', 0
};

static void HIDUSB_GetDescriptor(USB_SetupPacket *setup_packet)
{
	uint16_t *descriptor = 0;
	uint16_t length = 0;

	switch (setup_packet->wValue.H) {
	case USB_DEVICE_DESC_TYPE:
		descriptor =  (uint16_t *) USB_DeviceDescriptor;
		length = sizeof (USB_DeviceDescriptor);
		break;

	case USB_CFG_DESC_TYPE:
		descriptor = (uint16_t *) USB_ConfigurationDescriptor;
		length =  sizeof (USB_ConfigurationDescriptor);
		break;

	case USB_REPORT_DESC_TYPE:
		descriptor = (uint16_t *) USB_ReportDescriptor;
		length =  sizeof (USB_ReportDescriptor);
		break;

	case USB_STR_DESC_TYPE:
		switch (setup_packet->wValue.L) {
		case 0x00:
			descriptor = (uint16_t *) USB_LangIDStringDescriptor;
			length = sizeof (USB_LangIDStringDescriptor);
			break;

		case 0x01:
			descriptor = (uint16_t *) USB_VendorStringDescriptor;
			length = sizeof (USB_VendorStringDescriptor);
			break;

		case 0x02:
			descriptor = (uint16_t *) USB_ProductStringDescriptor;
			length = sizeof (USB_ProductStringDescriptor);
			break;

		default:
			break;
		}
		break;

	default:
		break;
	}
	if (length > setup_packet->wLength) {
		length = setup_packet->wLength;
	}
	USB_SendData(0, descriptor, length);
}

static uint8_t HIDUSB_PacketIsCommand(uint8_t *data)
{
	size_t i;

	for (i = 0; i < sizeof (Command) - 1; i++) {
		if (data[i] != Command[i]) {
			return 0xff;
		}
 	}
	return data[sizeof (Command) - 1];
}

static uint64_t extended_key[2 * ROUNDS];

void InitFlashVariables() {
	/* Initialize Flash Page Settings */
	CurrentPage = 0;
	CurrentWritePage = 0;
	CurrentPageOffset = 0;
}

static void HIDUSB_HandleData(uint8_t *data)
{
	uint64_t* p = PageData;
	if ((CurrentPage & 0x01) != 0) p += (PAGE_SIZE >> 3);
	memcpy(p + (CurrentPageOffset >> 3), data, MAX_PACKET_SIZE);
	CurrentPageOffset += MAX_PACKET_SIZE;
	if (CurrentPageOffset == COMMAND_SIZE) {
		switch (HIDUSB_PacketIsCommand(data)) {
			case 0x00:
				/* Reset Page Command */
				InitFlashVariables();

#if FIRMWARE_KEY1 == 0x0123456789ABCDEF
#warning Using default firmware key (0x0123456789ABCDEF). Please consider to use your own key for security
#endif
				static const uint64_t key[2] = {
					FIRMWARE_KEY1, FIRMWARE_KEY2
				};
				extend_key(extended_key, key);
				set_iv((uint64_t*)data + sizeof(Command));
			break;

			case 0x01:
				/* Reboot MCU Command */
				UploadFinished = true;
			break;

			default:
				break;
		}
	} else if (CurrentPageOffset >= PAGE_SIZE) {
		CurrentPage++;
		CurrentPageOffset = 0;
	}

	if((CurrentPageOffset == 0) /*|| (CurrentPageOffset == 1024)*/){
		USB_SendData(ENDP1, (uint16_t *)Command, sizeof (Command));
	}
}

bool CanFlash() {
	return (CurrentPage > CurrentWritePage);
}

void FlashPage() {
	uint64_t* p = PageData;
	if ((CurrentWritePage & 0x01) != 0) p += (PAGE_SIZE >> 3);
	decrypt(p, p, PAGE_SIZE, extended_key);
	//LED1_ON;
	uint16_t* page_address = (uint16_t *)(FLASH_BASE_ADDRESS + MIN_PAGE * PAGE_SIZE + (CurrentWritePage * PAGE_SIZE));
	FLASH_WritePage(page_address, (uint16_t *)p, PAGE_SIZE / 2);
	CurrentWritePage++;
	//LED1_OFF;

	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
	SET_RX_STATUS(0, EP_RX_VALID);
	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

void USB_Reset(void)
{
	/* Set buffer descriptor table offset in PMA memory */
	// [MEMO] We don't change BTABLE
	//WRITE_REG(*BTABLE, BTABLE_OFFSET);

	/* Initialize Endpoint 0 */
	TOGGLE_REG(EP0REG[ENDP0],
		   EP_DTOG_RX | EP_T_FIELD | EP_KIND | EP_DTOG_TX | EPTX_STAT | EPADDR_FIELD,
		   0 | EP_CONTROL | 0,
		   EP_RX_VALID);

	/* Set transmission buffer address for endpoint 0 in buffer descriptor table */
	BTABLE_ADDR_FROM_OFFSET(ENDP0, BTABLE_OFFSET)[USB_ADDRn_TX] = ENDP0_TXADDR;

	/* Set reception buffer address for endpoint 0 in buffer descriptor table */
	BTABLE_ADDR_FROM_OFFSET(ENDP0, BTABLE_OFFSET)[USB_ADDRn_RX] = ENDP0_RXADDR;

	/* BLSIZE=1, NUM_BLOCK=1 */
	BTABLE_ADDR_FROM_OFFSET(ENDP0, BTABLE_OFFSET)[USB_COUNTn_RX] = 0x8400;

	/* Initialize Endpoint 1 */
	TOGGLE_REG(EP0REG[ENDP1],
		   EP_DTOG_RX | EP_T_FIELD | EP_KIND | EP_DTOG_TX | EPADDR_FIELD,
		   1 | EP_INTERRUPT | 0,
		   EP_RX_DIS | EP_TX_NAK);

	/* Set transmission buffer address for endpoint 1 in buffer descriptor table */
	BTABLE_ADDR_FROM_OFFSET(ENDP1, BTABLE_OFFSET)[USB_ADDRn_TX] = ENDP1_TXADDR;

	/* Clear device address and enable USB function */
	WRITE_REG(*DADDR, DADDR_EF | 0);

	DeviceAddress = 0;
}

void USB_EPHandler(uint16_t status)
{
	uint8_t endpoint = READ_BIT(status, USB_ISTR_EP_ID);
	uint16_t endpoint_status = EP0REG[endpoint];

	/* OUT and SETUP packets (data reception) */
	if (READ_BIT(endpoint_status, EP_CTR_RX)) {
		uint16_t rx_status = EP_RX_VALID;
		if (endpoint == 0) {
			/* Copy from packet area to user buffer */
			uint32_t rx_length = USB_PMA2Buffer(endpoint);

			/* If control endpoint */
			if (READ_BIT(endpoint_status, USB_EP0R_SETUP)) {
				USB_SetupPacket *setup_packet = (USB_SetupPacket *) RxTxBuffer[endpoint].RXB;
				// Class request
				if ((setup_packet->bmRequestType & 0x60) == 0x20) {
					if (CurrentPage >= CurrentWritePage + 2) {
						rx_status = EP_RX_NAK;
					}
					USB_SendData(0, 0, 0);
				} else {
					switch (setup_packet->bRequest) {

					case USB_REQUEST_SET_ADDRESS:
						DeviceAddress = setup_packet->wValue.L;
						USB_SendData(0, 0, 0);
						break;

					case USB_REQUEST_GET_DESCRIPTOR:
						HIDUSB_GetDescriptor(setup_packet);
						break;

					case USB_REQUEST_GET_STATUS:
						{
							uint16_t DeviceStatus = 0;
							USB_SendData(0, (uint16_t *) &DeviceStatus, 2);
						}
						break;

					case USB_REQUEST_GET_CONFIGURATION:
						USB_SendData(0, (uint16_t *) &DeviceConfigured, 1);
						break;

					case USB_REQUEST_SET_CONFIGURATION:
						DeviceConfigured = 1;
						USB_SendData(0, 0, 0);
						break;

					case USB_REQUEST_GET_INTERFACE:
						USB_SendData(0, 0, 0);
						break;

					default:
						USB_SendData(0, 0, 0);
						SET_TX_STATUS(ENDP0, EP_TX_STALL);
						break;
					}
				}
			} else if (rx_length) {
				/* OUT packet */
				HIDUSB_HandleData((uint8_t *) RxTxBuffer[endpoint].RXB);
			}

		}
		SET_RX_STATUS(endpoint, rx_status);
	}
	// IN
	if (READ_BIT(endpoint_status, EP_CTR_TX)) {

		/* Something transmitted */
		if (DeviceAddress) {

			/* Set device address and enable function */
			WRITE_REG(*DADDR, DADDR_EF | DeviceAddress);
			DeviceAddress = 0;
		}
#if defined(SUPPORT_OVER_64_BYTES_TRANSMISSION)
		USB_Buffer2PMA(endpoint);
#endif
		SET_TX_STATUS(endpoint, (endpoint == ENDP1) ? EP_TX_NAK : EP_TX_VALID);
	}
}
