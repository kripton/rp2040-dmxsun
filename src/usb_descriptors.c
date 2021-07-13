/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "tusb.h"

#include <pico/stdlib.h>
#include <pico/unique_id.h>

#include "version.h"
#include "boardconfig.h"

// TODO: Get a USB ID for the "native" protocol
#define DEFAULT_VID 0x16C0
#define DEFAULT_PID 0x088B

// String descriptor indices
enum
{
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_CDC_ACM_IFNAME,
  STRID_CDC_ECM_IFNAME,
  STRID_MAC
};

// Available interfaces
enum {
    ITF_NUM_HID,
    ITF_NUM_CDC_ACM_CMD,
    ITF_NUM_CDC_ACM_DATA,
    ITF_NUM_CDC_ECM_CMD,
    ITF_NUM_CDC_ECM_DATA,
    ITF_NUM_TOTAL
};

// Available configurations
// Configuration array: RNDIS and CDC-ECM
// - Windows only works with RNDIS
// - MacOS only works with CDC-ECM
// - Linux will work on both
// Note index is Num-1x
enum
{
  CONFIG_ID_RNDIS = 0,
  CONFIG_ID_ECM   = 1,
  CONFIG_ID_COUNT
};

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = DEFAULT_VID, // Possibly overwritten by function below
    .idProduct          = DEFAULT_PID, // Possibly overwritten by function below
    .bcdDevice          = VERSION_BCD,
    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,
    .bNumConfigurations = CONFIG_ID_COUNT
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const *tud_descriptor_device_cb(void) {
  uint8_t usbProtocol = getUsbProtocol();
  if (usbProtocol == 1) {
    // JaRule emulation
    desc_device.idVendor = 0x1209;
    desc_device.idProduct = 0xaced;
  } else if (usbProtocol == 2) {
    // uDMX emulation
    desc_device.idVendor = 0x16C0;
    desc_device.idProduct = 0x05DC;
  } else if (usbProtocol == 3) {
    // OpenDMX emulation
    desc_device.idVendor = 0x0403;
    desc_device.idProduct = 0x6001;
  } else if (usbProtocol == 4) {
    // Nodle U1 emulation
    desc_device.idVendor = 0x16C0;
    desc_device.idProduct = 0x088B;
  } else if (usbProtocol == 5) {
    // Nodle U1 emulation
    desc_device.idVendor = 0x0403;
    desc_device.idProduct = 0xec70;
  }

    return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// HID Report Descriptor
//--------------------------------------------------------------------+

uint8_t const desc_hid_report[] =
{
    TUD_HID_REPORT_DESC_GENERIC_INOUT(CFG_TUD_HID_BUFSIZE)
};

// Invoked when received GET HID REPORT DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    return desc_hid_report;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
// TODO: Those will probably have to change, depending on USB emulation selected!
#define  CONFIG_RNDIS_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN + TUD_CDC_DESC_LEN + TUD_RNDIS_DESC_LEN)
#define  CONFIG_ECM_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN + TUD_CDC_DESC_LEN + TUD_CDC_ECM_DESC_LEN)

#define EPNUM_HID_OUT            0x02
#define EPNUM_HID_IN             0x81

#define EPNUM_CDC_ACM_CMD        0x83
#define EPNUM_CDC_ACM_OUT        0x04
#define EPNUM_CDC_ACM_IN         0x84
#define USBD_CDC_CMD_MAX_SIZE       8
#define USBD_CDC_IN_OUT_MAX_SIZE   64

#define EPNUM_CDC_ECM_CMD        0x85
#define EPNUM_CDC_ECM_OUT        0x06
#define EPNUM_CDC_ECM_IN         0x86

uint8_t const rndis_configuration[] =
{
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(CONFIG_ID_RNDIS+1, ITF_NUM_TOTAL, 0, CONFIG_RNDIS_TOTAL_LEN,
        TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),

    // Interface number, string index, protocol, report descriptor len, EP In & Out address, size & polling interval
    TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE,
        sizeof(desc_hid_report), EPNUM_HID_OUT, EPNUM_HID_IN,
        CFG_TUD_HID_BUFSIZE, 5),

    // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_ACM_CMD, STRID_CDC_ACM_IFNAME, EPNUM_CDC_ACM_CMD,
        USBD_CDC_CMD_MAX_SIZE, EPNUM_CDC_ACM_OUT, EPNUM_CDC_ACM_IN,
        USBD_CDC_IN_OUT_MAX_SIZE),

    // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_RNDIS_DESCRIPTOR(ITF_NUM_CDC_ECM_CMD, STRID_CDC_ECM_IFNAME, EPNUM_CDC_ECM_CMD, 8, EPNUM_CDC_ECM_OUT, EPNUM_CDC_ECM_IN, CFG_TUD_NET_ENDPOINT_SIZE),
};

uint8_t const ecm_configuration[] =
{
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(CONFIG_ID_RNDIS+1, ITF_NUM_TOTAL, 0, CONFIG_RNDIS_TOTAL_LEN,
        TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),

    // Interface number, string index, protocol, report descriptor len, EP In & Out address, size & polling interval
    TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE,
        sizeof(desc_hid_report), EPNUM_HID_OUT, EPNUM_HID_IN,
        CFG_TUD_HID_BUFSIZE, 5),

    // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_ACM_CMD, STRID_CDC_ACM_IFNAME, EPNUM_CDC_ACM_CMD,
        USBD_CDC_CMD_MAX_SIZE, EPNUM_CDC_ACM_OUT, EPNUM_CDC_ACM_IN,
        USBD_CDC_IN_OUT_MAX_SIZE),

    // Interface number, description string index, MAC address string index, EP notification address and size, EP data address (out, in), and size, max segment size.
    TUD_CDC_ECM_DESCRIPTOR(ITF_NUM_CDC_ECM_CMD, STRID_CDC_ECM_IFNAME, STRID_MAC, EPNUM_CDC_ECM_CMD, 64, EPNUM_CDC_ECM_OUT, EPNUM_CDC_ECM_IN, CFG_TUD_NET_ENDPOINT_SIZE, CFG_TUD_NET_MTU),
};

// Configuration array: RNDIS and CDC-ECM
// - Windows only works with RNDIS
// - MacOS only works with CDC-ECM
// - Linux will work on both
// Note index is Num-1x
static uint8_t const * const configuration_arr[2] =
{
  [CONFIG_ID_RNDIS] = rndis_configuration,
  [CONFIG_ID_ECM  ] = ecm_configuration
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  return (index < CONFIG_ID_COUNT) ? configuration_arr[index] : NULL;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const *string_desc_arr[] =
{
    [STRID_LANGID]         = (const char[]) {0x09, 0x04},           // 0: is supported language is English (0x0409)
    [STRID_MANUFACTURER]   = "OpenLightingProject",                 // 1: Manufacturer
    [STRID_PRODUCT]        = "rp2040-dongle http://255.255.255.255",// 2: Product
    [STRID_SERIAL]         = "RP2040_0123456789ABCDEF",             // 3: Serial, fallback here, it's dynamically created later
    [STRID_CDC_ACM_IFNAME] = "Debugging Console",                   // 4: CDC ACM interface name
    [STRID_CDC_ECM_IFNAME] = "Network Interface",                   // 5: CDC ECM interface name, also handled in tud_descriptor_string_cb
    [STRID_MAC]            = "000000000000"                         // 6: MAC address is handled in tud_descriptor_string_cb
};

static uint16_t _desc_str[128];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    char *str = NULL;
    uint8_t chr_count = 0;

    if (index == STRID_SERIAL) {
        // Serial number has been requested, construct it from the unique board id
        pico_unique_board_id_t board_id;
        pico_get_unique_board_id(&board_id);

        char serial[33];
        str = serial;

        snprintf(serial, 32, "RP2040_");

        for (int i = 0; (i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES) && (i < 8); ++i) {
            snprintf(serial + i*2 + 7, 32, "%02x", board_id.id[i]);
        }
    } else if (index == STRID_PRODUCT) {
      // Network interface name has been requested. Get our IP there
      char product[64];
      uint32_t ip = getOwnIp();
      str = product;
      snprintf(product, 64, "Visit http://%d.%d.%d.%d/",
        (uint8_t)(ip& 0xff),
        (uint8_t)((ip >> 8) & 0xff),
        (uint8_t)((ip >> 16) & 0xff),
        (uint8_t)((ip >> 24) & 0xff)
      );
    }

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else if (index == STRID_MAC) {
        // Convert MAC address directly into UTF-16
        for (unsigned i=0; i<sizeof(tud_network_mac_address); i++)
        {
          _desc_str[1+chr_count++] = "0123456789ABCDEF"[(tud_network_mac_address[i] >> 4) & 0xf];
          _desc_str[1+chr_count++] = "0123456789ABCDEF"[(tud_network_mac_address[i] >> 0) & 0xf];
        }
    } else {
        // Convert ASCII string into UTF-16

        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) return NULL;

        if (str == NULL) {
            str = (char*)string_desc_arr[index];
        }

        // Cap at max char
        chr_count = strlen(str);
        if (chr_count > 63) chr_count = 63;

        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

    return _desc_str;
}
