#include <inttypes.h>

#ifndef NFC_H_
#define NFC_H_

#define PN532_I2C_ADDRESS                   (0x48 >> 1)

uint32_t get_firmware_version();

int NFC_listen_for_tag();

void NFC_read_tag_data();

void NFC_read_tag_metadata(uint8_t *id);

void NFC_setup();

#endif /* NFC2_H_ */
