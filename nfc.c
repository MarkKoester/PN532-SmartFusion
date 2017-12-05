#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>
#include "nfc.h"
#include "drivers/mss_spi/mss_spi.h"
#include "drivers/mss_i2c/mss_i2c.h"
#include "drivers/mss_gpio/mss_gpio.h"

#define PN532_DEBUG

#define PN532_PREAMBLE                      (0x00)
#define PN532_STARTCODE1                    (0x00)
#define PN532_STARTCODE2                    (0xFF)
#define PN532_POSTAMBLE                     (0x00)
#define PN532_ACK_LENGTH                    (6)
#define PN532_FRAME_HEADER_BYTES            (7)

#define PN532_HOSTTOPN532                   (0xD4)
#define PN532_PN532TOHOST                   (0xD5)
#define PN532_ERROR                         (0x7F)

#define PN532_I2C_READBIT                   (0x01)
#define PN532_I2C_BUSY                      (0x00)
#define PN532_I2C_READY                     (0x01)
#define PN532_I2C_READYTIMEOUT              (20)

#define PN532_MIFARE_ISO14443A              (0x00)

#define PN532_COMMAND_INLISTPASSIVETARGET   (0x4A)
#define PN532_COMMAND_INDATAEXCHANGE        (0x40)
#define PN532_COMMAND_SAMCONFIGURATION      (0x14)

//ACK frame used to sync
uint8_t pn532_ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
uint8_t pn532_nack[] = {0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00};
uint8_t pn532response_firmwarevers[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};
static uint8_t frame_buffer[255 + PN532_FRAME_HEADER_BYTES + 1];

int is_ready()
{
	int irq = MSS_GPIO_get_inputs() & 1;
	return !irq;
}

void wait_ready()
{
	//TODO timeout
	while (!is_ready());
}

void I2C_send(uint8_t *buffer, uint8_t length)
{
	MSS_I2C_write
	(
		&g_mss_i2c1,
		PN532_I2C_ADDRESS,
		buffer,
		length,
		MSS_I2C_RELEASE_BUS
	);
	MSS_I2C_wait_complete(&g_mss_i2c1, MSS_I2C_NO_TIMEOUT);
}

// Retry while ready byte != 0x1
// Response has form:
//   [0x01], [0x00], [0x00, 0xFF], [0xLEN], [0xCHKLEN], [0xXX, 0xXX...], [0xCHKDATA], [0x00]
// Leading byte will be 1 when I2C slave is ready to send data
// Returns pointer to start of response frame
uint8_t *I2C_read(uint8_t length)
{
	// Make room for leading 1
	length += 1;

	//Not sure if this works but it should retry when the first byte isn't 1
	//We might want this to give up after a while so it doesn't hang
	do {
		MSS_I2C_read
		(
	    	&g_mss_i2c1,
	    	PN532_I2C_ADDRESS,
	    	frame_buffer,
	    	length,
	    	MSS_I2C_RELEASE_BUS
		);
		MSS_I2C_wait_complete(&g_mss_i2c1, MSS_I2C_NO_TIMEOUT);
	} while (frame_buffer[0] != 1);

	return frame_buffer + 1;
}

// Checks that PN532 sends ACK
// Similar to NFC_read but I think it has to be separate because the ACK isn't a standard frame
void send_ack()
{
	I2C_send(pn532_ack, 6);
}

void send_nack()
{
	I2C_send(pn532_nack, 6);
}



uint8_t *seek_start(uint8_t *buf)
{
	uint8_t prev = 0xFF;
	uint8_t current = 0xFF;
	uint16_t max_offset = 263;

	uint16_t offset = 0;
	while ((offset < max_offset) && !(prev == 0 && current == 0xFF)) {
		offset += 1;
		prev = current;
		current = *buf;
		buf += 1;
	}

	assert(offset < max_offset);
	return buf;
}

int read_ack()
{
	wait_ready();
	uint8_t *response = I2C_read(PN532_ACK_LENGTH);

	return (0 == strncmp((char *)response, (char *)pn532_ack, 6));
}

//Parse response frame, retransmit if necessary
//Return data and length
uint8_t NFC_read(uint8_t *read_data, uint8_t data_length)
{
	int8_t length;
	while (1) {
		wait_ready();
		uint8_t *response_pos = I2C_read(data_length + PN532_FRAME_HEADER_BYTES);

		//preamble - read random bytes until 00 FF
		//start code - parse 00 FF
		response_pos = seek_start(response_pos);

		//length - read length
		length = *response_pos++;
		//LCS - check that LCS + length == 0
		int8_t length_check = *response_pos++;

		if (length + length_check) {
			send_nack();
			continue;
		}

		int8_t data_sum = 0;

		//parse d5 (PN532 to controller indicator), add to accumulator
		if (*response_pos == PN532_ERROR) {
			//application level error
			printf("Application error\r\n");
			return 0;
		}
		assert(*response_pos == PN532_PN532TOHOST);
		data_sum += *response_pos++;
		//data - read length bytes. first byte of data is command code, add each to accumulator
		uint8_t i;
		for (i = 0; i < length - 1; i++) {
			read_data[i] = *response_pos;
			data_sum += *response_pos;
			response_pos++;
		}
		//DCS - check that accumulator + DCS == 0
		int8_t data_check = *response_pos++;
		if (data_sum + data_check) {
			send_nack();
			continue;
		}
		//postamble - arbitrary bytes
		send_ack();
		break;
	}

	return length - 1;
}

/*
 * Writes command to NFC and checks the ack
 * Returns 0 for success and 1 for error
 */
int NFC_write(uint8_t *write_data, uint8_t length)
{
	uint8_t index = 0;
	length += 1;

	frame_buffer[index++] = PN532_PREAMBLE;
	frame_buffer[index++] = PN532_PREAMBLE;
	frame_buffer[index++] = PN532_STARTCODE2;
	frame_buffer[index++] = length;
	frame_buffer[index++] = ~length + 1;

	uint8_t checksum = 0;
	checksum += PN532_HOSTTOPN532;
	frame_buffer[index++] = PN532_HOSTTOPN532;

	uint8_t i;
	for (i = 0; i < length - 1; i++) {
		frame_buffer[index++] = write_data[i];
		checksum += write_data[i];
	}
	frame_buffer[index++] = ~checksum + 1;
	frame_buffer[index++] = PN532_POSTAMBLE;

	I2C_send(frame_buffer, index);

	if (!read_ack()) {
		return 1;
	}
	return 0;
}

//[03] 32 01 06 07
uint32_t get_firmware_version()
{
	uint32_t response = 12;

	uint8_t cmd[] = { 0x02 };
	if (NFC_write(cmd, 1)) {
		return 0;
	}

	uint8_t read_buffer[5];
	NFC_read(read_buffer, 5);

	return response;
}

int NFC_listen_for_tag()
{
	uint8_t write_buf[3];
	write_buf[0] = PN532_COMMAND_INLISTPASSIVETARGET;
	write_buf[1] = 1;  // max 1 cards at once (we can set this to 2 later)
	write_buf[2] = PN532_MIFARE_ISO14443A;

	if (NFC_write(write_buf, 3)){
		//error handler here TODO
		return 1;
	}

	printf("Waiting for card...\r\n");
	return 0;
}

uint8_t in_data_exchange(uint8_t *write_data, uint8_t write_data_len, uint8_t *response)
{
	uint8_t write_len = write_data_len + 2;

	uint8_t write_buffer[write_len];
	write_buffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	write_buffer[1] = 1; // Target 1
	memcpy(write_buffer + 2, write_data, write_data_len);
	NFC_write(write_buffer, write_len);

	uint8_t read_data[20];
	uint8_t length = NFC_read(read_data, 20);
	memcpy(response, read_data + 1, length - 1);
	return length - 1;
}

int authenticate_mifare(uint8_t block, uint8_t *key, int key_num, uint8_t *uid)
{
	uint8_t write_buffer[12];
	write_buffer[0] = key_num ? 0x61 : 0x60; //MiFare authenticate with key A or B
	write_buffer[1] = block;
	memcpy(write_buffer + 2, key, 6);
	memcpy(write_buffer + 8, uid, 4);

	uint8_t read_data;
	in_data_exchange(write_buffer, 12, &read_data);

	if (read_data == 0x14) {
		printf("Invalid key\r\n");
		return 1;
	}
	if (read_data != 0x00) {
		printf("Authentication failed with code: %x\r\n", read_data);
		return 1;
	}
	return 0;
}

// 4b 01 01 00 04 08 04 02 5f 14 ab
void NFC_read_tag_metadata(uint8_t *id) {
	uint8_t read_buf[20];
	NFC_read(read_buf, 20);

	uint8_t num_targets = read_buf[1];
	printf("Found %d target\r\n", num_targets);

	uint16_t sens_res = read_buf[3] << 8 | read_buf[4];
	uint8_t sel_res = read_buf[5];
	uint8_t id_length = read_buf[6];
	assert(id_length == 4);
	memcpy(id, read_buf + 7, id_length);
}

// Read tag metadata and read data from tag 1

void NFC_read_tag_data(uint8_t *read_buf)
{
	uint8_t id[4];
	read_tag_metadata(id);

	uint8_t key[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };  // Mifare default key
	if (authenticate_mifare(5, key, 1, id)) {
		return;
	}

	uint8_t write_data[] = { 0x30, 0x05 };
	return in_data_exchange(write_data, 2, read_buf);
}

void NFC_setup()
{
	get_firmware_version();

	uint8_t write_buf[] = { PN532_COMMAND_SAMCONFIGURATION,
			              0x01,  // normal mode
			              0x14,  // timeout 50ms * 20 = 1 second
			              0x01   // use IRQ signal
	                    };

	uint8_t read_buf[1];

	NFC_write(write_buf, 4);
	NFC_read(read_buf, 1);

	assert(read_buf[0] == 0x15);
}
