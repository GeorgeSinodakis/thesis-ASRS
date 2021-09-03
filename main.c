#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>

///////////////////////////////////////////////motors
#define motor_PORT				PORTA
#define motor_DDR				DDRA
#define motor_PIN				PINA

#define x_direction_pin			0
#define x_step_pin				1

#define y_direction_pin			2
#define y_step_pin				3

#define H_in_pin				4
#define H_out_pin				5

#define tain_near_pin			6
#define tain_away_pin			7

////////////////////////////////////////////////

//////////////////////////////////////////////termatikoi
#define term_PORT				PORTD
#define term_DDR				DDRD
#define term_PIN				PIND

#define x_negative_term_pin		0
#define x_positive_term_pin		7

#define y_positive_term_pin		6
#define y_negative_term_pin		1

#define tain_near_term_pin		2
#define tain_away_term_pin		3

#define in_term_pin				4
#define out_term_pin			5
///////////////////////////////////////////////

///////////////////////////////////////////////spi
#define spi_PORT	PORTB
#define spi_DDR		DDRB
#define spi_PIN		PINB

#define CLK		7
#define MISO	6
#define MOSI	5
#define CS		4
//////////////////////////////////////////////

// x axis 40mm/rotation
// 80steps/mm
#define x					1
#define x_acc_dec_steps		800
#define x_ocr_max			51199 //312.5Hz = 5.8rpm = 3.9mm/s
#define x_ocr_min			5119 //3125Hz = 58.56rpm = 39.06mm/s
#define x_dir_positive		0
#define x_dir_negative		1
#define x_manual_period		1000	//microseconds

// y axis 8mm/rotation
// 400steps/mm
#define y					0
#define y_acc_dec_steps		800
#define y_ocr_max			51199	//312.5Hz = 5.8rpm = 0.78mm/s
#define y_ocr_min			3839	//4166.6Hz = 78.12rpm = 10.41mm/s
#define y_dir_positive		1	//direction=high=up
#define y_dir_negative		0	//direction=low=down
#define y_manual_period		200	//microseconds

#define PN532_PACKBUFFSIZ					64
#define PN532_PREAMBLE                      (0x00)
#define PN532_STARTCODE1                    (0x00)
#define PN532_STARTCODE2                    (0xFF)
#define PN532_POSTAMBLE                     (0x00)
#define PN532_HOSTTOPN532                   (0xD4)
#define PN532_PN532TOHOST                   (0xD5)
#define PN532_COMMAND_DIAGNOSE              (0x00)
#define PN532_COMMAND_GETFIRMWAREVERSION    (0x02)
#define PN532_COMMAND_GETGENERALSTATUS      (0x04)
#define PN532_COMMAND_READREGISTER          (0x06)
#define PN532_COMMAND_WRITEREGISTER         (0x08)
#define PN532_COMMAND_READGPIO              (0x0C)
#define PN532_COMMAND_WRITEGPIO             (0x0E)
#define PN532_COMMAND_SETSERIALBAUDRATE     (0x10)
#define PN532_COMMAND_SETPARAMETERS         (0x12)
#define PN532_COMMAND_SAMCONFIGURATION      (0x14)
#define PN532_COMMAND_POWERDOWN             (0x16)
#define PN532_COMMAND_RFCONFIGURATION       (0x32)
#define PN532_COMMAND_RFREGULATIONTEST      (0x58)
#define PN532_COMMAND_INJUMPFORDEP          (0x56)
#define PN532_COMMAND_INJUMPFORPSL          (0x46)
#define PN532_COMMAND_INLISTPASSIVETARGET   (0x4A)
#define PN532_COMMAND_INATR                 (0x50)
#define PN532_COMMAND_INPSL                 (0x4E)
#define PN532_COMMAND_INDATAEXCHANGE        (0x40)
#define PN532_COMMAND_INCOMMUNICATETHRU     (0x42)
#define PN532_COMMAND_INDESELECT            (0x44)
#define PN532_COMMAND_INRELEASE             (0x52)
#define PN532_COMMAND_INSELECT              (0x54)
#define PN532_COMMAND_INAUTOPOLL            (0x60)
#define PN532_COMMAND_TGINITASTARGET        (0x8C)
#define PN532_COMMAND_TGSETGENERALBYTES     (0x92)
#define PN532_COMMAND_TGGETDATA             (0x86)
#define PN532_COMMAND_TGSETDATA             (0x8E)
#define PN532_COMMAND_TGSETMETADATA         (0x94)
#define PN532_COMMAND_TGGETINITIATORCOMMAND (0x88)
#define PN532_COMMAND_TGRESPONSETOINITIATOR (0x90)
#define PN532_COMMAND_TGGETTARGETSTATUS     (0x8A)
#define PN532_RESPONSE_INDATAEXCHANGE       (0x41)
#define PN532_RESPONSE_INLISTPASSIVETARGET  (0x4B)
#define PN532_WAKEUP                        (0x55)
#define PN532_SPI_STATREAD                  (0x02)
#define PN532_SPI_DATAWRITE                 (0x01)
#define PN532_SPI_DATAREAD                  (0x03)
#define PN532_SPI_READY                     (0x01)
#define PN532_I2C_ADDRESS                   (0x48 >> 1)
#define PN532_I2C_READBIT                   (0x01)
#define PN532_I2C_BUSY                      (0x00)
#define PN532_I2C_READY                     (0x01)
#define PN532_I2C_READYTIMEOUT              (20)
#define PN532_MIFARE_ISO14443A              (0x00)
#define MIFARE_CMD_AUTH_A                   (0x60)
#define MIFARE_CMD_AUTH_B                   (0x61)
#define MIFARE_CMD_READ                     (0x30)
#define MIFARE_CMD_WRITE                    (0xA0)
#define MIFARE_CMD_TRANSFER                 (0xB0)
#define MIFARE_CMD_DECREMENT                (0xC0)
#define MIFARE_CMD_INCREMENT                (0xC1)
#define MIFARE_CMD_STORE                    (0xC2)
#define MIFARE_ULTRALIGHT_CMD_WRITE         (0xA2)
#define NDEF_URIPREFIX_NONE                 (0x00)
#define NDEF_URIPREFIX_HTTP_WWWDOT          (0x01)
#define NDEF_URIPREFIX_HTTPS_WWWDOT         (0x02)
#define NDEF_URIPREFIX_HTTP                 (0x03)
#define NDEF_URIPREFIX_HTTPS                (0x04)
#define NDEF_URIPREFIX_TEL                  (0x05)
#define NDEF_URIPREFIX_MAILTO               (0x06)
#define NDEF_URIPREFIX_FTP_ANONAT           (0x07)
#define NDEF_URIPREFIX_FTP_FTPDOT           (0x08)
#define NDEF_URIPREFIX_FTPS                 (0x09)
#define NDEF_URIPREFIX_SFTP                 (0x0A)
#define NDEF_URIPREFIX_SMB                  (0x0B)
#define NDEF_URIPREFIX_NFS                  (0x0C)
#define NDEF_URIPREFIX_FTP                  (0x0D)
#define NDEF_URIPREFIX_DAV                  (0x0E)
#define NDEF_URIPREFIX_NEWS                 (0x0F)
#define NDEF_URIPREFIX_TELNET               (0x10)
#define NDEF_URIPREFIX_IMAP                 (0x11)
#define NDEF_URIPREFIX_RTSP                 (0x12)
#define NDEF_URIPREFIX_URN                  (0x13)
#define NDEF_URIPREFIX_POP                  (0x14)
#define NDEF_URIPREFIX_SIP                  (0x15)
#define NDEF_URIPREFIX_SIPS                 (0x16)
#define NDEF_URIPREFIX_TFTP                 (0x17)
#define NDEF_URIPREFIX_BTSPP                (0x18)
#define NDEF_URIPREFIX_BTL2CAP              (0x19)
#define NDEF_URIPREFIX_BTGOEP               (0x1A)
#define NDEF_URIPREFIX_TCPOBEX              (0x1B)
#define NDEF_URIPREFIX_IRDAOBEX             (0x1C)
#define NDEF_URIPREFIX_FILE                 (0x1D)
#define NDEF_URIPREFIX_URN_EPC_ID           (0x1E)
#define NDEF_URIPREFIX_URN_EPC_TAG          (0x1F)
#define NDEF_URIPREFIX_URN_EPC_PAT          (0x20)
#define NDEF_URIPREFIX_URN_EPC_RAW          (0x21)
#define NDEF_URIPREFIX_URN_EPC              (0x22)
#define NDEF_URIPREFIX_URN_NFC              (0x23)
#define PN532_GPIO_P30                      (0)
#define PN532_GPIO_VALIDATIONBIT            (0x80)
#define PN532_GPIO_P31                      (1)
#define PN532_GPIO_P32                      (2)
#define PN532_GPIO_P33                      (3)
#define PN532_GPIO_P35                      (5)
#define PN532_GPIO_P34                      (4)

const uint16_t ocr_x[800] PROGMEM	=
{	51199, 50629, 50071, 49525, 48992, 48469, 47958, 47457, 46967, 46486, 46016, 45554, 45102, 44659, 44225, 43799, 43381, 42970, 42568, 42173, 41785, 41405,
	41031, 40664, 40303, 39949, 39601, 39259, 38923, 38592, 38267, 37948, 37634, 37325, 37021, 36721, 36427, 36138, 35852, 35572, 35296, 35024, 34756, 34492,
	34232, 33976, 33724, 33476, 33231, 32990, 32752, 32518, 32287, 32059, 31835, 31613, 31395, 31179, 30967, 30757, 30551, 30347, 30145, 29947, 29751, 29557,
	29366, 29178, 28992, 28808, 28627, 28447, 28270, 28096, 27923, 27753, 27584, 27418, 27253, 27091, 26930, 26772, 26615, 26460, 26307, 26156, 26006, 25858,
	25712, 25567, 25424, 25283, 25143, 25004, 24868, 24732, 24598, 24466, 24335, 24205, 24077, 23950, 23825, 23701, 23578, 23456, 23335, 23216, 23098, 22981,
	22866, 22751, 22638, 22526, 22415, 22305, 22196, 22088, 21981, 21875, 21771, 21667, 21564, 21462, 21361, 21261, 21162, 21064, 20967, 20871, 20775, 20681,
	20587, 20494, 20402, 20311, 20221, 20131, 20043, 19955, 19867, 19781, 19695, 19610, 19526, 19442, 19360, 19277, 19196, 19115, 19035, 18956, 18877, 18799,
	18722, 18645, 18569, 18493, 18418, 18344, 18270, 18197, 18124, 18052, 17981, 17910, 17840, 17770, 17701, 17632, 17564, 17496, 17429, 17363, 17297, 17231,
	17166, 17101, 17037, 16974, 16910, 16848, 16786, 16724, 16662, 16602, 16541, 16481, 16422, 16363, 16304, 16246, 16188, 16130, 16073, 16017, 15960, 15904,
	15849, 15794, 15739, 15685, 15631, 15577, 15524, 15471, 15419, 15367, 15315, 15263, 15212, 15162, 15111, 15061, 15011, 14962, 14913, 14864, 14816, 14768,
	14720, 14672, 14625, 14578, 14531, 14485, 14439, 14393, 14348, 14303, 14258, 14213, 14169, 14125, 14081, 14038, 13994, 13952, 13909, 13866, 13824, 13782,
	13741, 13699, 13658, 13617, 13576, 13536, 13496, 13456, 13416, 13377, 13337, 13298, 13260, 13221, 13183, 13145, 13107, 13069, 13031, 12994, 12957, 12920,
	12884, 12847, 12811, 12775, 12739, 12704, 12668, 12633, 12598, 12563, 12528, 12494, 12460, 12426, 12392, 12358, 12325, 12291, 12258, 12225, 12192, 12160,
	12127, 12095, 12063, 12031, 11999, 11968, 11936, 11905, 11874, 11843, 11812, 11781, 11751, 11721, 11691, 11661, 11631, 11601, 11572, 11542, 11513, 11484,
	11455, 11426, 11397, 11369, 11341, 11312, 11284, 11256, 11228, 11201, 11173, 11146, 11119, 11091, 11064, 11038, 11011, 10984, 10958, 10931, 10905, 10879,
	10853, 10827, 10801, 10776, 10750, 10725, 10700, 10675, 10650, 10625, 10600, 10575, 10551, 10526, 10502, 10478, 10454, 10430, 10406, 10382, 10358, 10335,
	10311, 10288, 10265, 10242, 10219, 10196, 10173, 10150, 10127, 10105, 10083, 10060, 10038, 10016, 9994, 9972, 9950, 9928, 9907, 9885, 9864, 9842, 9821,
	9800, 9779, 9758, 9737, 9716, 9695, 9675, 9654, 9634, 9613, 9593, 9573, 9553, 9533, 9513, 9493, 9473, 9453, 9434, 9414, 9395, 9375, 9356, 9337, 9318, 9299,
	9280, 9261, 9242, 9223, 9204, 9186, 9167, 9149, 9130, 9112, 9094, 9076, 9058, 9040, 9022, 9004, 8986, 8968, 8951, 8933, 8915, 8898, 8881, 8863, 8846, 8829,
	8812, 8795, 8778, 8761, 8744, 8727, 8710, 8694, 8677, 8661, 8644, 8628, 8611, 8595, 8579, 8563, 8547, 8531, 8515, 8499, 8483, 8467, 8451, 8436, 8420, 8404,
	8389, 8373, 8358, 8343, 8327, 8312, 8297, 8282, 8267, 8252, 8237, 8222, 8207, 8192, 8177, 8163, 8148, 8134, 8119, 8105, 8090, 8076, 8061, 8047, 8033, 8019,
	8005, 7991, 7977, 7963, 7949, 7935, 7921, 7907, 7893, 7880, 7866, 7852, 7839, 7825, 7812, 7799, 7785, 7772, 7759, 7745, 7732, 7719, 7706, 7693, 7680, 7667,
	7654, 7641, 7628, 7616, 7603, 7590, 7578, 7565, 7552, 7540, 7527, 7515, 7502, 7490, 7478, 7465, 7453, 7441, 7429, 7417, 7405, 7393, 7381, 7369, 7357, 7345,
	7333, 7321, 7309, 7298, 7286, 7274, 7263, 7251, 7239, 7228, 7217, 7205, 7194, 7182, 7171, 7160, 7148, 7137, 7126, 7115, 7104, 7093, 7082, 7071, 7060, 7049,
	7038, 7027, 7016, 7005, 6994, 6984, 6973, 6962, 6952, 6941, 6930, 6920, 6909, 6899, 6888, 6878, 6868, 6857, 6847, 6837, 6826, 6816, 6806, 6796, 6785, 6775,
	6765, 6755, 6745, 6735, 6725, 6715, 6705, 6695, 6686, 6676, 6666, 6656, 6647, 6637, 6627, 6617, 6608, 6598, 6589, 6579, 6570, 6560, 6551, 6541, 6532, 6522,
	6513, 6504, 6495, 6485, 6476, 6467, 6458, 6448, 6439, 6430, 6421, 6412, 6403, 6394, 6385, 6376, 6367, 6358, 6349, 6340, 6332, 6323, 6314, 6305, 6297, 6288,
	6279, 6270, 6262, 6253, 6245, 6236, 6228, 6219, 6210, 6202, 6194, 6185, 6177, 6168, 6160, 6152, 6143, 6135, 6127, 6118, 6110, 6102, 6094, 6086, 6078, 6069,
	6061, 6053, 6045, 6037, 6029, 6021, 6013, 6005, 5997, 5989, 5982, 5974, 5966, 5958, 5950, 5942, 5935, 5927, 5919, 5912, 5904, 5896, 5889, 5881, 5873, 5866,
	5858, 5851, 5843, 5836, 5828, 5821, 5813, 5806, 5798, 5791, 5784, 5776, 5769, 5762, 5754, 5747, 5740, 5733, 5725, 5718, 5711, 5704, 5697, 5689, 5682, 5675,
	5668, 5661, 5654, 5647, 5640, 5633, 5626, 5619, 5612, 5605, 5598, 5591, 5585, 5578, 5571, 5564, 5557, 5550, 5544, 5537, 5530, 5523, 5517, 5510, 5503, 5497,
	5490, 5483, 5477, 5470, 5464, 5457, 5451, 5444, 5438, 5431, 5425, 5418, 5412, 5405, 5399, 5392, 5386, 5380, 5373, 5367, 5361, 5354, 5348, 5342, 5335, 5329,
	5323, 5317, 5310, 5304, 5298, 5292, 5286, 5280, 5273, 5267, 5261, 5255, 5249, 5243, 5237, 5231, 5225, 5219, 5213, 5207, 5201, 5195, 5189, 5183, 5177, 5171,
	5166, 5160, 5154, 5148, 5142, 5136, 5131, 5125, 5119
};

const uint16_t ocr_y[800] PROGMEM	=
{	51199, 50421, 49666, 48933, 48222, 47531, 46859, 46206, 45571, 44954, 44353, 43767, 43197, 42642, 42101, 41573, 41058, 40556, 40067, 39588, 39121, 38665,
	38220, 37784, 37359, 36943, 36536, 36138, 35748, 35367, 34994, 34629, 34271, 33920, 33577, 33240, 32911, 32587, 32270, 31959, 31654, 31355, 31061, 30773,
	30490, 30213, 29940, 29672, 29409, 29150, 28896, 28647, 28402, 28160, 27923, 27690, 27461, 27235, 27014, 26795, 26581, 26369, 26161, 25957, 25755, 25557,
	25361, 25169, 24979, 24792, 24608, 24427, 24249, 24073, 23899, 23728, 23560, 23394, 23230, 23068, 22909, 22752, 22597, 22444, 22293, 22144, 21997, 21852,
	21709, 21568, 21429, 21291, 21155, 21021, 20889, 20758, 20629, 20501, 20376, 20251, 20128, 20007, 19887, 19768, 19651, 19535, 19421, 19308, 19196, 19086,
	18977, 18869, 18762, 18656, 18552, 18449, 18347, 18246, 18146, 18047, 17950, 17853, 17757, 17663, 17569, 17477, 17385, 17294, 17205, 17116, 17028, 16941,
	16855, 16770, 16685, 16602, 16519, 16437, 16356, 16276, 16196, 16118, 16040, 15963, 15886, 15810, 15735, 15661, 15587, 15515, 15442, 15371, 15300, 15230,
	15160, 15091, 15023, 14955, 14888, 14821, 14755, 14690, 14625, 14561, 14497, 14434, 14372, 14310, 14248, 14187, 14127, 14067, 14007, 13949, 13890, 13832,
	13775, 13718, 13661, 13605, 13550, 13494, 13440, 13386, 13332, 13278, 13225, 13173, 13121, 13069, 13018, 12967, 12916, 12866, 12817, 12767, 12718, 12670,
	12621, 12574, 12526, 12479, 12432, 12386, 12340, 12294, 12249, 12203, 12159, 12114, 12070, 12026, 11983, 11940, 11897, 11854, 11812, 11770, 11729, 11687,
	11646, 11606, 11565, 11525, 11485, 11445, 11406, 11367, 11328, 11290, 11251, 11213, 11175, 11138, 11101, 11064, 11027, 10990, 10954, 10918, 10882, 10846,
	10811, 10776, 10741, 10706, 10672, 10638, 10604, 10570, 10536, 10503, 10470, 10437, 10404, 10372, 10339, 10307, 10275, 10243, 10212, 10181, 10149, 10118,
	10088, 10057, 10027, 9996, 9966, 9936, 9907, 9877, 9848, 9819, 9790, 9761, 9732, 9704, 9676, 9647, 9619, 9592, 9564, 9536, 9509, 9482, 9455, 9428, 9401, 9375,
	9348, 9322, 9296, 9270, 9244, 9218, 9193, 9167, 9142, 9117, 9092, 9067, 9042, 9018, 8993, 8969, 8945, 8921, 8897, 8873, 8849, 8826, 8802, 8779, 8756, 8733,
	8710, 8687, 8664, 8642, 8619, 8597, 8575, 8553, 8531, 8509, 8487, 8465, 8444, 8422, 8401, 8380, 8359, 8338, 8317, 8296, 8275, 8255, 8234, 8214, 8193, 8173,
	8153, 8133, 8113, 8093, 8074, 8054, 8035, 8015, 7996, 7977, 7958, 7938, 7920, 7901, 7882, 7863, 7845, 7826, 7808, 7789, 7771, 7753, 7735, 7717, 7699, 7681,
	7663, 7646, 7628, 7611, 7593, 7576, 7558, 7541, 7524, 7507, 7490, 7473, 7457, 7440, 7423, 7407, 7390, 7374, 7357, 7341, 7325, 7309, 7293, 7277, 7261, 7245,
	7229, 7213, 7198, 7182, 7166, 7151, 7136, 7120, 7105, 7090, 7075, 7060, 7045, 7030, 7015, 7000, 6985, 6971, 6956, 6941, 6927, 6912, 6898, 6884, 6870, 6855,
	6841, 6827, 6813, 6799, 6785, 6771, 6758, 6744, 6730, 6716, 6703, 6689, 6676, 6662, 6649, 6636, 6623, 6609, 6596, 6583, 6570, 6557, 6544, 6531, 6518, 6506,
	6493, 6480, 6468, 6455, 6442, 6430, 6418, 6405, 6393, 6380, 6368, 6356, 6344, 6332, 6320, 6308, 6296, 6284, 6272, 6260, 6248, 6236, 6225, 6213, 6201, 6190,
	6178, 6167, 6155, 6144, 6133, 6121, 6110, 6099, 6088, 6076, 6065, 6054, 6043, 6032, 6021, 6010, 6000, 5989, 5978, 5967, 5956, 5946, 5935, 5924, 5914, 5903,
	5893, 5882, 5872, 5862, 5851, 5841, 5831, 5820, 5810, 5800, 5790, 5780, 5770, 5760, 5750, 5740, 5730, 5720, 5710, 5700, 5691, 5681, 5671, 5661, 5652, 5642,
	5633, 5623, 5614, 5604, 5595, 5585, 5576, 5566, 5557, 5548, 5539, 5529, 5520, 5511, 5502, 5493, 5484, 5475, 5465, 5457, 5448, 5439, 5430, 5421, 5412, 5403,
	5394, 5386, 5377, 5368, 5359, 5351, 5342, 5334, 5325, 5317, 5308, 5300, 5291, 5283, 5274, 5266, 5258, 5249, 5241, 5233, 5224, 5216, 5208, 5200, 5192, 5184,
	5175, 5167, 5159, 5151, 5143, 5135, 5127, 5120, 5112, 5104, 5096, 5088, 5080, 5073, 5065, 5057, 5049, 5042, 5034, 5026, 5019, 5011, 5004, 4996, 4989, 4981,
	4974, 4966, 4959, 4951, 4944, 4937, 4929, 4922, 4915, 4907, 4900, 4893, 4886, 4878, 4871, 4864, 4857, 4850, 4843, 4836, 4829, 4822, 4815, 4808, 4801, 4794,
	4787, 4780, 4773, 4766, 4759, 4753, 4746, 4739, 4732, 4726, 4719, 4712, 4705, 4699, 4692, 4685, 4679, 4672, 4666, 4659, 4653, 4646, 4640, 4633, 4627, 4620,
	4614, 4607, 4601, 4595, 4588, 4582, 4576, 4569, 4563, 4557, 4550, 4544, 4538, 4532, 4526, 4519, 4513, 4507, 4501, 4495, 4489, 4483, 4477, 4471, 4465, 4459,
	4453, 4447, 4441, 4435, 4429, 4423, 4417, 4411, 4405, 4399, 4394, 4388, 4382, 4376, 4370, 4365, 4359, 4353, 4348, 4342, 4336, 4330, 4325, 4319, 4314, 4308,
	4302, 4297, 4291, 4286, 4280, 4275, 4269, 4264, 4258, 4253, 4247, 4242, 4236, 4231, 4226, 4220, 4215, 4210, 4204, 4199, 4194, 4188, 4183, 4178, 4172, 4167,
	4162, 4157, 4152, 4146, 4141, 4136, 4131, 4126, 4121, 4115, 4110, 4105, 4100, 4095, 4090, 4085, 4080, 4075, 4070, 4065, 4060, 4055, 4050, 4045, 4040, 4035,
	4030, 4026, 4021, 4016, 4011, 4006, 4001, 3996, 3992, 3987, 3982, 3977, 3972, 3968, 3963, 3958, 3954, 3949, 3944, 3939, 3935, 3930, 3925, 3921, 3916, 3912,
	3907, 3902, 3898, 3893, 3889, 3884, 3879, 3875, 3870, 3866, 3861, 3857, 3852, 3848, 3844, 3839
};

volatile uint32_t x_position = 20000, y_position = 100000, regular_steps = 0, acc_steps = 0, dec_steps = 0;
volatile uint8_t axis = x, end = 0; //gia na kserei to interrupt poio moter na kinisei,gia na kserei h move_to pote teleiwse h kinish

//|---|---|---|
//| 6 | 7 | 8 |
//|---|---|---|
//| 3 | 4 | 5 |
//|---|---|---|
//| 0 | 1 | 2 | 9
//|---|---|---|---
//|           |
volatile uint32_t x_coordinates[10] = {5, 125, 240, 5, 125, 240, 5, 125, 240, 373}; //teleutaia tou tainiodromou SE MM!!!!
volatile uint32_t y_coordinates[10] = {50, 50, 50, 165, 165, 165, 282, 282, 282, 25}; //teleutaia tou tainiodromou	SE MM!!!!
volatile uint32_t ids[9] = {0x50c72d52, 0x80f03752, 0x200e2052, 0x9392d66a, 0x839fde6a, 0x53a6dd6a, 0x70862e52, 0x101b2752, 0x12592a13}; //ta id twn kartwn

volatile uint8_t pn532_packetbuffer[PN532_PACKBUFFSIZ];
uint8_t pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
uint8_t pn532response_firmwarevers[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};

uint8_t spi_tranceiver (uint8_t data)
{
	SPDR = data;
	while (!(SPSR & (1 << SPIF) ));
	return (SPDR);
}
void pn532_readspidata(uint8_t* buff, uint8_t n)
{
	PORTB &= ~(1 << CS); // write LOW to CS pin
	_delay_us(2000);
	spi_tranceiver(PN532_SPI_DATAREAD);

	for (uint8_t i = 0; i < n; i++)
	{
		_delay_us(1000);
		buff[i] = spi_tranceiver(0);
	}

	PORTB |= (1 << CS); // write HIGH to CS pin
}
uint8_t pn532_spi_readack(void)
{
	uint8_t ackbuff[6];

	pn532_readspidata(ackbuff, 6);

	return (0 == strncmp((char*) ackbuff, (char*) pn532ack, 6)) ? 1 : 0;
}
uint8_t pn532_readspistatus(void)
{
	PORTB &= ~(1 << CS); // write LOW to CS pin
	_delay_us(2000);

	spi_tranceiver(PN532_SPI_STATREAD);

	uint8_t s = spi_tranceiver(0);

	PORTB |= (1 << CS); // write HIGH to CS pin
	return s;
}
void pn532_spiwritecommand(uint8_t* cmd, uint8_t cmdlen)
{
	uint8_t checksum;
	cmdlen++;
	PORTB &= ~(1 << CS); // write LOW to CS pin
	_delay_us(2000);

	spi_tranceiver(PN532_SPI_DATAWRITE);
	checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
	spi_tranceiver(PN532_PREAMBLE);
	spi_tranceiver(PN532_PREAMBLE);
	spi_tranceiver(PN532_STARTCODE2);
	spi_tranceiver(cmdlen);
	spi_tranceiver(~cmdlen + 1);
	spi_tranceiver(PN532_HOSTTOPN532);

	checksum += PN532_HOSTTOPN532;
	for (uint8_t i = 0; i < cmdlen - 1; i++)
	{
		spi_tranceiver(cmd[i]);
		checksum += cmd[i];
	}

	spi_tranceiver(~checksum);
	spi_tranceiver(PN532_POSTAMBLE);
	PORTB |= (1 << CS); // write HIGH to CS pin
}
uint8_t pn532_sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout)
{
	uint16_t timer = 0;

	// write the command
	pn532_spiwritecommand(cmd, cmdlen);

	// wait for the chip to say it's ready
	while (pn532_readspistatus() != PN532_SPI_READY)
	{
		if (timeout != 0)
		{
			timer += 10;
			if (timer > timeout)
			{
				return 0;
			}
		}
		_delay_ms(10);
	}

	// read acknowledgement
	if (0 == pn532_spi_readack())
	{
		return 0;
	}

	timer = 0;
	// wait for the chip to say it's ready
	while (pn532_readspistatus() != PN532_SPI_READY)
	{
		if (timeout != 0)
		{
			timer += 10;
			if (timer > timeout)
			{
				return 0;
			}
		}
		_delay_ms(10);
	}

	return 1;
}
uint8_t pn532_getFirmwareVersion(void)
{
	pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

	if (0 == pn532_sendCommandCheckAck(pn532_packetbuffer, 1, 1000)) return 0;

	// read data packet
	pn532_readspidata(pn532_packetbuffer, 12);

	if (pn532_packetbuffer[6] != 50) return 0;
	if (pn532_packetbuffer[7] != 1) return 0;
	if (pn532_packetbuffer[8] != 6) return 0;
	if (pn532_packetbuffer[9] != 7) return 0;

	return 1;
}
uint8_t pn532_SAMConfig(void) {
	pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
	pn532_packetbuffer[1] = 0x01; // normal mode
	pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
	pn532_packetbuffer[3] = 0x01; // use IRQ pin!

	if (0 == pn532_sendCommandCheckAck(pn532_packetbuffer, 4, 1000)) return 0;

	// read data packet
	pn532_readspidata(pn532_packetbuffer, 8);

	return (pn532_packetbuffer[5] == 0x15);
}
uint32_t pn532_readPassiveTargetID(uint8_t cardbaudrate)
{
	uint32_t i = 0;
	pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
	pn532_packetbuffer[1] = 1; // max 1 card at once
	pn532_packetbuffer[2] = cardbaudrate;

	if (0 == pn532_sendCommandCheckAck(pn532_packetbuffer, 3, 1000)) return 0x00; // no cards read

	// read data packet
	pn532_readspidata(pn532_packetbuffer, 20);

	if (pn532_packetbuffer[7] != 1) return 0;

	i = pn532_packetbuffer[13];
	i <<= 8;
	i |= pn532_packetbuffer[14];
	i <<= 8;
	i |= pn532_packetbuffer[15];
	i <<= 8;
	i |= pn532_packetbuffer[16];

	return i;
}
uint8_t pn532_authenticateBlock(uint8_t cardnumber, uint32_t cid, uint8_t blockaddress, uint8_t authtype, uint8_t* keys)
{
	pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	pn532_packetbuffer[1] = cardnumber;

	if (authtype == MIFARE_CMD_AUTH_A)
	{
		pn532_packetbuffer[2] = MIFARE_CMD_AUTH_A;
	}
	else
	{
		pn532_packetbuffer[2] = MIFARE_CMD_AUTH_B;
	}
	pn532_packetbuffer[3] = blockaddress; // Address can be 0-63 for MIFARE 1K card

	pn532_packetbuffer[4] = keys[0];
	pn532_packetbuffer[5] = keys[1];
	pn532_packetbuffer[6] = keys[2];
	pn532_packetbuffer[7] = keys[3];
	pn532_packetbuffer[8] = keys[4];
	pn532_packetbuffer[9] = keys[5];

	pn532_packetbuffer[10] = ((cid >> 24) & 0xFF);
	pn532_packetbuffer[11] = ((cid >> 16) & 0xFF);
	pn532_packetbuffer[12] = ((cid >> 8) & 0xFF);
	pn532_packetbuffer[13] = ((cid >> 0) & 0xFF);

	if (0 == pn532_sendCommandCheckAck(pn532_packetbuffer, 14, 1000)) return 0;

	// read data packet
	pn532_readspidata(pn532_packetbuffer, 2 + 6);

	if ((pn532_packetbuffer[6] = 0x41) && (pn532_packetbuffer[7] == 0x00))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
uint8_t pn532_readMemoryBlock(uint8_t cardnumber, uint8_t blockaddress, uint8_t* block)
{
	pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	pn532_packetbuffer[1] = cardnumber;
	pn532_packetbuffer[2] = MIFARE_CMD_READ;
	pn532_packetbuffer[3] = blockaddress;

	if (0 == pn532_sendCommandCheckAck(pn532_packetbuffer, 4, 1000)) return 0;

	// read data packet
	pn532_readspidata(pn532_packetbuffer, 18 + 6);
	for (uint8_t i = 8; i < 18 + 6; i++)
	{
		block[i - 8] = pn532_packetbuffer[i];
	}

	if ((pn532_packetbuffer[6] == 0x41) && (pn532_packetbuffer[7] == 0x00))
	{
		return 1; // read successful
	}
	else
	{
		return 0;
	}
}
uint8_t pn532_writeMemoryBlock(uint8_t cardnumber, uint8_t blockaddress, uint8_t* block)
{
	pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	pn532_packetbuffer[1] = cardnumber;
	pn532_packetbuffer[2] = MIFARE_CMD_WRITE;
	pn532_packetbuffer[3] = blockaddress;

	for (uint8_t byte = 0; byte < 16; byte++)
	{
		pn532_packetbuffer[4 + byte] = block[byte];
	}

	if (0 == pn532_sendCommandCheckAck(pn532_packetbuffer, 4 + 16, 1000)) return 0;

	// read data packet
	pn532_readspidata(pn532_packetbuffer, 2 + 6);

	if ((pn532_packetbuffer[6] == 0x41) && (pn532_packetbuffer[7] == 0x00))
	{
		return 1; // write successful
	}
	else
	{
		return 0;
	}
}
uint8_t pn532_begin(void)
{
	PORTB &= ~(1 << CS);
	_delay_ms(1000);
	pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
	pn532_sendCommandCheckAck(pn532_packetbuffer, 1, 1000);
	PORTB |= 1 << CS;
	if (1 != pn532_getFirmwareVersion())
	{
		return 1; //getFirmwareVersion error
	}
	if (1 != pn532_SAMConfig())
	{
		return 2; //SAMConfig error
	}
	return 0;
}

void uart_send_number(uint32_t value, uint8_t format)
{
	char str[10] = {};
	ultoa(value, str, format);
	uint8_t i = 0;
	while (str[i] != '\0')
	{
		while ( !( UCSRA & (1 << UDRE)) );
		UDR = str[i];
		i++;
	}
}
void uart_send_number_ln(uint32_t value, uint8_t format)
{
	char str[10] = {};
	ultoa(value, str, format);
	uint8_t i = 0;
	while (str[i] != '\0')
	{
		while ( !( UCSRA & (1 << UDRE)) );
		UDR = str[i];
		i++;
	}
	while ( !( UCSRA & (1 << UDRE)) );
	UDR = 10;	//new line
	while ( !( UCSRA & (1 << UDRE)) );
	UDR = 13;	//carriage return
}
void uart_send_string_ln( char *a)
{
	while (*a != '\0')
	{
		while ( !( UCSRA & (1 << UDRE)) );
		UDR = (*a);
		a++;
	}
	while ( !( UCSRA & (1 << UDRE)) );
	UDR = 10;	//new line
	while ( !( UCSRA & (1 << UDRE)) );
	UDR = 13;	//carriage return
}
void uart_send_string( char *a)
{
	while (*a != '\0')
	{
		while ( !( UCSRA & (1 << UDRE)) );
		UDR = (*a);
		a++;
	}
}
void uart_send_character(char a)
{
	while ( !( UCSRA & (1 << UDRE)) );
	UDR = (a);
}
void uart_send_character_ln(char a)
{
	while ( !( UCSRA & (1 << UDRE)) );
	UDR = (a);
	while ( !( UCSRA & (1 << UDRE)) );
	UDR = 10;	//new line
	while ( !( UCSRA & (1 << UDRE)) );
	UDR = 13;	//carriage return
}
void uart_init(void)
{
	UCSRA = 1 << U2X; //setting uart 8bits,1stop,no parity,2Mbaud
	UCSRB = 1 << RXEN | 1 << TXEN;
	UCSRC = 1 << URSEL | 1 << UCSZ1 | 1 << UCSZ0;
	UBRRH = 0;
	UBRRL = 0;
}

ISR(TIMER1_COMPA_vect)	//kanei olh thn kinisi
{
	if (axis == x)
	{
		if ((!(term_PIN & (1 << x_negative_term_pin))) && (!(term_PIN & (1 << x_positive_term_pin))))
		{
			if (acc_steps)
			{
				motor_PORT |= 1 << x_step_pin;
				OCR1A = pgm_read_dword(& ocr_x[x_acc_dec_steps - acc_steps]);
				acc_steps--;
			}
			else if (regular_steps)
			{
				motor_PORT |= 1 << x_step_pin;
				regular_steps--;
			}
			else if (dec_steps)
			{
				motor_PORT |= 1 << x_step_pin;
				OCR1A = pgm_read_dword(& ocr_x[dec_steps - 1]);
				dec_steps--;
			}
			else
			{
				end = 1;
			}
		}
		else
		{
			end = 1;
		}
	}
	else if (axis == y)
	{
		if ((!(term_PIN & (1 << y_positive_term_pin))) && (!(term_PIN & (1 << y_negative_term_pin))))
		{
			if (acc_steps)
			{
				motor_PORT |= 1 << y_step_pin;
				OCR1A = pgm_read_dword(& ocr_y[y_acc_dec_steps - acc_steps]);
				acc_steps--;
			}
			else if (regular_steps)
			{
				motor_PORT |= 1 << y_step_pin;
				regular_steps--;
			}
			else if (dec_steps)
			{
				motor_PORT |= 1 << y_step_pin;
				OCR1A = pgm_read_dword(& ocr_y[dec_steps - 1]);
				dec_steps--;
			}
			else
			{
				end = 1;
			}
		}
		else
		{
			end = 1;
		}
	}
	motor_PORT &= ~(1 << x_step_pin | 1 << y_step_pin);
}
void rel_move_to(int32_t mm_x, int32_t mm_y)	//kinei ton peronoforo me sxetikes syntetagmenes
{
	mm_x *= 80;
	mm_y *= 400;
	move_to((x_position + mm_x) / 80, (y_position + mm_y) / 400);
	return;
}
void move_to(uint32_t target_x, uint32_t target_y)	//paei ton peronoforo ekei poy tou zhtame
{
	target_x *= 80;
	target_y *= 400;
	uint32_t difference = 0;
	////////////////////////////////////////////// x axis/////////////////////////////////////////////
	if (target_x != x_position)
	{
		motor_PORT &= ~(1 << x_step_pin);
		axis = x;
		OCR1A = x_ocr_max;
		if (target_x < x_position)	//kinhsh pros ta arnitika
		{
#if(x_dir_negative)
			{
				motor_PORT |= (1 << x_direction_pin);
			}
#else
			{
				motor_PORT &= ~(1 << x_direction_pin);
			}
#endif
			difference = x_position - target_x;
			x_position = target_x;
			if (difference <= (2 * x_acc_dec_steps))	//an ta vimata einai ligotera apo oso xreiazetai gia na epitaxinei
			{	//den tha epitaxinei, tha exei statherh taxitita
				acc_steps = 0;
				regular_steps = difference;
				dec_steps = 0;
			}
			else
			{
				regular_steps = difference - (2 * x_acc_dec_steps);
				acc_steps = x_acc_dec_steps;
				dec_steps = x_acc_dec_steps;
			}
		}
		else	//kinisi pros ta thetika
		{
#if(x_dir_positive)
			{
				motor_PORT |= (1 << x_direction_pin);
			}
#else
			{
				motor_PORT &= ~(1 << x_direction_pin);
			}
#endif
			difference = target_x - x_position;
			x_position = target_x;
			if (difference <= (2 * x_acc_dec_steps))	//an ta vimata einai ligotera apo oso xreiazetai gia na epitaxinei
			{	//den tha epitaxinei, tha exei statherh taxitita
				acc_steps = 0;
				regular_steps = difference;
				dec_steps = 0;
			}
			else
			{
				regular_steps = difference - (2 * x_acc_dec_steps);
				acc_steps = x_acc_dec_steps;
				dec_steps = x_acc_dec_steps;
			}
		}
		end = 0;
		sei();
		while (!end);
		cli();
	}
	////////////////////////////////////////////// y axis/////////////////////////////////////////////
	if (target_y != y_position)
	{
		motor_PORT &= ~(1 << y_step_pin);
		axis = y;
		OCR1A = y_ocr_max;
		if (target_y < y_position)	//kinhsh pros ta arnitika
		{
#if(y_dir_negative)
			{
				motor_PORT |= (1 << y_direction_pin);
			}
#else
			{
				motor_PORT &= ~(1 << y_direction_pin);
			}
#endif
			difference = y_position - target_y;
			y_position = target_y;
			if (difference <= (2 * y_acc_dec_steps))	//an ta vimata einai ligotera apo oso xreiazetai gia na epitaxinei
			{	//den tha epitaxinei, tha exei statherh taxitita
				acc_steps = 0;
				regular_steps = difference;
				dec_steps = 0;
			}
			else
			{
				regular_steps = difference - (2 * y_acc_dec_steps);
				acc_steps = y_acc_dec_steps;
				dec_steps = y_acc_dec_steps;
			}
		}
		else	//kinisi pros ta thetika
		{
#if(y_dir_positive)
			{
				motor_PORT |= (1 << y_direction_pin);
			}
#else
			{
				motor_PORT &= ~(1 << y_direction_pin);
			}
#endif
			difference = target_y - y_position;
			y_position = target_y;
			if (difference <= (2 * y_acc_dec_steps))	//an ta vimata einai ligotera apo oso xreiazetai gia na epitaxinei
			{	//den tha epitaxinei, tha exei statherh taxitita
				acc_steps = 0;
				regular_steps = difference;
				dec_steps = 0;
			}
			else
			{
				regular_steps = difference - (2 * y_acc_dec_steps);
				acc_steps = y_acc_dec_steps;
				dec_steps = y_acc_dec_steps;
			}
		}
		end = 0;
		sei();
		while (!end);
		cli();
	}
}
void H_in(void)	//mazevei to H
{
	if (!(term_PIN & (1 << in_term_pin)))
	{
		motor_PORT = (1 << H_in_pin);
		uint8_t i = 0;
		while (i < 3)
		{
			if (term_PIN & (1 << in_term_pin))
			{
				i++;
			}
			else
			{
				i = 0;
			}
			_delay_ms(20);
		}
		motor_PORT = 0;
	}
	return;
}
void H_out(void)	//vgazei to H
{
	if (!(term_PIN & (1 << out_term_pin)))
	{
		motor_PORT = (1 << H_out_pin);
		uint8_t i = 0;
		while (i < 3)
		{
			if (term_PIN & (1 << out_term_pin))
			{
				i++;
			}
			else
			{
				i = 0;
			}
			_delay_ms(10);
		}
		motor_PORT = 0;
	}
	return;
}
void store_package_rfid()	//apothikevei to paketo apo ton tain sthn thesh pou tairiazei sto id
{
	H_in();										//mazepse to H an den einai mazemeno
	move_to(x_coordinates[9], y_coordinates[9]);	//phgaine ston tainiodromo
	_delay_ms(500);
	H_out();									//vgale eksw to H
	_delay_ms(500);
	rel_move_to(0, 30); //20mm			//shkwse to paketo
	uint32_t id = 0;
	id = pn532_readPassiveTargetID(PN532_MIFARE_ISO14443A);
	_delay_ms(500);
	H_in();										//mazepse to H
	_delay_ms(500);
	uint8_t i = 0;
	while ((ids[i] != id) && (i < 9))
	{
		i++;
	}
	if (i > 8)
	{
		H_in();
		return;
	}
	move_to(x_coordinates[i], y_coordinates[i] + 30);
	_delay_ms(500);
	H_out();									//vgale to H
	_delay_ms(500);
	rel_move_to(0, -30); //20mm //katevase to paketo
	_delay_ms(500);
	H_in();										//mazepse to H
	return;
}
void retreive_package_rfid(uint32_t id)	//epistrefei to paketo apo thn thesh pou tairiazei sto id ston tain
{
	H_in();
	uint8_t i = 0;
	while (ids[i] != id)
	{
		i++;
	}
	move_to(x_coordinates[i], y_coordinates[i]);
	_delay_ms(500);
	H_out();
	_delay_ms(500);
	rel_move_to(0, 30);
	_delay_ms(500);
	H_in();
	_delay_ms(500);
	move_to(x_coordinates[9], y_coordinates[9] + 30);
	_delay_ms(500);
	H_out();
	_delay_ms(500);
	rel_move_to(0, -30);
	_delay_ms(500);
	H_in();
	return;
}
void store_package(uint8_t place)	//apothikevei to paketo apo ton tain sthn thesh place
{
	H_in();										//mazepse to H an den einai mazemeno
	move_to(x_coordinates[9], y_coordinates[9]);	//phgaine ston tainiodromo
	_delay_ms(500);
	H_out();									//vgale eksw to H
	_delay_ms(500);
	rel_move_to(0, 30); //20mm			//shkwse to paketo
	_delay_ms(500);
	H_in();										//mazepse to H
	_delay_ms(500);
	move_to(x_coordinates[place], (y_coordinates[place] + 30));
	_delay_ms(500);
	H_out();									//vgale to H
	_delay_ms(500);
	rel_move_to(0, -30); //20mm //katevase to paketo
	_delay_ms(500);
	H_in();										//mazepse to H
	return;
}
void retreive_package(uint8_t place)	//epistrefei to paketo apo thn thesh place ston tain
{
	H_in();
	move_to(x_coordinates[place], y_coordinates[place]);
	_delay_ms(500);
	H_out();
	_delay_ms(500);
	rel_move_to(0, 30);
	_delay_ms(500);
	H_in();
	_delay_ms(500);
	move_to(x_coordinates[9], y_coordinates[9] + 30);
	_delay_ms(500);
	H_out();
	_delay_ms(500);
	rel_move_to(0, -30);
	_delay_ms(500);
	H_in();
	return;
}
void tain_near(void)	//molis afisoume paketo ston tain, to fernei konta sthn kataskeyh
{
	while (term_PIN & (1 << tain_away_term_pin));
	_delay_ms(1500);
	motor_PORT |= 1 << tain_near_pin;
	motor_PORT &= ~(1 << tain_away_pin);
	while (term_PIN & (1 << tain_near_term_pin));
	_delay_ms(3000);
	motor_PORT &= ~(1 << tain_away_pin | 1 << tain_near_pin);
}
void tain_away(void)	//apomakrinei to paketo apo thn kataskeyh
{
	while (term_PIN & (1 << tain_near_term_pin));
	_delay_ms(1500);
	motor_PORT |= 1 << tain_away_pin;
	motor_PORT &= ~(1 << tain_near_pin);
	while (term_PIN & (1 << tain_away_term_pin));
	_delay_ms(2000);
	motor_PORT &= ~(1 << tain_away_pin | 1 << tain_near_pin);
}
void homing(void)	//vriskei thn arxh twn aksonwn
{
	H_in();	//mazepse to H an den einai mazemeno
	/////////////////////////////////////////////////// x
#if(x_dir_negative)
	{
		motor_PORT |= (1 << x_direction_pin);
	}
#else
	{
		motor_PORT &= ~(1 << x_direction_pin);
	}
#endif
	while (!(term_PIN & (1 << x_negative_term_pin)))
	{
		motor_PORT |= 1 << x_step_pin;
		_delay_ms(0.2);
		motor_PORT &= ~(1 << x_step_pin);
		_delay_ms(0.2);
	}
	motor_PORT ^= (1 << x_direction_pin);
	while ((term_PIN & (1 << x_negative_term_pin)))
	{
		motor_PORT |= 1 << x_step_pin;
		_delay_ms(3);
		motor_PORT &= ~(1 << x_step_pin);
		_delay_ms(3);
	}
	x_position = 0;
	/////////////////////////////////////////////////// y
#if(y_dir_negative)
	{
		motor_PORT |= (1 << y_direction_pin);
	}
#else
	{
		motor_PORT &= ~(1 << y_direction_pin);
	}
#endif
	while (!(term_PIN & (1 << y_negative_term_pin)))
	{
		motor_PORT |= 1 << y_step_pin;
		_delay_ms(0.1);
		motor_PORT &= ~(1 << y_step_pin);
		_delay_ms(0.1);
	}
	motor_PORT ^= (1 << y_direction_pin);
	while ((term_PIN & (1 << y_negative_term_pin)))
	{
		motor_PORT |= 1 << y_step_pin;
		_delay_ms(0.5);
		motor_PORT &= ~(1 << y_step_pin);
		_delay_ms(0.5);
	}
	y_position = 0;
	return;
}


int main(void)
{
	//init spi port
	spi_PORT |= 1 << CS;
	spi_DDR |= 1 << MOSI | 1 << CLK | 1 << CS;
	SPCR = 1 << MSTR | 1 << SPR0 | 1 << SPE | 1 << DORD;

	//init motor port
	motor_DDR |= 1 << x_direction_pin | 1 << x_step_pin | 1 << y_direction_pin | 1 << y_step_pin | 1 << H_in_pin | 1 << H_out_pin | 1 << tain_near_pin | 1 << tain_away_pin;
	motor_PORT &= ~( 1 << x_direction_pin | 1 << x_step_pin | 1 << y_direction_pin | 1 << y_step_pin | 1 << H_in_pin | 1 << H_out_pin | 1 << tain_near_pin | 1 << tain_away_pin);

	//init term port
	term_DDR &= ~(1 << x_negative_term_pin | 1 << x_positive_term_pin | 1 << y_negative_term_pin | 1 << y_positive_term_pin | 1 << in_term_pin | 1 << out_term_pin | 1 << tain_near_term_pin | 1 << tain_away_term_pin);
	term_PORT |= 1 << x_negative_term_pin | 1 << x_positive_term_pin | 1 << y_negative_term_pin | 1 << y_positive_term_pin | 1 << in_term_pin | 1 << out_term_pin;
	// otan oi termatikoi den einai patimenoi dinoun 0 kai otan patithoun 1, xreiazontai pull up
	// otan ta ir den einai activate dinoun 0 kai otan einai dinoun 1, den xreiazontai pull up

	//delay gia na prolavoun na sikwthoun oi pullup
	_delay_ms(1);

	//init timer for motor driving
	TCCR1B = 1 << WGM12 | 1 << CS10; //prescaler=1
	TIMSK = 1 << OCIE1A;

	//uart_init();

	pn532_begin();

	homing();

	while (1)
	{
		//// meta to homing phgaine ston tainiodromo
		move_to(x_coordinates[9], y_coordinates[9]);

		////molis afhsoume paketo ston tainiodromo ferto konta stin kataskevh
		tain_near();

		////molis erthei konta sthn kataskevh apothikeuse to sthn apothiki
		store_package_rfid();
	}
}
