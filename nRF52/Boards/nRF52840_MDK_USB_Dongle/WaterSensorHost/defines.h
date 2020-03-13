#define NRF5
#define NRF52840
#define ARDUINO_ARCH_NRF5

#define RADIO_MODE_MODE_Nrf_250Kbit (2UL) /*!< Deprecated enumerator -  250 kbit/s Nordic proprietary radio mode */
#define MY_SERIAL_OUTPUT_SIZE (120u)
#define MY_BAUD_RATE (115200ul)

#include "NRF5\MyHwNRF5.h"

// From MySensorsCore.h
#define MY_WAKE_UP_BY_TIMER			((int8_t)-1)		//!< Sleeping wake up by timer
#define MY_SLEEP_NOT_POSSIBLE		((int8_t)-2)		//!< Sleeping not possible
#define INTERRUPT_NOT_DEFINED		((uint8_t)255)	//!< _sleep() param: no interrupt defined
#define MODE_NOT_DEFINED				((uint8_t)255)	//!< _sleep() param: no mode defined
#define VALUE_NOT_DEFINED				((uint8_t)255)	//!< Value not defined
#define FUNCTION_NOT_SUPPORTED	((uint16_t)0)		//!< Function not supported
#define V2_MYS_HEADER_PROTOCOL_VERSION      (2u) //!< Protocol version
#define V2_MYS_HEADER_SIZE                  (7u) //!< Header size
#define V2_MYS_HEADER_MAX_MESSAGE_SIZE      (32u) //!< Max payload size
// ---------------------

// From MyMessage.h
#define V2_MYS_HEADER_VSL_VERSION_POS       (0) //!< bitfield position version
#define V2_MYS_HEADER_VSL_VERSION_SIZE      (2u) //!< size version field
#define V2_MYS_HEADER_VSL_SIGNED_POS        (2u) //!< bitfield position signed field
#define V2_MYS_HEADER_VSL_SIGNED_SIZE       (1u) //!< size signed field
#define V2_MYS_HEADER_VSL_LENGTH_POS        (3u) //!< bitfield position length field
#define V2_MYS_HEADER_VSL_LENGTH_SIZE       (5u) //!< size length field

#define V2_MYS_HEADER_CEP_COMMAND_POS       (0) //!< bitfield position command field
#define V2_MYS_HEADER_CEP_COMMAND_SIZE      (3u) //!< size command field
#define V2_MYS_HEADER_CEP_ECHOREQUEST_POS   (3u) //!< bitfield position echo request field
#define V2_MYS_HEADER_CEP_ECHOREQUEST_SIZE   (1u) //!< size echo request field
#define V2_MYS_HEADER_CEP_ECHO_POS          (4u) //!< bitfield position echo field
#define V2_MYS_HEADER_CEP_ECHO_SIZE         (1u) //!< size echo field
#define V2_MYS_HEADER_CEP_PAYLOADTYPE_POS   (5u) //!< bitfield position payload type field
#define V2_MYS_HEADER_CEP_PAYLOADTYPE_SIZE  (3u) //!< size payload type field

#define MAX_MESSAGE_SIZE                    V2_MYS_HEADER_MAX_MESSAGE_SIZE	//!< The maximum size of a message (including header)
#define HEADER_SIZE                         V2_MYS_HEADER_SIZE	//!< The size of the header
#define MAX_PAYLOAD_SIZE                    (MAX_MESSAGE_SIZE - HEADER_SIZE) //!< The maximum size of a payload depends on #MAX_MESSAGE_SIZE and #HEADER_SIZE

// deprecated in 3.0.0
#define MAX_PAYLOAD                         MAX_PAYLOAD_SIZE //!< \deprecated in 3.0.0 The maximum size of a payload depends on #MAX_MESSAGE_SIZE and #HEADER_SIZE
// ------------------------

// From MyTransportHAL.h
#define INVALID_SNR         ((int16_t)-256)	//!< INVALID_SNR
#define INVALID_RSSI        ((int16_t)-256)	//!< INVALID_RSSI
#define INVALID_PERCENT     ((int16_t)-100)	//!< INVALID_PERCENT
#define INVALID_LEVEL       ((int16_t)-256)	//!< INVALID_LEVEL

#define BROADCAST_ADDRESS			(255u)			//!< broadcasts are addressed to ID 255

// ------------------------
