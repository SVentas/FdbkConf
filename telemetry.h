#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <QtGlobal>

/* Empty message ID.                       */
#define TELEMETRY_MSG_NOMSG             0x00
/* Telemetry message header size in bytes. */
#define TELEMETRY_MSG_HDR_SIZE          0x04
/* Telemetry message signature byte.       */
#define TELEMETRY_MSG_SIGNATURE         0xAA
/* Telemetry buffer size in bytes.         */
#define TELEMETRY_MSG_BUFFER_SIZE       0x0080
/* Streaming buffer size in bytes.         */
#define TELEMETRY_MSG_SIZE_BYTES_MAX    0x0400
/* Telemetry data_size member offset.      */
#define TELEMETRY_MSG_DATA_SIZE_ID      0x02

/* Number of samples in streaming buffer.  */
#define STREAMING_BUF_DEPTH             16
/* Number of samples in plotting buffer.   */
#define PLOTTING_BUF_DEPTH              (STREAMING_BUF_DEPTH * 2)

typedef struct tagTelemetryMessage {
    quint8 msg_id;     /* Telemetry message ID.           */
    quint8 signature;  /* Telemetry message signature.    */
    quint16 data_size; /* Size of telemetry message data. */
    char data[TELEMETRY_MSG_BUFFER_SIZE]; /* Data buffer. */
} __attribute__((packed)) TelemetryMessage, *PTelemetryMessage;

Q_DECLARE_METATYPE(TelemetryMessage);

#endif // TELEMETRY_H
