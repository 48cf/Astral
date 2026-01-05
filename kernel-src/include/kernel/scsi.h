#ifndef _SCSI_H
#define _SCSI_H

#include <stdint.h>

#include <kernel/iovec.h>

typedef struct scsi_cmd scsi_cmd_t;
typedef struct scsi_host scsi_host_t;

typedef enum {
	SCSI_CMD_DIR_TO_DEVICE,
	SCSI_CMD_DIR_TO_HOST,
} scsi_cmd_dir_t;

struct scsi_cmd {
	uint8_t cdb[16];
	uint8_t cdb_length;

	scsi_cmd_dir_t dir;
	uint8_t status;
	iovec_iterator_t *iov;

	void (*complete)(scsi_host_t *, scsi_cmd_t *, int status);
	void *completion_ctx;
};

typedef struct {
	int (*submit)(scsi_host_t *, scsi_cmd_t *);
} scsi_host_ops_t;

struct  scsi_host {
	scsi_host_ops_t *ops;
	uint32_t block_size;
};

void scsi_register(scsi_host_t *host);

#endif
