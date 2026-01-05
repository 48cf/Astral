#include <kernel/block.h>
#include <kernel/pmm.h>
#include <kernel/scsi.h>
#include <logging.h>

#define SCSI_CDB_OP_INQUIRY 0x12
#define SCSI_CDB_OP_READ_CAPACITY10 0x25
#define SCSI_CDB_OP_READ10 0x28
#define SCSI_CDB_OP_WRITE10 0x2a

typedef struct __attribute__((packed)) {
	uint8_t op;
	uint8_t evpd : 1;
	uint8_t rsvd : 7;
	uint8_t page_code;
	uint16_t alloc_length;
	uint8_t control;
} scsi_cdb_inquiry_t;

typedef struct __attribute__((packed)) {
	uint8_t op;
	uint8_t rsvd1;
	uint32_t lba;
	uint8_t rsvd2[3];
	uint8_t control;
} scsi_cdb_read_capacity10_t;

typedef struct __attribute__((packed)) {
	uint8_t op;
	uint8_t rsvd1 : 3;
	uint8_t fua : 1;
	uint8_t dpo : 1;
	uint8_t rsvd2 : 3;
	uint32_t lba;
	uint8_t group_number : 5;
	uint8_t rsvd3 : 3;
	uint16_t transfer_length;
	uint8_t control;
} scsi_cdb_read10_t;

typedef struct __attribute__((packed)) {
	uint8_t op;
	uint8_t rsvd1 : 3;
	uint8_t fua : 1;
	uint8_t dpo : 1;
	uint8_t rsvd2 : 3;
	uint32_t lba;
	uint8_t group_number : 5;
	uint8_t rsvd3 : 3;
	uint16_t transfer_length;
	uint8_t control;
} scsi_cdb_write10_t;

static void scsi_cdb_inquiry6(scsi_cmd_t *cmd, uint16_t alloc_length) {
	scsi_cdb_inquiry_t cdb = {0};
	cdb.op = SCSI_CDB_OP_INQUIRY;
	cdb.alloc_length = cpu_to_be_w(alloc_length);

	memcpy(cmd->cdb, &cdb, sizeof(cdb));
	cmd->cdb_length = sizeof(cdb);
}

static void scsi_cdb_read_capacity10(scsi_cmd_t *cmd) {
	scsi_cdb_read_capacity10_t cdb = {0};
	cdb.op = SCSI_CDB_OP_READ_CAPACITY10;

	memcpy(cmd->cdb, &cdb, sizeof(cdb));
	cmd->cdb_length = sizeof(cdb);
}

static void scsi_cdb_read10(scsi_cmd_t *cmd, uint32_t lba, uint16_t xfer_len) {
	scsi_cdb_read10_t cdb = {0};
	cdb.op = SCSI_CDB_OP_READ10;
	cdb.lba = cpu_to_be_d(lba);
	cdb.transfer_length = cpu_to_be_w(xfer_len);

	memcpy(cmd->cdb, &cdb, sizeof(cdb));
	cmd->cdb_length = sizeof(cdb);
}

static void scsi_cdb_write10(scsi_cmd_t *cmd, uint32_t lba, uint16_t xfer_len) {
	scsi_cdb_write10_t cdb = {0};
	cdb.op = SCSI_CDB_OP_WRITE10;
	cdb.lba = cpu_to_be_d(lba);
	cdb.transfer_length = cpu_to_be_w(xfer_len);

	memcpy(cmd->cdb, &cdb, sizeof(cdb));
	cmd->cdb_length = sizeof(cdb);
}

static int scsi_write(void *private, iovec_iterator_t *buffer, uintmax_t lba, size_t count) {
	scsi_host_t *host = private;

	scsi_cmd_t cmd = {0};
	scsi_cdb_write10(&cmd, (uint32_t)lba, (uint16_t)count);

	cmd.dir = SCSI_CMD_DIR_TO_DEVICE;
	cmd.iov = buffer;

	return host->ops->submit(host, &cmd);
}

static int scsi_read(void *private, iovec_iterator_t *buffer, uintmax_t lba, size_t count) {
	scsi_host_t *host = private;

	scsi_cmd_t cmd = {0};
	scsi_cdb_read10(&cmd, (uint32_t)lba, (uint16_t)count);

	cmd.dir = SCSI_CMD_DIR_TO_HOST;
	cmd.iov = buffer;

	return host->ops->submit(host, &cmd);
}

static int block_dev_id = 0;

void scsi_register(scsi_host_t *host) {
	char name[8];
	snprintf(name, sizeof(name), "scsi%d", __atomic_fetch_add(&block_dev_id, 1, __ATOMIC_SEQ_CST));

	uint8_t inquiry_data[36];
	uint8_t read_capacity_data[8];

	{
		scsi_cmd_t cmd = {0};
		scsi_cdb_inquiry6(&cmd, sizeof(inquiry_data));

		iovec_t iovec;
		iovec.addr = inquiry_data;
		iovec.len = sizeof(inquiry_data);

		iovec_iterator_t iov;
		iovec_iterator_init(&iov, &iovec, 1);

		cmd.dir = SCSI_CMD_DIR_TO_HOST;
		cmd.iov = &iov;

		int res = host->ops->submit(host, &cmd);
		__assert(res == 0);
	}

	{
		scsi_cmd_t cmd = {0};
		scsi_cdb_read_capacity10(&cmd);

		iovec_t iovec;
		iovec.addr = read_capacity_data;
		iovec.len = sizeof(read_capacity_data);

		iovec_iterator_t iov;
		iovec_iterator_init(&iov, &iovec, 1);

		cmd.dir = SCSI_CMD_DIR_TO_HOST;
		cmd.iov = &iov;

		int res = host->ops->submit(host, &cmd);
		__assert(res == 0);
	}

	uint32_t max_lba = be_to_cpu_d(*(uint32_t *)&read_capacity_data[0]);
	uint32_t block_size = be_to_cpu_d(*(uint32_t *)&read_capacity_data[4]);

	host->block_size = block_size;

	printf("%s: %u blocks with %u bytes per block\n", name, max_lba + 1, block_size);

	blockdesc_t desc = {0};
	desc.private = host;
	desc.type = BLOCK_TYPE_DISK;
	desc.lbaoffset = 0;
	desc.blockcapacity = max_lba + 1;
	desc.blocksize = block_size;
	desc.write = scsi_write;
	desc.read = scsi_read;

	block_register(&desc, name);
}
