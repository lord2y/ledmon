/*
 * SuperMicro IPMI LED control
 * Copyright (C) 2022, Exoscale
 */

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libgen.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/sysinfo.h>
#include <sys/file.h>

#if _HAVE_DMALLOC_H
#include <dmalloc.h>
#endif

#include "config/config.h"
#include "ibpi.h"
#include "list.h"
#include "utils.h"
#include "amd.h"
#include "ipmi.h"

#define SM_CHAN 0x0
#define SM_SLAVE_ADDR 0x0

static uint8_t amd_ibpi_ipmi_register[] = {
	[IBPI_PATTERN_PFA] = 0x41,
	[IBPI_PATTERN_LOCATE] = 0x0,
	[IBPI_PATTERN_LOCATE_OFF] = 0x1,
	[IBPI_PATTERN_FAILED_DRIVE] = 0x44,
	[IBPI_PATTERN_FAILED_ARRAY] = 0x45,
	[IBPI_PATTERN_REBUILD] = 0x46,
	[IBPI_PATTERN_HOTSPARE] = 0x47,
};

/* The path we are given should be similar to
 * /sys/devices/pci0000:e0/0000:e0:03.3/0000:e3:00.0
 *                                      ^^^^^^^^^^
 * We need to retrieve the address from the path (indicated above)
 * then use it to find the corresponding address for a slot in
 * /sys/bus/pci/slots to determine the correct port for the NVMe device.
 */
static int _get_ipmi_nvme_port(char *path)
{
	int rc;
	char *p, *f;
	struct list dir;
	const char *dir_path;
	char *port_name;
	int port = -1;

	p = strrchr(path, '/');
	if (!p) {
		log_error("Couldn't parse NVMe path to determine port\n");
		return -1;
	}

	p++;

	/* p now points to the address, remove the bits after the '.' */
	f = strchr(p, '.');
	if (!f) {
		log_error("Couldn't parse NVMe port address\n");
		return -1;
	}

	*f = '\0';

	rc = scan_dir("/sys/bus/pci/slots", &dir);
	if (rc)
		return -1;

	list_for_each(&dir, dir_path) {
		port_name = get_text(dir_path, "address");
		if (port_name && !strcmp(port_name, p)) {
			char *dname = strrchr(dir_path, '/');

			dname++;
			if (str_toi(&port, dname, NULL, 0) != 0)
				return -1;
			break;
		}
	}

	list_erase(&dir);

	return port;
}


static int _get_amd_ipmi_drive(const char *start_path,
			       struct amd_drive *drive)
{
	int found;
	char path[PATH_MAX];

	found = _find_file_path(start_path, "nvme", path, PATH_MAX);
	if (found) {
		drive->port = _get_ipmi_nvme_port(path);
		if (drive->port < 0) {
			log_error("Could not retrieve port number\n");
			return -1;
		}

		drive->drive_bay = 1 << (drive->port - 1);
		drive->dev = AMD_NVME_DEVICE;
	}

	log_debug("AMD Drive: port: %d, bay %x\n", drive->port,
		  drive->drive_bay);

	return 0;
}

static int _set_ipmi_register(int enable, uint8_t reg,
			      struct amd_drive *drive)
{
	int rc;
	int status, data_sz;
	uint8_t drives_status;
	uint8_t new_drives_status;
	uint8_t cmd_data[5];

	memset(cmd_data, 0, sizeof(cmd_data));

	cmd_data[0] = 0x6c;
	cmd_data[1] = 0x1;
	cmd_data[2] = 0x0;
	cmd_data[3] = 0x0;
	cmd_data[4] = reg;

	/* Find current register setting */
	status = 0;

	log_debug("Retrieving current register status\n");
	log_debug(REG_FMT_2, "channel", cmd_data[0], "slave addr", cmd_data[1]);
	log_debug(REG_FMT_2, "len", cmd_data[2], "register", cmd_data[3]);

	rc = ipmicmd(BMC_SA, 0x0, 0x30, 0x70, 5, &cmd_data, 1, &data_sz,
		     &status);
	if (rc) {
		log_error("Could not determine current register %x setting\n",
			  reg);
		return rc;
	}

	drives_status = status;

	if (enable)
		new_drives_status = drives_status | drive->drive_bay;
	else
		new_drives_status = drives_status & ~drive->drive_bay;

	/* Set the appropriate status */
	status = 0;
	cmd_data[4] = new_drives_status;

	log_debug("Updating register status: %x -> %x\n", drives_status,
		  new_drives_status);
	log_debug(REG_FMT_2, "channel", cmd_data[0], "slave addr", cmd_data[1]);
	log_debug(REG_FMT_2, "len", cmd_data[2], "register", cmd_data[3]);
	log_debug(REG_FMT_1, "status", cmd_data[4]);

	rc = ipmicmd(BMC_SA, 0x0, 0x30, 0x70, 5, &cmd_data, 1, &data_sz,
		     &status);
	if (rc) {
		log_error("Could not enable register %x\n", reg);
		return rc;
	}

	return 0;
}

static int _enable_ibpi_state(struct amd_drive *drive, enum ibpi_pattern ibpi)
{
	log_debug("Enabling %s LED\n", ibpi2str(ibpi));
	return _set_ipmi_register(1, amd_ibpi_ipmi_register[ibpi], drive);
}

static int _disable_ibpi_state(struct amd_drive *drive, enum ibpi_pattern ibpi)
{
	log_debug("Disabling %s LED\n", ibpi2str(ibpi));
	return _set_ipmi_register(0, amd_ibpi_ipmi_register[ibpi], drive);
}

static int _disable_all_ibpi_states(struct amd_drive *drive)
{
	int rc;

	rc = _disable_ibpi_state(drive, IBPI_PATTERN_PFA);
	rc |= _disable_ibpi_state(drive, IBPI_PATTERN_LOCATE);
	rc |= _disable_ibpi_state(drive, IBPI_PATTERN_LOCATE_OFF);
	rc |= _disable_ibpi_state(drive, IBPI_PATTERN_FAILED_DRIVE);
	rc |= _disable_ibpi_state(drive, IBPI_PATTERN_FAILED_ARRAY);
	rc |= _disable_ibpi_state(drive, IBPI_PATTERN_REBUILD);

	return rc;
}

int _amd_ipmi_sm_em_enabled(const char *path)
{
	int rc;
	int status, data_sz;
	uint8_t cmd_data[4];
	struct amd_drive drive;

	log_debug("Enabling _adm_ipmi_sm_em_enabled(%s)\n", path);

	memset(&drive, 0, sizeof(struct amd_drive));

	cmd_data[0] = SM_CHAN;
	cmd_data[1] = SM_SLAVE_ADDR;
	cmd_data[2] = 0x1;
	cmd_data[3] = 0x6c;

	status = 0;
	rc = ipmicmd(BMC_SA, 0x0, 0x30, 0x70, 4, &cmd_data, 1,
		     &data_sz, &status);

	log_debug("rc => %i\n", rc);

	if (rc) {
		log_error("Can't determine status for SM-AMD platform\n");
		return 0;
	}
	log_debug("status => %i\n", status);

	return 1;
}

int _amd_ipmi_sm_write(struct block_device *device, enum ibpi_pattern ibpi)
{
	int rc;
	struct amd_drive drive;

	log_info("\n");
	log_info("Setting %s...", ibpi2str(ibpi));

	rc = _get_amd_ipmi_drive(device->cntrl_path, &drive);
	if (rc)
		return rc;

	if ((ibpi == IBPI_PATTERN_NORMAL) ||
	    (ibpi == IBPI_PATTERN_ONESHOT_NORMAL)) {
		rc = _disable_all_ibpi_states(&drive);
		return rc;
	}

	if (ibpi == IBPI_PATTERN_LOCATE_OFF) {
		rc = _disable_ibpi_state(&drive, IBPI_PATTERN_LOCATE_OFF);
		return rc;
	}

	/*rc = _enable_smbus_control(&drive);
	if (rc)
		return rc;
	*/

	rc = _enable_ibpi_state(&drive, ibpi);
	if (rc)
		return rc;

	return 0;
}

char *_amd_ipmi_sm_get_path(const char *cntrl_path, const char *sysfs_path)
{
	char *t;

	/* For NVMe devices we can just dup the path sysfs path */
	if (strstr(cntrl_path, "nvme"))
		return strdup(sysfs_path);

	/*
	 * For SATA devices we need everything up to 'ataXX/' in the path
	 */
	t = strstr(cntrl_path, "ata");
	if (!t)
		return NULL;

	/* Move to the '/' after the ataXX piece of the path and terminate the
	 * string there.
	 */
	t = strchr(t, '/');
	if (!t)
		return NULL;

	return strndup(cntrl_path, (t - cntrl_path) + 1);
}

