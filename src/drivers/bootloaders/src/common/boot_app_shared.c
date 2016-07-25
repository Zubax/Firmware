/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>

#include "chip.h"
#include "stm32.h"

#include <errno.h>
#include "boot_app_shared.h"
#include "crc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOOTLOADER_COMMON_APP_SIGNATURE         0xB0A04150u
#define BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE  0xB0A0424Cu


/*  CAN_FiRx where (i=0..27|13, x=1, 2)
 *                      STM32_CAN1_FIR(i,x)
 * Using i = 2 does not requier there block
 * to be enabled nor FINIT in CAN_FMR to be set.
 * todo:Validate this claim on F2, F3
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/**
 * This is a stopgap measure to adapt the bootloader to new bootloader-app interface.
 * Normally we would left the old interface in place until the new bootloader is finished,
 * but unfortunately the old approach for parameter passing (via CAN filter registers) does not
 * work on STM32F446.
 * This change does not affect the rest of the bootloader code.
 * Refer to the Zubax Babel bootloader implementation for additional info.
 */
struct app_shared_t
{
    uint32_t reserved_a;                        ///< Reserved for future use
    uint32_t reserved_b;                        ///< Reserved for future use
    uint32_t can_bus_speed;
    uint8_t uavcan_node_id;
    uint8_t uavcan_fw_server_node_id;           ///< Not used when transitioning from bootloader to app
    char uavcan_file_name[201];                 ///< Not used when transitioning from bootloader to app
    uint8_t stay_in_bootloader;                 ///< Not used when transitioning from bootloader to app
    uint64_t crc;
};

#define SHARED_STRUCT_ADR       0x20000000
#define SHARED_STRUCT_PTR       ((struct app_shared_t*)(SHARED_STRUCT_ADR))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: read
 ****************************************************************************/

inline static bool read(bootloader_app_shared_t *pshared)
{
	pshared->signature = BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE;   // Always valid...
	pshared->bus_speed = SHARED_STRUCT_PTR->can_bus_speed;
	pshared->node_id = SHARED_STRUCT_PTR->uavcan_node_id;

	const uint64_t read_crc = SHARED_STRUCT_PTR->crc;

	SHARED_STRUCT_PTR->crc = CRC64_INITIAL;
	const uint32_t* as_words = (uint32_t*)SHARED_STRUCT_PTR;
	for (int i = 0; i < sizeof(struct app_shared_t) / 4 - 2; i++)
	{
		SHARED_STRUCT_PTR->crc = crc64_add_word(SHARED_STRUCT_PTR->crc, as_words[i]);
	}
	SHARED_STRUCT_PTR->crc ^= CRC64_OUTPUT_XOR;

	return read_crc == SHARED_STRUCT_PTR->crc;
}

/****************************************************************************
 * Name: write
 ****************************************************************************/

inline static void write(bootloader_app_shared_t *pshared)
{
	memset((void*)SHARED_STRUCT_ADR, 0, sizeof(struct app_shared_t));

	// So ugly, but this is a dirty hack
	if ((pshared->bus_speed > 0 && pshared->bus_speed <= 1000000) && (pshared->node_id <= 125)) {

		SHARED_STRUCT_PTR->can_bus_speed = pshared->bus_speed;
		SHARED_STRUCT_PTR->uavcan_node_id = pshared->node_id;

		SHARED_STRUCT_PTR->crc = CRC64_INITIAL;
		const uint32_t* as_words = (uint32_t*)SHARED_STRUCT_PTR;
		for (int i = 0; i < sizeof(struct app_shared_t) / 4 - 2; i++)
		{
			SHARED_STRUCT_PTR->crc = crc64_add_word(SHARED_STRUCT_PTR->crc, as_words[i]);
		}
		SHARED_STRUCT_PTR->crc ^= CRC64_OUTPUT_XOR;
	}
}

/****************************************************************************
 * Name: bootloader_app_shared_init
 ****************************************************************************/
static void bootloader_app_shared_init(bootloader_app_shared_t *pshared, eRole_t role)
{
	memset(pshared, 0, sizeof(bootloader_app_shared_t));

	if (role != Invalid) {
		pshared->signature =
			(role ==
			 App ? BOOTLOADER_COMMON_APP_SIGNATURE :
			 BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE);
	}

}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: bootloader_app_shared_read
 *
 * Description:
 *   Based on the role requested, this function will conditionally populate
 *   a bootloader_app_shared_t structure from the physical locations used
 *   to transfer the shared data to/from an application (internal data) .
 *
 *   The functions will only populate the structure and return a status
 *   indicating success, if the internal data has the correct signature as
 *   requested by the Role AND has a valid crc.
 *
 * Input Parameters:
 *   shared - A pointer to a bootloader_app_shared_t return the data in if
 *   the internal data is valid for the requested Role
 *   role   - An eRole_t of App or BootLoader to validate the internal data
 *            against. For a Bootloader this would be the value of App to
 *            read the application passed data.
 *
 * Returned value:
 *   OK     - Indicates that the internal data has been copied to callers
 *            bootloader_app_shared_t structure.
 *
 *  -EBADR  - The Role or crc of the internal data was not valid. The copy
 *            did not occur.
 *
 ****************************************************************************/

__EXPORT
int bootloader_app_shared_read(bootloader_app_shared_t *shared,
			       eRole_t role)
{
	int rv = -EBADR;
	bootloader_app_shared_t working;

	if (read(&working)) {
		*shared = working;
		rv = OK;
	}

	return rv;
}

/****************************************************************************
 * Name: bootloader_app_shared_write
 *
 * Description:
 *   Based on the role, this function will commit the data passed
 *   into the physical locations used to transfer the shared data to/from
 *   an application (internal data) .
 *
 *   The functions will populate the signature and crc the data
 *   based on the provided Role.
 *
 * Input Parameters:
 *   shared - A pointer to a bootloader_app_shared_t data to commit to
 *   the internal data for passing to/from an application.
 *   role   - An eRole_t of App or BootLoader to use in the internal data
 *            to be passed to/from an application. For a Bootloader this
 *            would be the value of Bootloader to write to the passed data.
 *            to the application via the internal data.
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/
__EXPORT
void bootloader_app_shared_write(bootloader_app_shared_t *shared,
				 eRole_t role)
{
	bootloader_app_shared_t working = *shared;
	working.signature =
		(role ==
		 App ? BOOTLOADER_COMMON_APP_SIGNATURE :
		 BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE);
	write(&working);

}

/****************************************************************************
 * Name: bootloader_app_shared_invalidate
 *
 * Description:
 *   Invalidates the data passed the physical locations used to transfer
 *   the shared data to/from an application (internal data) .
 *
 *   The functions will invalidate the signature and crc and shoulf be used
 *   to prevent deja vu.
 *
 * Input Parameters:
 *   None.
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/

__EXPORT
void bootloader_app_shared_invalidate(void)
{
	bootloader_app_shared_t working;
	bootloader_app_shared_init(&working, Invalid);
	write(&working);
}
