/*
 * Field.h
 *
 *  Created on: Nov 8, 2016
 *      Author: jurge
 */

#ifndef SRC_MOD_WORLD_INTERFACE_FIELD_H_
#define SRC_MOD_WORLD_INTERFACE_FIELD_H_


#include "FieldConfig.h"

namespace trs {
	void FillSharedMemoryFromFieldConfig(const FieldConfig& fieldConfig);
	FieldConfig FillFieldConfigFromSharedMemory();
} // namespace

#endif /* SRC_MOD_WORLD_INTERFACE_FIELD_H_ */
