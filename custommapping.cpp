/*
    Custom Mapping Library. Written by Benjamin Jack Cullen.
*/

#include "custommapping.h"
#include <Arduino.h>
#include <stdlib.h>
#include "wtgps300p.h"
#include "wt901.h"
#include "multiplexers.h"
#include "matrix.h"

/**
 * @struct CustomMappingStruct
 */
struct CustomMappingStruct mappingData={
    .center_map_x0=0,
    .center_map_x1=0,
    .char_map_value={
    "Digital", // 0
    "YawGPATT", // 1
    "RollGPATT", // 2
    "PitchGPATT", // 3
    "Gyro0AccX", // 4
    "Gyro0AccY", // 5
    "Gyro0AccZ", // 6
    "Gyro0AngX", // 7
    "Gyro0AngY", // 8
    "Gyro0AngZ", // 9
    "Gyro0MagX", // 10not
    "Gyro0MagY", // 11
    "Gyro0MagZ", // 12
    "Gyro0GyroX", // 13
    "Gyro0GyroY", // 14
    "Gyro0GyroZ", // 15
    // reduce the following using an index array
    "ADMPlex0_0", // 16
    "ADMPlex0_1", // 17
    "ADMPlex0_2", // 18
    "ADMPlex0_3", // 19
    "ADMPlex0_4", // 20
    "ADMPlex0_5", // 21
    "ADMPlex0_6", // 22
    "ADMPlex0_7", // 23
    "ADMPlex0_8", // 24
    "ADMPlex0_9", // 25
    "ADMPlex0_10", // 26
    "ADMPlex0_11", // 27
    "ADMPlex0_12", // 28
    "ADMPlex0_13", // 29
    "ADMPlex0_14", // 30
    "ADMPlex0_15", // 31
    },
    .map_mode={
      {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-39
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 40-49
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50-59
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 60-69
      }
    },
  .char_map_mode_names={
    "Min To Max",
    "Dual Axis 0",
    "Dual Axis 1",
  },
  .mapping_config={
    {
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 0-4
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 5-9
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 10-14
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 15-19
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 20-24
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 25-29
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 30-34
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 35-39
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 40-44
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 45-49
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 50-54
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 55-59
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 60-64
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 65-69
    }
  },
  .mapped_value={
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-39
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 40-49
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50-59
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 60-69
    }
  },
};

void center_map(int map_slot, int32_t var) {
    // Offset correction (center to 0).
    int32_t centered=var - mappingData.mapping_config[0][map_slot][INDEX_CMAP_CENTER];
    // Piecewise linear scaling.
    int32_t normalized;
    if (centered <= 0) {
         // Avoid division by zero and normalize.
        if (mappingData.mapping_config[0][map_slot][INDEX_CMAP_NEG_RANGE] != 0) {
            normalized=(centered * mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX]) /
            mappingData.mapping_config[0][map_slot][INDEX_CMAP_NEG_RANGE];}
        else {normalized=0;}
    }
    else {
         // Avoid division by zero and normalize.
        if (mappingData.mapping_config[0][map_slot][INDEX_CMAP_POS_RANGE] != 0) {
            normalized=(centered * mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX]) /
            mappingData.mapping_config[0][map_slot][INDEX_CMAP_POS_RANGE];}
        else {normalized=0;}
    }
    // Clamping to [-OUTPUT_MAX, OUTPUT_MAX].
    if (normalized > mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX])
        {normalized=mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX];}
    else if (normalized < -mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX])
        {normalized=-mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX];}
    // Apply deadzone.
    if (normalized > -mappingData.mapping_config[0][map_slot][INDEX_CMAP_DEADZONE] &&
        normalized < mappingData.mapping_config[0][map_slot][INDEX_CMAP_DEADZONE])
        {normalized=0;}
    // Apply opposite cutoff.
    if (normalized <= 0)
        {mappingData.center_map_x1=0; mappingData.center_map_x0=abs(normalized);}
    else if (normalized >= 0)
        {mappingData.center_map_x0=0; mappingData.center_map_x1=abs(normalized);}
    // Select either x0 or x1 according to configuration.
    if (mappingData.map_mode[0][map_slot]==MAP_CENTER_X0)
        {mappingData.mapped_value[0][map_slot]=mappingData.center_map_x0;}
    else if (mappingData.map_mode[0][map_slot]==MAP_CENTER_X1)
        {mappingData.mapped_value[0][map_slot]=mappingData.center_map_x1;}
}

/**
 * @brief Map Value Mode Select.
 * 
 * @note Supports mapping negative numbers.
 * 
 * @param map_slot Specify Slot.
 * @param map_input_value Only specify values in range of int32_t
 * 
 * To do: Allow mapping up to doubles so that any signed number
 * from DOUBLE_MIN to DOUBLE_MAX can be specified, weather int or floating point.
 */
void map_values_helper(int map_slot, int32_t map_input_value) {

    // Mode 0: Map Min to Max
    if (mappingData.map_mode[0][map_slot]==MAP_MIN_TO_MAX) {
        int32_t emin = mappingData.mapping_config[0][map_slot][INDEX_MAP_EMIN];
        int32_t emax = mappingData.mapping_config[0][map_slot][INDEX_MAP_EMAX];
        int32_t omin = mappingData.mapping_config[0][map_slot][INDEX_MAP_OMIN];
        int32_t omax = mappingData.mapping_config[0][map_slot][INDEX_MAP_OMAX];

        // Check that range is valid (i.e., not equal)
        if (emax != emin) {
            int32_t inputVal = map_input_value;

            // Allow negative or reversed ranges
            mappingData.mapped_value[0][map_slot] =
                (inputVal - emin) * (omax - omin) / (emax - emin) + omin;
        }
    }
    // Mode 1 & 2: Center Mapping
    else if (mappingData.map_mode[0][map_slot]==MAP_CENTER_X0 ||
             mappingData.map_mode[0][map_slot]==MAP_CENTER_X1)
             {center_map(map_slot, map_input_value);}
}

void map_values(void) {
    for (int map_slot=0; map_slot < MAX_MAPPABLE_VALUES; map_slot++) {
        switch ((int)mappingData.mapping_config[0][map_slot][INDEX_MAP_VALUE]) {
        case INDEX_MAPPABLE_VALUES_DIGITAL:
            mappingData.mapped_value[0][map_slot]=0; // todo: allow custom mapping for 0/1
            break;
        case INDEX_MAPPABLE_VALUES_YAWGPATT:
            map_values_helper(map_slot, atol(gpattData.yaw));
            break;
        case INDEX_MAPPABLE_VALUES_ROLLGPATT:
            map_values_helper(map_slot, atol(gpattData.roll));
            break;
        case INDEX_MAPPABLE_VALUES_PITCHGPATT:
            map_values_helper(map_slot, atol(gpattData.pitch));
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ACCX:
            map_values_helper(map_slot, (int32_t)gyroData.gyro_0_acc_x);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ACCY:
            map_values_helper(map_slot, (int32_t)gyroData.gyro_0_acc_y);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ACCZ:
            map_values_helper(map_slot, (int32_t)gyroData.gyro_0_acc_z);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ANGX:
            map_values_helper(map_slot, (int32_t)gyroData.gyro_0_ang_x);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ANGY:
            map_values_helper(map_slot, (int32_t)gyroData.gyro_0_ang_y);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ANGZ:
            map_values_helper(map_slot, (int32_t)gyroData.gyro_0_ang_z);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0MAGX:
            map_values_helper(map_slot, (int32_t)gyroData.gyro_0_mag_x);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0MAGY:
            map_values_helper(map_slot, (int32_t)gyroData.gyro_0_mag_y);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0MAGZ:
            map_values_helper(map_slot, (int32_t)gyroData.gyro_0_mag_z);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0GYROX:
            map_values_helper(map_slot, (int32_t)gyroData.gyro_0_gyr_x);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0GYROY:
            map_values_helper(map_slot, (int32_t)gyroData.gyro_0_gyr_y);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0GYROZ:
            map_values_helper(map_slot, (int32_t)gyroData.gyro_0_gyr_z);
            break;
        // reduce the following using an index array
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_0:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[0]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_1:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[1]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_2:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[2]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_3:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[3]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_4:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[4]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_5:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[5]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_6:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[6]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_7:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[7]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_8:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[8]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_9:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[9]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_10:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[10]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_11:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[11]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_12:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[12]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_13:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[13]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_14:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[14]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_15:
            map_values_helper(map_slot, (int32_t)ad_mux_0.data[15]);
            break;
        default:
            mappingData.mapped_value[0][map_slot]=0;
        }
    }
}

/**
 * @brief Get Target value to be mapped.
 * 
 * @param mapslot Specify target mapslot.
 * @return Char value is returned so that map value types are returned as a uniform type.
 */
double get_mapping_input_value(int map_slot) {
    
    double result=NAN;
    switch ((int)mappingData.mapping_config[0][map_slot][INDEX_MAP_VALUE]) {

        case INDEX_MAPPABLE_VALUES_DIGITAL:
            result = 0;
            break;

        case INDEX_MAPPABLE_VALUES_YAWGPATT:
            result = strtod(gpattData.yaw, NULL);
            break;
        case INDEX_MAPPABLE_VALUES_ROLLGPATT:
            result = strtod(gpattData.roll, NULL);
            break;
        case INDEX_MAPPABLE_VALUES_PITCHGPATT:
            result = strtod(gpattData.pitch, NULL);
            break;

        case INDEX_MAPPABLE_VALUES_GYRO0ACCX:
            result = (double)gyroData.gyro_0_acc_x;
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ACCY:
            result = (double)gyroData.gyro_0_acc_y;
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ACCZ:
            result = (double)gyroData.gyro_0_acc_z;
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ANGX:
            result = (double)gyroData.gyro_0_ang_x;
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ANGY:
            result = (double)gyroData.gyro_0_ang_y;
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ANGZ:
            result = (double)gyroData.gyro_0_ang_z;
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0MAGX:
            result = (double)gyroData.gyro_0_mag_x;
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0MAGY:
            result = (double)gyroData.gyro_0_mag_y;
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0MAGZ:
            result = (double)gyroData.gyro_0_mag_z;
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0GYROX:
            result = (double)gyroData.gyro_0_gyr_x;
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0GYROY:
            result = (double)gyroData.gyro_0_gyr_y;
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0GYROZ:
            result = (double)gyroData.gyro_0_gyr_z;
            break;

        case INDEX_MAPPABLE_VALUES_ADMPLEX0_0:
            result = (double)ad_mux_0.data[0];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_1:
            result = (double)ad_mux_0.data[1];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_2:
            result = (double)ad_mux_0.data[2];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_3:
            result = (double)ad_mux_0.data[3];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_4:
            result = (double)ad_mux_0.data[4];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_5:
            result = (double)ad_mux_0.data[5];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_6:
            result = (double)ad_mux_0.data[6];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_7:
            result = (double)ad_mux_0.data[7];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_8:
            result = (double)ad_mux_0.data[8];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_9:
            result = (double)ad_mux_0.data[9];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_10:
            result = (double)ad_mux_0.data[10];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_11:
            result = (double)ad_mux_0.data[11];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_12:
            result = (double)ad_mux_0.data[12];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_13:
            result = (double)ad_mux_0.data[13];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_14:
            result = (double)ad_mux_0.data[14];
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_15:
            result = (double)ad_mux_0.data[15];
            break;

        // --- Default fallback ---
        default:
            result = 0.0;
            break;
    }
    // printf("[get_mapping_input_value] returning: %lld\n", result);
    // return value_buff;
    return result;
}

void set_mapping_default(int map_slot) {
    mappingData.map_mode[0][map_slot]=0;
    matrixData.index_mapped_value[0][map_slot]=0;
    mappingData.mapping_config[0][map_slot][0]=0;
    mappingData.mapping_config[0][map_slot][1]=0;
    mappingData.mapping_config[0][map_slot][2]=0;
    mappingData.mapping_config[0][map_slot][3]=0;
    mappingData.mapping_config[0][map_slot][4]=0;
    mappingData.mapping_config[0][map_slot][5]=0;
    mappingData.mapped_value[0][map_slot]=0;
}

void set_all_mapping_default(void) {
    for (int Mi=0; Mi < MAX_MAP_SLOTS; Mi++) {set_mapping_default(Mi);}
    Serial.println("$MAPPINGNEW");
}