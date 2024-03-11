#ifndef __LH_LOCATION_H
#define __LH_LOCATION_H

/**
 * @defgroup    drv_lh_location    DotBot lh location driver
 * @ingroup     drv
 * @brief       Local computation of the LH location in the DotBot
 *
 * @{
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @copyright Inria, 2024
 * @}
 */

#include <stdint.h>
#include <stdbool.h>
#include "lh2.h"

//=========================== defines ==========================================

/// LH location algorithm to compute location
typedef enum {
    DB_LH_LOC_ONELH_2D,               ///< One-LH2_2D algorithm  
    DB_LH_LOC_TWOLH_2D,               ///< Two-LH2_2D algorithm  
} db_lh_loc_algorithm_t;

/// Calibration values for the oneLH-2D algorithm
typedef struct __attribute__((packed)) {
    float H[9];  ///< Homography matrix from LH2 angles to ground cartesian coordinates. order = {(0,0), (0,1), (0,2), (1,0), ..., (2,2)}
} db_one_lh_calibration_t;

/// Calibration values for the twoLH-2D algorithm
typedef struct __attribute__((packed)) {
    float R[9];  ///< Rotation matrix of the LH basestation. order = {(0,0), (0,1), (0,2), (1,0), ..., (2,2)}
    float t[3];  ///< Translation vector of the LH basestation. column vector
    float n[3];  ///< Normal vector of the ground plane. row vector
    float zeta;  ///< Zeta value of the twoLH-2D algorithm
} db_two_lh_calibration_t;

/// Cartesian coordinates for the LH location
typedef struct __attribute__((packed)) {
    float x;  ///< X - coordinate of the DotBot
    float y;  ///< Y - coordinate of the DotBot
    float z;  ///< Z - coordinate of the DotBot
} db_cartesian_3D_vector_t;

/// Single structure for the calibration values
typedef struct __attribute__((packed)) {
    db_one_lh_calibration_t oneLH_calib[LH2_BASESTATION_COUNT];  ///< selected poly is the polyomial # (between 0 and 31) that the demodulation code thinks the demodulated bits are a part of, initialize to error state
    db_two_lh_calibration_t twoLH_calib[LH2_BASESTATION_COUNT];  ///< LFSR location is the position in a given polynomial's LFSR that the decoded data is, initialize to error state
} db_lh_local_calibration_t;

//=========================== public ===========================================

/**
 * @brief Compute the (X,Y) location of the DotBot using the lighthouse data and the calibration computed at a PC.
 *
 * @param[in]   lh2 pointer to the lh2 instance
 * @param[in]   basestation which basestation to use for the computation
 * @param[in]   algorithm which algorithm to use
 * @param[out]  position pointer to the cartesian coordinates of the computed position
 * @return      True, new position computed. False, No position computed.
 */
bool db_lh_loc_compute_position(db_lh2_t *lh2, uint8_t basestation, db_lh_loc_algorithm_t algorithm, db_cartesian_3D_vector_t position);

/**
 * @brief Add a oneLH_2D algorithm calibration from a lighthouse.
 *
 * @param[in]   calib pointer to the new calibration structure
 * @param[in]   basestation pointer to the lh2 instance
 */
void db_lh_loc_add_oneLH_2D_calibration(db_one_lh_calibration_t *calib, uint8_t basestation);

/**
 * @brief Add a twoLH_2D algorithm calibration from a lighthouse
 *
 * @param[in]   calib pointer to the new calibration structure
 * @param[in]   basestation pointer to the lh2 instance
 */
void db_lh_loc_add_twoLH_2D_calibration(db_one_lh_calibration_t *calib, uint8_t basestation);


#endif 
