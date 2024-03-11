

/**
 * @file
 * @ingroup bsp_qdec
 *
 * @brief  nRF5340-specific definition of the "qdec" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lh_location.h"
#include "lh2.h"

//=========================== defines ==========================================

typedef struct {
    db_lh_local_calibration_t calib;                  ///< Stores the calibration values of oneLH-2d and twoLH-2D calibration
    bool                      oneLH_2D_calib_status;  ///< Array containing which LH have oneLH-2D calibrations available.
    bool                      twoLH_2D_calib_status;  ///< Array containing which LH have twoLH-2D calibrations available.
} lh_loc_vars_t;

typedef enum {
    DB_LH_LOC_CALIB_UNAVAILABLE,  ///< Calibration not available for this particular basestation/algorithm
    DB_LH_LOC_CALIB_AVAILABLE,    ///< Calibration available for this particular basestation/algorithm
} lh_loc_calib_status_t;

//=========================== variables ========================================

static lh_loc_vars_t _lh_loc_vars = { 0 };  ///< local data of the LH2 driver
//=========================== prototypes =======================================

/**
 * @brief OneLH2-2D location algorithm
 *
 * @param[in]   lh2 pointer to the lh2 instance
 * @param[in]   calib pointer to the oneLH calibration data
 */
void _oneLH_2D_algorithm(db_lh2_t *lh2, db_one_lh_calibration_t *calib);

/**
 * @brief TwoLH2-2D location algorithm
 *
 * @param[in]   lh2 pointer to the lh2 instance
 * @param[in]   calib pointer to the twoLH calibration data
 */
void _twoLH_2D_algorithm(db_lh2_t *lh2, db_two_lh_calibration_t *calib);

//=========================== public ===========================================

void db_lh_loc_init(db_lh2_t *lh2, const gpio_t *gpio_d, const gpio_t *gpio_e) {
}

void db_lh_loc_compute_position(db_lh2_t *lh2) {
    return (int16_t)_qdec_devs[qdec]->ACC;
}

//=========================== private ==========================================

void _oneLH_2D_algorithm(db_lh2_t *lh2, db_one_lh_calibration_t *calib) {

    // Local copy of the calibration values
    float H[9];
    memcpy(H, &calib->H, 9);

    //
}