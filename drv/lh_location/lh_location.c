

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
#include <math.h>
#include "lh_location.h"
#include "lh2.h"

//=========================== defines ==========================================

typedef struct {
    db_lh_local_calibration_t calib;                                         ///< Stores the calibration values of oneLH-2d and twoLH-2D calibration
    bool                      oneLH_2D_calib_status[LH2_BASESTATION_COUNT];  ///< Array containing which LH have oneLH-2D calibrations available.
    bool                      twoLH_2D_calib_status[LH2_BASESTATION_COUNT];  ///< Array containing which LH have twoLH-2D calibrations available.
} lh_loc_vars_t;

typedef enum {
    DB_LH_LOC_CALIB_UNAVAILABLE,  ///< Calibration not available for this particular basestation/algorithm
    DB_LH_LOC_CALIB_AVAILABLE,    ///< Calibration available for this particular basestation/algorithm
} lh_loc_calib_status_t;

//=========================== variables ========================================

static lh_loc_vars_t _lh_loc_vars = { 0 };  ///< local data of the LH2 driver

static uint32_t basestation_to_period[LH2_BASESTATION_COUNT] = {
    959000,
    957000,
    953000,
    949000
};

//=========================== prototypes =======================================

/**
 * @brief OneLH2-2D location algorithm
 *
 * @param[in]   lh2 pointer to the lh2 instance
 * @param[in]   basestation which basestation data to process.
 * @param[in]   calib pointer to the oneLH calibration data
 * @param[out]   output resulting vector of the transformation.
 */
void _oneLH_2D_algorithm(db_lh2_t *lh2, uint8_t basestation, db_one_lh_calibration_t *calib, db_cartesian_3D_vector_t *output);

/**
 * @brief TwoLH2-2D location algorithm
 *
 * @param[in]   lh2 pointer to the lh2 instance
 * @param[in]   calib pointer to the twoLH calibration data
 */
void _twoLH_2D_algorithm(db_lh2_t *lh2, db_two_lh_calibration_t *calib);

/**
 * @brief Trnasform  lfsr location from one basestation
 *
 * @param[in]   count_1 lfsr count of the first sweep
 * @param[in]   count_2 lfsr count of the second sweep
 * @param[in]   basestation which basestation produced these sweeps
 * @param[out]   cam_pts LH camera pixels (x,y)
 */
void _calculate_camera_point(uint32_t count_1, uint32_t count_2, uint8_t basestation, float cam_pts[2]);

/**
 * @brief Performs the following multiplication:  Matrix @ Vector
 *
 * @param[in]   matrix 3x3 matrix to multiply
 * @param[in]   vector 3x1 column vector to multiply
 * @param[out]   result result of the multiplication 3x1
 */
void multiplyMatrixByVector(float matrix[3][3], float vector[3], float result[3]);

//=========================== public ===========================================

bool db_lh_loc_compute_position(db_lh2_t *lh2, uint8_t basestation, db_lh_loc_algorithm_t algorithm, db_cartesian_3D_vector_t *position) {

    // Select which algorithm to use
    switch (algorithm) {

        // Use the one_LH2-2D algorithm
        case DB_LH_LOC_ONELH_2D:
        {
            // Check that a calibration is available for this lighthouse
            if (_lh_loc_vars.oneLH_2D_calib_status[basestation]) {

                // Check that there is data available in both sweeps
                if ((lh2->locations[0][basestation].lfsr_location != 0) && (lh2->locations[1][basestation].lfsr_location != 0)) {

                    // Compute location
                    _oneLH_2D_algorithm(lh2, basestation, &_lh_loc_vars.calib.oneLH_calib[basestation], position);

                    // Return success
                    return true;
                }
            }
        } break;

        // Use the one_LH2-2D algorithm
        case DB_LH_LOC_TWOLH_2D:
        {

        } break;

        // No matching algorithm, just leave
        default:
            break;
    }

    // If no position was calculated return an error
    return false;
}

void db_lh_loc_set_oneLH2D_calibration(db_one_lh_calibration_t *new_calib, uint8_t basestation) {

    // Copy the new calibration into the local storage
    memcpy(_lh_loc_vars.calib.oneLH_calib[basestation].H, new_calib->H, sizeof(float) * 9);

    // Set the status  of the calibration
    _lh_loc_vars.oneLH_2D_calib_status[basestation] = DB_LH_LOC_CALIB_AVAILABLE;
}

void db_lh_loc_set_twoLH2D_calibration(db_two_lh_calibration_t *new_calib, uint8_t basestation) {

    // Copy the new calibration into the local storage
    memcpy(_lh_loc_vars.calib.twoLH_calib[basestation].R, new_calib->R, sizeof(float) * 9);
    memcpy(_lh_loc_vars.calib.twoLH_calib[basestation].n, new_calib->n, sizeof(float) * 3);
    memcpy(_lh_loc_vars.calib.twoLH_calib[basestation].t, new_calib->t, sizeof(float) * 3);
    _lh_loc_vars.calib.twoLH_calib[basestation].zeta = new_calib->zeta;

    // Set the status  of the calibration
    _lh_loc_vars.twoLH_2D_calib_status[basestation] = DB_LH_LOC_CALIB_AVAILABLE;
}
//=========================== private ==========================================

void _oneLH_2D_algorithm(db_lh2_t *lh2, uint8_t basestation, db_one_lh_calibration_t *calib, db_cartesian_3D_vector_t *output) {

    // Local copy of the calibration values
    float H[3][3];
    memcpy(H, &calib->H, sizeof(H));

    // Copy the points to transform.
    uint32_t count_1 = lh2->locations[0][basestation].lfsr_location;
    uint32_t count_2 = lh2->locations[1][basestation].lfsr_location;

    // Transform lsfr location to LH_camera pixel points
    float cam_pts[2];
    _calculate_camera_point(count_1, count_2, basestation, cam_pts);

    // Perform the perspective transformation.
    float v_output[3];
    float homogeneous_cam_pts[3] = { cam_pts[0], cam_pts[1], 1.0 };
    multiplyMatrixByVector(H, homogeneous_cam_pts, v_output);

    // De-homogenize the resulting vector.
    output->x = v_output[0] / v_output[2];
    output->y = v_output[1] / v_output[2];
    output->z = 1.0;
}

void _calculate_camera_point(uint32_t count_1, uint32_t count_2, uint8_t basestation, float cam_pts[2]) {
    uint32_t period;
    float    a1, a2;

    // Determine the period based on poly
    period = basestation_to_period[basestation];

    // Calculate angles
    a1 = (count_1 * 8.0 / period) * 2 * M_PI;
    a2 = (count_2 * 8.0 / period) * 2 * M_PI;

    // Calculate cam_x and cam_y
    cam_pts[0] = -tan(0.5 * (a1 + a2));
    cam_pts[1] = -sin(fabs(a2 - a1) / 2.0 - 1.0471975511965976) / 0.5773502691896257;
}

void multiplyMatrixByVector(float matrix[3][3], float vector[3], float result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = 0;  // Initialize result element to 0
        for (int j = 0; j < 3; j++) {
            result[i] += matrix[i][j] * vector[j];
        }
    }
}