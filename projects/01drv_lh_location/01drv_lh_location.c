/**
 * @file
 * @ingroup samples_drv
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to calculate the position of the DotBot using the 1LH_2D algorithm.
 *

 *
 * @date 2024
 *
 * @copyright Inria, 2024
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "board_config.h"
#include "lh2.h"
#include "lh_location.h"

//=========================== defines ==========================================

//=========================== variables ========================================

static db_lh2_t _lh2;

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Initialize the board core features (voltage regulator)
    db_board_init();

    // Initialize the LH2
    db_lh2_init(&_lh2, &db_lh2_d, &db_lh2_e);
    db_lh2_start();

    while (1) {
        // wait until something happens e.g. an SPI interrupt
        __WFE();

        // the location function has to be running all the time
        db_lh2_process_location(&_lh2);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
