/**
 *  @file
 *  @brief   constants for the ball model
 *  @curator Rene van de Molengraft
 */

#ifndef CONSTANTS_BALL_MODELL_HPP
#define CONSTANTS_BALL_MODELL_HPP

/* define options for hypothesis association flag */
typedef enum {
    ASSOCIATE_WITH_BALL = 1,
    ASSOCIATE_WITH_CLUTTER,
    ASSOCIATE_WITH_NEW,
    ASSOCIATE_NONE
} associate_e;

#define MAXHYP                  500             /* maximum number of hypotheses */
#define MA_N                    20              /* number of samples in MA confidence */
#define MAXFEATBUF              60              /* maximum number of stored features in a hypothesis */

const int BM_SUCCESS              =  0;              /* success */
const int BM_ERROR_MAXHYP         = -1;              /* MAXHYP exceeded */
const int BM_ERROR_NORM           = -2;              /* normalization error */
const int BM_ERROR_NO_HYP         = -3;              /* no hypotheses */
const int BM_ASSOCIATION_ERROR    = -4;              /* association not allowed */
const int BM_FIT_ERROR            = -5;              /* fit not possible */

#endif  // CONSTANTS_BM_H
