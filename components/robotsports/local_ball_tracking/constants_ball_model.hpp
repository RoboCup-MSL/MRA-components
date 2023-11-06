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
#define MAXHIST                 1               /* number of past generations to be logged */
#define MAXFEATBUF              60              /* maximum number of stored features in a hypothesis */

#define BM_SUCCESS               0              /* success */
#define BM_ERROR_MAXHYP         -1              /* MAXHYP exceeded */
#define BM_ERROR_NORM           -2              /* normalization error */
#define BM_ERROR_NO_HYP         -3              /* no hypotheses */
#define BM_ASSOCIATION_ERROR    -4              /* association not allowed */
#define BM_FIT_ERROR            -5              /* fit not possible */

#endif  // CONSTANTS_BM_H
