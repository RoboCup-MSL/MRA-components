/**
 *  @file
 *  @brief   constants for the ball model
 *  @curator Rene van de Molengraft
 */

#ifndef CONSTANTS_BALL_MODELL_HPP
#define CONSTANTS_BALL_MODELL_HPP

/* ball velocity estimator */
#define CLIP_LOWER_BALL_VELOCITY    0.10 // was 0.25
#define CLIP_UPPER_BALL_VELOCITY    12.0

#define MAXBALLS_OV		20			/* maximum number of candidate balls found by omnivision and send to tracker
 	 	 	 	 	 	 	 	 	   NUM_BALLS defined in omni.h */

#define MAXBALLS_FC		3			/* maximum number of candidate balls found by front_cam and send to tracker */
#define MAXBALLS_LRF	10      	/* maximum number of candidate balls found by LRF-field and send to tracker*/


#ifndef DEFASSOCFLAG
#define DEFASSOCFLAG
/* define options for hypothesis association flag */
enum {  ASSOCIATE_WITH_BALL = 1,
        ASSOCIATE_WITH_CLUTTER,
        ASSOCIATE_WITH_NEW,
        ASSOCIATE_NONE
};
#endif

#define MAXHYP                  500             /* maximum number of hypotheses */
#define MA_N                    20              /* number of samples in MA confidence */
#define MAXHIST                 1               /* number of past generations to be logged */
#define ALPHA                   0.2            /* factor in best ball criterion (was 0.1) */
#define MAXFEATBUF              60              /* maximum number of stored features in a hypothesis */
#define MIN_ALLOWED_DET         0.000001        /* minimum allowed determinant for least squares fit */
#define MIN_NUMBER_OF_FEAT      4               /* minimum number of features to allow for fit */

#define LOWER_CONF_BOUND        0.5             /* if MA of hypothesis is below this bound, ball is thrown away */
												/* if this bound is set to zero, a low confidence ball will remain to be tracked. */

#define GRAVITY                 9.81            /* gravity constant in m/s² */

#define BM_SUCCESS               0              /* success */
#define BM_ERROR_MAXHYP         -1              /* MAXHYP exceeded */
#define BM_ERROR_NORM           -2              /* normalization error */
#define BM_ERROR_NO_HYP         -3              /* no hypotheses */
#define BM_ASSOCIATION_ERROR    -4              /* association not allowed */
#define BM_FIT_ERROR            -5              /* fit not possible */

#endif  // CONSTANTS_BM_H
