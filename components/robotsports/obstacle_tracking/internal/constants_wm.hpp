
/* Header file with definitions for the world model source */

/* #include "global_par.h" */
#include "objectdef.hpp"

#define DIM                 5               /* Dimension of measurement */
#define DIM_MCCOMM          9               /* IsAlive + ball_xyz + current_xyo + labeling + comm_time */
#define GRAV_CONST       9.81               /* Gravitational constant */

/* defines for the sequential clustering algorithm */
#define MAXFIL          10000               /* maximum number of Kalman filters */
#define MAXHYP_W         1000                /* maximum number of hypotheses */

#define EPS2            1.0e-6              /* just a very small number, used by kalman_update */
#define RADIUS          0.25                /* radius is fixed for now */
#define DIM             5                   /* dimension of feature (currently 5: x, y, r, label, t) */
#define LABEL_OFFSET    12                  /* starting label for opponents */
#define FIELDMARGIN     0.6                 /* margin-outside-field for object clipping */
#define FIELDWIDTH 		12.0
#define FIELDLENGTH 	20.0


#define MODE_LOCAL      0
#define MODE_GLOBL      1
