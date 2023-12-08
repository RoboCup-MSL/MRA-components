/**
 *  @file
 *  @brief   object definitions
 *  @curator Ton Peijnenburg
 */

/* object processing */
#define MAXNOBJ_LOCAL               10      /* as tracked by local tracker */
#define MAXNOBJ_GLOBAL              12      /* as tracked by global tracker */
#define MAX_TURTLES                 7

/* defines for the sequential clustering algorithm */
#define MAXFIL          10000               /* maximum number of Kalman filters */
#define MAXHYP_W         1000                /* maximum number of hypotheses */

#define LABEL_OFFSET    12                  /* starting label for opponents */
#define GRAV_CONST       9.81               /* Gravitational constant */


#define EPS2            1.0e-6              /* just a very small number, used by kalman_update */
#define RADIUS          0.25                /* radius is fixed for now */
#define DIM             5                   /* dimension of feature (currently 5: x, y, r, label, t) */

//TODO:
#define NRPLAYERS_COMPLETE_TEAM 10
#define BM_SUCCESS 0
#define OBJECT_PLAYER_US 1
#define OBJECT_PLAYER_THEM 10



// TODO decide on which value to use for sizing of arrays
// Tech United uses the following:
//   MAXNOBJ_LOCAL 10
//   MAXNOBJ_GLOBAL 12
//   MAX_TURTLES * MAXNOBJ_LOCAL equals 7 * 10 = 70
//
// Robot Sports uses
//   NUM_OBSTACLES 20
