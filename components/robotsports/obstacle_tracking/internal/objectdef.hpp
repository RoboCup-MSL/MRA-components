/**
 *  @file
 *  @brief   object definitions
 *  @curator Ton Peijnenburg
 */
#ifndef OBJECTDEF_HPP
#define OBJECTDEF_HPP 1

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
#define OBJECT_PLAYER_US 1
#define OBJECT_PLAYER_THEM 10

const int NO_FREE_LABEL_ID = -1;
const int NO_OBJECT_ID = -1;
const int SC_NO_ASSOCIATING_OBJECT_FOUND = -2;


typedef enum SequenceClusterCode_e {
    SC_SUCCESS = 0,
    SC_TOO_MANY_HYPOTESES = -1,
    SC_OUT_OF_KALLMAN_FILTERS = -2,
    SC_KALLMAN_NEGATIVE_TIME = -3,
    SC_MAX_REACHED = -4,
    SC_ERROR_ASSOCIATE_WITH_EXISTING_OBJECT_FREE_FILTER = -5,
    SC_ERROR_ASSOCIATE_WITH_EXISTING_OBJECT_NO_OBJECT = -6,
    SC_ERROR_ASSOCIATE_WITH_EXISTING_OBJECT_NO_FREE_LABEL = -7,
    SC_MAX_HYPOTHESES_REACHED = -8,
    SC_ERROR_IN_GENERATE_OFFSPRING_CLUTTER = -9,
    SC_ERROR_IN_GENERATE_OFFSPRING_ASSCOCIATE_NEW = -10,
    SC_ERROR_IN_GENERATE_OFFSPRING_ASSCOCIATE_EXISTING = -11,
    SC_ERROR_ASSOCIATE_WITH_NEW_OBJECT = -12,
    SC_ERROR_ASSOCIATE_WITH_NEW_OBJECT_NO_FREE_LABEL = -13,
    SC_CLIPTIME_ERROR = -14,
    SC_CLIPRECT_ERROR = -15,
    SC_NHYP_CONTROL_ERROR = -16,
    SC_NORMALISATION_FAILED = -17,
    SC_TOO_MANY_OBJECTS = -18,

} SequenceClusterCode_e;

// TODO decide on which value to use for sizing of arrays
// Tech United uses the following:
//   MAXNOBJ_LOCAL 10
//   MAXNOBJ_GLOBAL 12
//   MAX_TURTLES * MAXNOBJ_LOCAL equals 7 * 10 = 70
//
// Robot Sports uses
//   NUM_OBSTACLES 20
#endif
