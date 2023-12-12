/**
 *  @file
 *  @brief   sequential clustering header
 *  @curator Ton Peijnenburg
 */

#ifndef SC_TO_INCLUDE
#define SC_TO_INCLUDE

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sys/time.h>

#include "../../local_obstacle_tracking/internal/objectdef.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#define ASSOC_BUFFER_LENGTH 10

typedef struct tag_hypothesis_w {
    int hypothesis_id;                /* hypothesis identifier */
    int association_id;     /* identifier of the Kalman filter object that is associated with the current measurement */
    int filter_id[MAXNOBJ_GLOBAL];  /* array of Kalman filter identifiers, representing the detected objects according to this hypothesis */
    unsigned number_of_objects;               /* number of detected objects */
    double probalility;               /* probability of this hypothesis */
} hypothesis_w;

typedef struct tag_kalman_filter_w {
    double xh[4];           /* state estimate */
    bool active;             /* 1 means used, 0 means free */
    double time_last_update;  /* time of last update */
    double time_birth;           /* time of birth */
    unsigned associations[ASSOC_BUFFER_LENGTH];    /* number of associations per time step (ring buffer of length 8) */
    int assoc_ptr;          /* pointer to the current association index */
    double radius;          /* radius */
    int label;              /* object label, 0 = unknown object (default), 1..7 = turtle, 8.. = opponent */
} kalman_filter_w;

typedef struct tag_free_filter_w {
    unsigned index[MAXFIL];      /* index of free filters */
    unsigned ifree;              /* next free index */
    unsigned nfree;              /* number of free filters in current step */
} free_filter_w;

typedef struct tag_free_label_w {
    unsigned index[MAXFIL];      /* index of free labels for opponent labeling */
    unsigned ifree;              /* next free index */
    unsigned nfree;              /* number of free labels in current step */
} free_label_w;

typedef struct tag_scw_parameters {
    double pclutter;        /* probability that new measurement is clutter */
    double alpha;           /* pnew = alpha * pexist */
    double sigmax;          /* standard deviation measurement noise, x-direction */
    double sigmay;          /* standard deviation measurement noise, y-direction */
    unsigned nkeep;              /* number of hypotheses saved for next step */
    double pfactor;         /* reject hypotheses with p < pmax/pfactor */
    unsigned nselect;            /* number of selected obstacles from input list */
    unsigned maxnobj;            /* maximum number of objects in an hypothesis */
    double clipradius;      /* clipping radius relative to own position */
    double kscale;          /* scale factor for Kalman gain */
    double maxage;          /* maximum age for non-updated objects */
    double labelbound;      /* minimum required likelihood for object association */
} scw_parameters;

/* global data structure */
typedef struct tag_scw_global_data {
    hypothesis_w hyp[MAXHYP_W], hyp2[MAXHYP_W];
    kalman_filter_w kal[MAXFIL];
    scw_parameters par;
    free_filter_w ff;
    free_label_w fl;
    unsigned number_hypotheses;
    bool done;
    double current_time;
} scw_global_data;

// the following are external functions
SequenceClusterCode_e init_sc_wm(scw_global_data& pscgd, double time);
SequenceClusterCode_e sc_wm(double timestamp, double* pobj, double* pr, double* pobj_birthdate, double* pobj_assoc_buffer, double* plabel, unsigned& pnobj, double* pobst, int maxobst,  scw_global_data& pscgd);

#ifdef __cplusplus
}
#endif

#endif  // SC_WM_INCLUDE
