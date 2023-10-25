/**
 *  @file
 *  @brief   sequential clustering header
 *  @curator Ton Peijnenburg
 */

#ifndef SEQ_CLUSTERING_BALL_MODEL_INCLUDE
#define SEQ_CLUSTERING_BALL_MODEL_INCLUDE

//#define BMDEBUG

#include <cstdio>
#include <cmath>
#include <cstring>
#include <sys/time.h>

#include "seq_clustering_balldef.hpp"
#include "constants_ball_model.hpp"


typedef struct tag_ball_observer {
        double xh[6];                   /* state estimate: x, xdot, y, ydot, z, zdot */
        double tupd;                    /* time of last update */
        int label;                      /* type of last added feature */
        int dupd;                       /* update flag, 0 = no update required, 1 = associate object with new feature */
        double time;                    /* time of current state estimate */
        int uid;                        /* uid set at creation */
} ball_observer;

typedef struct tag_history_t {
        double t;
        double x[6];
        int association_flag;
        ball_feature_t bfeat;
        double p_prior;
        double p_prediction;
        double p_likelihood;
        double p_posterior;
} history_t;

typedef struct tag_featbuf_t {
        double timestamp[MAXFEATBUF];
        double x[MAXFEATBUF];
        double y[MAXFEATBUF];
        double z[MAXFEATBUF];
        double sigma[MAXFEATBUF];
        double exp_time[MAXFEATBUF];
        int isFree[MAXFEATBUF];
} featbuf_t;

typedef struct tag_hypothesis {
        ball_observer obs;              /* observer representing the detected ball according to this hypothesis */
        int nobj;                       /* number of detected balls (0 or 1) */
        double p;                       /* probability of this hypothesis */
        int association_flag;           /* association of last feature (ASSOCIATE_WITH_CLUTTER or ASSOCIATE_WITH_BALL) */
        double ma_buf[MA_N+1];          /* MA confidence filter */
        double mavg;
        int ma_idx;
        int ma_first;
        int updated_in_timestep;
        history_t hist[MAXHIST];
        int hist_idx;
        int hist_full;
        featbuf_t fbuf;
        int fbuf_idx;
        int nfbuf;                      /* number of valid features in buffer, start at 0 */
} hypothesis;

typedef struct tag_sc_parameters {
        int nkeep;                      /* number of hypotheses saved for next step */
        double pfactor;                 /* reject hypotheses with p < pmax/pfactor */
        double maxage;                  /* maximum age for non-updated objects */
        double alpha;                   /* Pnew=alpha*(1-MA) */
        double beta;
        double min_allowed_sigma;       /* to prevent observer from exploding */
        double exp_time_free;
        double exp_time_non_free;
} sc_parameters;

/* global data structure */
typedef struct tag_sc_global_data {
        hypothesis hyp[MAXHYP], hyp2[MAXHYP];
        sc_parameters par;
        int nhyp;
        int new_uid;
        int track_uid;                  /* ball uid to keep track off */
        int next_done;
        int is_kicked;
        double vx0;			/* initial ball speed on kick */
        double vy0;
} sc_global_data;

// the following are external functions
int seq_clustering_ball_model(ball_estimate_t* pball, ball_feature_t* pbfeat, double time, int inext, sc_global_data * pscgd);
int init_seq_clustering(sc_global_data * pscgd);
int seq_clustering_print_hypotheses(sc_global_data * pscgd);


#endif  // SEQ_CLUSTERING_BALL_MODEL_INCLUDE
