/**
 *  @file
 *  @brief   sequential clustering algorithm
 *  @curator Ton Peijnenburg
 */

/* sequential clustering algorithm, Rene van de Molengraft, March, 2008 */

/*

 Inspired by Schubert & Sidenblath, Sequential clustering with particle filters - Estimating the number of clusters from data,
 Proc. 8th Int. Conf. Information Fusion, 2005

 Instead of using a particle filter, we apply a discrete filter to the set of hypotheses, which is of itself already a discrete stochast.

 As the ball is likely to move, we use a constant-velocity model-based Kalman filter to estimate the state of the ball.

 Last updated: March, 19th, 2008, added bound checking and error handling
 February, 13th, 2009, adapted for sorted measurements with varying time interval in between (kalman_update)
 added object labeling on creation of new object
 May, 8th, 2011, adapted for ball model
 May, 13th, 2011, assume a single ball; removed all book keeping for multiple objects
 June, 26th, 2011, exchanged recursive Kalman filter for batch-wise least-squares estimator (integral formulation)
 May, 20th, 2012, smooth free/non-free transition and kick-aware speed estimation

 NOTE: it is assumed that all ball features are within the field.

 TODO: make association sigma's and observer gains dependent on sensor type.
 add Branch Hopping Prevention
 maxage clipping assumes system time (gettimeofday)
 */

#include <cstring> // memcpy
#include <cmath>
#include "logging.hpp"
#include <string> // memcpy

#include "constants_ball_model.hpp"
#include "seq_clustering_ball_model.hpp"
#include "seq_clustering_ball_model_log.hpp"
#include "seq_clustering_best_uid.hpp"

/* function declaration */
static int generate_offspring(ball_feature_t *pbfeat, sc_global_data *pscgd);
#ifdef BMDEBUG
static int fbuf_print(hypothesis *phyp);
#endif
static int fbuf_init(hypothesis *phyp);
static int fbuf_add(hypothesis *phyp, ball_feature_t *pbfeat, sc_global_data *pscgd);
static void swap_index(int i, int j, int *idx);
static void isort_descending(int *idx, double *iarr, int size);
static void isort_ascending(int *idx, double *iarr, int size);
static double observer_update(ball_feature_t *pbfeat, sc_global_data *pscgd);
static int likelihood_update(ball_feature_t *pbfeat, sc_global_data *pscgd);
static int normalization(sc_global_data *pscgd);
static int sequence_clustering_nhyp_controller(int inext, sc_global_data *pscgd);
static int mape(sc_global_data *pscgd);

#ifdef BMDEBUG

static int fbuf_print(hypothesis *phyp) {
    MRA_LOG_DEBUG("hypothesis %d has %d feature(s) in buffer:", phyp->obs.uid, phyp->nfbuf);
    for (int i = 0; i < phyp->nfbuf; i++) {
        MRA_LOG_DEBUG("feature %d: t=%f, x=%f, y=%f, z=%f, sigma=%f", i, phyp->fbuf.timestamp[i], phyp->fbuf.x[i],
                phyp->fbuf.y[i], phyp->fbuf.z[i], phyp->fbuf.sigma[i]);
    }
    return BM_SUCCESS;
}

#endif

static std::string BM_result_to_string(int result) {
    std::string result_string = "";
    switch (result) {
    case BM_SUCCESS:
        result_string = "success";
        break;
    case BM_ERROR_MAXHYP:
        result_string = "MAXHYP exceeded";
        break;
    case BM_ERROR_NORM:
        result_string = "normalization error";
        break;
    case BM_ERROR_NO_HYP:
        result_string = "no hypotheses";
        break;
    case BM_ASSOCIATION_ERROR:
        result_string = "association not allowed";
        break;
    case BM_FIT_ERROR:
        result_string = "fit not possible";
        break;
    default:
        result_string = "unknown result";
        break;
    }

    return result_string;
}

static int fbuf_init(hypothesis *phyp) {
    /* clear feature buf */
    memset(&(phyp->fbuf), 0, sizeof(featbuf_t));

    /* no valid features yet */
    phyp->nfbuf = 0;

    /* start position in buffer */
    phyp->fbuf_idx = 0;

    return BM_SUCCESS;
}

static int fbuf_add(hypothesis *phyp, ball_feature_t *pbfeat, sc_global_data *pscgd) {
    phyp->fbuf.timestamp[phyp->fbuf_idx] = pbfeat->timestamp;
    phyp->fbuf.x[phyp->fbuf_idx] = pbfeat->x;
    phyp->fbuf.y[phyp->fbuf_idx] = pbfeat->y;
    phyp->fbuf.z[phyp->fbuf_idx] = pbfeat->z;
    phyp->fbuf.sigma[phyp->fbuf_idx] = pbfeat->sigma;
    /* set expiration time for feature */
    switch (pbfeat->isFree) {
    case 0:
        phyp->fbuf.exp_time[phyp->fbuf_idx] = pscgd->par.exp_time_non_free;
        break;
    case 1:
        phyp->fbuf.exp_time[phyp->fbuf_idx] = pscgd->par.exp_time_free;
        break;
    }
    phyp->fbuf.isFree[phyp->fbuf_idx] = pbfeat->isFree;

    if (phyp->nfbuf < MAXFEATBUF) {
        phyp->nfbuf++;
    }

    /* to next position in ring buffer */
    phyp->fbuf_idx++;
    if (phyp->fbuf_idx >= MAXFEATBUF) {
        phyp->fbuf_idx = 0;
    }

#ifdef BMDEBUG
    fbuf_print(phyp);
#endif
    return BM_SUCCESS;
}

static int fbuf_cleanup(hypothesis *phyp) {
    int idx1, idx2, i;

    idx1 = phyp->fbuf_idx - 1;
    if (idx1 < 0) {
        idx1 = MAXFEATBUF - 1;
    }

    idx2 = idx1 - 1;
    if (idx2 < 0) {
        idx2 = MAXFEATBUF - 1;
    }

    if (phyp->fbuf.exp_time[idx1] > phyp->fbuf.exp_time[idx2]) {
        /* transition from short to long expiration time */
        for (i = 0; i < MAXFEATBUF; i++) {
            if (phyp->fbuf.timestamp[i] < (phyp->fbuf.timestamp[idx1] - phyp->fbuf.exp_time[idx2])) {
                phyp->fbuf.timestamp[i] -= phyp->fbuf.exp_time[idx1];
            }
        }
    }
    return BM_SUCCESS;
}

static int ma_init(hypothesis *phyp) {
    int i;

    phyp->ma_first = 1;

    phyp->ma_idx = MA_N;
    if (phyp->ma_idx >= MA_N + 1) {
        phyp->ma_idx = phyp->ma_idx - (MA_N + 1);
    }

    /* initialize buffer */
    for (i = 0; i < MA_N + 1; i++) {
        phyp->ma_buf[i] = 0.01;
    }

    phyp->mavg = 0.01;

    return BM_SUCCESS;
}

static int ma_add(double val, hypothesis *phyp) {
    /* store val in buffer */
    phyp->ma_buf[phyp->ma_idx] = val;

    /* next index */
    phyp->ma_idx++;
    if (phyp->ma_idx >= MA_N + 1) {
        phyp->ma_idx = phyp->ma_idx - (MA_N + 1);
    }

    return BM_SUCCESS;
}

static double ma_get(hypothesis *phyp) {
    int idx1, idx2;

    idx1 = phyp->ma_idx - 1;
    if (idx1 < 0) {
        idx1 += MA_N + 1;
    }

    idx2 = idx1 - MA_N;
    if (idx2 < 0) {
        idx2 += MA_N + 1;
    }

    phyp->mavg += (phyp->ma_buf[idx1] - phyp->ma_buf[idx2]) / MA_N;

    return phyp->mavg;
}

static int fit_line_xy(double *theta, double *val, double *timestamp, double *sigma, int *idx, int n, double *exp_time,
        int *isFree, sc_global_data *pscgd) {
    int i;
    double a, b, c, d, e, f, w, det, t;

    /* too few data points */
    if (n < MIN_NUMBER_OF_FEAT) {
        return BM_FIT_ERROR;
    }

    /* initialization */
    a = 0.;
    b = 0.;
    c = 0.;
    d = 0.;
    e = 0.;
    f = 0.;

    for (i = 0; i < n; i++) {
        /* check for expiration of features in buffer */
        if (timestamp[idx[i]] < (timestamp[idx[0]] - exp_time[idx[0]])) {
            if (i < MIN_NUMBER_OF_FEAT) {
                return BM_FIT_ERROR;
            } /* not enough features to fit line */
            break;
        }

        /* use inverse variance as weighting factor */
        if (sigma[idx[i]] < pscgd->par.min_allowed_sigma) {
            sigma[idx[i]] = pscgd->par.min_allowed_sigma;
        }
        w = 1. / (sigma[idx[i]] * sigma[idx[i]]);

        /* most recent feature is at t=0 */
        t = timestamp[idx[i]] - timestamp[idx[0]];

        /* compute regression matrix and right-hand side */
        a += w * t * t;
        b += w * t;
        d += w;

        e += w * t * val[idx[i]];
        f += w * val[idx[i]];
    }

    //         MRA_LOG_DEBUG("fit_line_xy: using history of %d features in regression.", i);

    /* regression matrix is symmetric */
    c = b;

    /* determinant of regression matrix */
    det = a * d - b * c;
    if (fabs(det) < MIN_ALLOWED_DET) {
        return BM_FIT_ERROR;
    }

    /* compute optimal parameters in weighted least-squares sense */
    theta[0] = (d * e - b * f) / det;
    theta[1] = (a * f - c * e) / det;

    return BM_SUCCESS;
}

static int fit_curve_z(double *theta, double *val, double *timestamp, double *sigma, int *idx, int n, double *exp_time,
        int *isFree, sc_global_data *pscgd) {
    int i;
    double a, b, c, d, e, f, w, det, t, t2, v;

    /* too few data points */
    if (n < MIN_NUMBER_OF_FEAT) {
        return BM_FIT_ERROR;
    } /* 1 is too few, set to 4 to ignore few-points-estimate */

    /* initialization */
    a = 0.;
    b = 0.;
    c = 0.;
    d = 0.;
    e = 0.;
    f = 0.;

    for (i = 0; i < n; i++) {
        /* check for expiration of features in buffer and check if z is (close to ) 0 */
        if ((timestamp[idx[i]] < timestamp[idx[0]] - exp_time[idx[0]]) || (val[idx[i]] <= MIN_HEIGHT_IN_AIR)) {
            if (i < MIN_NUMBER_OF_FEAT) {
                return BM_FIT_ERROR;
            } /* not enough features to fit line */
            break;
        }

        /* use inverse variance as weighting factor */
        if (sigma[idx[i]] < pscgd->par.min_allowed_sigma) {
            sigma[idx[i]] = pscgd->par.min_allowed_sigma;
        }
        w = 1. / (sigma[idx[i]] * sigma[idx[i]]);

        /* most recent feature is at t=0 */
        t = timestamp[idx[i]] - timestamp[idx[0]];

        /* compute regression matrix and right-hand side */
        t2 = t * t;
        a += w * t2;
        b += w * t;
        d += w;

        /* gravity compensation */
        v = val[idx[i]] + 0.5 * GRAVITY * t2;

        e += w * t * v;
        f += w * v;
    }

    //MRA_LOG_DEBUG("fit_curve_z: using history of %d features in regression.", i);

    /* regression matrix is symmetric */
    c = b;

    /* determinant of regression matrix */
    det = a * d - b * c;
    if (fabs(det) < MIN_ALLOWED_DET) {
        return BM_FIT_ERROR;
    }

    /* compute optimal parameters in weighted least-squares sense */
    theta[0] = (d * e - b * f) / det;
    theta[1] = (a * f - c * e) / det;

    return BM_SUCCESS;
}

static void swap_index(int i, int j, int *idx) {
    int idx_bak = idx[i];
    idx[i] = idx[j];
    idx[j] = idx_bak;
}

static void isort_descending(int *idx, double *iarr, int size) {
    int i, j;

    for (i = 0; i < size; i++) {
        idx[i] = i;
    }
    for (i = 0; i < size; i++) {
        for (j = i; j < size; j++) {
            if (iarr[idx[j]] > iarr[idx[i]]) {
                swap_index(i, j, idx);
            }
        }
    }
}

static void isort_ascending(int *idx, double *iarr, int size) {
    int i, j;

    for (i = 0; i < size; i++) {
        idx[i] = i;
    }

    for (i = 0; i < size; i++) {
        for (j = i; j < size; j++) {
            if (iarr[idx[j]] < iarr[idx[i]]) {
                swap_index(i, j, idx);
            }
        }
    }
}

int seq_clustering_print_hypothesis(hypothesis *phyp, int i) {
    /* print hypothesis to screen */

    MRA_LOG_DEBUG("Hypothesis: %d", i);
    switch (phyp->nobj) {
    case 0:
        MRA_LOG_DEBUG("   No ball.");
        break;
    case 1:
        MRA_LOG_DEBUG("   Uid = %d", phyp->obs.uid);
        MRA_LOG_DEBUG("   Ball at (%f, %f, %f)", phyp->obs.xh[0], phyp->obs.xh[2], phyp->obs.xh[4]);
        MRA_LOG_DEBUG("   Type = %d", phyp->obs.label);
        MRA_LOG_DEBUG("   Tupd = %f", phyp->obs.tupd);
        if (phyp->obs.dupd == BU_TODO) {
            MRA_LOG_DEBUG("   Ball will be updated.");
        }
        MRA_LOG_DEBUG("   Association flag = %d.", phyp->association_flag);
        MRA_LOG_DEBUG("   Time = %f", phyp->obs.time);
        break;
    }
    MRA_LOG_DEBUG("   Probability = %f", phyp->p);
    MRA_LOG_DEBUG("   Confidence  = %f", phyp->mavg);

    return BM_SUCCESS;
}

int seq_clustering_print_hypotheses(sc_global_data *pscgd) {
    /* print overview to screen */
    for (int i = 0; i < pscgd->nhyp; i++) {
        seq_clustering_print_hypothesis(&(pscgd->hyp[i]), i);
    }
    return BM_SUCCESS;
}

static int associate_with_existing_ball(int i, int j, ball_feature_t *pbfeat, sc_global_data *pscgd) {
    double pexist;

    if (j >= MAXHYP) {
        return BM_ERROR_MAXHYP;
    }

    /* copy hypothesis */
    memcpy(&(pscgd->hyp2[j]), &(pscgd->hyp[i]), sizeof(hypothesis));

    /* add feature to buffer */
    fbuf_add(&(pscgd->hyp2[j]), pbfeat, pscgd);

    fbuf_cleanup(&(pscgd->hyp2[j]));

    /* add feature conf to MA filter */
    ma_add(pbfeat->conf, &(pscgd->hyp2[j]));

    /* MA confidence filter */
    ma_get(&(pscgd->hyp2[j]));

    /* update probability of hypothesis with prediction model */
    pexist = pscgd->par.alpha / 3.;
    pscgd->hyp2[j].p = pscgd->hyp2[j].p * pexist;

    /* store last feature type with ball */
    pscgd->hyp2[j].obs.label = pbfeat->type;

    /* ball needs to be updated */
    pscgd->hyp2[j].obs.dupd = BU_TODO;
    pscgd->hyp2[j].association_flag = ASSOCIATE_WITH_BALL;

    /* ball inherits timestamp from feature */
    pscgd->hyp2[j].obs.tupd = pbfeat->timestamp;

    return BM_SUCCESS;
}

static int associate_with_new_ball(int i, int j, ball_feature_t *pbfeat, sc_global_data *pscgd) {
    if (j >= MAXHYP) {
        return BM_ERROR_MAXHYP;
    }

    /* copy hypothesis */
    memcpy(&(pscgd->hyp2[j]), &(pscgd->hyp[i]), sizeof(hypothesis));

    /* no valid features yet for new ball */
    fbuf_init(&(pscgd->hyp2[j]));

    /* assign unique label */
    pscgd->hyp2[j].obs.uid = pscgd->new_uid;
    pscgd->new_uid++; /* uid for next ball... */

    /* add feature to buffer */
    fbuf_add(&(pscgd->hyp2[j]), pbfeat, pscgd);

    fbuf_cleanup(&(pscgd->hyp2[j]));

    /* clear moving average for new ball */
    ma_init(&(pscgd->hyp2[j]));

    /* add feature conf to MA filter */
    ma_add(pbfeat->conf, &(pscgd->hyp2[j]));

    /* MA confidence filter */
    ma_get(&(pscgd->hyp2[j]));

    pscgd->hyp2[j].nobj = 1; /* we have a new ball... */

    /* update probability of hypothesis with prediction model */
    double pnew = 1. - (pscgd->par.alpha + pscgd->par.beta) / 3.;
    if (pnew < 0.) {
        pnew = 0.;
    }
    pscgd->hyp2[j].p = pscgd->hyp2[j].p * pnew;

    /* initialize filter initial condition at measurement z (zero-velocity) */
    pscgd->hyp2[j].obs.xh[0] = pbfeat->x;
    pscgd->hyp2[j].obs.xh[1] = 0.0;
    pscgd->hyp2[j].obs.xh[2] = pbfeat->y;
    pscgd->hyp2[j].obs.xh[3] = 0.0;
    pscgd->hyp2[j].obs.xh[4] = pbfeat->z;
    pscgd->hyp2[j].obs.xh[5] = 0.0;

    /* initialize time of state estimate */
    pscgd->hyp2[j].obs.time = pbfeat->timestamp;

    /* store last feature type with ball */
    pscgd->hyp2[j].obs.label = pbfeat->type;

    /* ball does not need update on instantiation */
    pscgd->hyp2[j].obs.dupd = BU_NONE;
    pscgd->hyp2[j].association_flag = ASSOCIATE_WITH_NEW;

    /* ball inherits timestamp from feature */
    pscgd->hyp2[j].obs.tupd = pbfeat->timestamp;

    return BM_SUCCESS;
}

static int associate_with_clutter(int i, int j, ball_feature_t *pbfeat, sc_global_data *pscgd) {
    if (j >= MAXHYP) {
        return BM_ERROR_MAXHYP;
    }

    /* copy hypothesis */
    memcpy(&(pscgd->hyp2[j]), &(pscgd->hyp[i]), sizeof(hypothesis));

    /* don't update MA as this is not a ball */

    /* update probability of hypothesis with prediction model */
    pscgd->hyp2[j].p *= pscgd->par.beta / 3.;

    /* do (ball propagation and) likelihood correction */
    pscgd->hyp2[j].obs.dupd = BU_TODO;
    pscgd->hyp2[j].association_flag = ASSOCIATE_WITH_CLUTTER;

    return BM_SUCCESS;
}

static int generate_offspring(ball_feature_t *pbfeat, sc_global_data *pscgd) {
    /* generate offspring for hypotheses */

    int i, j, iret;

    j = 0; /* counter for next generation of hypotheses */

    for (i = 0; i < pscgd->nhyp; i++) {

#ifdef BMDEBUG
        MRA_LOG_DEBUG("hyp %d: nobj = %d   p = %f", i, pscgd->hyp[i].nobj, pscgd->hyp[i].p);
#endif

        switch (pscgd->hyp[i].nobj) {
        case 0: /* there's no ball yet */

            /* feature is new ball */
            iret = associate_with_new_ball(i, j, pbfeat, pscgd);
            if (iret < 0) {
                return iret;
            }
            j++; /* increment counter */

            break;
        case 1: /* there's a ball already */

            /* feature is existing ball */
            iret = associate_with_existing_ball(i, j, pbfeat, pscgd);
            if (iret < 0) {
                return iret;
            }
            j++; /* increment counter */

            /* feature is clutter */
            iret = associate_with_clutter(i, j, pbfeat, pscgd);
            if (iret < 0) {
                return iret;
            }
            j++; /* increment counter */

            /* feature is new ball (remove old) */
            iret = associate_with_new_ball(i, j, pbfeat, pscgd);
            if (iret < 0) {
                return iret;
            }
            j++; /* increment counter */

            break;
        }
    }

    pscgd->nhyp = j; /* new number of active hypotheses */
    memcpy(pscgd->hyp, pscgd->hyp2, pscgd->nhyp * sizeof(hypothesis)); /* copy back new generation */

    return BM_SUCCESS;
}

static double observer_update(ball_feature_t *pbfeat, sc_global_data *pscgd) {
    /* update ball for all hypotheses */

    int idx[MAXFEATBUF], i, j, n, c1, c2;
    double theta[2];
    ball_feature_t bfeat_reconstructed;
    double deltat;

    /* make copy of ball feature */
    memcpy(&bfeat_reconstructed, pbfeat, sizeof(ball_feature_t));
    /* deltat for kick speed reconstruction */
    deltat = pscgd->par.exp_time_non_free / (MIN_NUMBER_OF_FEAT + 1);
    //        mexPrintf("deltat = %f\n", deltat);

    for (i = 0; i < pscgd->nhyp; i++) {

        c1 = pscgd->hyp[i].obs.dupd == BU_TODO;
        c2 = pscgd->hyp[i].association_flag == ASSOCIATE_WITH_BALL;

        if (c1 && c2) { /* do ball update */

            /*	if ball is kicked, reset ball buffer and set initial speed via reconstructed past features */
            if (pscgd->is_kicked) {

                /* clear history for this hypothesis */
                fbuf_init(&(pscgd->hyp[i]));

                for (j = 0; j < MIN_NUMBER_OF_FEAT; j++) {
                    /* add reconstructed features */
                    n = MIN_NUMBER_OF_FEAT - j - 1;
                    bfeat_reconstructed.timestamp = pbfeat->timestamp - n * deltat;
                    bfeat_reconstructed.x = pbfeat->x - n * deltat * pscgd->vx0;
                    bfeat_reconstructed.y = pbfeat->y - n * deltat * pscgd->vy0;
                    fbuf_add(&(pscgd->hyp[i]), &bfeat_reconstructed, pscgd);
                }
                fbuf_cleanup(&(pscgd->hyp[i]));
            }

            /* sort stored features */
            isort_descending(idx, pscgd->hyp[i].fbuf.timestamp, MAXFEATBUF);

            //                        mexPrintf("nbuf van hyp %d = %d\n", i, pscgd->hyp[i].nfbuf);
            switch (pscgd->hyp[i].nfbuf) {
            case 0:
            case 1:
                pscgd->hyp[i].obs.xh[0] = pscgd->hyp[i].fbuf.x[idx[0]];
                pscgd->hyp[i].obs.xh[1] = 0.;
                pscgd->hyp[i].obs.xh[2] = pscgd->hyp[i].fbuf.y[idx[0]];
                pscgd->hyp[i].obs.xh[3] = 0.;
                pscgd->hyp[i].obs.xh[4] = pscgd->hyp[i].fbuf.z[idx[0]];
                pscgd->hyp[i].obs.xh[5] = 0.;
                break;
            default:
                if (fit_line_xy(theta, pscgd->hyp[i].fbuf.x, pscgd->hyp[i].fbuf.timestamp, pscgd->hyp[i].fbuf.sigma,
                        idx, pscgd->hyp[i].nfbuf, pscgd->hyp[i].fbuf.exp_time, pscgd->hyp[i].fbuf.isFree,
                        pscgd) == BM_SUCCESS) {
                    pscgd->hyp[i].obs.xh[0] = theta[1];
                    pscgd->hyp[i].obs.xh[1] = theta[0];
                    //                                        mexPrintf("hyp %d vx = %f\n", i, theta[0]);
                } else {
                    MRA_LOG_DEBUG("observer_update: fit_line_xy failed (x).");
                    pscgd->hyp[i].obs.xh[0] = pscgd->hyp[i].fbuf.x[idx[0]];
                    pscgd->hyp[i].obs.xh[1] = 0.0;
                }

                if (fit_line_xy(theta, pscgd->hyp[i].fbuf.y, pscgd->hyp[i].fbuf.timestamp, pscgd->hyp[i].fbuf.sigma,
                        idx, pscgd->hyp[i].nfbuf, pscgd->hyp[i].fbuf.exp_time, pscgd->hyp[i].fbuf.isFree,
                        pscgd) == BM_SUCCESS) {
                    pscgd->hyp[i].obs.xh[2] = theta[1];
                    pscgd->hyp[i].obs.xh[3] = theta[0];
                    //                                        mexPrintf("hyp %d vy = %f\n", i, theta[0]);
                } else {
                    MRA_LOG_DEBUG("observer_update: fit_line_xy failed (y).");
                    pscgd->hyp[i].obs.xh[2] = pscgd->hyp[i].fbuf.y[idx[0]];
                    pscgd->hyp[i].obs.xh[3] = 0.;
                }

                if (fit_curve_z(theta, pscgd->hyp[i].fbuf.z, pscgd->hyp[i].fbuf.timestamp, pscgd->hyp[i].fbuf.sigma,
                        idx, pscgd->hyp[i].nfbuf, pscgd->hyp[i].fbuf.exp_time, pscgd->hyp[i].fbuf.isFree,
                        pscgd) == BM_SUCCESS) {
                    pscgd->hyp[i].obs.xh[4] = theta[1];
                    pscgd->hyp[i].obs.xh[5] = theta[0];
                } else {
                    MRA_LOG_DEBUG("observer_update: fit_curve_z failed (z).");
                    pscgd->hyp[i].obs.xh[4] = pscgd->hyp[i].fbuf.z[idx[0]];
                    pscgd->hyp[i].obs.xh[5] = 0.;
                }
                break;
            }
#ifdef BMDEBUG
            MRA_LOG_DEBUG("tupd = %f, time = %f", pscgd->hyp[i].obs.tupd, pscgd->hyp[i].obs.time);
#endif
            /* ball time becomes update time */
            pscgd->hyp[i].obs.time = pscgd->hyp[i].obs.tupd;
        } else {
#ifdef BMDEBUG
            MRA_LOG_DEBUG("hyp %d not updated.", i);
#endif
        }
    }
    return BM_SUCCESS;
}

static int likelihood_update(ball_feature_t *pbfeat, sc_global_data *pscgd) {
    /* likelihood update of measurement z for all hypotheses that need to be updated */

    int i, j;
    double sigmax, sigmay, sigmaz, tttx, ttty, tttz, p = 0.0, pmax, pawn;

    sigmax = pbfeat->sigma;
    sigmay = sigmax;
    sigmaz = sigmax;

    /* analyse likelihood of new ball for this feature */
    pmax = 0.;
    for (j = 0; j < pscgd->nhyp; j++) {
        /* do not evaluate for the new balls themselves (hyp 2, 5, 8 etc) */
        if ((pscgd->nhyp > 1) && ((j + 1) % 3 > 0)) {
            tttx = pbfeat->x - pscgd->hyp[j].obs.xh[0];
            ttty = pbfeat->y - pscgd->hyp[j].obs.xh[2];
            tttz = pbfeat->z - pscgd->hyp[j].obs.xh[4];
            p = exp(
                    -0.5
                            * ((tttx * tttx) / (sigmax * sigmax) + (ttty * ttty) / (sigmay * sigmay)
                                    + (tttz * tttz) / (sigmaz * sigmaz)));
            MRA_LOG_DEBUG("  Existing ball hypothesis %d: p = %40.33f", j, p);
            if (p > pmax) {
                pmax = p;
            }
        }
    }
    pawn = 1. - pmax;

    for (i = 0; i < pscgd->nhyp; i++) {
        switch (pscgd->hyp[i].association_flag) {
        case ASSOCIATE_WITH_BALL:
            tttx = pbfeat->x - pscgd->hyp[i].obs.xh[0];
            ttty = pbfeat->y - pscgd->hyp[i].obs.xh[2];
            tttz = pbfeat->z - pscgd->hyp[i].obs.xh[4];
            p = exp(
                    -0.5
                            * ((tttx * tttx) / (sigmax * sigmax) + (ttty * ttty) / (sigmay * sigmay)
                                    + (tttz * tttz) / (sigmaz * sigmaz)));
#ifdef BMDEBUG
            MRA_LOG_DEBUG("hyp %d: ASSOCIATE_WITH_BALL: p factor = %f", i, p);
#endif
            /* reset update flag */
            pscgd->hyp[i].obs.dupd = BU_DONE;

            break;
        case ASSOCIATE_WITH_CLUTTER:
            tttx = pbfeat->x - pscgd->hyp[i].obs.xh[0];
            ttty = pbfeat->y - pscgd->hyp[i].obs.xh[2];
            tttz = pbfeat->z - pscgd->hyp[i].obs.xh[4];
            p = 1.
                    - exp(
                            -0.5
                                    * ((tttx * tttx) / (sigmax * sigmax) + (ttty * ttty) / (sigmay * sigmay)
                                            + (tttz * tttz) / (sigmaz * sigmaz)));
#ifdef BMDEBUG
            MRA_LOG_DEBUG("hyp %d: ASSOCIATE_WITH_CLUTTER: p factor = %f", i, p);
#endif
            /* reset update flag */
            pscgd->hyp[i].obs.dupd = BU_NONE;

            break;
        case ASSOCIATE_WITH_NEW:
            p = pawn;
#ifdef BMDEBUG
            MRA_LOG_DEBUG("hyp %d: ASSOCIATE_WITH_NEW: p factor = %f", i, p);
#endif
            /* reset update flag */
            pscgd->hyp[i].obs.dupd = BU_NONE;

            break;
        }
        pscgd->hyp[i].p = p * pscgd->hyp[i].p;
#ifdef BMDEBUG
        MRA_LOG_DEBUG("hyp %d: p = %f", i, pscgd->hyp[i].p);
#endif
    }

    return BM_SUCCESS;
}

static int normalization(sc_global_data *pscgd) {
    /* normalize probability of all hypotheses to sum one */

    int i;
    double sum = 0.0;

    for (i = 0; i < pscgd->nhyp; i++) {
#ifdef BMDEBUG
        MRA_LOG_DEBUG("norm: hyp[%d]: p = %f", i, pscgd->hyp[i].p);
#endif
        sum += pscgd->hyp[i].p;
    }
    if (sum <= 0.0) {
        return BM_ERROR_NORM;
    }
    for (i = 0; i < pscgd->nhyp; i++) {
        pscgd->hyp[i].p /= sum;
    }
    return BM_SUCCESS;
}

#ifndef NOCLIPCONF
static int clip_conf(sc_global_data *pscgd) {
    /* clip hypotheses w.r.t. balls that have been fed with low-confidence features for a while */
    for (int i = 0; i < pscgd->nhyp; i++) {
        if (pscgd->hyp[i].mavg < LOWER_CONF_BOUND) {
            pscgd->hyp[i].nobj = 0; /* throw away ball */
        }
    }
    return BM_SUCCESS;
}
#endif

static int sequence_clustering_set_track_uid_to_best(best_uid *puid, sc_global_data *pscgd) {
    //         int iret, id_valid;
    //
    //         /* set track_uid to best ball uid */
    //         iret = uid_get_uid(0, puid);
    //         id_valid = iret != UID_ERROR;
    //         switch (id_valid) {
    //         case 0:
    //                 /* no valid uid found */
    //                 return BM_ERROR_NO_HYP;
    //                 break;
    //         case 1:
    //                 /* valid uid found */
    //                 pscgd->track_uid = iret;
    //                 break;
    //         }
    int iret = uid_get_uid(0, puid);
    if (iret != UID_ERROR) {
        /* valid uid found */
        pscgd->track_uid = iret;
        return UID_SUCCESS;
    } else {
        /* no valid uid found */
        return BM_ERROR_NO_HYP;
    }
}

static int sequence_clustering_nhyp_controller(int inext, sc_global_data *pscgd) {
    /* filter discrete probability distribution of the hypotheses */

    int i, j, ifilter, idx[MAXHYP], n, uid_exists, id_valid, iret, k;
    double p[MAXHYP], psmall, pfilter, pmax;
    best_uid buid;

    /* sort p */
    n = pscgd->nhyp;
    if (n == 0) {
        return BM_ERROR_NO_HYP;
    }

    for (i = 0; i < n; i++) {
        p[i] = pscgd->hyp[i].p;
        idx[i] = i;
    }
    isort_ascending(idx, p, n);

    psmall = p[idx[n - 1]] / pscgd->par.pfactor;
#ifdef BMDEBUG
    MRA_LOG_DEBUG("psmall = %f", psmall);
#endif
    ifilter = n - pscgd->par.nkeep;
    if (ifilter < 0) {
        ifilter = 0;
    }

    pfilter = p[idx[ifilter]];
#ifdef BMDEBUG
    MRA_LOG_DEBUG("pfilter = %f", pfilter);
#endif
    /* apply filter action to the system... */
    j = 0;
    for (i = 0; i < n; i++) {
#ifdef BMDEBUG
        MRA_LOG_DEBUG("filter hyp %d: p = %f, uid = %d", idx[n - 1 - i], pscgd->hyp[idx[n - 1 - i]].p,
                pscgd->hyp[idx[n - 1 - i]].obs.uid);
#endif
        if ((pscgd->hyp[idx[n - 1 - i]].p >= pfilter) && (pscgd->hyp[idx[n - 1 - i]].p > psmall)
                && (j < pscgd->par.nkeep)) {
            memcpy(&(pscgd->hyp2[j]), &(pscgd->hyp[idx[n - 1 - i]]), sizeof(hypothesis));
            j++;
        }
    }

    if (j == 0) {
        return BM_ERROR_NO_HYP;
    }

    /* create list of at most MAXBEST best balls */
    i = 0;
    pmax = pscgd->hyp[idx[n - 1 - i]].p;
    uid_clear(&buid);
    while ((uid_get_n(&buid) < MAXBEST) && (pscgd->hyp[idx[n - 1 - i]].p >= ALPHA * pmax) && (i < n)) {
        uid_add(pscgd->hyp[idx[n - 1 - i]].obs.uid, pscgd->hyp[idx[n - 1 - i]].p, &buid);
        i++;
        if (n - 1 - i < 0) {
            break;
        }
    }

    //uid_print(&buid);

    MRA_LOG_DEBUG("track_uid voor = %d", pscgd->track_uid);
    /* check inext */
    if (inext && (!pscgd->next_done)) {
        /* next best ball selected by user */
        iret = uid_get_id(pscgd->track_uid, &buid);
        MRA_LOG_DEBUG("current track_uid = %d at id = %d", pscgd->track_uid, iret);
        uid_exists = iret != UID_ERROR;
        switch (uid_exists) {
        case 0:
            /* current track_uid not valid anymore, set track_uid to best ball uid */
            MRA_LOG_DEBUG("current track_uid not valid anymore, set track_uid to best ball uid.");
            iret = sequence_clustering_set_track_uid_to_best(&buid, pscgd);
            if (iret != UID_SUCCESS) {
                return iret;
            }
            break;
        case 1:
            /* track_uid still belongs to best balls, take next best ball */
            MRA_LOG_DEBUG("track_uid still belongs to best balls, take next best ball.");
            k = iret + 1;
            if (k > uid_get_n(&buid) - 1) {
                k = 0;
            }
            MRA_LOG_DEBUG("new id = %d", k);
            /* set track_uid to new ball uid */
            iret = uid_get_uid(k, &buid);
            MRA_LOG_DEBUG("Take next ball with id=%d and uid=%d", k, iret);
            id_valid = iret != UID_ERROR;
            switch (id_valid) {
            case 0:
                /* no valid uid found */
                return BM_ERROR_NO_HYP;
                break;
            case 1:
                /* valid uid found */
                pscgd->track_uid = iret;
                pscgd->next_done = 1;
                break;
            }
            break;
        }
    } else {
        uid_exists = uid_get_id(pscgd->track_uid, &buid) != UID_ERROR;
        switch (uid_exists) {
        case 0:
            /* current track_uid not valid anymore, set track_uid to best ball uid */
            MRA_LOG_DEBUG("current track_uid not valid anymore, set track_uid to best ball uid.");
            iret = sequence_clustering_set_track_uid_to_best(&buid, pscgd);
            if (iret != UID_SUCCESS) {
                return iret;
            }
            break;
        case 1:
            /* track_uid still belongs to best balls, keep current track_uid */
            MRA_LOG_DEBUG("track_uid still belongs to best balls, keep current track_uid.");
            break;
        }
    }

    MRA_LOG_DEBUG("track_uid na = %d, p = %f", pscgd->track_uid, buid.p[uid_get_id(pscgd->track_uid, &buid)]);

#ifdef BMDEBUG
    MRA_LOG_DEBUG("new number of hypotheses = %d", j);
#endif
    pscgd->nhyp = j; /* new number of active hypotheses */

    memcpy(pscgd->hyp, pscgd->hyp2, pscgd->nhyp * sizeof(hypothesis)); /* copy back new generation */

    return BM_SUCCESS;
}

static int mape(sc_global_data *pscgd) {
    /* get Maximum A Posteriori estimate from hypotheses with uid=track_uid */
    int i, i_mape = 0;

    /* find first hypothesis with uid=track_uid */
    for (i = 0; i < pscgd->nhyp; i++) {
        if (pscgd->hyp[i].obs.uid == pscgd->track_uid) {
            i_mape = i;
            break;
        }
    }

    if (i_mape < (pscgd->nhyp - 1)) {
        /* check if there exist better hypotheses with uid=track_uid */
        for (i = i_mape + 1; i < pscgd->nhyp; i++) {
            if ((pscgd->hyp[i].obs.uid == pscgd->track_uid) && (pscgd->hyp[i].p > pscgd->hyp[i_mape].p)) {
                i_mape = i;
            }
        }
    }
    return i_mape;
}

static int sequence_clustering_init_hyp(hypothesis *phyp) {
    /* initialize hypotheses */
    for (int i = 0; i < MAXHYP; i++) {
        (phyp + i)->nobj = 0;
        (phyp + i)->p = 1.0;
        ma_init(phyp + i);
        log_init(phyp + i);
        fbuf_init(phyp + i);
    }

    return BM_SUCCESS;
}

int init_seq_clustering(sc_global_data *pscgd) {
    /* tunable parameters of clustering algorithm */
    pscgd->par.nkeep = 16;
    pscgd->par.pfactor = 100.0;
    pscgd->par.maxage = 100.;

    /* initial number of hypotheses */
    pscgd->nhyp = 1;

    /* initialize hypotheses */
    sequence_clustering_init_hyp(pscgd->hyp);
    sequence_clustering_init_hyp(pscgd->hyp2);

    pscgd->new_uid = 0;

    pscgd->track_uid = INVALID_UID;

    return BM_SUCCESS;
}

int seq_clustering_ball_model(ball_estimate_t *pball, ball_feature_t *pbfeat, double time, int inext,
        sc_global_data *pscgd) {
    /*
     inputs:  pbfeat    - array of ball features (x, y, z, conf, dist, type, isFree, inAir, timestamp), padded with zeros
     time      - current nominal time
     inext     - if 1, then go to next best ball
     pscgd     - pointer to global workspace

     outputs: pball     - ball estimate (x, y, z, xdot, ydot, zdot, hconf, isUpd, label, timestamp), x, y are last feature
     pball2    - ball estimate, x,y are estimates
     */
    int n, i, j, i_mape, iret, idx;
    double vel, alpha;

    int idx_t[MAXBALLS], valid[MAXBALLS];
    double ts[MAXBALLS];

    pscgd->next_done = 0;

    //double t1 = get_time();

    /* find number of valid features */
    n = 0;
    for (i = 0; i < MAXBALLS; i++) {
#ifdef BMDEBUG
        MRA_LOG_DEBUG(" feature %d: x = %f, y = %f, z = %f, conf = %f", i, (pbfeat + i)->x, (pbfeat + i)->y,
                (pbfeat + i)->z, (pbfeat + i)->conf);
#endif
        if ((pbfeat + i)->conf > 0.) { /* valid features have positive confidence value */
            valid[n] = i; /* remember valid features */
            ts[n] = (pbfeat + i)->timestamp; /* store valid timestamps */
            n++;
        }
    }
#ifdef BMDEBUG
    MRA_LOG_DEBUG("sc_bm reports: %d valid features found at time %f.", n, time);
    for (i = 0; i < n; i++) {
        MRA_LOG_DEBUG("valid[%d] = %d, ts[%d] = %f", i, valid[i], i, ts[i]);
    }
#endif
    /* reset updated_in_timestep */
    for (j = 0; j < MAXHYP; j++) {
        pscgd->hyp[j].updated_in_timestep = 0;
    }

    if (n > 0) {
        /* sort w.r.t. ascending timestamps */
        isort_ascending(idx_t, ts, n);
#ifdef BMDEBUG
        for (i = 0; i < n; i++) {
            MRA_LOG_DEBUG("%d", idx_t[i]);
        }
#endif

#ifdef BMDEBUG
        MRA_LOG_DEBUG("sc_bm reports:");
        MRA_LOG_DEBUG("   time = %f s", time);
        MRA_LOG_DEBUG("   %d measurements at times:", n);
        for (i = 0; i < n; i++) {
            MRA_LOG_DEBUG("      %f x = %f, y = %f, z = %f, conf = %f", (pbfeat + valid[idx_t[i]])->timestamp,
                    (pbfeat + valid[idx_t[i]])->x, (pbfeat + valid[idx_t[i]])->y, (pbfeat + valid[idx_t[i]])->z,
                    (pbfeat + valid[idx_t[i]])->conf);
        }
#endif

        for (i = 0; i < n; i++) {
            /* propagate set of hypotheses */
            iret = generate_offspring(pbfeat + valid[idx_t[i]], pscgd);
            if (iret < 0) {
                return iret;
            }

#ifdef BMDEBUG
            MRA_LOG_DEBUG("Processing feature #%d:", i);
            seq_clustering_print_hypotheses(pscgd);
#endif

            /* apply observer update: prediction + measurement if associated */
            MRA_LOG_DEBUG("LINE: = %d", __LINE__);
            iret = observer_update(pbfeat + valid[idx_t[i]], pscgd);
            if (iret >= 0) {
                /* apply likelihood update */
                MRA_LOG_DEBUG("LINE: = %d", __LINE__);
                iret = likelihood_update(pbfeat + valid[idx_t[i]], pscgd);
            }
            if (iret >= 0) {
                MRA_LOG_DEBUG("LINE: = %d", __LINE__);
                /* normalize probability distribution */
                iret = normalization(pscgd);
            }
#ifndef NOCLIPCONF
            if (iret >= 0) {
                MRA_LOG_DEBUG("LINE: = %d", __LINE__);
                iret = clip_conf(pscgd);
            }
#endif

#ifdef BMDEBUG
            if (iret >= 0) {
                MRA_LOG_DEBUG("LINE: = %d", __LINE__);
                seq_clustering_print_hypotheses(pscgd);
            }
#endif
            if (iret >= 0) {
                /* gating of discrete probability distribution */
                MRA_LOG_DEBUG("LINE: = %d", __LINE__);
                iret = sequence_clustering_nhyp_controller(inext, pscgd);
            }

            if (iret < 0) {
                MRA_LOG_DEBUG("LINE: = %d", __LINE__);
#ifdef BMDEBUG
                MRA_LOG_DEBUG("seq_clustering_ball_model result = %s", BM_result_to_string(iret).c_str());
#endif
                return iret;
            }

            /* mark updated hypotheses as updated_in_timestep */
            for (j = 0; j < pscgd->nhyp; j++) {
                if ((pscgd->hyp[j].association_flag == ASSOCIATE_WITH_BALL)
                        || (pscgd->hyp[j].association_flag == ASSOCIATE_WITH_NEW)) {
                    pscgd->hyp[j].updated_in_timestep++;
                }
            }
        }
    } else {
    }

    /* Maximum A Posteriori (MAP) estimate */
    i_mape = mape(pscgd);
    MRA_LOG_DEBUG("p of winner = %f", pscgd->hyp[i_mape].p);

#ifdef BMDEBUG
    MRA_LOG_DEBUG("Winning hypothesis at t = %f: i_mape = %d, uid = %d, p = %f", time, i_mape,
            pscgd->hyp[i_mape].obs.uid, pscgd->hyp[i_mape].p);
    switch (pscgd->hyp[i_mape].nobj) {
    case 0:
        MRA_LOG_DEBUG("-> No ball.");
        break;
    case 1:
        MRA_LOG_DEBUG("-> Ball position: ");
        MRA_LOG_DEBUG("%f %f %f", pscgd->hyp[i_mape].obs.xh[0], pscgd->hyp[i_mape].obs.xh[2],
                pscgd->hyp[i_mape].obs.xh[4]);
        break;
    }
#endif

    /* return result in ball_estimate */
    idx = pscgd->hyp[i_mape].fbuf_idx - 1;
    if (idx < 0) {
        idx = MAXFEATBUF - 1;
    }
    pball->x = pscgd->hyp[i_mape].fbuf.x[idx]; /* the most recent feature */
    pball->y = pscgd->hyp[i_mape].fbuf.y[idx];
    pball->xhat = pscgd->hyp[i_mape].obs.xh[0];
    pball->yhat = pscgd->hyp[i_mape].obs.xh[2];

    pball->z = pscgd->hyp[i_mape].obs.xh[4];
    pball->xdot = pscgd->hyp[i_mape].obs.xh[1];
    pball->ydot = pscgd->hyp[i_mape].obs.xh[3];
    pball->zdot = pscgd->hyp[i_mape].obs.xh[5];

    /* apply ball velocity clipping */
    vel = hypot(pball->xdot, pball->ydot);

    MRA_LOG_DEBUG("xdot = %f, ydot = %f, vel = %f", pball->xdot, pball->ydot, vel);
    if (vel < CLIP_LOWER_BALL_VELOCITY) {
        pball->xdot = 0.0; /* set velocity to zero */
        pball->ydot = 0.0;
    }
    if (vel > CLIP_UPPER_BALL_VELOCITY) {
        alpha = CLIP_UPPER_BALL_VELOCITY / vel; /* scale velocity to match maximum */
        pball->xdot *= alpha;
        pball->ydot *= alpha;
    }

    if (pscgd->hyp[i_mape].nobj <= 0) {
        pball->hconf = 0.0;
    } else {
        pball->hconf = pscgd->hyp[i_mape].mavg;
    }

    pball->isUpd = pscgd->hyp[i_mape].updated_in_timestep > 0;
    MRA_LOG_DEBUG("Winning hypothesis updated_in_timestep = %d", pball->isUpd);

    pball->label = pscgd->hyp[i_mape].obs.label;

    pball->timestamp = pscgd->hyp[i_mape].obs.time;

    //        MRA_LOG_DEBUG("cputime = %d ms", (int) ((get_time() - t1) * 1000) );

    return BM_SUCCESS;
}
