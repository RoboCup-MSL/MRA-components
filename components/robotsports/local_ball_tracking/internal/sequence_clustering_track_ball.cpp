/**
 Sequential clustering algorithm

 Original author: Rene van de Molengraft, March, 2008

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

#include "sequence_clustering_track_ball.hpp"

#include <cstring> // memcpy
#include <cmath>
#include "logging.hpp"
#include <string> // memcpy

#include "../RobotsportsLocalBallTracking_datatypes.hpp"

#include "sequence_clustering_best_uid.hpp"

/* function declaration */
static int generate_offspring(const ball_candidate_t& pbfeat, sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params);

static int fbuf_print(hypothesis_t& r_hypothesis);
static int fbuf_init(hypothesis_t& r_hypothesis);
static int fbuf_add(hypothesis_t& r_hypothesis, const ball_candidate_t& pbfeat, sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params);
static void swap_index(int i, int j, int *idx);
static void isort_descending(int *idx, double *iarr, unsigned size);
static void isort_ascending(int *idx, double *iarr, unsigned size);
static double observer_update(const ball_candidate_t& pbfeat, sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params);
static int likelihood_update(const ball_candidate_t& pbfeat, sc_global_data_t& r_global_data);
static int normalization(sc_global_data_t& r_global_data);
static int sequence_clustering_nhyp_controller(int inext, sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params);
static int mape(sc_global_data_t& r_global_data);

// 2nd alpha defintion
#define ALPHA 0.2

static int fbuf_print(hypothesis_t& r_hypothesis) {
    MRA_LOG_DEBUG("hypothesis %d has %d feature(s) in buffer:", r_hypothesis.observer.uid, r_hypothesis.number_valid_buffers);
    for (unsigned i = 0; i < r_hypothesis.number_valid_buffers; i++) {
        MRA_LOG_DEBUG("feature %d: t=%f, x=%f, y=%f, z=%f, sigma=%f", i, r_hypothesis.fbuf.timestamp[i], r_hypothesis.fbuf.x[i],
                r_hypothesis.fbuf.y[i], r_hypothesis.fbuf.z[i], r_hypothesis.fbuf.sigma[i]);
    }
    return BM_SUCCESS;
}

static int fbuf_init(hypothesis_t& r_hypothesis)
{
        /* clear feature buf */
        memset(&(r_hypothesis.fbuf), 0, sizeof(featbuf_t));

        /* no valid features yet */
        r_hypothesis.number_valid_buffers = 0;

        /* start position in buffer */
        r_hypothesis.fbuf_idx = 0;

        return BM_SUCCESS;
}

static int ma_init(hypothesis_t& r_hypothesis)
{
        int i;

        r_hypothesis.ma_first = 1;

        r_hypothesis.ma_idx = MA_N;
        if (r_hypothesis.ma_idx >= MA_N + 1) {
            r_hypothesis.ma_idx = r_hypothesis.ma_idx - (MA_N + 1);
        }

        /* initialize buffer */
        for (i = 0; i < MA_N + 1; i++) {
            r_hypothesis.ma_buf[i] = 0.01;
        }

        r_hypothesis.mavg = 0.01;

        return BM_SUCCESS;
}


static int init_hyp(hypothesis_t hypothesises[MAXHYP]) {
    /* initialize hypotheses */
    for (int i = 0; i < MAXHYP; i++) {
        hypothesises[i].ball_detected = false;
        hypothesises[i].probability = 1.0;
        ma_init(hypothesises[i]);
        fbuf_init(hypothesises[i]);
    }

    return BM_SUCCESS;
}



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

static int fbuf_add(hypothesis_t& r_hypothesis, const ball_candidate_t& pbfeat, sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params) {
    r_hypothesis.fbuf.timestamp[r_hypothesis.fbuf_idx] = pbfeat.timestamp;
    r_hypothesis.fbuf.x[r_hypothesis.fbuf_idx] = pbfeat.x;
    r_hypothesis.fbuf.y[r_hypothesis.fbuf_idx] = pbfeat.y;
    r_hypothesis.fbuf.z[r_hypothesis.fbuf_idx] = pbfeat.z;
    r_hypothesis.fbuf.sigma[r_hypothesis.fbuf_idx] = pbfeat.sigma;
    /* set expiration time for feature */
    if (pbfeat.is_free) {
        r_hypothesis.fbuf.exp_time[r_hypothesis.fbuf_idx] = params.exp_time_free();
    }
    else {
        r_hypothesis.fbuf.exp_time[r_hypothesis.fbuf_idx] = params.exp_time_non_free();
    }
    r_hypothesis.fbuf.isFree[r_hypothesis.fbuf_idx] = pbfeat.is_free;

    if (r_hypothesis.number_valid_buffers < params.max_features_buffer()) {
        r_hypothesis.number_valid_buffers++;
    }

    /* to next position in ring buffer */
    r_hypothesis.fbuf_idx++;
    if (r_hypothesis.fbuf_idx >= params.max_features_buffer()) {
        r_hypothesis.fbuf_idx = 0;
    }


    fbuf_print(r_hypothesis);
    return BM_SUCCESS;
}

static int fbuf_cleanup(hypothesis_t& r_hypothesis, MRA::RobotsportsLocalBallTracking::Params const &params) {
    int idx1, idx2;

    if (r_hypothesis.fbuf_idx ==  0) {
        idx1 = params.max_features_buffer() - 1;
    }
    else {
        idx1 = r_hypothesis.fbuf_idx - 1;
    }

    idx2 = idx1 - 1;
    if (idx2 < 0) {
        idx2 = params.max_features_buffer() - 1;
    }

    if (r_hypothesis.fbuf.exp_time[idx1] > r_hypothesis.fbuf.exp_time[idx2]) {
        /* transition from short to long expiration time */
        for (unsigned i = 0; i < params.max_features_buffer(); i++) {
            if (r_hypothesis.fbuf.timestamp[i] < (r_hypothesis.fbuf.timestamp[idx1] - r_hypothesis.fbuf.exp_time[idx2])) {
                r_hypothesis.fbuf.timestamp[i] -= r_hypothesis.fbuf.exp_time[idx1];
            }
        }
    }
    return BM_SUCCESS;
}

static int ma_add(double val, hypothesis_t& r_hypothesis) {
    /* store val in buffer */
    r_hypothesis.ma_buf[r_hypothesis.ma_idx] = val;

    /* next index */
    r_hypothesis.ma_idx++;
    if (r_hypothesis.ma_idx >= MA_N + 1) {
        r_hypothesis.ma_idx = r_hypothesis.ma_idx - (MA_N + 1);
    }

    return BM_SUCCESS;
}

static double ma_get(hypothesis_t& r_hypothesis) {
    int idx1, idx2;

    idx1 = r_hypothesis.ma_idx - 1;
    if (idx1 < 0) {
        idx1 += MA_N + 1;
    }

    idx2 = idx1 - MA_N;
    if (idx2 < 0) {
        idx2 += MA_N + 1;
    }

    r_hypothesis.mavg += (r_hypothesis.ma_buf[idx1] - r_hypothesis.ma_buf[idx2]) / MA_N;

    return r_hypothesis.mavg;
}

static int fit_line_xy(double *theta, double *val, double *timestamp, double *sigma, int *idx, unsigned n, double *exp_time,
        bool *isFree, sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params) {
    unsigned i;
    double a, b, c, d, e, f, w, det, t;

    /* too few data points */
    if (n < params.mininum_number_of_features()) {
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
            if (i < params.mininum_number_of_features()) {
                return BM_FIT_ERROR;
            } /* not enough features to fit line */
            break;
        }

        /* use inverse variance as weighting factor */
        if (sigma[idx[i]] < params.min_allowed_sigma()) {
            sigma[idx[i]] = params.min_allowed_sigma();
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

    MRA_LOG_DEBUG("fit_line_xy: using history of %d features in regression.", i);

    /* regression matrix is symmetric */
    c = b;

    /* determinant of regression matrix */
    det = a * d - b * c;
    if (fabs(det) < params.minimum_allowed_determinant()) {
        return BM_FIT_ERROR;
    }

    /* compute optimal parameters in weighted least-squares sense */
    theta[0] = (d * e - b * f) / det;
    theta[1] = (a * f - c * e) / det;

    return BM_SUCCESS;
}

static int fit_curve_z(double *theta, double *val, double *timestamp, double *sigma, int *idx, unsigned n, double *exp_time,
        bool *isFree, sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params) {
    unsigned i;
    double a, b, c, d, e, f, w, det, t, t2, v;

    /* too few data points */
    if (n < params.mininum_number_of_features()) {
        return BM_FIT_ERROR;
    } /* 1 is too few, set to 4 to ignore few-points-estimate */

    /* initialization */
    a = 0.0;
    b = 0.0;
    c = 0.0;
    d = 0.0;
    e = 0.0;
    f = 0.0;

    for (i = 0; i < n; i++) {
        /* check for expiration of features in buffer and check if z is (close to ) 0 */
        if ((timestamp[idx[i]] < timestamp[idx[0]] - exp_time[idx[0]]) || (val[idx[i]] <= params.min_height_in_air())) {
            if (i < params.mininum_number_of_features()) {
                return BM_FIT_ERROR;
            } /* not enough features to fit line */
            break;
        }

        /* use inverse variance as weighting factor */
        if (sigma[idx[i]] < params.min_allowed_sigma()) {
            sigma[idx[i]] = params.min_allowed_sigma();
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
        v = val[idx[i]] + 0.5 * params.gravity() * t2;

        e += w * t * v;
        f += w * v;
    }

    //MRA_LOG_DEBUG("fit_curve_z: using history of %d features in regression.", i);

    /* regression matrix is symmetric */
    c = b;

    /* determinant of regression matrix */
    det = a * d - b * c;
    if (fabs(det) < params.minimum_allowed_determinant()) {
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

static void isort_descending(int *idx, double *iarr, unsigned size) {
    unsigned i, j;

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

static void isort_ascending(int *idx, double *iarr, unsigned size) {
    unsigned i, j;

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

int seq_clustering_print_hypothesis(hypothesis_t& r_hypothesis, int i) {
    /* print hypothesis to screen */

    MRA_LOG_DEBUG("Hypothesis: %d", i);
    if (r_hypothesis.ball_detected) {
        MRA_LOG_DEBUG("   Uid = %d (ball detected)", r_hypothesis.observer.uid);
        MRA_LOG_DEBUG("   Ball at (%f, %f, %f)", r_hypothesis.observer.xh[0], r_hypothesis.observer.xh[2], r_hypothesis.observer.xh[4]);
        MRA_LOG_DEBUG("   Type = %d", r_hypothesis.observer.label);
        MRA_LOG_DEBUG("   time_last_update = %f", r_hypothesis.observer.time_last_update);
        if (r_hypothesis.observer.ball_update == BU_TODO) {
            MRA_LOG_DEBUG("   Ball will be updated.");
        }
        MRA_LOG_DEBUG("   Association flag = %d.", r_hypothesis.association_flag);
        MRA_LOG_DEBUG("   Time = %f", r_hypothesis.observer.time);
    }
    else {
        MRA_LOG_DEBUG("   No ball.");
    }

    MRA_LOG_DEBUG("   Probability = %f", r_hypothesis.probability);
    MRA_LOG_DEBUG("   Confidence  = %f", r_hypothesis.mavg);

    return BM_SUCCESS;
}

int seq_clustering_print_hypotheses(sc_global_data_t& r_global_data) {
    /* print overview to screen */
    for (unsigned i = 0; i < r_global_data.number_of_hypothesis; i++) {
        seq_clustering_print_hypothesis(r_global_data.hypothesis[i], i);
    }
    return BM_SUCCESS;
}

static int associate_with_existing_ball(int i, int j, const ball_candidate_t& pbfeat, sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params) {
    double pexist;

    if (j >= MAXHYP) {
        return BM_ERROR_MAXHYP;
    }

    /* copy hypothesis */
    memcpy(&(r_global_data.hypothesis2[j]), &(r_global_data.hypothesis[i]), sizeof(hypothesis_t));

    /* add feature to buffer */
    fbuf_add(r_global_data.hypothesis2[j], pbfeat, r_global_data, params);

    fbuf_cleanup(r_global_data.hypothesis2[j], params);

    /* add feature conf to MA filter */
    ma_add(pbfeat.confidence, r_global_data.hypothesis2[j]);

    /* MA confidence filter */
    ma_get(r_global_data.hypothesis2[j]);

    /* update probability of hypothesis with prediction model */
    pexist = params.alpha() / 3.0;
    r_global_data.hypothesis2[j].probability = r_global_data.hypothesis2[j].probability * pexist;

    /* store last feature type with ball */
    r_global_data.hypothesis2[j].observer.label = pbfeat.type;

    /* ball needs to be updated */
    r_global_data.hypothesis2[j].observer.ball_update = BU_TODO;
    r_global_data.hypothesis2[j].association_flag = ASSOCIATE_WITH_BALL;

    /* ball inherits timestamp from feature */
    r_global_data.hypothesis2[j].observer.time_last_update = pbfeat.timestamp;

    return BM_SUCCESS;
}

static int associate_with_new_ball(int i, unsigned j, const ball_candidate_t& pbfeat, sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params) {
    if (j >= params.max_hypoteses()) {
        return BM_ERROR_MAXHYP;
    }

    /* copy hypothesis */
    memcpy(&(r_global_data.hypothesis2[j]), &(r_global_data.hypothesis[i]), sizeof(hypothesis_t));

    /* no valid features yet for new ball */
    fbuf_init(r_global_data.hypothesis2[j]);

    /* assign unique label */
    r_global_data.hypothesis2[j].observer.uid = r_global_data.new_uid;
    r_global_data.new_uid++; /* uid for next ball... */

    /* add feature to buffer */
    fbuf_add(r_global_data.hypothesis2[j], pbfeat, r_global_data, params);

    fbuf_cleanup(r_global_data.hypothesis2[j], params);

    /* clear moving average for new ball */
    ma_init(r_global_data.hypothesis2[j]);

    /* add feature conf to MA filter */
    ma_add(pbfeat.confidence, r_global_data.hypothesis2[j]);

    /* MA confidence filter */
    ma_get(r_global_data.hypothesis2[j]);

    r_global_data.hypothesis2[j].ball_detected = true; /* we have a new ball... */

    /* update probability of hypothesis with prediction model */
    double pnew = 1. - (params.alpha() + params.beta()) / 3.0;
    if (pnew < 0.0) {
        pnew = 0.0;
    }
    r_global_data.hypothesis2[j].probability = r_global_data.hypothesis2[j].probability * pnew;

    /* initialize filter initial condition at measurement z (zero-velocity) */
    r_global_data.hypothesis2[j].observer.xh[0] = pbfeat.x;
    r_global_data.hypothesis2[j].observer.xh[1] = 0.0;
    r_global_data.hypothesis2[j].observer.xh[2] = pbfeat.y;
    r_global_data.hypothesis2[j].observer.xh[3] = 0.0;
    r_global_data.hypothesis2[j].observer.xh[4] = pbfeat.z;
    r_global_data.hypothesis2[j].observer.xh[5] = 0.0;

    /* initialize time of state estimate */
    r_global_data.hypothesis2[j].observer.time = pbfeat.timestamp;

    /* store last feature type with ball */
    r_global_data.hypothesis2[j].observer.label = pbfeat.type;

    /* ball does not need update on instantiation */
    r_global_data.hypothesis2[j].observer.ball_update = BU_NONE;
    r_global_data.hypothesis2[j].association_flag = ASSOCIATE_WITH_NEW;

    /* ball inherits timestamp from feature */
    r_global_data.hypothesis2[j].observer.time_last_update = pbfeat.timestamp;

    return BM_SUCCESS;
}

static int associate_with_clutter(int i, int j, const ball_candidate_t& pbfeat, sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params) {
    if (j >= MAXHYP) {
        return BM_ERROR_MAXHYP;
    }

    /* copy hypothesis */
    memcpy(&(r_global_data.hypothesis2[j]), &(r_global_data.hypothesis[i]), sizeof(hypothesis_t));

    /* don't update MA as this is not a ball */

    /* update probability of hypothesis with prediction model */
    r_global_data.hypothesis2[j].probability *= params.beta() / 3.0;

    /* do (ball propagation and) likelihood correction */
    r_global_data.hypothesis2[j].observer.ball_update = BU_TODO;
    r_global_data.hypothesis2[j].association_flag = ASSOCIATE_WITH_CLUTTER;

    return BM_SUCCESS;
}

static int generate_offspring(const ball_candidate_t &pbfeat, sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params) {
    /* generate offspring for hypotheses */

    int iret = 0;;
    int nxt_gen_hyp_counter = 0;  /* counter for next generation of hypotheses */

    for (unsigned hypothese_idx = 0; hypothese_idx < r_global_data.number_of_hypothesis; hypothese_idx++) {


        MRA_LOG_DEBUG("hyp %d: ball_detected = %d   probability = %f", hypothese_idx, r_global_data.hypothesis[hypothese_idx].ball_detected, r_global_data.hypothesis[hypothese_idx].probability);

        if (r_global_data.hypothesis[hypothese_idx].ball_detected) {
            /* there's a ball already */

            /* feature is existing ball */
            iret = associate_with_existing_ball(hypothese_idx, nxt_gen_hyp_counter, pbfeat, r_global_data, params);
            MRA_LOG_DEBUG("generate_offspring::associate_with_existing_ball iret: %d", iret );
            if (iret < 0) {
                return iret;
            }
            nxt_gen_hyp_counter++; /* increment counter */

            /* feature is clutter */
            iret = associate_with_clutter(hypothese_idx, nxt_gen_hyp_counter, pbfeat, r_global_data, params);
            MRA_LOG_DEBUG("generate_offspring::associate_with_clutter iret: %d", iret );
            if (iret < 0) {
                return iret;
            }
            nxt_gen_hyp_counter++; /* increment counter */

            /* feature is new ball (remove old) */
            iret = associate_with_new_ball(hypothese_idx, nxt_gen_hyp_counter, pbfeat, r_global_data, params);
            MRA_LOG_DEBUG("generate_offspring::associate_with_new_ball (remove old) iret: %d", iret );
            if (iret < 0) {
                return iret;
            }
            nxt_gen_hyp_counter++; /* increment counter */

        }
        else {
            /* there's no ball yet */

            /* feature is new ball */
            iret = associate_with_new_ball(hypothese_idx, nxt_gen_hyp_counter, pbfeat, r_global_data, params);
            MRA_LOG_DEBUG("generate_offspring::associate_with_new_ball (no ball yet) iret: %d", iret );
            if (iret < 0) {
                return iret;
            }
            nxt_gen_hyp_counter++; /* increment counter */
        }
    }

    r_global_data.number_of_hypothesis = nxt_gen_hyp_counter; /* new number of active hypotheses */
    memcpy(r_global_data.hypothesis, r_global_data.hypothesis2, r_global_data.number_of_hypothesis * sizeof(hypothesis_t)); /* copy back new generation */

    return BM_SUCCESS;
}

static double observer_update(const ball_candidate_t& pbfeat, sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params) {
    /* update ball for all hypotheses */

    int n, c1, c2, idx[MAXFEATBUF];
    double theta[2];
    ball_candidate_t bfeat_reconstructed;
    double deltat;

    /* make copy of ball feature */
    memcpy(&bfeat_reconstructed, &pbfeat, sizeof(ball_candidate_t));
    /* deltat for kick speed reconstruction */
    deltat = params.exp_time_non_free() / (params.mininum_number_of_features() + 1);

    for (unsigned i = 0; i < r_global_data.number_of_hypothesis; i++) {

        c1 = r_global_data.hypothesis[i].observer.ball_update == BU_TODO;
        c2 = r_global_data.hypothesis[i].association_flag == ASSOCIATE_WITH_BALL;

        if (c1 && c2) { /* do ball update */

            /*	if ball is kicked, reset ball buffer and set initial speed via reconstructed past features */
            if (r_global_data.is_kicked) {

                /* clear history for this hypothesis */
                fbuf_init(r_global_data.hypothesis[i]);

                for (unsigned j = 0; j < params.mininum_number_of_features(); j++) {
                    /* add reconstructed features */
                    n = params.mininum_number_of_features() - j - 1;
                    bfeat_reconstructed.timestamp = pbfeat.timestamp - n * deltat;
                    bfeat_reconstructed.x = pbfeat.x - n * deltat * r_global_data.vx0;
                    bfeat_reconstructed.y = pbfeat.y - n * deltat * r_global_data.vy0;
                    fbuf_add(r_global_data.hypothesis[i], bfeat_reconstructed, r_global_data, params);
                }
                fbuf_cleanup(r_global_data.hypothesis[i], params);
            }

            /* sort stored features */
            isort_descending(idx, r_global_data.hypothesis[i].fbuf.timestamp, params.max_features_buffer());

            if  (r_global_data.hypothesis[i].number_valid_buffers == 0 || r_global_data.hypothesis[i].number_valid_buffers == 1) {
                r_global_data.hypothesis[i].observer.xh[0] = r_global_data.hypothesis[i].fbuf.x[idx[0]];
                r_global_data.hypothesis[i].observer.xh[1] = 0.;
                r_global_data.hypothesis[i].observer.xh[2] = r_global_data.hypothesis[i].fbuf.y[idx[0]];
                r_global_data.hypothesis[i].observer.xh[3] = 0.;
                r_global_data.hypothesis[i].observer.xh[4] = r_global_data.hypothesis[i].fbuf.z[idx[0]];
                r_global_data.hypothesis[i].observer.xh[5] = 0.;
            }
            else{
                if (fit_line_xy(theta, r_global_data.hypothesis[i].fbuf.x, r_global_data.hypothesis[i].fbuf.timestamp, r_global_data.hypothesis[i].fbuf.sigma,
                        idx, r_global_data.hypothesis[i].number_valid_buffers, r_global_data.hypothesis[i].fbuf.exp_time, r_global_data.hypothesis[i].fbuf.isFree,
                        r_global_data, params) == BM_SUCCESS) {
                    r_global_data.hypothesis[i].observer.xh[0] = theta[1];
                    r_global_data.hypothesis[i].observer.xh[1] = theta[0];
                } else {
                    MRA_LOG_DEBUG("observer_update: fit_line_xy failed (x).");
                    r_global_data.hypothesis[i].observer.xh[0] = r_global_data.hypothesis[i].fbuf.x[idx[0]];
                    r_global_data.hypothesis[i].observer.xh[1] = 0.0;
                }

                if (fit_line_xy(theta, r_global_data.hypothesis[i].fbuf.y, r_global_data.hypothesis[i].fbuf.timestamp, r_global_data.hypothesis[i].fbuf.sigma,
                        idx, r_global_data.hypothesis[i].number_valid_buffers, r_global_data.hypothesis[i].fbuf.exp_time, r_global_data.hypothesis[i].fbuf.isFree,
                        r_global_data, params) == BM_SUCCESS) {
                    r_global_data.hypothesis[i].observer.xh[2] = theta[1];
                    r_global_data.hypothesis[i].observer.xh[3] = theta[0];
                } else {
                    MRA_LOG_DEBUG("observer_update: fit_line_xy failed (y).");
                    r_global_data.hypothesis[i].observer.xh[2] = r_global_data.hypothesis[i].fbuf.y[idx[0]];
                    r_global_data.hypothesis[i].observer.xh[3] = 0.;
                }

                if (fit_curve_z(theta, r_global_data.hypothesis[i].fbuf.z, r_global_data.hypothesis[i].fbuf.timestamp, r_global_data.hypothesis[i].fbuf.sigma,
                        idx, r_global_data.hypothesis[i].number_valid_buffers, r_global_data.hypothesis[i].fbuf.exp_time, r_global_data.hypothesis[i].fbuf.isFree,
                        r_global_data, params) == BM_SUCCESS) {
                    r_global_data.hypothesis[i].observer.xh[4] = theta[1];
                    r_global_data.hypothesis[i].observer.xh[5] = theta[0];
                } else {
                    MRA_LOG_DEBUG("observer_update: fit_curve_z failed (z).");
                    r_global_data.hypothesis[i].observer.xh[4] = r_global_data.hypothesis[i].fbuf.z[idx[0]];
                    r_global_data.hypothesis[i].observer.xh[5] = 0.;
                }
            }

            MRA_LOG_DEBUG("time_last_update = %f, time = %f", r_global_data.hypothesis[i].observer.time_last_update, r_global_data.hypothesis[i].observer.time);
            /* ball time becomes update time */
            r_global_data.hypothesis[i].observer.time = r_global_data.hypothesis[i].observer.time_last_update;
        } else {

            MRA_LOG_DEBUG("hyp %d not updated.", i);
        }
    }
    return BM_SUCCESS;
}

static int likelihood_update(const ball_candidate_t& pbfeat, sc_global_data_t& r_global_data) {
    /* likelihood update of measurement z for all hypotheses that need to be updated */

    double sigmax, sigmay, sigmaz, tttx, ttty, tttz, probability = 0.0, pmax, pawn;

    sigmax = pbfeat.sigma;
    sigmay = sigmax;
    sigmaz = sigmax;

    /* analyse likelihood of new ball for this feature */
    pmax = 0.;
    for (unsigned j = 0; j < r_global_data.number_of_hypothesis; j++) {
        /* do not evaluate for the new balls themselves (hyp 2, 5, 8 etc) */
        if ((r_global_data.number_of_hypothesis > 1) && ((j + 1) % 3 > 0)) {
            tttx = pbfeat.x - r_global_data.hypothesis[j].observer.xh[0];
            ttty = pbfeat.y - r_global_data.hypothesis[j].observer.xh[2];
            tttz = pbfeat.z - r_global_data.hypothesis[j].observer.xh[4];
            probability = exp(
                    -0.5
                            * ((tttx * tttx) / (sigmax * sigmax) + (ttty * ttty) / (sigmay * sigmay)
                                    + (tttz * tttz) / (sigmaz * sigmaz)));
            MRA_LOG_DEBUG("  Existing ball hypothesis %d: probability = %40.33f", j, probability);
            if (probability > pmax) {
                pmax = probability;
            }
        }
    }
    pawn = 1. - pmax;

    for (unsigned i = 0; i < r_global_data.number_of_hypothesis; i++) {
        switch (r_global_data.hypothesis[i].association_flag) {
        case ASSOCIATE_WITH_BALL:
            tttx = pbfeat.x - r_global_data.hypothesis[i].observer.xh[0];
            ttty = pbfeat.y - r_global_data.hypothesis[i].observer.xh[2];
            tttz = pbfeat.z - r_global_data.hypothesis[i].observer.xh[4];
            probability = exp(-0.5* ((tttx * tttx) / (sigmax * sigmax) + (ttty * ttty) / (sigmay * sigmay)
                           + (tttz * tttz) / (sigmaz * sigmaz)));
            MRA_LOG_DEBUG("hyp %d: ASSOCIATE_WITH_BALL: probability factor = %f", i, probability);
            /* reset update flag */
            r_global_data.hypothesis[i].observer.ball_update = BU_DONE;
            break;
        case ASSOCIATE_WITH_CLUTTER:
            tttx = pbfeat.x - r_global_data.hypothesis[i].observer.xh[0];
            ttty = pbfeat.y - r_global_data.hypothesis[i].observer.xh[2];
            tttz = pbfeat.z - r_global_data.hypothesis[i].observer.xh[4];
            probability = 1.- exp(-0.5* ((tttx * tttx) / (sigmax * sigmax) + (ttty * ttty) / (sigmay * sigmay)
                               + (tttz * tttz) / (sigmaz * sigmaz)));

            MRA_LOG_DEBUG("hyp %d: ASSOCIATE_WITH_CLUTTER: probability factor = %f", i, probability);
            /* reset update flag */
            r_global_data.hypothesis[i].observer.ball_update = BU_NONE;
            break;
        case ASSOCIATE_WITH_NEW:
            probability = pawn;

            MRA_LOG_DEBUG("hyp %d: ASSOCIATE_WITH_NEW: p factor = %f", i, probability);
            /* reset update flag */
            r_global_data.hypothesis[i].observer.ball_update = BU_NONE;

            break;
        case ASSOCIATE_NONE:

            break;
        }
        r_global_data.hypothesis[i].probability = probability * r_global_data.hypothesis[i].probability;

        MRA_LOG_DEBUG("hyp %d: probability = %f", i, r_global_data.hypothesis[i].probability);
    }

    return BM_SUCCESS;
}

static int normalization(sc_global_data_t& r_global_data) {
    /* normalize probability of all hypotheses to sum one */

    double sum = 0.0;

    for (unsigned i = 0; i < r_global_data.number_of_hypothesis; i++) {

        MRA_LOG_DEBUG("norm: hyp[%d]: p = %f", i, r_global_data.hypothesis[i].probability);
        sum += r_global_data.hypothesis[i].probability;
    }
    if (sum <= 0.0) {
        return BM_ERROR_NORM;
    }
    for (unsigned i = 0; i < r_global_data.number_of_hypothesis; i++) {
        r_global_data.hypothesis[i].probability /= sum;
    }
    return BM_SUCCESS;
}

#ifndef NOCLIPCONF
static int clip_conf(sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params) {
    /* clip hypotheses w.r.t. balls that have been fed with low-confidence features for a while */
    for (unsigned i = 0; i < r_global_data.number_of_hypothesis; i++) {
        if (r_global_data.hypothesis[i].mavg < params.lower_confidence_bound()) {
            MRA_LOG_DEBUG("clip_: throw away ball confidence too low. value: %6.3f < %6.3f (threshold)", r_global_data.hypothesis[i].mavg,params.lower_confidence_bound());
            r_global_data.hypothesis[i].ball_detected = false; /* throw away ball */
        }
    }
    return BM_SUCCESS;
}
#endif

static int sequence_clustering_set_track_uid_to_best(const best_uid_t& best_uid, sc_global_data_t& r_global_data) {
     /* set track_uid to best ball uid */
    int iret = uid_get_uid(0, best_uid);
    if (iret != UID_ERROR) {
        /* valid uid found */
        r_global_data.track_uid = iret;
        return BM_SUCCESS;
    } else {
        MRA_LOG_DEBUG("BM_ERROR_NO_HYP: no valid uid found");
        return BM_ERROR_NO_HYP;
    }
}

static int sequence_clustering_nhyp_controller(int inext, sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params) {
    /* filter discrete probability distribution of the hypotheses */

    int ifilter, idx[MAXHYP],iret, k;
    bool uid_exists;
    double p[MAXHYP], psmall, pfilter, pmax;
    best_uid_t buid;

    /* sort p */
    unsigned n = r_global_data.number_of_hypothesis;
    if (n == 0) {
        MRA_LOG_DEBUG("BM_ERROR_NO_HYP: r_global_data.nhyp");
        return BM_ERROR_NO_HYP;
    }

    for (unsigned i = 0; i < n; i++) {
        p[i] = r_global_data.hypothesis[i].probability;
        idx[i] = i;
    }
    isort_ascending(idx, p, n);

    psmall = p[idx[n-1]] / params.pfactor();

    MRA_LOG_DEBUG("psmall = %f", psmall);
    ifilter = n - params.nkeep();
    if (ifilter < 0) {
        ifilter = 0;
    }

    pfilter = p[idx[ifilter]];

    MRA_LOG_DEBUG("pfilter = %f", pfilter);
    /* apply filter action to the system... */
    unsigned j = 0;
    for (unsigned i = 0; i < n; i++) {
        MRA_LOG_DEBUG("filter hyp %d: probability = %f, uid = %d", idx[n - 1 - i], r_global_data.hypothesis[idx[n - 1 - i]].probability, r_global_data.hypothesis[idx[n - 1 - i]].observer.uid);
        if ((r_global_data.hypothesis[idx[n-1-i]].probability >= pfilter) &&
            (r_global_data.hypothesis[idx[n-1-i]].probability > psmall) &&
            (j < params.nkeep())) {
            memcpy(&(r_global_data.hypothesis2[j]), &(r_global_data.hypothesis[idx[n-1-i]]), sizeof(hypothesis_t));
            j++;
        }
    }

    if (j == 0) {
        MRA_LOG_DEBUG("BM_ERROR_NO_HYP: apply filter action to the system, but j == 0");
        return BM_ERROR_NO_HYP;
    }

    /* create list of at most MAXBEST best balls */
    unsigned i = 0;
    pmax = r_global_data.hypothesis[idx[n-1-i]].probability;
    uid_clear(buid);
    while ((uid_get_n(buid) < MAXBEST) && (r_global_data.hypothesis[idx[n-1-i]].probability >= ALPHA * pmax) && (i < n)) {
        uid_add(r_global_data.hypothesis[idx[n-1-i]].observer.uid, r_global_data.hypothesis[idx[n-1-i]].probability, buid);
        i++;
        if (n - 1 - i < 0) {
            break;
        }
    }

    uid_print(buid);


    MRA_LOG_DEBUG("track_uid for = %d", r_global_data.track_uid);
    /* check inext */
    if (inext && (!r_global_data.next_done)) {
        /* next best ball selected by user */
        iret = uid_get_id(r_global_data.track_uid, buid);

        MRA_LOG_DEBUG("current track_uid = %d at id = %d", r_global_data.track_uid, iret);
        uid_exists = iret != UID_ERROR;
        if (uid_exists) {
            /* track_uid still belongs to best balls, take next best ball */

            MRA_LOG_DEBUG("track_uid still belongs to best balls, take next best ball.");
            k = iret + 1;
            if (k > uid_get_n(buid) - 1) {
                k = 0;
            }

            MRA_LOG_DEBUG("new id = %d", k);
            /* set track_uid to new ball uid */
            iret = uid_get_uid(k, buid);

            MRA_LOG_DEBUG("Take next ball with id=%d and uid=%d", k, iret);
            if (iret != UID_ERROR) {
                /* valid uid found */
                r_global_data.track_uid = iret;
                r_global_data.next_done = 1;
            }
            else {
                MRA_LOG_DEBUG("BM_ERROR_NO_HYP: no valid uid found");
                return BM_ERROR_NO_HYP;
            }
        }
        else {
            /* current track_uid not valid anymore, set track_uid to best ball uid */
            MRA_LOG_DEBUG("current track_uid not valid anymore, set track_uid to best ball uid.");
            iret = sequence_clustering_set_track_uid_to_best(buid, r_global_data);
            if (iret != UID_SUCCESS) {
                return iret;
            }
        }
    } else {
        uid_exists = uid_get_id(r_global_data.track_uid, buid) != UID_ERROR;
        if (uid_exists) {
            /* track_uid still belongs to best balls, keep current track_uid */

            MRA_LOG_DEBUG("track_uid still belongs to best balls, keep current track_uid.");
        }
        else {
            /* current track_uid not valid anymore, set track_uid to best ball uid */
            MRA_LOG_DEBUG("current track_uid not valid anymore, set track_uid to best ball uid.");
            iret = sequence_clustering_set_track_uid_to_best(buid, r_global_data);
            if (iret != UID_SUCCESS) {
                return iret;
            }
        }
    }


    MRA_LOG_DEBUG("track_uid na = %d, probability = %f", r_global_data.track_uid, buid.probability[uid_get_id(r_global_data.track_uid, buid)]);

    MRA_LOG_DEBUG("new number of hypotheses = %d", j);
    r_global_data.number_of_hypothesis = j; /* new number of active hypotheses */

    memcpy(r_global_data.hypothesis, r_global_data.hypothesis2, r_global_data.number_of_hypothesis * sizeof(hypothesis_t)); /* copy back new generation */

    return BM_SUCCESS;
}

static int mape(sc_global_data_t& r_global_data) {
    /* get Maximum A Posteriori estimate from hypotheses with uid=track_uid */
    unsigned i, i_mape = 0;

    /* find first hypothesis with uid=track_uid */
    for (i = 0; i < r_global_data.number_of_hypothesis; i++) {
        if (r_global_data.hypothesis[i].observer.uid == r_global_data.track_uid) {
            i_mape = i;
            break;
        }
    }

    if (i_mape < (r_global_data.number_of_hypothesis - 1)) {
        /* check if there exist better hypotheses with uid=track_uid */
        for (i = i_mape + 1; i < r_global_data.number_of_hypothesis; i++) {
            if ((r_global_data.hypothesis[i].observer.uid == r_global_data.track_uid) && (r_global_data.hypothesis[i].probability > r_global_data.hypothesis[i_mape].probability)) {
                i_mape = i;
            }
        }
    }
    return i_mape;
}

int sequence_clustering_initialize(sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params) {
    /* initial number of hypotheses */
    r_global_data.number_of_hypothesis = 1;

    /* initialize hypotheses */
    init_hyp(r_global_data.hypothesis);
    init_hyp(r_global_data.hypothesis2);

    r_global_data.new_uid = 0;

    r_global_data.track_uid = INVALID_UID;

    return BM_SUCCESS;
}


int sequence_clustering_track_ball(ball_estimate_t& r_ball_estimates, const std::vector<ball_candidate_t>& pbfeat, double time, unsigned inext,
        sc_global_data_t& r_global_data, MRA::RobotsportsLocalBallTracking::Params const &params, const unsigned max_num_balls)
{

    /*
     inputs:  pbfeat    - array of ball features (x, y, z, conf, dist, type, isFree, inAir, timestamp), padded with zeros
     time      - current nominal time
     inext     - if 1, then go to next best ball
     r_global_data     - pointer to global workspace

     outputs: const ball_estimate_t& ball_estimates     - ball estimate (x, y, z, xdot, ydot, zdot, hconf, isUpd, label, timestamp), x, y are last feature
     pball2    - ball estimate, x,y are estimates
     */
    unsigned n, i, j, i_mape, iret, idx;
    double vel, alpha;

    int idx_t[max_num_balls], valid[max_num_balls];
    double ts[max_num_balls];

    r_global_data.next_done = 0;

    //double t1 = get_time();

    /* find number of valid features */
    n = 0;
    for (i = 0; i < max_num_balls; i++) {

        MRA_LOG_DEBUG(" feature %d: x = %f, y = %f, z = %f, conf = %f", i, pbfeat[i].x, pbfeat[i].y, pbfeat[i].z, pbfeat[i].confidence);
        if (pbfeat[i].confidence > 0.0) { /* valid features have positive confidence value */
            valid[n] = i; /* remember valid features */
            ts[n] = pbfeat[i].timestamp; /* store valid timestamps */
            n++;
        }
    }

    MRA_LOG_DEBUG("sc_bm reports: %d valid features found at time %f.", n, time);
    for (i = 0; i < n; i++) {
        MRA_LOG_DEBUG("valid[%d] = %d, ts[%d] = %f", i, valid[i], i, ts[i]);
    }
    /* reset updated_in_timestep */
    for (j = 0; j < MAXHYP; j++) {
        r_global_data.hypothesis[j].updated_in_timestep = 0;
    }

    if (n > 0) {
        /* sort w.r.t. ascending timestamps */
        isort_ascending(idx_t, ts, n);

        for (i = 0; i < n; i++) {
            MRA_LOG_DEBUG("%d", idx_t[i]);
        }


        MRA_LOG_DEBUG("sc_bm reports:");
        MRA_LOG_DEBUG("   time = %f s", time);
        MRA_LOG_DEBUG("   %d measurements at times:", n);
        for (i = 0; i < n; i++) {
            MRA_LOG_DEBUG("      %f x = %f, y = %f, z = %f, conf = %f", pbfeat[valid[idx_t[i]]].timestamp,
                    pbfeat[valid[idx_t[i]]].x, pbfeat[valid[idx_t[i]]].y, pbfeat[valid[idx_t[i]]].z,
                    pbfeat[valid[idx_t[i]]].confidence);
        }

        for (i = 0; i < n; i++) {
            /* propagate set of hypotheses */
            iret = generate_offspring(pbfeat[valid[idx_t[i]]], r_global_data, params);
            MRA_LOG_DEBUG("iret generate_offspring = %d", iret);

            if (iret >= 0) {
                MRA_LOG_DEBUG("Processing feature #%d:", i);
                iret = seq_clustering_print_hypotheses(r_global_data);
                MRA_LOG_DEBUG("iret after seq_clustering_print_hypotheses = %d", iret);
            }

            /* apply observer update: prediction + measurement if associated */
            if (iret >= 0) {
                iret = observer_update(pbfeat[valid[idx_t[i]]], r_global_data, params);
                MRA_LOG_DEBUG("iret after observer_update = %d", iret);
            }
            if (iret >= 0) {
                /* apply likelihood update */
                iret = likelihood_update(pbfeat[valid[idx_t[i]]], r_global_data);
                MRA_LOG_DEBUG("iret after likelihood_update = %d", iret);
            }
            if (iret >= 0) {
                /* normalize probability distribution */
                iret = normalization(r_global_data);
                MRA_LOG_DEBUG("iret after normalization = %d", iret);
            }
#ifndef NOCLIPCONF
            if (iret >= 0) {
                iret = clip_conf(r_global_data, params);
                MRA_LOG_DEBUG("iret after clip_conf = %d", iret);
            }
#endif


            if (iret >= 0) {
                iret = seq_clustering_print_hypotheses(r_global_data);
                MRA_LOG_DEBUG("iret after seq_clustering_print_hypotheses = %d", iret);
            }
            if (iret >= 0) {
                /* gating of discrete probability distribution */
                iret = sequence_clustering_nhyp_controller(inext, r_global_data, params);
                MRA_LOG_DEBUG("iret after sequence_clustering_nhyp_controller = %d", iret);
            }

            if (iret < 0) {
                MRA_LOG_DEBUG("seq_clustering_ball_model result = %d (= %s)", iret, BM_result_to_string(iret).c_str());
                return iret;
            }

            /* mark updated hypotheses as updated_in_timestep */
            for (j = 0; j < r_global_data.number_of_hypothesis; j++) {
                if ((r_global_data.hypothesis[j].association_flag == ASSOCIATE_WITH_BALL)
                        || (r_global_data.hypothesis[j].association_flag == ASSOCIATE_WITH_NEW)) {
                    r_global_data.hypothesis[j].updated_in_timestep++;
                }
            }
        }
    } else {
    }

    /* Maximum A Posteriori (MAP) estimate */
    i_mape = mape(r_global_data);
    MRA_LOG_DEBUG("probability of winner = %f", r_global_data.hypothesis[i_mape].probability);


    MRA_LOG_DEBUG("Winning hypothesis at t = %f: i_mape = %d, uid = %d, probability = %f", time, i_mape,
            r_global_data.hypothesis[i_mape].observer.uid, r_global_data.hypothesis[i_mape].probability);
    if (r_global_data.hypothesis[i_mape].ball_detected) {
        MRA_LOG_DEBUG("-> Ball position: ");
        MRA_LOG_DEBUG("%f %f %f", r_global_data.hypothesis[i_mape].observer.xh[0], r_global_data.hypothesis[i_mape].observer.xh[2],
                r_global_data.hypothesis[i_mape].observer.xh[4]);
    }
    else {
        MRA_LOG_DEBUG("-> No ball.");
    }


    /* return result in ball_estimate */
    if (r_global_data.hypothesis[i_mape].fbuf_idx == 0) {
        idx = params.max_features_buffer() - 1;
    }
    else {
        idx = r_global_data.hypothesis[i_mape].fbuf_idx - 1;
    }

    r_ball_estimates.x = r_global_data.hypothesis[i_mape].fbuf.x[idx]; /* the most recent feature */
    r_ball_estimates.y = r_global_data.hypothesis[i_mape].fbuf.y[idx];
    r_ball_estimates.xhat = r_global_data.hypothesis[i_mape].observer.xh[0];
    r_ball_estimates.yhat = r_global_data.hypothesis[i_mape].observer.xh[2];

    r_ball_estimates.z = r_global_data.hypothesis[i_mape].observer.xh[4];
    r_ball_estimates.xdot = r_global_data.hypothesis[i_mape].observer.xh[1];
    r_ball_estimates.ydot = r_global_data.hypothesis[i_mape].observer.xh[3];
    r_ball_estimates.zdot = r_global_data.hypothesis[i_mape].observer.xh[5];

    /* apply ball velocity clipping */
    vel = hypot(r_ball_estimates.xdot, r_ball_estimates.ydot);

    MRA_LOG_DEBUG("xdot = %f, ydot = %f, vel = %f", r_ball_estimates.xdot, r_ball_estimates.ydot, vel);
    if (vel < params.velocity_ball_clip_lower()) {
        r_ball_estimates.xdot = 0.0; /* set velocity to zero */
        r_ball_estimates.ydot = 0.0;
    }
    if (vel > params.velocity_ball_clip_upper()) {
        alpha = params.velocity_ball_clip_upper() / vel; /* scale velocity to match maximum */
        r_ball_estimates.xdot *= alpha;
        r_ball_estimates.ydot *= alpha;
    }

    if (r_global_data.hypothesis[i_mape].ball_detected) {
        r_ball_estimates.hconf = r_global_data.hypothesis[i_mape].mavg;
    } else {
        r_ball_estimates.hconf = 0.0;
    }

    r_ball_estimates.is_updated = r_global_data.hypothesis[i_mape].updated_in_timestep > 0;
    MRA_LOG_DEBUG("Winning hypothesis updated_in_timestep = %d", r_ball_estimates.is_updated);

    r_ball_estimates.label = r_global_data.hypothesis[i_mape].observer.label;
    r_ball_estimates.timestamp = r_global_data.hypothesis[i_mape].observer.time;


    return BM_SUCCESS;
}
