/**
 *  @file
 *  @brief   sequential clustering header
 *  @curator Ton Peijnenburg
 */

#ifndef SEQ_CLUSTERING_BALL_MODEL_INCLUDE
#define SEQ_CLUSTERING_BALL_MODEL_INCLUDE

#include "constants_ball_model.hpp"
#include "RobotsportsLocalBallTracking.hpp"
#include "sequence_clustering_balldef.hpp"

typedef struct tag_ball_observer {
	double        xh[6]; /* state estimate: x, xdot, y, ydot, z, zdot */
	double        time_last_update; /* time of last update */
	balltype_e    label; /* type of last added feature */
	ball_update_e ball_update; /* update flag, 0 = no update required, 1 = associate object with new feature */
	double        time; /* time of current state estimate */
	int           uid; /* uid set at creation */
} ball_observer;

typedef struct tag_featbuf_t {
	double timestamp[MAXFEATBUF];
	double x[MAXFEATBUF];
	double y[MAXFEATBUF];
	double z[MAXFEATBUF];
	double sigma[MAXFEATBUF];
	double exp_time[MAXFEATBUF];
	bool isFree[MAXFEATBUF];
} featbuf_t;

typedef struct tag_hypothesis {
	ball_observer observer; /* observer representing the detected ball according to this hypothesis */
	bool ball_detected;
	double probability; /* probability of this hypothesis */
	associate_e association_flag; /* association of last feature (ASSOCIATE_WITH_CLUTTER or ASSOCIATE_WITH_BALL) */
	double ma_buf[MA_N + 1]; /* MA confidence filter */
	double mavg;
	int ma_idx;
	int ma_first;
	unsigned updated_in_timestep;
	featbuf_t fbuf;
	unsigned fbuf_idx;
	unsigned number_valid_buffers; /* number of valid features in buffer, start at 0 */
} hypothesis_t;

/* global data structure */
typedef struct tag_sc_global_data {
//	std::vector<>hypothesis> hyp;
//	std::vector<hypothesis> hyp2;
    hypothesis_t hypothesis[MAXHYP];
    hypothesis_t hypothesis2[MAXHYP];
	unsigned number_of_hypothesis;
	int new_uid;
	int track_uid; /* ball uid to keep track off */
	bool next_done;
	bool is_kicked;  // not in use, future usage
	double vx0; /* initial ball speed on kick */
	double vy0;
} sc_global_data;

// the following are external functions
int sequence_clustering_ball_model(ball_estimate_t& r_ball_estimates, const std::vector<ball_candidate_t>& pbfeat, double time, unsigned inext, sc_global_data& r_global_data, MRA::RobotsportsLocalBallTracking::ParamsType const &params,
        const unsigned max_num_balls);

#endif  // SEQ_CLUSTERING_BALL_MODEL_INCLUDE
