/**
 *  @file
 *  @brief   log functions, not used
 *  @curator Rene van de Molengraft
 */

#include <cstring> // memcpy
#include "seq_clustering_ball_model_log.hpp"
#include "constants_ball_model.hpp"


// it was static

int log_init(hypothesis *phyp) {
	phyp->hist_idx = 0;
	phyp->hist_full = 0;

	return 0;
}

int log_write_t(double t, hypothesis *phyp) {
	phyp->hist[phyp->hist_idx].t = t;

	return 0;
}

int log_write_association_flag(double association_flag,
		hypothesis *phyp) {
	phyp->hist[phyp->hist_idx].association_flag = association_flag;

	return 0;
}

int log_write_x(double *x, hypothesis *phyp) {
	memcpy(&(phyp->hist[phyp->hist_idx].x[0]), x, 6 * sizeof(double));

	return 0;
}

int log_write_bfeat(ball_feature_t *pbfeat, hypothesis *phyp) {
	memcpy(&(phyp->hist[phyp->hist_idx].bfeat), pbfeat, sizeof(ball_feature_t));

	return 0;
}

int log_write_p_prior(double p_prior, hypothesis *phyp) {
	phyp->hist[phyp->hist_idx].p_prior = p_prior;

	return 0;
}

int log_write_p_prediction(double p_prediction, hypothesis *phyp) {
	phyp->hist[phyp->hist_idx].p_prediction = p_prediction;
	return 0;
}

int log_write_p_likelihood(double p_likelihood, hypothesis *phyp) {
	phyp->hist[phyp->hist_idx].p_likelihood = p_likelihood;
	return 0;
}

int log_write_p_posterior(double p_posterior, hypothesis *phyp) {
	phyp->hist[phyp->hist_idx].p_posterior = p_posterior;
	return 0;
}

int log_update(hypothesis *phyp) {
	phyp->hist_idx++;
	if (phyp->hist_idx >= MAXHIST) {
		phyp->hist_idx = 0;
		phyp->hist_full = 1;
	}
	return 0;
}

int logfile_open(FILE *fp) {
	fp = fopen("bmlog.log", "w");
	return 0;
}

int logfile_close(FILE *fp) {
	fclose(fp);
	return 0;
}

