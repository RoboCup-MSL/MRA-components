/**
 *  @file
 *  @brief   log functions, not used
 *  @curator Jurge van Eijck
 */

#ifndef BM_LOG_H
#define BM_LOG_H

#include "seq_clustering_ball_model.hpp"

int log_init(phypothesis phyp);
int log_write_t(double t, phypothesis phyp);
int log_write_association_flag(double association_flag, phypothesis phyp);
int log_write_x(double* x, phypothesis phyp);
int log_write_bfeat(ball_feature_t* pbfeat, phypothesis phyp);
int log_write_p_prior(double p_prior, phypothesis phyp);
int log_write_p_prediction(double p_prediction, phypothesis phyp);
int log_write_p_likelihood(double p_likelihood, phypothesis phyp);
int log_write_p_posterior(double p_posterior, phypothesis phyp);
int log_update(phypothesis phyp);
int logfile_open(FILE* fp);
int logfile_close(FILE* fp);

#endif
