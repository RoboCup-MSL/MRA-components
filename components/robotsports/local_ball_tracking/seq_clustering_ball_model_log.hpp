/**
 *  @file
 *  @brief   log functions, not used
 *  @curator Jurge van Eijck
 */

#ifndef BM_LOG_H
#define BM_LOG_H

#include "seq_clustering_ball_model.hpp"

int log_init(hypothesis *phyp);
int log_write_t(double t, hypothesis *phyp);
int log_write_association_flag(double association_flag, hypothesis *phyp);
int log_write_x(double* x, hypothesis* phyp);
int log_write_bfeat(ball_feature_t* pbfeat, hypothesis *phyp);
int log_write_p_prior(double p_prior, hypothesis *phyp);
int log_write_p_prediction(double p_prediction, hypothesis *phyp);
int log_write_p_likelihood(double p_likelihood, hypothesis *phyp);
int log_write_p_posterior(double p_posterior, hypothesis *phyp);
int log_update(hypothesis* phyp);
int logfile_open(FILE* fp);
int logfile_close(FILE* fp);

#endif
