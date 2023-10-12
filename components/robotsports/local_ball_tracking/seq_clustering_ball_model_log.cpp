/**
 *  @file
 *  @brief   log functions, not used
 *  @curator Rene van de Molengraft
 */

#include "seq_clustering_ball_model_log.hpp"
#include "constants_ball_model.hpp"

#define M_STATIC 
// it was static

M_STATIC int log_init(phypothesis phyp)
{
        phyp->hist_idx = 0;
        phyp->hist_full = 0;

        return 0;
}





M_STATIC int log_write_t(double t, phypothesis phyp)
{
        phyp->hist[phyp->hist_idx].t = t;

        return 0;
}





M_STATIC int log_write_association_flag(double association_flag, phypothesis phyp)
{
        phyp->hist[phyp->hist_idx].association_flag = association_flag;

        return 0;
}




M_STATIC int log_write_x(double* x, phypothesis phyp)
{
        memcpy(&(phyp->hist[phyp->hist_idx].x[0]), x, 6 * sizeof(double) );

        return 0;
}





M_STATIC int log_write_bfeat(ball_feature_t* pbfeat, phypothesis phyp)
{
        memcpy(&(phyp->hist[phyp->hist_idx].bfeat), pbfeat, sizeof(ball_feature_t) );

        return 0;
}





M_STATIC int log_write_p_prior(double p_prior, phypothesis phyp)
{
        phyp->hist[phyp->hist_idx].p_prior = p_prior;

        return 0;
}




M_STATIC int log_write_p_prediction(double p_prediction, phypothesis phyp)
{
        phyp->hist[phyp->hist_idx].p_prediction = p_prediction;

        return 0;
}




M_STATIC int log_write_p_likelihood(double p_likelihood, phypothesis phyp)
{
        phyp->hist[phyp->hist_idx].p_likelihood = p_likelihood;

        return 0;
}




M_STATIC int log_write_p_posterior(double p_posterior, phypothesis phyp)
{
        phyp->hist[phyp->hist_idx].p_posterior = p_posterior;

        return 0;
}





M_STATIC int log_update(phypothesis phyp)
{
        phyp->hist_idx++;
        if (phyp->hist_idx >= MAXHIST) {
                phyp->hist_idx = 0;
                phyp->hist_full = 1;
        }

        return 0;
}





M_STATIC int logfile_open(FILE* fp)
{
        fp = fopen("bmlog.log", "w");

        return 0;
}





M_STATIC int logfile_close(FILE* fp)
{
        fclose(fp);

        return 0;
}

