/**
 *  @file
 *  @brief   ball unique identifier
 *  @curator Jurge van Eijck
 */

#ifndef BEST_UID_HPP
#define BEST_UID_H 1

const int UID_SUCCESS = 0;
const int UID_ERROR = -1;

const int MAXBEST = 1;
const int INVALID_UID = -1;


typedef struct tag_best_uid {
        int ball_uid[MAXBEST];  /* list of best ball uid's */
        double probability[MAXBEST];      /* probability of best ball uid's */
        int nbest_uid;          /* number of best uid's */
} best_uid_t;


int uid_clear(best_uid_t& best_uid);
int uid_add(int uid, double probability, best_uid_t& best_uid);
int uid_get_id(int uid, const best_uid_t& best_uid);
int uid_get_uid(int id, const best_uid_t& best_uid);
int uid_get_n(const best_uid_t& best_uid);
int uid_print(const best_uid_t& best_uid);

#endif

