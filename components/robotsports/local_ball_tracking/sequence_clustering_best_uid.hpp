/**
 *  @file
 *  @brief   ball unique identifier
 *  @curator Jurge van Eijck
 */

#ifndef BEST_UID_HPP
#define BEST_UID_H 1

#define UID_SUCCESS      0
#define UID_ERROR       -1

#define MAXBEST          1
#define INVALID_UID     -1


typedef struct tag_best_uid {
        int ball_uid[MAXBEST];  /* list of best ball uid's */
        double p[MAXBEST];      /* probability of best ball uid's */
        int nbest_uid;          /* number of best uid's */
} best_uid_t;


int uid_clear(best_uid_t& best_uid);
int uid_add(int uid, double p, best_uid_t& best_uid);
int uid_get_id(int uid, const best_uid_t& best_uid);
int uid_get_uid(int id, const best_uid_t& best_uid);
int uid_get_n(const best_uid_t& best_uid);
int uid_print(const best_uid_t& best_uid);

#endif

