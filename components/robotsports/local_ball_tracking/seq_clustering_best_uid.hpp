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
} best_uid;


int uid_clear(best_uid* puid);
int uid_add(int uid, double p, best_uid* puid);
int uid_get_id(int uid, best_uid* puid);
int uid_get_uid(int id, best_uid* puid);
int uid_get_n(best_uid* puid);
int uid_print(best_uid* puid);

#endif

