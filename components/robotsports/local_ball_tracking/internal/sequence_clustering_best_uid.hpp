/**
 *  @file
 *  @brief   ball unique identifier
 *  @curator Jurge van Eijck
 */

#ifndef BEST_UID_HPP
#define BEST_UID_H 1

typedef enum uid_result_e {
    UID_ERROR = -1,
    UID_SUCCESS = 0,
} uid_result_e;


const int MAXBEST = 1;
const int INVALID_UID = -1;


typedef struct tag_best_uid {
        int ball_uid[MAXBEST];       /* list of best ball uid's */
        double probability[MAXBEST]; /* probability of best ball uid's */
        int nbest_uid;               /* number of best uid's */
} best_uid_t;


uid_result_e uid_clear(best_uid_t& best_uid);
uid_result_e uid_add(int uid, double probability, best_uid_t& best_uid);
int uid_get_id(int uid, const best_uid_t& best_uid);
int uid_get_uid(int id, const best_uid_t& best_uid);
int  uid_get_n(const best_uid_t& best_uid);
uid_result_e uid_print(const best_uid_t& best_uid);

#endif

