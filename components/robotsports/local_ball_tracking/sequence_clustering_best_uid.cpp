/**
 *  @file
 *  @brief   ball unique identifier
 *  @curator Rene van de Molengraft
 */
#include "logging.hpp"
#include "sequence_clustering_best_uid.hpp"

int uid_clear(best_uid_t& best_uid) {
    int i;

    for (i = 0; i < MAXBEST; i++) {
        best_uid.ball_uid[i] = INVALID_UID;
        best_uid.probability[i] = 0.0;
    }
    best_uid.nbest_uid = 0;

    return UID_SUCCESS;
}

int uid_add(int uid, double p, best_uid_t& best_uid) {
    int i = 0;

    while ((i < best_uid.nbest_uid) && (best_uid.ball_uid[i] != uid)) {
        i++;
    }
    if (i == best_uid.nbest_uid) {
        if (best_uid.nbest_uid < MAXBEST) {
            best_uid.ball_uid[best_uid.nbest_uid] = uid;
            best_uid.probability[best_uid.nbest_uid] = p;
            best_uid.nbest_uid += 1;
        } else {
            return UID_ERROR; /* MAXBEST exceeded */
        }
    } else {
        return UID_ERROR; /* uid already exists */
    }
    return UID_SUCCESS;
}

int uid_get_id(int uid, const best_uid_t& best_uid) {
    int i = 0;
    while ((i < best_uid.nbest_uid) && (best_uid.ball_uid[i] != uid)) {
        i++;
    }
    if (i < best_uid.nbest_uid) {
        return i;
    }
    return UID_ERROR; /* uid does not exist */
}

int uid_get_uid(int id, const best_uid_t& best_uid) {
    if (id < best_uid.nbest_uid) {
        return best_uid.ball_uid[id];
    } else {
        return UID_ERROR; /* id does not contain a valid uid */
    }
}

int uid_get_n(const best_uid_t& best_uid) {
    return best_uid.nbest_uid;
}

int uid_print(const best_uid_t& best_uid) {
    MRA_LOG_DEBUG("Total of %d uid's:", best_uid.nbest_uid);

    for (int i = 0; i < MAXBEST; i += 1) {
        MRA_LOG_DEBUG("id = %d, uid = %d, probability = %f", i, best_uid.ball_uid[i], best_uid.probability[i]);
    }
    return UID_SUCCESS;
}

