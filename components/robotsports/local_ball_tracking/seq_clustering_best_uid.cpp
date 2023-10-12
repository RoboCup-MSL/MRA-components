/**
 *  @file
 *  @brief   ball unique identifier
 *  @curator Rene van de Molengraft
 */
#include "seq_clustering_best_uid.hpp"

int uid_clear(best_uid *puid)
{
    int i;

    for (i = 0; i < MAXBEST; i++) {
        puid->ball_uid[i] = INVALID_UID;
        puid->p[i] = 0.;
    }
    puid->nbest_uid = 0;

    return UID_SUCCESS;
}





int uid_add(int uid, double p, best_uid* puid)
{
        int i = 0;
        while ( (i < puid->nbest_uid) && (puid->ball_uid[i] != uid) ) {
                i += 1;
        }
        if (i == puid->nbest_uid) {
                if (puid->nbest_uid < MAXBEST) {
                        puid->ball_uid[puid->nbest_uid] = uid;
                        puid->p[puid->nbest_uid] = p;
                        puid->nbest_uid += 1;
                } else {
                        return UID_ERROR; /* MAXBEST exceeded */
                }
        } else {
                return UID_ERROR; /* uid already exists */
        }
        return UID_SUCCESS;
}





int uid_get_id(int uid, best_uid* puid)
{
        int i = 0;
        while ( (i < puid->nbest_uid) && (puid->ball_uid[i] != uid) ) {
                i += 1;
        }
        if (i < puid->nbest_uid) {
                return i;
        }
        return UID_ERROR; /* uid does not exist */
}





int uid_get_uid(int id, best_uid* puid)
{
        if (id < puid->nbest_uid) {
                return puid->ball_uid[id];
        } else {
                return UID_ERROR; /* id does not contain a valid uid */
        }
}





int uid_get_n(best_uid* puid)
{
        return puid->nbest_uid;
}


int uid_print(best_uid* puid)
{
// TODO
//	logAlways("Total of %d uid's:", puid->nbest_uid);
//
//	for (int i = 0; i < MAXBEST; i += 1) {
//		logAlways("id = %d, uid = %d, p = %f", i, puid->ball_uid[i], puid->p[i]);
//	}
	return UID_SUCCESS;
}




