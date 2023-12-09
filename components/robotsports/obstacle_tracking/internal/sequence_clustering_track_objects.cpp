
/* sequential clustering algorithm, Rene van de Molengraft, March, 2008 */

/*
 
  Inspired by Schubert & Sidenblath, Sequential clustering with particle filters - Estimating the number of clusters from data,
  Proc. 8th Int. Conf. Information Fusion, 2005
 
  Instead of using a particle filter, we apply a discrete filter to the set of hypotheses, which is of itself already a discrete stochast.
 
  As the objects may be moving, we use a constant-velocity model-based Kalman filter to estimate the states of the detected objects.
 
  Last updated: March, 19th, 2008, added bound checking and error handling
        February, 13th, 2009, adapted for sorted measurements with varying time interval in between (kalman_update)
                                      added object labeling on creation of new object
 *
 * NOTE: obstacles POSITIONS are directly fed through!! Velocities are estimated not positions!!
 */
 
#include "sequence_clustering_track_objects.hpp"

#include "logging.hpp"

//TODO
#define FIELDMARGIN     0.6                 /* margin-outside-field for object clipping */
#define FIELDWIDTH      12.0
#define FIELDLENGTH     20.0




static void swap_index(unsigned idx_1, unsigned idx_2, unsigned *idx) {
    unsigned idx_bak;

    idx_bak = idx[idx_1];
    idx[idx_1] = idx[idx_2];
    idx[idx_2] = idx_bak;
}


static void isort_ascending(unsigned *idx, double *iarr, unsigned size) {
    for (unsigned i = 0; i < size; i++) {
        idx[i] = i;
    }

    for (unsigned i = 0; i < size; i++) {
        for (unsigned j = i; j < size; j++) {
            if (iarr[idx[j]] < iarr[idx[i]]) {
                swap_index(i, j, idx);
            }
        }
    }
}



//static SequenceClusterCode_e print_hypothesis_w(hypothesis_w* phyp, int i, scw_global_data& pscgd)
//{
///*  print hypothesis to screen */
//
//    int j, idx;
//
//    MRA_LOG_DEBUG("Hypothesis: %d", i);
//    MRA_LOG_DEBUG("   Number of objects = %d", phyp->number_of_objects);
//    if ( phyp->number_of_objects>0 ) {
//        for (j=0; j<pscgd.hyp[i].number_of_objects; j++) {
//        	idx=phyp->filter_id[j];
//        	MRA_LOG_DEBUG("   Object %d at (%f, %f)", j, pscgd.kal[idx].xh[0], pscgd.kal[idx].xh[2]);
//        	MRA_LOG_DEBUG("   Label = %d", pscgd.kal[idx].label);
//        }
//    }
//    MRA_LOG_DEBUG("   Probability = %f", phyp->probalility);
//
//    return SC_SUCCESS;
//}
//
//
//static SequenceClusterCode_e print_hypotheses_w(scw_global_data& pscgd)
//{
///* print overview to screen */
//
//    int i;
//
//    for (i=0; i<pscgd.number_hypotheses; i++) {
//        print_hypothesis_w(&(pscgd.hyp[i]), i, pscgd);
//    }
//
//    return SC_SUCCESS;
//}


static SequenceClusterCode_e free_unused_filters(scw_global_data &pscgd) {
    /* free unused filters!!! */

    unsigned i, filters_to_free, k, idx, label_in_use[MAXFIL];

    /* clear all Kalman filter's activity bit */
    for (i = 0; i < MAXFIL; i++) {
        pscgd.kal[i].active = 0;
    }

    /* check which Kalman filters are still being used */
    for (i = 0; i < pscgd.number_hypotheses; i++) {
        for (k = 0; k < pscgd.hyp[i].number_of_objects; k++) {
            idx = pscgd.hyp[i].filter_id[k];
            //MRA_LOG_DEBUG("Kalman filter %d is still in use.", idx);
            pscgd.kal[idx].active = 1;
        }
    }

    /* build free filter index */
    filters_to_free = 0;
    for (i = 0; i < MAXFIL; i++) {
        if (pscgd.kal[i].active == 0) {
            pscgd.kal[i].time_birth = 0.0;
            pscgd.kal[i].assoc_ptr = 0;
            for (k = 0; k < ASSOC_BUFFER_LENGTH; k++) {
                pscgd.kal[i].associations[k] = 0;
            }
            pscgd.ff.index[filters_to_free] = i;
            filters_to_free++;
        }
    }

    /*  number of free filters */
    pscgd.ff.nfree = filters_to_free;
    //  MRA_LOG_DEBUG("Just marked %d free filters", j);

    /* reset free filter start index */
    pscgd.ff.ifree = 0;

    /* build activity array for opponent labels */
    for (i = 0; i < MAXFIL; i++) {
        label_in_use[i] = 0;
    }
    for (i = 0; i < MAXFIL; i++) {
        if (pscgd.kal[i].active == 1) {
            if (pscgd.kal[i].label >= LABEL_OFFSET) {
                /* this is a labeled opponent object */
                label_in_use[pscgd.kal[i].label - LABEL_OFFSET] = 1;
            }
        }
    }

    /* build free label index */
    filters_to_free = 0;
    for (i = 0; i < MAXFIL; i++) {
        if (label_in_use[i] == 0) {
            pscgd.fl.index[filters_to_free] = i + LABEL_OFFSET;
            filters_to_free++;
        }
    }

    /* number of free labels */
    pscgd.fl.nfree = filters_to_free;
    // MRA_LOG_DEBUG("Just marked %d free labels", j);

    /* reset free label start index */
    pscgd.fl.ifree = 0;

    return SC_SUCCESS;
}





static int get_free_filter(scw_global_data&  pscgd)
{
    int filter_id;
    
    filter_id=pscgd.ff.index[pscgd.ff.ifree];
    pscgd.ff.ifree++;
    if ( pscgd.ff.ifree>pscgd.ff.nfree ) {
        MRA_LOG_DEBUG("Out of Kalman filters!");
        filter_id = SC_OUT_OF_KALLMAN_FILTERS;
        //jve-TODO: object_process.filter_error = 1;
    }
    
    return filter_id;
}





static int get_free_label(scw_global_data&  pscgd)
{
    int label_id;
    
    label_id=pscgd.fl.index[pscgd.fl.ifree];
    pscgd.fl.ifree++;
    if ( pscgd.fl.ifree>pscgd.fl.nfree ) {
        MRA_LOG_DEBUG("Out of labels with ifree %d and nfree %d!", pscgd.fl.ifree, pscgd.fl.nfree);
        label_id = NO_FREE_LABEL_ID;
    }
    
    return label_id;
}


static int find_associating_object(double *z, scw_global_data& pscgd)
{
/*  search for associating existing object for measurement z */
    
    int imax = -1;
    double tttx, ttty, probability, max_probability = 0.0;
    
    for (auto i = 0;  i < MAXFIL; i++) {
        if ( pscgd.kal[i].active ) {
            tttx = z[0] - pscgd.kal[i].xh[0];
            ttty = z[1] - pscgd.kal[i].xh[2];
            probability = exp( -0.5*( (tttx*tttx)/(pscgd.par.sigmax*pscgd.par.sigmax)+(ttty*ttty)/(pscgd.par.sigmay*pscgd.par.sigmay) ) );
            if ( probability > max_probability ) {
                max_probability = probability;
                imax = i;
            }
        }
    }

    if ( max_probability <= pscgd.par.labelbound ) {
        MRA_LOG_DEBUG("---> No associating object found.");
        return SC_NO_ASSOCIATING_OBJECT_FOUND;
    }

    MRA_LOG_DEBUG("---> Associating object found with index %d, probability is %f", imax, max_probability);
    return imax;
}


static SequenceClusterCode_e associate_with_existing_object(int i, int j, unsigned iobj, double *z, scw_global_data &pscgd) {
    unsigned filter_id, k, m, nobj, idx;
    int free_id;
    double pnew;

    if (j >= MAXHYP_W) {
        return SC_MAX_HYPOTHESES_REACHED;
    }

    memcpy(&(pscgd.hyp2[j]), &(pscgd.hyp[i]), sizeof(hypothesis_w)); /* copy hypothesis */

    /*  probability of this hypothesis */
    pnew = pscgd.par.alpha * (1 - pscgd.par.pclutter) / (pscgd.hyp2[j].number_of_objects + pscgd.par.alpha);
    if (pscgd.hyp2[j].number_of_objects == 0) {
        return SC_ERROR_ASSOCIATE_WITH_EXISTING_OBJECT_NO_OBJECT;
    }
    pscgd.hyp2[j].probalility = pscgd.hyp2[j].probalility * (1 - pscgd.par.pclutter - pnew)
            / pscgd.hyp2[j].number_of_objects;

    /*  all objects of old hypothesis must get a new filter */
    for (k = 0; k < pscgd.hyp[i].number_of_objects; k++) {
        /* give object a new filter id */
        free_id = get_free_filter(pscgd);
        if (free_id == SC_OUT_OF_KALLMAN_FILTERS) {
            return SC_ERROR_ASSOCIATE_WITH_EXISTING_OBJECT_FREE_FILTER;
        }
        pscgd.hyp2[j].filter_id[k] = free_id;

        /* copy old filter state to new one and set active */
        memcpy(pscgd.kal[free_id].xh, pscgd.kal[pscgd.hyp[i].filter_id[k]].xh, 4 * sizeof(double));
        pscgd.kal[free_id].time_last_update = pscgd.kal[pscgd.hyp[i].filter_id[k]].time_last_update;
        pscgd.kal[free_id].label = pscgd.kal[pscgd.hyp[i].filter_id[k]].label;
        pscgd.kal[free_id].time_birth = pscgd.kal[pscgd.hyp[i].filter_id[k]].time_birth;
        pscgd.kal[free_id].assoc_ptr = pscgd.kal[pscgd.hyp[i].filter_id[k]].assoc_ptr;
        memcpy(pscgd.kal[free_id].associations, pscgd.kal[pscgd.hyp[i].filter_id[k]].associations,
                ASSOC_BUFFER_LENGTH * sizeof(int));
        pscgd.kal[free_id].active = 1;
    }

    /*  associate measurement with filter of object iobj */
    pscgd.hyp2[j].association_id = pscgd.hyp2[j].filter_id[iobj];

    pscgd.hyp2[j].hypothesis_id = j; /* label hypothesis */

    filter_id = pscgd.hyp2[j].filter_id[iobj];
    if (z[3] > 0) {
        /* inherit measurement label for known object */
        pscgd.kal[filter_id].label = z[3];
        //MRA_LOG_DEBUG("---> Label for object %d inherited from measurement, label is %d.", filter_id, pscgd.kal[filter_id].label);
        /* number of objects in this hypothesis */
        nobj = pscgd.hyp2[j].number_of_objects;
        /* check if any other object in this hypothesis has the same turtle's label, if so, change it to a free opponent label! */
        for (m = 0; m < nobj; m++) {
            if (m != iobj) {
                idx = pscgd.hyp2[j].filter_id[m];
                if (pscgd.kal[idx].label == (int) z[3]) {
                    //MRA_LOG_DEBUG("---> hyp %d: EXIST OBJECT duplicate label found (turtle %d)!", j, (int) z[3]);
                    pscgd.kal[idx].label = get_free_label(pscgd);
                    if (pscgd.kal[idx].label == NO_FREE_LABEL_ID) {
                        return SC_ERROR_ASSOCIATE_WITH_EXISTING_OBJECT_NO_FREE_LABEL;
                    }

                    //MRA_LOG_DEBUG(" ... changed duplicate label to o%d", pscgd.kal[idx].label);
                }
            }
        }
    }

    return SC_SUCCESS;
}


static SequenceClusterCode_e associate_with_clutter(int i, int j, scw_global_data& pscgd)
{
    if (j >= MAXHYP_W) {
        return SC_MAX_HYPOTHESES_REACHED;
    }

    memcpy(&(pscgd.hyp2[j]), &(pscgd.hyp[i]), sizeof(hypothesis_w)); /* copy hypothesis */

    pscgd.hyp2[j].probalility = pscgd.hyp2[j].probalility * pscgd.par.pclutter;
    pscgd.hyp2[j].association_id = -1; /* not associated with any object */

    pscgd.hyp2[j].hypothesis_id = j; /* label hypothesis */

    return SC_SUCCESS;
}


static SequenceClusterCode_e associate_with_new_object(int i, int j, double *z, scw_global_data &pscgd) {
    unsigned k, m, nobj, idx;
    unsigned filter1_assoc_ptr, filter2_assoc_ptr;
    double pnew;
    int free_id,  filter_id, assoc_id;

    if (j >= MAXHYP_W) {
        return SC_TOO_MANY_HYPOTESES;
    }

    if (pscgd.hyp[i].number_of_objects == pscgd.par.maxnobj) {
        return SC_MAX_REACHED;
    }

    memcpy(&(pscgd.hyp2[j]), &(pscgd.hyp[i]), sizeof(hypothesis_w)); /* copy hypothesis */

    pscgd.hyp2[j].number_of_objects++; /* new object */

    /* probability of this hypothesis */
    pnew = pscgd.par.alpha * (1 - pscgd.par.pclutter) / (pscgd.hyp[i].number_of_objects + pscgd.par.alpha);
    pscgd.hyp2[j].probalility = pscgd.hyp2[j].probalility * pnew;

    /* all objects of old hypothesis must get a new filter */
    for (k = 0; k < pscgd.hyp[i].number_of_objects; k++) {
        /* give object a new filter id */
        free_id = get_free_filter(pscgd);
        if ((free_id == SC_OUT_OF_KALLMAN_FILTERS) || (k >= MAXNOBJ_GLOBAL)) {
            return SC_ERROR_ASSOCIATE_WITH_NEW_OBJECT;
        }
        pscgd.hyp2[j].filter_id[k] = free_id;

        /*      copy old filter state to new one and set active */
        memcpy(pscgd.kal[free_id].xh, pscgd.kal[pscgd.hyp[i].filter_id[k]].xh, 4 * sizeof(double));
        pscgd.kal[free_id].time_last_update = pscgd.kal[pscgd.hyp[i].filter_id[k]].time_last_update;
        pscgd.kal[free_id].label = pscgd.kal[pscgd.hyp[i].filter_id[k]].label;
        pscgd.kal[free_id].time_birth = pscgd.kal[pscgd.hyp[i].filter_id[k]].time_birth;
        pscgd.kal[free_id].assoc_ptr = pscgd.kal[pscgd.hyp[i].filter_id[k]].assoc_ptr;
        memcpy(pscgd.kal[free_id].associations, pscgd.kal[pscgd.hyp[i].filter_id[k]].associations,
                ASSOC_BUFFER_LENGTH * sizeof(int));
        pscgd.kal[free_id].active = 1;
    }

    /*  create new object */
    filter_id = get_free_filter(pscgd);
    if ((filter_id == SC_OUT_OF_KALLMAN_FILTERS) || ((pscgd.hyp2[j].number_of_objects - 1) >= MAXNOBJ_GLOBAL)) {
        return SC_ERROR_ASSOCIATE_WITH_NEW_OBJECT;
    }
    pscgd.hyp2[j].filter_id[pscgd.hyp2[j].number_of_objects - 1] = filter_id;

    /*  initialize filter initial condition at measurement z (zero-velocity) and activate filter */
    pscgd.kal[filter_id].xh[0] = z[0];
    pscgd.kal[filter_id].xh[2] = z[1];
    //MRA_LOG_DEBUG("new filter and new object with x %f and y %f", z[0], z[1]);

    if (z[3] > 0) {
        /*          inherit measurement label for known object */
        pscgd.kal[filter_id].label = z[3];
        //MRA_LOG_DEBUG("---> Label for object %d inherited from measurement, label is %d.", filter_id, pscgd.kal[filter_id].label);
        /* number of objects in this hypothesis */
        nobj = pscgd.hyp2[j].number_of_objects;
        /* check if any other object in this hypothesis has the same turtle's label, if so, change it to a free opponent label! */
        for (m = 0; m < (nobj - 1); m++) {
            idx = pscgd.hyp2[j].filter_id[m];
            if (pscgd.kal[idx].label == (int) z[3]) {
                //MRA_LOG_DEBUG("---> hyp %d: NEW OBJECT duplicate label found (turtle %d)!", j, (int) z[3]);
                pscgd.kal[idx].label = get_free_label(pscgd);
                if (pscgd.kal[idx].label == NO_FREE_LABEL_ID) {
                    return SC_ERROR_ASSOCIATE_WITH_NEW_OBJECT_NO_FREE_LABEL;
                }

                //MRA_LOG_DEBUG(" ... changed duplicate label to o%d", pscgd.kal[idx].label);
            }
        }
    } else {
        /*          check if this new object associates with an existing object */
        assoc_id = find_associating_object(z, pscgd);
        if (assoc_id == SC_NO_ASSOCIATING_OBJECT_FOUND) {
            /* no object found, assign new label */
            free_id = get_free_label(pscgd);
            if (free_id == NO_FREE_LABEL_ID) {
                return SC_ERROR_ASSOCIATE_WITH_NEW_OBJECT_NO_FREE_LABEL;
            }
            pscgd.kal[filter_id].label = free_id;
            pscgd.kal[filter_id].time_birth = z[4];
            //MRA_LOG_DEBUG("---> New label for object %d, label is %d.", filter_id, free_id);
        } else {
            /* inherit label from associating object */
            pscgd.kal[filter_id].label = pscgd.kal[assoc_id].label;
            /* inherit birth date from associating object */
            pscgd.kal[filter_id].time_birth = pscgd.kal[assoc_id].time_birth;
            /* merge associations */
            for (k = 0; k < ASSOC_BUFFER_LENGTH; k++) {
                filter1_assoc_ptr = (k + pscgd.kal[filter_id].assoc_ptr) % ASSOC_BUFFER_LENGTH;
                filter2_assoc_ptr = (k + pscgd.kal[assoc_id].assoc_ptr) % ASSOC_BUFFER_LENGTH;
                pscgd.kal[filter_id].associations[filter1_assoc_ptr] += pscgd.kal[assoc_id].associations[filter2_assoc_ptr];
            }
            //MRA_LOG_DEBUG("---> Label for object %d inherited from existing object, label is %d.", filter_id, pscgd.kal[filter_id].label);
        }
    }

    pscgd.kal[filter_id].active = 1;

    /* associate measurement with filter of new object */
    pscgd.hyp2[j].association_id = filter_id;

    pscgd.hyp2[j].hypothesis_id = j; /* label hypothesis */

    return SC_SUCCESS;
}


static SequenceClusterCode_e generate_offspring(double *z, scw_global_data &pscgd) {
    /*  generate offspring for hypotheses */
    unsigned i, j, k;

    j = 0; /* counter for next generation of hypotheses */

    for (i = 0; i < pscgd.number_hypotheses; i++) {

        for (k = 0; k < pscgd.hyp[i].number_of_objects; k++) {
            if (associate_with_existing_object(i, j, k, z, pscgd) < 0) {
                return SC_ERROR_IN_GENERATE_OFFSPRING_ASSCOCIATE_EXISTING;
            }
            j++; /* increment counter */
        }

        if (associate_with_clutter(i, j, pscgd) != SC_SUCCESS) {
            return SC_ERROR_IN_GENERATE_OFFSPRING_CLUTTER;
        }
        j++; /* increment counter */

        SequenceClusterCode_e iret = associate_with_new_object(i, j, z, pscgd);
        if (iret != SC_MAX_REACHED and iret != SC_SUCCESS) {
            return SC_ERROR_IN_GENERATE_OFFSPRING_ASSCOCIATE_NEW;
        }
        if (iret == SC_MAX_REACHED) {
            j--;
        }
        j++; /* increment counter */

    }

    pscgd.number_hypotheses = j; /* new number of active hypotheses */
    memcpy(pscgd.hyp, pscgd.hyp2, pscgd.number_hypotheses * sizeof(hypothesis_w)); /* copy back new generation */

    return SC_SUCCESS;
}


static SequenceClusterCode_e  printAssocId(scw_global_data& pscgd, int id)
{
/*  Print kalman filter with id 'int id' with its association buffer */
    MRA_LOG_DEBUG("Id: %i \t Label: %i \t State: %f x %f x %f x %f \t Assoc: %i %i %i %i %i %i %i %i \t birth: %f \t active: %i \t assoc_ptr: %i", id, pscgd.kal[id].label,
            pscgd.kal[id].xh[0],
            pscgd.kal[id].xh[1],
            pscgd.kal[id].xh[2],
            pscgd.kal[id].xh[3],
            pscgd.kal[id].associations[0],
            pscgd.kal[id].associations[1],
            pscgd.kal[id].associations[2],
            pscgd.kal[id].associations[3],
            pscgd.kal[id].associations[4],
            pscgd.kal[id].associations[5],
            pscgd.kal[id].associations[6],
            pscgd.kal[id].associations[7],
            pscgd.kal[id].time_birth,
            pscgd.kal[id].active,
            pscgd.kal[id].assoc_ptr);
    return SC_SUCCESS;
}

static double kalman_update(double t, double* z, scw_global_data& pscgd)
{
    /* Update (associated) filters for all hypotheses */
    unsigned  i, k, idx;
    double k11, k21, k32, k42, tz, stepsize;
    
    tz=z[4]; /* 5th element is time stamp */
    stepsize=tz-t;
    
    k11=pscgd.par.kscale*0.41427948684846;
    k21=pscgd.par.kscale*1.08233129230522;
    
    k32=k11;
    k42=k21;
    
    for (i=0; i<pscgd.number_hypotheses; i++) {
        for (k=0; k<pscgd.hyp[i].number_of_objects; k++) {
            if ( k>=MAXNOBJ_GLOBAL ) {return -1.0;}
            idx=pscgd.hyp[i].filter_id[k];
            if ( tz>t ) {
                /* predict if tz greater than t */

                /* prediction of object state */
                pscgd.kal[idx].xh[0]=pscgd.kal[idx].xh[0]+stepsize*pscgd.kal[idx].xh[1];
                /* kal[idx].xh[1]=kal[idx].xh[1]; */
                pscgd.kal[idx].xh[2]=pscgd.kal[idx].xh[2]+stepsize*pscgd.kal[idx].xh[3];
                /* kal[idx].xh[3]=kal[idx].xh[3]; */
            }
            if ( (tz-t+EPS2)>=0.0 ) { /* update if tz greater than or equal to t */
                /* measurement update if associated */
                if ( pscgd.hyp[i].association_id==idx ) {
                    pscgd.kal[idx].xh[0]=pscgd.kal[idx].xh[0]+k11*(z[0]-pscgd.kal[idx].xh[0]);
                    pscgd.kal[idx].xh[1]=pscgd.kal[idx].xh[1]+k21*(z[0]-pscgd.kal[idx].xh[0]);
                    pscgd.kal[idx].xh[2]=pscgd.kal[idx].xh[2]+k32*(z[1]-pscgd.kal[idx].xh[2]);
                    pscgd.kal[idx].xh[3]=pscgd.kal[idx].xh[3]+k42*(z[1]-pscgd.kal[idx].xh[2]);
                    /* store time of update */
                    pscgd.kal[idx].time_last_update=tz;
                    /* update associations */
                    pscgd.kal[idx].associations[pscgd.kal[idx].assoc_ptr]++;
                 /* inherit radius */
                    pscgd.kal[idx].radius=z[2];
                }
            }
        }
    }
    
    t = tz;
    
    return t;
}


static SequenceClusterCode_e likelihood_update(double *z, scw_global_data &pscgd) {
    /*  likelihood update of measurement z for all hypotheses */

    unsigned i, idx;
    double tttx, ttty, p;

    for (i = 0; i < pscgd.number_hypotheses; i++) {
        idx = pscgd.hyp[i].association_id;
        if (idx >= 0) {
            tttx = z[0] - pscgd.kal[idx].xh[0];
            ttty = z[1] - pscgd.kal[idx].xh[2];
            p = exp(-0.5* ((tttx * tttx) / (pscgd.par.sigmax * pscgd.par.sigmax)
                         + (ttty * ttty) / (pscgd.par.sigmay * pscgd.par.sigmay)));
            pscgd.hyp[i].probalility = p * pscgd.hyp[i].probalility;
        }
    }

    return SC_SUCCESS;
}


static SequenceClusterCode_e normalization(scw_global_data& pscgd)
{
    /* Normalize probability of all hypotheses to sum one */
    
    unsigned i;
    double sum;
    
    sum=0.0;
    for (i=0; i<pscgd.number_hypotheses; i++) {
        sum=sum+pscgd.hyp[i].probalility;
    }
    if ( sum<=0.0 ) {
        return SC_NORMALISATION_FAILED;
    }
    for (i=0; i<pscgd.number_hypotheses; i++) {
        pscgd.hyp[i].probalility=pscgd.hyp[i].probalility/sum;
    }
    
    return SC_SUCCESS;
}



static SequenceClusterCode_e updateRingBuffers(scw_global_data &pscgd) {
    /*  increment ring buffer pointer for all active kalman filters
     *  clear the next buffer input
     */
    int i;
    for (i = 0; i < MAXFIL; ++i) {
        if (!pscgd.kal[i].active) {
            continue;
        }
        pscgd.kal[i].assoc_ptr++;
        pscgd.kal[i].assoc_ptr = pscgd.kal[i].assoc_ptr % ASSOC_BUFFER_LENGTH;
        pscgd.kal[i].associations[pscgd.kal[i].assoc_ptr] = 0;
    }

    return SC_SUCCESS;
}

static unsigned buffer_sum(scw_global_data& pscgd, unsigned id)
{
    /* calculate the association buffer sum */
    unsigned i, sum = 0;
    for(i = 0; i < ASSOC_BUFFER_LENGTH; ++i) {
        sum += pscgd.kal[id].associations[i];
    }
    return sum;
}

static SequenceClusterCode_e clip_time(double timestamp, scw_global_data &pscgd) {
    /*  clip hypotheses w.r.t. objects that have not been updated for a while */

    unsigned i, j, k, m, idx, x[MAXNOBJ_GLOBAL];

    for (i = 0; i < pscgd.number_hypotheses; i++) {
        k = 0;
        while (k < pscgd.hyp[i].number_of_objects) {
            if (k >= MAXNOBJ_GLOBAL) {
                return SC_CLIPTIME_ERROR;
            }
            idx = pscgd.hyp[i].filter_id[k];
            if ((timestamp - pscgd.kal[idx].time_last_update) > pscgd.par.maxage) {
                /*if ( (pscgd.current_time-pscgd.kal[idx].tupd)>pscgd.par.maxage ) {*/

                /* rebuild filter_id array */
                j = 0;
                for (m = 0; m < pscgd.hyp[i].number_of_objects; m++) {
                    if (m != k) {
                        if ((m >= MAXNOBJ_GLOBAL) || (j >= MAXNOBJ_GLOBAL)) {
                            return SC_CLIPTIME_ERROR;
                        }
                        x[j] = pscgd.hyp[i].filter_id[m];
                        j++;
                    }
                }

                pscgd.hyp[i].number_of_objects = pscgd.hyp[i].number_of_objects - 1; /* throw away object */
                memcpy(pscgd.hyp[i].filter_id, x, pscgd.hyp[i].number_of_objects * sizeof(int));
                k = -1;
                MRA_LOG_DEBUG("cliptime: threw away object based on too old");
            }
            k = k + 1;
        }
    }

    return SC_SUCCESS;
}

static SequenceClusterCode_e clip_rect(double xmin, double xmax, double ymin, double ymax, scw_global_data &pscgd) {
    /*  clip hypotheses at rectangular state space boundary */

    unsigned i, j, k, m, idx, x[MAXNOBJ_GLOBAL];
    double p[2];

    for (i = 0; i < pscgd.number_hypotheses; i++) {
        k = 0;
        while (k < pscgd.hyp[i].number_of_objects) {
            if (k >= MAXNOBJ_GLOBAL) {
                return SC_CLIPRECT_ERROR;
            }
            idx = pscgd.hyp[i].filter_id[k];
            p[0] = pscgd.kal[idx].xh[0];
            p[1] = pscgd.kal[idx].xh[2];
            if ((p[0] < xmin) || (p[0] > xmax) || (p[1] < ymin) || (p[1] > ymax)) {

                /*              rebuild filter_id array */
                j = 0;
                for (m = 0; m < pscgd.hyp[i].number_of_objects; m++) {
                    if (m != k) {
                        if ((m >= MAXNOBJ_GLOBAL) || (j >= MAXNOBJ_GLOBAL)) {
                            return SC_CLIPRECT_ERROR;
                        }
                        x[j] = pscgd.hyp[i].filter_id[m];
                        j++;
                    }
                }

                pscgd.hyp[i].number_of_objects = pscgd.hyp[i].number_of_objects - 1; /* throw away object */
                memcpy(pscgd.hyp[i].filter_id, x, pscgd.hyp[i].number_of_objects * sizeof(int));
                k = -1;
                MRA_LOG_DEBUG("clip_rect: threw away object outside field");
            }
            k = k + 1;
        }
    }

    return SC_SUCCESS;
}



static SequenceClusterCode_e nhyp_controller(scw_global_data &pscgd) {
    /*  filter discrete probability distribution of the hypotheses */

    unsigned i, sum, ifilter, idx[MAXHYP_W];
    double p[MAXHYP_W], psmall, pfilter;

    /*  sort p */
    for (i = 0; i < pscgd.number_hypotheses; i++) {
        p[i] = pscgd.hyp[i].probalility;
        idx[i] = i;
    }
    isort_ascending(idx, p, pscgd.number_hypotheses);

    psmall = p[idx[pscgd.number_hypotheses - 1]] / pscgd.par.pfactor;

    /*  compute mean number of objects */
    sum = 0;
    for (i = 0; i < pscgd.number_hypotheses; i++) {
        sum = sum + pscgd.hyp[i].number_of_objects;
    }
    if (pscgd.number_hypotheses == 0) {
        return SC_NHYP_CONTROL_ERROR;
    }

    ifilter = pscgd.number_hypotheses - pscgd.par.nkeep - 1;
    if (ifilter < 0) {
        return SC_SUCCESS; /* no filtering necessary */
    }

    pfilter = p[idx[ifilter]];

    /*  apply filter action to the system... */
    unsigned nr_active_hypotheses = 0;
    for (i = 0; i < pscgd.number_hypotheses; i++) {
        if (pscgd.hyp[i].probalility > pfilter && pscgd.hyp[i].probalility > psmall) {
            memcpy(&(pscgd.hyp2[nr_active_hypotheses]), &(pscgd.hyp[i]), sizeof(hypothesis_w));
            pscgd.hyp2[nr_active_hypotheses].hypothesis_id = nr_active_hypotheses; /* label hypothypothesis hyp[MAXHYP_W]hesis */
            nr_active_hypotheses++;
        }
    }

    pscgd.number_hypotheses = nr_active_hypotheses; /* new number of active hypotheses */
    memcpy(pscgd.hyp, pscgd.hyp2, pscgd.number_hypotheses * sizeof(hypothesis_w)); /* copy back new generation */

    return SC_SUCCESS;
}

static unsigned mape(scw_global_data &pscgd) {
    /* get Maximum A Posteriori estimate from hypotheses */
    unsigned i_mape = 0;
    for (unsigned hyp_idx = 1; hyp_idx < pscgd.number_hypotheses; hyp_idx++) {
        if (pscgd.hyp[hyp_idx].probalility > pscgd.hyp[i_mape].probalility) {
            i_mape = hyp_idx;
        }
    }
    return i_mape;
}


static SequenceClusterCode_e init_hyp(hypothesis_w *phyp) {
    /* initialize hypotheses */
    for (unsigned hyp_idx = 0; hyp_idx < MAXHYP_W; hyp_idx++) {
        (phyp + hyp_idx)->hypothesis_id = hyp_idx;
        (phyp + hyp_idx)->association_id = -1;
        for (unsigned filter_idx = 0; filter_idx < MAXNOBJ_GLOBAL; filter_idx++) {
            (phyp + hyp_idx)->filter_id[filter_idx] = 0;
        }
        (phyp + hyp_idx)->number_of_objects = 0;
        (phyp + hyp_idx)->probalility = 1.0;
    }

    return SC_SUCCESS;
}


static SequenceClusterCode_e select_n_obstacles(double *z, unsigned *pn, double *pobst, unsigned maxobst, double *mypos, double bound) {
    /*
     Selects the n closest obstacles from the list. If number of detected obstacles is less than n,
     the actual number of obstacles is returned in n. z is the set of (x, y, r, label, t) tuples representing
     the 2D-position, radius and label of the selected obstacles. pobst is the complete list of (x, y, r, label, t) tuples
     representing the detected obstacles. maxobst is the maximum number of obstacles in pobst.
     mypos is the 2D-position of the turtle itself.
     */

    unsigned nobst, i, j, m;
    unsigned idx[MAX_TURTLES * MAXNOBJ_LOCAL], idx2[MAX_TURTLES * MAXNOBJ_LOCAL];
    double dist2[MAX_TURTLES * MAXNOBJ_LOCAL];

    if (maxobst > MAX_TURTLES * MAXNOBJ_LOCAL) {
        MRA_LOG_DEBUG("select_n_obstacles reports: maxobst must be <= %d!", MAX_TURTLES*MAXNOBJ_LOCAL);
        maxobst = MAX_TURTLES * MAXNOBJ_LOCAL;
    }

    /*  determine number of detected obstacles */
    nobst = 0;
    m = 0;
    while ((m < MAX_TURTLES * MAXNOBJ_LOCAL) & (nobst < maxobst)) {
        /* if radius is larger than zero OR if label is larger than 0 */
        if (pobst[DIM * m + 2] > 0 || pobst[DIM * m + 3] > 0.5) {
            idx2[nobst] = m;
            nobst++;
        }
        m++;
    }

    MRA_LOG_DEBUG("found %d obstacles.", nobst);

    /* sort obstacles */
    for (i = 0; i < nobst; i++) {
        dist2[i] = (pobst[DIM * idx2[i]] - mypos[0]) * (pobst[DIM * idx2[i]] - mypos[0])
                + (pobst[DIM * idx2[i] + 1] - mypos[1]) * (pobst[DIM * idx2[i] + 1] - mypos[1]);
        idx[i] = i;
    }
    isort_ascending(idx, dist2, nobst);

    /*    take n closest obstacles for which dist2<=bound^2 */
    /*    i=0;
     k=0;
     while ( (k<*pn) && (i<nobst) ) {
     if ( dist2[idx[i]]<=bound*bound ) {
     k++;
     }
     i++;
     }
     *pn=k; */

    *pn = nobst;

    /* prepare measurements */
    for (i = 0; i < *pn; i++) {
        for (j = 0; j < DIM; j++) {
            z[DIM * i + j] = pobst[DIM * idx2[idx[i]] + j];
        }
    }

    return SC_SUCCESS;
}





SequenceClusterCode_e sc_wm(double timestamp, double *pobj, double *pr, double *pobj_birthdate, double *pobj_assoc_buffer, double *plabel,
        unsigned &r_number_of_objects, double *pobst, int maxobst, scw_global_data &pscgd) {
    /*
     inputs:  pobst     - (x, y, r, label, t) tuples of detected obstacles, padded with zeros
     maxobst   - maximum number of detected obstacles
     time      - actual time
     pscgd     - pointer to global workspace

     outputs: pobj      - (x, xdot, y, ydot) position and velocity of detected objects
     pr        - radius of detected objects
     plabel    - label of detected objects
     pnobj     - pointer to number of detected objects
     */

    unsigned n = 0, k = 0, idx = 0, imeas = 0, i_mape = 0, idx_t[MAX_TURTLES * MAXNOBJ_LOCAL], i = 0;
    double x[2] = { 0.0, 0.0 };
    double zm[DIM * MAX_TURTLES * MAXNOBJ_LOCAL], z[DIM];
    double ts[MAX_TURTLES * MAXNOBJ_LOCAL];

    n = pscgd.par.nselect;
    MRA_LOG_DEBUG("new iteration of sc_wm with %d obstacles detected", maxobst);
    select_n_obstacles(zm, &n, pobst, maxobst, x, 1000.0); /* select all obstacles */

    if (n > 0) {
        /* sort obstacles with respect to ascending time */
        for (i = 0; i < n; i++) {
            ts[i] = zm[DIM * i + 4];
        }
        isort_ascending(idx_t, ts, n);

        MRA_LOG_DEBUG("sc reports:");
        MRA_LOG_DEBUG("   time = %f s", time);
        MRA_LOG_DEBUG("   %d measurements at times:", n);
        for (i = 0; i < n; i++) {
            MRA_LOG_DEBUG("      %f", ts[idx_t[i]]);
        }

        for (imeas = 0; imeas < n; imeas++) {
            for (i = 0; i < DIM; i++) {
                z[i] = zm[DIM * idx_t[imeas] + i];
            }
            MRA_LOG_DEBUG("z[0]=%f \t z[1]=%f \t z[2]=%f \t z[3]=%f \t z[4]=%f", z[0], z[1], z[2], z[3], z[4]);
            /* Overwrite timestamps that are too old to pscgd.current_time */
            if ((z[4] - pscgd.current_time + EPS2) < 0.0) {
                /* overwrite timestamp */
                z[4] = pscgd.current_time;
            }

            /* propagate set of hypotheses */
            auto result_value = generate_offspring(z, pscgd);
            if (result_value != SC_SUCCESS) {
                return result_value;
            }

            /* apply Kalman update: prediction + measurement if associated */
            pscgd.current_time = kalman_update(pscgd.current_time, z, pscgd);
            if (pscgd.current_time < 0.0) {
                MRA_LOG_DEBUG("current time = %f", pscgd.current_time);
                return SC_KALLMAN_NEGATIVE_TIME;
            }

            /* Apply likelihood update */
            result_value = likelihood_update(z, pscgd);
            if (result_value != SC_SUCCESS) {
                return result_value;
            }

            /* Normalize probability distribution */
            result_value = normalization(pscgd);
            if (result_value != SC_SUCCESS) {
                return result_value;
            }

            /* Clip objects outside field */
            result_value = clip_rect(-FIELDWIDTH / 2.0 - FIELDMARGIN, FIELDWIDTH / 2.0 + FIELDMARGIN,
                    -FIELDLENGTH / 2.0 - FIELDMARGIN, FIELDLENGTH / 2.0 + FIELDMARGIN, pscgd);
            if (result_value != SC_SUCCESS) {
                return result_value;
            }

            /* Clip objects that have not be updated for a while */
            result_value = clip_time(timestamp, pscgd);
            if (result_value != SC_SUCCESS) {
                return result_value;
            }

            /* Gating of discrete probability distribution */
            result_value = nhyp_controller(pscgd);
            if (result_value != SC_SUCCESS) {
                return result_value;
            }

            /* Clear unused filters */
            result_value = free_unused_filters(pscgd);
            if (result_value != SC_SUCCESS) {
                return result_value;
            }
        }
    } else {
        /*      clip objects that have not be updated for a while */
        auto result_value = clip_time(timestamp, pscgd);
        if (result_value != SC_SUCCESS) {
            return result_value;
        }
        /* Clear unused filters */
        result_value = free_unused_filters(pscgd);
        if (result_value != SC_SUCCESS) {
            return result_value;
        }
    }

    /* Maximum A Posteriori (MAP) estimate */
    i_mape = mape(pscgd);

    /* return result */
    r_number_of_objects = pscgd.hyp[i_mape].number_of_objects;

    /* print all kalman filters */
    // printAssocId(pscgd);
    unsigned obst_i = 0;
    unsigned max_obst = r_number_of_objects;
    for (k = 0; k < max_obst; k++) {
        if (k >= MAXNOBJ_GLOBAL) {
            return SC_TOO_MANY_OBJECTS;
        }
        idx = pscgd.hyp[i_mape].filter_id[k];
        printAssocId(pscgd, idx);
        pobj[4 * obst_i] = pscgd.kal[idx].xh[0]; /* Put Kalman x position */
        pobj[4 * obst_i + 1] = pscgd.kal[idx].xh[1]; /* Put velocity xdot */
        pobj[4 * obst_i + 2] = pscgd.kal[idx].xh[2]; /* Put Kalman y position */
        pobj[4 * obst_i + 3] = pscgd.kal[idx].xh[3]; /* Put velocity ydot */
        plabel[obst_i] = (double) pscgd.kal[idx].label;
        pr[obst_i] = RADIUS;
        pobj_assoc_buffer[obst_i] = (double) buffer_sum(pscgd, idx);
        pobj_birthdate[obst_i] = pscgd.kal[idx].time_birth;
        obst_i++;

    }
    r_number_of_objects = max_obst;

    /* update ring buffer pointers and clear next buffer index */
    auto result = updateRingBuffers(pscgd);
    if (result != SC_SUCCESS) {
        return result;
    }

    return SC_SUCCESS;
}

SequenceClusterCode_e init_sc_wm(scw_global_data &pscgd, double time) {

    /*  tunable parameters of clustering algorithm */
    //  these are initialized in higher-level init
    pscgd.par.pclutter = 0.01;
    pscgd.par.alpha = 0.02;
    pscgd.par.sigmax = 0.3;
    pscgd.par.sigmay = 0.3;
    pscgd.par.nkeep = 25;
    pscgd.par.pfactor = 100.0;
    pscgd.par.nselect = 32;
    pscgd.par.maxnobj = 20;
    pscgd.par.clipradius = 10.0;
    pscgd.par.kscale = 0.1;
    pscgd.par.maxage = 0.5;
    pscgd.par.labelbound = 0.95;

    /* initial number of hypotheses */
    pscgd.number_hypotheses = 1;

    /* initialize time */
    pscgd.current_time = time;
    /*pscgd.current_time=0.0;*/

    /* initialize hypotheses */
    auto result = init_hyp(pscgd.hyp);
    if (result != SC_SUCCESS) {
        return result;
    }

    result = init_hyp(pscgd.hyp2);
    if (result != SC_SUCCESS) {
        return result;
    }

    /* initialize free filter index */
    result = free_unused_filters(pscgd);
    if (result != SC_SUCCESS) {
        return result;
    }

    return SC_SUCCESS;
}

