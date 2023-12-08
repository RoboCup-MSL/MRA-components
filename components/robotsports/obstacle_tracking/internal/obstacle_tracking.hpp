/**
 *  @file
 *  @brief   object tracking process
 *  @curator Ton Peijnenburg
 */

#ifndef OBJECT_PROCESS_H
#define OBJECT_PROCESS_H

#include <stdint.h>

#include "objectdef.hpp"
#include "sequence_clustering_track_objects.hpp"
#include "RobotsportsObstacleTracking_datatypes.hpp"

#define OBJECT_LABEL_OWNTEAM 10

// Position definition
typedef struct pos
{
  double x; // x coordinate
  double y; // y coordinate
  double z; // z coordinate
  double r; // rotation in radials
} pos_t;


typedef struct color
{
  long r;
  long g;
  long b;
} color_t;

typedef struct object_s {

    long        type;
    pos_t       pos;    // Position in field CS
    pos_t       vel;    // Velocity in field CS
    double      size;
    double      confidence; // Quality of observation indication
                // How should this be defined ?
                // Requirement: 1=as good as it gets, lower is worse with a linear ratio
                // So if your vision accuracy with the ball closest is 3cm, you should
                // fill in 1 when it is closest. When it later is far away with 3m accuracy,
                // fill in 0.01, as it is 100x less accurate. This way the kalman filter
                // will know that it can expect a higher standard deviation.
    //long      label;  // Label for use with object tracking
    double      ts; // Timestamp of the position and speed

    color_t     col;    // main color (used for debugging)

} object_t;



// The following struct defines data stored for the task.
// The framework uses this struct so it becomes part of the blackboard.
//typedef struct object_process_output_s
//{
//    object_t object[NUM_OBSTACLES];
//    object_t object_prev[NUM_OBSTACLES];
//} object_process_output_t;
//

typedef struct object_process_s
{
    double	ts;		// Time stamp of data
    double last_processed_vision_ts;	// last processed vision data timestamp
    double update_interval_vision;		// update rate for vision (in seconds)
    double last_processed_selves_ts;	// last processed shared selves timestamp
    double update_interval_selves;

    // Below data defined 
    long        nr_objects;

    object_t    out[MAXNOBJ_GLOBAL];
    long		nobj;
    long		use_shared_selves;		// if true, use self positions of team mates shared through network
    double		min_conf_shared_selves;	// minimum confidence level to include shared selves

    // interface to Turtle2 code
    int             cnt_iskicking;
    long            dump_sc;                // trigger for status dump

    // parameters for Turtle2 code
    double par_pclutter;        /* probability that new measurement is clutter */
    double par_alpha;           /* pnew = alpha * pexist */
    double par_sigmax;          /* standard deviation measurement noise, x-direction */
    double par_sigmay;          /* standard deviation measurement noise, y-direction */
    long   par_nkeep;           /* number of hypotheses saved for next step */
    double par_pfactor;         /* reject hypotheses with p < pmax/pfactor */
    long   par_nselect;         /* number of selected obstacles from input list */
    long   par_maxnobj;         /* maximum number of objects in an hypothesis */
    double par_clipradius;      /* clipping radius relative to own position */
    double par_kscale;          /* scale factor for Kalman gain */
    double par_maxage;          /* maximum age for non-updated objects */
    double par_labelbound;      /* minimum required likelihood for object association */

    // parameter that indicates error in filter, e.g. out of Kalman filters
    long   filter_error;		// is set if error in filter occurs
    long   reset_on_error;		// reset entire object filter if error occurs
    double filter_reset_time;	// time when filter was last reset

} obstacle_tracking_t;


void obstacle_tracking(double timestamp,
                        MRA::RobotsportsObstacleTracking::Input const& input,
                        MRA::RobotsportsObstacleTracking::Params const &params,
                        MRA::RobotsportsObstacleTracking::State &state,
                        MRA::RobotsportsObstacleTracking::Output &output);


#endif  // OBJECT_PROCESS_H
