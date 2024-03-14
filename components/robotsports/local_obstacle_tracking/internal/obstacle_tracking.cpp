/**
 *  @file
 *  @brief   obstacle tracking process
 *  @curator Ton Peijnenburg
 *  @desc    determine best obstacles out of obstacle observations from various sensors
 *           using code from Tech United, written by Rene van de Molengraft
 *           a very opportunistic interface is chosen, that
 *           leaves much of the original data structure in place
 */

#include "../../local_obstacle_tracking/RobotsportsLocalObstacleTracking_datatypes.hpp"
#include "../../local_obstacle_tracking/internal/obstacle_tracking.hpp"

#include <unistd.h>
#include <string.h>
#include <cmath>

#include "../../local_obstacle_tracking/internal/obstacle_definitions.hpp"
#include "../../local_obstacle_tracking/internal/sequence_clustering_track_obstacles.hpp"
#include "logging.hpp"

static scw_global_data pscgd;
static double obstacles_detected[DIM*MAX_TURTLES*MAXNOBJ_LOCAL];
static double pobj[4*MAX_TURTLES*MAXNOBJ_LOCAL];
static double pobj_radius[MAX_TURTLES*MAXNOBJ_LOCAL];
static double pobj_birthdate[MAX_TURTLES*MAXNOBJ_LOCAL];
static double pobj_assoc_buffer[MAX_TURTLES*MAXNOBJ_LOCAL];
static double pobj_label[MAX_TURTLES*MAXNOBJ_LOCAL];


obstacle_tracking_t obstacle_process;

static int processObstacles(MRA::RobotsportsLocalObstacleTracking::ObstacleCandidate obstacle_candidate, double* obstacleData)
{
    /* copy obstacle position */
    obstacleData[0] = obstacle_candidate.pose_fcs().x();			// x position in extrapolated coordinates
    obstacleData[1] = obstacle_candidate.pose_fcs().y();			// y position in extrapolated coordinates
    obstacleData[2] = obstacle_candidate.radius(); 			// radius of obstacle
    obstacleData[3] = 0;							// label = 0 is for unknown (default)
    obstacleData[4] = google::protobuf::util::TimeUtil::TimestampToMilliseconds(obstacle_candidate.timestamp()) / 1000.0;				// timestamp

    return 1;
}

int obstacle_tracking_initialize(double timestamp)
{
    pos_t    zero_pos    = { 0.0, 0.0, 0.0, 0.0 };
    obstacle_t zero_obstacle = { 0, zero_pos, zero_pos, 0.0, 0.0, 0.0 };

    // initialize obstacle memory in output part of the sensor
    for (int i = 0; i < MAXNOBJ_GLOBAL; i++) {
        obstacle_process.out[i]= zero_obstacle;
    }

    obstacle_process.nobj = 0;
    obstacle_process.last_processed_vision_ts =  -1.0;
    obstacle_process.update_interval_vision = 0.05;
    obstacle_process.last_processed_selves_ts = -1.0;
    obstacle_process.update_interval_selves = 0.05;

    // initialize static data structure for administration of obstacles by/from filter
    for(int i=0;i<MAXNOBJ_GLOBAL;i++){
        pobj[4*i]		= 0.0;
        pobj[4*i+1]		= 0.0;
        pobj[4*i+2]		= 0.0;
        pobj[4*i+3]		= 0.0;
        pobj_radius[i]	= 0.0;
        pobj_label[i]	= 0.0;
        pobj_birthdate[i]		= 1e10;		// put birth date in the future
        pobj_assoc_buffer[i]	= 0.0;
    }

    // default minimum confidence level for shared selves
    obstacle_process.use_shared_selves = 1;
    obstacle_process.min_conf_shared_selves = 0.5;

    // initialize parameters
    // parameters for Turtle2 code
    obstacle_process.par_pclutter = 0.01;	/* probability that new measurement is clutter, default value is 0.001, changed to 0.02 after experimentation on 20170404 */
    obstacle_process.par_alpha = 0.01;	/* pnew = alpha * pexist, default value is 0.01 */
    obstacle_process.par_sigmax = 0.25;	/* standard deviation measurement noise, x-direction, default value is 0.250 */
    obstacle_process.par_sigmay = 0.25;	/* standard deviation measurement noise, y-direction, default value is 0.250 */
    obstacle_process.par_nkeep = 20;		/* number of hypotheses saved for next step, default is 30 */
    obstacle_process.par_pfactor = 10.0;	/* reject hypotheses with p < pmax/pfactor, default is 100.0 */
    obstacle_process.par_nselect = 32;		/* number of selected obstacles from input list, default is 385 */
    obstacle_process.par_maxnobj = 10;		/* maximum number of obstacles in an hypothesis, default is 20 */
    obstacle_process.par_clipradius = 1000.0;/* clipping radius relative to own position, default is 1000.0 */
    obstacle_process.par_kscale = 0.5;	/* scale factor for Kalman gain, default is 0.9 */
    obstacle_process.par_maxage = 1.0;	/* maximum age for non-updated obstacles, was 0.5 */
    obstacle_process.par_labelbound = 0.60;	/* minimum required likelihood for obstacle association, default value is 0.95 */

    obstacle_process.filter_error =  0;		// this parameter is set if an error in the filter occurs, e.g. out of Kalman filters
    obstacle_process.reset_on_error =  1;

    // initialize data structure for sequential clustering with first hypothesis
    return init_sc_wm(pscgd, timestamp);
}

void obstacle_tracking(double timestamp,
                        MRA::RobotsportsLocalObstacleTracking::Input const& input,
                        MRA::RobotsportsLocalObstacleTracking::Params const &params,
                        MRA::RobotsportsLocalObstacleTracking::State &state,
                        MRA::RobotsportsLocalObstacleTracking::Output &output)
{
    if (not state.is_initialized()) {
        obstacle_tracking_initialize(timestamp);
        state.set_is_initialized(true);
    }

	// only now we fill with obstacles
    /* check for each sensor if new data is received */

    pos_t		zero_pos    = { 0.0, 0.0, 0.0, 0.0 };
    obstacle_t	zero_obstacle = { 0, zero_pos, zero_pos, 0.0, 0.0, 0.0 };

    // reset filter if error has occurred and reset_on_error is set to true
    if (obstacle_process.filter_error and obstacle_process.reset_on_error) {
        obstacle_process.filter_error = 0;
        obstacle_process.filter_reset_time = timestamp;
    	init_sc_wm(pscgd, timestamp);
    }

    // copy filter parameters from shared memory so they can be updated run-time (not compile-time)
    pscgd.par.pclutter	= obstacle_process.par_pclutter;	// tunable parameter, default value is 0.001
    pscgd.par.alpha		= obstacle_process.par_alpha;		// tunable parameter, default value is 0.01
    pscgd.par.sigmax	= obstacle_process.par_sigmax;		// tunable parameter, default value is 0.250
    pscgd.par.sigmay	= obstacle_process.par_sigmay;		// tunable parameter, default value is 0.250
    pscgd.par.nkeep		= (int) obstacle_process.par_nkeep;		// tunable parameter, default value is 30
    pscgd.par.pfactor	= obstacle_process.par_pfactor;		// tunable parameter, default value is 100.0
    pscgd.par.nselect	= (int)obstacle_process.par_nselect;		// tunable parameter, default value is 385
    pscgd.par.maxnobj	= (int)obstacle_process.par_maxnobj;		// tunable parameter, default value is 20
    if (pscgd.par.maxnobj > MAXNOBJ_GLOBAL) {
        pscgd.par.maxnobj = MAXNOBJ_GLOBAL;
    }
    pscgd.par.clipradius = obstacle_process.par_clipradius;	// tunable parameter, default value is 1000.0
    pscgd.par.kscale	= obstacle_process.par_kscale;		// tunable parameter, default value is 0.9
    pscgd.par.maxage	= obstacle_process.par_maxage;		// tunable parameter, default value is 0.3
    pscgd.par.labelbound = obstacle_process.par_labelbound;	// default value is 0.95

    double last_processed_vision_timestamp = obstacle_process.last_processed_vision_ts;

    // first sensor is omni_vision
    // check for updated obstacle positions first, only then update obstacle positions in extrapolated coordinates
    // TODO may be replaced by just single check of vision updates master ts field hw.omni.ts with latest timestamp
    // check if input data is newer than data stored during the last sample

    // initialize obstacle counter
    unsigned obstacles_this_time = 0;

    int nr_obstacles = input.obstacle_candidates_size();
    MRA_LOG_DEBUG("INPUT nr_obstacles =  %d", nr_obstacles);


    // start with omni vision reports
    // process when > 0 obstacles have been reported
    if (nr_obstacles > 0) {
    	// include limit on nr_obstacles? does not seem to be required; nr_obstacles should always be less or equal to maximum amount of reported obstacles
        // get time stamp for new information
        double last_omni_timestamp = -1.0;
        for (int i = 0; i < nr_obstacles; i++) {
            double obstacle_ts = google::protobuf::util::TimeUtil::TimestampToMilliseconds(input.obstacle_candidates(i).timestamp()) / 1000.0;
            MRA_LOG_DEBUG("obstacle [%d] ts: %f", i, obstacle_ts);
            if (obstacle_ts > last_omni_timestamp) {
                last_omni_timestamp = obstacle_ts;
            }
        }
        MRA_LOG_DEBUG("last_omni_timestamp: %f", last_omni_timestamp);
        // check for new features from omni camera - a sinle new obstacle is enough!
        //printf("obstacle_process: obstacles %d last_omni %f last_obstacle_process %f\n", nr_obstacles, last_omni_timestamp, last_processed_vision_timestamp);
        MRA_LOG_DEBUG("last_processed_vision_timestamp: %f", last_processed_vision_timestamp);
        if (last_omni_timestamp > last_processed_vision_timestamp) {
            // we have new features!
            // copy to Tech United data structure called obstacles_detected array
            // first, copy obstacles found by omni_vision
            for (auto i = 0; i < nr_obstacles; i++) {
                // processobstacles will move the data to the Tech United data structure
            	// add only observations that are newer than (last update + interval)
                double obstacle_ts = google::protobuf::util::TimeUtil::TimestampToMilliseconds(input.obstacle_candidates(i).timestamp()) / 1000.0;
            	if (obstacle_ts > last_processed_vision_timestamp) {
					processObstacles(input.obstacle_candidates(i), &obstacles_detected[obstacles_this_time*DIM]);
					MRA_LOG_DEBUG("process_obstacles has added obstacle at (%f,%f) with label %d and timestamp %f\n",
					        obstacles_detected[obstacles_this_time*DIM],
					        obstacles_detected[obstacles_this_time*DIM+1],
					        obstacles_detected[obstacles_this_time*DIM+3],
					        obstacles_detected[obstacles_this_time*DIM+4]);
					obstacles_this_time++;
					// only update last_processed_vision_ts if at least one omni obstacle has been used for updating the filter
					obstacle_process.last_processed_vision_ts = last_omni_timestamp;
            	}
            }
        }
        MRA_LOG_DEBUG("obstacle_process.last_processed_vision_ts: %f", obstacle_process.last_processed_vision_ts);
    }

    if (obstacles_this_time > MAX_TURTLES*MAXNOBJ_LOCAL) {
        MRA_LOG_DEBUG("warning: obstacle_process has more input obstacles this time than can be handled");
	}
    MRA_LOG_DEBUG("obstacle_process obstacles than can be handled: %d", obstacles_this_time);

    // stub with zero obstacles;
    auto obstacle_candidate = MRA::RobotsportsLocalObstacleTracking::ObstacleCandidate();
    for (int k = obstacles_this_time; k < MAX_TURTLES*MAXNOBJ_LOCAL; k++) {
		processObstacles(obstacle_candidate, &obstacles_detected[k*DIM]);
    }

    // update administration
    obstacle_process.nr_obstacles = obstacles_this_time;

    // now run sc_bm code if obstacles are found
    unsigned pnobj;
    int ret = sc_wm(timestamp, pobj, pobj_radius, pobj_birthdate, pobj_assoc_buffer, pobj_label, pnobj, obstacles_detected, obstacles_this_time,  pscgd);
    MRA_LOG_DEBUG("obstacle process | time: %6.3f  return: %3d  nr_obstacles: %3d obstacles: %3d pnob: %3d", timestamp, ret, nr_obstacles, obstacles_this_time, pnobj);
    if (ret == SC_SUCCESS) {
        // update obstacle positions in world model since a successful step has been done
        // copy position and velocity data, and size, for all obstacles reported by filter
        //    	printf("#obstacles clustered %d\n", pnobj);
        unsigned i = 0;
        output.mutable_obstacles()->Clear();
        for (i = 0; i < pnobj; i++) {
            obstacle_process.out[i].pos.x = pobj[4*i];
            obstacle_process.out[i].vel.x = pobj[4*i+1];
            obstacle_process.out[i].pos.y = pobj[4*i+2];
            obstacle_process.out[i].vel.y = pobj[4*i+3];
            obstacle_process.out[i].size =  pobj_radius[i];
            obstacle_process.out[i].ts = timestamp;
            if (pobj_label[i] < OBSTACLE_LABEL_OWNTEAM) {
                obstacle_process.out[i].type = OBSTACLE_PLAYER_US;
            }
            else {
                obstacle_process.out[i].type = OBSTACLE_PLAYER_THEM;
            }
            obstacle_process.out[i].confidence = 0.8;
            // TODO temporary assignment of obstacle filter label to col.r for debugging purposes
            //      may be used for association of opponent robot detections - this is reported but may be difficult due to noise
            obstacle_process.out[i].col.r = pobj_label[i];

            auto trackedObstacle = MRA::Datatypes::TrackedObject();
            trackedObstacle.mutable_pos_vel_fcs()->mutable_position()->set_x(obstacle_process.out[i].pos.x);;
            trackedObstacle.mutable_pos_vel_fcs()->mutable_velocity()->set_x(obstacle_process.out[i].vel.x);
            trackedObstacle.mutable_pos_vel_fcs()->mutable_position()->set_y(obstacle_process.out[i].pos.y);;
            trackedObstacle.mutable_pos_vel_fcs()->mutable_velocity()->set_y(obstacle_process.out[i].vel.y);
            auto timestamp_obj = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(obstacle_process.out[i].ts * 1000);
            trackedObstacle.mutable_timestamp()->CopyFrom(timestamp_obj);
            trackedObstacle.set_confidence(obstacle_process.out[i].confidence);
            trackedObstacle.set_type(obstacle_process.out[i].type);
            output.mutable_obstacles()->Add()->CopyFrom(trackedObstacle);
        }
        // stub remaining out fields with zero obstacles
        while (i < MAXNOBJ_GLOBAL) {
            obstacle_process.out[i] = zero_obstacle;
            i++;
        }
        // include number of (valid) obstacles
        obstacle_process.nobj = (long)pnobj;
        // include time stamp
        obstacle_process.ts = timestamp;
    } // if return is successful, otherwise no update

}
