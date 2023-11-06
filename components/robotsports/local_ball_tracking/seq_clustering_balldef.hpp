/**
 *  @file
 *  @brief   ball definitions
 *  @curator Ton Peijnenburg
 */

#ifndef SEQ_CLUSTERING_BALLDEF_HPP
#define SEQ_CLUSTERING_BALLDEF_HPP

/* define ball types */
typedef enum balltype_e {
        OV_b = 1,
        LRFTOP_b,
        LRFBOT_b,
        FC_b,
        STEREO_b,
        ARTIFICIAL_b
} balltype_e;

/* define options for ball update flag */
typedef enum ball_update_e {
        BU_NONE = 0,
        BU_TODO,
        BU_DONE
} ball_update_e;

/* DEFINITION OF BALL INFORMATION IN BALLSHARED.H */
/* mergedBall_xyz_xyzdot contains the ball from the turtles own tracker, extrapolated to the current time.
 * seeBall is one if a all feature is found very recently ( about 50 ms )
 * confidence of the merged ball is the confidence of the ball from the tracker. 
 * This confidence will go to zero if no ball is added to the tracker for 1 second
 * usedBallTurtleID contains the turtleID of the used ball in the software. When you want to know which ball is used
 * in the software of a given robot, this value has to be checked. By checking this variable, one can also
 * see if a communicated ball is used or the ball seen by the turtle itself */

/* struct definition of ball feature */
typedef struct tag_ball_feature_t {
        double x;                       /* x-position */
        double y;                       /* y-position */
        double z;                       /* z-position */
        double conf;                    /* confidence */
        double dist;                    /* distance to robot */
        long  type;                     /* sensor label */
        double sigma;                   /* standard deviation of sensor noise */
        bool   isFree;                  /* is rolling freely (0 or 1) */
        bool   inAir;                   /* is flying in the air (0 or 1) */
        double timestamp;               /* timestamp */
        long  initializeBallVelFlag;   /* initialize ball velocity if one */
        double initializeBallVel_xy[2]; /* initialize ball velocity at kick vel_xy */
} ball_feature_t;

/* struct definition of ball estimate */
typedef struct tag_ball_estimate_t {
        double x;               /* x-position */ /* last x feature */
        double y;               /* y-position */ /* last y feature */
        double xhat;            /* x-position */ /* x estimate */
        double yhat;            /* y-position */ /* y estimate */
        double z;               /* z-position */
        double xdot;            /* x-velocity */
        double ydot;            /* y-velocity */
        double zdot;            /* z-velocity */
        double hconf;           /* Moving Average confidence of winning hypothesis */
        long isUpd;            /* is winning hypothesis updated with feature in last sample */
        long label;            /* type of last added feature in winning hypothesis */
        double timestamp;       /* ball timestamp */
} ball_estimate_t;

#endif // SEQ_CLUSTERING_BALLDEF_H
