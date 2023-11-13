#ifndef SEQ_CLUSTERING_BALLDEF_HPP
#define SEQ_CLUSTERING_BALLDEF_HPP

/* define ball types */
typedef enum balltype_e {
        OMNIVISION_b = 1,
        LRFTOP_b,
        LRFBOT_b,
        FRONT_CAMERA_b,
        STEREO_b,
        ARTIFICIAL_b
} balltype_e;

/* define options for ball update flag */
typedef enum ball_update_e {
        BU_NONE = 0,
        BU_TODO,
        BU_DONE
} ball_update_e;

/* struct definition of ball candidate */
typedef struct tag_ball_candidate_t {
        double x;                       /* x-position */
        double y;                       /* y-position */
        double z;                       /* z-position */
        double confidence;              /* confidence */
        balltype_e  type;               /* sensor label */
        double sigma;                   /* standard deviation of sensor noise */
        bool   is_free;                 /* is rolling freely (0 or 1) */
        bool   in_air;                  /* is flying in the air (0 or 1) */
        double timestamp;               /* timestamp */
} ball_candidate_t;

/* struct definition of ball estimate */
typedef struct tag_ball_estimate_t {
        double x;               /* x-position */ /* last x candidate */
        double y;               /* y-position */ /* last y candidate */
        double xhat;            /* x-position */ /* x estimate */
        double yhat;            /* y-position */ /* y estimate */
        double z;               /* z-position */
        double xdot;            /* x-velocity */
        double ydot;            /* y-velocity */
        double zdot;            /* z-velocity */
        double hconf;           /* Moving Average confidence of winning hypothesis */
        bool is_updated;        /* is winning hypothesis updated with candidate in last sample */
        balltype_e label;       /* type of last added candidate in winning hypothesis */
        double timestamp;       /* ball timestamp */
} ball_estimate_t;

#endif // SEQ_CLUSTERING_BALLDEF_H
