/*
 * bot_loop_closer.cpp
 *
 *  Created on: Oct 22, 2009
 *      Author: abachrac
 */
//local stuff
//#include "IsamSlam.h"

#include <iostream>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <opencv/highgui.h>
#include <getopt.h>

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <bot_frames/bot_frames.h>

#include <scanmatch/ScanMatcher.hpp>

#include <lcmtypes/bot2_param.h>
#include <lcmtypes/sm_rigid_transform_2d_t.h>

//using namespace isam;
using namespace std;
using namespace scanmatch;

//add a new node every this many meters
#define LINEAR_DIST_TO_ADD 1.0
//and or this many radians
#define ANGULAR_DIST_TO_ADD 1.0

typedef struct {
    lcm_t * lcm;
    ScanMatcher * sm;
    ScanMatcher * sm_incremental;
    BotFrames *frames;
    BotParam *param;

    int optimize;
    int scanmatchBeforAdd;
    double odometryConfidence;
    int publish_pose;
    int do_drawing;
    int draw_map;
    char * chan;
    char * rchan;
    int beam_skip; //downsample ranges by only taking 1 out of every beam_skip points
    double spatialDecimationThresh; //don't discard a point if its range is more than this many std devs from the mean range (end of hallway)
    double maxRange; //discard beams with reading further than this value
    double maxUsableRange; //only draw map out to this far for MaxRanges...
    float validBeamAngles[2]; //valid part of the field of view of the laser in radians, 0 is the center beam
    sm_rigid_transform_2d_t * prev_odom;
    sm_rigid_transform_2d_t * prev_sm_odom;

    int verbose;
    double primary_laser_offset;
    //double rear_laser_offset;
    //double rear_laser_ang_offset;

    bot_core_planar_lidar_t * r_laser_msg;
    int64_t r_utime;

} app_t;


void app_destroy(app_t *app);

////////////////////////////////////////////////////////////////////
//where all the work is done
////////////////////////////////////////////////////////////////////
static void laser_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)), const char * channel __attribute__((unused)), const bot_core_planar_lidar_t * msg,
			  void * user  __attribute__((unused)))
{
    static int count = 0;
    count++;
    if(count == 40){
        count = 0;
        fprintf(stderr,".");
    }

    app_t * app = (app_t *) user;
    ////////////////////////////////////////////////////////////////////
    //Project ranges into points, and decimate points so we don't have too many
    ////////////////////////////////////////////////////////////////////
    smPoint * points = (smPoint *) calloc(msg->nranges, sizeof(smPoint));


    int numValidPoints = sm_projectRangesAndDecimate(app->beam_skip, app->spatialDecimationThresh, msg->ranges,
                                                     msg->nranges, msg->rad0, msg->radstep, points, app->maxRange, app->validBeamAngles[0], app->validBeamAngles[1]);
    if (numValidPoints < 30) {
        fprintf(stderr, "WARNING! NOT ENOUGH VALID POINTS! numValid=%d\n", numValidPoints);
        return;
    }

    double sensor_to_body[12];
    if (!bot_frames_get_trans_mat_3x4 (app->frames, app->chan,
                                   "body",
                                   sensor_to_body)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        return;
    }

    //transform to body pose
    double pos_s[3] = {0}, pos_b[3];
    for(int i=0; i < numValidPoints; i++){
        pos_s[0] = points[i].x;
        pos_s[1] = points[i].y;
        bot_vector_affine_transform_3x4_3d (sensor_to_body, pos_s , pos_b);
        points[i].x = pos_b[0];
        points[i].y = pos_b[1];
    }


    ////////////////////////////////////////////////////////////////////
    //Actually do the matching
    ////////////////////////////////////////////////////////////////////

    //this does it incrementally - so this maintains the frame

    ScanTransform r = app->sm_incremental->matchSuccessive(points, numValidPoints, SM_HOKUYO_UTM, sm_get_utime(), NULL); //don't have a better estimate than prev, so just set prior to NULL

    sm_rigid_transform_2d_t odom;
    memset(&odom, 0, sizeof(odom));
    odom.utime = msg->utime;
    odom.pos[0] = r.x;
    odom.pos[1] = r.y;
    odom.theta = r.theta;
    memcpy(odom.cov, r.sigma, 9 * sizeof(double));

    if (app->publish_pose) {
        bot_core_pose_t pose;
        memset(&pose, 0, sizeof(pose));
        pose.pos[0] = odom.pos[0];//prop_pose.x();
        pose.pos[1] = odom.pos[1];//prop_pose.y();
        //fprintf(stderr,"Publishing : %f,%f\n",  pose.pos[0] , pose.pos[1]);
        double rpy[3] = { 0, 0, odom.theta};
        bot_roll_pitch_yaw_to_quat(rpy, pose.orientation);

        pose.utime = msg->utime;
        bot_core_pose_t_publish(app->lcm, "POSE", &pose);

    }
    return;

    free(points);
    sm_tictoc("laser_handler");

}

void app_destroy(app_t *app)
{
    //TODO: there is a double free mess cuz scan matcher free's scans that are held onto elsewhere :-/
    //LEAK everything for now!!!

    //  if (app) {
    //    if (app->chan)
    //      free(app->chan);
    //
    //    lcm_destroy(app->lcm);
    //    if (app->prev_odom)
    //      botlcm_rigid_transform_2d_t_destroy(app->prev_odom);
    //
    //    if (app->isamslam)
    //      delete app->isamslam;
    //    fprintf(stderr, "deleted isamSlam object\n");
    //    if (app->sm)
    //      delete app->sm;
    //    if (app->sm_incremental)
    //      delete app->sm_incremental;
    //
    //    free(app);
    //  }

    // dump timing stats
    sm_tictoc(NULL);
    exit (1);

}

static void usage(const char *name)
{
    fprintf(stderr, "usage: %s [options]\n"
            "\n"
            "  -h, --help                      Shows this help text and exits\n"
            "  -a,  --aligned                  Subscribe to aligned_laser (ROBOT_LASER) msgs\n"
            "  -c, --chan <LCM CHANNEL>        Input lcm channel default:\"LASER\" or \"ROBOT_LASER\"\n"
            "  -s, --scanmatch                 Run Incremental scan matcher every time a node gets added to map\n"
            "  -d, --draw                      Show window with scan matches \n"
            //"  -p, --publish_pose              publish POSE messages\n"
            "  -v, --verbose                   Be verbose\n"
            "  -m, --mode  \"HOKUYO_UTM\"|\"SICK\" configures low-level options.\n"
            "\n"
            "Low-level laser options:\n"
            "  -M, --mask <min,max>            Mask min max angles in (radians)\n"
            "  -B, --beamskip <n>              Skip every n beams \n"
            "  -D, --decimation <value>        Spatial decimation threshold (meters?)\n"
            "  -R, --range <range>             Maximum range (meters)\n", name);
}

sig_atomic_t still_groovy = 1;

static void sig_action(int signal, siginfo_t *s, void *user)
{
    still_groovy = 0;
}

int read_parameters(app_t * app)
{
    BotParam *c = app->param;//bot_param_new_from_server(app->lcm, 1);//globals_get_config();
    BotFrames *frames = bot_frames_get_global (app->lcm, c);
    //char key[2048];

    char *coord_frame;
    coord_frame = bot_param_get_planar_lidar_coord_frame (c, app->chan);

    if (!coord_frame) {
        fprintf (stderr, "\tError determining %s laser coordinate frame\n", app->chan);
        return -1;
    }

    BotTrans primary_laser_to_body;
    if (!bot_frames_get_trans (frames, coord_frame, "body", &primary_laser_to_body))
        fprintf (stderr, "\tError determining %s laser coordinate frame\n", app->chan);
    else
        fprintf(stderr,"\t%s Laser Pos : (%f,%f,%f)\n", app->chan,
                primary_laser_to_body.trans_vec[0], primary_laser_to_body.trans_vec[1], primary_laser_to_body.trans_vec[2]);

    app->primary_laser_offset = primary_laser_to_body.trans_vec[0];


    // coord_frame = NULL;
    // coord_frame = bot_param_get_planar_lidar_coord_frame (c, "SKIRT_REAR");

    // if (!coord_frame)
    //     fprintf (stderr, "\tError determining rear laser coordinate frame\n");

    // BotTrans rear_laser_to_body;
    // if (!bot_frames_get_trans (frames, coord_frame, "body", &rear_laser_to_body))
    //     fprintf (stderr, "\tError determining rear laser coordinate frame\n");
    // else
    //     fprintf(stderr,"\tRear Laser Pos : (%f,%f,%f)\n",
    //             rear_laser_to_body.trans_vec[0], rear_laser_to_body.trans_vec[1], rear_laser_to_body.trans_vec[2]);

    // app->rear_laser_offset = rear_laser_to_body.trans_vec[0];

    // double rear_rpy[3];
    // bot_quat_to_roll_pitch_yaw (rear_laser_to_body.rot_quat, rear_rpy);
    // app->rear_laser_ang_offset = rear_rpy[2];



    // ouble position[3];
    // sprintf(key, "%s.%s.%s.position", "calibration", "planar_lidars", "SKIRT_FRONT");
    // if(3 != bot_param_get_double_array(c, key, position, 3)){
    //     fprintf(stderr,"\tError Reading Params\n");
    // }else{
    //     fprintf(stderr,"\tFront Laser Pos : (%f,%f,%f)\n",position[0], position[1], position[2]);
    // }
    // app->primary_laser_offset = position[0];

    // double rpy[3];
    // sprintf(key, "%s.%s.%s.rpy", "calibration", "planar_lidars", "SKIRT_FRONT");
    // if(3 != bot_param_get_double_array(c, key, rpy, 3)){
    //     fprintf(stderr,"\tError Reading Params\n");
    // }else{
    //     fprintf(stderr,"\tFront Laser RPY : (%f,%f,%f)\n",rpy[0], rpy[1], rpy[2]);
    // }

    // sprintf(key, "%s.%s.%s.position", "calibration", "planar_lidars", "SKIRT_REAR");
    // if(3 != bot_param_get_double_array(c, key, position, 3)){
    //     fprintf(stderr,"\tError Reading Params\n");
    // }else{
    //     fprintf(stderr,"\tRear Laser Pos : (%f,%f,%f)\n",position[0], position[1], position[2]);
    // }
    // app->rear_laser_offset = position[0];

    // sprintf(key, "%s.%s.%s.rpy", "calibration", "planar_lidars", "SKIRT_REAR");
    // if(3 != bot_param_get_double_array(c, key, rpy, 3)){
    //     fprintf(stderr,"\tError Reading Params\n");
    // }else{
    //     fprintf(stderr,"\tRear Laser RPY : (%f,%f,%f)\n",rpy[0], rpy[1], rpy[2]);
    // }
    // app->rear_laser_ang_offset  = carmen_degrees_to_radians(rpy[2]);

    return 1;
}


int main(int argc, char *argv[])
{
    setlinebuf(stdout);

    app_t *app = (app_t *) calloc(1, sizeof(app_t));
    app->prev_odom = (sm_rigid_transform_2d_t *) calloc(1, sizeof(sm_rigid_transform_2d_t)); //set initial prev_odom to zero...
    app->prev_sm_odom = (sm_rigid_transform_2d_t *) calloc(1, sizeof(sm_rigid_transform_2d_t)); //set initial prev_odom to zero...
    memset(app->prev_odom, 0, sizeof(sm_rigid_transform_2d_t));
    memset(app->prev_sm_odom, 0, sizeof(sm_rigid_transform_2d_t));

    bool alignLaser = false;
    app->chan = NULL;
    app->rchan = NULL;
    app->verbose = 0;
    app->do_drawing = 0;
    app->publish_pose = 0;
    app->scanmatchBeforAdd = 0;
    app->odometryConfidence = LINEAR_DIST_TO_ADD;
    app->draw_map = 0;

    // set to default values
    //TODO:these should be read from command line or something...
    //parameters for a hokuyo with the helicopters mirror's attached
    app->validBeamAngles[0] = -100;//-2.358;
    app->validBeamAngles[1] = +100;//2.358;
    app->beam_skip = 3;
    app->spatialDecimationThresh = .2;
    app->maxRange = 29.7;
    app->maxUsableRange = 8;

    const char *optstring = "hc:v";
    char c;
    struct option long_opts[] = { { "help", no_argument, 0, 'h' },
                                  { "chan", required_argument, 0, 'c' },
                                  { "verbose", no_argument, 0,'v' },
                                  { 0, 0, 0, 0 } };

    app->scanmatchBeforAdd = 1;
    app->publish_pose = 1;

    while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
        switch (c) {
        case 'c':
            free(app->chan);
            app->chan = strdup(optarg);
            fprintf(stderr,"Main Laser Channel : %s\n", app->chan);
            break;
        case 'v':
            app->verbose = 1;
            break;
        case 'h':
        default:
            usage(argv[0]);
        return 1;
        }
    }

    if (app->chan == NULL) {
        app->chan = strdup("SKIRT_FRONT");
    }


   //we take the floor that we are on as the 0th floor for now
    app->lcm = lcm_create(NULL);
    if (!app->lcm) {
        fprintf(stderr, "ERROR: lcm_create() failed\n");
        return 1;
    }

    app->param = bot_param_new_from_server(app->lcm, 1);
    int ret = read_parameters(app);
    if (ret == -1)
        return -1;

    app->frames = bot_frames_get_global (app->lcm, app->param);

    fprintf(stderr,"Primary laser Offset %f\n",app->primary_laser_offset);


    if (app->verbose) {
        if (alignLaser)
            printf("INFO: Listening for robot_laser msgs on %s\n", app->chan);
        else
            printf("INFO: Listening for laser msgs on %s\n", app->chan);

        printf("INFO: Do Draw:%d\n", app->do_drawing);
        printf("INFO: publish_pose:%d\n", app->publish_pose);
        printf("INFO: Max Range:%lf\n", app->maxRange);
        printf("INFO: SpatialDecimationThresh:%lf\n", app->spatialDecimationThresh);
        printf("INFO: Beam Skip:%d\n", app->beam_skip);
        printf("INFO: validRange:%f,%f\n", app->validBeamAngles[0], app->validBeamAngles[1]);
    }

    //initialize tictoc for threading

    if (app->sm->isUsingIPP())
        fprintf(stderr, "Using IPP\n");
    else
        fprintf(stderr, "NOT using IPP\n");

    //hardcoded loop closing scan matcher params
    double metersPerPixel_lc = .02; //translational resolution for the brute force search
    double thetaResolution_lc = .01; //angular step size for the brute force search
    int useGradientAscentPolish_lc = 1; //use gradient descent to improve estimate after brute force search
    int useMultires_lc = 3; // low resolution will have resolution metersPerPixel * 2^useMultiRes
    bool useThreads_lc = true;


    //hardcoded scan matcher params
    double metersPerPixel = .02; //translational resolution for the brute force search
    double thetaResolution = .02; //angular step size for the brute force search
    sm_incremental_matching_modes_t matchingMode = SM_GRID_COORD; //use gradient descent to improve estimate after brute force search
    int useMultires = 3; // low resolution will have resolution metersPerPixel * 2^useMultiRes

    double initialSearchRangeXY = .15; //nominal range that will be searched over
    double initialSearchRangeTheta = .1;//sachi was 0.1

    //SHOULD be set greater than the initialSearchRange
    double maxSearchRangeXY = .3; //if a good match isn't found I'll expand and try again up to this size...
    double maxSearchRangeTheta = .3; //if a good match isn't found I'll expand and try again up to this size...

    int maxNumScans = 30; //keep around this many scans in the history
    double addScanHitThresh = .90; //add a new scan to the map when the number of "hits" drops below this

    int useThreads = 1;

    //create the incremental scan matcher object
    app->sm_incremental = new ScanMatcher(metersPerPixel, thetaResolution, useMultires, useThreads, false);
    //set the scan matcher to start at pi/2...
    ScanTransform startPose;
    memset(&startPose, 0, sizeof(startPose));
    app->sm_incremental->initSuccessiveMatchingParams(maxNumScans, initialSearchRangeXY, maxSearchRangeXY,
                                                      initialSearchRangeTheta, maxSearchRangeTheta, matchingMode, addScanHitThresh, false, .3, &startPose);

    fprintf(stderr, "Subscribing : %s\n", app->chan);
    bot_core_planar_lidar_t_subscribe(app->lcm, app->chan, laser_handler, app);

    struct sigaction new_action;
    new_action.sa_sigaction = sig_action;
    sigemptyset(&new_action.sa_mask);
    new_action.sa_flags = 0;

    sigaction(SIGINT, &new_action, NULL);
    sigaction(SIGTERM, &new_action, NULL);
    sigaction(SIGKILL, &new_action, NULL);
    sigaction(SIGHUP, &new_action, NULL);

    /* sit and wait for messages */
    while (still_groovy)
        lcm_handle(app->lcm);

    app_destroy(app);

    return 0;

fail:
    app_destroy (app);
    return -1;
}
