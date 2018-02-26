/*
 * ptam_library.cpp
 *
 *  Created on: 23 Nov 2016
 *      Author: toky
 */




/*
 *
 *
 * This benchmark file can run any SLAM algorithm compatible with the SLAMBench API
 * We recommend you to generate a library which is compatible with this application.
 *
 * The interface works that way :
 *   - First benchmark.cpp will call an initialisation function of the SLAM algorithm. (void sb_init())
 *     This function provides the compatible interface with the SLAM algorithm.
 *   - Second benchmark.cpp load an interface (the textual interface in our case)
 *   - Then for every frame, the benchmark.cpp will call sb_process()
 *
 *
 */


#include <timings.h>
#include <SLAMBenchAPI.h>

#include "OpenGL.h"
#include <gvars3/instances.h>
#include <stdlib.h>
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"
#include "ARDriver.h"
#include "MapViewer.h"
#include "MapPoint.h"

#include <io/SLAMFrame.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/CameraSensorFinder.h>

#include <string>

static const double default_distortion_parameter = 0.1;

static const double default_rotation_estimator_blur = 0.75;
static const double default_coarse_min_velocity = 0.006;
static const double default_good_quality_threshold = 0.3;
static const double default_lost_quality_threshold = 0.13;
static const double default_min_tukey_sigma = 0.4;
static const double default_candidate_minshitomasi_score   = 70.0;

static const int  default_use_rotation_estimator = 1;
static const int  default_disable_coarse  = 0;


static const int default_max_patch_per_frame = 1000;
static const int default_mini_patch_max_ssd             = 100000;
static const unsigned int default_coarse_min                     = 20;
static const unsigned int default_coarse_max                     = 60;
static const unsigned int default_coarse_range                   = 30;
static const int default_coarse_sub_pix_it              = 8;

static const  unsigned int default_first_space_bar = 1;
static const  unsigned int default_second_space_bar = 3;

static const std::string default_tracker_estimator = "Tukey";
static const std::string default_bundle_estimator = "Tukey";




	    double distortion_parameter;
	    double candidate_minshitomasi_score  ;
	    double rotation_estimator_blur;
	    double coarse_min_velocity;
	    double good_quality_threshold;
	    double lost_quality_threshold;
	    double min_tukey_sigma;

	    int use_rotation_estimator;
	    int disable_coarse ;

	    int max_patch_per_frame;
	    int mini_patch_max_ssd            ;
	    unsigned int coarse_min                    ;
	    unsigned int coarse_max                    ;
	    unsigned int coarse_range                  ;
	    int coarse_sub_pix_it             ;


	    std::string tracker_estimator;
	    std::string bundle_estimator;



 CVD::Image<CVD::byte> mimFrameBW;

 Map *mpMap;
 MapMaker *mpMapMaker;
 Tracker *mpTracker;
 ATANCamera *mpCamera;
 ARDriver *mpARDriver;
 MapViewer *mpMapViewer;
 sb_uint2 inputSize ;

 unsigned int first_space_bar = 0;
 unsigned int second_space_bar = 0;

 bool mbDone;

 static std::vector<unsigned char>     grey_raw;

 static slambench::io::CameraSensor *grey_sensor = nullptr;


 //=========================================================================
 // SLAMBench output values
 //=========================================================================


 slambench::outputs::Output *pose_output = nullptr;
 slambench::outputs::Output *frame_output = nullptr;
 slambench::outputs::Output *pointcloud_output = nullptr;


 //=========================================================================
 // SLAMBench API
 //=========================================================================



 bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings)  {

	        slam_settings->addParameter(TypedParameter<double>("dp", "distortion_parameter",     "distortion_parameter",      &distortion_parameter, &default_distortion_parameter));
	        slam_settings->addParameter(TypedParameter<double>("cms", "candidate_minshitomasi_score",     "candidate_minshitomasi_score",      &candidate_minshitomasi_score, &default_candidate_minshitomasi_score));
	        slam_settings->addParameter(TypedParameter<double>("reb", "rotation_estimator_blur",     "rotation_estimator_blur",      &rotation_estimator_blur, &default_rotation_estimator_blur));
	        slam_settings->addParameter(TypedParameter<double>("cmv", "coarse_min_velocity",         "coarse_min_velocity",      &coarse_min_velocity, &default_coarse_min_velocity));
	        slam_settings->addParameter(TypedParameter<double>("gqt", "good_quality_threshold",     "good_quality_threshold",      &good_quality_threshold, &default_good_quality_threshold));
	        slam_settings->addParameter(TypedParameter<double>("lqt", "lost_quality_threshold",     "lost_quality_threshold",      &lost_quality_threshold, &default_lost_quality_threshold));
	        slam_settings->addParameter(TypedParameter<double>("mts", "min_tukey_sigma",     "min_tukey_sigma",      &min_tukey_sigma, &default_min_tukey_sigma));
	        slam_settings->addParameter(TypedParameter<int>("ure", "use_rotation_estimator",     "use_rotation_estimator",      &use_rotation_estimator, &default_use_rotation_estimator));
	        slam_settings->addParameter(TypedParameter<int>("dc", "disable_coarse",     "disable_coarse",      &disable_coarse, &default_disable_coarse));
	        slam_settings->addParameter(TypedParameter<int>("maxpf", "max_patch_per_frame",     "max_patch_per_frame",      &max_patch_per_frame, &default_max_patch_per_frame));
	        slam_settings->addParameter(TypedParameter<int>("minpf", "mini_patch_max_ssd",     "mini_patch_max_ssd",      &mini_patch_max_ssd, &default_mini_patch_max_ssd));
	        slam_settings->addParameter(TypedParameter<unsigned int>("cmin", "coarse_min",     "coarse_min",      &coarse_min, &default_coarse_min));
	        slam_settings->addParameter(TypedParameter<unsigned int>("xmax", "coarse_max",     "coarse_max",      &coarse_max, &default_coarse_max));
	        slam_settings->addParameter(TypedParameter<unsigned int>("crange", "coarse_range",     "coarse_range",      &coarse_range, &default_coarse_range));
	        slam_settings->addParameter(TypedParameter<int>("cspt", "coarse_sub_pix_it",     "coarse_sub_pix_it",      &coarse_sub_pix_it, &default_coarse_sub_pix_it));
	        slam_settings->addParameter(TypedParameter<unsigned int>("fsbar", "first_space_bar",     "first_space_bar",      &first_space_bar, &default_first_space_bar));
	        slam_settings->addParameter(TypedParameter<unsigned int>("ssbar", "second_space_bar",     "second_space_bar",      &second_space_bar, &default_second_space_bar));
	        slam_settings->addParameter(TypedParameter<std::string>("te", "tracker_estimator",     "tracker_estimator",      &tracker_estimator, &default_tracker_estimator));
	        slam_settings->addParameter(TypedParameter<std::string>("be", "bundle_estimator",     "bundle_estimator",      &bundle_estimator, &default_bundle_estimator));


     return true;
 }

 bool sb_init_slam_system(SLAMBenchLibraryHelper * slam_settings) {



   /*****************************************************
    * *** Collect sensors
    ******************************************************/

 	slambench::io::CameraSensorFinder sensor_finder;
	grey_sensor = sensor_finder.FindOne(slam_settings->get_sensors(), {{"camera_type", "grey"}});
	
 	if (grey_sensor == nullptr) {
 		std::cerr << "Invalid sensors found, Grey not found." << std::endl;
 		return false;
 	}

 	// check sensor frame and pixel format
 	if(grey_sensor->PixelFormat != slambench::io::pixelformat::G_I_8) {
 		std::cerr << "Grey sensor is not in G_I_8 format" << std::endl;
 		return false;
 	}
 	if(grey_sensor->FrameFormat != slambench::io::frameformat::Raster) {
 		std::cerr << "Grey sensor is not in raster format" << std::endl;
 	}


 	 inputSize   = {grey_sensor->Width,grey_sensor->Height};
     ImageRef ptam_imge_size ( grey_sensor->Width,grey_sensor->Height);
     mimFrameBW.resize(ptam_imge_size);



     TooN::Vector<5> vect ;
     // 0 - normalized x focal length
     vect[0] = grey_sensor->Intrinsics[0];
     // 1 - normalized y focal length
     vect[1] = grey_sensor->Intrinsics[1];
     // 2 - normalized x offset
     vect[2] = grey_sensor->Intrinsics[2];
     // 3 - normalized y offset
     vect[3] = grey_sensor->Intrinsics[3];
     // 4 - w (distortion parameter)
     vect[4] = distortion_parameter;


     GVars3::GV3::get< typename TooN::Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters") =  vect;

     GVars3::GV3::get<double>("MapMaker.CandidateMinShiTomasiScore") = candidate_minshitomasi_score;

     GVars3::GV3::get<double>("Tracker.RotationEstimatorBlur") = rotation_estimator_blur;
     GVars3::GV3::get<double>("Tracker.CoarseMinVelocity") = coarse_min_velocity;
     GVars3::GV3::get<double>("Tracker.TrackingQualityGood") = good_quality_threshold;
     GVars3::GV3::get<double>("Tracker.TrackingQualityLost") = lost_quality_threshold;
     GVars3::GV3::get<double>("Bundle.MinTukeySigma") = min_tukey_sigma;

     GVars3::GV3::get<int>("Tracker.UseRotationEstimator") = use_rotation_estimator;
     GVars3::GV3::get<int>("Tracker.DisableCoarse") = disable_coarse;

     GVars3::GV3::get<int>("Tracker.MaxPatchesPerFrame") = max_patch_per_frame;
     GVars3::GV3::get<int>("Tracker.MiniPatchMaxSSD") = mini_patch_max_ssd;
     GVars3::GV3::get<unsigned int>("Tracker.CoarseMin") = coarse_min;
     GVars3::GV3::get<unsigned int>("Tracker.CoarseMax") = coarse_max;
     GVars3::GV3::get<unsigned int>("Tracker.CoarseRange") = coarse_range;
     GVars3::GV3::get<int>("Tracker.CoarseSubPixIts") = coarse_sub_pix_it;

     GVars3::GV3::get<std::string>("TrackerMEstimator") = tracker_estimator;
     GVars3::GV3::get<std::string>("BundleMEstimator") = bundle_estimator;


     std::cerr << "Create camera" << std::endl;
     mpCamera = new ATANCamera("Camera");
     mpCamera->SetImageSize(ptam_imge_size);


     std::cerr << "Create map" << std::endl;
     mpMap = new Map;

     std::cerr << "Create mpMapMaker" << std::endl;
     mpMapMaker = new MapMaker(*mpMap, *mpCamera);

     std::cerr << "Create mpTracker" << std::endl;
     mpTracker  = new Tracker(ptam_imge_size, *mpCamera, *mpMap, *mpMapMaker);



 	//=========================================================================
 	// DECLARE OUTPTUS
 	//=========================================================================


 	pose_output = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);
 	slam_settings->GetOutputManager().RegisterOutput(pose_output);
 	pose_output->SetActive(true);

 	frame_output = new slambench::outputs::Output("Frame", slambench::values::VT_FRAME);
 	frame_output->SetKeepOnlyMostRecent(true);
 	slam_settings->GetOutputManager().RegisterOutput(frame_output);
 	frame_output->SetActive(true);

 	 pointcloud_output = new slambench::outputs::Output("PointCloud", slambench::values::VT_POINTCLOUD);
 	 pointcloud_output->SetKeepOnlyMostRecent(true);
 	 slam_settings->GetOutputManager().RegisterOutput(pointcloud_output);
 	 pointcloud_output->SetActive(true);




     return true;
 }

 bool sb_update_frame (SLAMBenchLibraryHelper * slam_settings, slambench::io::SLAMFrame* s) {
		if(s->FrameSensor == grey_sensor) {
			memcpy((void*) mimFrameBW.data(), s->GetData(), s->GetSize());
			s->FreeData();
			return true;
		}

		return false;
 }

 bool sb_process_once (SLAMBenchLibraryHelper * slam_settings)  {

     // emulate human click
     static int  frame = 0;


     mpTracker->TrackFrame(mimFrameBW, false);

     if (frame == first_space_bar) {Tracker::GUICommandCallBack(mpTracker,"PokeTracker","");}
     if (frame == second_space_bar) {Tracker::GUICommandCallBack(mpTracker,"PokeTracker","");}

     frame ++ ;

     std::cerr << mpTracker->GetMessageForUser() << std::endl;
     return true;
 }

 template<typename P>
 inline  Eigen::Matrix4f toMatrix4( const TooN::SE3<P> & p){

     TooN::Matrix<4,4,float,TooN::ColMajor> M;

     M.slice<0,0,3,3>() = p.get_rotation().get_matrix();
     M.T()[3].slice<0,3>() = p.get_translation();
     M[3] = TooN::makeVector(0,0,0,1);
     Eigen::Matrix4f R;
     std::memcpy(&R(0,0), &(M[0][0]),16*sizeof(float));
     return R;






 }

bool sb_get_pose ( Eigen::Matrix4f * mat) {

     *mat = toMatrix4(mpTracker->GetCurrentPose());
     return true;
 }

 bool sb_get_tracked  (bool* tracked) {
     *tracked =  mpMap->IsGood();
     return true;
 }



 bool sb_clean_slam_system() {

 	 delete pose_output;
 	 delete frame_output;
 	 delete pointcloud_output;

     delete mpTracker;
     delete mpMapMaker;
     delete mpMap;
     delete mpCamera;

     return true;
 }



 bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *latest_output) {
 	(void)lib;

 	slambench::TimeStamp ts = *latest_output;

 	if(pose_output->IsActive()) {

 		Eigen::Matrix4f mat =  toMatrix4(mpTracker->GetCurrentPose());

 		std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
 		pose_output->AddPoint(ts, new slambench::values::PoseValue(mat));
 	}




 		if(frame_output->IsActive()) {

 			frame_output->AddPoint(*latest_output,
 					new slambench::values::FrameValue(inputSize.x, inputSize.y,
 							slambench::io::pixelformat::EPixelFormat::G_I_8 , mimFrameBW.data()));
 		}



 		if(pointcloud_output->IsActive()) {
 			slambench::values::ColoredPointCloudValue *point_cloud = new slambench::values::ColoredPointCloudValue();


 		     for(size_t i=0; i<mpMap->vpPoints.size(); i++) {
 		    	slambench::values::ColoredPoint3DF new_vertex;


 		         // Color
 		         int level = mpMap->vpPoints[i]->nSourceLevel;
 		         switch (level) {
 		         case 0 :  new_vertex.R = 255; new_vertex.G = 000; new_vertex.B = 000; break;
 		         case 1 :  new_vertex.R = 255; new_vertex.G = 255; new_vertex.B = 000; break;
 		         case 2 :  new_vertex.R = 000; new_vertex.G = 255; new_vertex.B = 000; break;
 		         case 3 :  new_vertex.R = 000; new_vertex.G = 000; new_vertex.B = 150; break;
 		         default:  new_vertex.R = 255; new_vertex.G = 255; new_vertex.B = 150; break;
 		         }

 		         // Position
 		         auto v3Pos = mpMap->vpPoints[i]->v3WorldPos;
 		         new_vertex.X = v3Pos[0];
 		         new_vertex.Y = v3Pos[1];
 		         new_vertex.Z = v3Pos[2];

 		    	point_cloud->AddPoint(new_vertex);

 		     }




 			// Take lock only after generating the map
 			std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
 			pointcloud_output->AddPoint(ts, point_cloud);

 		}




 	return true;
 }









