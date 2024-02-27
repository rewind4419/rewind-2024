
#include <iostream>
#include <iomanip>

#include "opencv2/opencv.hpp"

#include "net_common.h"

#include "apriltag.h"
#include "apriltag_pose.h"

#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"

using namespace std;
using namespace cv;


int main(int argc, char *argv[])
{

    cout << "Enabling video capture" << endl;

    TickMeter meter;
    meter.start();

    // Initialize camera
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
        return -1;
    }

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

    float frame_counter = 0.0f;
    meter.stop();
    meter.reset();

    Mat frame, gray;
    while (true) 
    {
        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);

        // construct april tag data
        int num_april_tags = 0;
        std::vector<NAprilTag> april_tags;

        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) 
        {
            float tagsize;
            apriltag_detection_t* tag;
    	    zarray_get(detections, i, &tag);

            //Primary Filter
            if (tag->hamming > 0) continue;

            NAprilTag april_tag = {};
            april_tag.id = tag->id;
            
            //Secondary Filter
            if(april_tag.id == 583 || april_tag.id == 584) tagsize = 0.1016; // size of tag in meters
            else if(april_tag.id == 585 || april_tag.id == 586) tagsize = 0.1524 ; // size of tag in meters
            else continue;

            num_april_tags++;


            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(frame, Point(det->p[0][0], det->p[0][1]), Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]), Point(det->p[2][0], det->p[2][1]), Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0xff, 0, 0), 2);



            apriltag_pose_t pose;
            // pose estimation            

            apriltag_detection_info_t info;
            info.det = tag;
            info.tagsize = tagsize; // size of tag in meters
            info.fx = info.fy = 2600;
            info.cx = frame.cols / 2;
            info.cy = frame.rows / 2;
        
            double err = estimate_tag_pose(&info, &pose);

            for (int j=0; j<3*3; ++j)
            april_tag.orientation[j] = pose.R->data[j];
            for (int j=0; j<3; ++j)
            april_tag.translation[j] = pose.t->data[j];

            april_tag.error = err;

            matd_destroy(pose.R);
            matd_destroy(pose.t);
        
            april_tags.push_back(april_tag);

        }

        for (int i = 0; i < april_tags.size(); i++)
        {
            if( april_tags[i].id != 0)
            {
                printf("Tag %d (x,y,z) = (%f, %f, %f) \n", april_tags[i].id, april_tags[i].translation[0], april_tags[i].translation[1], april_tags[i].translation[2]);    
                printf("Norm = %f \n", sqrt( (april_tags[i].translation[0] * april_tags[i].translation[0] ) + ( april_tags[i].translation[1] * april_tags[i].translation[1] ) + ( april_tags[i].translation[2] * april_tags[i].translation[2] )));    
            }
        }

        waitKey(1);
        imshow("Tag Detections", frame);
        apriltag_detections_destroy(detections);
    }

    apriltag_detector_destroy(td);
    tagStandard41h12_destroy(tf);

    return 0;
}
