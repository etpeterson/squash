#include <iostream>
//#include <functional>
#include <opencv2/opencv.hpp>
#include "opencv2/nonfree/features2d.hpp"
//#include <opencv2/core/core_c.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/objdetect/objdetect.hpp>

//using namespace cv;

cv::Point2f pt(-1,-1);
cv::Point2f xyloc(-1,-1);
bool wasclick=false,wasrelease=true;


/* This was my pie in the sky attempt to do everything super C++ like, but I don't think it's worth my time at this point, I'll just create a longer main function
 
class image_interaction {
private:
    int keyresponse,keyflag;
    bool ispaused=false;
    cv::Mat frame, frame_raw;
    cv::VideoCapture cap;
    //virtual int keycallback(),clickcallback();
    std::function<void(void)> keycallback;
    std::function<void(int,int,int,int,void*)> clickcallback;
public:
    image_interaction(cv::VideoCapture , int , const std::function<void(void)> , const std::function<void(int,int,int,int,void*)> );
    //static void image_callback(int event, int x, int y, int flags, void* image); //maybe this should be input by the user
    //void run();
};
                           

image_interaction::image_interaction(cv::VideoCapture cap, int keyflag, const std::function<void(void)> keycallback, const std::function<void(int,int,int,int,void*)> clickcallback) : cap(cap), keyflag(keyflag), keycallback(keycallback), clickcallback(clickcallback){
    //should probably put most if this in a run function
    //cap=cap_in;
    //keyflag=keyflag_in;
    //keycallback=keycallback_in;
    //clickcallback=clickcallback_in;
    
    cv::namedWindow("frame",1); //create the window
    //cv::setMouseCallback("frame", CallBackFunc, (void*)&xyloc); //create the mouse callback for point passback
    cv::setMouseCallback("frame",clickcallback,(void*) &frame);
    while(cap.isOpened()){
        //some basic movie controls
        std::cout<<"click location "<<xyloc<<std::endl;
        keyresponse=cv::waitKey(1);
        //waittime=1;
        if (keyresponse==113){ //q
            std::cout<<"Goodbye"<<std::endl;
            break;
        }else if (keyresponse==112 || keyresponse==32){ //pause (p or space bar)
            //waittime=0; //wait forever next time
            if (ispaused==false){
                ispaused=true;
            }else{
                ispaused=false;
                
            }
            
        }else if (keyresponse==keyflag){ //d for define squash court
            std::cout<<"Defining the squash court"<<std::endl;
            keycallback();
        }
        //std::cout<<cv::waitKey(0)<<std::endl;
        
        //cv::Mat frame;
        if (ispaused==false){
            cap >> frame_raw; // get a new frame from camera
        }
        //frame=frame_raw;
        frame_raw.copyTo(frame);
        
        //cvtColor(frame, edges, COLOR_BGR2GRAY);
        //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        //Canny(edges, edges, 0, 30, 3);
        cv::line(frame,pt,xyloc,CV_RGB(255,255,255));
        imshow("frame", frame);
        
        //if(waitKey(1) >= 0)
        //    break;
    }

  }
*/


void image_callback(int event, int x, int y, int flags, void* image)
{
    //cv::Point* xylocptr = (cv::Point*) xyloc;
    //cv::Mat* imageptr = (cv::Mat*) image;
    //cv::Point2i point(x,y);
    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
        pt.x=x;
        pt.y=y;
        //xylocptr->x=x;
        //xylocptr->y=y;
    }
    else if  ( event == cv::EVENT_RBUTTONDOWN )
    {
        std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if  ( event == cv::EVENT_MBUTTONDOWN )
    {
        std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if ( event == cv::EVENT_MOUSEMOVE )
    {
        std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;
        std::cout << "Original clock location (" << pt.x << ", " << pt.y << ")" << std::endl;
        xyloc.x=x;
        xyloc.y=y;
        //point.x=x;
        //point.y=y;
        //cv::line(*imageptr,pt,point,CV_RGB(255,255,255));
        
    }
    
}




//the squash court class
class squash_court {
    //http://www.ussquash.com/wp-content/uploads/2014/03/131023_Rules-of-Singles-Squash-2014.pdf
protected:
    const int nverts=22; //number of verticies of a squash court
    //cv::Point2i court_img_verts[22]; //all the court verticies
    std::vector< std::vector<cv::Point2f> > court_img_verts; //must be floats otherwise we get strange issues from calibrateCamera, like sizes are not equal, etc
    const std::string vert_names[22] = {"FW_SW_L", "FW_SW_R", "SL_SW_L", "SL_SW_R", "T_SW_L", "T_SW_R", "F_SW_L","F_SW_R","BW_SW_L","BW_SW_R","BW_F_L","BW_F_R","BW_HCL","F_HCL","SW_SL_L","SW_SL_R","SB_SL_L","SB_SL_R","SB_SW_L","SB_SW_R","SB_L","SB_R"};
    const std::string vert_descriptions[22] = {"front wall line and side wall line left", "front wall line and side wall line right", "service line and side wall left", "service line and side wall right", "tin and side wall left", "tin and side wall right", "floor and side wall left","floor and side wall right","back wall line and side wall line left","back wall line and side wall line right","back wall and floor left","back wall and floor right","back wall and half court line","short line and half court line","side wall and short line left","side wall, and short line right","service box and short line left","service box and short line right","service box and side wall left","service box and side wall right","service box lone corner left","service box lone corner right"};
    /*cv::Point2i FW_SW_L, FW_SW_R; //front wall line and side wall line left and right
    cv::Point2i SL_SW_L, SL_SW_R; //service line and side wall left and right
    cv::Point2i T_SW_L, T_SW_R; //tin and side wall left and right
    cv::Point2i F_SW_L, F_SW_R; //floor and side wall left and right
    cv::Point2i BW_SW_L, BW_SW_R; //back wall line and side wall line left and right
    cv::Point2i BW_F_L, BW_F_R; //back wall and floor left and right
    cv::Point2i BW_HCL, F_HCL; //back wall and half court line and floor and half court line
    cv::Point2i F_SW_SL_L, F_SW_SL_R; //floor, side wall, and short line left and right
    cv::Point2i SB_SL_L, SB_SL_R; //service box and short line left and right
    cv::Point2i SB_SW_L, SB_SW_R; //service box and side wall left and right
    cv::Point2i SB_L, SB_R; //service box lone corner left and right*/
    
    //cv::Point3f court_verts[22]; //the actual 3D locations of the court verticies
    std::vector< std::vector<cv::Point3f> > court_verts; //all the court verticies in actual locations (sized in the constructor)
    //std::vector<cv::Point3f> court_verts;
    //let's define the back left as (0,0,0), and everything can be positive from there in meters
    //so x is across the court, y is front to back, and z is vertical
    
    /* TODO add the actual positions of all the points in true 3D in meters
    cv::Point2i FW_SW_L, FW_SW_R; //front wall line and side wall line left and right
    cv::Point2i SL_SW_L, SL_SW_R; //service line and side wall left and right
    cv::Point2i T_SW_L, T_SW_R; //tin and side wall left and right
    cv::Point2i F_SW_L, F_SW_R; //floor and side wall left and right
    cv::Point2i BW_SW_L, BW_SW_R; //back wall line and side wall line left and right
    cv::Point2i BW_F_L, BW_F_R; //back wall and floor left and right
    cv::Point2i BW_HCL, F_HCL; //back wall and half court line and floor and half court line
    cv::Point2i F_SW_SL_L, F_SW_SL_R; //floor, side wall, and short line left and right
    cv::Point2i SB_SL_L, SB_SL_R; //service box and short line left and right
    cv::Point2i SB_SW_L, SB_SW_R; //service box and side wall left and right
    cv::Point2i SB_L, SB_R; //service box lone corner left and right
     */
    
    //camera section
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double rms;
    
public:
    void get_corners(cv::Mat);
    static void court_call_back(int , int , int , int , void *);
    void draw_court(cv::Mat);
    void setup_camera_matrix(cv::Point2i);
    void setup_rvec();
    void setup_tvec();
    void camera_calibration(cv::Point2i,int);
    
    squash_court();
    friend std::ostream& operator<< (std::ostream &out, squash_court &sc);
    
};

squash_court::squash_court(){
    //make the vectors the right size
    //std::vector<cv::Point2i> test;
    court_img_verts.resize(1);
    court_img_verts[0].resize(nverts);
    court_verts.resize(1);
    court_verts[0].resize(nverts);
    
    //std::cout<<court_img_verts.size()<<" "<<court_img_verts[0].size()<<std::endl;
    
    //front y is 9.750
    //left x is 0.0
    //right x is 6.400
    //top front z is 4.570
    //top back z is 2.130
    
    //{left/right,front/back,up/down}
    //court_verts[0][0].y=0;
    //court_verts[0][0].x=0.000;
    //court_verts[0][0].y=9.750; court_verts[0][0].z=4.570;
    court_verts[0][0]={0.000,9.750,4.595}; //front wall line and side wall line left
    court_verts[0][1]={6.400,9.750,4.595}; //front wall line and side wall line right
    court_verts[0][2]={0.000,9.750,1.805}; //service line and side wall left
    court_verts[0][3]={6.400,9.750,1.805}; //service line and side wall right
    court_verts[0][4]={0.000,9.750,0.455}; //tin and side wall left. NOTE: the tin height for womens is 0.505 (480+25), and 0.455 (430+25) for mens
    court_verts[0][5]={6.400,9.750,0.455}; //tin and side wall right. NOTE: the tin height for womens is 0.505 (480+25), and 0.455 (430+25) for mens
    court_verts[0][6]={0.000,9.750,0.000}; //floor and side wall left
    court_verts[0][7]={6.400,9.750,0.000}; //floor and side wall right
    court_verts[0][8]={0.000,0.000,2.155}; //back wall line and side wall line left
    court_verts[0][9]={6.400,0.000,2.155}; //back wall line and side wall line right
    court_verts[0][10]={0.000,0.000,0.000}; //back wall and floor left
    court_verts[0][11]={6.400,0.000,0.000}; //back wall and floor right
    court_verts[0][12]={3.200,0.000,0.000}; //back wall and half court line
    court_verts[0][13]={3.200,4.285,0.000}; //floor and half court line
    court_verts[0][14]={0.000,4.285,0.000}; //floor, side wall, and short line left
    court_verts[0][15]={6.400,4.285,0.000}; //floor, side wall, and short line right
    court_verts[0][16]={1.625,4.285,0.000}; //service box and short line left
    court_verts[0][17]={4.775,4.285,0.000}; //service box and short line right
    court_verts[0][18]={0.000,2.635,0.000}; //service box and side wall left
    court_verts[0][19]={6.400,2.635,0.000}; //service box and side wall right
    court_verts[0][20]={1.625,2.635,0.000}; //service box lone corner left
    court_verts[0][21]={4.775,2.635,0.000}; //service box lone corner right
    
    
    
    //camera section
    //cameraMatrix.eye(3, 3, CV_64F);
    //distCoeffs.eye(8, 1, CV_64F);
    //cameraMatrix=cv::Mat::eye
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F); //we need to tweak this later once the image size is known
    distCoeffs = cv::Mat::zeros(5, 1, CV_64F); //only a 5 parameter fit
    rms=0;
    rvecs.resize(1);
    rvecs[0] = cv::Mat::zeros(1, 3, CV_64F); //Try initializing zeros
    tvecs.resize(1);
    tvecs[0] = cv::Mat::zeros(1, 3, CV_64F); //Try initializing zeros and fill in later
    //std::vector<cv::Mat> rvecs, tvecs;
}

//the overloaded cout
std::ostream& operator<< (std::ostream &out, squash_court &sc)
{
    for(int i=0;i<sc.nverts;i++){
        out<<i<<" "<<sc.court_img_verts[i]<<" "<<sc.vert_descriptions[i]<<std::endl;
    }
    // Since operator<< is a friend of the Point class, we can access
    // Point's members directly.
    /*out << "(" << cPoint.m_dX << ", " <<
    cPoint.m_dY << ", " <<
    cPoint.m_dZ << ")"; */
    return out;
}

//the call back for the squash court definition
void squash_court::court_call_back(int event, int x, int y, int flags, void* image){
    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        //std::cout << "CCB Left button of the mouse is down - position (" << x << ", " << y << ")" << std::endl;
        pt.x=x;
        pt.y=y;
        wasclick=true;
        wasrelease=false;
        //xylocptr->x=x;
        //xylocptr->y=y;
    }else if ( event == cv::EVENT_MOUSEMOVE ){
        //std::cout << "CCB Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;
        //std::cout << "CCB Original clock location (" << pt.x << ", " << pt.y << ")" << std::endl;
        xyloc.x=x;
        xyloc.y=y;
        //point.x=x;
        //point.y=y;
        //cv::line(*imageptr,pt,point,CV_RGB(255,255,255));
    }else if ( event == cv::EVENT_LBUTTONUP){
        //std::cout << "CCB Left button of the mouse is up" << std::endl;
        wasrelease=true;
        wasclick=false; //this is set elsewhere, but we can do it again
    }
    
    //}else if ( event == cv::EVENT_LBUTTONDBLCLK){
    //    std::cout << "CCB Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;

    //}

}

//the get corners function
void squash_court::get_corners(cv::Mat frame_raw){
    /* this function gets the corners by asking the user to click on them.
     I should really let the users drag the corners around, but that would take some reworking of the code */
    
    cv::Mat frame_corners;
    int keyresponse=-1;
    cv::Point2i p2i_zeros(0,0);
    int court_vert_idx=0;
    cv::Point2i framesize=frame_raw.size();
    float mindist=1.0e12,dprod=0.0;
    int minidx=0;
    
    //set up the camera matrix (it's kind of complicated)
    setup_camera_matrix(framesize);
    /*double f = 55.0; // focal length in mm (55 is about what it finds, but I don't know if that's right
    double sx = 36.0, sy = 24.0;             // sensor size (total guess that it's full frame)
    cameraMatrix.at<double>(0,2)=static_cast<double>(framesize.x)/2.0; //finish setting the cameraMatrix because calibrateCamera can't do it for me
    cameraMatrix.at<double>(1,2)=static_cast<double>(framesize.y)/2.0;  //finish setting cameraMatrix
    cameraMatrix.at<double>(0,0)=static_cast<double>(framesize.x)*f/sx; //fx
    cameraMatrix.at<double>(1,1)=static_cast<double>(framesize.y)*f/sy; //fy
    std::cout<<"camera matrix"<<std::endl<<cameraMatrix<<std::endl;  //this doesn't seem to change*/
    /* This seems to be a decent camera matrix, which is not too different than what I am sending in
     [2207.302662623425, 0, 640;
     0, 1241.607747725677, 360;
     0, 0, 1]*/
    //cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    
    //set up the tvec
    setup_tvec();
    /*tvecs[0].at<double>(0)=3.2; //3.2m in x (centered)
    tvecs[0].at<double>(1)=-10.0; //5m back
    tvecs[0].at<double>(2)=5.0; //5m up */
    /* this is an example tvec, which isn't too different than the input, but still isn't correct
     [-3.186344372084879, 4.024008286339618, 16.0116542958002]*/
    
    //set up the rvec
    setup_rvec();
    /*rvecs[0].at<double>(0)=2.0; //not sure what this is
    rvecs[0].at<double>(1)=0.0; //not sure what this is
    rvecs[0].at<double>(2)=0.0; //not sure what this is */
    /* This is based off of an output rvec which put the court just about in the middle of the screen
     [2.011442578743535, -0.009058716752575486, 0.0001397269699944912]*/
    
    //I need to add a while loop and some other key processing to keep it in here!
    std::cout<<"Press b to begin setting points, s to set the points, and u to undo the previous point"<<std::endl;
    
    //we took a blind guess at the beginning, so let's just project the points!
    cv::projectPoints(court_verts[0], rvecs[0], tvecs[0], cameraMatrix, distCoeffs, court_img_verts[0]);
    
    while(keyresponse!='q'){
        //std::cout<<"CCB keyresponse"<<keyresponse<<std::endl;
    //void (squash_court::*ccb)(int);
    //ccb=&squash_court::court_call_back;
    frame_raw.copyTo(frame_corners);
    cv::namedWindow("frame callback",2); //create the window
    //cv::setMouseCallback("frame", CallBackFunc, (void*)&xyloc); //create the mouse callback for point passback
    cv::setMouseCallback("frame callback",&squash_court::court_call_back,0);
    
        
        if(keyresponse=='u' || keyresponse=='s' || keyresponse=='b'){
            if (keyresponse=='u'){
                court_vert_idx--;
            }
            else if(keyresponse=='s'){
                court_img_verts[0][court_vert_idx]=pt;
                court_vert_idx++;
            }
            if(court_vert_idx>=nverts){ //if we can start trying to estimate the camera pose
                //std::cout<<"camera matrix"<<cameraMatrix<<std::endl;
                std::cout<<"Calibrating the pose. Please wait..."<<std::endl;
                //Note, it may be bad to calibrate many times, I may need to be careful here!
                //cv::solvePnP(court_verts[0], court_img_verts[0], cameraMatrix, distCoeffs, rvecs[0], tvecs[0],true,CV_ITERATIVE); //This result is wonky...
                //cv::solvePnPRansac(court_verts[0], court_img_verts[0], cameraMatrix, distCoeffs, rvecs[0], tvecs[0],true,10); //Ransac has issues, something must be wrong here!
                rms=cv::calibrateCamera(court_verts, court_img_verts, framesize, cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS |   CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K1 | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3);
                //CV_CALIB_FIX_ASPECT_RATIO | //fixing the aspect ratio was not good, it distorted the result
                //fix principal point could be removed, especially when finding the distortion matrix
                //note that if we let the distortions correct then the camera matrix also gets a little crazy
                //I think we can fairly safely assume that the camera is basically distortionless
                //all the above arguments to calibrateCamera mean that it's essentially not optimizing the distortion matrix, only the camera matrix, rotation and translation vectors
                std::cout<<"RMS error = "<<rms<<std::endl;
                std::cout<<"camera matrix"<<std::endl<<cameraMatrix<<std::endl;  //this doesn't seem to change
                //std::cout<<"distortion coefficients"<<std::endl<<distCoeffs<<std::endl; //forced to be zero above
                //std::cout<<"rotation vectors"<<rvecs.size()<<" "<<rvecs[0].size()<<std::endl;
                std::cout<<"rotation vector"<<std::endl<<rvecs[0]<<std::endl;
                //std::cout<<"translation vectors"<<tvecs.size()<<" "<<tvecs[0].size()<<std::endl;
                std::cout<<"translation vectors"<<std::endl<<tvecs[0]<<std::endl;
                std::cout<<"Projecting points"<<std::endl;
                cv::projectPoints(court_verts[0], rvecs[0], tvecs[0], cameraMatrix, distCoeffs, court_img_verts[0]);

            }
        //std::cout<<"point"<<FW_SW_L<<std::endl;
        //if(court_img_verts[court_vert_idx]==p2i_zeros){
    //std::cout<<"Click on the front wall line and side wall line left and then right"<<std::endl;
            
            if (court_vert_idx>=nverts){ //so we don't run out of the court_vert_idx loop!
                cv::waitKey(0); //pause so we can look at it
                break;
            }
            std::cout<<court_vert_idx<<" of 22. Click on the "<<vert_descriptions[court_vert_idx]<<" and then press s"<<std::endl;
            
        }
        
        
        //draw circles on all the image points
        for(int i=0;i<nverts;i++){
            cv::circle(frame_corners, court_img_verts[0][i], 2, CV_RGB(0,255,0));
        }
        cv::line(frame_corners,pt,xyloc,CV_RGB(255,255,255));
        imshow("frame callback", frame_corners);
        keyresponse=cv::waitKey(1);
           
           /*FW_SW_L, FW_SW_R; //front wall line and side wall line left and right
           cv::Point2i SL_SW_L, SL_SW_R; //service line and side wall left and right
           cv::Point2i T_SW_L, T_SW_R; //tin and side wall left and right
           cv::Point2i F_SW_L, F_SW_R; //floor and side wall left and right
           cv::Point2i BW_SW_L, BW_SW_R; //back wall line and side wall line left and right
           cv::Point2i BW_F_L, BW_F_R; //back wall and floor left and right
           cv::Point2i BW_HCL, F_HCL; //back wall and half court line and floor and half court line
           cv::Point2i F_SW_SL_L, F_SW_SL_R; //floor, side wall, and short line left and right
           cv::Point2i SB_SL_L, SB_SL_R; //service box and short line left and right
           cv::Point2i SB_SW_L, SB_SW_R; //service box and side wall left and right
           cv::Point2i SB_L, SB_R;*/
    }
    //std::cout<<squash_court;
    std::cout<<"RMS error = "<<rms<<std::endl;
    std::cout<<"camera matrix"<<std::endl<<cameraMatrix<<std::endl;  //this doesn't seem to change
    //std::cout<<"distortion coefficients"<<std::endl<<distCoeffs<<std::endl;
    //std::cout<<"rotation vectors"<<rvecs.size()<<" "<<rvecs[0].size()<<std::endl;
    std::cout<<"rotation vector"<<std::endl<<rvecs[0]<<std::endl;
    //std::cout<<"translation vectors"<<tvecs.size()<<" "<<tvecs[0].size()<<std::endl;
    std::cout<<"translation vectors"<<std::endl<<tvecs[0]<<std::endl;
    //redraw the points and image one last time for inspection
    /*for(int i=0;i<nverts;i++){
        cv::circle(frame_corners, court_img_verts[0][i], 2, CV_RGB(0,255,0));
    }*/
    draw_court(frame_corners);
    imshow("frame callback", frame_corners);
    cv::waitKey(0);
    std::cout<<"Done defining the court"<<std::endl;
    
    
    /* this section lets you define the court by dragging the verticies around, this probably should just replace the click based method above*/
    std::cout<<"Drag the corners you want to move and press c to recalibrate the camera"<<std::endl;
    keyresponse=-1;
    while(keyresponse!='q'){
        frame_raw.copyTo(frame_corners);
        keyresponse=cv::waitKey(1);
        if (keyresponse=='c'){ //if we want to calibrate
            camera_calibration(framesize, 0);
            cv::projectPoints(court_verts[0], rvecs[0], tvecs[0], cameraMatrix, distCoeffs, court_img_verts[0]);
            std::cout<<"RMS error = "<<rms<<std::endl;
            std::cout<<"camera matrix"<<std::endl<<cameraMatrix<<std::endl;  //this doesn't seem to change
            std::cout<<"rotation vector"<<std::endl<<rvecs[0]<<std::endl;
            std::cout<<"translation vectors"<<std::endl<<tvecs[0]<<std::endl;
        }
        if (wasclick==true) { //find the closest point
            //std::cout<<"wasclick is true, finding the closest point"<<std::endl;
            mindist=1.0e10;
            for(int i=0;i<nverts;i++){
                dprod=fabs(pt.x-court_img_verts[0][i].x)+fabs(pt.y-court_img_verts[0][i].y); //the distance doesn't have to be perfect
                if (dprod<mindist) {
                    mindist=dprod; //set the minimum distance
                    minidx=i;
                }
            }
            //std::cout<<"the closest point is "<<minidx<<" with a distance of "<<mindist<<std::endl;
            wasclick=false;
        }
        if (wasrelease==false) {
            court_img_verts[0][minidx]=xyloc;
            //std::cout<<"setting points"<<std::endl;
        }
        draw_court(frame_corners);
        imshow("frame callback", frame_corners);
    }
    cv::destroyWindow("frame callback"); //close the window now that we're done calibrating
}

void squash_court::camera_calibration(cv::Point2i framesize, int npoints)  //should I pass by reference? currently the 2nd argument isn't used
{
    //I want to crop court_verts and court_img_verts down to npoints and use that in the calibration
    //court_verts
    rms=cv::calibrateCamera(court_verts, court_img_verts, framesize, cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_PRINCIPAL_POINT |  CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K1 | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3);
    //CV_CALIB_FIX_ASPECT_RATIO | //we shouldn't be fixing the aspect ratio, it distorted the result
    //fix principal point could be removed, especially when finding the distortion matrix
    //note that if we let the distortions correct then the camera matrix also gets a little crazy
    //I think we can fairly safely assume that the camera is basically distortionless
    //all the above arguments to calibrateCamera mean that it's essentially not optimizing the distortion matrix, only the camera matrix, rotation and translation vectors
}

void squash_court::setup_camera_matrix(cv::Point2i framesize) //should I pass by reference?
{
    double f = 55.0; // focal length in mm (55 is about what it finds, but I don't know if that's right
    double sx = 36.0, sy = 21.0;             // sensor size, full frame is 36x24, but this one seems less square?
    std::cout<<"Setting up the camera matrix vector"<<std::endl;
    cameraMatrix.at<double>(0,2)=static_cast<double>(framesize.x)/2.0; //finish setting the cameraMatrix because calibrateCamera can't do it for me
    cameraMatrix.at<double>(1,2)=static_cast<double>(framesize.y)/2.0;  //finish setting cameraMatrix
    cameraMatrix.at<double>(0,0)=static_cast<double>(framesize.x)*f/sx; //fx
    cameraMatrix.at<double>(1,1)=static_cast<double>(framesize.y)*f/sy; //fy
    std::cout<<"camera matrix"<<std::endl<<cameraMatrix<<std::endl;  //this doesn't seem to change
    /*camera matrix
     [1900.143582407142, 0, 640;
     0, 1829.372842887497, 360;
     0, 0, 1]*/
}

void squash_court::setup_tvec()
{
    //set up the tvec
    std::cout<<"Setting up the translation vector"<<std::endl;
    //think of these translations as happening to the court, not the camera!
    tvecs[0].at<double>(0)=-3.2; //-3.2m in x (centered)
    tvecs[0].at<double>(1)=2.5; //2.5m up (maybe)
    tvecs[0].at<double>(2)=14.0; //14m back (maybe)
    std::cout<<"translation vectors"<<std::endl<<tvecs[0]<<std::endl;
    /* this is an example tvec, which isn't too different than the input, but still isn't correct
     [-3.181589803420535, 2.668127841422967, 13.74342575277559]*/
    
}

void squash_court::setup_rvec()
{
    //set up the rvec
    std::cout<<"Setting up the rotation vector"<<std::endl;
    rvecs[0].at<double>(0)=1.8; //not sure what this is
    rvecs[0].at<double>(1)=0.0; //not sure what this is
    rvecs[0].at<double>(2)=0.0; //not sure what this is
    std::cout<<"rotation vector"<<std::endl<<rvecs[0]<<std::endl;
    /* This is based off of an output rvec which put the court just about in the middle of the screen
     [1.818635848296208, -0.008306380664320337, 0.005670786435876811]*/
}

void squash_court::draw_court(cv::Mat frame)
{
    //init some stuff that we need
    std::ostringstream convert;
    //cv::fontQt("font") fontQT;
    //cvInitFont(font, CV_FONT_HERSHEY_PLAIN, double hscale, double vscale, double shear=0, int thickness=1, int line_type=8 )
    //put circles on each point
    for(int i=0;i<nverts;i++){
        cv::circle(frame, court_img_verts[0][i], 2, CV_RGB(0,255,0));
        convert << i;
        cv::putText(frame, convert.str(), court_img_verts[0][i], CV_FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0));
        convert.str(""); //empty it
        convert.clear(); //empty it (again)
    }
    //draw lines between the circles
    //horizontal (ish) lines
    cv::line(frame,court_img_verts[0][0],court_img_verts[0][1],CV_RGB(0,255,0)); //front wall line
    cv::line(frame,court_img_verts[0][2],court_img_verts[0][3],CV_RGB(0,255,0)); //service line
    cv::line(frame,court_img_verts[0][4],court_img_verts[0][5],CV_RGB(0,255,0)); //tin
    cv::line(frame,court_img_verts[0][6],court_img_verts[0][7],CV_RGB(0,255,0)); //front floor
    cv::line(frame,court_img_verts[0][0],court_img_verts[0][8],CV_RGB(0,255,0)); //left side wall line
    cv::line(frame,court_img_verts[0][1],court_img_verts[0][9],CV_RGB(0,255,0)); //right side wall line
    cv::line(frame,court_img_verts[0][8],court_img_verts[0][9],CV_RGB(0,255,0)); //back wall line
    cv::line(frame,court_img_verts[0][10],court_img_verts[0][12],CV_RGB(0,255,0)); //back left floor
    cv::line(frame,court_img_verts[0][11],court_img_verts[0][12],CV_RGB(0,255,0)); //back right floor
    cv::line(frame,court_img_verts[0][12],court_img_verts[0][13],CV_RGB(0,255,0)); //half court line
    cv::line(frame,court_img_verts[0][14],court_img_verts[0][16],CV_RGB(0,255,0)); //short line service box left
    cv::line(frame,court_img_verts[0][16],court_img_verts[0][13],CV_RGB(0,255,0)); //service box half court line left
    cv::line(frame,court_img_verts[0][17],court_img_verts[0][13],CV_RGB(0,255,0)); //service box half court line right
    cv::line(frame,court_img_verts[0][17],court_img_verts[0][15],CV_RGB(0,255,0)); //short line service box right
    cv::line(frame,court_img_verts[0][18],court_img_verts[0][20],CV_RGB(0,255,0)); //service box horizontal left
    cv::line(frame,court_img_verts[0][19],court_img_verts[0][21],CV_RGB(0,255,0)); //service box horizontal right
    //vertical (ish) lines
    cv::line(frame,court_img_verts[0][0],court_img_verts[0][2],CV_RGB(0,255,0)); //front wall left side top
    cv::line(frame,court_img_verts[0][2],court_img_verts[0][4],CV_RGB(0,255,0)); //front wall left side middle
    cv::line(frame,court_img_verts[0][4],court_img_verts[0][6],CV_RGB(0,255,0)); //front wall left side tin
    cv::line(frame,court_img_verts[0][1],court_img_verts[0][3],CV_RGB(0,255,0)); //front wall right side top
    cv::line(frame,court_img_verts[0][3],court_img_verts[0][5],CV_RGB(0,255,0)); //front wall right side middle
    cv::line(frame,court_img_verts[0][5],court_img_verts[0][7],CV_RGB(0,255,0)); //front wall right side tin
    cv::line(frame,court_img_verts[0][6],court_img_verts[0][14],CV_RGB(0,255,0)); //floor front to short line left
    cv::line(frame,court_img_verts[0][14],court_img_verts[0][18],CV_RGB(0,255,0)); //floor service box side left
    cv::line(frame,court_img_verts[0][18],court_img_verts[0][10],CV_RGB(0,255,0)); //floor service box back left
    cv::line(frame,court_img_verts[0][7],court_img_verts[0][15],CV_RGB(0,255,0)); //floor front to short line right
    cv::line(frame,court_img_verts[0][15],court_img_verts[0][19],CV_RGB(0,255,0)); //floor service box side right
    cv::line(frame,court_img_verts[0][19],court_img_verts[0][11],CV_RGB(0,255,0)); //floor service box back right
    cv::line(frame,court_img_verts[0][13],court_img_verts[0][12],CV_RGB(0,255,0)); //half court line
    cv::line(frame,court_img_verts[0][16],court_img_verts[0][20],CV_RGB(0,255,0)); //service box vertical left
    cv::line(frame,court_img_verts[0][17],court_img_verts[0][21],CV_RGB(0,255,0)); //service box vertical right
    cv::line(frame,court_img_verts[0][8],court_img_verts[0][10],CV_RGB(0,255,0)); //back vertical left
    cv::line(frame,court_img_verts[0][9],court_img_verts[0][11],CV_RGB(0,255,0)); //back vertical right

}


class squash_game: private squash_court
{
protected:
    std::vector<cv::Point2f> player1_img_loc, player2_img_loc, ball_img_loc; //player and ball image locations
    std::vector<cv::Point3f> player1_loc, player2_loc, ball_loc; //player and ball physical locations
    double coef_rest=0.5; //coefficient of restitution for the ball
    
    
public:
    void find_court_background();
    void track_ball();
    void track_player();
    void ball_physics(); //calculate
    
    //now we need to step through the image, track the ball and probably the players too (probably using backgroundsubtractor)
    //then we need to reconstruct the true ball location using SURF and a masked region (with help from a kalman filter probably)
    //probably kalman filters in both image space and true space for both players and the ball! I definitely want to track them all
    //I should probably kalman filter the coefficient of restitution (bouncyness) as well
    //I think gravity can be included in kalman with a control matrix!

    
    
};




//the original callback for mouse events
void CallBackFunc(int event, int x, int y, int flags, void* image)
{
    //cv::Point* xylocptr = (cv::Point*) xyloc;
    //cv::Mat* imageptr = (cv::Mat*) image;
    //cv::Point2i point(x,y);
    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
        pt.x=x;
        pt.y=y;
        //xylocptr->x=x;
        //xylocptr->y=y;
    }
    /*else if  ( event == cv::EVENT_RBUTTONDOWN )
    {
        std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if  ( event == cv::EVENT_MBUTTONDOWN )
    {
        std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }*/
    else if ( event == cv::EVENT_MOUSEMOVE )
    {
        //std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;
        //std::cout << "Original clock location (" << pt.x << ", " << pt.y << ")" << std::endl;
        xyloc.x=x;
        xyloc.y=y;
        //point.x=x;
        //point.y=y;
        //cv::line(*imageptr,pt,point,CV_RGB(255,255,255));
        
    }
    
}

int main(int argc, char* argv[])
{
    cv::Point2i test;
    //set up some variables
    int keyresponse;
    int waittime=1;
    int ispaused=0;
    //int xyloc[2];
    //cv::Point2i xyloc(100,100);
    
    //VideoCapture cap(0); // open the default camera
    if(argc!=2){
        std::cout<<"Usage"<<std::endl<<argv[0]<<" file_to_process"<<std::endl;
    }
    std::cout<<"Opening video file "<<argv[1]<<std::endl;
    cv::VideoCapture cap(argv[1]);
    if(!cap.isOpened()){  // check if we succeeded
        std::cout<<"Sorry, we couldn't read the file"<<std::endl;
        return -1;
    }
    
    cv::Mat frame; //create a frame
    cv::Mat frame_raw, frame_court_define;
    cv::namedWindow("frame",1); //create the window
    //cv::setMouseCallback("frame", CallBackFunc, (void*)&xyloc); //create the mouse callback for point passback
    cv::setMouseCallback("frame",CallBackFunc,(void*) &frame);
    
    
    /* define the squash court */
    squash_court court; //initialize the squash court
    //cap >>frame;
    //court.get_corners(frame);
    
    
    while(cap.isOpened()){
        //some basic movie controls
        //std::cout<<"click location "<<xyloc<<std::endl;
        keyresponse=cv::waitKey(1); //no keyresponse is -1
        waittime=1;
        if (keyresponse=='q'){ //q
            std::cout<<"Exiting the initial loop"<<std::endl;
            break;
        }else if (keyresponse=='p' || keyresponse==32){ //pause (p or space bar)
            //waittime=0; //wait forever next time
            if (ispaused==0){
                ispaused=1;
            }else{
                ispaused=0;
            
            }
            
        }else if (keyresponse=='d'){ //d for define squash court
            std::cout<<"Setting the squash court define frame"<<std::endl;
            frame_raw.copyTo(frame_court_define);
            //court.get_corners(frame);
        }
        //std::cout<<keyresponse<<std::endl;
        //std::cout<<cv::waitKey(0)<<std::endl;
        
        //cv::Mat frame;
        if (ispaused==0){
            //cap >> frame_raw; // get a new frame from camera
            cap.read(frame_raw);
        }
        //frame=frame_raw;
        frame_raw.copyTo(frame);
        
        //cvtColor(frame, edges, COLOR_BGR2GRAY);
        //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        //Canny(edges, edges, 0, 30, 3);
        cv::line(frame,pt,xyloc,CV_RGB(255,255,255));
        imshow("frame", frame);
        
        //if(waitKey(1) >= 0)
        //    break;
    }
    
    //define the squash court
    court.get_corners(frame_court_define);
    //std::cout<<court;
    
    //https://github.com/Itseez/opencv/blob/master/modules/video/src/kalman.cpp
    //https://courses.cs.washington.edu/courses/cse466/11au/calendar/14-StateEstimation-posted.pdf
    //now we need to step through the image, track the ball and probably the players too (probably using backgroundsubtractor)
    //then we need to reconstruct the true ball location using SURF and a masked region (with help from a kalman filter probably)
    //probably kalman filters in both image space and true space for both players and the ball! I definitely want to track them all
    std::cout<<"Rewinding to the beginning and playing again"<<std::endl;
    keyresponse=-1;
    cap.set(CV_CAP_PROP_POS_FRAMES, 0); //rewind back to the beginning
    cv::SURF surfdet(400,4,2,true,true); //don't bother with the orientation of the features
    cv::Mat frame_prev,frame_output;
    cv::vector<cv::KeyPoint> keypoints_prev,keypoints;
    cv::Mat descriptor_prev,descriptor;
    cv::vector<cv::DMatch> surfmatch,surfmatch_init;
    cv::BFMatcher bfmatch(cv::NORM_L1,1); //brute force matcher
    cv::vector<char> matchmask;
    //Ptr<GenericDescriptorMatcher> descriptorMatcher = GenericDescriptorMatcher::create(alg_name, params_filename);
    
    //create a mask
    cv::Mat mask=cv::Mat::zeros(frame.size(),CV_8U);
    cv::Mat roi(mask,cv::Rect(400,200,700,500)); //the order is x,y
    roi=cv::Scalar(255,255,255);
    
    
    while(cap.isOpened() && keyresponse!='q'){
        cap.read(frame); //current frame is 1 after this
        //std::cout<<"frame "<<cap.get(CV_CAP_PROP_POS_FRAMES)<<std::endl;
        if (cap.get(CV_CAP_PROP_POS_FRAMES)>1) { //copy previous data so we can match adjacent images
            keypoints_prev=keypoints; //copy them over
            descriptor.copyTo(descriptor_prev); //copy
            frame.copyTo(frame_prev); //copy
        }
        surfdet(frame,mask,keypoints,descriptor); //detect points in current image
        bfmatch.match(descriptor_prev, descriptor, surfmatch); //match points to previous image
        
        //clean matches
        /*matchmask.clear();
        for (int i=0; i<surfmatch_init.size(); i++) {
            if (surfmatch_init[i].distance>=0.0) { //if it's a bad match
                //surfmatch.push_back(surfmatch_init[i]);
                //keypoints_final.push_back(keypoints[i]);
                //keypoints_prev_final.push_back(keypoints_prev[surfmatch_init[i].trainIdx]);
                matchmask.push_back(0);
            }else{
                matchmask.push_back(1);
            }
            
        }*/
        //surfmatch_init.clear();//clear everything from surfmatch_init
        
        if (cap.get(CV_CAP_PROP_POS_FRAMES)>1) {
            //std::cout<<keypoints[0].pt<<" "<<keypoints_prev[0].pt<<" "<<surfmatch[0].distance<<std::endl;
            std::cout<<"number of matches= "<<surfmatch.size()<<std::endl;
            cv::drawMatches(frame_prev, keypoints_prev, frame, keypoints, surfmatch, frame_output,cv::Scalar::all(-1),cv::Scalar::all(-1),matchmask);
            imshow("matches",frame_output);
        }
        
        imshow("frame", frame);
        
        keyresponse=cv::waitKey(1); //no keyresponse is -1
    }
    for (int i=0;i<surfmatch.size();i++){
        std::cout<<"match "<<i<<" "<<surfmatch[i].distance<<std::endl;
    }
    
    
    //exit the program
    std::cout<<"Goodbye"<<std::endl;
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
