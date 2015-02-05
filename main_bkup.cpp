#include <iostream>
#include <opencv2/opencv.hpp>
//#include <opencv2/core/core_c.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/objdetect/objdetect.hpp>

//using namespace cv;

cv::Point2i pt(-1,-1);


/*global start and end x and y queues, or maybe two queue of Point2i?
 I need to then draw this on the next image in the series, or maybe all of them?
 Then we move on to the processing section*/

void CallBackFunc(int event, int x, int y, int flags, void* image)
{
    //cv::Point* xylocptr = (cv::Point*) xyloc;
    cv::Mat* imageptr = (cv::Mat*) image;
    cv::Point2i point(x,y);
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
        point.x=x;
        point.y=y;
        cv::line(*imageptr,pt,point,CV_RGB(255,255,255));
        
    }
    
}

int main(int argc, char* argv[])
{
    //set up some variables
    int keyresponse;
    int waittime=1;
    //int xyloc[2];
    cv::Point2i xyloc(100,100);
    
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
    cv::namedWindow("frame",1); //create the window
    //cv::setMouseCallback("frame", CallBackFunc, (void*)&xyloc); //create the mouse callback for point passback
    cv::setMouseCallback("frame",CallBackFunc,(void*) &frame);
    
    while(cap.isOpened()){
        //some basic movie controls
        std::cout<<"click location "<<xyloc<<std::endl;
        keyresponse=cv::waitKey(waittime);
        waittime=1;
        if (keyresponse==113){ //q
            std::cout<<"Goodbye"<<std::endl;
            break;
        }else if (keyresponse==112 || keyresponse==32){ //pause (p or space bar)
            waittime=0; //wait forever next time
        }
        //std::cout<<cv::waitKey(0)<<std::endl;
        
        //cv::Mat frame;
        cap >> frame; // get a new frame from camera
        //cvtColor(frame, edges, COLOR_BGR2GRAY);
        //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        //Canny(edges, edges, 0, 30, 3);
        cv::line(frame,pt,xyloc,CV_RGB(255,255,255));
        imshow("frame", frame);
        
        //if(waitKey(1) >= 0)
        //    break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
