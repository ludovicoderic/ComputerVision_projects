#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;
 
int main (int argc, char* argv[])
{
    if(argc!=3){
        cerr << "Syntax: " << argv[0] << "<image>" << "<dest>" << endl;
        exit(-1);
    }

    Mat img = imread(argv[1]);
    HOGDescriptor hog;
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
 
    while (true){
       
        vector<Rect> found, found_filtered;
        hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.05, 2);
        size_t i, j;
        for (i=0; i<found.size(); i++) 
        {
            Rect r = found[i];
            for (j=0; j<found.size(); j++) {
                if (j!=i && (r & found[j]) == r)
                    break;
            }
            if (j== found.size())
                found_filtered.push_back(r);
        }
 
        for (i=0; i<found_filtered.size(); i++) 
        {
            Rect r = found_filtered[i];
            r.x += cvRound(r.width*0.1);
		    r.width = cvRound(r.width*0.8);
		    r.y += cvRound(r.height*0.07);
		    r.height = cvRound(r.height*0.8);
		    rectangle(img, r.tl(), r.br(), Scalar(0,255,0), 3);        
        }
 
        imshow("opencv", img);
        if  ( waitKey(10) >= 0 ) 
            break; 
       
    }
    imwrite(argv[2],img);
    return 0;
}