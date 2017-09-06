#include <csv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;

/**
 * @brief main
 * @param argc
 * @param argv
 *
 * in the csv, specify
 *  fx fy cx cy k1 k2 p1 p2
 *
 *  k1 k2 > distorsion parameters (potentially up to 6 parameters, here 2 used)
 *  p1 p2 > tangent distortion parameters
 *
 * http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
 *
 *
 * @return
 */
int main(int argc, char* argv[])
{

    Mat input, undistorted;

    double fx = 1507.24472;
    double fy = 1507.24472;
    double cx = 610.076118;
    double cy = 518.058638;
    double k1 = -0.252027;
    double k2 = -0.006313;
    double p1 = -0.006733;
    double p2 = -0.000684;


    if (argc == 3)
    {
        try
        {
            cout << "Input Image: " << argv[1] << "\t";
            input = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);   // Read the file, color or not

            io::CSVReader<8, io::trim_chars<>, io::no_quote_escape<';'> > settings(argv[2]);
            settings.read_row(fx, fy, cx, cy, k1, k2, p1, p2);

            Mat cameramatrix = (Mat1d(3, 3) << fx, 0, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f);
            Mat distortionCoefficients = (Mat1d(1, 4) << k1, k2, p1, p2);

            //cout << "CameraMatrix\n" << cameramatrix << endl;
            //cout << "distortionCoefficients\n" << distortionCoefficients << endl;

            if (!input.data)
            {
                cout <<  "Could not open or find the image" << endl ;
                return -1;
            }

            undistort(input, undistorted, cameramatrix, distortionCoefficients);

            //namedWindow("Input Image", WINDOW_AUTOSIZE );
            //imshow("Input Image", input );
            //
            //namedWindow("Undistorted Image", WINDOW_AUTOSIZE );
            //imshow("Undistorted Image", undistorted );

            string fullname = argv[1];
            size_t lastindex = fullname.find_last_of(".");
            string rawname = fullname.substr(0, lastindex);

            imwrite(rawname + "_calibrated.png", undistorted);
            cout << "Output Image: " << rawname + "_calibrated.png" << endl;

            waitKey(0);
            return 0;

        }
        catch (exception& e)
        {
            cout << e.what() << endl;
            return -1;
        }
    }
    else
    {
        cout << "Undistort single image\nUsage: rosrun republish_usad2 republish_usad2_undistortImage inputimage settings.csv" << endl;
    }
}
