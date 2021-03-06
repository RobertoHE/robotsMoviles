/* n-dimensional Fast Marching example with the main functions used */

#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sstream>
#include <time.h>

#include "fmm/fmdata/fmcell.h"
#include "ndgridmap/ndgridmap.hpp"
#include "console/console.h"
#include "fmm/fmm.hpp"
#include "fm2/fm2.hpp"
#include "fm2/fm2star.hpp"
#include "fmm/fmdata/fmfibheap.hpp"
#include "fmm/fmdata/fmpriorityqueue.hpp"
#include "fmm/fmdata/fmdaryheap.hpp"
#include "fmm/fmdata/fmdaryheap.hpp"
#include "io/maploader.hpp"
#include "io/gridplotter.hpp"
#include "io/gridwriter.hpp"
#include "io/gridpoints.hpp"
#include "gradientdescent/gradientdescent.hpp"

#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <mqueue.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include "/home/pi/repos/UUGear/RaspberryPi/src/UUGear.h"

using namespace std;
using namespace std::chrono;
using namespace cv;


void global_to_relative(double x, double y, double dx, double dy, double alfa, double &rx, double &yr);
void compareR(double &w, int &v);
void compareL(double &w, int &v);
int R_Servo=11;
int L_Servo=12;

int main(int argc, const char ** argv)
{
    constexpr unsigned int ndims = 2; // Setting two dimensions.
    constexpr unsigned int ndims3 = 3; // Setting three dimensions.

    	setupUUGear();
        setShowLogs(1);

        /* replace the device id with yours (listed by lsuu) */
        UUGearDevice dev = attachUUGearDevice ("UUGear-Arduino-6746-1908");
	cout << dev.fd << endl;
 	if (dev.fd == -1)
            {
            cout << "UUGEAR Fail" << endl;
            return 2;
        }

	attachServo(&dev, R_Servo);
        attachServo(&dev, L_Servo);


    time_point<std::chrono::steady_clock> start, end; // Time measuring.
    double time_elapsed;

    console::info("Parsing input arguments.");
    string filename1, filename2, filename_vels;
    console::parseArguments(argc,argv, "-map1", filename1);
    console::parseArguments(argc,argv, "-map2", filename2);
    console::parseArguments(argc,argv, "-vel", filename_vels);



    console::info("OpenCV Transform...");
    // Load the image
    Mat src = imread(filename2);
    // Check if everything was fine
    if (!src.data)
        return -1;
    //imshow("Source Image", src);

    //Binary threshold
    Mat bw;
    cvtColor(src, bw, CV_BGR2GRAY);
    threshold(bw, bw, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    //imshow("Binary Image", bw);

    // Dilate a bit the dist image
    Mat kernel1 = Mat::ones(3, 3, CV_8UC1);
    for(int i=0;i<5;i++)
        erode(bw, bw, kernel1);
    //imshow("dilate", bw);

    // Perform the distance transform algorithm
    Mat dist;
    distanceTransform(bw, dist, CV_DIST_C, 5);

    // Normalize the distance image for range = {0.0, 1.0}
    // so we can visualize and threshold it
    normalize(dist, dist, 0, 255, NORM_MINMAX);
    //imshow("Distance Transform Image", dist);
    imwrite("mapdis.jpg",dist);

    //LUT
    Mat save(dist.size(),CV_8UC1);
    dist.convertTo(dist,CV_8UC1);
    convertScaleAbs(dist,dist);

    Mat lookUpTable(1, 256, CV_8UC1);
    uchar* p = lookUpTable.data;
    for( int i = 0; i < 256; ++i){
    //        p[i] = 255*log(i+1)/log(256);
    //        p[i] = sqrt(255*i);
            p[i] = cbrt(255*255*i);
    }
    LUT(dist,lookUpTable,save);
    //imshow("save",save);

    //save file
    imwrite( "map.jpg", save);

    //waitKey(0);
//    waitKey(100);



    console::info("Creating grid from image and testing Fast Marching Method..");
    nDGridMap<FMCell, ndims> grid;
    MapLoader::loadMapFromImg("map.jpg", grid);

    std::array<unsigned int, ndims> coords_init, coords_goal;

    coords_init[0] = 15;
    coords_init[1] = 100;

    coords_goal[0] = 235;
    coords_goal[1] =100;


    vector<unsigned int> init_points;
    unsigned int idx, goal;
    grid.coord2idx(coords_init, idx);
    init_points.push_back(idx);
    grid.coord2idx(coords_goal, goal);

    FMM< nDGridMap<FMCell, ndims> > fmm;
    fmm.setEnvironment(&grid);
    fmm.setInitialAndGoalPoints(init_points, goal);
    fmm.compute();
        cout << "\tElapsed FM time: " << fmm.getTime() << " ms" << endl;
    console::info("Plotting the results and saving into test_fm.txt");
  //  GridPlotter::plotArrivalTimes(grid);
    GridWriter::saveGridValues("test_fm.txt", grid);

    console::info("Computing gradient descent ");
    typedef typename std::vector< std::array<double, ndims> > Path; // A bit of short-hand.

    Path path;
    std::vector <double> path_velocity; // Velocities profile

        start = steady_clock::now();
    GradientDescent< nDGridMap<FMCell, ndims> > grad;
    grad.apply(grid,goal,path,path_velocity);
        end = steady_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed gradient descent time: " << time_elapsed << " ms" << endl;
    GridWriter::savePath("test_path.txt", grid, path);
    GridWriter::savePathVelocity("path_velocity.txt", grid, path, path_velocity);
  //  GridPlotter::plotMapPath(grid,path);

    //open path file
    ifstream file("test_path.txt");
    string str;
    float x,y;
    vector<pair<float,float>> path_vector;
    //remove 4 first lines
    for(int i =0 ;i < 4; i++)
        getline(file, str);
    //store in a vector
    while(file >> x >> y){
        pair<float,float> pos;
        pos.first= x;
        pos.second = y;
        //cout << pos.first << ", " << pos.second << endl;
        path_vector.push_back(pos);
    }

    double xpath, ypath, xrobot, yrobot, xrelative, yrelative;

    int speed = 6; // velocidad total lineal del robot
    double distance; //distancia entre el robot y el punto del path
    double distance_threshold = 2.0;
    float distance_wheel = 8.5; //separación entre las ruedas en cm
    float wheel_radius = 3; //radio de la rueda en cm
    double teta;
    float basetime = 500.0; //ms

    cout << "------" << endl;

    //inicialize robot position
    xrobot =path_vector.back().first;
    yrobot =path_vector.back().second;
    double alfa = 0;

    int vel_r, vel_l;


    //read the path from the vector
    for (vector<pair<float, float> >::iterator i = path_vector.end(); i != path_vector.begin(); i--) {
        pair <float,float> actual_point;
        actual_point = make_pair(i->first,i->second);
        xpath=actual_point.first;
        ypath=actual_point.second;
cout << "----" << endl;
cout << "Coord Path x:" << xpath << "       Coord Path y:"<< ypath << endl;

        global_to_relative(xpath, ypath, xrobot, yrobot, alfa, xrelative, yrelative);//paso a coordenadas globales a relativas al robot
        cout << "Coord Path x:" << xrelative << "       Coord Path y:"<< yrelative << endl;

        distance = sqrt(xrelative*xrelative+yrelative*yrelative);//distancia entre el robot y el punto del path
        cout << "Distance:" <<distance << endl;

        if(distance <= distance_threshold){//elegir el proximo punto de la trayectoria
            continue;
        }

        teta = atan2(yrelative, xrelative);//cálculo del ángulo del robot con el punto del a trayectoria
        cout <<"Angulo: "<< teta * 180 / 3.1415 << endl;

        double turning_speed = 0.25 * teta;//en radianes parámetro configurable del controlador P

        double R_Wheel_Speed = (speed + distance_wheel*turning_speed );//velocidad lineal de las ruedas
        double L_Wheel_Speed = (speed - distance_wheel*turning_speed );
        cout << "Rueda derecha: " << R_Wheel_Speed << "     Rueda izquierda: " << L_Wheel_Speed << endl;
        double Omega_R = R_Wheel_Speed / wheel_radius;
        double Omega_L = L_Wheel_Speed / wheel_radius;
        cout << "Rueda derecha: " << Omega_R << "     Rueda izquierda: " << Omega_L << endl;

        // Adecuar velocidad

        compareR(Omega_R,vel_r);
        compareL(Omega_L,vel_l);
	
	//Mover los servos

        writeServo(&dev, R_Servo, vel_r);
        writeServo(&dev, L_Servo, vel_l );

    cout << "pulse:" <<  vel_r << ", " << vel_l  << endl;
	R_Wheel_Speed = Omega_R*wheel_radius;
	L_Wheel_Speed = Omega_L*wheel_radius;
    cout << "w: "<< Omega_R << ", " << Omega_L << endl;
cout << "v: " << R_Wheel_Speed << ", " << L_Wheel_Speed << endl;

        //calculo de posición mediante odometria, desde el eje de referencia global
        xrobot = xrobot + (((R_Wheel_Speed + L_Wheel_Speed)/2.0)*cos(teta))*basetime/1000.0;// e = v*t
        yrobot = yrobot + (((R_Wheel_Speed + L_Wheel_Speed)/2.0)*sin(teta))*basetime/1000.0;
        double alfa = alfa +  ((R_Wheel_Speed + L_Wheel_Speed)/distance_wheel)*basetime/1000.0;
        cout << "Coordenadas movidas real  x: " << xrobot << "   y:" << yrobot << endl;
        cout << "Teta: "<<  teta * (180 / 3.1415) << endl;// proximo angulo a desplazar el eje de referencia


        
        usleep(basetime*1000);



    }
    detachServo(&dev, R_Servo);
    detachServo(&dev, L_Servo);
    detachUUGearDevice (&dev);
//	usleep(100000);

    cleanupUUGear();
//	usleep(100000);
    cout << "FIN" << endl;

    return 0;
}

void global_to_relative(double x, double y, double dx, double dy, double alfa, double &xr, double &yr){

    xr = cos(alfa)*x - sin(alfa)*y - dx; //dx y dy es el desplazamiento del robot sobre el eje de coordenadas global
    yr = sin(alfa)*x + cos(alfa)*y - dy;

    /*
    coordsR_Path[0] = x - dx;
    coordsR_Path[1] = y - dy;
    */
    /*
    coordsR_Path[0] = cos(alfa)*x - sin(alfa)*y + dy; //dx y dy es el desplazamiento del robot sobre el global
    coordsR_Path[1] = sin(alfa)*x + cos(alfa)*y - dx;
    */


}
void compareL(double &w, int &v){
    float vector_w[]={
        0,
        0.5351946599,
        1.064946662,
        1.266771231,
        1.72614981,
        2.327105669,
        2.963766654,
        3.452299619,
        3.878509449,
        4.30355158,
        4.553032831,
        4.908738521,
        5.150151891,
        5.416539058,
        5.711986643,
        5.711986643,
        5.817764173,
        5.927533309,
        6.04152433};

    int vector_v[]={
        82,
        83,
        84,
        85,
        86,
        87,
        88,
        89,
        90,
        91,
        92,
        93,
        94,
        95,
        96,
        97,
        98,
        110,
        120
    };

    float error=1000000;
    int iter;

    for(int i=0;i<(sizeof(vector_w)/4);i++){
        if(error > ((vector_w[i]-w)*(vector_w[i]-w))){
                iter = i;
                error = ((vector_w[i]-w)*(vector_w[i]-w));
        }
    }
    w = vector_w[iter];
    v = vector_v[iter];
}

void compareR(double &w, int &v){
    float vector_w[]={
        0,
        0.9941748904,
        1.56298142,
        1.795195802,
        2.309994598,
        2.908882087,
        3.452299619,
        3.926990817,
        4.487989505,
        4.759988869,
        5.067084925,
        5.324733311,
        5.711986643,
        5.817764173,
        5.817764173,
        5.927533309,
        6.159985595
    };

    int vector_v[]={
        90,
        89,
        88,
        87,
        86,
        85,
        84,
        83,
        82,
        81,
        80,
        79,
        78,
        77,
        76,
        75,
        64
    };

    float error=1000000;
    int iter;

    for(int i=0;i<(sizeof(vector_w)/sizeof(*vector_w));i++){
	 if(error > ((vector_w[i]-w)*(vector_w[i]-w)) ){
                iter = i;
                error = ((vector_w[i]-w)*(vector_w[i]-w));
        }
    }
    w = vector_w[iter];
    v = vector_v[iter];
}
