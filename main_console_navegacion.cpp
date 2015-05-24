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

using namespace std;
using namespace std::chrono;
using namespace cv;

double global_to_relative(double x,double y,double dy, double teta);
std::array<double, 2> coords_Robot, coords_Path, coordsR_Robot, coordsR_Path, teta;//global coordinates

int main(int argc, const char ** argv)
{
    constexpr unsigned int ndims = 2; // Setting two dimensions.
    constexpr unsigned int ndims3 = 3; // Setting three dimensions.




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
    //GridPoints::selectMapPoints(grid, coords_init, coords_goal);
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
//    GridPlotter::plotArrivalTimes(grid);
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
//    GridPlotter::plotMapPath(grid,path);

    bool navegacion=true;
    int speed= 100; // velocidad total lineal del robot
    teta = 90;//giro del sistema de referencia 90 grados al principio
    double distance = 0;//distancia entre el robot y el punto del path
    int distance_wheel = 10; //separación entre las ruedas en cm
    int wheel_radius = 5; //radio de la rueda en cm

    while(navegacion){

        global_to_relative(coords_Path[0], coords_Path[1], coords_Robot[0], coords_Robot[1], teta);//paso a coordenadas globales a relativas al robot
        distance = coordsR_Path[0]*coordsR_Path[0] + coordsR_Path[1]*coordsR_Path[1];
        distance = sqrt(distance);//distancia entre el robot y el punto del path
        teta = atan2(coordsR_Path[1], coordsR_Path[0]);//cálculo del ángulo del robot con el punto del a trayectoria
        double turning_speed = 10 * teta; // parametro de controlador P

        R_Wheel_Speed = (speed + distance_wheel*turning_speed );//velocidad lineal de las ruedas
        L_Wheel_Speed = (speed - distance_wheel*turning_speed );

        Omega_R = R_Wheel_Speed / wheel_radius;
        Omega_L = L_Wheel_Speed / wheel_radius;

        //calculo de posición mediante odometria, desde el eje de referencia global
        coords_Robot[1] = (-(R_Wheel_Speed+ L_Wheel_Speed)/2)*sin(teta);
        coords_Robot[0] = (-(R_Wheel_Speed+ L_Wheel_Speed)/2)*cos(teta);
        teta = teta + ((R_Wheel_Speed + L_Wheel_Speed)/distance_wheel);

        //cambio del punto de la trayecotira si está lo suficientemente cerca

        waitKey(100);
    }


    return 0;
}


void global_to_relative(double x,double y, double dx, double dy, double teta){

    coordsR_Path = cos(teta)*x - sin(teta)*y + dy; //suma de la coordenada global actual de robot
    coordsR_Path = sin(teta)*x + cos(teta)*y - dx;
}
