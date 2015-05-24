/* n-dimensional Fast Marching example with the main functions used */

#include <iostream>
#include <cmath>
#include <math.h>
#include <chrono>
#include <array>
#include <string>


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

void global_to_relative(double x,double y, double dx, double dy, double teta);
std::array<double, 2> coords_Robot, coords_Path, coordsR_Robot, coordsR_Path;//global
double teta;

int main(int argc, const char ** argv)
{

    std::array<unsigned int, 2> coords_init, coords_goal;

    coords_init[0] = 15;
    coords_init[1] = 100;

    coords_Robot[0] = coords_init[0];
    coords_Robot[1] = coords_init[1];


    coords_goal[0] = 235;
    coords_goal[1] =100;

    coords_Path[0] = 18;
    coords_Path[1]= 102.725;

    bool navegacion=true;

    int speed= 100; // velocidad total lineal del robot
    teta = 3.1415/2;//giro del sistema de referencia 90 grados al principio
    double distance = 0;//distancia entre el robot y el punto del path
    int distance_wheel = 10; //separación entre las ruedas en cm
    int wheel_radius = 5; //radio de la rueda en cm

    while(navegacion){

        global_to_relative(coords_Path[0], coords_Path[1], coords_Robot[0], coords_Robot[1], teta);//paso a coordenadas globales a relativas al robot
        cout << "Coord Path x:" << coordsR_Path[0] << "       Coord Path y:"<< coordsR_Path[1] << endl;

        distance = coordsR_Path[0]*coordsR_Path[0] + coordsR_Path[1]*coordsR_Path[1];
        distance = sqrt(distance);//distancia entre el robot y el punto del path
        cout << "Distance:" <<distance << endl;

/*

        teta = atan2(coordsR_Path[1], coordsR_Path[0]);//cálculo del ángulo del robot con el punto del a trayectoria
        double turning_speed = 10 * teta; // parametro de controlador P

        R_Wheel_Speed = (speed + distance_wheel*turning_speed );//velocidad lineal de las ruedas
        L_Wheel_Speed = (speed - distance_wheel*turning_speed );

        Omega_R = R_Wheel_Speed / wheel_radius;
        Omega_L = L_Wheel_Speed / wheel_radius;

        //calculo de posición mediante odometria, desde el eje de referencia global
        coords_Robot[1] = (-(R_Wheel_Speed+ L_Wheel_Speed)/2)*sin(teta);
        coords_Robot[0] = (-(R_Wheel_Speed+ L_Wheel_Speed)/2)*cos(teta);
        teta = teta + ((R_Wheel_Speed + L_Wheel_Speed)/distance_wheel);*/

         navegacion = false;
    }


    return 0;
}


void global_to_relative(double x,double y, double dx, double dy, double teta){

    coordsR_Path[0] = cos(teta)*x - sin(teta)*y + dy; //suma de la coordenada global actual de robot
    coordsR_Path[1] = sin(teta)*x + cos(teta)*y - dx;
}

