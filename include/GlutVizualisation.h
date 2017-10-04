#pragma once
#define _USE_MATH_DEFINES

#include <freeglut.h>
#include <gl.h>
#include <glu.h>


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <string>
#include <sstream>
#include <time.h>

#include "bitmap.h"
#include "Geometry.h"
#include "DrawFunc.h"
#include "MapHandler.h"
#include "Simulator.h"
#include "OptimalTrajectory.h"


using namespace std;

/****************************************************************/
/*** Definitions and structs  ***********************************/
/****************************************************************/

#define NTEXTURE 35     // The number of textures (in the models-textures dir)
#define SCENE_SCALE 0.2

// Type definitions for vertex-coordinates, normals, texture-coordinates,
// and triangles (via the indices of 3 vertices).
typedef GLfloat vertex[3];
typedef GLfloat normal[3];
typedef GLfloat texCoord[2];
typedef GLint vertexIndex;
typedef vertexIndex triangle[3];

// A type for a 2D texture, with height and width in pixels
typedef struct
{
int height;
int width;
GLubyte *rgbData;        // Array of bytes with the colour data for the texture
} texture;

// A type for holding polygons
typedef struct
{
Point2D p1;
Point2D p2;
Point2D p3;
Point2D p4;
} Quad;

// A type for the camera
typedef struct
{
GLfloat x ,y, z;       // Position of camera
GLfloat theta, alpha;  // Rotation of camera (around x and y)
} camera;

// A type for a scene object
typedef struct
{
double x, y, z;            // Current position if object
GLfloat alpha, beta, gamma; // Current rotation of object (around x, y, z)
GLfloat scale;              // Current scale of object
GLuint list_no;             // DisplayList number
GLuint texture;             // Current texture of object
GLfloat texturescale;       // Current texturescale of  object

} SceneObject;

// A type for the ground plane
typedef struct
{
GLuint displayList;    // Display list for ground
GLuint texture;   // Current texture on ground
GLfloat scale;  // Current scale on ground texture
vector<Quad> grassEdge;
} GroundPlane;

// A definition of states
typedef enum { MOVE_CAM,TOPVIEW, FOLLOW, SETWP, PLANNERMODE, SCALE_GROUND, EXIT
} statemachine;



/****************************************************************/
/*** Function declarations    ***********************************/
/****************************************************************/

texture* loadTexture(char *fileName);
void getTexture(int i);
void makeMenu();

void initGL(); // init
void init_lights(); // init lights
void init_scene(); //init scene

void init_ground(GLfloat size, GLfloat delta_text, int rect_per_text, int groundtexture); // init ground plane
void init_DisplayList(int id); // init a display list

void display(); // display callback function
void update_camera(); // function for updating the camera

// function to get the intersection between a ray and a plane
bool getIntersection(int x, int y, GLfloat intersection[], GLfloat pointOnPlane[], GLfloat planeNormal[]);

void reshape_cb(int w, int h); // reshape window callback function
void mouse_cb(int button, int state, int x, int y); // mouse pressing callback function
void mouse_press_tracking_cb(int x, int y); // mouse pressing and moving callback funstion
void mouse_wheel_cb(int wheel, int direction, int x, int y); // mouse wheel callback function
void keyboard_cb(unsigned char key, int x, int y); // keyboard press callback funtion
void menu_cb(int id);  // normal change state menu callback
void menu1_cb(int id); // change ground texture menu callback
void printDirections(char* directions);




/****************************************************************/
/*** A class for handling vizualisation in OpenGL      **********/
/****************************************************************/
class glutVizualisation
{
    /****************************************************************/
    /*** Private varibles ********************************************/
    /****************************************************************/

    public:

        /****************************************************************/
        /*** Constructor Destructor  ************************************/
        /****************************************************************/
        glutVizualisation ();
        ~glutVizualisation();

        /****************************************************************/
        /*** Public methods *********************************************/
        /****************************************************************/

        void initVizualisation();
        void reDisplay();
        void updateObjects();
        void setSimulationModel(Model* simulationModel);
        void setObstacleMap(ObstacleMap* obstacleMapIn);
        void setDrawOffset(double offset_x, double offset_y);
		void setOptimalTrajectory(OptimalTrajectory *optimalTrajectory_in);
        void setEgoState(State& state_in);
        bool getRunState();

    private:

};

