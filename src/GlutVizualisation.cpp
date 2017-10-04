#include "GlutVizualisation.h"


glutVizualisation::glutVizualisation()
{

}

void glutVizualisation::initVizualisation()
{
    initGL();
}

void glutVizualisation::reDisplay()
{
    updateObjects();
    glutMainLoopEvent();
    glutPostRedisplay();
}



/****************************************************************/
/*** C stuff for glut         ***********************************/
/****************************************************************/

/****************************************************************/
/*** Global variables         ***********************************/
/****************************************************************/
int WIDTH = 1600;
int HEIGHT = 1116;

double drawOffset_x;
double drawOffset_y;

double setWP_x;
double setWP_y;
double setWP_heading;

double mouse_x;
double mouse_y;

double simulationTime		= 0;
int	   numberOfItterations	= 0;

SceneObject sceneObjs[2];			// An array with details of the objects in a scene
int nObjects = 0;                   // How many objects there are in the scene currently.
int cObject = -1;                   // The current selected object

texture* textures[NTEXTURE];        // An array of texture pointers

camera cam;                         // The camera
GroundPlane ground;                 // The ground plane
statemachine state = MOVE_CAM;      // A statemachine with initial value MOVE_CAM
GLuint DisplayLists[20];            // An array with display lists

Model               *egoModel;      // Dynamic model of the car
State               *egoState;
ObstacleMap         *obstacleMap;   // Pointer to the obstacle map
OptimalTrajectory   *optimalTrajectory; // Pointer to optimal trajectory 

int mouse_state;                    // Variable to store which mouse button have been pressed
bool mouse_reset = true;            // Variable to keep track of when a mouse button have been released
// with initial value true

char dataDir[200];                  // Stores the directory name for the meshes and textures.
char *dirDefault1 = "..\\data\\models-textures";

char *textureMenuEntries[NTEXTURE] = {
    "1 Plain", "2 Rust", "3 Concrete", "4 Carpet", "5 Beach Sand",
    "6 Rocky", "7 Brick", "8 Water", "9 Paper", "10 Marble",
    "11 Wood", "12 Scales", "13 Fur", "14 Denim", "15 Hessian",
    "16 Orange Peel", "17 Ice Crystals", "18 Grass", "19 Corrugated Iron", "20 Styrofoam",
    "21 Bubble Wrap", "22 Leather", "23 Camouflage", "24 Asphalt", "25 Scratched Ice",
    "26 Rattan", "27 Snow", "28 Dry Mud", "29 Old Concrete", "30 Leopard Skin" };

enum WPstateEnum{SET_POINT, SET_HEADING, FINISHED};

WPstateEnum setWPstate;

bool RUN = false;
bool DRAW_TREE = true;
bool DRAW_SAFE = false;
bool DRAW_TRAJ = false;
bool DRAW_CONTROL = false;
bool FOLLOW_CAR = false;


/****************************************************************/
/*** Function implementations ***********************************/
/****************************************************************/
void initGL()
{
    strcpy(dataDir, dirDefault1);

    /****************************************************************/
    /*** Setup Open GL stuff                    *********************/
    /****************************************************************/
    int argc = 1;
    char *argv[1] = {(char*)"Init"};
    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(WIDTH,HEIGHT);          // set window size
    glutInitWindowPosition(30,30);        // set window position
    glutCreateWindow("OpenGL - Awesome plot thingy");

    GLint w = (GLint)glutGet(GLUT_WINDOW_WIDTH);      // get window width
    GLint h = (GLint)glutGet(GLUT_WINDOW_HEIGHT);     // get window height

    glViewport(0.0f,0.0f,w,h);                                  // set the viewport

    glMatrixMode(GL_PROJECTION);                          // in matrix mode "projection":
    glLoadIdentity();                                   //    load indentity matrix
    gluPerspective(45.0f, w/h, 0.1f, 10000000.0f);           //    set gluPerspective
    glMatrixMode(GL_MODELVIEW);

    glEnable(GL_NORMALIZE);                               // enable auto normalisation
    glEnable(GL_TEXTURE_2D);                              // enable 2D textures

    glClearDepth(1.0f);                                   // depth buffer setup
    glEnable(GL_DEPTH_TEST);                              // enable depth testing
    glDepthFunc(GL_LEQUAL);                               // set the type of depth testing

    glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
    glEnable ( GL_COLOR_MATERIAL );
    glEnable(GL_LIGHTING);                                // enable lighting


    init_lights();                                        // init lights

    cam.alpha = 50.0; cam.theta = 50.0;                   // set initial camera values
    cam.x = 0.0; cam.y = 0.0; cam.z = 30.0;

    ground.texture = 18;                                  // set initial ground plane values
    ground.scale = 20.0f;
    init_ground(500.0f, ground.scale, 1, ground.texture); // init ground display list(100.0/500.0)*ground.scale
    // size: 50
    // size of one texture-square: ground.scale
    // number of small squares per texture-square: 100/50*ground.scale (makes the total number of small squares always 100)
    // texture: ground.texture

    for(int i=0; i<NTEXTURE; i++) textures[i]=NULL;

    glutDisplayFunc(display);
    glutReshapeFunc(reshape_cb);             // callback if window is reshaped
    glutMotionFunc(mouse_press_tracking_cb); // callback when mouse is pressed and moving in window
    glutMouseFunc(mouse_cb);                 // callback if mouse-button is clicked
    glutMouseWheelFunc(mouse_wheel_cb);
    glutKeyboardFunc(keyboard_cb);           // callback if key is pressed

    makeMenu();
    init_scene();

}

/* init light sources */
void init_lights()
{
    // make an only ambient light source: GL_LIGHT0
    GLfloat ambient[]  = {0.3f, 0.3f, 0.3f, 1.0f};   // white ambient color
    GLfloat diffuse[]  = {0.0f, 0.0f, 0.0f, 1.0f};   // no diffuse color
    GLfloat specular[] = {0.0f, 0.0f, 0.0f, 1.0f};   // no specular color

    glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);  // set ambient for ambient light source
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);  // set diffuse for ambient light source
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular); // set specular for ambient light source
    glEnable(GL_LIGHT0);                         // enable GL_LIGHT0

}

/* init scene objects */
void init_scene()
{
		// initiazlise object properties for new object to default values below
		sceneObjs[nObjects].x = 0.0;
		sceneObjs[nObjects].y = 0.0;
		sceneObjs[nObjects].z = 0.0;
		sceneObjs[nObjects].alpha = 0.0;
		sceneObjs[nObjects].beta = 0.0;
		sceneObjs[nObjects].gamma = 0.0;
		sceneObjs[nObjects].scale = SCENE_SCALE;
		sceneObjs[nObjects].texture = 1;
		sceneObjs[nObjects].texturescale =1.0;
		sceneObjs[nObjects].list_no = 0; // display list id should be same as menu id

		cObject = nObjects;      // change current object to this object (last added)
		nObjects++;
		init_DisplayList(0);    // initialize a display list with id same as menu id

		sceneObjs[nObjects].x = 0.0;
		sceneObjs[nObjects].y = 0.0;
		sceneObjs[nObjects].z = 0.0;
		sceneObjs[nObjects].alpha = 0.0;
		sceneObjs[nObjects].beta = 0.0;
		sceneObjs[nObjects].gamma = 0.0;
		sceneObjs[nObjects].scale = SCENE_SCALE;
		sceneObjs[nObjects].texture = 1;
		sceneObjs[nObjects].texturescale = 1.0;
		sceneObjs[nObjects].list_no = 1; // display list id should be same as menu id

		cObject = nObjects;      // change current object to this object (last added)
		nObjects++;

		init_DisplayList(1);    // initialize a display list with id same as menu id
}

/* init a display list*/
void init_DisplayList(int id)
{
		if(id==0)
		{
			DisplayLists[id] = glGenLists(1);              // generate 1 display list and attach to DisplayList[id]
			glNewList(DisplayLists[id], GL_COMPILE);       // start display list, and set compilation mode

			GLfloat ambient[]  = {1.0, 1.0, 1.0, 1.0};      // white ambient
			glMaterialfv(GL_FRONT, GL_AMBIENT,   ambient);  // set ambient for ground

			getTexture(31);                      // get ground texture
			glBindTexture(GL_TEXTURE_2D, 31);    // set ground texture to current texture
			glutSolidSphere(1,20,20);
			//FRONT
			glBegin(GL_POLYGON);
			glNormal3f(0.0, 1.0, 0.0);
			glTexCoord2f(1.0f, 0.0f);  glVertex3f( 1.5,  0.0, -5.0 );
			glTexCoord2f(1.0f, 1.0f);  glVertex3f( 1.5,  3.0, -3.0 );
			glTexCoord2f(0.0f, 1.0f);  glVertex3f(-1.5,  3.0, -3.0 );
			glTexCoord2f(0.0f, 0.0f);  glVertex3f(-1.5,  0.0, -5.0 );
			glEnd();

			getTexture(34);                      // get ground texture
			glBindTexture(GL_TEXTURE_2D, 35);    // set ground texture to current texture
			//BACK
			glBegin(GL_POLYGON);
			glNormal3f(0.0, -1.0, 0.0);
			glTexCoord2f(1.0f, 0.0f); 	glVertex3f(  1.5,  0.0, 3.0 );
			glTexCoord2f(1.0f, 1.0f);	glVertex3f(  1.5,  3.0, 3.0 );
			glTexCoord2f(0.0f, 1.0f);	glVertex3f( -1.5,  3.0, 3.0 );
			glTexCoord2f(0.0f, 0.0f);	glVertex3f( -1.5,  0.0, 3.0 );
			glEnd();

			getTexture(33);                      // get ground texture
			glBindTexture(GL_TEXTURE_2D, 33);    // set ground texture to current texture
			// RIGHT
			glBegin(GL_POLYGON);
			glNormal3f(1.0, 0.0, 0.0);

			glTexCoord2f(1.0f, 0.0f); glVertex3f(  1.5,  0.0, -5.0 );
			glTexCoord2f(1.0f, 1.0f); glVertex3f(  1.5,  3.0, -3.0 );
			glTexCoord2f(0.0f, 1.0f); glVertex3f(  1.5,  3.0, 3.0 );
			glTexCoord2f(0.0f, 0.0f); glVertex3f(  1.5,  0.0, 3.0 );
			glEnd();

			getTexture(32);                      // get ground texture
			glBindTexture(GL_TEXTURE_2D, 32);    // set ground texture to current texture
			// LEFT
			glBegin(GL_POLYGON);
			glNormal3f(-1.0, 0.0, 0.0);
			glTexCoord2f(1.0f, 0.0f); glVertex3f(  -1.5,  0.0, -5.0 );
			glTexCoord2f(1.0f, 1.0f); glVertex3f(  -1.5,  3.0, -3.0 );
			glTexCoord2f(0.0f, 1.0f); glVertex3f(  -1.5,  3.0, 3.0 );
			glTexCoord2f(0.0f, 0.0f); glVertex3f(  -1.5,  0.0, 3.0 );
			glEnd();

			getTexture(34);                      // get ground texture
			glBindTexture(GL_TEXTURE_2D, 34);    // set ground texture to current texture
			// TOP
			glBegin(GL_POLYGON);
			glColor3f(   1.0,  1.0,  1.0 );
			glTexCoord2f(1.0f, 0.0f); glVertex3f( -1.5, 3.0, 3.0 );
			glTexCoord2f(1.0f, 1.0f); glVertex3f(  1.5,  3.0, 3.0 );
			glTexCoord2f(0.0f, 1.0f); glVertex3f(  1.5,  3.0, -3.0 );
			glTexCoord2f(0.0f, 0.0f); glVertex3f( -1.5, 3.0, -3.0 );
			glEnd();

			glBindTexture(GL_TEXTURE_2D, 0); // set default texture

			glEndList(); // end display list
		}
		else if(id == 1)
		{
			DisplayLists[id] = glGenLists(1);              // generate 1 display list and attach to DisplayList[id]
			glNewList(DisplayLists[id], GL_COMPILE);       // start display list, and set compilation mode

			GLfloat ambient[]  = {0, 0, 0, 0};      // white ambient
			glMaterialfv(GL_FRONT, GL_AMBIENT,   ambient);  // set ambient for ground
	

			RectObstacle obstacle=obstacleMap->dynamicObstacles.front();

			double x1 = (obstacle.perimiter[0].a.x-obstacle.pos.x) - drawOffset_x;
			double x2 = (obstacle.perimiter[1].a.x-obstacle.pos.x) - drawOffset_x;
			double x3 = (obstacle.perimiter[2].a.x-obstacle.pos.x) - drawOffset_x;
			double x4 = (obstacle.perimiter[3].a.x-obstacle.pos.x) - drawOffset_x;

			double y1 = (obstacle.perimiter[0].a.y-obstacle.pos.y) - drawOffset_y;
			double y2 = (obstacle.perimiter[1].a.y-obstacle.pos.y) - drawOffset_y;
			double y3 = (obstacle.perimiter[2].a.y-obstacle.pos.y) - drawOffset_y;
			double y4 = (obstacle.perimiter[3].a.y-obstacle.pos.y) - drawOffset_y;

			double height = 3;

			glBegin(GL_POLYGON);
			glColor3f(   0.0,  0.0,  0.0 );
					glVertex3f( (GLfloat)x1, (GLfloat)height, -(GLfloat)y1 );
					glVertex3f( (GLfloat)x2, (GLfloat)height, -(GLfloat)y2 );
					glVertex3f( (GLfloat)x3, (GLfloat)height, -(GLfloat)y3 );
					glVertex3f( (GLfloat)x4, (GLfloat)height, -(GLfloat)y4 );
			glEnd();

			glBegin(GL_POLYGON);
					glColor3f(   1.0,  0.0,  0.0 );
					glVertex3f( (GLfloat)x1, (GLfloat)height, -(GLfloat)y1 );
					glVertex3f( (GLfloat)x1, (GLfloat)0.0, -(GLfloat)y1 );
					glVertex3f( (GLfloat)x2, (GLfloat)0.0, -(GLfloat)y2 );
					glVertex3f( (GLfloat)x2, (GLfloat)height, -(GLfloat)y2 );
			glEnd();

			glBegin(GL_POLYGON);	
					glVertex3f( (GLfloat)x2, (GLfloat)height, -(GLfloat)y2 );
					glVertex3f( (GLfloat)x2, (GLfloat)0.0, -(GLfloat)y2 );
					glVertex3f( (GLfloat)x3, (GLfloat)0.0, -(GLfloat)y3 );
					glVertex3f( (GLfloat)x3, (GLfloat)height, -(GLfloat)y3 );
			glEnd();

			glBegin(GL_POLYGON);
					glVertex3f( (GLfloat)x3, (GLfloat)height, -(GLfloat)y3 );
					glVertex3f( (GLfloat)x3, (GLfloat)0.0, -(GLfloat)y3 );
					glVertex3f( (GLfloat)x4, (GLfloat)0.0, -(GLfloat)y4 );
					glVertex3f( (GLfloat)x4, (GLfloat)height, -(GLfloat)y4 );
			glEnd();

			glBegin(GL_POLYGON);
					glVertex3f( (GLfloat)x4, (GLfloat)height, -(GLfloat)y4 );
					glVertex3f( (GLfloat)x4, (GLfloat)0.0, -(GLfloat)y4 );
					glVertex3f( (GLfloat)x1, (GLfloat)0.0, -(GLfloat)y1 );
					glVertex3f( (GLfloat)x1, (GLfloat)height, -(GLfloat)y1 );
			glEnd();

			//glBindTexture(GL_TEXTURE_2D, 0);

			glEndList(); // end display list
		}
}

texture* loadTexture(char *fileName)
{
    texture* t = (texture*)malloc(sizeof (texture));
    BITMAPINFO *info;

    t->rgbData = LoadDIBitmap(fileName, &info);
    t->height=info->bmiHeader.biHeight;
    t->width=info->bmiHeader.biWidth;

    return t;
}

/* getTexture(i) loads texture i if it isn't already loaded.
   After calling getTexture(i), you can make texture i the current texture using
   glBindTexture(GL_TEXTURE_2D, i);    (Use i=0 to return to the default plain texture.)
   You can then scale the texture via:   (See the textbook, section 8.8.3.)
   glMatrixMode(GL_TEXTURE);
   You must call getTexture(i) at least once before using texture i.*/
void getTexture(int i)
{
    // getTexture(i) loads texture i if it isn't already loaded.
    char fileName[220];
    if(i<1 || i>NTEXTURE) {
        printf("Error in getTexture - wrong texture number");

    }
    if(textures[i-1] != NULL)
        return;
    sprintf(fileName, "%s/texture%d.bmp", dataDir, i);

    textures[i-1] = loadTexture(fileName);

    glBindTexture(GL_TEXTURE_2D, i);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textures[i-1]->width, textures[i-1]->height,
            0, GL_RGB, GL_UNSIGNED_BYTE, textures[i-1]->rgbData);
    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, textures[i-1]->width, textures[i-1]->height, GL_RGB,
            GL_UNSIGNED_BYTE, textures[i-1]->rgbData);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

    glBindTexture(GL_TEXTURE_2D, 0);  // Back to default texture
}

/* init ground plane */
void init_ground(GLfloat size, GLfloat delta_text, int rect_per_text, int groundtexture)
{
    //Point2D utm_origo = Point2D(650458, 6561411);
    Point2D utm_origo = Point2D(0, 0);

    Quad left;
    left.p2 = Point2D(-100,-50)*SCENE_SCALE + utm_origo;
    left.p3 = Point2D(200,-50)*SCENE_SCALE + utm_origo;
    left.p4 = Point2D(200,-100)*SCENE_SCALE+ utm_origo;
    left.p1 = Point2D(-100,-100)*SCENE_SCALE+ utm_origo;



    ground.displayList = glGenLists(1);         // generate 1 display list and attach to ground.displayList
    glNewList(ground.displayList, GL_COMPILE);  // start display list, and set compilation mode

    // draw plane
    {

        getTexture(groundtexture);                      // get ground texture
        glBindTexture(GL_TEXTURE_2D, groundtexture);    // set ground texture to current texture

        GLfloat ambient[]  = {1.0, 1.0, 1.0, 1.0};      // white ambient
        glMaterialfv(GL_FRONT, GL_AMBIENT,   ambient);  // set ambient for ground

        glPushMatrix();
        glBegin(GL_QUADS);
        glNormal3f(0.0, 1.0, 0.0);                 // set normal to up (x = 0, y = 1, z = 0)

        for(GLfloat xx=-size/2; xx<size/2; xx+=delta_text)     // loop through texture-squares as big as defined by delta_text
            for(GLfloat zz=-size/2; zz<size/2; zz+=delta_text) //   (one texure pice applied to every one of these squares)
            {
                for(GLfloat i=0.0f; i<rect_per_text; i++)     // loop through number of squares defined by rect_per_square within a
                    for(GLfloat j=0.0f; j<rect_per_text; j++) //   texture-square (applies every texture pies to a number of smaller squares)
                    {
                        GLfloat delta = delta_text/rect_per_text; // calculate delta, i.e. size for smaller squares
                        GLfloat x = xx+i*delta;                   // calculate x-pos for smaller square
                        GLfloat z = zz+j*delta;                   // calculate z-pos for smaller square


                        // set texture coordinate and respective vertex coordinate for every corner in the smaller square
                        glTexCoord2f((0.0f+i)/rect_per_text, (0.0f+j)/rect_per_text);  glVertex3f(x + utm_origo.x*SCENE_SCALE       , 0.0f, -(z           + utm_origo.y*SCENE_SCALE));
                        glTexCoord2f((0.0f+i)/rect_per_text, (1.0f+j)/rect_per_text);  glVertex3f(x + utm_origo.x*SCENE_SCALE       , 0.0f, -(z + delta   + utm_origo.y*SCENE_SCALE));
                        glTexCoord2f((1.0f+i)/rect_per_text, (1.0f+j)/rect_per_text);  glVertex3f(x+delta + utm_origo.x*SCENE_SCALE , 0.0f, -(z + delta   + utm_origo.y*SCENE_SCALE));
                        glTexCoord2f((1.0f+i)/rect_per_text, (0.0f+j)/rect_per_text);  glVertex3f(x+delta + utm_origo.x*SCENE_SCALE , 0.0f, -(z           + utm_origo.y*SCENE_SCALE));
                    }
            }
        glEnd();
        glPopMatrix();

        glBindTexture(GL_TEXTURE_2D, 0); // set default texture
    }
    // draw arrows
    {
        GLfloat ambient[]  = {0.0, 0.0, 0.0, 1.0};      // no ambient

        glPushMatrix();
        glScalef((GLfloat)SCENE_SCALE, (GLfloat)SCENE_SCALE, (GLfloat)SCENE_SCALE); // set the scale for object i
        glLineWidth(2);
        glBegin(GL_LINES);
        glVertex3f(-5.0f, 0.01f, 0.0f); // line representing the x-axis
        glVertex3f( 5.0f, 0.01f, 0.0f);

        glVertex3f( 0.0f, 0.01f,5.0f); // line representing the z-axis
        glVertex3f( 0.0f, 0.01f, -5.0f);

        glVertex3f( 5.0f, 0.01f, 0.0f); // line forming one part of arrow in end of x-axis
        glVertex3f( 4.3f, 0.01f, -0.8f);

        glVertex3f( 5.0f, 0.01f, 0.0f); // line forming other part of arrow in end of x-axis
        glVertex3f( 4.3f, 0.01f,0.8f);

        glVertex3f( 0.0f, 0.01f, -5.0f); // line forming one part of arrow in end of z-axis
        glVertex3f( 0.8f, 0.01f, -4.3f);

        glVertex3f( 0.0f, 0.01f, -5.0f); // line forming other part of arrow in end of z-axis
        glVertex3f(-0.8f, 0.01f, -4.3f);
        glEnd();
        glPopMatrix();
    }

    //draw asphalt

    getTexture(24);                      // get ground texture
    glBindTexture(GL_TEXTURE_2D, 24);    // set ground texture to current texture

    GLfloat ambient[]  = {1.0, 1.0, 1.0, 1.0};      // white ambient
    glMaterialfv(GL_FRONT, GL_AMBIENT,   ambient);  // set ambient for ground

    glPushMatrix();
    glBegin(GL_POLYGON);
    glNormal3f(0.0, 1.0, 0.0);                 // set normal to up (x = 0, y = 1, z = 0)

    glTexCoord2f(5.0f, 0.0f); glVertex3f( 0*SCENE_SCALE, 0.1, 50*SCENE_SCALE );
    glTexCoord2f(5.0f, 5.0f); glVertex3f(  100*SCENE_SCALE,  0.1, 50*SCENE_SCALE );
    glTexCoord2f(0.0f, 5.0f); glVertex3f(  100*SCENE_SCALE,  0.1, -50*SCENE_SCALE );
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 0*SCENE_SCALE, 0.1, -50*SCENE_SCALE );

    glEnd();
    glBegin(GL_POLYGON);
    glNormal3f(0.0, 1.0, 0.0);                 // set normal to up (x = 0, y = 1, z = 0)

    glTexCoord2f(1.0f, 0.0f); glVertex3f( 100*SCENE_SCALE, 0.1, 6*SCENE_SCALE );
    glTexCoord2f(1.0f, 1.0f); glVertex3f(  120*SCENE_SCALE,  0.1, 6*SCENE_SCALE );
    glTexCoord2f(0.0f, 1.0f); glVertex3f(  120*SCENE_SCALE,  0.1, -6*SCENE_SCALE );
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 100*SCENE_SCALE, 0.1, -6*SCENE_SCALE );

    glEnd();
    glBegin(GL_POLYGON);
    glNormal3f(0.0, 1.0, 0.0);                 // set normal to up (x = 0, y = 1, z = 0)

    glTexCoord2f(150.0f, 0.0f); glVertex3f( 120*SCENE_SCALE, 0.1, 1500*SCENE_SCALE );
    glTexCoord2f(150.0f, 1.0f); glVertex3f(  130*SCENE_SCALE,  0.1, 1500*SCENE_SCALE );
    glTexCoord2f(0.0f, 1.0f); glVertex3f(  130*SCENE_SCALE,  0.1, -100*SCENE_SCALE );
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 120*SCENE_SCALE, 0.1, -100*SCENE_SCALE );

    glEnd();
    glPopMatrix();

    /*glBegin(GL_POLYGON);
    glNormal3f(0.0, 1.0, 0.0);                 // set normal to up (x = 0, y = 1, z = 0)

    glTexCoord2f(1.0f, 0.0f); glVertex3f( 120*SCENE_SCALE, 0.1, -90*SCENE_SCALE );
    glTexCoord2f(1.0f, 10.0f); glVertex3f( -30*SCENE_SCALE,  0.1, -90*SCENE_SCALE );
    glTexCoord2f(0.0f, 10.0f); glVertex3f(  -30*SCENE_SCALE,  0.1, -100*SCENE_SCALE );
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 120*SCENE_SCALE, 0.1, -100*SCENE_SCALE );

    glEnd();
    glPopMatrix();*/

    glBegin(GL_POLYGON);
    glNormal3f(0.0, 1.0, 0.0);                 // set normal to up (x = 0, y = 1, z = 0)

    glTexCoord2f(1.0f, 0.0f); glVertex3f( 120*SCENE_SCALE, 0.1, -90*SCENE_SCALE );
    glTexCoord2f(1.0f, 10.0f); glVertex3f( 50*SCENE_SCALE,  0.1, -90*SCENE_SCALE );
    glTexCoord2f(0.0f, 10.0f); glVertex3f( 50*SCENE_SCALE,  0.1, -100*SCENE_SCALE );
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 120*SCENE_SCALE, 0.1, -100*SCENE_SCALE );

    glEnd();
    glPopMatrix();

    glBegin(GL_POLYGON);
    glNormal3f(0.0, 1.0, 0.0);                 // set normal to up (x = 0, y = 1, z = 0)

    glTexCoord2f(10.0f, 0.0f); glVertex3f( -30*SCENE_SCALE, 0.1, 100*SCENE_SCALE );
    glTexCoord2f(10.0f, 1.0f); glVertex3f( -40*SCENE_SCALE,  0.1, 100*SCENE_SCALE );
    glTexCoord2f(0.0f, 1.0f); glVertex3f(  -40*SCENE_SCALE,  0.1, -100*SCENE_SCALE );
    glTexCoord2f(0.0f, 0.0f); glVertex3f( -30*SCENE_SCALE, 0.1, -100*SCENE_SCALE );

    glEnd();
    glPopMatrix();

    glPushMatrix();
    glBegin(GL_POLYGON);
    glNormal3f(0.0, 1.0, 0.0);                 // set normal to up (x = 0, y = 1, z = 0)

    glTexCoord2f(1.0f, 0.0f); glVertex3f( 0*SCENE_SCALE, 0.1, 6*SCENE_SCALE );
    glTexCoord2f(1.0f, 1.0f); glVertex3f(  -30*SCENE_SCALE,  0.1, 6*SCENE_SCALE );
    glTexCoord2f(0.0f, 1.0f); glVertex3f(  -30*SCENE_SCALE,  0.1, -6*SCENE_SCALE );
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 0*SCENE_SCALE, 0.1, -6*SCENE_SCALE );

    glEnd();
    glPopMatrix();

    glBindTexture(GL_TEXTURE_2D, 0); // set default texture




    glEndList(); // end display list
}

/****************************************************************/
/*** Glut callbacks *********************************************/
/****************************************************************/

/* rehsape callback */
void reshape_cb(int w, int h)
{
    WIDTH = w;
    HEIGHT = h;

    glViewport(0,0,w,h);         // set the viewport origin and with and height

    glMatrixMode(GL_PROJECTION);                                   // in matrix mode "projection":
    glLoadIdentity();                                            //    load indentity matrix
    gluPerspective(45.0f, (GLfloat)w/(GLfloat)h, 0.1f, 100.0f);  //    set gluPerspective
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glutPostRedisplay();         // call on display callback in the end of glut main loop
}

/* set camera position and rotation */
void update_camera()
{

    if(FOLLOW_CAR)
    {
        cam.x = egoState->y*SCENE_SCALE - drawOffset_y*SCENE_SCALE;
        cam.y = egoState->x*SCENE_SCALE - drawOffset_x*SCENE_SCALE;
    }


    glTranslatef(cam.x, -cam.y, -cam.z); // set the position and rotation of the camera perspective
    glRotatef(cam.theta, 1.0, 0.0, 0.0);
    glRotatef(cam.alpha, 0.0, 1.0, 0.0);

}

/* mouse click callback */
void mouse_cb(int button, int mstate, int x, int y)
{
    mouse_state = button;     // set mouse_state to which button that have been pressed/released
    printf("Mouse state: %d %d\n", mouse_state, mstate);
    if(mstate == GLUT_UP)     // if release:
        mouse_reset = true;   //   set mouse_reset to true

    if(mouse_state == 3)
        cam.z -= 0.8f;
    else if(mouse_state == 4)
        cam.z += 0.8f;//   change the z-position of camera accoring to mouse movements in y-direction

    if(cam.z < 3) cam.z = 3.0f;                 //   don't let camera sweep througt origo

}

/* mouse wheel callback */
void mouse_wheel_cb(int wheel, int direction, int x, int y)
{

    printf("MouseWheel: %d %d\n", wheel, direction);
    if(direction == 1)
    {
        cam.z -= 0.8f;
    }
    else
    {
        cam.z += 0.8f;//   change the z-position of camera accoring to mouse movements in y-direction
    }
    //if(cam.z < 3) cam.z = 3.0f;                 //   don't let camera sweep througt origo

    glutPostRedisplay(); // call on display callback in the end of glut main loop

}

/* mouse press and move callback */
void mouse_press_tracking_cb(int x, int y)
{

    static int last_x = 0; // variable to save last x position of mouse
    static int last_y = 0; // variable to save last x position of mouse

    static bool setWPxy = true;

    mouse_x = x;
    mouse_y = y;

    GLfloat intersection[3];
    GLfloat pointOnPlane[] = {0.0, 0.0, 0.0};  // one point on ground plane
    GLfloat planeNormal[] = {0.0, 1.0, 0.0};   // normal of ground plane

    if(mouse_reset)          // if mouse_reset is true, i.e. a mouse button have been released
    {                        // since last callback
        last_x = x;          //   "reset" x_last and y_last
        last_y = y;
        mouse_reset = false; //   set mouse_reset to false
    }


    switch(state) // depending on current state:
    {
    case MOVE_CAM: // if state is MOVE_CAM
        if(mouse_state == GLUT_LEFT_BUTTON)          // if left mouse button is pressed, move camera:
        {
            cam.alpha += (GLfloat)(x-last_x)/10;     //   rotate camera around y-axis (alpha) according to mouse movements i x-direction
            cam.theta += (GLfloat)(y-last_y)/10;     //   rotate camera around x-axis (theta) according to mouse movements i y-direction
            if(cam.alpha > 360)    cam.alpha -= 360; //   if any degree is > 360 or < 0, convert to be in range [0, 360]
            else if(cam.alpha < 0) cam.alpha += 360;
            if(cam.theta > 360)    cam.theta -= 360;
            else if(cam.theta < 0) cam.theta += 360;
        }
        break;
    case SETWP:

        pointOnPlane[1] = sceneObjs[0].y; // want to move object on the plane parallell to the ground plane, at a hight

        if(!getIntersection(x, y, intersection, pointOnPlane, planeNormal)) // get position on the above defined plane corresponding
        {                                                                  //   to the current mouse position, save in variable intersection
            break;
        }
        if(setWPstate == SET_POINT)
        {
            //   corresponding to the objects height above ground plane
            setWP_x =  intersection[0] / SCENE_SCALE + drawOffset_x; // change objects x and z position accordingly to the variable intersection
            setWP_y = -intersection[2] / SCENE_SCALE + drawOffset_y;

            if(mouse_state == GLUT_RIGHT_BUTTON)
            {
                setWPxy = false;
            }
        }
        else if(setWPstate == SET_HEADING)
        {
            double xi = intersection[0] / SCENE_SCALE + drawOffset_x;
            double yi = -intersection[2] / SCENE_SCALE + drawOffset_y;

            setWP_heading = atan2(yi - setWP_y, xi - setWP_x);

        }

        break;

    default:
        break;
    }

    if(mouse_state == GLUT_MIDDLE_BUTTON)
    {
        if(state != SETWP)
            state = MOVE_CAM;

        FOLLOW_CAR = false;

        cout << cam.z << endl;
        double sens = -15*cam.z + 330;
        if(sens < 50 && cam.z > 50)
            sens = 20;

        else if(sens < 50)
            sens = 50;

        cam.x += (GLfloat)(x-last_x)/sens;
        cam.y += (GLfloat)(y-last_y)/sens;
    }

    last_x = x; // save the current x and y position of mouse in the variables last_x and last_y for next time the callback is run
    last_y = y;

    glutPostRedisplay(); // call on display callback in the end of glut main loop
}

/* calculates the intesection between the plane defined by the point pointOnPlane and normal planeNormal and the ray starting at
   x and y (in screen coordinates) and going through to the farest end of the viewing volume. The result is saved in intersection. */
bool getIntersection(int x, int y, GLfloat intersection[], GLfloat pointOnPlane[], GLfloat planeNormal[])
{
    GLint viewport[4];               // array to save the x and y-values and the width and height of the viewport
    GLdouble modelview[16];          // array to save the modelview matrix
    GLdouble projection[16];         // array to save the projection matrix
    GLdouble x_near, y_near, z_near; // variables to save the position of the beginning of the ray (in openGL coordninates)
    GLdouble x_far,  y_far,  z_far;  // variables to save the position of the end of the ray (in openGL coordninates)

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);   // get hte modelview matrix
    glGetDoublev(GL_PROJECTION_MATRIX, projection); // get the projection matrix
    glGetIntegerv(GL_VIEWPORT, viewport);           // get the viewport (x, y, width, height)

    y = (float)viewport[3] - (float)y;              // turn around the y-axis => y = (height of viewport - y)

    // project the screen x and y coordinates on the near end of the viewing volume (0.0) and save the corresponding openGL coordinates
    //   in the variables x_near, y_near and z_near
    gluUnProject((float)x, (float)y, 0.0f, modelview, projection, viewport, &x_near, &y_near, &z_near);
    // project the screen x and y coordinates on the far end of the viewing volume (1.0) and save the corresponding openGL coordinates
    //   in the variables x_far, y_far and z_far
    gluUnProject((float)x, (float)y, 1.0f, modelview, projection, viewport, &x_far,  &y_far,  &z_far );

    GLfloat noraml_DOT_ray = planeNormal[0]*(x_far-x_near) +  // calculate the dot product between the plane normal and the ray
            planeNormal[1]*(y_far-y_near) +
            planeNormal[2]*(z_far-z_near);
    if(noraml_DOT_ray != 0)
    {
        GLfloat normal_DOT_near = planeNormal[0]*x_near +     // calculate the dot product between the plane normal and the start
                planeNormal[1]*y_near +     //   point of the ray
                planeNormal[2]*z_near;

        GLfloat normal_DOT_point = planeNormal[0]*pointOnPlane[0] +   // calculate the dot product between the plane normal and the
                planeNormal[1]*pointOnPlane[1] +   //   point given on the plane
                planeNormal[2]*pointOnPlane[2];

        GLfloat t = (normal_DOT_point - normal_DOT_near)/noraml_DOT_ray;  // calculate the distance to the ray intersection with the plane
        if(t < 0) return false;                                           // if distance is less than zero => intersection is behind the
        //    camera, return false;
        intersection[0] = x_near-(x_near-x_far)*t;  // save the intersection as the point you get to by following the ray a distance
        intersection[1] = y_near-(y_near-y_far)*t;  //   of t from the start point of the ray
        intersection[2] = z_near-(z_near-z_far)*t;

        return true;                                // return true => intersection saved correctly in varable intersection
    }
    // return false if the dot product between the plane normal and the ray is zero => no intersection
    return false;   //   possible since the ray and the plane are parallell
}

/* function that print directions to the user on the promt */
void printDirections(char* directions)
{
    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n%s\n\n\n\n\n\n\n\n\n\n", directions);
}

/* keyboard press callback */
void keyboard_cb(unsigned char key, int x, int y)
{


    if(key == 27 || key == 'q' || key == 'Q') // if 'ESC' or 'q' is pressed, terminate program
        exit(EXIT_SUCCESS);
    else if(key == 's' || key == 'S')
        RUN = !RUN;
}

/* make the menu */
void makeMenu()
{
    // Submenus level 2 for "Change ground texture"
    int id_ground1 = glutCreateMenu(menu1_cb);
    for(int i=0;  i<10; i++) glutAddMenuEntry(textureMenuEntries[i], i+1);
    int id_ground2 = glutCreateMenu(menu1_cb);
    for(int i=10; i<20; i++) glutAddMenuEntry(textureMenuEntries[i], i+1);
    int id_ground3 = glutCreateMenu(menu1_cb);
    for(int i=20; i<30; i++) glutAddMenuEntry(textureMenuEntries[i], i+1);

    // Submenu level 1 for "Change ground texture"
    int id_ground = glutCreateMenu(menu1_cb);
    glutAddSubMenu(" 1-10   ",  id_ground1);
    glutAddSubMenu(" 11-20   ", id_ground2);
    glutAddSubMenu(" 21-30   ", id_ground3);

    // Main manu
    glutCreateMenu(menu_cb);
    glutAddMenuEntry(" Start/Stop   ", 100);
    glutAddMenuEntry(" Move camera / Zoom   ", MOVE_CAM);
    glutAddMenuEntry(" Top view   ", TOPVIEW);
    glutAddMenuEntry(" Follow cam   ", FOLLOW);
    //glutAddSubMenu  (" Change ground texture...   ", id_ground);
    //glutAddMenuEntry(" Scale ground texture   ", SCALE_GROUND);
    glutAddMenuEntry(" Exit", EXIT);
    glutAttachMenu(GLUT_RIGHT_BUTTON);    // Attach menu to the right mouse button
}

/* menu callback (normal change state menu callback) */
void menu_cb(int id)
{
    if(id != -1 && id < 100)    // if id is valid ( != -1 )
    {
        if(id == TOPVIEW)
        {
            cam.theta = 90;
            cam.alpha = 90;
        }
        if(id == FOLLOW)
        {
            FOLLOW_CAR = !FOLLOW_CAR;
        }
        else
        {
            state = (statemachine)id; //   change state to id
        }
    }
    else if(id == 100)
        RUN = !RUN;

    if(state == EXIT)       // if state is EXIT
        exit(EXIT_SUCCESS); //   terminate program
}

/* menu3 callback (change ground texture menu callback) */
void menu1_cb(int id)
{
    ground.texture = id; // set texture id to the same as manu id
    init_ground(500.0f, 1.0f, 1, ground.texture); // reinitialize ground display list with new texture

    state = MOVE_CAM;    // change state to MOVE_CAM
    glutPostRedisplay(); // call on display callback in the end of glut main loop
}

/* display callback */
void display()
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    // update camera perspective
    update_camera();
    // draw ground
    glCallList(ground.displayList);

    // draw scene objects
    for(int i=0; i<nObjects; i++)
    {
			glPushMatrix();
			glTranslatef(sceneObjs[i].x - drawOffset_x*SCENE_SCALE, sceneObjs[i].y, -(sceneObjs[i].z - drawOffset_y*SCENE_SCALE));// set the position and rotation for object i
			glRotatef(sceneObjs[i].alpha, 1.0, 0.0, 0.0);
			glRotatef(sceneObjs[i].beta , 0.0, 1.0, 0.0);
			glRotatef(sceneObjs[i].gamma, 0.0, 0.0, 1.0);

			glScalef(sceneObjs[i].scale, sceneObjs[i].scale, sceneObjs[i].scale); // set the scale for object i

			glMatrixMode(GL_TEXTURE);
			glPushMatrix();
			glScalef(sceneObjs[i].texturescale, sceneObjs[i].texturescale, sceneObjs[i].texturescale); // set the texture scale for object i
			glMatrixMode(GL_MODELVIEW);


			getTexture(sceneObjs[i].texture);                       // get object texture
			//glBindTexture(GL_TEXTURE_2D, sceneObjs[i].texture);     // set object texture to current texture

			glCallList(DisplayLists[sceneObjs[i].list_no]);                       // call display list to draw object i (draw model 'list_no')

			glMatrixMode(GL_TEXTURE);
			glPopMatrix();

			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();
    }

	GLfloat notVisitedColor[] = { 0.0f, 1.0f, 0.0f };
	GLfloat notVisitedColorLine[] = { 1.0f, 1.0f, 0.0f };
	drawOptimalTrajectorylist(optimalTrajectory->get_optimal_trajectory_state_vector(),SCENE_SCALE, drawOffset_x, drawOffset_y, notVisitedColor, notVisitedColorLine);

    //draw obstacles
    drawRectObstacle(&obstacleMap->staticObstacles, SCENE_SCALE, drawOffset_x, drawOffset_y);

    ostringstream ss;
    if(state == SETWP)
    {
        GLfloat wpColor[] = {0.0f, 1.0f, 0.0f};
        drawSolidCircle(0.5f, (setWP_x - drawOffset_x)*SCENE_SCALE, 0.15, (setWP_y - drawOffset_y)*SCENE_SCALE, 30, wpColor);
        GLfloat colorTriangle[] = {1.0f, 0.0f, 0.0f};
        drawTrianlge(setWP_heading - M_PI/2, (setWP_x - drawOffset_x)*SCENE_SCALE, (setWP_y - drawOffset_y)*SCENE_SCALE, 0.5f, colorTriangle);

        ss << "x: " << setWP_x;
        putText(ss.str(), mouse_x + 30, HEIGHT - mouse_y - 10, WIDTH, HEIGHT);

        ss.str("");
        ss.clear();
        ss << "y: " << setWP_y;
        putText(ss.str(), mouse_x + 30, HEIGHT - mouse_y - 30, WIDTH, HEIGHT);

        ss.str("");
        ss.clear();
        ss << "Heading: " << setWP_heading*180/M_PI;
        putText(ss.str(), mouse_x + 30 , HEIGHT - mouse_y - 50, WIDTH, HEIGHT);
    }

    //print information text

    ss.str("");
    ss.clear();
    ss << "Position: x:" << (int)egoState->x;
    putText(ss.str(), 20, HEIGHT -40, WIDTH, HEIGHT);

    ss.str("");
    ss.clear();
    ss << " y:" << (int)egoState->y;
    putText(ss.str(), 200, HEIGHT -40, WIDTH, HEIGHT);

    ss.str("");
    ss.clear();
    ss << "Velocity: " << egoState->v;
    putText(ss.str(), 20, HEIGHT -60, WIDTH, HEIGHT);

    ss.str("");
    ss.clear();
    ss << "Heading: " << (int)(egoState->heading*180/M_PI);
    putText(ss.str(), 20, HEIGHT -80, WIDTH, HEIGHT);

    glutSwapBuffers(); // swap buffer (with GLUT_DOUBLE and glutSwapBuffers, the effect of clearing is removed)
    glFlush();         // end diplay callback with flush
}

void glutVizualisation::updateObjects()
{

    //Move visualization objects
	//The car
    sceneObjs[0].x		= egoState->x*SCENE_SCALE;
    sceneObjs[0].y		= 0.0f;
    sceneObjs[0].z		= egoState->y*SCENE_SCALE;
    sceneObjs[0].alpha	= 0.0f;
    sceneObjs[0].beta	= egoState->heading*180/M_PI-90;
    sceneObjs[0].gamma	= 0.0f;

	//First object
	sceneObjs[1].x		= (obstacleMap->dynamicObstacles.begin()->pos.x)*SCENE_SCALE;
    sceneObjs[1].y		= 0.0f;
	sceneObjs[1].z		= (obstacleMap->dynamicObstacles.begin()->pos.y)*SCENE_SCALE;
	sceneObjs[1].alpha	= 0.0f;
	sceneObjs[1].beta	= (obstacleMap->dynamicObstacles.begin()->orientation)*180/M_PI-90;
	sceneObjs[1].gamma	= 0.0f;
	
}

void glutVizualisation::setSimulationModel(Model* simulationModel)
{
    egoModel = simulationModel;
}

void glutVizualisation::setObstacleMap(ObstacleMap* obstacleMapIn)
{
    obstacleMap = obstacleMapIn;
}

bool glutVizualisation::getRunState()
{
    return RUN;
}

void glutVizualisation::setDrawOffset(double offset_x, double offset_y)
{
    drawOffset_x = offset_x;
    drawOffset_y = offset_y;
}

void glutVizualisation::setOptimalTrajectory(OptimalTrajectory *optimalTrajectory_in)
{
	optimalTrajectory = optimalTrajectory_in;
}

void glutVizualisation::setEgoState(State& state_in)
{
    egoState = &state_in;
}