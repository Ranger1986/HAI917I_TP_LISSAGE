// -------------------------------------------
// gMini : a minimal OpenGL/GLUT application
// for 3D graphics.
// Copyright (C) 2006-2008 Tamy Boubekeur
// All rights reserved.
// -------------------------------------------

// -------------------------------------------
// Disclaimer: this code is dirty in the
// meaning that there is no attention paid to
// proper class attribute access, memory
// management or optimisation of any kind. It
// is designed for quick-and-dirty testing
// purpose.
// -------------------------------------------

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>
#include <cstdio>
#include <cstdlib>

#include <algorithm>
#include <GL/glut.h>
#include <float.h>
#include "src/Vec3.h"
#include "src/Camera.h"


struct Triangle {
    inline Triangle () {
        v[0] = v[1] = v[2] = 0;
    }
    inline Triangle (const Triangle & t) {
        v[0] = t.v[0];   v[1] = t.v[1];   v[2] = t.v[2];
    }
    inline Triangle (unsigned int v0, unsigned int v1, unsigned int v2) {
        v[0] = v0;   v[1] = v1;   v[2] = v2;
    }
    unsigned int & operator [] (unsigned int iv) { return v[iv]; }
    unsigned int operator [] (unsigned int iv) const { return v[iv]; }
    inline virtual ~Triangle () {}
    inline Triangle & operator = (const Triangle & t) {
        v[0] = t.v[0];   v[1] = t.v[1];   v[2] = t.v[2];
        return (*this);
    }
    // membres indices des sommets du triangle:
    unsigned int v[3];
};


struct Mesh {
    std::vector< Vec3 > vertices; //array of mesh vertices positions
    std::vector< Vec3 > normals; //array of vertices normals useful for the display
    std::vector< Triangle > triangles; //array of mesh triangles
    std::vector< Vec3 > triangle_normals; //triangle normals to display face normals

    //Compute face normals for the display
    void computeTrianglesNormals(){

        triangle_normals.clear();
        //Iterate over mesh triangles
        for( unsigned int i = 0 ; i < triangles.size() ;i++ ){
            //Triangle i normal is the normalized cross product of two of its edges
            const Vec3 & e0 = vertices[triangles[i][1]] - vertices[triangles[i][0]];
            const Vec3 & e1 = vertices[triangles[i][2]] - vertices[triangles[i][0]];

            Vec3 n = Vec3::cross( e0, e1 );
            n.normalize();
            triangle_normals.push_back( n );
        }
    }

    //Compute vertices normals as the average of its incident faces normals
    void computeVerticesNormals(){

        normals.clear();
        normals.resize( vertices.size(), Vec3(0., 0., 0.) );
        for( unsigned int i = 0 ; i < triangles.size() ;i++ ){
            for( unsigned int t = 0 ; t < 3 ; t++ )
                normals[ triangles[i][t] ] += triangle_normals[i];
        }
        for( unsigned int i = 0 ; i < vertices.size() ;i++ )
            normals[ i ].normalize();

    }

    void computeNormals(){
        computeTrianglesNormals();
        computeVerticesNormals();
    }
};

//Transformation made of a rotation and translation
struct Transformation {
    Mat3 rotation;
    Vec3 translation;
};

//Basis ( origin, i, j ,k )
struct Basis {
    inline Basis ( Vec3 const & i_origin,  Vec3 const & i_i, Vec3 const & i_j, Vec3 const & i_k) {
        origin = i_origin; i = i_i ; j = i_j ; k = i_k;
    }

    inline Basis ( ) {
        origin = Vec3(0., 0., 0.);
        i = Vec3(1., 0., 0.) ; j = Vec3(0., 1., 0.) ; k = Vec3(0., 0., 1.);
    }
    Vec3 operator [] (unsigned int ib) {
        if(ib==0) return i;
        if(ib==1) return j;
        return k;}

    Vec3 origin;
    Vec3 i;
    Vec3 j;
    Vec3 k;
};

//Poinset data
struct PointSet {
    std::vector< Vec3 > positions;
    std::vector< Vec3 > normals;
};

//Plane defined by a point and a normal
struct Plane {
    inline Plane ( Vec3 const & i_point,  Vec3 const & i_normal ) {
        point = i_point; normal = i_normal;
    }

    inline Plane ( ) {
        point = Vec3(0., 0., 0.);
        normal = Vec3(0., 1., 0.);
    }
    Vec3 point;
    Vec3 normal;
};

//Input mesh loaded at the launch of the application
Mesh mesh;
//Mesh on which a transformation is applied
Mesh transformed_mesh;

Mesh ellipsoide;
Mesh transformed_ellipsoide;

Transformation mesh_transformation;
Mat3 normal_transformation;

Basis basis;
Basis transformed_basis;

PointSet projection_on_basis[3];
PointSet transformed_projection_on_basis[3];

Plane planes[3];
Plane transformed_planes[3];

unsigned int plane_id;

bool display_normals;
bool display_smooth_normals;
bool display_mesh;
bool display_transformed_mesh;
bool display_ellipsoide;
bool display_basis;
bool display_plane;

// -------------------------------------------
// OpenGL/GLUT application code.
// -------------------------------------------

static GLint window;
static unsigned int SCREENWIDTH = 1600;
static unsigned int SCREENHEIGHT = 900;
static Camera camera;
static bool mouseRotatePressed = false;
static bool mouseMovePressed = false;
static bool mouseZoomPressed = false;
static int lastX=0, lastY=0, lastZoom=0;
static bool fullScreen = false;



// ------------------------------------
// Projections
// ------------------------------------

Vec3 project( Vec3 const & input_point, Plane const & i_plane ) {
    Vec3 cx = input_point - i_plane.point;
    return input_point - Vec3::dot(cx, i_plane.normal) * i_plane.normal;
}

Vec3 project( Vec3 const & input_point, Vec3 const & i_origin, Vec3 const & i_axis ) {
    return i_origin + Vec3::dot( input_point - i_origin, i_axis ) * i_axis;
}

void project( std::vector<Vec3> const & i_points, std::vector<Vec3> & o_points, Vec3 const & i_origin, Vec3 const & i_axis ) {
    o_points.clear();
    for( unsigned int i = 0 ; i < i_points.size() ; i++ )
        o_points.push_back( project( i_points[i], i_origin, i_axis) );
}

void project( std::vector<Vec3> const & i_points, std::vector<Vec3> & o_points, Plane const & i_plane ) {
    o_points.clear();
    for( unsigned int i = 0 ; i < i_points.size() ; i++ )
        o_points.push_back( project( i_points[i], i_plane ) );
}
// ------------------------------------


// ------------------------------------
// File I/O
// ------------------------------------
bool saveOFF( const std::string & filename ,
              std::vector< Vec3 > & i_vertices ,
              std::vector< Vec3 > & i_normals ,
              std::vector< Triangle > & i_triangles,
              bool save_normals = true ) {
    std::ofstream myfile;
    myfile.open(filename.c_str());
    if (!myfile.is_open()) {
        std::cout << filename << " cannot be opened" << std::endl;
        return false;
    }

    myfile << "OFF" << std::endl ;

    unsigned int n_vertices = i_vertices.size() , n_triangles = i_triangles.size();
    myfile << n_vertices << " " << n_triangles << " 0" << std::endl;

    for( unsigned int v = 0 ; v < n_vertices ; ++v ) {
        myfile << i_vertices[v][0] << " " << i_vertices[v][1] << " " << i_vertices[v][2] << " ";
        if (save_normals) myfile << i_normals[v][0] << " " << i_normals[v][1] << " " << i_normals[v][2] << std::endl;
        else myfile << std::endl;
    }
    for( unsigned int f = 0 ; f < n_triangles ; ++f ) {
        myfile << 3 << " " << i_triangles[f][0] << " " << i_triangles[f][1] << " " << i_triangles[f][2];
        myfile << std::endl;
    }
    myfile.close();
    return true;
}

void openOFF( std::string const & filename,
              std::vector<Vec3> & o_vertices,
              std::vector<Vec3> & o_normals,
              std::vector< Triangle > & o_triangles,
              bool load_normals = true )
{
    std::ifstream myfile;
    myfile.open(filename.c_str());
    if (!myfile.is_open())
    {
        std::cout << filename << " cannot be opened" << std::endl;
        return;
    }

    std::string magic_s;

    myfile >> magic_s;

    if( magic_s != "OFF" )
    {
        std::cout << magic_s << " != OFF :   We handle ONLY *.off files." << std::endl;
        myfile.close();
        exit(1);
    }

    int n_vertices , n_faces , dummy_int;
    myfile >> n_vertices >> n_faces >> dummy_int;

    o_vertices.clear();
    o_normals.clear();

    for( int v = 0 ; v < n_vertices ; ++v )
    {
        float x , y , z ;

        myfile >> x >> y >> z ;
        o_vertices.push_back( Vec3( x , y , z ) );

        if( load_normals ) {
            myfile >> x >> y >> z;
            o_normals.push_back( Vec3( x , y , z ) );
        }
    }

    o_triangles.clear();
    for( int f = 0 ; f < n_faces ; ++f )
    {
        int n_vertices_on_face;
        myfile >> n_vertices_on_face;

        if( n_vertices_on_face == 3 )
        {
            unsigned int _v1 , _v2 , _v3;
            myfile >> _v1 >> _v2 >> _v3;

            o_triangles.push_back(Triangle( _v1, _v2, _v3 ));
        }
        else if( n_vertices_on_face == 4 )
        {
            unsigned int _v1 , _v2 , _v3 , _v4;
            myfile >> _v1 >> _v2 >> _v3 >> _v4;

            o_triangles.push_back(Triangle(_v1, _v2, _v3 ));
            o_triangles.push_back(Triangle(_v1, _v3, _v4));
        }
        else
        {
            std::cout << "We handle ONLY *.off files with 3 or 4 vertices per face" << std::endl;
            myfile.close();
            exit(1);
        }
    }

}

// ------------------------------------

void computeBBox( std::vector< Vec3 > const & i_positions, Vec3 & bboxMin, Vec3 & bboxMax ) {
    bboxMin = Vec3 ( FLT_MAX , FLT_MAX , FLT_MAX );
    bboxMax = Vec3 ( FLT_MIN , FLT_MIN , FLT_MIN );
    for(unsigned int pIt = 0 ; pIt < i_positions.size() ; ++pIt) {
        for( unsigned int coord = 0 ; coord < 3 ; ++coord ) {
            bboxMin[coord] = std::min<float>( bboxMin[coord] , i_positions[pIt][coord] );
            bboxMax[coord] = std::max<float>( bboxMax[coord] , i_positions[pIt][coord] );
        }
    }
}

void setEllipsoide( Mesh & o_mesh, Basis i_basis, int nX=20, int nY=20 )
{
    o_mesh.vertices.clear();
    o_mesh.normals.clear();
    o_mesh.triangles.clear();
    float thetaStep = 2*M_PI/(nX-1);
    float phiStep = M_PI/(nY-1);

    Mat3 rot = Mat3::getFromRows(i_basis[0], i_basis[1], i_basis[2]);
    Mat3 n_rot = Mat3::inverse(rot);
    n_rot.transpose();



    for(int i=0; i<nX;i++)
    {
        for(int j=0;j<nY;j++)
        {
            float t = thetaStep*i;
            float p = phiStep*j - M_PI/2;

            Vec3 position(cos(t)*cos(p), sin(t)*cos(p), sin(p));
            Vec3 normal = n_rot*position;

            normal.normalize();

            o_mesh.vertices.push_back(rot*position+i_basis.origin);
            o_mesh.normals.push_back(normal);
        }
    }
    for(int i=0; i<nX-1;i++)
    {
        for(int j=0;j<nY-1;j++)
        {
            o_mesh.triangles.push_back(Triangle(i*nY+j, (i+1)*nY+j, (i+1)*nY+j+1));
            o_mesh.triangles.push_back(Triangle(i*nY+j, (i+1)*nY+j+1, i*nY+j+1));
        }
    }


    ellipsoide.computeTrianglesNormals();
}

// ------------------------------------
// Application initialization
// ------------------------------------
void initLight () {
    GLfloat light_position1[4] = {22.0f, 16.0f, 50.0f, 0.0f};
    GLfloat direction1[3] = {-52.0f,-16.0f,-50.0f};
    GLfloat color1[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat ambient[4] = {0.3f, 0.3f, 0.3f, 0.5f};

    glLightfv (GL_LIGHT1, GL_POSITION, light_position1);
    glLightfv (GL_LIGHT1, GL_SPOT_DIRECTION, direction1);
    glLightfv (GL_LIGHT1, GL_DIFFUSE, color1);
    glLightfv (GL_LIGHT1, GL_SPECULAR, color1);
    glLightModelfv (GL_LIGHT_MODEL_AMBIENT, ambient);
    glEnable (GL_LIGHT1);
    glEnable (GL_LIGHTING);
}

void init () {
    camera.resize (SCREENWIDTH, SCREENHEIGHT);
    initLight ();
    glCullFace (GL_BACK);
    glDisable (GL_CULL_FACE);
    glDepthFunc (GL_LESS);
    glEnable (GL_DEPTH_TEST);
    glClearColor (0.2f, 0.2f, 0.3f, 1.0f);
    glEnable(GL_COLOR_MATERIAL);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    display_normals = false;
    display_mesh = true;
    display_transformed_mesh = true;
    display_plane = false;
    display_smooth_normals = true;

    plane_id = 0;
}


// ------------------------------------
// Rendering.
// ------------------------------------

void drawPointSet( std::vector< Vec3 > const & i_positions  ) {
    glDisable(GL_LIGHTING);
    glPointSize(2);
    glBegin(GL_POINTS);
    for(unsigned int pIt = 0 ; pIt < i_positions.size() ; ++pIt) {
        glVertex3f( i_positions[pIt][0] , i_positions[pIt][1] , i_positions[pIt][2] );
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

void drawVector( Vec3 const & i_from, Vec3 const & i_to ) {

    glBegin(GL_LINES);
    glVertex3f( i_from[0] , i_from[1] , i_from[2] );
    glVertex3f( i_to[0] , i_to[1] , i_to[2] );
    glEnd();
}

void drawAxis( Vec3 const & i_origin, Vec3 const & i_direction ) {

    glLineWidth(4); // for example...
    drawVector(i_origin, i_origin + i_direction);
}

void drawReferenceFrame( Vec3 const & origin, Vec3 const & i, Vec3 const & j, Vec3 const & k ) {

    glDisable(GL_LIGHTING);
    glColor3f( 0.8, 0.2, 0.2 );
    drawAxis( origin, i );
    glColor3f( 0.2, 0.8, 0.2 );
    drawAxis( origin, j );
    glColor3f( 0.2, 0.2, 0.8 );
    drawAxis( origin, k );
    glEnable(GL_LIGHTING);

}

void drawReferenceFrame( Basis & i_basis ) {
    drawReferenceFrame( i_basis.origin, i_basis.i, i_basis.j, i_basis.k );
}

void drawPlane( Plane const & i_plane ) {


    Vec3 j = i_plane.normal;
    Vec3 i = i_plane.normal.getOrthogonal();
    i.normalize();
    Vec3 k = Vec3::cross(i,j);

    Mat3 tr = Mat3::getFromCols(i,j,k);

    Vec3 iplanes_vertices [] ={
        tr*Vec3(-1.,0.,1.)+i_plane.point,
        tr*Vec3(1.,0.,1.)+i_plane.point,
        tr*Vec3(1.,0.,-1.)+i_plane.point,
        tr*Vec3(-1.,0.,-1.)+i_plane.point
    };

    glBegin(GL_QUADS);
    glNormal3f( i_plane.normal[0], i_plane.normal[1], i_plane.normal[2]);
    for(unsigned int i = 0 ; i < 4 ; i ++)
        glVertex3f( iplanes_vertices[i][0] , iplanes_vertices[i][1] , iplanes_vertices[i][2] );
    glEnd();

    Vec3 to = 0.5*i_plane.normal;
    glColor3f(0.8,1,0.5); // GREEN
    drawAxis( i_plane.point, to );
}

void drawSmoothTriangleMesh( Mesh const & i_mesh ) {
    glBegin(GL_TRIANGLES);
    for(unsigned int tIt = 0 ; tIt < i_mesh.triangles.size(); ++tIt) {
        Vec3 p0 = i_mesh.vertices[i_mesh.triangles[tIt][0]];
        Vec3 n0 = i_mesh.normals[i_mesh.triangles[tIt][0]];

        Vec3 p1 = i_mesh.vertices[i_mesh.triangles[tIt][1]];
        Vec3 n1 = i_mesh.normals[i_mesh.triangles[tIt][1]];

        Vec3 p2 = i_mesh.vertices[i_mesh.triangles[tIt][2]];
        Vec3 n2 = i_mesh.normals[i_mesh.triangles[tIt][2]];

        glNormal3f( n0[0] , n0[1] , n0[2] );
        glVertex3f( p0[0] , p0[1] , p0[2] );
        glNormal3f( n1[0] , n1[1] , n1[2] );
        glVertex3f( p1[0] , p1[1] , p1[2] );
        glNormal3f( n2[0] , n2[1] , n2[2] );
        glVertex3f( p2[0] , p2[1] , p2[2] );
    }
    glEnd();

}

void drawTriangleMesh( Mesh const & i_mesh ) {
    glBegin(GL_TRIANGLES);
    for(unsigned int tIt = 0 ; tIt < i_mesh.triangles.size(); ++tIt) {
        Vec3 p0 = i_mesh.vertices[i_mesh.triangles[tIt][0]];
        Vec3 p1 = i_mesh.vertices[i_mesh.triangles[tIt][1]];
        Vec3 p2 = i_mesh.vertices[i_mesh.triangles[tIt][2]];

        Vec3 n = i_mesh.triangle_normals[tIt];

        glNormal3f( n[0] , n[1] , n[2] );

        glVertex3f( p0[0] , p0[1] , p0[2] );
        glVertex3f( p1[0] , p1[1] , p1[2] );
        glVertex3f( p2[0] , p2[1] , p2[2] );
    }
    glEnd();

}

void drawMesh( Mesh const & i_mesh ){
    if(display_smooth_normals)
        drawSmoothTriangleMesh(i_mesh) ;
    else {
        drawTriangleMesh(i_mesh) ;
    }
}

void drawVectorField( std::vector<Vec3> const & i_positions, std::vector<Vec3> const & i_directions ) {
    glLineWidth(1.);
    for(unsigned int pIt = 0 ; pIt < i_directions.size() ; ++pIt) {
        Vec3 to = i_positions[pIt] + 0.02*i_directions[pIt];
        drawVector(i_positions[pIt], to);
    }
}

void drawNormals(Mesh const& i_mesh){

    if(display_smooth_normals){
        drawVectorField( i_mesh.vertices, i_mesh.normals );
    } else {
        std::vector<Vec3> triangle_baricenters;
        for ( const Triangle& triangle : i_mesh.triangles ){
            Vec3 triangle_baricenter (0.,0.,0.);
            for( unsigned int i = 0 ; i < 3 ; i++ )
                triangle_baricenter += i_mesh.vertices[triangle[i]];
            triangle_baricenter /= 3;
            triangle_baricenters.push_back(triangle_baricenter);
        }

        drawVectorField( triangle_baricenters, i_mesh.triangle_normals );
    }
}

//Draw fonction
void draw () {

    if( display_mesh ){
        glColor3f(0.8,1,0.8);
        drawMesh(mesh);

        if(display_normals){
            glColor3f(1.,0.,0.);
            drawNormals(mesh);
        }

        if( display_basis ){
            drawReferenceFrame(basis);

            for( unsigned int i = 0 ; i < 3 ; i++ ){
                glColor3f(1.,1,1.);
                drawPointSet(projection_on_basis[i].positions);
            }
        }

        if( display_plane ){
            glColor3f(0.94,0.81,0.38);
            drawPlane(planes[plane_id]);

            PointSet projection_on_plane;
            project( mesh.vertices, projection_on_plane.positions, planes[plane_id] );

            glColor3f(1.,1,1.);
            drawPointSet( projection_on_plane.positions );
        }

        if( display_ellipsoide ){
            glColor3f(0.39,0.57,0.67);
            drawMesh(ellipsoide);
        }
    }

    if( display_transformed_mesh ){
        glColor3f(0.8,0.8,1);
        drawMesh(transformed_mesh);

        if(display_normals){
            glColor3f(1.,0.,0.);
            drawNormals(transformed_mesh);
        }

        if( display_basis ){

            drawReferenceFrame(transformed_basis);

            for( unsigned int i = 0 ; i < 3 ; i++ ){
                glColor3f(1.,1,1.);
                drawPointSet(transformed_projection_on_basis[i].positions);
            }
        }

        if( display_plane ){
            glColor3f(0.94,0.81,0.38);
            drawPlane(transformed_planes[plane_id]);

            PointSet projection_on_plane;
            project( transformed_mesh.vertices, projection_on_plane.positions, transformed_planes[plane_id] );

            glColor3f(1.,1,1.);
            drawPointSet( projection_on_plane.positions );
        }

        if( display_ellipsoide ){
            glColor3f(0.39,0.57,0.67);
            drawMesh(transformed_ellipsoide);
        }
    }



}

void display () {
    glLoadIdentity ();
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera.apply ();
    draw ();
    glFlush ();
    glutSwapBuffers ();
}

void idle () {
    glutPostRedisplay ();
}

// ------------------------------------
// User inputs
// ------------------------------------
//Keyboard event
void key (unsigned char keyPressed, int x, int y) {
    switch (keyPressed) {
    case 'f':
        if (fullScreen == true) {
            glutReshapeWindow (SCREENWIDTH, SCREENHEIGHT);
            fullScreen = false;
        } else {
            glutFullScreen ();
            fullScreen = true;
        }
        break;


    case 'w':
        GLint polygonMode[2];
        glGetIntegerv(GL_POLYGON_MODE, polygonMode);
        if(polygonMode[0] != GL_FILL)
            glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
        else
            glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
        break;


    case 'b': //Toggle basis display
        display_basis = !display_basis;
        break;

    case 'e': //Toggle ellipsoide display
        display_ellipsoide = !display_ellipsoide;
        break;

    case 'n': //Press n key to display normals
        display_normals = !display_normals;
        break;

    case '1': //Toggle loaded mesh display
        display_mesh = !display_mesh;
        break;

    case '2': //Toggle transformed mesh display
        display_transformed_mesh = !display_transformed_mesh;
        break;

    case 'p': //Toggle plane display
        display_plane = !display_plane;
        break;

    case '+': //change plane to display
        plane_id++;
        if( plane_id > 2 ) plane_id = 0;
        break;

    case 's': //Switches between face normals and vertices normals
        display_smooth_normals = !display_smooth_normals;
        break;

    default:
        break;
    }
    idle ();
}

//Mouse events
void mouse (int button, int state, int x, int y) {
    if (state == GLUT_UP) {
        mouseMovePressed = false;
        mouseRotatePressed = false;
        mouseZoomPressed = false;
    } else {
        if (button == GLUT_LEFT_BUTTON) {
            camera.beginRotate (x, y);
            mouseMovePressed = false;
            mouseRotatePressed = true;
            mouseZoomPressed = false;
        } else if (button == GLUT_RIGHT_BUTTON) {
            lastX = x;
            lastY = y;
            mouseMovePressed = true;
            mouseRotatePressed = false;
            mouseZoomPressed = false;
        } else if (button == GLUT_MIDDLE_BUTTON) {
            if (mouseZoomPressed == false) {
                lastZoom = y;
                mouseMovePressed = false;
                mouseRotatePressed = false;
                mouseZoomPressed = true;
            }
        }
    }
    idle ();
}

//Mouse motion, update camera
void motion (int x, int y) {
    if (mouseRotatePressed == true) {
        camera.rotate (x, y);
    }
    else if (mouseMovePressed == true) {
        camera.move ((x-lastX)/static_cast<float>(SCREENWIDTH), (lastY-y)/static_cast<float>(SCREENHEIGHT), 0.0);
        lastX = x;
        lastY = y;
    }
    else if (mouseZoomPressed == true) {
        camera.zoom (float (y-lastZoom)/SCREENHEIGHT);
        lastZoom = y;
    }
}


void reshape(int w, int h) {
    camera.resize (w, h);
}

// ------------------------------------
// Start of graphical application
// ------------------------------------
int main (int argc, char ** argv) {
    if (argc > 2) {
        exit (EXIT_FAILURE);
    }
    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize (SCREENWIDTH, SCREENHEIGHT);
    window = glutCreateWindow ("TP HAI714I");

    init ();
    glutIdleFunc (idle);
    glutDisplayFunc (display);
    glutKeyboardFunc (key);
    glutReshapeFunc (reshape);
    glutMotionFunc (motion);
    glutMouseFunc (mouse);
    key ('?', 0, 0);

    //Unit sphere mesh loaded with precomputed normals
    openOFF("data/elephant_n.off", mesh.vertices, mesh.normals, mesh.triangles);

    mesh.computeNormals();

    basis = Basis();

    srand(time(NULL));
    Mat3 scale (2, 0., 0.,
                0.0, 0.5, 0.,
                0.0, 0., 1.);
    mesh_transformation.rotation = Mat3::RandRotation()*scale;

    mesh_transformation.translation = Vec3( -1.0 + 2.0 * ((double)(rand()) / (double)(RAND_MAX)),-1.0 + 2.0 * ((double)(rand()) / (double)(RAND_MAX)),-1.0 + 2.0 * ((double)(rand()) / (double)(RAND_MAX)) );

    normal_transformation = Mat3::inverse(mesh_transformation.rotation);
    normal_transformation.transpose();

    transformed_basis = Basis( mesh_transformation.translation,
                               mesh_transformation.rotation * basis.i,
                               mesh_transformation.rotation * basis.j,
                               mesh_transformation.rotation * basis.k );//Not normalized vectors

    for( unsigned int i = 0 ; i < mesh.vertices.size() ; ++i ) {
        transformed_mesh.vertices.push_back( mesh_transformation.rotation * mesh.vertices[i] + mesh_transformation.translation );
        Vec3 normal = normal_transformation * mesh.normals[i];
        normal.normalize();
        transformed_mesh.normals.push_back( normal );
    }

    for( unsigned int i = 0 ; i < mesh.triangles.size() ; ++i ) {
        Vec3 normal = normal_transformation * mesh.triangle_normals[i];
        normal.normalize();
        transformed_mesh.triangle_normals.push_back( normal );
    }

    transformed_mesh.triangles = mesh.triangles;

    for( unsigned int i = 0 ; i < 3 ; i++ ){
        project(mesh.vertices, projection_on_basis[i].positions, basis.origin, basis[i]);
    }


    for( unsigned int i = 0 ; i < 3 ; i++ ){
        planes[i] = Plane(basis.origin, basis[i] );
        transformed_planes[i].point = mesh_transformation.rotation*planes[i].point + mesh_transformation.translation;
        transformed_planes[i].normal = normal_transformation*planes[i].normal ;
        transformed_planes[i].normal.normalize();

        project(transformed_mesh.vertices, transformed_projection_on_basis[i].positions, transformed_basis.origin, transformed_basis[i]/transformed_basis[i].length());
    }

    Vec3 bbmin, bbmax, center;
    computeBBox(mesh.vertices, bbmin, bbmax );
    Vec3 bboxCenter = (bbmin + bbmax) / 2.f;

    setEllipsoide(ellipsoide, Basis( bboxCenter,
                                     0.5*(bbmax[0] - bbmin[0])*basis.i,
            0.5*(bbmax[1] - bbmin[1])*basis.j,
            0.5*(bbmax[2] - bbmin[2])*basis.k));

    for( unsigned int i = 0 ; i < ellipsoide.vertices.size() ; ++i ) {
        transformed_ellipsoide.vertices.push_back( mesh_transformation.rotation*ellipsoide.vertices[i] + mesh_transformation.translation );

        Vec3 normal = normal_transformation*ellipsoide.normals[i] ;
        normal.normalize();

        transformed_ellipsoide.normals.push_back( normal );
    }

    transformed_ellipsoide.triangles = ellipsoide.triangles;

    transformed_ellipsoide.computeTrianglesNormals();

    glutMainLoop ();
    return EXIT_SUCCESS;
}

