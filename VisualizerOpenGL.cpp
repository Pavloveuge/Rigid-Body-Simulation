#include "VisualizerOpenGL.h"

void VisualizerOpenGL::ChangeSize(int w, int h) {
    // check zero division
    if (h == 0)
        h = 1;
    float ratio =  w * 1.0 / h;
    // using projection matrix
    glMatrixMode(GL_PROJECTION);
    // clear matrix
    glLoadIdentity();
    // set viewport parameters
    glViewport(0, 0, w, h);
    // set correct perspective
    gluPerspective(45.0f, ratio, 0.1f, 100.0f);
    // back to projection matrix
    glMatrixMode(GL_MODELVIEW);
}

void VisualizerOpenGL::DrawCylinder(Matrix R){
    double radius = CurrentInstance->m.GetParams().radius;
    double height = CurrentInstance->m.GetParams().height;
    glBegin(GL_LINES);
    glColor3f(0.0f, .0f, 255.0f);
    for (float i = 0.0; i <= 2 * PI; i += 0.02){
        Matrix top(3, 1);
        Matrix bottom(3, 1);
        top(0, 0) =  cos(i)*radius; top(1, 0) = sin(i) * radius; top(2, 0) = height / 2;
        bottom(0, 0) =  cos(i)*radius; bottom(1, 0) = sin(i) * radius; bottom(2, 0) = -height / 2;
        top = R * top;
        bottom = R * bottom;
        glVertex3f(top(0, 0), top(1, 0), top(2, 0));
        glVertex3f(bottom(0, 0), bottom(1, 0), bottom(2, 0));
    }
    glEnd();
}

void VisualizerOpenGL::ShowFloor(){
    glColor3f(0.0f, 255.0f, 0.0f);
    glBegin(GL_QUADS);
    // Green
    glVertex3f(10.0f, 10.0f, 0.0f);
    glVertex3f(10.0f, -10.0f, 0.0f);
    glVertex3f(-10.0f, -10.0f, 0.0f);
    glVertex3f(-10.0f, 10.0f, 0.0f);
    glEnd();
}

void VisualizerOpenGL::RenderScene(){
    // clear buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // reset transformation
    glLoadIdentity();
    // set camera
    gluLookAt( 0.0f, 25.0f, -5.0f,
               0.0f, 0.0f, 0.0f,
               0.0f, 0.0f, 3.0f);
    CurrentInstance->m.NextRK4();
    ShowFloor();
    glColor3f(1.0f, 0.5f, 0.0f);
    glPushMatrix();
    glPointSize(10);
    glBegin(GL_POINTS);
    glColor3f(0,125,0);
    Matrix center = CurrentInstance->m.GetState().c;
    glVertex3f(center(0, 0), center(1,0), center(2, 0));
    std::cout << center(0, 0) << " " << center(1, 0) << " " << center(2, 0) << "\n";
    glEnd();
    glPointSize(1);
    glTranslatef(center(0, 0), center(1, 0), center(2, 0));
    DrawCylinder(CurrentInstance->m.GetState().R);
    glPopMatrix();
    glutSwapBuffers();
}

void VisualizerOpenGL::Init(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100,100);
    glutInitWindowSize(1920,1080);
    glutCreateWindow("Rigid-Body-Simulation");

    glutDisplayFunc(VisualizerOpenGL::RenderCallback);
    glutReshapeFunc(VisualizerOpenGL::ChangeCallback);
    glutTimerFunc(1, VisualizerOpenGL::TimerCallback, 0);

// main loop
    glutMainLoop();
}
void VisualizerOpenGL::RenderCallback(){
    CurrentInstance->RenderScene();
}

void VisualizerOpenGL::TimerCallback(int value){
    glutPostRedisplay();  // Redraw windows
    glutTimerFunc(1, TimerCallback, 0); // Setup next timer
}

void VisualizerOpenGL::ChangeCallback(int w, int h){
    CurrentInstance->ChangeSize(w, h);
}

VisualizerOpenGL::VisualizerOpenGL(){
    CurrentInstance = this;
    CurrentInstance->m = Model();
}

VisualizerOpenGL::VisualizerOpenGL(State& state, Params params){
    CurrentInstance = this;
    CurrentInstance->m = Model(state, params);
}