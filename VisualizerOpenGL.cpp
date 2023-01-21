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

void VisualizerOpenGL::DrawCube(){
    Matrix BaryCenter = CurrentInstance->m.GetState().c;
    Matrix RMatrix = CurrentInstance->m.GetState().R;

    std::vector<std::vector<double>> points(8, {0, 0, 0}); // инициализация точек куба
    std::vector<double> correction = {0, 0, 0};
    double side_size = CurrentInstance->m.GetParams().size_side;

    for (int i=0; i < 8; i++){
        correction[0] = (-0.5 + (i & 1)) * side_size;
        correction[1] = (-0.5 + (i & 2) / 2) * side_size;
        correction[2] = (-0.5 + (i & 4) / 4) * side_size;

        std::vector<double> ans = {0, 0, 0};
        for (int j = 0; j < 3; j++){
            ans[j] = correction[0] * RMatrix(j, 0) + correction[1] * RMatrix(j, 1) + correction[2] * RMatrix(j, 2);
        }
        correction = ans; // вычислили смещение от-но барицентра для каждой точки

        for (int j = 0; j < 3; j++){
            points[i][j] = BaryCenter(j, 0) + correction[j]; // получили итоговые координаты
        }

    }

    glBegin(GL_QUADS); // рисуем грани
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(points[6][0], points[6][1], points[6][2]);
    glVertex3f(points[2][0], points[2][1], points[2][2]);
    glVertex3f(points[3][0], points[3][1], points[3][2]);
    glVertex3f(points[7][0], points[7][1], points[7][2]);

// Bottom face (y = -1.0f)
    glColor3f(0.0f, 1.0f, 0.0f);     // Green
    glVertex3f(points[5][0], points[5][1], points[5][2]);
    glVertex3f(points[1][0], points[1][1], points[1][2]);
    glVertex3f(points[0][0], points[0][1], points[0][2]);
    glVertex3f(points[4][0], points[4][1], points[4][2]);

// Front face  (z = 1.0f)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(points[7][0], points[7][1], points[7][2]);
    glVertex3f(points[3][0], points[3][1], points[3][2]);
    glVertex3f(points[1][0], points[1][1], points[1][2]);
    glVertex3f(points[5][0], points[5][1], points[5][2]);

// Back face (z = -1.0f)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(points[4][0], points[4][1], points[4][2]);
    glVertex3f(points[0][0], points[0][1], points[0][2]);
    glVertex3f(points[2][0], points[2][1], points[2][2]);
    glVertex3f(points[6][0], points[6][1], points[6][2]);

// Left face (x = -1.0f)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(points[3][0], points[3][1], points[3][2]);
    glVertex3f(points[2][0], points[2][1], points[2][2]);
    glVertex3f(points[0][0], points[0][1], points[0][2]);
    glVertex3f(points[1][0], points[1][1], points[1][2]);

// Right face (x = 1.0f)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(points[6][0], points[6][1], points[6][2]);
    glVertex3f(points[7][0], points[7][1], points[7][2]);
    glVertex3f(points[5][0], points[5][1], points[5][2]);
    glVertex3f(points[4][0], points[4][1], points[4][2]);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
    gluLookAt( 20.0f, 20.0f, -5.0f,
               0.0f, 0.0f, 0.0f,
               0.0f, 0.0f, 8.0f);
    CurrentInstance->m.NextRK4();
    ShowFloor();
    glColor3f(1.0f, 0.5f, 0.0f);
    DrawCube();
    glPushMatrix();
    glPointSize(10);
    glBegin(GL_POINTS);
    glColor3f(0,125,0);
    Matrix center = CurrentInstance->m.GetState().c;
    glVertex3f(center(0, 0), center(1,0), center(2, 0));
    std::cout << center(0, 0) << " " << center(1, 0) << " " << center(2, 0) << "\n";
//    std::cout << "-------------------------\n";
//    CurrentInstance->m.GetState().R.print();
//    std::cout << "-------------------------\n";
    glEnd();
    glutSwapBuffers();
}

void VisualizerOpenGL::Init(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100,100);
    glutInitWindowSize(1024,768);
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