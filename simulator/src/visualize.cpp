#include "visualize.hpp"
#include "linkconf_ver2.hpp"

#define POLYGON_N 24
#define RED 0.8, 0.16, 0.21
#define BLUE 0.11, 0.35, 0.8
#define ORANGE 0.9, 0.5, 0.15


using namespace std;
using namespace std::chrono;

extern Timer system_time;

Simulation* Visualize::v_simulation;
stateClass* Visualize::state_visu;
double Visualize::time_current;
double Visualize::time_pre;
double Visualize::ratio;


Visualize::Visualize(){
  
  time_current = 0;
  time_pre = 0;
  ratio = IMAGE_WIDTH /IMAGE_HEIGHT;
}

void Visualize::display()
{ 
  time_pre = time_current;
  time_current = v_simulation->timer;
  if(time_current != time_pre){//位置ループ前の時刻と現在時刻が異なるなら実行
    printSimulation();
  }
}

void Visualize::printSimulation()
{
  
  double spring_angle = 135;
  glClear(GL_COLOR_BUFFER_BIT);//バッファの消去(glClearColorで指定した色で塗りつぶす)

//頂点の指定
  glColor3d(BLUE);//色の指定
  glPointSize(12);//サイズの指定
  glBegin(GL_POINTS);//位置指定(GL_POINTS:各頂点を単独の点として扱う)
  glVertex2d(state_visu->joint[0].x*PLOT_RATE/ratio+X_DISP, state_visu->joint[0].z*PLOT_RATE+Y_DISP);//(x,y)
  glVertex2d(state_visu->joint[1].x*PLOT_RATE/ratio+X_DISP, state_visu->joint[1].z*PLOT_RATE+Y_DISP);
  glVertex2d(state_visu->joint[2].x*PLOT_RATE/ratio+X_DISP, state_visu->joint[2].z*PLOT_RATE+Y_DISP);
  glEnd();//処理の終了

//線の指定(下腿リンク)
  glLineWidth(6);//線の太さを指定
  glColor3d(BLUE);
  glBegin(GL_LINE_STRIP);//最初の頂点から最後の頂点まで、線分を連結して描画
  glVertex2d(state_visu->joint[0].x*PLOT_RATE/ratio+X_DISP, state_visu->joint[0].z*PLOT_RATE+Y_DISP);//足首関節
  glVertex2d(state_visu->joint[1].x*PLOT_RATE/ratio+X_DISP, state_visu->joint[1].z*PLOT_RATE+Y_DISP);//膝関節
  glEnd();

//大腿リンク
  glLineWidth(6);
  glColor3d(RED);
  glBegin(GL_LINE_STRIP);
  glVertex2d(state_visu->joint[1].x*PLOT_RATE/ratio+X_DISP, state_visu->joint[1].z*PLOT_RATE+Y_DISP);
  glVertex2d(state_visu->joint[2].x*PLOT_RATE/ratio+X_DISP, state_visu->joint[2].z*PLOT_RATE+Y_DISP);
  glEnd(); 

  printGround();//地面の描画

//タイヤ
  glLineWidth(4);
  glColor3d(BLUE);
  printCircle(POLYGON_N, state_visu->joint[0].x*PLOT_RATE/ratio+X_DISP, state_visu->joint[0].z*PLOT_RATE+Y_DISP,WHEEL_R);
  glEnd();

//モータリンク
  glLineWidth(6);
  glColor3d(ORANGE);
  glBegin(GL_LINE_STRIP);
  glVertex2d(state_visu->joint[1].x*PLOT_RATE/ratio+X_DISP, state_visu->joint[1].z*PLOT_RATE+Y_DISP);
  glVertex2d(state_visu->joint[1].x+0.05*cos(state_visu->pose[3]+state_visu->pose[2])*PLOT_RATE/ratio+X_DISP, state_visu->joint[1].z+0.05*sin(state_visu->pose[3]+state_visu->pose[2])*PLOT_RATE+Y_DISP);
  glEnd(); 

  glLineWidth(6);
  glColor3d(0, 0.6, 0);
  glBegin(GL_LINE_STRIP);
  glVertex2d(state_visu->joint[2].x*PLOT_RATE/ratio+X_DISP, state_visu->joint[2].z*PLOT_RATE+Y_DISP);
  glVertex2d(state_visu->joint[2].x+L_3*cos(state_visu->pose[5]+state_visu->pose[4]+state_visu->pose[3]+state_visu->pose[2])*PLOT_RATE/ratio+X_DISP, state_visu->joint[2].z+L_3*sin(state_visu->pose[5]+state_visu->pose[4]+state_visu->pose[3]+state_visu->pose[2])*PLOT_RATE+Y_DISP);
  glEnd(); 

  
  // glLineWidth(10);
  // glColor3d(1.0, 0.4, 0.0);
  // for(int i=0;i<3; i++)
  // {
  //   printArc(state_visu->joint[i+2].x, state_visu->joint[i+2].z,WHEEL_R*0.8,state_visu->torque[2-i]/10.);
  // }

  glColor3d(ORANGE);
  printArc(state_visu->joint[1].x, state_visu->joint[1].z,WHEEL_R*0.8,-state_visu->external_forces[4]);
  // glColor3d(0.0, 0.8, 0.5);
  // if(abs(state_visu->springTorque)>0.01){
  //   printArc(state_visu->joint[1].x, state_visu->joint[1].z,WHEEL_R*0.8,-state_visu->springTorque/(0.9/5.));
  // }
  glColor3d(0.0, 0.8, 0.5);
  // if(abs(state_visu->external_forces[4])>0.01){
  //   printArc(state_visu->joint[1].x, state_visu->joint[1].z,WHEEL_R*0.8,-state_visu->external_forces[4]);
  // }
  // printArc(state_visu->joint[1].x, state_visu->joint[1].z,WHEEL_R*0.8,-state_visu->external_forces[4]*100);

  glColor3d(0.0, 0.0, 1.0);
  stringstream time,jt,ft,mt,bt,st,rw;
  time << "TIME : " <<  v_simulation->timer <<" [sec]";
  jt << "LINK JOINT MOTOR TORQUE : " << state_visu->external_forces[4] <<" [N m]"; 
  ft << "WHEEL MOTOR TORQUE : " <<  state_visu->torque<<" [N m]";
  // st << "Spring Torque: " << state_visu->springTorque;
 
  double font_pose = 20;
  DrawString(time.str(), IMAGE_WIDTH, IMAGE_HEIGHT, font_pose, font_pose);
  DrawString(jt.str(), IMAGE_WIDTH, IMAGE_HEIGHT, font_pose, font_pose*2);
  DrawString(ft.str(), IMAGE_WIDTH, IMAGE_HEIGHT, 20, 60);
  DrawString(mt.str(), IMAGE_WIDTH, IMAGE_HEIGHT, 20, 80);
  DrawString(bt.str(), IMAGE_WIDTH, IMAGE_HEIGHT, 20, 100);
  // if(abs(state_visu->springTorque)>0.01)DrawString(st.str(), IMAGE_WIDTH, IMAGE_HEIGHT, 20, 100);
  // DrawString(st.str(), IMAGE_WIDTH, IMAGE_HEIGHT, 20, 100);
  glFlush();  
}

void Visualize::printCircle(int n, double x, double y, double r){
  double rate;
   glBegin(GL_LINE_STRIP); // ポリゴンの描画
  // 円を描画
  for (int i = 0; i < n+1; i++) {
    // 座標を計算
    rate = (double)i / n;
    double _x = r * cos(2.0 * M_PI * rate) * PLOT_RATE / RATIO_XY  + x ;
    double _y = r * sin(2.0 * M_PI * rate) *PLOT_RATE + y;
    glVertex3f(_x, _y, 0.0); // 頂点座標を指定
  }
  glEnd(); 
}

void Visualize::printArc(double x,double y,double r,double rad){
  double rate;
  int n = 32;
  glBegin(GL_LINE_STRIP); // ポリゴンの描画
  // 円を描画
  rad = rad * 2*M_PI;
  for (int i = 0; i < n+1; i++) {
    // 座標を計算
    rate = (double)i / n;
    double _x = r * cos(rad * rate) * PLOT_RATE / RATIO_XY  + x *PLOT_RATE/ratio+X_DISP;
    double _y = r * sin(rad * rate) * PLOT_RATE + y*PLOT_RATE+Y_DISP;
    glVertex3f(_x, _y, 0.0); // 頂点座標を指定
  }
  glEnd(); 
  double arrowSize = 0.013;
  if(rad<0) arrowSize *= -1;
  if(rad==0) arrowSize = 0;
  double _x = (r) * PLOT_RATE / RATIO_XY  + x *PLOT_RATE/ratio+X_DISP;
  double _x_1 = (r+arrowSize) * PLOT_RATE / RATIO_XY  + x *PLOT_RATE/ratio+X_DISP;
  double _x_2 = (r-arrowSize) * PLOT_RATE / RATIO_XY  + x *PLOT_RATE/ratio+X_DISP;
  double _y = (y-arrowSize) *PLOT_RATE+Y_DISP;
  glBegin(GL_TRIANGLES);
  glVertex2d(_x_1, y*PLOT_RATE+Y_DISP);
  glVertex2d(_x_2, y*PLOT_RATE+Y_DISP);
  glVertex2d(_x, _y);
  glEnd(); 
}

void Visualize::printGround(){
  glLineWidth(6);
  glColor3d(0, 1.0, 0);
  glBegin(GL_LINE_STRIP);
  glVertex2d(-1, Y_DISP);
  glVertex2d( STEP_X*PLOT_RATE/ratio+X_DISP, 0*PLOT_RATE+Y_DISP);
  // glEnd();

  // glBegin(GL_LINE_STRIP);
  glVertex2d(STEP_X*PLOT_RATE/ratio+X_DISP, STEP_HIGHT*PLOT_RATE+Y_DISP);
  glVertex2d(1, STEP_HIGHT*PLOT_RATE+Y_DISP);
  glEnd();  
}

// void Visualize::printWheel(void){
//   glLineWidth(6);
//   glColor3d(0, 1.0, 0);
//   glBegin(GL_LINE_STRIP);
// }

void Visualize::DrawString(string str, int w, int h, int x0, int y0)
{
    glDisable(GL_LIGHTING);
    // 平行投影にする
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, w, h, 0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // 画面上にテキスト描画
    glRasterPos2f(x0, y0);
    int size = (int)str.size();
    for(int i = 0; i < size; ++i){
        char ic = str[i];
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ic);
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void Visualize::printWheelTorque(double x, double y, double torque){

}

void Visualize::init(void)
{
  glClearColor(0.7, 0.7, 0.7, 1.0);//背景色(red,green,blue,透明度)
}

void Visualize::idle(void)
{ 
  glutPostRedisplay();//glutDisplayFunc()を一度だけ実行（displayを一度だけ実行）

}

int Visualize::visualize(int argc, char *argv[], Simulation& _simulation)//メイン関数
{ 
  v_simulation = &_simulation;
  state_visu = &_simulation.currentState;
  time_pre = time_current;
  time_current = _simulation.timer;
  glutInitWindowPosition(800, 50);//ウィンドウ位置の指定
  glutInitWindowSize(IMAGE_WIDTH, IMAGE_HEIGHT);//ウィンドウサイズの指定
  glutInit(&argc, argv);//環境の初期化
  glutInitDisplayMode(GLUT_RGBA);//ディスプレイモードの指定(カラーモードをRGBAに指定)
  glutCreateWindow(argv[0]);//ウインドウの作成
  glutDisplayFunc(display);//描画時に呼び出される関数を指定(関数名:display)
  init();//背景色を設定
  // glutPostRedisplay();
  glutIdleFunc(idle);//プログラムアイドル状態時に呼び出される関数
  printSimulation();
  glutMainLoop();//OpenGLの終了
  return 0;
}