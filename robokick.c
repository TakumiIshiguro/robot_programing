/*
 * Robot Programming  Project 2 : Robo-Kick Simulator（シンプル版）
 *
 *   ロボットの脚がボールを蹴るシミュレーション
 *     - ボール : 重力, 転がり摩擦, 床・壁でのバウンド
 *     - 脚     : 股関節まわりに回転する振り子（トルクで制御）
 *     - キック : 足とボールの衝突（運動量 + 反発係数）
 *
 *   ビルド : make
 *   実行   : ./robokick
 *   操作   : k = キック,  r = リセット,  q = 終了
 */
/*
 * ---- OpenGL と GLUT について ----
 *   OpenGL : 図形を描くためのライブラリ。関数名は gl～ で始まる。
 *            （gluOrtho2D のように glu～ で始まるのは補助ライブラリ GLU の関数）
 *   GLUT   : ウィンドウを開く・キー入力を受け取る・一定時間ごとに関数を呼ぶ，
 *            などを行うライブラリ。関数名は glut～ で始まる。
 *
 *   GLUT のプログラムは「コールバック方式」で動く。
 *     main() で「画面を描くときはこの関数」「キーが押されたらこの関数」
 *     のように関数を登録し，最後に glutMainLoop() を呼ぶと，
 *     あとは GLUT が必要なときに登録した関数を呼び出してくれる。
 *
 *   このプログラムで登録している関数:
 *     display()  : 画面を描く
 *     reshape()  : ウィンドウの大きさが変わったとき（起動時にも1回呼ばれる）
 *     keyboard() : キーが押されたとき
 *     simu()     : 10ms ごと（シミュレーションを進める）
 */
#include <GL/glut.h>   /* GLUT（OpenGL と GLU も一緒に読み込まれる） */
#include <math.h>
#include <stdlib.h>

/* ================= 定数 ================= */
#define PI          3.14159265358979
#define G           9.8      /* 重力加速度 [m/s^2]        */
#define DT          0.001    /* シミュレーションの時間間隔 [s] */
#define STEPS       10       /* 1フレーム(10ms)あたりの計算回数 */

#define RHO         0.75     /* 反発係数                  */
#define MU_ROLL     0.08     /* 転がり摩擦係数            */

#define KICK_TORQUE 100.0    /* キック時の関節トルク [Nm] */
#define KP          600.0    /* PD制御 比例ゲイン         */
#define KD          50.0     /* PD制御 微分ゲイン         */

#define WALL_LEFT   (-0.8)   /* 左の壁の x 座標 [m]       */
#define WALL_RIGHT  ( 6.8)   /* 右の壁の x 座標 [m]       */

/* ================= データ構造 ================= */
typedef struct {
    double m, r, I;          /* 質量, 半径, 慣性モーメント */
    double x, y, tht;        /* 位置, 回転角              */
    double dx, dy, dtht;     /* 速度, 角速度              */
    double ddx, ddy, ddtht;  /* 加速度, 角加速度          */
} Ball;

typedef enum { STAND, BACKSWING, SWING } LegMode;

typedef struct {
    double hip_x, hip_y;     /* 股関節の位置              */
    double L;                /* 股関節から足の中心までの長さ */
    double foot_r;           /* 足（円）の半径            */
    double shin_w;           /* すね（棒の部分）の太さの半分 */
    double m, I;            /* 質量, 股関節まわりの慣性モーメント */
    double tht, dtht, ddtht; /* 関節角(真下=0, 前方が正), 角速度, 角加速度 */
    double tau;              /* 関節トルク                */
    LegMode mode;            /* 動作モード                */
} Leg;

/* ================= グローバル変数 ================= */
Ball ball;
Leg  leg;

/* ================= 初期化 ================= */
void init(void)
{
    ball.m = 0.43;
    ball.r = 0.11;
    ball.I = 2.0 / 3.0 * ball.m * ball.r * ball.r;   /* 中空の球 */
    ball.x = 0.30;   ball.y = ball.r;  ball.tht = 0.0;
    ball.dx = 0.0;   ball.dy = 0.0;    ball.dtht = 0.0;

    leg.hip_x = 0.0;  leg.hip_y = 0.95;
    leg.L = 0.87;
    leg.foot_r = 0.07;
    leg.shin_w = 0.035;
    leg.m = 6.0;
    leg.I = leg.m * leg.L * leg.L / 3.0;   /* 一様な棒を端で回す場合 */
    leg.tht = 0.0;  leg.dtht = 0.0;
    leg.mode = STAND;
}

/* 脚の上で，股関節から距離 s の点の位置（s = L なら足の中心） */
double leg_point_x(double s) { return leg.hip_x + s * sin(leg.tht); }
double leg_point_y(double s) { return leg.hip_y - s * cos(leg.tht); }

/* 足の中心位置 */
double foot_x(void) { return leg_point_x(leg.L); }
double foot_y(void) { return leg_point_y(leg.L); }

/* ================= 脚の制御 ================= */
/*   STAND     : 真下(0度)を保つ
 *   BACKSWING : 後ろ(-50度)へ振り上げる
 *   SWING     : 一定トルクで前へ振り抜く → 45度を超えたら STAND に戻る */
void control_leg(void)
{
    double target;

    switch (leg.mode) {
    case STAND:
        target  = 0.0;
        leg.tau = KP * (target - leg.tht) - KD * leg.dtht;
        break;

    case BACKSWING:
        target  = -50.0 * PI / 180.0;
        leg.tau = KP * (target - leg.tht) - KD * leg.dtht;
        if (leg.tht < -45.0 * PI / 180.0) leg.mode = SWING;
        break;

    case SWING:
        leg.tau = KICK_TORQUE;
        if (leg.tht > 45.0 * PI / 180.0) leg.mode = STAND;
        break;
    }
}

/* ================= 運動の計算 ================= */
int ball_is_rolling(void)
{
    return ball.y <= ball.r && ball.dy == 0.0;
}

void move_ball(void)
{
    double f_x = 0.0;
    double f_y = -ball.m * G;               /* 重力 */
    double tau_z = 0.0;                     /* ボールを回すトルク（なし） */

    if (ball_is_rolling()) {
        f_y = 0.0;                          /* 床からの抗力で重力と釣り合う */

        /* 転がり摩擦 f = mu m g（進行方向と逆向き） */
        double friction = MU_ROLL * ball.m * G;
        if (fabs(ball.dx) < friction / ball.m * DT)
            ball.dx = 0.0;                  /* 止まった */
        else if (ball.dx > 0.0)
            f_x = -friction;
        else
            f_x = friction;
    }

    ball.ddx   = f_x / ball.m;
    ball.ddy   = f_y / ball.m;
    ball.ddtht = tau_z / ball.I;

    ball.x   = ball.x   + ball.dx   * DT + ball.ddx   * DT * DT / 2.0;
    ball.y   = ball.y   + ball.dy   * DT + ball.ddy   * DT * DT / 2.0;
    ball.tht = ball.tht + ball.dtht * DT + ball.ddtht * DT * DT / 2.0;

    ball.dx   = ball.dx   + ball.ddx   * DT;
    ball.dy   = ball.dy   + ball.ddy   * DT;
    ball.dtht = ball.dtht + ball.ddtht * DT;

    /* 地面を転がっている間は滑らない : omega = -V / r */
    if (ball_is_rolling())
        ball.dtht = -ball.dx / ball.r;
}

void move_leg(void)
{
    /* I * ddtht = tau - (重力によるトルク) */
    double gravity_torque = leg.m * G * (leg.L / 2.0) * sin(leg.tht);

    leg.ddtht = (leg.tau - gravity_torque) / leg.I;
    leg.tht  = leg.tht  + leg.dtht * DT + leg.ddtht * DT * DT / 2.0;
    leg.dtht = leg.dtht + leg.ddtht * DT;
}

/* ================= 衝突 ================= */
/* 床・壁 : m V(T+dT) = -rho m V(T-dT) */
void bounce_ball(void)
{
    if (ball.y < ball.r && ball.dy < 0.0) {           /* 床 */
        ball.y  = ball.r;
        ball.dy = -RHO * ball.dy;
        if (ball.dy < 0.15) ball.dy = 0.0;            /* 小さな跳ねは止めて転がりへ */
    }
    if (ball.x > WALL_RIGHT - ball.r && ball.dx > 0.0) {  /* 右の壁 */
        ball.x  = WALL_RIGHT - ball.r;
        ball.dx = -RHO * ball.dx;
    }
    if (ball.x < WALL_LEFT + ball.r && ball.dx < 0.0) {   /* 左の壁 */
        ball.x  = WALL_LEFT + ball.r;
        ball.dx = -RHO * ball.dx;
    }
}

/* 脚とボール :
 *   mB VB(T+dT) + mL VL(T+dT) = rho ( mB VB(T-dT) + mL VL(T-dT) )
 * 接触方向(脚→ボール)の撃力 J を求めて，ボールと脚の速度を更新する。
 * 脚は股関節まわりに回転するので，脚の質量として等価質量を使う。
 *
 *   s      : 当たった点の股関節からの距離（足なら s = L）
 *   radius : その点での脚の太さ（足なら足の半径, すねなら太さの半分）
 *   戻り値 : 接触していれば 1, していなければ 0                       */
int hit_leg(double s, double radius)
{
    double nx = ball.x - leg_point_x(s);
    double ny = ball.y - leg_point_y(s);
    double dist = sqrt(nx * nx + ny * ny);

    if (dist > ball.r + radius) return 0;            /* 接触していない */
    nx /= dist;
    ny /= dist;                                      /* 単位ベクトル */

    /* 当たった点の速度（脚の回転による : 速さ = s * 角速度） */
    double leg_dx = s * leg.dtht * cos(leg.tht);
    double leg_dy = s * leg.dtht * sin(leg.tht);

    /* 接触方向の相対速度（負なら近づいている） */
    double v_rel = (ball.dx - leg_dx) * nx + (ball.dy - leg_dy) * ny;

    if (v_rel < 0.0) {
        /* 脚の点が進む向きと接触方向のなす角の cos */
        double c = cos(leg.tht) * nx + sin(leg.tht) * ny;

        /* 接触方向から見た脚の等価質量 mL = I / (s c)^2 （逆数で持つ） */
        double inv_m_ball = 1.0 / ball.m;
        double inv_m_leg  = (s * c) * (s * c) / leg.I;

        double J = -(1.0 + RHO) * v_rel / (inv_m_ball + inv_m_leg);

        ball.dx += J * nx * inv_m_ball;          /* ボールは +J を受ける */
        ball.dy += J * ny * inv_m_ball;
        leg.dtht -= J * s * c / leg.I;           /* 脚は -J を受ける     */
    }

    /* めり込みを解消する */
    double overlap = ball.r + radius - dist;
    ball.x += overlap * nx;
    ball.y += overlap * ny;
    if (ball.y < ball.r) ball.y = ball.r;
    return 1;
}

/* 足（先端の円）→ すね（棒の部分）の順に当たり判定をする */
void kick_ball(void)
{
    if (hit_leg(leg.L, leg.foot_r)) return;          /* 足に当たった */

    /* すねの上でボールに一番近い点を求める：
     * 股関節→ボールのベクトルを脚の向き (sin, -cos) に射影し，0～L に収める */
    double s = (ball.x - leg.hip_x) * sin(leg.tht)
             - (ball.y - leg.hip_y) * cos(leg.tht);
    if (s < 0.0)   s = 0.0;
    if (s > leg.L) s = leg.L;

    hit_leg(s, leg.shin_w);
}

/* ================= 描画 ================= */
/*
 * ---- OpenGL での図形の描き方 ----
 *   glColor3d(R, G, B);       これから描く色を指定（各 0.0～1.0。例: 1,0,0 = 赤）
 *   glBegin(図形の種類);       図形を描き始める
 *     glVertex2d(x, y);       頂点の座標を1つ指定（必要な数だけ並べる）
 *     ...
 *   glEnd();                  図形を描き終える
 *
 *   glBegin に渡す図形の種類:
 *     GL_LINES      : 頂点を2つずつ組にして線分を描く
 *     GL_LINE_STRIP : 頂点を順につないだ折れ線（最後と最初はつながない）
 *     GL_LINE_LOOP  : 頂点を順につないで閉じた輪郭（最後と最初もつなぐ）
 *     GL_POLYGON    : 頂点を順につないだ多角形を塗りつぶす
 *
 *   座標は reshape() の gluOrtho2D で決めた範囲（単位 m）で指定する。
 */

/* 中心 (cx, cy)，半径 r の円を描く。mode は GL_POLYGON か GL_LINE_LOOP */
void draw_circle(double cx, double cy, double r, GLenum mode)
{
    int i;
    glBegin(mode);
    for (i = 0; i < 36; i++) {       /* 円周上の36点を頂点にする（36角形で円を近似） */
        double a = 2.0 * PI * i / 36;
        glVertex2d(cx + r * cos(a), cy + r * sin(a));
    }
    glEnd();
}

/* 床と左右の壁を描く */
void draw_ground(void)
{
    glColor3d(0.0, 0.0, 0.0);        /* 黒 */
    glLineWidth(2.0);                /* 線の太さ（ピクセル） */
    glBegin(GL_LINE_STRIP);          /* 左の壁 → 床 → 右の壁 を1本の折れ線で描く */
    glVertex2d(WALL_LEFT, 3.0);
    glVertex2d(WALL_LEFT, 0.0);
    glVertex2d(WALL_RIGHT, 0.0);
    glVertex2d(WALL_RIGHT, 3.0);
    glEnd();
}

/* ボールを描く */
void draw_ball(void)
{
    glColor3d(0.7, 0.9, 1.0);        /* 水色で塗りつぶし */
    draw_circle(ball.x, ball.y, ball.r, GL_POLYGON);

    glColor3d(0.0, 0.0, 0.0);        /* 黒で輪郭 */
    draw_circle(ball.x, ball.y, ball.r, GL_LINE_LOOP);

    /* 回転が分かるように，中心から角度 tht の方向へ半径の線を1本描く */
    glBegin(GL_LINES);
    glVertex2d(ball.x, ball.y);
    glVertex2d(ball.x + ball.r * cos(ball.tht), ball.y + ball.r * sin(ball.tht));
    glEnd();
}

/* 脚を描く */
void draw_leg(void)
{
    /* すね：股関節から足までを，太さ 2*shin_w の長方形で描く（当たり判定と同じ太さ）
     *   (wx, wy) は脚に垂直な方向へ shin_w だけずらすベクトル */
    double wx = leg.shin_w * cos(leg.tht);
    double wy = leg.shin_w * sin(leg.tht);

    glColor3d(0.3, 0.5, 0.6);
    glBegin(GL_POLYGON);             /* 長方形の4つの角を順に指定して塗りつぶす */
    glVertex2d(leg.hip_x + wx, leg.hip_y + wy);
    glVertex2d(foot_x()  + wx, foot_y()  + wy);
    glVertex2d(foot_x()  - wx, foot_y()  - wy);
    glVertex2d(leg.hip_x - wx, leg.hip_y - wy);
    glEnd();

    /* 足：円（塗りつぶし + 黒い輪郭） */
    glColor3d(0.5, 0.7, 0.8);
    draw_circle(foot_x(), foot_y(), leg.foot_r, GL_POLYGON);
    glColor3d(0.0, 0.0, 0.0);
    draw_circle(foot_x(), foot_y(), leg.foot_r, GL_LINE_LOOP);

    /* 股関節：小さな黒い円 */
    draw_circle(leg.hip_x, leg.hip_y, 0.04, GL_POLYGON);
}

/* 画面を描く（GLUT から呼ばれる。glutDisplayFunc で登録）
 *   glClear で前の絵を消してから全部を描き直す。
 *   ダブルバッファ（main の GLUT_DOUBLE）を使っているので，絵は裏の画面に描かれ，
 *   最後の glutSwapBuffers で表と裏を入れ替えて表示する。
 *   こうすると描いている途中の絵が見えず，ちらつかない。                     */
void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT);    /* 画面を背景色（glClearColor の色）で塗りつぶす */
    draw_ground();
    draw_leg();
    draw_ball();
    glutSwapBuffers();               /* 描き終えた裏の画面を表に出す */
}

/* ================= シミュレーションループ ================= */
/* 10ms ごとに呼ばれ，DT 刻みで STEPS 回だけ運動を計算する。
 *   glutTimerFunc で登録した関数は「1回だけ」呼ばれるので，
 *   最後に自分自身をもう一度登録して，10ms ごとに繰り返し呼ばれるようにする。
 *   引数 value は glutTimerFunc の3番目の引数がそのまま渡される（ここでは使わない）。 */
void simu(int value)
{
    int i;
    for (i = 0; i < STEPS; i++) {
        control_leg();    /* 制御入力       */
        move_leg();       /* 脚の運動       */
        move_ball();      /* ボールの運動   */
        kick_ball();      /* 足とボールの衝突 */
        bounce_ball();    /* 床・壁との衝突 */
    }
    glutPostRedisplay();             /* 「画面を描き直して」と GLUT に頼む → display() が呼ばれる */
    glutTimerFunc(10, simu, 0);      /* 10ms 後にまた simu() を呼ぶよう登録する */
}

/* ================= ユーザコマンド ================= */
/* キーが押されたときに呼ばれる（glutKeyboardFunc で登録）
 *   key  : 押されたキーの文字
 *   x, y : そのときのマウスの位置（ウィンドウ上のピクセル座標。ここでは使わない） */
void keyboard(unsigned char key, int x, int y)
{
    switch (key) {
    case 'k':
        if (leg.mode == STAND) leg.mode = BACKSWING;
        break;
    case 'r':
        init();
        break;
    case 'q':
        exit(0);
    }
}

/* ウィンドウの大きさが変わったときに呼ばれる（glutReshapeFunc で登録）
 *   w, h : 新しいウィンドウの幅と高さ（ピクセル）
 *   ここで「ウィンドウのどこに，どの座標範囲を表示するか」を決める。 */
void reshape(int w, int h)
{
    glViewport(0, 0, w, h);          /* ウィンドウ全体を描画に使う */

    glMatrixMode(GL_PROJECTION);     /* これから「表示範囲」の設定をする */
    glLoadIdentity();                /* 前の設定をリセット */
    gluOrtho2D(-1.0, 7.0, -0.5, 3.5);   /* 表示範囲: x は -1～7 m, y は -0.5～3.5 m
                                           （左端, 右端, 下端, 上端 の順） */
    glMatrixMode(GL_MODELVIEW);      /* 設定の対象を「図形の配置」に戻しておく */
}

int main(int argc, char *argv[])
{
    /* ---- ウィンドウを作る ---- */
    glutInit(&argc, argv);                         /* GLUT の初期化（最初に必ず呼ぶ） */
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);  /* カラー表示 + ダブルバッファ */
    glutInitWindowSize(1000, 500);                 /* ウィンドウの大きさ（ピクセル） */
    glutCreateWindow("Robo-Kick Simulator");       /* ウィンドウを作る（引数はタイトル） */
    glClearColor(1.0, 1.0, 1.0, 1.0);              /* 背景色を白にする（R, G, B, 不透明度） */

    init();                                        /* ボールと脚の初期状態を設定 */

    /* ---- コールバック関数を登録する ---- */
    glutDisplayFunc(display);        /* 画面を描くときは display() */
    glutReshapeFunc(reshape);        /* ウィンドウの大きさが変わったら reshape() */
    glutKeyboardFunc(keyboard);      /* キーが押されたら keyboard() */
    glutTimerFunc(10, simu, 0);      /* 10ms 後に simu() を呼ぶ（引数: 待ち時間[ms], 関数, simu に渡す値） */

    /* ---- イベントループ開始 ----
     * ここから先は GLUT が，登録した関数を必要なときに呼び出し続ける。
     * この関数からは戻ってこない（終了は keyboard() の exit(0)）。 */
    glutMainLoop();
    return 0;
}
