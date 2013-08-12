#include <iostream>
using namespace std;

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <signal.h>
#include <ypspur.h>
#include <math.h>
#include <ssm.hpp>
#include <string.h>
#include <ssmtype/spur-odometry.h>

#include "ssm-laser.hpp"
#include "ConfigManager.hpp"
#include "qstream.hpp"

#define BITFLAG_WAYPOINT	0x01
#define BITFLAG_STOPPOINT	0x02
#define BITFLAG_GOALPOINT 0x04
// 04 08 10 20 40 80 ...
//ただの通過地点		A
//一時停止地点		C
//ゴール					E

#define RAD(deg) ((deg)*M_PI/180) //deg→rad変換
#define DEG(rad) ((rad)*180/M_PI) //rad→deg変換
#define SAFETY_MARGIN_X 0.6
#define SAFETY_MARGIN_Y 0.25

void trans(void);

////なぜかこっちに書かないとeclipseでエラーが出る....コンパイルはできるけど
//// ---> waypoint
//class waypoint
//{
//public:
//	double x;
//	double y;
//	unsigned char bitflag;
//
//	waypoint(){}
//	~waypoint(){}
//	waypoint(const waypoint &obj){
//		this->bitflag = obj.bitflag;
//		this->x = obj.x;
//		this->y = obj.y;
//	}
//	void show(void){
//		printf("x:%.3lf\ty:%.3lf\t", x, y);
////		printf("bitflag:%c=0x%x=", bitflag, bitflag );
////		// ---> ビット表示
////		int i;
////		for(i=128; i; i >>= 1){
////			if(i & bitflag){ cout << "1"; }
////			else{	cout <<"0"; }
////		}
////		// <--- ビット表示
//		cout << " way:" << iswaypoint()
//				<< " stop:" << isstoppoint()
//				<< " goal:" << isgoalpint() << endl;
//	}
//
////	bool iswaypoint(void){ return ((bitflag & BITFLAG_WAYPOINT) && BITFLAG_WAYPOINT); }
//	bool iswaypoint(void){ return (bitflag & BITFLAG_WAYPOINT); }	//こっちでいいはず
//	bool isstoppoint(void){ return (bitflag & BITFLAG_STOPPOINT); }
//	bool isgoalpint(void){ return (bitflag & BITFLAG_GOALPOINT); }
//
//};
//// <--- waypoint

FILE *fp; //keiro.dat読み込み用
//const char *f_path = "../../../TKG/RouteEditor/keiro.dat";	//デフォルトの経路データの在り処:これ変えろ

const char *f_path[3] = {"keiro0.dat", "keiro1.dat", "keiro2.dat"};
bool isSpecialPath = false;

int dest = 1;	//これは常に次の向かう通過地点(NEW!!)
int route = 0;		//これは常に次の向かう通過地点の含まれる経路(NEW!!)

SSMApi<Spur_Odometry>	ssm_odom;	// ポテンシャル法用のオドメトリ

int sokuiki_fs_id = 0;
bool speedy = false;

const char *configure_file_name = "sample.conf";	//configファイルのデフォルトの在り処
string _config_filepath = "DefaultLoop.conf";
bool output_sample_conf = false;	//経路ファイルまでのパス
//vector< vector<waypoint> > waypoint_mat;	//２次元配列のwaypoint
double th;	//ロボットの姿勢

// ---> configuration
double vel = 0.3;
double accel = 0.5;
double angvel = 1.0;
double angaccel = 1.0;
double detection_angle = 180.0;	//degree
string pipe_path;
double safety_margin_x = 0.6;		//ロボットの中心から前方の距離
double safety_margin_y = 0.25;	//この２倍の幅が範囲
double Pot_dist = 2.0;			//ポテンシャル法dection distance
double k_gravity = 1.0;			//引力係数
double k_repulsion = 1.0;		//斥力係数
// <--- configuration

int timeover = 300;	//ポテンシャル法発生まで待つ時間(sec)

iqstream qin;					//一時停止解除のパイプ
SSMSOKUIKIData3D fs;	//衝突回避のための測域センサ情報
SSMApi<int> sound0("sound", 0);

//SSMに流す経路情報 現在向かっているwaypointの経路とwaypoint
#define SNAME_PINFO "Pinfo"
typedef struct {

	int route;
	int waypoint;

}route_waypoint;

SSMApi<route_waypoint> Pinfo(SNAME_PINFO, 1);	//経路情報を流す

// ---> send play sound que
void sound_play(int i){
	sound0.data = i;
	sound0.write();
}
// <--- send play sound que

void wait_restart_key(void);

// ---> spur safety stop
void my_stop(void){	//ロボットがそのばその姿勢でほぼ完全に停止する関数
	double v, w;

	Spur_stop();
	Spur_get_vel(&v, &w);
	while(fabs(v) > 0.01 || fabs(w) > 0.01 ){
		usleepSSM(10000);
		Spur_get_vel(&v, &w);
	}
}
// <--- spur safety stop

// ---> 途中停止・終了
void ctrlC(int aStatus){
	my_stop();
	Spur_free();

	Pinfo.release();
	sound0.release();
	fs.close();
	endSSM();

	printf("ctrl-C!\n");
	signal(SIGINT, NULL);
	exit(aStatus);
}
void setSigInt(){
	signal(SIGINT, ctrlC);
}
// <--- 途中停止・終了

// --->safety detection
int is_safety(void){	//指定範囲内に障害物があるかどうかの判定。
											//障害物あり : flg=1. 障害物なし : flg=0
	int i;
	int flg = 0;
	fs.readLast();
	for(i=0; i < (int)fs.property.numPoints; i++){
		if(fs.data[i].status == ssm::laser::STATUS_NO_REFLECTION)continue;
		if(fs.data[i].isError() == false
				&& fs.data[i].reflect.x > 0.24
				&& fs.data[i].reflect.x < safety_margin_x
				&& fabs(fs.data[i].reflect.y) < safety_margin_y){
			flg = 1;
//			return 1;
		}
	}
	return flg;
}
// <--- safety detection

// --- > option analize
bool optAnalize(int argc, char **argv){
	int opt;

	while((opt = getopt(argc, argv, "f:F:t:g:i:shG")) != -1){
		switch(opt)
		{
		case 'h' : {	//随時更新していくこと
			cerr
			<< "HELP!" << endl
			<< "ヘルプの館へようこそ..." << endl
			<< "-h \t\t: このヘルプを召喚する" << endl
			<< "-f ROUTE FILE NAME\t: 経路ファイル(keiro.dat)を指定して実行する" << endl
			<< "-t WAYPOINT NUMBER\t: スタート地点を指定の通過地点に変更(スタート時のロボットの姿勢は進行方向に設定されます)" << endl
			<< "-g CONFIGURE FILE NAME\t: configファイルを指定して実行する" << endl
			<< "-i SOKUIKI_FS SSM ID\t: 衝突回避のためのsokuiki_fsのSSM id" << endl
			<< "-s \t\t: waypointで一時停止しない(安全確認のための一時停止地点では停止)" << endl
			<< "-G \t\t: サンプルconfigファイルおげええ" << endl

			<< "e.g) ./PathPlanner -f keiro.dat -g sample.conf" << endl

			<< "それではさらばだ" << endl;
			return false;
		}break;

		case 'f' : {
			f_path[0] = optarg;
			cerr << "経路ファイル指定 : " << f_path[0] << endl;
		}break;

		case 't' : {
			dest = 1 + atoi(optarg);
			cerr << "スタート地点指定 : " << dest - 1 << endl;
		}break;

		case 'g' : {
			_config_filepath = optarg;

			string config_filepath = _config_filepath;
			configure_file_name = config_filepath.c_str();
			cerr << "configファイル指定 : "  << configure_file_name << endl;
		}break;

		case 'i' : {
			sokuiki_fs_id = atoi(optarg);
			cerr << "sokuiki_fs id 指定 : " << sokuiki_fs_id << endl;
		}break;

		case 's' : {
			speedy == true;
			cerr << "スピードモード！！" << endl;
		}break;

		case 'G' : {
			output_sample_conf = true;
		}break;

		default : {	//以上なオプションがついてる場合、警告して終了
			cerr
			<< "なんかヘンなオプションがついてるからとりあえず終了するね"<< endl
			<< "'-h'ってやってみて"<< endl;
			return false;
		}break;
		}
	}
	return true;	//オプションに異常なし!
}
// <---  option analize

// ---> Cofiguration Analyze with MTM programs
void ConfigAnalyze(const char *filename, bool OutputSample){
	mtm::ConfigManager config;	//コンフィグクラス
	config.SetDoubleArgument("YPSpur max velocity : ");
	config.SetDoubleArgument("YPSpur max acceleration : ");
	config.SetDoubleArgument("YPSpur max angle velocity : ");
	config.SetDoubleArgument("YPSpur max angle acceleration : ");
	config.SetDoubleArgument("YPSpur Spur_near_pos detection angle : ");
	config.SetStringArgument("restart que pipe path : ");
	config.SetDoubleArgument("Pre-Crash Safety Margin x : ");
	config.SetDoubleArgument("Pre-Crash Safety Margin y : ");
	config.SetDoubleArgument("Potential method repulsion detection distance : ");
	config.SetDoubleArgument("Potential method gravity gain : ");
	config.SetDoubleArgument("Potential method repulsion gain : ");
	config.SetIntArgument("Potential method launch time : ");
	config.SetIntArgument("sokuiki_fs SSM id : ");
	if( OutputSample ){
		config.OutputSampleFile("sample.conf");			//OutPutSampleがtrueのときサンプルファイル出力
	}else{
		config.Read(filename);
		config.GetDoubleArgument("YPSpur max velocity : ", &vel, 0.3);
		cout << "YPSpur max velocity : " << vel << endl;
		config.GetDoubleArgument("YPSpur max acceleration : ", &accel, 0.5);
		cout << "YPSpur max acceleration : " << accel << endl;
		config.GetDoubleArgument("YPSpur max angle velocity : ", &angvel, 1.0);
		cout << "YPSpur max angle velocity : " << angvel << endl;
		config.GetDoubleArgument("YPSpur max angle acceleration : ", &angaccel, 1.0);
		cout << "YPSpur max angle acceleration : " << angaccel << endl;
		config.GetDoubleArgument("YPSpur Spur_near_pos detection angle : ", &detection_angle, 10.0);
		cout << "YPSpur Spur_near_pos detection angle : " << detection_angle << endl;
		config.GetStringArgument("restart que pipe path : ", &pipe_path, "../../../data/pipe");
		cout << "restart que pipe path : " << pipe_path << endl;
		config.GetDoubleArgument("Pre-Crash Safety Margin x : ", &safety_margin_x, 0.6);
		cout << "Pre-Crash Safety Margin x : " << safety_margin_x << endl;
		config.GetDoubleArgument("Pre-Crash Safety Margin y : ", &safety_margin_y, 0.25);
		cout << "Pre-Crash Safety Margin y : " << safety_margin_y << endl;
		config.GetDoubleArgument("Potential method repulsion detection distance : ", &Pot_dist, 2.0);
		cout << "Potential method repulsion detection distance : " << Pot_dist << endl;
		config.GetDoubleArgument("Potential method gravity gain : ", &k_gravity, 1.0);
		cout << "Potential method gravity gain : " << k_gravity << endl;
		config.GetDoubleArgument("Potential method repulsion gain : ", &k_repulsion, 1.0);
		cout << "Potential method repulsion gain : " << k_repulsion << endl;
		config.GetIntArgument("Potential method launch time : ", &timeover, 180);
		cout << "Potential method launch time : " << timeover << endl;
		config.GetIntArgument("sokuiki_fs SSM id : ", &sokuiki_fs_id, 0);
		cout << "sokuiki_fs SSM id : " << sokuiki_fs_id << endl;

	}
}
// <--- Cofiguration Analyze with MTM programs





/*// ---> waypoint
class waypoint
{
public:
	double x;
	double y;
	unsigned char bitflag;

	waypoint(){}
	~waypoint(){}
	waypoint(const waypoint &obj){
		this->bitflag = obj.bitflag;
		this->x = obj.x;
		this->y = obj.y;
	}
	void show(void){
		printf("x:%.3lf\ty:%.3lf\t", x, y);
		printf("bitflag:%c=0x%x=", bitflag, bitflag );
		// ---> ビット表示
		int i;
		for(i=128; i; i >>= 1){
			if(i & bitflag){ cout << "1"; }
			else{	cout <<"0"; }
		}
		// <--- ビット表示
		cerr << " way:" << iswaypoint()
				<< " stop:" << isstoppoint()
				<< " goal:" << isgoalpint() << endl;
	}

	bool iswaypoint(void){ return ((bitflag & BITFLAG_WAYPOINT) && BITFLAG_WAYPOINT); }
	bool isstoppoint(void){ return ((bitflag & BITFLAG_STOPPOINT) && BITFLAG_STOPPOINT); }
	bool isgoalpint(void){ return ((bitflag & BITFLAG_GOALPOINT) && BITFLAG_GOALPOINT); }

};
*/// <--- waypoint

