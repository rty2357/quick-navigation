//============================================================================
// Name        : TC12_PathPlanner.cpp
// Author      : YSD
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//
//まずは最も簡単なものから作る
//
//プロセス起動時にkeiro.datを全部読み込むようにする。（構造体の配列）[完了]
//衝突回避[完了]
//途中停止・終了[完了]
//通過地点で停止してそのば回転(rotate)[完了]
//現在向かっているwaypointをSSMに流す[完了]
//最後にありえない行を読み込んでしまうのを直すべし[たぶんなおった]
//URGハンドラー（ペデ組）を導入したい。→coordinate-converterが必要[完了]
//衝突回避の検知範囲を新たにす[とりあえずURGより前にした]←M1のみ対応となるっよおよよｙ←Loopもだいたい同じでOK
//urg-coordinate-converterが流す情報を新たに利用するようにする[完了]
//変数名を新たにする(waypoint(struct keiro), route(keiro), num_waypoints(keiro_num), dest(keiro))[完了]
//
//次にやること
//とりあえず一時停止を入れるべし[完了?]←デバッグせよ
//
//複数経路(障害物回避)を考える
//・複数経路を読み込むようにする[完了]
//・他の経路に移るタイミングとその処理[完了]
//・他の経路のどの経路のどの通過地点に移るアルゴリズム（評価）と計算
//・その経路のその通過地点に移る処理を考える
//
//vel accel とか configファイルに入れる[完了]
//障害物回避の範囲
//
//障害物回避の指定範囲をロボット正面から経路上に変更?←もう少し考えよ
//
//オプション( -? で実行)
//途中の経路番号からスタート（地点の情報を入れる）[完了]
//途中の地点を入力し、最近点の経路を検索してスタート(いるか？)
//keiro.datを読み込むときにコメント(#)をとばす機能を追加したい←いらない?
//経路その２，その３に映るときの動作
//ctr-cで停止	するときに(日付時刻?)位置、姿勢、向かっていた通過地点をはきだすようにする。それを最後に表示
//起動オプションでそれを読み込んで途中からスタートできるようにする。
//
//けすぶぶん　きのう
//keiro.datを読み込むのを起動オプションのみにする
//複数経路を単一経路に
//
//============================================================================

#include "YSD_PP_util.hpp"

//なぜかこっちに書かないとeclipseでエラーが出る....コンパイルはできるけど
// ---> waypoint
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
	//		printf("bitflag:%c=0x%x=", bitflag, bitflag );
	//		// ---> ビット表示
	//		int i;
	//		for(i=128; i; i >>= 1){
	//			if(i & bitflag){ cout << "1"; }
	//			else{	cout <<"0"; }
	//		}
	//		// <--- ビット表示
	cout << " way:" << iswaypoint()
	<< " stop:" << isstoppoint()
	<< " goal:" << isgoalpint() << endl;
    }

    //	bool iswaypoint(void){ return ((bitflag & BITFLAG_WAYPOINT) && BITFLAG_WAYPOINT); }
    bool iswaypoint(void){ return (bitflag & BITFLAG_WAYPOINT); }	//こっちでいいはず
    bool isstoppoint(void){ return (bitflag & BITFLAG_STOPPOINT); }
    bool isgoalpint(void){ return (bitflag & BITFLAG_GOALPOINT); }

};
// <--- waypoint

vector< vector<waypoint> > waypoint_mat;	//２次元配列のwaypoint


int main(int argc, char *argv[]) {

    // ---> initialize
    // ---> option analize
    cout << "option analize..." << endl;
    if(!optAnalize(argc, argv)){
	return -1;	//オプションに異常がある場合、終了
    }
    // <--- option analize

    // ---> configuration file analize
    cout <<  "confguration analyze... : " << configure_file_name << endl;
    ConfigAnalyze(configure_file_name, output_sample_conf);
    if(output_sample_conf) return 0;
    // <--- configuration file analize

    // ---> keiro.dat の読み込み
    int num_route = 1;

    //	vector< vector<waypoint> > waypoint_mat;	//２次元配列のwaypoint
    waypoint_mat.resize(num_route);
    waypoint buff;	//vectorのpush_backのためのバッファ

    const char *keiro_path  = f_path[0];

    for (int ir=0; ir<(int)waypoint_mat.size(); ir++){
	if((fp = fopen(keiro_path, "r")) == NULL){
	    fprintf(stderr, "ERROR : cannot open %s\n", keiro_path);	//経路ファイルがオープンできないときは異常終了
	    return -1;
	}else{
	    printf("route file opened : %s\n", f_path[ir]);

	    int i=0;
	    int ret;
	    while(1){
		//			waypoint1.resize(i+1);
		//			ret = fscanf(fp, "%c %lf %lf\n", &waypoint1[i].bitflag, &waypoint1[i].x, &waypoint1[i].y);

		//fscanfで失敗した場合の処理がいるううよ←エラーのときもEOFが出るよ?
		ret = fscanf(fp, "%c %lf %lf\n", &buff.bitflag, &buff.x, &buff.y);	//本番仕様
		//			ret = fscanf(fp, "%lf %lf\n", &buff.x, &buff.y);	//ルートエディターが不完全な場合の暫定版
		//			buff.bitflag = 'A';																//ルートエディターが不完全な場合の暫定版

		if(ret == EOF){		//次の通過点がない(ゴール)であるかの判定
		    break;
		}
		waypoint_mat[ir].push_back(buff);
		i++;
	    }
	    fclose(fp);
	}
    }
    // <--- keiro.dat の読み込み

    // ---> 読み込んだkeiro.datを表示。確認用だからコメントアウトしていいよ
    printf("%d route found\n", (int)waypoint_mat.size());
    for(int ir=0; ir<(int)waypoint_mat.size(); ir++){
	printf("route[%d]: %d waypoints\n", ir, (int)waypoint_mat[ir].size());
			for(int wp=0; wp<(int)waypoint_mat[ir].size(); wp++){
				printf("waypoint[%d][%3d] ", ir, wp);
				waypoint_mat[ir][wp].show();
			}
    }
    // <--- 読み込んだkeiro.datを表示

    //指定したスタート地点が通過地点数より多い場合、警告して終了
    if(dest >= (int)waypoint_mat[0].size() || dest < 0 ){
	printf("スタート地点 %d dest %d\n", dest-1, dest);
	printf("スタート地点の指定が間違っています\n");
	return -1;
    }

    // ---> SSM data initialize
    int ret;

    ret = initSSM();
    if( ret != 1){
	fprintf(stderr, "initSSM failure\n");
	return -1;
    }

    ret = ssm_odom.openWait(SNAME_ADJUST, 0, 0.0, SSM_READ);
    if( ret != 1 ){
	fprintf(stderr, "open ssm_odom failure\n");
	return -1;
    }

    ret = fs.openWait(SSM_NAME_SOKUIKI_3D_FS, sokuiki_fs_id, 0.0, SSM_READ) && fs.getProperty();	//fsのssmdデータとプロパティ情報の取得
    if( ret != 1 ){
	fprintf(stderr, "open sokuiki_fs failure %s\n", SSM_NAME_SOKUIKI_3D_FS);
	return -1;
    }

    fs.readLast();
    cerr << fs.property << endl;	//プロパティ情報を一覧表示

    if( !Pinfo.create(5.0, 0.1) ){return 1;}	//waipointの情報

//    if( !sound0.create(5.0, 0.1) ){return 1;}	//サウンド発音命令

    // <--- SSM data initialize

    // ---> spur init
    if(!Spur_init()){
	fprintf(stderr, "ERROR : cannot open spur\n");
	return -1;
    }
    // <--- spur init

    // ---> spurのパラメータ設定
    Spur_set_vel( vel );
    Spur_set_accel( accel );
    Spur_set_angvel( angvel );
    Spur_set_angaccel( angaccel );
    // <--- spurのパラメータ設定

    // ---> スタート位置の設定
    //	double th;	//ロボットの姿勢
    th = atan2(waypoint_mat[route][dest].y - waypoint_mat[route][dest-1].y, waypoint_mat[route][dest].x - waypoint_mat[route][dest-1].x);

    //	if(dest > 1){	//途中からスタートするときはときは、必ず次の通過地点の方向を向いている
    ////		Spur_set_pos_GL(waypoint_mat[route][dest-1].x, waypoint_mat[route][dest-1].y, th);
    //		wait_restart_key();
    //		printf("スタートの位置、姿勢\nwaypoint[%d][%3d] : %lf %lf %lf pi\n"
    //				, route, dest-1, waypoint_mat[route][dest-1].x, waypoint_mat[route][dest-1].y, th/M_PI);
    //	}else{	//原点からスタートするときは(0,0,0)に設定
    //		Spur_set_pos_GL( waypoint_mat[route][0].x, waypoint_mat[route][0].y, th );
    //		printf("スタートの位置、姿勢\n原点からスタート : waypoint[%d][%3d] : %lf %lf %lf pi\n"
    //				, route, dest-1, waypoint_mat[route][0].x, waypoint_mat[route][0].y, th/M_PI);
    //	}
//    Spur_set_pos_GL( waypoint_mat[route][0].x, waypoint_mat[route][0].y, th );
    printf("スタートの位置、姿勢\n原点からスタート : waypoint[%d][%3d] : %lf %lf %lf pi\n"
    , route, dest-1, waypoint_mat[route][0].x, waypoint_mat[route][0].y, th/M_PI);
    // <--- スタート位置の設定

    // 安全に終了できるように設定
    setSigInt();

    printf("Initialize FINISHED.\n");
    // <--- initialize

    // --- > operation
    printf("Operation START...\n");

    // ---> 最初の動作
    printf("waypoint[%d][%3d] : %lf, %lf, %lf pi に向かいます\n",route , dest, waypoint_mat[route][dest].x, waypoint_mat[route][dest].y, th/M_PI);	//目標通過地点を表示
    //	Spur_line_GL(waypoint_mat[route][dest].x, waypoint_mat[route][dest].y, th);
    Pinfo.data.route = route;
    Pinfo.data.waypoint = dest;
    Pinfo.write();

//    sound_play(13);
    Spur_free();

//    // ---> qstreamを使う場合
//    printf("open que special file... : ");
//    cout << pipe_path << endl;
//    qin.open(pipe_path.c_str());
//    printf("pipe prepared!!!!\n");
//    // <--- qstreamを使う場合

    printf("Spur-free mode\n");
    wait_restart_key();
    //	Spur_set_pos_GL( waypoint_mat[route][0].x, waypoint_mat[route][0].y, th );
    // <--- 最初の動作

    while(1){	// ---> Operation Loop

	//障害物をURGのデータから検出して”障害物の情報を渡す”プロセスを別につくるようにすると、
	//センサを変更してもパスプランナのプログラムの変更の必要がなくなる。

	// ---> 衝突回避
	if(is_safety() == 1){	//正面に障害物がある場合、障害物がなくなるまで停止
	    my_stop();
	    printf("pre-crash safety!!!\n");
	    int time = 0;
	    while(is_safety() == 1){
		//				sound_play(1);
		if(0)	//ここには絶対にいかない
		{	//WatchDogをつけてポテンシャル法で回避するようにしたい
		    sleepSSM(1.0);	//1sec のサイクル
		    time++;
		    if(time > timeover)
		    {
			trans();    //事実上の暴走モード。危険なため使わない
			break;
		    }
		}
		else
		{
//		    sound_play(9);
		    usleepSSM(20000);	//20m sec のサイクル 50サイクルで1sec
		}
	    }
	    printf("ikuzee!!!!\n");
	    Spur_line_GL(waypoint_mat[route][dest].x, waypoint_mat[route][dest].y, th);	//正面の障害物がなくなったら、再び目標通過地点に向かうSpurコマンドを発行
	}
	// <--- 衝突回避

	Spur_line_GL(waypoint_mat[route][dest].x, waypoint_mat[route][dest].y, th);

	if(Spur_over_line_GL(waypoint_mat[route][dest].x, waypoint_mat[route][dest].y, th) == 1)	//通過点に到達したかの判定
	{
	    printf("waypoint[%d][%3d] に到達しました\n", route, dest);

	    if(speedy == false)	my_stop();	//一時停止するならする

	    if(waypoint_mat[route][dest].isgoalpint() == true || dest >= (int)waypoint_mat[route].size() - 1 )	//次の地点がゴール（もしくは次のwaypointがない）の場合、breakして終了
	    {
		break;
	    }
	    if(waypoint_mat[route][dest].isstoppoint() == true /*&& flg == false*/)	//到達した地点が一時停止地点の場合
	    {
//		sound_play(4);
		wait_restart_key();
//		sound_play(1);
	    }
	    if(waypoint_mat[route][dest].iswaypoint() == true)	//到達した地点が通過地点の場合
	    {
		dest++;
		double x_r, y_r, th_r;
		Spur_get_pos_GL(&x_r, &y_r, &th_r);
		th = atan2(waypoint_mat[route][dest].y - y_r, waypoint_mat[route][dest].x - x_r);	//次の通過地点に向かう角度を計算

int _ret;
		Spur_spin_GL(th);	//その場で次の通過地点に向かって回転
		while( (_ret = Spur_near_ang_GL(th, RAD(detection_angle))) != 1){

		    usleepSSM(5000);
		}
		Spur_line_GL(waypoint_mat[route][dest].x, waypoint_mat[route][dest].y, th);	//次の通過地点に向かうSpurコマンド発行
		Pinfo.data.route = 0;
		Pinfo.data.waypoint = dest;
		Pinfo.write();
//		sound_play(13);

		printf("waypoint[%d][%3d] : %lf, %lf, %lf pi に向かいます\n",route, dest, waypoint_mat[route][dest].x, waypoint_mat[route][dest].y, th/M_PI);	//目標通過地点を表示
	    }
	}
	usleepSSM(5000);

    }// <--- Operation Loop
    // <---  operation

    { // ---> finalize
	my_stop();
	Spur_free();
	printf("Here is the Goal!!!\n");
//	sound_play(3);
	Pinfo.release();
//	sound0.release();
	fs.close();
	endSSM();
	fprintf(stderr, "end SSM.\n");
    } // <--- finalize

    return 0;
}

void trans()
{
    printf(" 最　後　の　悪　あ　が　き　\ntrans!!!!!!!!!!!!!!!!!\n");

    double gravity_x;
    double gravity_y;
    double gravity_th;
    double repulsion_x=0.0;
    double repulsion_y=0.0;
    double subgoal_x, subgoal_y, subgoal_th;
    double r_xy;
    double d;

    while(1)
    {		// ---> ポテンシャル法で抜け出す
//	sound_play(5);

	//終了判定
	if( Spur_over_line_GL(waypoint_mat[route][dest].x, waypoint_mat[route][dest].x, th) == 1 ){
	    printf("trans finish!!!!!!!!!!!!!!!!!!!!!!!\n");
//	    sound_play(0);
	    break;
	}

	if(fs.readNew()){
	    ssm_odom.readTime(fs.time);
	    // ---> 引力の計算
	    d = sqrt( (waypoint_mat[route][dest].x-ssm_odom.data.x)*(waypoint_mat[route][dest].x-ssm_odom.data.x)
	    + (waypoint_mat[route][dest].y-ssm_odom.data.y)*(waypoint_mat[route][dest].y-ssm_odom.data.y) );
	    gravity_th = atan2( waypoint_mat[route][dest].y-ssm_odom.data.y, waypoint_mat[route][dest].x-ssm_odom.data.x);
	    gravity_x = d * cos(gravity_th);
	    gravity_y = d * sin(gravity_th);
	    // <--- 引力の計算

	    // ---> 斥力の計算
	    for(int i = 0; i<(int)fs.property.numPoints; i++){
		if(fs.data[i].isError() == false){
		    r_xy = fs.data[i].reflect.x*fs.data[i].reflect.x + fs.data[i].reflect.y*fs.data[i].reflect.y ;
		    if(r_xy < Pot_dist*Pot_dist){
			repulsion_x = repulsion_x - fs.data[i].reflect.x;
			repulsion_y = repulsion_y - fs.data[i].reflect.y;
		    }
		}
	    }
	    repulsion_x = k_repulsion * repulsion_x;
	    repulsion_y = k_repulsion * repulsion_y;
	    // <--- 斥力の計算

	    subgoal_x = k_gravity*gravity_x + k_repulsion*repulsion_x ;
	    subgoal_y = k_gravity*gravity_y + k_repulsion*repulsion_y ;
	    subgoal_th = atan2( subgoal_y , subgoal_x );

	    Spur_orient_FS( subgoal_th );

	    gravity_th=0.0;
	    gravity_x=0.0;
	    gravity_y=0.0;
	    repulsion_x = 0.0;
	    repulsion_y = 0.0;
	    r_xy=0.0;
	    usleepSSM(20000);
	}else{
	    usleepSSM(20000);
	}

    }		// <--- ポテンシャル法で抜け出す

}

// ---> wait for restart que
void wait_restart_key(void){
    printf("1.安全が確認された場合、Gキーを押してください\n");
    while(1){
	string key;
//	qin>>key;
	cin>>key;
	cout << key << endl;
	if( key == "g" || key == "G" ){	//goサインが出た場合
	    printf("安全が確認されたので動作を再開します\n");
	    break;
	}else if( key == "r" || key == "R" ){	//途中のwaypointから起動する場合
	    Spur_set_pos_GL(waypoint_mat[route][dest-1].x, waypoint_mat[route][dest-1].y, th);
	    printf("Spur_set_pos_GL(%lf, %lf, %lf)\n", waypoint_mat[route][dest-1].x, waypoint_mat[route][dest-1].y, th );
	}else{
	    printf("2.安全が確認された場合、Gキーを押してください\n");
	}
	usleepSSM(5000);
    }
}
// <--- wait for restart que

// ここはプログラムのおわりですううううううaaa
