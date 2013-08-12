/*
テキストファイルから設定された値?を読んでくるクラス
2012/9/18　MTM

問題点：
「MTM_VALUE」と、「MTM_VALUE_DOUBLE」のように、片方が片方を内包するような変数名を設定すると、おかしな値を読み取ってくる
あとコンフィグファイルでスペースがあること前提に書いたので
MTM_VALUE string
を
MTM_VALUEstring
にすると、「string」ではなく「tring」になってしまう・・・(IntとDouble型はたぶん大丈夫)
あとSetとGet２回呼び出すのめんどくさいのでなんとかしたい

いまのところint,double,std::stringに対応

つかいかた?===================
(1)
MyConfigure.confに、
---
EXAMPLE_1 100
EXAMPLE_2 10.0
EXAMPLE_3 string_data
---
って書いておく

(2)使うcppファイルで、
#include"ConfigManager.hpp"　														このファイルをインクルード

(3)
ConfigureManager.config;																インスタンス生成

(4)
config.SetIntArgument("EXAMPLE_1");													インスタンスに変数名を登録　これがtxtファイルに書いてある名前と完全に同じなら取得可能
config.SetDoubleArgument("EXAMPLE_2");
config.SetStringArgument("EXAMPLE_3");

(5)
config.Read("MyConfigure.conf");													ファイルを読んでくる

(6)
int ex1;			config.GetIntArgument("EXAMPLE_1",&ex1);						読んできたファイルから変数の値を取得する
double ex2;		config.GetDoubleArgument("EXAMPLE_2",&ex2);
int nai;			if(!config.GetIntArgument("SONNA_ARGUMENT_HA_NAI",&nai)){・・・	←存在しない引数を読み込もうとしたり、引数がファイル内に書かれなかったりして、引数が読み込めなかった場合には、falseが帰ってくる
std::string ex3;	config.GetStringArgument("EXAMPLE_3",&ex3,"default");		←その引数が見つからなかった場合のデフォルト値を指定できる

これで ex1 = 100　ex2 = 10.0 ex3 = "string_data"と代入されてるはず。

※Argumentをセットした状態でconfig.OutputSampleFiles("Sample.txt");とすれば、コンフィグファイルのサンプルが出力される。
※Configファイルでは「#」でコメントアウトできる・・・はず
===========================

更新：2012/10/12	１つの引数に対する複数の入力に対応

*/

#ifndef MTM_CONFIG_MANAGER
#define MTM_CONFIG_MANAGER

#include <iostream>
#include <string.h>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include "MTMOutFont.hpp"

namespace mtm{

//読み込みのための構造体定義===========================================================================
//double型
typedef struct{
	bool read;								//読み込まれたかどうか
	std::vector<double> value;		//読み込んだ値
	std::string arg;					//読み込む変数名
}DoubleArgument;
//int型
typedef struct{
	bool read;
	std::vector<int> value;
	std::string arg;
}IntArgument;
//std::string型
typedef struct{
	bool read;
	std::vector<std::string> value;
	std::string arg;
}StringArgument;

//コンフィグファイル読み込みクラス=========================================================================
class ConfigManager{
	public:
		//コンストラクタ、デストラクタ=======================================================
		ConfigManager(){
			ClearAllArguments();
		};
		virtual ~ConfigManager(){};
		//変数のセット=================================================================
		//ダブル型の読み込み変数をセット
		void SetDoubleArgument(std::string argName){
			DoubleArgument buffer;		//構造体の変数を定義
			buffer.arg.assign(argName);	//argにargNameを代入
			buffer.read = false;			//読み込み完了フラグを倒しておく
			buffer.value.resize(0);			//とりあえず0を代入
			d_args.push_back(buffer);	//この変数を配列の最後の要素として追加
		}
		//イント型の読み込み変数をセット
		void SetIntArgument(std::string argName){
			IntArgument buffer;
			buffer.arg.assign(argName);
			buffer.read = false;
			buffer.value.resize(0);
			i_args.push_back(buffer);
		}
		//ストリング型の読み込み変数をセット
		void SetStringArgument(std::string argName){
			StringArgument buffer;
			buffer.arg.assign(argName);
			buffer.read = false;
			buffer.value.resize(0);
			s_args.push_back(buffer);
		}
		//変数のクリア
		void ClearAllArguments(void){
			i_args.resize(0);
			d_args.resize(0);
			s_args.resize(0);
		}
		//変数の取得(1個だけ)===================================================================
		bool GetDoubleArgument(std::string argName, double *result){		//変数を取得したときはtrue、そうでないときはfalse
			if((int)d_args.size() >= 1){
				for(int i=0;i<(int)d_args.size();i++){				//配列の中を探す
					if(argName.compare(d_args[i].arg) == 0){	//d_args[i]のargとargNameが一緒＝入力された引数を見つけた
						if(d_args[i].read){						//この引数がすでにファイルから読み込まれていれば
							*result = d_args[i].value[0];			//resultにこの引数の値を代入して
							return true;							//trueを返して終了
						}	
						else{
							std::cerr << MTM_ERROR << "[ConfigManager]Cannot Find Argument \"" << argName << "\" From Configure File!" << MTM_FINISH;
							*result = 0.0;
							return false;
						}
					}
				}
			}
			std::cerr << MTM_ERROR << "[ConfigManager] Argument \"" << argName << "\" Is Not Input!" << MTM_FINISH;
			*result = 0.0;
			return false;
		}
		bool GetIntArgument(std::string argName, int *result){
			if((int)i_args.size() >= 1){
				for(int i=0;i<(int)i_args.size();i++){
					if(argName.compare(i_args[i].arg) == 0){
						if(i_args[i].read){
							*result = i_args[i].value[0];
							return true;
						}	
						else{
							std::cerr << MTM_ERROR << "[ConfigManager]Cannot Find Argument \"" << argName << "\" From Configure File!" << MTM_FINISH;
							*result = 0;
							return false;
						}
					}
				}
			}
		std::cerr << MTM_ERROR << "[ConfigManager] Argument \"" << argName << "\" Is Not Input!" << MTM_FINISH;
			*result = 0;
			return false;
		}
		bool GetStringArgument(std::string argName, std::string *result){
			if((int)s_args.size() >= 1){
				for(int i=0;i<(int)s_args.size();i++){
					if(argName.compare(s_args[i].arg) == 0){
						if(s_args[i].read){
							*result = s_args[i].value[0].c_str();
							return true;
						}	
						else{
							std::cerr << MTM_ERROR << "[ConfigManager]Cannot Find Argument \"" << argName << "\" From Configure File!" << MTM_FINISH;
							*result = "";
							return false;
						}
					}
				}
			}
			std::cerr << MTM_ERROR << "[ConfigManager] Argument \"" << argName << "\" Is Not Input!" << MTM_FINISH;
			*result = "";
			return false;
		}
		//デフォルト値を指定した取得===================================================================
		bool GetDoubleArgument(std::string argName, double *result, double def){		//変数を取得したときはtrue、そうでないときはfalse defには変数を取得できなかった時のデフォルト値を入れておく
			if(!GetDoubleArgument(argName,result)){
				*result = def;
				return false;
			}
			return true;
		}
		bool GetIntArgument(std::string argName, int *result, int def){
			if(!GetIntArgument(argName,result)){
				*result = def;
				return false;
			}
			return true;
		}
		bool GetStringArgument(std::string argName, std::string *result, std::string def){
			if(!GetStringArgument(argName,result)){
				*result = def.c_str();
				return false;
			}
			return true;
		}
		//変数の取得(複数)===================================================================
		bool GetDoubleArgumentArray(std::string argName, std::vector<double>& result){		//変数を取得したときはtrue、そうでないときはfalse
			if(d_args.size() >= 1){
				for(int i=0;i<(int)d_args.size();i++){				//配列の中を探す
					if(argName.compare(d_args[i].arg) == 0){	//d_args[i]のargとargNameが一緒＝入力された引数を見つけた
						if(d_args[i].read){						//この引数がすでにファイルから読み込まれていれば
							result.resize(d_args[i].value.size());
							for(int j=0;j<(int)result.size();j++)result[j] = d_args[i].value[j];
							return true;							//trueを返して終了
						}	
						else{
							std::cerr << MTM_ERROR << "[ConfigManager]Cannot Find Argument \"" << argName << "\" From Configure File!" << MTM_FINISH;
							result.resize(0);
							return false;
						}
					}
				}
			}
			std::cerr << MTM_ERROR << "[ConfigManager] Argument \"" << argName << "\" Is Not Input!" << MTM_FINISH;
			result.resize(0);
			return false;
		}
		bool GetIntArgumentArray(std::string argName, std::vector<int>& result){
			if(i_args.size() >= 1){
				for(int i=0;i<(int)i_args.size();i++){
					if(argName.compare(i_args[i].arg) == 0){
						if(i_args[i].read){
							result.resize(i_args[i].value.size());
							for(int j=0;j<(int)result.size();j++)result[j] = i_args[i].value[j];
							return true;
						}	
						else{
							std::cerr << MTM_ERROR << "[ConfigManager]Cannot Find Argument \"" << argName << "\" From Configure File!" << MTM_FINISH;
							result.resize(0);
							return false;
						}
					}
				}
			}
		std::cerr << MTM_ERROR << "[ConfigManager] Argument \"" << argName << "\" Is Not Input!" << MTM_FINISH;
			result.resize(0);
			return false;
		}
		bool GetStringArgumentArray(std::string argName, std::vector<std::string>& result){
			if(s_args.size() >= 1){
				for(int i=0;i<(int)s_args.size();i++){
					if(argName.compare(s_args[i].arg) == 0){
						if(s_args[i].read){
							result.resize(s_args[i].value.size());
							for(int j=0;j<(int)result.size();j++)result[j] = s_args[i].value[j];
							return true;
						}	
						else{
							std::cerr << MTM_ERROR << "[ConfigManager]Cannot Find Argument \"" << argName << "\" From Configure File!" << MTM_FINISH;
							result.resize(0);
							return false;
						}
					}
				}
			}
			std::cerr << MTM_ERROR << "[ConfigManager] Argument \"" << argName << "\" Is Not Input!" << MTM_FINISH;
			result.resize(0);
			return false;
		}
		//コンフィグファイルの読み込み=============================================================
		bool Read(const char *filename){
			std::ifstream ifs;
			std::string  line;

			ifs.open(filename);			//オープン
			if(!ifs){						//オープン失敗時の処理
				std::cerr << MTM_ERROR << "[ConfigManager]Cannot Open File \"" << filename << "\"!" << MTM_FINISH;
				return false;
			}
			int ret;
			while(getline(ifs,line)){											//１列そのまま読み込む(読み込めなくなるまで続ける)
				if(line.find("#",0) != 0){										//行の頭に # があったらコメントアウト（無視）
					if(d_args.size() >= 1){
						for(int i=0;i<(int)d_args.size();i++){
							ret = line.find(d_args[i].arg,0);
							if(ret != (int)std::string::npos){						//読み取った文字列からargを発見した場合
								line.erase(0,ret + d_args[i].arg.size());		//0からlineの中のargの最後の位置までをlineから削除
								d_args[i].value.push_back(atof(line.c_str()));			//スペースが入っていてもatofとatoiは大丈夫っぽい
								d_args[i].read = true;
							}
						}
					}
					if(i_args.size() >= 1){
						for(int i=0;i<(int)i_args.size();i++){
						ret = line.find(i_args[i].arg,0);
							if(ret != (int)std::string::npos){						//読み取った文字列からargを発見した場合
								line.erase(0,ret + i_args[i].arg.size());		//0からlineの中のargの最後の位置までをlineから削除
								i_args[i].value.push_back(atoi(line.c_str()));
								i_args[i].read = true;
							}
						}
					}
					if(s_args.size() >= 1){
						for(int i=0;i<(int)s_args.size();i++){
						ret = line.find(s_args[i].arg,0);
							if(ret != (int)std::string::npos){						//読み取った文字列からargを発見した場合
								line.erase(0,ret + s_args[i].arg.size() + 1);	//0からlineの中のargの最後の位置+1(スペース分)までをlineから削除
								s_args[i].value.push_back(line);
								s_args[i].read = true;
							}
						}
					}
				}
			}
			return true;
		}
		//サンプルのコンフィグファイルの出力========================================================================-
		bool OutputSampleFile(const char *filename){
			std::ofstream ofs;
			if(d_args.size() >= 1 || i_args.size() >= 1 || s_args.size() >= 1){
				ofs.open(filename);
				if(!ofs){						//オープン失敗時の処理
					std::cerr << MTM_ERROR << "[ConfigManager]Cannot Open File \"" << filename << "\"!" << MTM_FINISH;
					return false;
				}
				if(d_args.size() >= 1){		//全部0にしたサンプルを書き込む
					for(int i=0;i<(int)d_args.size();i++)ofs << d_args[i].arg << " 0.0000" << std::endl;
				}
				if(i_args.size() >= 1){
					for(int i=0;i<(int)i_args.size();i++)ofs << i_args[i].arg << " 0" << std::endl;
				}
				if(s_args.size() >= 1){
					for(int i=0;i<(int)s_args.size();i++)ofs << s_args[i].arg << " [string data]" << std::endl;
				}
				std::cout << MTM_INFO << "[ConfigManager] Sample Config File \"" << filename << "\" Generated" << MTM_FINISH;
				return true;
			}
			std::cerr << MTM_ERROR << "[ConfigManager]No Input Arguments !" << MTM_FINISH;
			return true;
		}
	private:		
		std::vector<DoubleArgument> d_args;	
		std::vector<IntArgument>    i_args;
		std::vector<StringArgument> s_args;
};

}
#endif
