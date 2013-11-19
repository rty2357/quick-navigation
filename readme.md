quick-navigation
==========

移動ロボットの自律走行ソフトウェアパッケージ

事前に走行した環境中を指示された経路に従って自律走行する

以下の機能を提供する

* 環境地図の作成
* 自律走行経路の編集
* 指定経路の自律走行


# 1 動作環境

* OS : Linux
* コンパイラ : g++
* 移動ロボット : レーザスキャナ(北陽電機社製 URGシリーズ)を水平向きに搭載した独立2輪駆動型移動ロボット
* 依存ライブラリ : 以下の筑波大学知能ロボット研究室の移動ロボット用ソフトウェアプラットフォームに依存
    * ssm
    * ypspur
    * scip2awd
    * Qt

# 2 構成

* **urg-proxy** 北陽電機社製 URGシリーズからスキャンデータを取得

* **ls-coordinate-converter** レーザスキャナのスキャンデータの座標変換

* **opsm-position-tracker** オドメトリとスキャンマッチングにより自己位置を推定し，同時に地図を作成

    スキャンマッチングの参照として，少し前の時点のスキャンデータを用いる
    
    作成される地図は複数スキャンにおける点群の密度による尤度場

* **particle-localizer** 自己位置の推定の推定および管理

    パーティクルフィルタによる確率的自己位置推定を行うプログラム
    
    オドメトリの動作モデルに従ってパーティクルを動作させる
    
    他のプロセスからパーティクルの評価を得て，リサンプリングを行う
  
* **opsm-particle-evaluator** レーザスキャナによる自己位置（パーティクル）の評価

    複数スキャンにおける点群の密度をもとに自己位置推定に有意な特徴に重み付けをして評価する

    参照スキャンとして **opsm-position-tracker** により作成した地図を必要とする

* **ysd-path-planner** 指示された経路上を走行する

    経路は **tkg-route-editor** により作成，編集する

* **tkg-route-editor** 経路編集GUI

    地図作成のために事前走行した際の軌跡を描画
    
    周囲の環境を示すために **opsm-position-tracker** により作成した地図を描画

* **visualizer** センサデータおよびロボットの状態の可視化

* **gndlib** 共通ライブラリ

* **ssmtype** ssmを通して共有するデータを定義したヘッダファイル群


# 3 コンパイル
makeで各プロセスを一括コンパイル

    $ make

# 4 使用方法
※ 本節で表記するコマンドはディレクトリ"quick-navigation"内での操作を示す

## 4.1 環境地図と経路の作成
以下のコマンドで新規にデータを作成する。

    $ ./qn-new <data-name>

作成されたデータはディレクトリ"data"以下に保存される

## 4.1.1 環境地図の作成

- 初期位置を決めてロボットを設置
- ロボットの制御ボードおよびレーザスキャナに電源を投入
- 以下のプログラムをそれぞれターミナルで起動
    - ssm-coordinator

            $ ssm-coordinator

    - ypspur-coordinator

            $ ypspur-coordinator -p <robot-param> -d <device-path,default:/dev/ttyUSB0>

    - urg-proxy

            $ cd urg-proxy
            $ ./launcher -p <urg-device-path,default:/dev/ttyACM0>

    - opsm-position-tracker

            $ cd opsm-position-tracker
            $ ./launcher

    - ls-coordinate-converter

            $ cd ls-coordinator-converter
            $ ./launcher

    - visualizer

            $ cd visualizer
            $ ./launcher

- リモコンで操作するなどして経路を走行
- killssmコマンドにより地図作成終了

         $ killssm

- 作成した地図を保存

         $ ./qn-save


## 4.1.2 経路の編集

- 以下のプログラムを起動
    - tkg-route-editor

            $ cd tkg-route-editor
            $ ./launcher

※ ウェイポイントの追加: 右ダブルクリック  
※ ウェイポイントの位置の変更: 右ドラッグ  
※  "save"をキー入力して保存  

- 作成した経路を保存

         $ ./qn-save


## 4.2 自律走行
シェルスクリプト **qn-open** により走行する環境のデータをロードする

    ./qn-open <データディレクトリ>


以下の手順により自律走行プログラムを起動
- 初期位置にロボットを設置
- ロボットの制御ボードおよびレーザスキャナに電源を投入
- 以下のプログラムを起動
    - ssm-coordinator

            $ ssm-coordinator

    - ypspur-coordinator

            $ ypspur-coordinator -p <robot-param> -d <device-path,default:/dev/ttyUSB0>

    - urg-proxy

            $ cd urg-proxy
            $ ./launcher -p <urg-device-path,default:/dev/ttyACM0>

    - particle-localizer

            $ cd particle-localizer
            $ ./launcher

    - opsm-particle-evaluator

            $ cd opsm-particle-evaluator
            $ ./launcher

    - ls-coordinate-converter

            $ cd ls-coordinate-converter
            $ ./launcher

    - visualizer

            $ cd visualizer
            $ ./launcher

    - ysd-path-planner

            $ cd ysd-path-planner
            $ ./launcher

