multilogger
===========
ssmのログ取得用シェルスクリプト
ファイルより，取得するログのリストを入力

    ./multilogger <list-file>


リストは以下の用にssm名とIDを':'で1行づつ指定する

        spur_odometry:0
        scan_data2d:2        


終了方法

    killssm
