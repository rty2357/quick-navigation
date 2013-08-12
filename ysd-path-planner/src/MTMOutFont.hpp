/*=================================================

	MTM Output Font
	2012/10/1
	コンソールの色エスケープシーケンス
	いちいち書くのが面倒なのでまとめただけ
	・・・ほかの名前とかぶる？？

==================================================*/

#ifndef MTM_OUTPUT_FONT
#define MTM_OUTPUT_FONT

#define MTM_RESET		"\x1b[39m\x1b[49m\x1b[0m"							//これ忘れるとやばいらしい
#define MTM_FINISH	"\x1b[39m\x1b[49m\x1b[0m　\n"						//改行付き

#define MTM_ERROR		"\x1b[41m\x1b[1m\x1b[33m"	//赤色背景太文字黄色　ド派手
#define MTM_INFO		"\x1b[47m\x1b[1m\x1b[34m"	//灰色背景太文字青色
#define MTM_DEBUG		"\x1b[44m\x1b[1m\x1b[33m"	//青色背景太文字青色

#endif
