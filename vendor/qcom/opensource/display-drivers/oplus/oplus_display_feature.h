/***************************************************************
  ** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
  **
  ** File : oplus_display_feature.h
  ** Description : oplus display panel char dev  /dev/oplus_panel
  ** Version : 1.0
  ** Date : 2023/08/23
  ** Author : Display
******************************************************************/

#ifndef _OPLUS_DISPLAY_FEATURE_H_
#define _OPLUS_DISPLAY_FEATURE_H_

#include "dsi_display.h"

#define DIMMING_TABLE_SIZE  64
#define MAPPING_SIZE        4096

struct oplus_display_feature {
    bool oplus_display_pwm_enable;
    struct pwm_device *pwm_dev;
    struct pinctrl *pwm_pin;
    struct pinctrl_state *pwm_pin_state;
    int pwm_period_us;
    int pwm_period_old_us;
    int pwm_duty_us;
    int pwm_test_flag;
    bool oplus_esd_flag;
    int (*oplus_display_pwm_update)(int);
    int (*oplus_bl_maping)(int);
    u32 dimming2_mapping[DIMMING_TABLE_SIZE];
    int dimming2_mapping_size;
};
struct oplus_display_feature *get_oplus_display_feature(void);
int oplus_display_feature_init(struct dsi_display *display);
void  oplus_display_feature_exit(void);

static int bl_mapping_table[MAPPING_SIZE] = {
    0,   11,   11,   11,   11,   11,   11,   11,   11,   11,   11,   11,   11,   12,   12,   12,   12,   12,   12,
   12,   12,   12,   12,   12,   13,   13,   13,   13,   13,   13,   13,   13,   13,   13,   13,   13,   14,   14,
   14,   14,   14,   14,   14,   14,   14,   14,   14,   14,   14,   15,   15,   15,   15,   15,   15,   15,   15,
   15,   15,   15,   15,   15,   15,   16,   16,   16,   16,   16,   16,   16,   16,   16,   16,   16,   16,   16,
   16,   16,   16,   16,   16,   16,   17,   17,   17,   17,   17,   17,   17,   17,   17,   17,   17,   17,   17,
   17,   17,   17,   17,   17,   17,   17,   17,   17,   17,   17,   17,   17,   17,   17,   17,   18,   18,   18,
   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,
   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,
   18,   18,   18,   18,   18,   18,   18,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,
   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,
   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   20,   20,   20,
   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,
   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,
   20,   20,   20,   20,   20,   20,   20,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,
   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,
   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   21,   22,   22,   22,
   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,
   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,   22,
   22,   22,   22,   22,   22,   22,   22,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,
   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,
   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   23,   24,   24,   24,
   24,   24,   24,   24,   25,   25,   25,   25,   25,   25,   26,   26,   26,   26,   26,   26,   26,   27,   27,
   27,   27,   27,   27,   28,   28,   28,   28,   28,   28,   28,   29,   29,   29,   29,   29,   29,   30,   30,
   30,   30,   30,   30,   31,   31,   31,   31,   31,   31,   31,   32,   32,   32,   32,   32,   32,   33,   33,
   33,   33,   33,   33,   33,   34,   34,   34,   34,   34,   34,   35,   35,   35,   35,   35,   35,   35,   36,
   36,   36,   36,   36,   36,   37,   37,   37,   37,   37,   37,   38,   38,   38,   38,   38,   38,   38,   39,
   39,   39,   39,   39,   39,   40,   40,   40,   40,   40,   40,   40,   41,   41,   41,   41,   41,   41,   42,
   42,   42,   42,   42,   42,   42,   43,   43,   43,   43,   43,   43,   44,   44,   44,   44,   44,   44,   45,
   45,   45,   45,   45,   45,   45,   46,   46,   46,   46,   46,   46,   47,   47,   47,   47,   47,   47,   47,
   48,   48,   48,   48,   48,   48,   49,   49,   49,   49,   49,   49,   49,   50,   50,   50,   50,   50,   50,
   51,   51,   51,   51,   51,   51,   52,   52,   52,   52,   52,   53,   53,   53,   53,   53,   54,   54,   54,
   54,   54,   55,   55,   55,   55,   55,   56,   56,   56,   56,   57,   57,   57,   57,   57,   58,   58,   58,
   58,   58,   59,   59,   59,   59,   59,   60,   60,   60,   60,   61,   61,   61,   61,   61,   62,   62,   62,
   62,   62,   63,   63,   63,   63,   63,   64,   64,   64,   64,   64,   65,   65,   65,   65,   66,   66,   66,
   66,   66,   67,   67,   67,   67,   67,   68,   68,   68,   68,   68,   69,   69,   69,   69,   70,   70,   70,
   70,   70,   71,   71,   71,   71,   71,   72,   72,   72,   72,   72,   73,   73,   73,   73,   73,   74,   74,
   74,   74,   75,   75,   75,   75,   75,   76,   76,   76,   76,   76,   77,   77,   77,   77,   77,   78,   78,
   78,   78,   79,   79,   79,   79,   79,   80,   80,   80,   80,   80,   81,   81,   81,   81,   81,   82,   82,
   82,   82,   82,   83,   83,   83,   83,   84,   84,   84,   84,   84,   85,   85,   85,   85,   85,   86,   86,
   86,   86,   86,   87,   87,   87,   87,   88,   88,   88,   88,   89,   89,   89,   90,   90,   90,   91,   91,
   91,   91,   92,   92,   92,   93,   93,   93,   94,   94,   94,   94,   95,   95,   95,   96,   96,   96,   97,
   97,   97,   97,   98,   98,   98,   99,   99,   99,  100,  100,  100,  100,  101,  101,  101,  102,  102,  102,
  103,  103,  103,  103,  104,  104,  104,  105,  105,  105,  106,  106,  106,  106,  107,  107,  107,  108,  108,
  108,  109,  109,  109,  109,  110,  110,  110,  111,  111,  111,  112,  112,  112,  112,  113,  113,  113,  114,
  114,  114,  115,  115,  115,  115,  116,  116,  116,  117,  117,  117,  118,  118,  118,  118,  119,  119,  119,
  120,  120,  120,  121,  121,  121,  121,  122,  122,  122,  123,  123,  123,  124,  124,  124,  124,  125,  125,
  125,  126,  126,  126,  127,  127,  127,  127,  128,  128,  128,  129,  129,  129,  130,  130,  130,  130,  131,
  131,  131,  132,  132,  132,  133,  133,  133,  133,  134,  134,  134,  135,  135,  135,  136,  136,  136,  137,
  137,  137,  138,  138,  138,  139,  139,  139,  140,  140,  140,  141,  141,  141,  142,  142,  143,  143,  143,
  144,  144,  144,  145,  145,  145,  146,  146,  146,  147,  147,  147,  148,  148,  149,  149,  149,  150,  150,
  150,  151,  151,  151,  152,  152,  152,  153,  153,  153,  154,  154,  154,  155,  155,  156,  156,  156,  157,
  157,  157,  158,  158,  158,  159,  159,  159,  160,  160,  160,  161,  161,  162,  162,  162,  163,  163,  163,
  164,  164,  164,  165,  165,  165,  166,  166,  166,  167,  167,  167,  168,  168,  169,  169,  169,  170,  170,
  170,  171,  171,  171,  172,  172,  172,  173,  173,  173,  174,  174,  175,  175,  175,  176,  176,  176,  177,
  177,  177,  178,  178,  178,  179,  179,  179,  180,  180,  180,  181,  181,  182,  182,  182,  183,  183,  183,
  184,  184,  184,  185,  185,  185,  186,  186,  186,  187,  187,  188,  188,  188,  189,  189,  189,  190,  190,
  191,  191,  191,  192,  192,  193,  193,  193,  194,  194,  195,  195,  195,  196,  196,  196,  197,  197,  198,
  198,  198,  199,  199,  200,  200,  200,  201,  201,  202,  202,  202,  203,  203,  203,  204,  204,  205,  205,
  205,  206,  206,  207,  207,  207,  208,  208,  209,  209,  209,  210,  210,  210,  211,  211,  212,  212,  212,
  213,  213,  214,  214,  214,  215,  215,  216,  216,  216,  217,  217,  217,  218,  218,  219,  219,  219,  220,
  220,  221,  221,  221,  222,  222,  223,  223,  223,  224,  224,  224,  225,  225,  226,  226,  226,  227,  227,
  228,  228,  228,  229,  229,  230,  230,  230,  231,  231,  231,  232,  232,  233,  233,  233,  234,  234,  235,
  235,  235,  236,  236,  237,  237,  237,  238,  238,  238,  239,  239,  240,  240,  240,  241,  241,  242,  242,
  242,  243,  243,  244,  244,  245,  245,  246,  246,  247,  247,  248,  248,  249,  249,  250,  250,  251,  251,
  252,  253,  253,  254,  254,  255,  255,  256,  256,  257,  257,  258,  258,  259,  259,  260,  260,  261,  262,
  262,  263,  263,  264,  264,  265,  265,  266,  266,  267,  267,  268,  268,  269,  269,  270,  271,  271,  272,
  272,  273,  273,  274,  274,  275,  275,  276,  276,  277,  277,  278,  278,  279,  280,  280,  281,  281,  282,
  282,  283,  283,  284,  284,  285,  285,  286,  286,  287,  287,  288,  289,  289,  290,  290,  291,  291,  292,
  292,  293,  293,  294,  294,  295,  295,  296,  296,  297,  298,  298,  299,  299,  300,  300,  301,  301,  302,
  302,  303,  303,  304,  304,  305,  305,  306,  307,  307,  308,  308,  309,  309,  310,  310,  311,  311,  312,
  312,  313,  313,  314,  314,  315,  316,  316,  316,  317,  317,  318,  318,  319,  319,  320,  320,  321,  321,
  322,  322,  323,  323,  324,  324,  325,  325,  326,  326,  327,  327,  328,  328,  329,  329,  330,  330,  331,
  331,  332,  332,  333,  333,  334,  334,  335,  335,  336,  336,  337,  337,  338,  338,  339,  339,  340,  340,
  341,  341,  342,  342,  343,  343,  344,  344,  345,  345,  346,  346,  347,  347,  348,  348,  349,  349,  350,
  350,  351,  351,  352,  352,  353,  353,  354,  354,  355,  355,  356,  356,  357,  357,  358,  358,  359,  359,
  360,  360,  361,  361,  362,  362,  363,  363,  364,  364,  365,  365,  366,  366,  367,  367,  368,  368,  369,
  369,  370,  370,  371,  371,  372,  372,  373,  373,  374,  374,  375,  375,  376,  376,  377,  377,  378,  378,
  379,  379,  380,  380,  381,  381,  382,  383,  383,  384,  384,  385,  386,  386,  387,  387,  388,  389,  389,
  390,  391,  391,  392,  392,  393,  394,  394,  395,  395,  396,  397,  397,  398,  399,  399,  400,  400,  401,
  402,  402,  403,  403,  404,  405,  405,  406,  406,  407,  408,  408,  409,  410,  410,  411,  411,  412,  413,
  413,  414,  414,  415,  416,  416,  417,  418,  418,  419,  419,  420,  421,  421,  422,  422,  423,  424,  424,
  425,  425,  426,  427,  427,  428,  429,  429,  430,  430,  431,  432,  432,  433,  433,  434,  435,  435,  436,
  437,  437,  438,  438,  439,  440,  440,  441,  441,  442,  443,  443,  444,  444,  445,  446,  446,  447,  448,
  448,  449,  449,  450,  451,  451,  452,  452,  453,  454,  454,  455,  456,  456,  457,  458,  458,  459,  460,
  460,  461,  462,  462,  463,  464,  464,  465,  466,  466,  467,  468,  468,  469,  470,  470,  471,  472,  472,
  473,  474,  474,  475,  476,  476,  477,  478,  478,  479,  480,  480,  481,  482,  482,  483,  484,  484,  485,
  486,  486,  487,  488,  488,  489,  490,  490,  491,  492,  492,  493,  494,  494,  495,  496,  496,  497,  498,
  498,  499,  500,  500,  501,  502,  502,  503,  504,  504,  505,  506,  506,  507,  508,  508,  509,  510,  510,
  511,  512,  512,  513,  514,  514,  515,  516,  516,  517,  518,  518,  519,  520,  520,  521,  522,  522,  523,
  524,  524,  525,  526,  526,  527,  528,  528,  529,  530,  530,  531,  532,  532,  533,  534,  534,  535,  536,
  536,  537,  538,  538,  539,  540,  541,  541,  542,  543,  543,  544,  545,  546,  546,  547,  548,  549,  549,
  550,  551,  551,  552,  553,  554,  554,  555,  556,  557,  557,  558,  559,  559,  560,  561,  562,  562,  563,
  564,  564,  565,  566,  567,  567,  568,  569,  570,  570,  571,  572,  572,  573,  574,  575,  575,  576,  577,
  578,  578,  579,  580,  580,  581,  582,  583,  583,  584,  585,  585,  586,  587,  588,  588,  589,  590,  591,
  591,  592,  593,  593,  594,  595,  596,  596,  597,  598,  599,  599,  600,  601,  601,  602,  603,  604,  604,
  605,  606,  606,  607,  608,  609,  609,  610,  611,  612,  612,  613,  614,  614,  615,  616,  617,  617,  618,
  619,  620,  620,  621,  622,  623,  624,  624,  625,  626,  627,  628,  629,  629,  630,  631,  632,  633,  633,
  634,  635,  636,  637,  638,  638,  639,  640,  641,  642,  643,  643,  644,  645,  646,  647,  647,  648,  649,
  650,  651,  652,  652,  653,  654,  655,  656,  656,  657,  658,  659,  660,  661,  661,  662,  663,  664,  665,
  666,  666,  667,  668,  669,  670,  670,  671,  672,  673,  674,  675,  675,  676,  677,  678,  679,  679,  680,
  681,  682,  683,  684,  684,  685,  686,  687,  688,  689,  689,  690,  691,  692,  693,  693,  694,  695,  696,
  697,  698,  698,  699,  700,  701,  702,  702,  703,  704,  705,  706,  707,  707,  708,  709,  710,  711,  712,
  712,  713,  714,  715,  716,  716,  717,  718,  719,  720,  720,  721,  722,  723,  724,  725,  725,  726,  727,
  728,  729,  729,  730,  731,  732,  733,  734,  734,  735,  736,  737,  738,  738,  739,  740,  741,  742,  742,
  743,  744,  745,  746,  747,  747,  748,  749,  750,  751,  751,  752,  753,  754,  755,  756,  756,  757,  758,
  759,  760,  760,  761,  762,  763,  764,  764,  765,  766,  767,  768,  769,  769,  770,  771,  772,  773,  773,
  774,  775,  776,  777,  778,  778,  779,  780,  781,  782,  782,  783,  784,  785,  786,  786,  787,  788,  789,
  790,  791,  791,  792,  793,  794,  795,  795,  796,  797,  798,  799,  800,  800,  801,  802,  803,  804,  805,
  806,  807,  808,  809,  810,  811,  812,  812,  813,  814,  815,  816,  817,  818,  819,  820,  821,  822,  823,
  824,  824,  825,  826,  827,  828,  829,  830,  831,  832,  833,  834,  835,  836,  836,  837,  838,  839,  840,
  841,  842,  843,  844,  845,  846,  847,  848,  848,  849,  850,  851,  852,  853,  854,  855,  856,  857,  858,
  859,  860,  860,  861,  862,  863,  864,  865,  866,  867,  868,  869,  870,  871,  872,  872,  873,  874,  875,
  876,  877,  878,  879,  880,  881,  882,  883,  884,  884,  885,  886,  887,  888,  889,  890,  891,  892,  893,
  894,  895,  896,  897,  898,  899,  900,  901,  902,  903,  904,  905,  906,  907,  908,  909,  910,  911,  912,
  913,  914,  915,  916,  917,  918,  919,  920,  921,  922,  923,  924,  925,  926,  927,  928,  929,  930,  931,
  932,  933,  934,  935,  936,  937,  938,  939,  940,  941,  942,  943,  944,  945,  946,  947,  948,  949,  950,
  951,  952,  953,  954,  955,  956,  957,  958,  959,  960,  961,  962,  963,  964,  965,  966,  967,  968,  969,
  970,  971,  972,  973,  974,  975,  976,  977,  978,  979,  980,  981,  982,  983,  984,  985,  986,  987,  988,
  989,  990,  991,  992,  993,  994,  995,  996,  997,  998,  999, 1000, 1001, 1002, 1003, 1004, 1005, 1006, 1007,
 1008, 1009, 1010, 1011, 1012, 1013, 1014, 1015, 1016, 1017, 1018, 1019, 1020, 1021, 1022, 1023, 1025, 1026, 1027,
 1028, 1029, 1030, 1031, 1032, 1033, 1034, 1035, 1036, 1037, 1038, 1039, 1040, 1041, 1042, 1043, 1044, 1045, 1046,
 1047, 1048, 1050, 1051, 1052, 1053, 1054, 1055, 1056, 1057, 1058, 1059, 1060, 1061, 1062, 1063, 1064, 1065, 1066,
 1067, 1068, 1069, 1070, 1071, 1072, 1073, 1075, 1076, 1077, 1078, 1079, 1080, 1081, 1082, 1083, 1084, 1085, 1086,
 1087, 1088, 1089, 1090, 1091, 1092, 1093, 1094, 1095, 1096, 1097, 1098, 1100, 1101, 1102, 1103, 1104, 1105, 1106,
 1107, 1108, 1109, 1110, 1111, 1112, 1113, 1114, 1115, 1116, 1117, 1118, 1119, 1120, 1121, 1122, 1123, 1124, 1125,
 1126, 1127, 1128, 1129, 1130, 1131, 1132, 1133, 1134, 1135, 1136, 1137, 1138, 1139, 1140, 1141, 1142, 1143, 1144,
 1145, 1146, 1147, 1148, 1149, 1150, 1151, 1152, 1153, 1154, 1155, 1156, 1157, 1158, 1159, 1160, 1161, 1162, 1163,
 1164, 1165, 1166, 1167, 1168, 1169, 1170, 1171, 1172, 1173, 1174, 1175, 1176, 1177, 1178, 1179, 1180, 1181, 1182,
 1183, 1184, 1185, 1186, 1187, 1188, 1189, 1190, 1191, 1192, 1193, 1194, 1195, 1196, 1197, 1198, 1199, 1200, 1201,
 1203, 1204, 1205, 1206, 1207, 1208, 1210, 1211, 1212, 1213, 1214, 1215, 1217, 1218, 1219, 1220, 1221, 1222, 1224,
 1225, 1226, 1227, 1228, 1229, 1231, 1232, 1233, 1234, 1235, 1236, 1238, 1239, 1240, 1241, 1242, 1243, 1245, 1246,
 1247, 1248, 1249, 1250, 1252, 1253, 1254, 1255, 1256, 1257, 1259, 1260, 1261, 1262, 1263, 1264, 1266, 1267, 1268,
 1269, 1270, 1271, 1273, 1274, 1275, 1276, 1277, 1278, 1280, 1281, 1282, 1283, 1284, 1285, 1287, 1288, 1289, 1290,
 1291, 1292, 1294, 1295, 1296, 1297, 1298, 1299, 1301, 1302, 1303, 1304, 1305, 1306, 1308, 1309, 1310, 1311, 1312,
 1314, 1315, 1316, 1317, 1319, 1320, 1321, 1322, 1323, 1325, 1326, 1327, 1328, 1330, 1331, 1332, 1333, 1335, 1336,
 1337, 1338, 1339, 1341, 1342, 1343, 1344, 1346, 1347, 1348, 1349, 1350, 1352, 1353, 1354, 1355, 1357, 1358, 1359,
 1360, 1362, 1363, 1364, 1365, 1366, 1368, 1369, 1370, 1371, 1373, 1374, 1375, 1376, 1378, 1379, 1380, 1381, 1382,
 1384, 1385, 1386, 1387, 1389, 1390, 1391, 1392, 1393, 1395, 1396, 1397, 1398, 1400, 1401, 1402, 1403, 1405, 1406,
 1407, 1408, 1409, 1411, 1412, 1413, 1414, 1416, 1417, 1418, 1419, 1421, 1422, 1423, 1424, 1425, 1427, 1428, 1429,
 1430, 1432, 1433, 1434, 1435, 1436, 1438, 1439, 1440, 1441, 1443, 1444, 1445, 1446, 1448, 1449, 1450, 1451, 1452,
 1454, 1455, 1456, 1457, 1459, 1460, 1461, 1462, 1463, 1465, 1466, 1467, 1468, 1470, 1471, 1472, 1473, 1475, 1476,
 1477, 1478, 1479, 1481, 1482, 1483, 1484, 1486, 1487, 1488, 1489, 1490, 1492, 1493, 1494, 1495, 1497, 1498, 1499,
 1500, 1502, 1503, 1504, 1505, 1506, 1508, 1509, 1510, 1511, 1513, 1514, 1515, 1516, 1517, 1519, 1520, 1521, 1522,
 1524, 1525, 1526, 1527, 1529, 1530, 1531, 1532, 1534, 1535, 1536, 1538, 1539, 1540, 1542, 1543, 1544, 1545, 1547,
 1548, 1549, 1551, 1552, 1553, 1555, 1556, 1557, 1558, 1560, 1561, 1562, 1564, 1565, 1566, 1568, 1569, 1570, 1572,
 1573, 1574, 1575, 1577, 1578, 1579, 1581, 1582, 1583, 1585, 1586, 1587, 1588, 1590, 1591, 1592, 1594, 1595, 1596,
 1598, 1599, 1600, 1601, 1603, 1604, 1605, 1607, 1608, 1609, 1611, 1612, 1613, 1615, 1616, 1617, 1618, 1620, 1621,
 1622, 1624, 1625, 1626, 1628, 1629, 1630, 1631, 1633, 1634, 1635, 1637, 1638, 1639, 1641, 1642, 1643, 1645, 1646,
 1647, 1649, 1650, 1652, 1653, 1655, 1656, 1657, 1659, 1660, 1662, 1663, 1665, 1666, 1667, 1669, 1670, 1672, 1673,
 1675, 1676, 1677, 1679, 1680, 1682, 1683, 1685, 1686, 1687, 1689, 1690, 1692, 1693, 1695, 1696, 1697, 1699, 1700,
 1702, 1703, 1705, 1706, 1707, 1709, 1710, 1712, 1713, 1715, 1716, 1717, 1719, 1720, 1722, 1723, 1725, 1726, 1727,
 1729, 1730, 1732, 1733, 1735, 1736, 1737, 1739, 1740, 1742, 1743, 1745, 1746, 1747, 1749, 1750, 1752, 1753, 1755,
 1756, 1757, 1759, 1760, 1762, 1763, 1765, 1766, 1767, 1769, 1770, 1772, 1773, 1775, 1776, 1777, 1779, 1780, 1782,
 1783, 1785, 1786, 1787, 1789, 1790, 1792, 1793, 1795, 1796, 1797, 1799, 1800, 1802, 1803, 1805, 1806, 1807, 1809,
 1810, 1812, 1813, 1815, 1816, 1817, 1819, 1820, 1822, 1823, 1825, 1826, 1827, 1829, 1830, 1832, 1833, 1835, 1836,
 1837, 1839, 1840, 1842, 1843, 1845, 1846, 1847, 1849, 1850, 1852, 1853, 1855, 1856, 1857, 1859, 1860, 1862, 1863,
 1865, 1866, 1867, 1869, 1870, 1872, 1873, 1875, 1876, 1877, 1879, 1880, 1882, 1883, 1885, 1886, 1888, 1889, 1891,
 1893, 1894, 1896, 1897, 1899, 1901, 1902, 1904, 1905, 1907, 1909, 1910, 1912, 1913, 1915, 1917, 1918, 1920, 1921,
 1923, 1925, 1926, 1928, 1929, 1931, 1933, 1934, 1936, 1937, 1939, 1941, 1942, 1944, 1945, 1947, 1949, 1950, 1952,
 1953, 1955, 1957, 1958, 1960, 1961, 1963, 1965, 1966, 1968, 1969, 1971, 1973, 1974, 1976, 1977, 1979, 1981, 1982,
 1984, 1985, 1987, 1989, 1990, 1992, 1993, 1995, 1997, 1998, 2000, 2001, 2003, 2005, 2006, 2008, 2009, 2011, 2013,
 2014, 2016, 2017, 2019, 2021, 2022, 2024, 2025, 2027, 2029, 2030, 2032, 2033, 2035, 2037, 2038, 2040, 2041, 2043,
 2045, 2046, 2048, 2049, 2051, 2053, 2054, 2056, 2057, 2059, 2061, 2062, 2064, 2065, 2067, 2069, 2070, 2072, 2073,
 2075, 2077, 2078, 2080, 2081, 2083, 2085, 2086, 2088, 2089, 2091, 2093, 2094, 2096, 2097, 2099, 2101, 2102, 2104,
 2105, 2107, 2109, 2110, 2112, 2113, 2115, 2117, 2118, 2120, 2121, 2123, 2125, 2126, 2128, 2129, 2131, 2133, 2134,
 2136, 2137, 2139, 2141, 2142, 2144, 2145, 2147, 2149, 2150, 2152, 2154, 2155, 2157, 2159, 2160, 2162, 2164, 2165,
 2167, 2169, 2170, 2172, 2174, 2175, 2177, 2178, 2180, 2182, 2183, 2185, 2187, 2188, 2190, 2192, 2193, 2195, 2197,
 2198, 2200, 2202, 2203, 2205, 2207, 2208, 2210, 2211, 2213, 2215, 2216, 2218, 2220, 2221, 2223, 2225, 2226, 2228,
 2230, 2231, 2233, 2235, 2236, 2238, 2240, 2241, 2243, 2244, 2246, 2248, 2249, 2251, 2253, 2254, 2256, 2258, 2259,
 2261, 2263, 2264, 2266, 2268, 2269, 2271, 2273, 2274, 2276, 2278, 2279, 2281, 2283, 2285, 2286, 2288, 2290, 2292,
 2293, 2295, 2297, 2299, 2300, 2302, 2304, 2306, 2307, 2309, 2311, 2312, 2314, 2316, 2318, 2319, 2321, 2323, 2325,
 2326, 2328, 2330, 2332, 2333, 2335, 2337, 2339, 2340, 2342, 2344, 2345, 2347, 2349, 2351, 2352, 2354, 2356, 2358,
 2359, 2361, 2363, 2365, 2366, 2368, 2370, 2372, 2373, 2375, 2377, 2378, 2380, 2382, 2384, 2385, 2387, 2389, 2391,
 2392, 2394, 2396, 2398, 2399, 2401, 2403, 2405, 2406, 2408, 2410, 2411, 2413, 2415, 2416, 2418, 2420, 2421, 2423,
 2425, 2426, 2428, 2430, 2431, 2433, 2435, 2437, 2438, 2440, 2442, 2443, 2445, 2447, 2448, 2450, 2452, 2453, 2455,
 2457, 2458, 2460, 2462, 2463, 2465, 2467, 2469, 2470, 2472, 2474, 2475, 2477, 2479, 2480, 2482, 2484, 2485, 2487,
 2489, 2490, 2492, 2494, 2495, 2497, 2499, 2501, 2502, 2504, 2506, 2507, 2509, 2511, 2512, 2514, 2516, 2517, 2519,
 2521, 2522, 2524, 2526, 2527, 2529, 2531, 2533, 2534, 2536, 2538, 2540, 2541, 2543, 2545, 2547, 2549, 2550, 2552,
 2554, 2556, 2558, 2559, 2561, 2563, 2565, 2567, 2568, 2570, 2572, 2574, 2575, 2577, 2579, 2581, 2583, 2584, 2586,
 2588, 2590, 2592, 2593, 2595, 2597, 2599, 2601, 2602, 2604, 2606, 2608, 2609, 2611, 2613, 2615, 2617, 2618, 2620,
 2622, 2624, 2626, 2627, 2629, 2631, 2633, 2635, 2636, 2638, 2640, 2642, 2643, 2645, 2647, 2649, 2651, 2652, 2654,
 2656, 2658, 2660, 2661, 2663, 2665, 2667, 2669, 2670, 2672, 2674, 2676, 2678, 2680, 2682, 2684, 2686, 2688, 2689,
 2691, 2693, 2695, 2697, 2699, 2701, 2703, 2705, 2707, 2708, 2710, 2712, 2714, 2716, 2718, 2720, 2722, 2724, 2726,
 2727, 2729, 2731, 2733, 2735, 2737, 2739, 2741, 2743, 2745, 2747, 2748, 2750, 2752, 2754, 2756, 2758, 2760, 2762,
 2764, 2766, 2767, 2769, 2771, 2773, 2775, 2777, 2779, 2781, 2783, 2785, 2786, 2788, 2790, 2792, 2794, 2796, 2798,
 2800, 2802, 2804, 2806, 2808, 2810, 2812, 2814, 2816, 2818, 2820, 2822, 2824, 2826, 2828, 2830, 2832, 2834, 2836,
 2838, 2840, 2843, 2845, 2847, 2849, 2851, 2853, 2855, 2857, 2859, 2861, 2863, 2865, 2867, 2869, 2871, 2873, 2875,
 2877, 2880, 2882, 2884, 2886, 2888, 2890, 2892, 2894, 2896, 2898, 2900, 2902, 2904, 2906, 2908, 2910, 2912, 2914,
 2917, 2919, 2921, 2923, 2925, 2927, 2929, 2931, 2933, 2935, 2937, 2939, 2941, 2943, 2945, 2947, 2949, 2951, 2954,
 2955, 2957, 2959, 2961, 2963, 2965, 2967, 2969, 2971, 2973, 2975, 2977, 2979, 2981, 2983, 2985, 2987, 2989, 2990,
 2992, 2994, 2996, 2998, 3000, 3002, 3004, 3006, 3008, 3010, 3012, 3014, 3016, 3018, 3020, 3022, 3024, 3025, 3027,
 3029, 3031, 3033, 3035, 3037, 3039, 3041, 3043, 3045, 3047, 3049, 3051, 3053, 3055, 3057, 3059, 3060, 3062, 3064,
 3066, 3068, 3070, 3072, 3074, 3076, 3078, 3080, 3082, 3084, 3086, 3088, 3090, 3092, 3094, 3096, 3098, 3100, 3102,
 3104, 3106, 3108, 3110, 3112, 3114, 3116, 3118, 3120, 3122, 3124, 3126, 3128, 3131, 3133, 3135, 3137, 3139, 3141,
 3143, 3145, 3147, 3149, 3151, 3153, 3155, 3157, 3159, 3161, 3163, 3165, 3168, 3170, 3172, 3174, 3176, 3178, 3180,
 3182, 3184, 3186, 3188, 3190, 3192, 3194, 3196, 3198, 3200, 3202, 3205, 3207, 3209, 3211, 3213, 3215, 3217, 3219,
 3221, 3223, 3225, 3227, 3229, 3231, 3233, 3235, 3237, 3239, 3242, 3244, 3246, 3248, 3250, 3252, 3255, 3257, 3259,
 3261, 3263, 3265, 3268, 3270, 3272, 3274, 3276, 3279, 3281, 3283, 3285, 3287, 3289, 3292, 3294, 3296, 3298, 3300,
 3302, 3305, 3307, 3309, 3311, 3313, 3316, 3318, 3320, 3322, 3324, 3326, 3329, 3331, 3333, 3335, 3337, 3339, 3342,
 3344, 3346, 3348, 3350, 3353, 3355, 3357, 3359, 3361, 3363, 3366, 3368, 3370, 3372, 3374, 3376, 3379, 3381, 3383,
 3385, 3387, 3390, 3392, 3394, 3396, 3398, 3400, 3403, 3405, 3407, 3409, 3411, 3413, 3416, 3418, 3420, 3422, 3424,
 3427, 3429, 3431, 3433, 3435, 3437, 3440, 3442, 3444, 3446, 3448, 3450, 3453, 3455, 3457, 3459, 3461, 3464, 3466,
 3468, 3470, 3472, 3474, 3477, 3479, 3481, 3483, 3485, 3487, 3490, 3492, 3494, 3496, 3498, 3501, 3503, 3505, 3507,
 3509, 3511, 3514, 3516, 3518, 3520, 3522, 3524, 3527, 3529, 3531, 3533, 3535, 3538, 3540, 3542, 3545, 3547, 3549,
 3552, 3554, 3556, 3559, 3561, 3563, 3566, 3568, 3570, 3573, 3575, 3578, 3580, 3582, 3585, 3587, 3589, 3592, 3594,
 3596, 3599, 3601, 3603, 3606, 3608, 3610, 3613, 3615, 3618, 3620, 3622, 3625, 3627, 3629, 3632, 3634, 3636, 3639,
 3641, 3643, 3646, 3648, 3650, 3653, 3655, 3658, 3660, 3662, 3665, 3667, 3669, 3672, 3674, 3676, 3679, 3681, 3683,
 3686, 3688, 3690, 3693, 3695, 3698, 3700, 3702, 3705, 3707, 3709, 3712, 3714, 3717, 3719, 3721, 3724, 3726, 3728,
 3731, 3733, 3736, 3738, 3740, 3743, 3745, 3747, 3750, 3752, 3755, 3757, 3759, 3762, 3764, 3766, 3769, 3771, 3774,
 3776, 3778, 3781, 3783, 3785, 3788, 3790, 3793, 3795, 3797, 3800, 3802, 3804, 3807, 3809, 3812, 3814, 3816, 3819,
 3821, 3823, 3826, 3828, 3831, 3833, 3835, 3838, 3840, 3842, 3845, 3847, 3850, 3852, 3854, 3857, 3859, 3862, 3864,
 3867, 3869, 3871, 3874, 3876, 3879, 3881, 3884, 3886, 3889, 3891, 3893, 3896, 3898, 3901, 3903, 3906, 3908, 3910,
 3913, 3915, 3918, 3920, 3923, 3925, 3928, 3930, 3932, 3935, 3937, 3940, 3942, 3945, 3947, 3949, 3952, 3954, 3957,
 3959, 3962, 3964, 3967, 3969, 3971, 3974, 3976, 3979, 3981, 3984, 3986, 3988, 3991, 3993, 3996, 3998, 4001, 4003,
 4006, 4009, 4012, 4015, 4018, 4021, 4024, 4027, 4030, 4033, 4036, 4039, 4042, 4045, 4048, 4052, 4055, 4058, 4061,
 4064, 4067, 4070, 4073, 4076, 4079, 4082, 4085, 4088, 4091, 4095
};


#endif /*_OPLUS_DISPLAY_FEATURE_H_*/
