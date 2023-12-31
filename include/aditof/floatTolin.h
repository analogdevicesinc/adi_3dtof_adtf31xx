// Copyright (C) Microsoft Corporation. All rights reserved.
// Portions Copyright (c) 2020 Analog Devices, Inc.

#ifndef FLOAT_TO_LIN_H
#define FLOAT_TO_LIN_H

#define NUM_FLOAT_LIN_LUT_ELEMENTS 2048

const short kFloaT2LinLut[NUM_FLOAT_LIN_LUT_ELEMENTS] = {
  0,      1,      2,      3,      4,      5,      6,      7,      8,      9,      10,     11,     12,     13,
  14,     15,     16,     17,     18,     19,     20,     21,     22,     23,     24,     25,     26,     27,
  28,     29,     30,     31,     32,     33,     34,     35,     36,     37,     38,     39,     40,     41,
  42,     43,     44,     45,     46,     47,     48,     49,     50,     51,     52,     53,     54,     55,
  56,     57,     58,     59,     60,     61,     62,     63,     64,     65,     66,     67,     68,     69,
  70,     71,     72,     73,     74,     75,     76,     77,     78,     79,     80,     81,     82,     83,
  84,     85,     86,     87,     88,     89,     90,     91,     92,     93,     94,     95,     96,     97,
  98,     99,     100,    101,    102,    103,    104,    105,    106,    107,    108,    109,    110,    111,
  112,    113,    114,    115,    116,    117,    118,    119,    120,    121,    122,    123,    124,    125,
  126,    127,    128,    129,    130,    131,    132,    133,    134,    135,    136,    137,    138,    139,
  140,    141,    142,    143,    144,    145,    146,    147,    148,    149,    150,    151,    152,    153,
  154,    155,    156,    157,    158,    159,    160,    161,    162,    163,    164,    165,    166,    167,
  168,    169,    170,    171,    172,    173,    174,    175,    176,    177,    178,    179,    180,    181,
  182,    183,    184,    185,    186,    187,    188,    189,    190,    191,    192,    193,    194,    195,
  196,    197,    198,    199,    200,    201,    202,    203,    204,    205,    206,    207,    208,    209,
  210,    211,    212,    213,    214,    215,    216,    217,    218,    219,    220,    221,    222,    223,
  224,    225,    226,    227,    228,    229,    230,    231,    232,    233,    234,    235,    236,    237,
  238,    239,    240,    241,    242,    243,    244,    245,    246,    247,    248,    249,    250,    251,
  252,    253,    254,    255,    256,    258,    260,    262,    264,    266,    268,    270,    272,    274,
  276,    278,    280,    282,    284,    286,    288,    290,    292,    294,    296,    298,    300,    302,
  304,    306,    308,    310,    312,    314,    316,    318,    320,    322,    324,    326,    328,    330,
  332,    334,    336,    338,    340,    342,    344,    346,    348,    350,    352,    354,    356,    358,
  360,    362,    364,    366,    368,    370,    372,    374,    376,    378,    380,    382,    384,    386,
  388,    390,    392,    394,    396,    398,    400,    402,    404,    406,    408,    410,    412,    414,
  416,    418,    420,    422,    424,    426,    428,    430,    432,    434,    436,    438,    440,    442,
  444,    446,    448,    450,    452,    454,    456,    458,    460,    462,    464,    466,    468,    470,
  472,    474,    476,    478,    480,    482,    484,    486,    488,    490,    492,    494,    496,    498,
  500,    502,    504,    506,    508,    510,    512,    516,    520,    524,    528,    532,    536,    540,
  544,    548,    552,    556,    560,    564,    568,    572,    576,    580,    584,    588,    592,    596,
  600,    604,    608,    612,    616,    620,    624,    628,    632,    636,    640,    644,    648,    652,
  656,    660,    664,    668,    672,    676,    680,    684,    688,    692,    696,    700,    704,    708,
  712,    716,    720,    724,    728,    732,    736,    740,    744,    748,    752,    756,    760,    764,
  768,    772,    776,    780,    784,    788,    792,    796,    800,    804,    808,    812,    816,    820,
  824,    828,    832,    836,    840,    844,    848,    852,    856,    860,    864,    868,    872,    876,
  880,    884,    888,    892,    896,    900,    904,    908,    912,    916,    920,    924,    928,    932,
  936,    940,    944,    948,    952,    956,    960,    964,    968,    972,    976,    980,    984,    988,
  992,    996,    1000,   1004,   1008,   1012,   1016,   1020,   1024,   1032,   1040,   1048,   1056,   1064,
  1072,   1080,   1088,   1096,   1104,   1112,   1120,   1128,   1136,   1144,   1152,   1160,   1168,   1176,
  1184,   1192,   1200,   1208,   1216,   1224,   1232,   1240,   1248,   1256,   1264,   1272,   1280,   1288,
  1296,   1304,   1312,   1320,   1328,   1336,   1344,   1352,   1360,   1368,   1376,   1384,   1392,   1400,
  1408,   1416,   1424,   1432,   1440,   1448,   1456,   1464,   1472,   1480,   1488,   1496,   1504,   1512,
  1520,   1528,   1536,   1544,   1552,   1560,   1568,   1576,   1584,   1592,   1600,   1608,   1616,   1624,
  1632,   1640,   1648,   1656,   1664,   1672,   1680,   1688,   1696,   1704,   1712,   1720,   1728,   1736,
  1744,   1752,   1760,   1768,   1776,   1784,   1792,   1800,   1808,   1816,   1824,   1832,   1840,   1848,
  1856,   1864,   1872,   1880,   1888,   1896,   1904,   1912,   1920,   1928,   1936,   1944,   1952,   1960,
  1968,   1976,   1984,   1992,   2000,   2008,   2016,   2024,   2032,   2040,   2048,   2064,   2080,   2096,
  2112,   2128,   2144,   2160,   2176,   2192,   2208,   2224,   2240,   2256,   2272,   2288,   2304,   2320,
  2336,   2352,   2368,   2384,   2400,   2416,   2432,   2448,   2464,   2480,   2496,   2512,   2528,   2544,
  2560,   2576,   2592,   2608,   2624,   2640,   2656,   2672,   2688,   2704,   2720,   2736,   2752,   2768,
  2784,   2800,   2816,   2832,   2848,   2864,   2880,   2896,   2912,   2928,   2944,   2960,   2976,   2992,
  3008,   3024,   3040,   3056,   3072,   3088,   3104,   3120,   3136,   3152,   3168,   3184,   3200,   3216,
  3232,   3248,   3264,   3280,   3296,   3312,   3328,   3344,   3360,   3376,   3392,   3408,   3424,   3440,
  3456,   3472,   3488,   3504,   3520,   3536,   3552,   3568,   3584,   3600,   3616,   3632,   3648,   3664,
  3680,   3696,   3712,   3728,   3744,   3760,   3776,   3792,   3808,   3824,   3840,   3856,   3872,   3888,
  3904,   3920,   3936,   3952,   3968,   3984,   4000,   4016,   4032,   4048,   4064,   4080,   4096,   4128,
  4160,   4192,   4224,   4256,   4288,   4320,   4352,   4384,   4416,   4448,   4480,   4512,   4544,   4576,
  4608,   4640,   4672,   4704,   4736,   4768,   4800,   4832,   4864,   4896,   4928,   4960,   4992,   5024,
  5056,   5088,   5120,   5152,   5184,   5216,   5248,   5280,   5312,   5344,   5376,   5408,   5440,   5472,
  5504,   5536,   5568,   5600,   5632,   5664,   5696,   5728,   5760,   5792,   5824,   5856,   5888,   5920,
  5952,   5984,   6016,   6048,   6080,   6112,   6144,   6176,   6208,   6240,   6272,   6304,   6336,   6368,
  6400,   6432,   6464,   6496,   6528,   6560,   6592,   6624,   6656,   6688,   6720,   6752,   6784,   6816,
  6848,   6880,   6912,   6944,   6976,   7008,   7040,   7072,   7104,   7136,   7168,   7200,   7232,   7264,
  7296,   7328,   7360,   7392,   7424,   7456,   7488,   7520,   7552,   7584,   7616,   7648,   7680,   7712,
  7744,   7776,   7808,   7840,   7872,   7904,   7936,   7968,   8000,   8032,   8064,   8096,   8128,   8160,
  8192,   8256,   8320,   8384,   8448,   8512,   8576,   8640,   8704,   8768,   8832,   8896,   8960,   9024,
  9088,   9152,   9216,   9280,   9344,   9408,   9472,   9536,   9600,   9664,   9728,   9792,   9856,   9920,
  9984,   10048,  10112,  10176,  10240,  10304,  10368,  10432,  10496,  10560,  10624,  10688,  10752,  10816,
  10880,  10944,  11008,  11072,  11136,  11200,  11264,  11328,  11392,  11456,  11520,  11584,  11648,  11712,
  11776,  11840,  11904,  11968,  12032,  12096,  12160,  12224,  12288,  12352,  12416,  12480,  12544,  12608,
  12672,  12736,  12800,  12864,  12928,  12992,  13056,  13120,  13184,  13248,  13312,  13376,  13440,  13504,
  13568,  13632,  13696,  13760,  13824,  13888,  13952,  14016,  14080,  14144,  14208,  14272,  14336,  14400,
  14464,  14528,  14592,  14656,  14720,  14784,  14848,  14912,  14976,  15040,  15104,  15168,  15232,  15296,
  15360,  15424,  15488,  15552,  15616,  15680,  15744,  15808,  15872,  15936,  16000,  16064,  16128,  16192,
  16256,  16320,  32767,  -1,     -2,     -3,     -4,     -5,     -6,     -7,     -8,     -9,     -10,    -11,
  -12,    -13,    -14,    -15,    -16,    -17,    -18,    -19,    -20,    -21,    -22,    -23,    -24,    -25,
  -26,    -27,    -28,    -29,    -30,    -31,    -32,    -33,    -34,    -35,    -36,    -37,    -38,    -39,
  -40,    -41,    -42,    -43,    -44,    -45,    -46,    -47,    -48,    -49,    -50,    -51,    -52,    -53,
  -54,    -55,    -56,    -57,    -58,    -59,    -60,    -61,    -62,    -63,    -64,    -65,    -66,    -67,
  -68,    -69,    -70,    -71,    -72,    -73,    -74,    -75,    -76,    -77,    -78,    -79,    -80,    -81,
  -82,    -83,    -84,    -85,    -86,    -87,    -88,    -89,    -90,    -91,    -92,    -93,    -94,    -95,
  -96,    -97,    -98,    -99,    -100,   -101,   -102,   -103,   -104,   -105,   -106,   -107,   -108,   -109,
  -110,   -111,   -112,   -113,   -114,   -115,   -116,   -117,   -118,   -119,   -120,   -121,   -122,   -123,
  -124,   -125,   -126,   -127,   -128,   -129,   -130,   -131,   -132,   -133,   -134,   -135,   -136,   -137,
  -138,   -139,   -140,   -141,   -142,   -143,   -144,   -145,   -146,   -147,   -148,   -149,   -150,   -151,
  -152,   -153,   -154,   -155,   -156,   -157,   -158,   -159,   -160,   -161,   -162,   -163,   -164,   -165,
  -166,   -167,   -168,   -169,   -170,   -171,   -172,   -173,   -174,   -175,   -176,   -177,   -178,   -179,
  -180,   -181,   -182,   -183,   -184,   -185,   -186,   -187,   -188,   -189,   -190,   -191,   -192,   -193,
  -194,   -195,   -196,   -197,   -198,   -199,   -200,   -201,   -202,   -203,   -204,   -205,   -206,   -207,
  -208,   -209,   -210,   -211,   -212,   -213,   -214,   -215,   -216,   -217,   -218,   -219,   -220,   -221,
  -222,   -223,   -224,   -225,   -226,   -227,   -228,   -229,   -230,   -231,   -232,   -233,   -234,   -235,
  -236,   -237,   -238,   -239,   -240,   -241,   -242,   -243,   -244,   -245,   -246,   -247,   -248,   -249,
  -250,   -251,   -252,   -253,   -254,   -255,   -256,   -258,   -260,   -262,   -264,   -266,   -268,   -270,
  -272,   -274,   -276,   -278,   -280,   -282,   -284,   -286,   -288,   -290,   -292,   -294,   -296,   -298,
  -300,   -302,   -304,   -306,   -308,   -310,   -312,   -314,   -316,   -318,   -320,   -322,   -324,   -326,
  -328,   -330,   -332,   -334,   -336,   -338,   -340,   -342,   -344,   -346,   -348,   -350,   -352,   -354,
  -356,   -358,   -360,   -362,   -364,   -366,   -368,   -370,   -372,   -374,   -376,   -378,   -380,   -382,
  -384,   -386,   -388,   -390,   -392,   -394,   -396,   -398,   -400,   -402,   -404,   -406,   -408,   -410,
  -412,   -414,   -416,   -418,   -420,   -422,   -424,   -426,   -428,   -430,   -432,   -434,   -436,   -438,
  -440,   -442,   -444,   -446,   -448,   -450,   -452,   -454,   -456,   -458,   -460,   -462,   -464,   -466,
  -468,   -470,   -472,   -474,   -476,   -478,   -480,   -482,   -484,   -486,   -488,   -490,   -492,   -494,
  -496,   -498,   -500,   -502,   -504,   -506,   -508,   -510,   -512,   -516,   -520,   -524,   -528,   -532,
  -536,   -540,   -544,   -548,   -552,   -556,   -560,   -564,   -568,   -572,   -576,   -580,   -584,   -588,
  -592,   -596,   -600,   -604,   -608,   -612,   -616,   -620,   -624,   -628,   -632,   -636,   -640,   -644,
  -648,   -652,   -656,   -660,   -664,   -668,   -672,   -676,   -680,   -684,   -688,   -692,   -696,   -700,
  -704,   -708,   -712,   -716,   -720,   -724,   -728,   -732,   -736,   -740,   -744,   -748,   -752,   -756,
  -760,   -764,   -768,   -772,   -776,   -780,   -784,   -788,   -792,   -796,   -800,   -804,   -808,   -812,
  -816,   -820,   -824,   -828,   -832,   -836,   -840,   -844,   -848,   -852,   -856,   -860,   -864,   -868,
  -872,   -876,   -880,   -884,   -888,   -892,   -896,   -900,   -904,   -908,   -912,   -916,   -920,   -924,
  -928,   -932,   -936,   -940,   -944,   -948,   -952,   -956,   -960,   -964,   -968,   -972,   -976,   -980,
  -984,   -988,   -992,   -996,   -1000,  -1004,  -1008,  -1012,  -1016,  -1020,  -1024,  -1032,  -1040,  -1048,
  -1056,  -1064,  -1072,  -1080,  -1088,  -1096,  -1104,  -1112,  -1120,  -1128,  -1136,  -1144,  -1152,  -1160,
  -1168,  -1176,  -1184,  -1192,  -1200,  -1208,  -1216,  -1224,  -1232,  -1240,  -1248,  -1256,  -1264,  -1272,
  -1280,  -1288,  -1296,  -1304,  -1312,  -1320,  -1328,  -1336,  -1344,  -1352,  -1360,  -1368,  -1376,  -1384,
  -1392,  -1400,  -1408,  -1416,  -1424,  -1432,  -1440,  -1448,  -1456,  -1464,  -1472,  -1480,  -1488,  -1496,
  -1504,  -1512,  -1520,  -1528,  -1536,  -1544,  -1552,  -1560,  -1568,  -1576,  -1584,  -1592,  -1600,  -1608,
  -1616,  -1624,  -1632,  -1640,  -1648,  -1656,  -1664,  -1672,  -1680,  -1688,  -1696,  -1704,  -1712,  -1720,
  -1728,  -1736,  -1744,  -1752,  -1760,  -1768,  -1776,  -1784,  -1792,  -1800,  -1808,  -1816,  -1824,  -1832,
  -1840,  -1848,  -1856,  -1864,  -1872,  -1880,  -1888,  -1896,  -1904,  -1912,  -1920,  -1928,  -1936,  -1944,
  -1952,  -1960,  -1968,  -1976,  -1984,  -1992,  -2000,  -2008,  -2016,  -2024,  -2032,  -2040,  -2048,  -2064,
  -2080,  -2096,  -2112,  -2128,  -2144,  -2160,  -2176,  -2192,  -2208,  -2224,  -2240,  -2256,  -2272,  -2288,
  -2304,  -2320,  -2336,  -2352,  -2368,  -2384,  -2400,  -2416,  -2432,  -2448,  -2464,  -2480,  -2496,  -2512,
  -2528,  -2544,  -2560,  -2576,  -2592,  -2608,  -2624,  -2640,  -2656,  -2672,  -2688,  -2704,  -2720,  -2736,
  -2752,  -2768,  -2784,  -2800,  -2816,  -2832,  -2848,  -2864,  -2880,  -2896,  -2912,  -2928,  -2944,  -2960,
  -2976,  -2992,  -3008,  -3024,  -3040,  -3056,  -3072,  -3088,  -3104,  -3120,  -3136,  -3152,  -3168,  -3184,
  -3200,  -3216,  -3232,  -3248,  -3264,  -3280,  -3296,  -3312,  -3328,  -3344,  -3360,  -3376,  -3392,  -3408,
  -3424,  -3440,  -3456,  -3472,  -3488,  -3504,  -3520,  -3536,  -3552,  -3568,  -3584,  -3600,  -3616,  -3632,
  -3648,  -3664,  -3680,  -3696,  -3712,  -3728,  -3744,  -3760,  -3776,  -3792,  -3808,  -3824,  -3840,  -3856,
  -3872,  -3888,  -3904,  -3920,  -3936,  -3952,  -3968,  -3984,  -4000,  -4016,  -4032,  -4048,  -4064,  -4080,
  -4096,  -4128,  -4160,  -4192,  -4224,  -4256,  -4288,  -4320,  -4352,  -4384,  -4416,  -4448,  -4480,  -4512,
  -4544,  -4576,  -4608,  -4640,  -4672,  -4704,  -4736,  -4768,  -4800,  -4832,  -4864,  -4896,  -4928,  -4960,
  -4992,  -5024,  -5056,  -5088,  -5120,  -5152,  -5184,  -5216,  -5248,  -5280,  -5312,  -5344,  -5376,  -5408,
  -5440,  -5472,  -5504,  -5536,  -5568,  -5600,  -5632,  -5664,  -5696,  -5728,  -5760,  -5792,  -5824,  -5856,
  -5888,  -5920,  -5952,  -5984,  -6016,  -6048,  -6080,  -6112,  -6144,  -6176,  -6208,  -6240,  -6272,  -6304,
  -6336,  -6368,  -6400,  -6432,  -6464,  -6496,  -6528,  -6560,  -6592,  -6624,  -6656,  -6688,  -6720,  -6752,
  -6784,  -6816,  -6848,  -6880,  -6912,  -6944,  -6976,  -7008,  -7040,  -7072,  -7104,  -7136,  -7168,  -7200,
  -7232,  -7264,  -7296,  -7328,  -7360,  -7392,  -7424,  -7456,  -7488,  -7520,  -7552,  -7584,  -7616,  -7648,
  -7680,  -7712,  -7744,  -7776,  -7808,  -7840,  -7872,  -7904,  -7936,  -7968,  -8000,  -8032,  -8064,  -8096,
  -8128,  -8160,  -8192,  -8256,  -8320,  -8384,  -8448,  -8512,  -8576,  -8640,  -8704,  -8768,  -8832,  -8896,
  -8960,  -9024,  -9088,  -9152,  -9216,  -9280,  -9344,  -9408,  -9472,  -9536,  -9600,  -9664,  -9728,  -9792,
  -9856,  -9920,  -9984,  -10048, -10112, -10176, -10240, -10304, -10368, -10432, -10496, -10560, -10624, -10688,
  -10752, -10816, -10880, -10944, -11008, -11072, -11136, -11200, -11264, -11328, -11392, -11456, -11520, -11584,
  -11648, -11712, -11776, -11840, -11904, -11968, -12032, -12096, -12160, -12224, -12288, -12352, -12416, -12480,
  -12544, -12608, -12672, -12736, -12800, -12864, -12928, -12992, -13056, -13120, -13184, -13248, -13312, -13376,
  -13440, -13504, -13568, -13632, -13696, -13760, -13824, -13888, -13952, -14016, -14080, -14144, -14208, -14272,
  -14336, -14400, -14464, -14528, -14592, -14656, -14720, -14784, -14848, -14912, -14976, -15040, -15104, -15168,
  -15232, -15296, -15360, -15424, -15488, -15552, -15616, -15680, -15744, -15808, -15872, -15936, -16000, -16064,
  -16128, -16192, -16256, -16320
};

short Convert11bitFloat2LinearVal(short input)
{
  return ((input < 2048) ? kFloaT2LinLut[input] : 0);
}

#endif  // FLOAT_TO_LIN_H